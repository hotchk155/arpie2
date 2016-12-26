//
// HEADER FILES
//
#include <system.h>
#include <memory.h>

// 
// PIC12F1822 CONFIG BYTES
//

// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// CONSTANTS
//

// Pin definitions
#define P_AN_SCK	lata.5
#define P_AN_RCK	lata.4
#define P_AN_DAT1	latc.5
#define P_AN_DAT2	latc.4
#define P_AN_DAT3	latc.3
#define P_CA_CLK	latc.2
#define P_DISP_EN	lata.2
#define P_KS1		porta.0
#define P_KS2		porta.1
#define P_KS3		porta.3

                   //76543210
#define TRISA_INIT 0b11001011
#define TRISC_INIT 0b11000011
#define WPUA_INIT  0b00001011

typedef unsigned char byte;

// definitions for the transmit buffer, used to return
// keypresses to the master
#define TX_BUFFER_MASK	0x0F
byte tx_buffer[16];
byte tx_head = 0;
byte tx_tail = 0;

// receive buffer for display data from the master
#define RX_BUFFER_MASK	0x3F
byte rx_buffer[64];
byte rx_head = 0;
byte rx_tail = 0;


/////////////////////////////////////////////////////////////////////////
//
//	Each display stored as 2 layers:
//	bytes 0-7  = standard raster bits
//	bytes 8-15 = brightness attribute bits
//	
//	The data is stored in a format that can be used directly during
//	screen refreshes, so incoming bitmaps require mapping before they
//	are loaded into these buffers
//
/////////////////////////////////////////////////////////////////////////
volatile byte disp_data0[16];	// data buffer for the first display 
volatile byte disp_data1[16]; 	// data buffer for the second display 
volatile byte disp_data2[16];	// data buffer for the third display 

// definitions used for debouncing the switches
#define DEBOUNCE_PERIOD 20 // units based on display update rate
volatile byte debouncer = 0;

// define a lookup table of the bits in the keyscan result which
// correspond to each of the switches. The array is indexed by the
// keycode (less one) that will be returned to master for a switch
#define A_BIT(b) ((long)1<<(b))
#define B_BIT(b) (((long)1<<(b))<<8)
#define C_BIT(b) (((long)1<<(b))<<16)
#define NUM_KEYS 23
long key_bit[NUM_KEYS] = {
	B_BIT(6), // A1
	C_BIT(1), // A2
	A_BIT(2), // A3
	B_BIT(0), // A4
	B_BIT(3), // A5
	C_BIT(7), // B1
	C_BIT(4), // B2
	C_BIT(0), // B3
	A_BIT(1), // B4
	A_BIT(4), // B5
	A_BIT(7), // B6
	B_BIT(2), // B7
	A_BIT(5), // B8
	C_BIT(6), // C1
	C_BIT(3), // C2
	B_BIT(5), // C3
	A_BIT(0), // C4
	A_BIT(6), // C5
	C_BIT(5), // D1
	C_BIT(2), // D2
	B_BIT(7), // D3
	B_BIT(4), // D4
	A_BIT(3)  // D5
};

// Define the duty periods for each phase. Each value
// is (256 - t) where t is the desired duty in timer0
// clock ticks
#define PHASE1_TMR0_INIT 250
#define PHASE2_TMR0_INIT 230
#define PHASE3_TMR0_INIT 0

// Define the states for the display update machine
enum 
{
	DISP_BEGIN,
	DISP_PHASE1,
	DISP_PHASE2,
	DISP_PHASE3
};

// The main state for the display refresh state machine
volatile byte disp_state = DISP_BEGIN;

// stores the final result of key scanning when display refresh complete
volatile long keyscan_result;

// working variables used during display update
volatile byte disp_count;
volatile byte *disp_raster0;
volatile byte *disp_raster1;
volatile byte *disp_raster2;
volatile byte *disp_bright0;
volatile byte *disp_bright1;
volatile byte *disp_bright2;
volatile long keyscan_accumulator;

////////////////////////////////////////////////////////////
//
// INTERRUPT HANDLER 
//
////////////////////////////////////////////////////////////
void interrupt( void )
{

	////////////////////////////////////////////////////////////
	// Timer 0 overflow - used to time the display multiplexing
	if(intcon.2)
	{
		byte d0, d1, d2;
	
		// clear interrupt flag
		intcon.2 = 0;
		
		// turn off the display
		P_DISP_EN = 1;					

		// run display update state machine
		switch(disp_state) 
		{
			// initial state, 
			case DISP_BEGIN: 
			
				// start by clocking a single "1" bit into
				// the column cathode shift register
				P_AN_DAT1 = 1;
				P_CA_CLK = 0;
				P_CA_CLK = 1;

				// set data pointers to the first location
				// for each display
				disp_raster0 = disp_data0;
				disp_raster1 = disp_data1;
				disp_raster2 = disp_data2;
				disp_bright0 = disp_data0+8;
				disp_bright1 = disp_data1+8;
				disp_bright2 = disp_data2+8;
				disp_count = 0;
				keyscan_accumulator = 0;
				disp_state = DISP_PHASE1;

				// we use the display clocking to provide a
				// time base for switch debouncing
				if(debouncer) 
					--debouncer;

				// fall through to next state

			// column display phase 1 
			case DISP_PHASE1:
		
				// activate the next column
				P_DISP_EN = 1;
				P_AN_DAT1 = 0;
				P_CA_CLK = 0;
				P_CA_CLK = 1;

				// fall through to next state
				
			case DISP_PHASE2:
			case DISP_PHASE3:
			
				// There are three timing phases used to get the three
				// duty cycles for DIM, NORMAL, BRIGHT levels
				//
				// <------------------BRIGHT----------------->  (defined as bits on in both raster and brightness layers)
				// <----------NORMAL----------->                (defined as bit on in raster layer only)
				// <-----DIM----->                              (defined as bit on in brightness layer only)
				// |   PHASE 1   |   PHASE 2   |   PHASE 3   |
				//
				switch(disp_state) 
				{
				case DISP_PHASE1: 
					d0 = *disp_raster0|*disp_bright0;
					d1 = *disp_raster1|*disp_bright1;
					d2 = *disp_raster2|*disp_bright2;
					break;
				case DISP_PHASE2:
					d0 = *disp_raster0;
					d1 = *disp_raster1;
					d2 = *disp_raster2;
					break;
				case DISP_PHASE3:
					d0 = *disp_raster0&*disp_bright0;
					d1 = *disp_raster1&*disp_bright1;
					d2 = *disp_raster2&*disp_bright2;
					break;
				}
				
				// load the anode shift registers
				byte mask = 0x01;
				while(mask)
				{
					P_AN_DAT1 = !!(d0 & mask);
					P_AN_DAT2 = !!(d1 & mask);
					P_AN_DAT3 = !!(d2 & mask);
					
					P_AN_SCK = 0;
					P_AN_SCK = 1;				
					mask <<= 1;
				}
				
				// activate the display
				P_AN_RCK = 0;			
				P_AN_RCK = 1;		
				P_DISP_EN = 0;					
				
				// prepare for the next state
				switch(disp_state) 
				{
				case DISP_PHASE1:
					disp_state = DISP_PHASE2;					
					tmr0 = PHASE1_TMR0_INIT;
					break;
				case DISP_PHASE2:
					disp_state = DISP_PHASE3;
					tmr0 = PHASE2_TMR0_INIT;
					break;				
				case DISP_PHASE3:					
					// Switch states are read in phase 3. We build up 
					// a 23 bit word of switch status
					keyscan_accumulator <<= 1;
					if(!P_KS1) keyscan_accumulator |= 1;
					keyscan_accumulator <<= 1;
					if(!P_KS2) keyscan_accumulator |= 1;
					keyscan_accumulator <<= 1;
					if(!P_KS3) keyscan_accumulator |= 1;
					
					// have we completed all 8 steps of the scan?
					if(++disp_count >= 8) 
					{
						// ok store the key scan result and start again
						keyscan_result = keyscan_accumulator;
						disp_state = DISP_BEGIN;						
					}
					else
					{
						// otherwise move to next bytes and start again
						++disp_raster0;
						++disp_raster1;
						++disp_raster2;
						++disp_bright0;
						++disp_bright1;
						++disp_bright2;
						disp_state = DISP_PHASE1;						
					}
					tmr0 = PHASE3_TMR0_INIT;
					break;
				}
		}
		
	}
	
	//////////////////////////////////////////////////////////
	// I2C SLAVE INTERRUPT
	// Called when there is activity on the I2C bus
	if(pir1.3) 
	{       
		byte d;
		pir1.3 = 0; // clear interrupt flag
		if(!ssp1stat.D_NOT_A) // master has sent our slave address
		{                
			d = ssp1buf; // read and discard address to clear BF flag
			
			// Is the master setting up a data READ?
			if(ssp1stat.R_NOT_W) 
			{          
				if(tx_head != tx_tail)
				{
					ssp1buf = tx_buffer[tx_tail];
					tx_tail = (tx_tail + 1)&TX_BUFFER_MASK;
				}
				else
				{
					ssp1buf = 0;                               
				}				
			}
			else 
			{
				ssp1buf = 0;
			}
		}
		else // DATA 
		{ 
			if(!ssp1stat.R_NOT_W) // MASTER IS WRITING TO SLAVE
			{
				d = ssp1buf;
				byte next_head = (rx_head + 1)&RX_BUFFER_MASK;
				if(next_head != rx_tail) 
				{
					rx_buffer[rx_head] = d;
					rx_head = next_head;
				}
			}
			else // MASTER IS READING FROM SLAVE
			{                                               
				ssp1con1.WCOL = 0; // clear write collision bit
				if(!ssp1con2.ACKSTAT) // was acknowledge received?
				{
					if(tx_head != tx_tail)
					{
						ssp1buf = tx_buffer[tx_tail];
						tx_tail = (tx_tail + 1)&TX_BUFFER_MASK;
					}
					else
					{
						ssp1buf = 0;                               
					}
				}
				else 
				{
					ssp1buf = 0;                               
				}
			}
		}
		ssp1con1.CKP = 1; // release clock
	}
}

////////////////////////////////////////////////////////////
//
// I2C SLAVE INITIALISATION
//
////////////////////////////////////////////////////////////
void i2c_init(byte addr) 
{
        ssp1stat.7 = 0; // slew rate control disabled on high speed i2c mode
        
        ssp1con1.3 = 0; // }
        ssp1con1.2 = 1; // } I2C slave mode
        ssp1con1.1 = 1; // } with 7 bit address
        ssp1con1.0 = 0; // }
        
        ssp1msk = 0b01111111;   // address mask bits 0-6
        ssp1add = addr<<1;      // set slave address
        pie1.3 = 1;     // SSP1IE
        pir1.3 = 0; // SSP1IF
        
        ssp1con1.5 = 1; // enable i2c
}

////////////////////////////////////////////////////////////
//
// CLEAR DISPLAY
//
////////////////////////////////////////////////////////////
void cls() 
{
	memset(disp_data0,0, sizeof(disp_data0));
	memset(disp_data1,0, sizeof(disp_data1));
	memset(disp_data2,0, sizeof(disp_data2));
}

////////////////////////////////////////////////////////////
// Store a byte of display data, mapping the bits ready
// for the display routines. The address is 
//
// 0-7 for raster rows of the first display
// 8-15 for raster rows of the second display
// 16-23 for raster rows of the first display
// 24-31 for brightness attributes for the first display
// 32-39 for brightness attributes for the second display
// 40-47 for brightness attributes for the third display
//
////////////////////////////////////////////////////////////
void store_data(byte addr, byte data)
{
	// order of column
	static const byte row_xlat[8] = { 0, 1, 2, 3, 7, 6, 5, 4 };

	// order of rows
	static const byte col_mask[8] = { 1<<7, 1<<6, 1<<5, 1<<4, 1<<3, 1<<2, 1<<0, 1<<1 };

	byte *write_base;
	switch(addr>>3) // divide by 8
	{
		case 0: write_base = disp_data0; break;
		case 1: write_base = disp_data1; break;
		case 2: write_base = disp_data2; break;
		case 3: write_base = disp_data0 + 8; break;
		case 4: write_base = disp_data1 + 8; break;
		case 5: write_base = disp_data2 + 8; break;
		default: return;
	}

	// for this address, we'll always be using the same bit
	// in the anode shift register data for each column
	byte write_mask = col_mask[addr & 0x7]; // mod 8
	byte read_mask = 0x80;
	for(byte i=0; i<8; ++i)
	{
		byte *ptr = write_base + row_xlat[i];
		if(data & read_mask)
			*ptr |= write_mask;
		else
			*ptr &= ~write_mask;
		read_mask >>= 1;
	}	
}

void test() 
{

	store_data(0, 0b00000000);
	store_data(1, 0b00110001);
	store_data(2, 0b00001010);
	store_data(3, 0b00111010);
	store_data(4, 0b01001010);
	store_data(5, 0b01001010);
	store_data(6, 0b00111010);
	store_data(7, 0b00000000);

	store_data(8, 0b00000000);
	store_data(9, 0b11011100);
	store_data(10, 0b00010010);
	store_data(11, 0b00010010);
	store_data(12, 0b00010010);
	store_data(13, 0b00010010);
	store_data(14, 0b00011100);
	store_data(15, 0b00010000);


	store_data(16, 0b00000000);
	store_data(17, 0b01001100);
	store_data(18, 0b00010010);
	store_data(19, 0b11011110);
	store_data(20, 0b01010000);
	store_data(21, 0b01010000);
	store_data(22, 0b01001110);
	store_data(23, 0b00000000);

}
void testw() 
{
	store_data(0, 0b00000000);
	store_data(1, 0b00011000);
	store_data(2, 0b00100100);
	store_data(3, 0b01000010);
	store_data(4, 0b01111110);
	store_data(5, 0b01000010);
	store_data(6, 0b01000010);
	store_data(7, 0b00000000);
}

////////////////////////////////////////////////////////////
//
// MAIN
//
////////////////////////////////////////////////////////////
void main()
{ 	
	// osc control / 16MHz / internal
	osccon = 0b01111010;

	ansela = 0;
	anselc = 0;
	trisa = TRISA_INIT;
	trisc = TRISC_INIT;


	wpua = WPUA_INIT;
	option_reg.7 = 0;

	i2c_init(123) ;

	// global enable interrupts
    intcon.7 = 1;
    intcon.6 = 1;
	
	// configure timer 0
	option_reg.5 = 0; // internal clock
	option_reg.3 = 0; // use prescaler
	option_reg.2 = 0; // }
	option_reg.1 = 1; // }
	option_reg.0 = 1; // }
	intcon.5 = 1; // enable timer 0 interrupt
	intcon.2 = 0; // clear timer 0 interrupt
	
	
	
	disp_state = DISP_BEGIN;
	cls();
	test();
	long last_keyscan = 0;

	// The main loop
	while(1) {
	
		/////////////////////////////////////////////////////////////////////
		// KEYPRESS PROCESSING
		if(!debouncer) // ignore all events while debouncing
		{
			// Check whether state of any switch has changed
			long this_keyscan = keyscan_result;
			if(this_keyscan != last_keyscan)
			{
				// get set of changed bits
				long delta = this_keyscan ^ last_keyscan;
				
				// store the new state
				last_keyscan = this_keyscan;
				
				// iterate through the lookup table of key bits
				for(byte k=0; k<NUM_KEYS; ++k)
				{
					// has switch key changed state?
					if(delta & key_bit[k])
					{
						// add one to index to get key code
						byte code = k+1;
						if(this_keyscan & key_bit[k]) 
						{
							// the switch has changed from not pressed to pressed,
							// so we need to ignore changes for a bit to account for
							// switch bounce
							debouncer = DEBOUNCE_PERIOD;
						}
						else
						{
							// The switch has changed from pressed to not pressed. No
							// need to debounce switch release, but we will flag this 
							// as a release event by setting bit 7 of the code
							code |= 0x80;
						}
						
						// place the keypress in the buffer to be sent to master
						byte next_head = (tx_head + 1)&TX_BUFFER_MASK;
						if(next_head != tx_tail)
						{
							tx_buffer[tx_head] = code;
							tx_head = next_head;
						}
					}
				}
			}
		}
	}
}

//
// END
//