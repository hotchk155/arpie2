
//
// Each command must be sent as a separate I2C transaction
// The first byte is the target address for the following
// data bytes
// 

// define commands to send to the display
#define CMD_CLS 0x80
#define CMD_TEST 0x81