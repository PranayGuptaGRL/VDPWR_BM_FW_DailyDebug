/*
 * FRAM.h
 *
 *  Created on: 16-Sep-2021
 *      Author: Vishal
 */

// +-+-+-+-+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+
// |A|u|t|h|o|r|:| |V|i|s|h|a|l| |K|a|k|a|d|e|
// +-+-+-+-+-+-+-+ +-+-+-+-+-+-+ +-+-+-+-+-+-+

// ____    ____    ____        ____    ____    ____            ___         __
///\  _`\ /\  _`\ /\  _`\     /\  _`\ /\  _`\ /\  _`\        /'___`\     /'__`\
//\ \ \L\_\ \ \L\ \ \ \L\ \   \ \ \L\ \ \ \L\ \ \,\L\_\     /\_\ /\ \   /\ \/\ \
// \ \  _\L\ \ ,__/\ \ ,  /    \ \ ,__/\ \ ,__/\/_\__ \     \/_/// /__  \ \ \ \ \
//  \ \ \L\ \ \ \/  \ \ \\ \    \ \ \/  \ \ \/   /\ \L\ \      // /_\ \__\ \ \_\ \
//   \ \____/\ \_\   \ \_\ \_\   \ \_\   \ \_\   \ `\____\    /\______/\_\\ \____/
//    \/___/  \/_/    \/_/\/ /    \/_/    \/_/    \/_____/    \/_____/\/_/ \/___/
//


#ifndef FRAM_H_
#define FRAM_H_


//*********************************Calibration / FRAM*************************************

//
// Defines
//
#define SLAVE_ADDRESS               0x57
#define EEPROM_HIGH_ADDR            0x00
#define EEPROM_LOW_ADDR             0x00
#define NUM_BYTES                   8
#define MAX_BUFFER_SIZE             14      // Max is currently 14 because of
// 2 address bytes and the 16-byte
// FIFO

//
// I2C message states for I2CMsg struct
//
#define MSG_STATUS_INACTIVE         0x0000 // Message not in use, do not send
#define MSG_STATUS_SEND_WITHSTOP    0x0010 // Send message with stop bit
#define MSG_STATUS_WRITE_BUSY       0x0011 // Message sent, wait for stop
#define MSG_STATUS_SEND_NOSTOP      0x0020 // Send message without stop bit
#define MSG_STATUS_SEND_NOSTOP_BUSY 0x0021 // Message sent, wait for ARDY
#define MSG_STATUS_RESTART          0x0022 // Ready to become master-receiver
#define MSG_STATUS_READ_BUSY        0x0023 // Wait for stop before reading data

//
// Error messages for read and write functions
//
#define ERROR_BUS_BUSY              0x1000
#define ERROR_STOP_NOT_READY        0x5555
#define SUCCESS                     0x0000

extern uint16_t FRAM_TX[10];
extern uint16_t FRAM_RX[10];

//
// Typedefs
//
struct I2CMsg
{
    uint16_t msgStatus;                  // Word stating what state msg is in.
                                         // See MSG_STATUS_* defines above.
    uint16_t slaveAddr;                  // Slave address tied to the message.
    uint16_t numBytes;                   // Number of valid bytes in message.
    uint16_t memoryHighAddr;             // EEPROM address of data associated
                                         // with message (high byte).
    uint16_t memoryLowAddr;              // EEPROM address of data associated
                                         // with message (low byte).
    uint16_t msgBuffer[MAX_BUFFER_SIZE]; // Array holding message data.
};

//
// Function Prototypes
//
void Init_I2C_Master(void);
uint16_t readData(struct I2CMsg *msg);
uint16_t writeData(struct I2CMsg *msg);
uint16_t FRAM_I2C_read(uint16_t byte_count, uint16_t Address, uint16_t *buffer);
uint16_t FRAM_I2C_write(uint16_t byte_count, uint16_t Address,
                        uint16_t *buffer);

void fail(void);
void pass(void);

__interrupt void I2C_FRAM_ISR(void);




#endif /* FRAM_H_ */
