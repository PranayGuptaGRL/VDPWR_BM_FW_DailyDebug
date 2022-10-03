/*
 * FRAM.c
 *
 *  Created on: 19-Aug-2021
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



#include"PDPPSManuIncludes.h"
#include "FRAM.h"

uint16_t STOP_CON_FLAG = 0;
uint16_t REG_ACCCESS_READY = 0;

struct I2CMsg i2cMsgOut = { MSG_STATUS_SEND_WITHSTOP,
                            SLAVE_ADDRESS,
                            NUM_BYTES,
                            EEPROM_HIGH_ADDR,
                            EEPROM_LOW_ADDR };

struct I2CMsg i2cMsgIn = { MSG_STATUS_SEND_NOSTOP,
                           SLAVE_ADDRESS,
                           NUM_BYTES,
                           EEPROM_HIGH_ADDR,
                           EEPROM_LOW_ADDR };

struct I2CMsg *currentMsgPtr;                   // Used in interrupt

uint16_t FRAM_TX[10] = { 0x00 };
uint16_t FRAM_RX[10] = { 0x00 };


/**
 * @brief this function used to read data from specific memory location from FRAM
 * @param byte_count number of bytes to read from FRAM.
 * @param Address starting address(memory location) to read data from FRAM.
 * @param buffer this is data buffer where received data is stored.
 * @return successful write returns 0x0000(SUCCESS)
 *            failure go to fail function.
 *
 * @note
 */
uint16_t FRAM_I2C_read(uint16_t byte_count, uint16_t Address, uint16_t *buffer)
{
    uint16_t i = 0, j = 0, status;
    i2cMsgIn.slaveAddr = SLAVE_ADDRESS;
    i2cMsgIn.memoryHighAddr = (uint8_t) Address >> 8;
    i2cMsgIn.memoryLowAddr = (uint8_t) Address & 0xFF;

    while (byte_count != 0)
    {
        if (byte_count < 8)
        {
            i2cMsgIn.numBytes = byte_count;
            status = readData(&i2cMsgIn);
            if (status != SUCCESS)
            {
                fail();
            }
            else
            {
                for (j = 0; j < byte_count; j++)
                {
                    buffer[i++] = i2cMsgIn.msgBuffer[j];
                }

                Address = Address + 8;
                i2cMsgIn.memoryHighAddr = (uint8_t) Address >> 8;
                i2cMsgIn.memoryLowAddr = (uint8_t) Address & 0xFF;
                byte_count -= byte_count;

            }

        }
        else
        {
            i2cMsgIn.numBytes = 8;
            status = readData(&i2cMsgIn);
            if (status != SUCCESS)
            {
                fail();
            }
            else
            {
                for (j = 0; j < 8; j++)
                {
                    buffer[i++] = i2cMsgIn.msgBuffer[j];
                }

                Address = Address + 8;
                i2cMsgIn.memoryHighAddr = (uint8_t) Address >> 8;
                i2cMsgIn.memoryLowAddr = (uint8_t) Address & 0xFF;
                byte_count -= 8;
            }

        }
    }

    return SUCCESS;
}

/**
 * this function is used to write data into FRAM with specific memory location.
 * @param byte_count number of bytes to write.
 * @param Address address(memory address) at which going to write data.
 * @param buffer user provided data to write into FRAM.
 * @return successful write returns 0x0000(SUCCESS)
 *            failure go to fail function.
 */
uint16_t FRAM_I2C_write(uint16_t byte_count, uint16_t Address, uint16_t *buffer)
{
    uint16_t i = 0, j = 0, status;
    i2cMsgOut.slaveAddr = SLAVE_ADDRESS;
    i2cMsgOut.memoryHighAddr = (uint8_t) Address >> 8;
    i2cMsgOut.memoryLowAddr = (uint8_t) Address & 0xFF;
    while (byte_count != 0)
    {
        if (byte_count < 8)
        {
            i2cMsgOut.numBytes = byte_count;
            for (j = 0; j < byte_count; j++)
            {
                i2cMsgOut.msgBuffer[j] = buffer[i++];
            }
            status = writeData(&i2cMsgOut);
            if (status != SUCCESS)
            {
                fail();
            }
            else
            {
                Address = Address + byte_count;
                i2cMsgOut.memoryHighAddr = (uint8_t) Address >> 8;
                i2cMsgOut.memoryLowAddr = (uint8_t) Address & 0xFF;
                byte_count -= byte_count;
            }
        }
        else
        {
            i2cMsgOut.numBytes = 8;
            for (j = 0; j < 8; j++)
            {
                i2cMsgOut.msgBuffer[j] = buffer[i++];
            }
            status = writeData(&i2cMsgOut);
            if (status != SUCCESS)
            {
                fail();
            }
            else
            {
                Address = Address + 8;
                i2cMsgOut.memoryHighAddr = (uint8_t) Address >> 8;
                i2cMsgOut.memoryLowAddr = (uint8_t) Address & 0xFF;
                byte_count -= 8;
            }

        }
    }
    return SUCCESS;
}

//
// Init_I2C_Master - Function to configure I2C A in FIFO mode.
//
void Init_I2C_Master()
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CB_BASE);

    //
    // I2C configuration. Use a 100k I2CCLK with a 33% duty cycle.
    //
    I2C_initMaster(I2CB_BASE, DEVICE_SYSCLK_FREQ, 100000, I2C_DUTYCYCLE_33);
    I2C_setBitCount(I2CB_BASE, I2C_BITCOUNT_8);
//    I2C_setSlaveAddress(I2CB_BASE, SLAVE_ADDRESS);
    I2C_setEmulationMode(I2CB_BASE, I2C_EMULATION_FREE_RUN);

    //
    // Enable stop condition and register-access-ready interrupts
    //
    I2C_enableInterrupt(I2CB_BASE,
    I2C_INT_STOP_CONDITION | I2C_INT_REG_ACCESS_RDY);

    //
    // FIFO configuration
    //
    I2C_enableFIFO(I2CB_BASE);
    I2C_clearInterruptStatus(I2CB_BASE, I2C_INT_RXFF | I2C_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    I2C_enableModule(I2CB_BASE);
}

//
// writeData - Function to send the data that is to be written to the EEPROM
//
uint16_t writeData(struct I2CMsg *msg)
{
    uint16_t i;

    //
    // Wait until the STP bit is cleared from any previous master
    // communication. Clearing of this bit by the module is delayed until after
    // the SCD bit is set. If this bit is not checked prior to initiating a new
    // message, the I2C could get confused.
    //
    DEVICE_DELAY_US(1000);
    if (I2C_getStopConditionStatus(I2CB_BASE))
    {
        return (ERROR_STOP_NOT_READY);
    }

    //
    // Setup slave address
    //
    I2C_setSlaveAddress(I2CB_BASE, SLAVE_ADDRESS);
    //DEVICE_DELAY_US(1000);
    //
    // Check if bus busy
    //
    if (I2C_isBusBusy(I2CB_BASE))
    {
        return (ERROR_BUS_BUSY);
    }

    //
    // Setup number of bytes to send msgBuffer and address
    //
    I2C_setDataCount(I2CB_BASE, (msg->numBytes + 2));

    //
    // Setup data to send
    //
    I2C_putData(I2CB_BASE, msg->memoryHighAddr);
    I2C_putData(I2CB_BASE, msg->memoryLowAddr);

    for (i = 0; i < msg->numBytes; i++)
    {
        I2C_putData(I2CB_BASE, msg->msgBuffer[i]);
    }

    //
    // Send start as master transmitter
    //
    I2C_setConfig(I2CB_BASE, I2C_MASTER_SEND_MODE);
    I2C_sendStartCondition(I2CB_BASE);
    DEVICE_DELAY_US(1000);
    I2C_sendStopCondition(I2CB_BASE);
//    while( I2C_getInterruptSource(I2CB_BASE) != I2C_INTSRC_ STOP_CON_FLAGDITION );
    if (STOP_CON_FLAG == 1)
        STOP_CON_FLAG = 0;

    return (SUCCESS);
}

//
// readData - Function to prepare for the data that is to be read from the EEPROM
//
uint16_t readData(struct I2CMsg *msg)
{
    uint16_t i;
    //
    // Wait until the STP bit is cleared from any previous master
    // communication. Clearing of this bit by the module is delayed until after
    // the SCD bit is set. If this bit is not checked prior to initiating a new
    // message, the I2C could get confused.
    //
    DEVICE_DELAY_US(1000);
    if (I2C_getStopConditionStatus(I2CB_BASE))
    {
        return (ERROR_STOP_NOT_READY);
    }

    //
    // Setup slave address
    //
    I2C_setSlaveAddress(I2CB_BASE, SLAVE_ADDRESS);

    //
    // If we are in the the address setup phase, send the address without a
    // stop condition.
    //
    if (msg->msgStatus == MSG_STATUS_SEND_NOSTOP)
    {
        //
        // Check if bus busy
        //
        // DEVICE_DELAY_US(1000);
        if (I2C_isBusBusy(I2CB_BASE))
        {
            return (ERROR_BUS_BUSY);
        }

        //
        // Send data to setup EEPROM address
        //
        I2C_setDataCount(I2CB_BASE, 2);
        I2C_putData(I2CB_BASE, msg->memoryHighAddr);
        I2C_putData(I2CB_BASE, msg->memoryLowAddr);
        I2C_setConfig(I2CB_BASE, I2C_MASTER_SEND_MODE);
        I2C_sendStartCondition(I2CB_BASE);
        DEVICE_DELAY_US(1000);

        if (REG_ACCCESS_READY == 1)
            REG_ACCCESS_READY = 0;

        // If a NACK is received, clear the NACK bit and command a stop.
        // Otherwise, move on to the read data portion of the communication.
        //
        /*if((I2C_getStatus(I2CB_BASE) & I2C_STS_NO_ACK) != 0)
         {
         I2C_sendStopCondition(I2CB_BASE);
         I2C_clearStatus(I2CB_BASE, I2C_STS_NO_ACK);
         }*/

        I2C_setDataCount(I2CB_BASE, (msg->numBytes));
        // I2C_setSlaveAddress(I2CB_BASE, SLAVE_ADDRESS|1);
        I2C_setConfig(I2CB_BASE, I2C_MASTER_RECEIVE_MODE);
        I2C_sendStartCondition(I2CB_BASE);
        I2C_sendStopCondition(I2CB_BASE);
        DEVICE_DELAY_US(1000);
        if (STOP_CON_FLAG == 1)
            STOP_CON_FLAG = 0;
        /*   while( I2C_getInterruptSource(I2CB_BASE) != I2C_INTSRC_ STOP_CON_FLAGDITION );*/
        for (i = 0; i < NUM_BYTES; i++)
        {
            i2cMsgIn.msgBuffer[i] = I2C_getData(I2CB_BASE);
        }

    }

    return (SUCCESS);
}

//
// I2C_FRAM_ISR - I2C A ISR (non-FIFO)
//
__interrupt void I2C_FRAM_ISR(void)
{
    I2C_InterruptSource intSource;

    //
    // Read interrupt source
    //
    intSource = I2C_getInterruptSource(I2CB_BASE);

    //
    // Interrupt source = stop condition detected
    //
    if (intSource == I2C_INT_STOP_CONDITION)
    {
        STOP_CON_FLAG = 1;
    }
    //
    // Interrupt source = Register Access Ready
    //
    // This interrupt is used to determine when the EEPROM address setup
    // portion of the read data communication is complete.

    else if (intSource == I2C_INTSRC_REG_ACCESS_RDY)
    {
        REG_ACCCESS_READY = 1;
    }
    else
    {
        //
        // Generate some error from invalid interrupt source
        //
        // asm("   ESTOP0");
    }

    //
    // Issue ACK to enable future group 8 interrupts
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}
//
// pass - Function to be called if data written matches data read
//
void pass(void)
{
    asm("   ESTOP0");
    for (;;)
        ;
}

//
// fail - Function to be called if data written does NOT match data read
//
void fail(void)
{
    asm("   ESTOP0");
    // for(;;);
}

//
// End of File
//
