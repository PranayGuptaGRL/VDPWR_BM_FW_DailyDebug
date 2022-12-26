/*
 * Peripheral.c
 *
 *  Created on: 09-Mar-2022
 *      Author: prana
 */
#include "Includes.h"
#include "PD_ControllerStruct.h"

#define LOW 0
#define HIGH 1
/**
 * @brief This Function is used to calculate No. of mS after which RS485 has to be resetted after receiving First Byte of any read data.
 * Time is calculated based on the total payload length(2nd Byte in received data) and for each 48Bytes 30mS should be a minimal delay
 * @param aPayLoadLength : total payload length received in read command . 2nd byte in read command indicates this value
 * @return return the timer value in mS
 * @author Pranay
 * @date 03Dec'20
 */
uint16_t gFetchTimerVal(uint16_t aPayLoadLength)
{

    //pranay,23May'22
    return ( ( (aPayLoadLength/48) + 1) * 30);

//    gControlStruct_t = (gControlStruct *)GetStructPtr();
    /**If payload length is >48 BYtes than increment count by 1 and reduce payload lenght by 48 and again call same function*/
    if(aPayLoadLength > 48)
    {
        gRS485ReInitDevCount  += 1;
        return (gFetchTimerVal(aPayLoadLength - 48));/**Calling the same function again with reduced payload length**/
    }
    /***if payload lenght is less than 48Bytes which will be in most of the cases return actual timer delay by multiplying with 30
     * so that timer will start based in our requirement - which is to reset rs485 after every read, so it is expected that for every 48Bytes of data it takes 30mS to read.
     *
     * */
    else if(aPayLoadLength <= 48)
    {
        return (gRS485ReInitDevCount * 30);
    }

}

/**
 * @brief The Interrupts from the rS485 module, MAX3140 will be generated for each individual bytes recveid from the host
 * Once the no of bytes become 255 bytes, then whole data is being received from the Host, the data need to be handled
 * This data handling will be done in Data Handler Thread, DataRxEvent is being set and infored the task to handle the rest
 * @author Prasanna
 * @date 02Sept'19
 */
bool RS485DataRecvIntrHandle(uint8_t *lReadBuf)
{
    bool lRetStatus = false;

//    gFunctStruct_t = (gFunctStruct *)GetStructPtr();

    if(gRS485RxIntrCount <= (gRS485ByteCount - 1))
    {
        if(0x80 == (lReadBuf[0] & 0x80))
        {
            if(gRS485RxIntrCount == 0)
            {
//                memset(gRS485RxBuf, 0, BUF_SIZE_256BYTE);
                /**Pranay,23March'21, To avoid Rs485 hung, Starting a 30mS timer and in expiry resetting rs485, if 1st byte is received will stop this timer and start timer based on API byte count*/
//                gFunctStruct_t->gFwHandle_t.FwTimerInfoStruct.gTimer2Info = Timer_RS485DevInit;
                MsgTimerStart(30, Timer1_RS485DevInit, TIMER1);
            }
            gRS485RxBuf[gRS485RxIntrCount] = lReadBuf[1];

            if(gRS485RxIntrCount == 1)
                gRS485ByteCount = (gRS485RxBuf[1] + 2);

            /**Pranay,03Dec'20, Controlcard will come to know the payload length to be read in 2nd byte[1:0] in every read operation, so after receiving 2nd byte based on total payload length
             * starting a timer with respective timer value and than resetting the rs485, to ensure communication doesnt hung after every read.**/
            if(gRS485RxIntrCount == 1)
            {
//                gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485ReInitDevCount = 1;
//                gFunctStruct_t->gFwHandle_t.FwTimerInfoStruct.gTimer2Info = Timer_RS485DevInit;
                gRS485ReInitDevCount = 1;
//                MsgTimerStart( gFetchTimerVal(gRS485RxBuf[1]) , Timer1_RS485DevInit, TIMER1);
                MsgTimerStart( ( ( (gRS485RxBuf[1] / 48) + 1) * 30), Timer1_RS485DevInit, TIMER1);

            }
            gRS485RxIntrCount++;
        }
        else
        {
//          DEBUG_LOG(DBG1, 0xE1, 0);
//            if(gErrRetryCount++ < 2);
//                SPIDataReadErrorHandler();
//            else
//            {
                SPI0ReConfig();

                RS485DeviceInit();  // RS485 initialization
//            }
            lRetStatus = false;
            return lRetStatus;
        }
    }
    if(gRS485RxIntrCount == gRS485ByteCount)   //Entire 255 bytes of data received means then set the event for data handling
    {
//        SPI0ReConfig();

        MsgTimerStop(TIMER0);
        MsgTimerStop(TIMER1);

        gRS485RxIntrCount = 0;
        gRS485ByteCount = 255;

        grlRs485Rx_CmTx_DataHandler();
        lRetStatus = true;

    }
    return lRetStatus;
}
/**
* This is the Generic Wrapper Function for SPI Read and Write Operation.
* @param byteAddress character variable which holds the address of the SPI Peripheral.
* @param byteCount character variable which holds the count that how many number of bytes need to be transfered.
* @param buffer character pointer which holds the base address of the buffer which we are writing to the peripheral.
* @param readBuffer character pointer which holds the base address of the buffer which we are reading from the peripheral.
* @param isRead integer variable which represents the type of operation performed '0' for write and '1' for read.
* @author Prasanna kumar
*/

bool
RS485SpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        bool  isRead)
{
    bool status = true;
    uint8_t i = 0;
    uint8_t lWriteBuf[2] = {0};
    uint8_t lRetryCount = 0;

    if(isRead)                  //SPI Read.
    {
        lWriteBuf[0] = RS485_READ_DATA >> 8;
        lWriteBuf[1] = RS485_READ_DATA & 0xFF;

        grlSpiTransfer (1, 2, lWriteBuf, readBuffer, READ);
    }
    else                      //SPI Write.
    {

//        MsgTimerStart( 0, Timer1_RS485DevInit, TIMER1);
        MsgTimerStart(( ( (byteCount / 48) + 1) * 30), Timer1_RS485DevInit, TIMER1);

        lWriteBuf[0] = RS485_WRITE_DATA >> 8;
        for(i = 0; i < byteCount; i++)
        {
            lWriteBuf[1] = buffer[i];
            do
            {
                status = grlSpiTransfer (byteAddress, 2, lWriteBuf, NULL, isRead);

            }while((status != true) && (lRetryCount++ < 2));
            status = true;lRetryCount = 0;
            DEVICE_DELAY_US(200);//200uS
        }
        if(byteCount == i)
        {
            MsgTimerStop(TIMER1);

            RS485WriteDataDriverDisable();
        }
    }
    return status;
}
bool grlSpiTransmitWords(uint8_t  *buffer, uint16_t  byteCount)
{
    uint8_t i = 0;
    uint16_t ldata = (buffer[0] << 8) | buffer[1];
    for( i = 0; i<byteCount; i++)
    {
        ldata = buffer[i] << 8;
        SPI_writeDataBlockingNonFIFO(SPIA_BASE, ldata);
//        DEVICE_DELAY_US(15);

    }

    return true;
}


uint8_t char_reverse(uint8_t data)
{
    data = (data & 0xF0) >> 4 | (data & 0x0F) << 4;
    data = (data & 0xCC) >> 2 | (data & 0x33) << 2;
    data = (data & 0xAA) >> 1 | (data & 0x55) << 1;
     return data;
}

/*
void display_data_command(uint8_t type,uint8_t data)
{
    uint8_t temp=0,i=0;
    uint8_t data_buffer[16];
    temp=data;
    if(type == DISPLAY_DATA)
    {
        data_buffer[0]=START_DATA;
        temp=temp & 0x0f ;
        data_buffer[1]=char_reverse(temp);
        temp=data;
        temp=temp & 0xf0;
        data_buffer[2]=(char_reverse(temp))<<4;

    }
    else if(type == DISPLAY_COMMAND)
    {
        data_buffer[0]=START_COMMAND;
        temp=temp & 0x0f ;
        data_buffer[1]=char_reverse(temp);
        temp=data;
        temp=temp & 0xf0;
        data_buffer[2]=(char_reverse(temp))<<4;

    }


    {
        LCD_CS_LOW;
        for(i=0;i<3;i++)
        {
          SPI_writeDataBlockingNonFIFO(SPIC_BASE, (data_buffer[i]<<8 ));
        }
        LCD_CS_HIGH;
        DEVICE_DELAY_US(2000);
    }
  //  DEVICE_DELAY_US(2000);
//        LCD_CS_HIGH;
//        LCD_CS_LOW;
//    SPI_writeDataBlockingNonFIFO(SPIC_BASE, (data_buffer[2]<<8 ));
//            LCD_CS_HIGH;
////    DEVICE_DELAY_US(5000);
//            LCD_CS_HIGH;
    }

*/


void display_data_command(uint8_t type,uint8_t data)
{
    uint8_t temp=0,i=0;
    GPIO_writePin(SPIC_LCD_CS_GPIO23, LOW);
    if(type == DISPLAY_DATA)
    {
        temp=START_DATA;
    }
    else if(type == DISPLAY_COMMAND)
    {
        temp=START_COMMAND;
    }

         for(i=0;i<8;i++)
           {

             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, LOW);
             if((temp&0x80)>>7==1)
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, HIGH);
             }
             else
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, LOW);
             }
             temp = temp << 1;
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);
           }

           for(i=0;i<4;i++)
           {
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, LOW);
             if((data&0x01)==1)
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, HIGH);
             }
             else
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, LOW);
             }
             data = data >> 1;
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);
           }
           for(i=0;i<4;i++)
           {
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, LOW);
             GPIO_writePin(SPIC_LCD_MOSI_GPIO20, LOW);
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);
           }
           for(i=0;i<4;i++)
           {
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, LOW);
             if((data&0x01)==1)
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, HIGH);
             }
             else
             {
               GPIO_writePin(SPIC_LCD_MOSI_GPIO20, LOW);
             }
             data = data >> 1;
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);
           }
           for(i=0;i<4;i++)
           {
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, LOW);
             GPIO_writePin(SPIC_LCD_MOSI_GPIO20, LOW);
             GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);
           }
           DEVICE_DELAY_US(10000);

           GPIO_writePin(SPIC_LCD_CS_GPIO23, HIGH);


}

void grllcddisplay(uint8_t line,char *display_data)
{
    int i,j;

 display_data_command(DISPLAY_COMMAND,0x80);  //set DDRAM address to 0x00
       if(line == 1)
       {
         display_data_command(DISPLAY_COMMAND,0x02);  //set DDRAM address to 0x00
          for(i=0;display_data[i]!='\0';i++){    //16 or 20.
              display_data_command(DISPLAY_DATA , display_data[i]);
           }
              DEVICE_DELAY_US(200000);
       }
       else if(line == 2)
       {
//           display_data_command(DISPLAY_COMMAND,0x2A);  //function set (extended command set)
//          display_data_command(DISPLAY_COMMAND, 0x18);
//           display_data_command(DISPLAY_COMMAND,0x80);  //set DDRAM address to 0x00
          display_data_command(DISPLAY_COMMAND, 0xC0);
          for(j=0;display_data[j]!='\0';j++){
              display_data_command(DISPLAY_DATA , display_data[j]);
           }
          DEVICE_DELAY_US(200000);
       }

}

bool grlSpiReceiveWords(uint8_t  *buffer, uint16_t  byteCount)
{

    uint8_t i = 0;
//    uint16_t lread = 0, lread1 = 0;
    for( i = 0; i < 2; i++)
        buffer[i] = SPI_readDataBlockingNonFIFO(SPIA_BASE);
//        lread = SPI_readDataBlockingNonFIFO(SPIA_BASE);

//    lread1 = SPI_readDataBlockingNonFIFO(SPIA_BASE);

    return true;
}
/**
* SPI read / write for programmer application.
* @param byteAddress integer variable which holds the particular gpio number of control card fx3.
* @param byteCount integer variable which represent Initial output on the GPIO if configured as output: CyFalse = 0, CyTrue = 1.
* @param buffer integer variable When set true, the output driver is enabled for outValue = CyFalse,otherwise tristated.
* @param readBuffer integer variable When set true, the output driver is enabled for outValue = CyTrue,otherwise tristated.
* @param isRead integer variable When set true, the input stage on the pin is enabled.
*/

bool
grlSpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        bool  isRead)
{
    bool status = true;

    if (byteCount == 0)
    {
        return true;
    }
    else {
        if (isRead) /*Read*/
        {
           RS485_CS_LOW;
           status = grlSpiTransmitWords (buffer, byteCount);

           DEVICE_DELAY_US(100);//Compulsory delay required between Write and read for read operation

           grlSpiReceiveWords(readBuffer, byteCount);

           RS485DataRecvIntrHandle(readBuffer);

           if (status != true)
            {
                RS485_CS_HIGH;
                return status;
            }
            RS485_CS_HIGH;
        }
        else /*Write*/
        {
            RS485_CS_LOW;
            status = grlSpiTransmitWords (buffer, byteCount);
            if (status != true)
            {
                RS485_CS_HIGH;
                return status;
            }
            DEVICE_DELAY_US(170);
            RS485_CS_HIGH;
        }
    }
    return true;
}

void RS485WriteDataDriverDisable()
{
    uint8_t lWriteBuf[2] = {0};

    lWriteBuf[0] = RS485_WRITE_DATA_DE_DISABLE >> 8;
    lWriteBuf[1] = RS485_WRITE_DATA_DE_DISABLE & 0xFF;

    grlSpiTransfer (1, 2, lWriteBuf, NULL, WRITE);
}
/**
* Use the READ CONFIGURATION register to read back the last configuration written to the UART.
In this mode, bits 15 and 14 of the DIN configuration word are required to be 0 and 1,respectively
to enable the READ CONFIGURATION mode. Clear bits 131 of the DIN word. Bit 0 is the test bit to
put the UART in test mode (see the Test Mode section). Table 3 shows the bit assignment for
the READ CONFIGURATION register.
* @author Prasanna kumar
*/

void RS485ReadConfig()
{
    uint8_t lWriteBuf[2] = {0}, lReadBuf[2] = {0};

    lWriteBuf[0] = RS485_READ_CONFIG >> 8;
    lWriteBuf[1] = RS485_READ_CONFIG & 0xFF;

    grlSpiTransfer (1, 2, lWriteBuf, lReadBuf, WRITE);

}

/**
* Configure the UART by writing a 16-bit word to the WRITE CONFIGURATION register, which programs
the baud rate, data-word length, parity enable, and enable of the 8-word receive FIFO.
* @author Prasanna kumar
*/

void RS485WriteConfig()
{
    uint8_t lWriteBuf[2] = {0};

    lWriteBuf[0] = RS485_WRITE_CONFIG >> 8;
    lWriteBuf[1] = RS485_WRITE_CONFIG & 0xFF;

    grlSpiTransfer (1, 2, lWriteBuf, NULL, WRITE);
}

/**
* Configuration of RS-485 read and write register.Before performing read and write operation
on RS-485 we need to configure the register which is present in the RS-485(MAX3140), in this
function we are configuring Write/Read configuration register.
* and Read configuration register.
* @author Prasanna kuamr
*/

void RS485DeviceInit()
{
    RS485WriteConfig();

    RS485ReadConfig();

    RS485WriteDataDriverDisable();

}
void ControllerGpioInit_CPU2()
{
    EALLOW;

    //
    //CPU2 Vbus PPS DAC Switch Ctrl IO init
    //
    GPIO_setDirectionMode(CPU2_CTRL_VBUS_GPIO12, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CPU2_CTRL_VBUS_GPIO12, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(CPU2_CTRL_VBUS_GPIO12, GPIO_CORE_CPU2);
    GPIO_setQualificationMode(CPU2_CTRL_VBUS_GPIO12, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_12_GPIO12);
    //
    //Venkat,15Nov'22, CPU2 : OCP trigger gpio to Fx3, when pulled low an interrupt will be generated on Fx3 end
    //
    GPIO_setDirectionMode(CPU2_OCP_TRIGGER_GPIO64, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(CPU2_OCP_TRIGGER_GPIO64, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(CPU2_OCP_TRIGGER_GPIO64, GPIO_CORE_CPU2);
    GPIO_setQualificationMode(CPU2_OCP_TRIGGER_GPIO64, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_64_GPIO64);
    EDIS;

//I2C A initialization to CPU2
    GPIO_setPinConfig(GPIO_0_I2CA_SDA);
    GPIO_setPinConfig(GPIO_1_I2CA_SCL);

    GPIO_setPadConfig(GPIO0_PIN_SDAA, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(GPIO1_PIN_SCLA, GPIO_PIN_TYPE_PULLUP);

    GPIO_setQualificationMode(GPIO0_PIN_SDAA, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(GPIO1_PIN_SCLA, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(0, GPIO_CORE_CPU2);
    GPIO_setMasterCore(1, GPIO_CORE_CPU2);

//I2C B initialization to CPU2
    GPIO_setPinConfig(GPIO_34_I2CB_SDA);
    GPIO_setPinConfig(GPIO_35_I2CB_SCL);

    GPIO_setPadConfig(GPIO34_PIN_SDAB, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(GPIO35_PIN_SCLB, GPIO_PIN_TYPE_PULLUP);

    GPIO_setQualificationMode(GPIO34_PIN_SDAB, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(GPIO35_PIN_SCLB, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(0, GPIO_CORE_CPU2);
    GPIO_setMasterCore(1, GPIO_CORE_CPU2);



}
void ControllerGpioInit_CPU1()
{

    EALLOW;

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
//    Device_initGPIO();

    GPIO_setDirectionMode(TC_PMOD_CONTROL_GPIO65, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(TC_PMOD_CONTROL_GPIO65, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(TC_PMOD_CONTROL_GPIO65, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(TC_PMOD_CONTROL_GPIO65, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_65_GPIO65);

    //
    //TC100 Enable Power supply IO
    //
    GPIO_setDirectionMode(TC100_EN_PS_GPIO66, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(TC100_EN_PS_GPIO66, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(TC100_EN_PS_GPIO66, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(TC100_EN_PS_GPIO66, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_66_GPIO66);

    //
    //FAN 1 - GPIO3 - On/OFF Control for Fan1
    //
    GPIO_setDirectionMode(FAN1_GPIO3, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(FAN1_GPIO3, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(FAN1_GPIO3, GPIO_CORE_CPU1);
//    GPIO_setQualificationMode(FAN1_GPIO3, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_3_GPIO3);

    //
    //FAN 1 - PWM - GPIO 2
    //
    GPIO_setDirectionMode(FAN1_PWM_GPIO2, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(FAN1_PWM_GPIO2, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(FAN1_PWM_GPIO2, GPIO_CORE_CPU1);
//    GPIO_setQualificationMode(FAN1_PWM_GPIO2, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_2_EPWM2A);

    //Fan 2 - GPIO 4

    GPIO_setDirectionMode(FAN2_GPIO4, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(FAN2_GPIO4, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(FAN2_GPIO4, GPIO_CORE_CPU1);
//    GPIO_setQualificationMode(FAN2_GPIO4, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_4_GPIO4);

    //
    //FAN 2 - PWM - GPIO5
    //
    GPIO_setDirectionMode(FAN2_PWM_GPIO5, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(FAN2_PWM_GPIO5, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(FAN2_PWM_GPIO5, GPIO_CORE_CPU1);
//    GPIO_setQualificationMode(FAN2_PWM_GPIO5, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_5_EPWM3B);

    //
    //RS485_IRQ_GPIO25
    //
    GPIO_setPinConfig(GPIO_25_GPIO25);

    GPIO_setPadConfig(RS485_IRQ_GPIO25, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(RS485_IRQ_GPIO25, GPIO_DIR_MODE_IN);
    GPIO_setMasterCore(RS485_IRQ_GPIO25, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(RS485_IRQ_GPIO25, GPIO_QUAL_SYNC);

    //
    //TC 100 Card Detection IO
    //
    GPIO_setPinConfig(GPIO_14_GPIO14);
    GPIO_setDirectionMode(TC_BOARD_DET_GPIO14, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(TC_BOARD_DET_GPIO14, GPIO_PIN_TYPE_OD);
    GPIO_setMasterCore(TC_BOARD_DET_GPIO14, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(TC_BOARD_DET_GPIO14, GPIO_QUAL_SYNC);

    EDIS;
    //Board detection
    GPIO_setInterruptType(GPIO_INT_XINT2, GPIO_INT_TYPE_BOTH_EDGES);
    GPIO_setInterruptPin(TC_BOARD_DET_GPIO14, GPIO_INT_XINT2);

    Interrupt_register(INT_XINT2, &TesterCardDetection);
    Interrupt_enable(INT_XINT2);
    GPIO_enableInterrupt(GPIO_INT_XINT2);

    //rs485
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_setInterruptPin(RS485_IRQ_GPIO25, GPIO_INT_XINT1);

    Interrupt_register(INT_XINT1, &gpioInterruptCbHandler);
    Interrupt_enable(INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
}

//configures the GPIOs and Allocates the shared peripherals
void CM_EthernetIOMuxHandler()
{
#if 0
    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_109_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_110_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_75_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_122_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_123_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_124_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_118_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_114_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_115_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_116_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_117_ENET_MII_RX_DATA3);
    GPIO_setPinConfig(GPIO_113_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_112_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_44_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_111_ENET_MII_RX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(108, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(108, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(108,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(119, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(119, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(119,1);

#else
    //
    // Set up EnetCLK to use SYSPLL as the clock source and set the
    // clock divider to 2.
    //
    // This way we ensure that the PTP clock is 100 MHz. Note that this value
    // is not automatically/dynamically known to the CM core and hence it needs
    // to be made available to the CM side code beforehand.
    SysCtl_setEnetClk(SYSCTL_ENETCLKOUT_DIV_2, SYSCTL_SOURCE_SYSPLL);

    //
    // Configure the GPIOs for ETHERNET.
    //

    //
    // MDIO Signals
    //
    GPIO_setPinConfig(GPIO_42_ENET_MDIO_CLK);
    GPIO_setPinConfig(GPIO_43_ENET_MDIO_DATA);
    //
    // Use this only for RMII Mode
    //GPIO_setPinConfig(GPIO_73_ENET_RMII_CLK);
    //

    //
    //MII Signals
    //
    GPIO_setPinConfig(GPIO_40_ENET_MII_CRS);
    GPIO_setPinConfig(GPIO_41_ENET_MII_COL);

    GPIO_setPinConfig(GPIO_59_ENET_MII_TX_DATA0);
    GPIO_setPinConfig(GPIO_60_ENET_MII_TX_DATA1);
    GPIO_setPinConfig(GPIO_61_ENET_MII_TX_DATA2);
    GPIO_setPinConfig(GPIO_62_ENET_MII_TX_DATA3);

    //
    //Use this only if the TX Error pin has to be connected
    //GPIO_setPinConfig(GPIO_46_ENET_MII_TX_ERR);
    //

    GPIO_setPinConfig(GPIO_56_ENET_MII_TX_EN);

    GPIO_setPinConfig(GPIO_52_ENET_MII_RX_DATA0);
    GPIO_setPinConfig(GPIO_53_ENET_MII_RX_DATA1);
    GPIO_setPinConfig(GPIO_54_ENET_MII_RX_DATA2);
    GPIO_setPinConfig(GPIO_55_ENET_MII_RX_DATA3);

    GPIO_setPinConfig(GPIO_51_ENET_MII_RX_ERR);
    GPIO_setPinConfig(GPIO_50_ENET_MII_RX_DV);

    GPIO_setPinConfig(GPIO_58_ENET_MII_TX_CLK);
    GPIO_setPinConfig(GPIO_49_ENET_MII_RX_CLK);

    //
    //Power down pin to bring the external PHY out of Power down
    //
    GPIO_setDirectionMode(68, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(68, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(68,1);

    //
    //PHY Reset Pin to be driven High to bring external PHY out of Reset
    //

    GPIO_setDirectionMode(67, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(67, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(67,1);

#endif
}

void SPI0ReConfig()
{

    //
    // Must put SPI into reset before configuring it.
    //
    SPI_disableModule(SPIA_BASE);

    //50MHZ is LSP freq
    // SPI configuration. Use a 2MHz SPICLK and 8-bit word size.
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_14);

    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
                  SPI_MODE_MASTER, 111000, 8);

    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);

    RS485_CS_HIGH;
    SPI_setSTESignalPolarity(SPIA_BASE,SPI_STE_ACTIVE_LOW);

}

void SPI0_Config()
{
    //
    // Configure SPI pins :
    //  GPIO16 - SPISIMO
    //  GPIO17 - SPISOMI
    //  GPIO18 - SPICLK
    //  GPIO19 - SPICS
    //

    //
    // GPIO16 is the SPISIMOA clock pin.
    //
    GPIO_setMasterCore(SPIA_MOSI_GPIO16, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_16_SPIA_SIMO);
    GPIO_setPadConfig(SPIA_MOSI_GPIO16, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SPIA_MOSI_GPIO16, GPIO_QUAL_ASYNC);

    //
    // GPIO17 is the SPISOMIA.
    //
    GPIO_setMasterCore(SPIA_MISO_GPIO17, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_17_SPIA_SOMI);
    GPIO_setPadConfig(SPIA_MISO_GPIO17, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SPIA_MISO_GPIO17, GPIO_QUAL_ASYNC);

    //
    // GPIO18 is the SPICLKA.
    //
    GPIO_setMasterCore(SPIOA_SCLK_GPIO18, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_18_SPIA_CLK);
    GPIO_setPadConfig(SPIOA_SCLK_GPIO18, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SPIOA_SCLK_GPIO18, GPIO_QUAL_ASYNC);

    //
    // GPIO11 is the SPICS.
    //
    GPIO_setMasterCore(SPIOA_CS_GPIO19, GPIO_CORE_CPU1);
    GPIO_setPinConfig(GPIO_19_GPIO19);
    GPIO_setPadConfig(SPIOA_CS_GPIO19, GPIO_PIN_TYPE_OD);
    GPIO_setQualificationMode(SPIOA_CS_GPIO19, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPIOA_CS_GPIO19, GPIO_DIR_MODE_OUT);
    //
    // Must put SPI into reset before configuring it.
    //
    SPI_disableModule(SPIA_BASE);

    //50MHZ is LSP freq
    // SPI configuration. Use a 2MHz SPICLK and 8-bit word size.
    //
    SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_14);

    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
                  SPI_MODE_MASTER, 111000, 8);

    SPI_setEmulationMode(SPIA_BASE, SPI_EMULATION_FREE_RUN);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(SPIA_BASE);

    RS485_CS_HIGH;
    SPI_setSTESignalPolarity(SPIA_BASE,SPI_STE_ACTIVE_LOW);


}

void SPI1_LCD_Config()
{
    //
    // Configure SPI pins :
    //  GPIO20 - SPISIMO
    //  GPIO21 - SPISOMI
    //  GPIO22 - SPICLK
    //  GPIO23 - SPICS
    //

    //
    // GPIO20 is the SPISIMOC clock pin.
    //
//    GPIO_setMasterCore(SPIC_LCD_MOSI_GPIO20, GPIO_CORE_CPU1);
//    GPIO_setPinConfig(GPIO_20_SPIC_SIMO);
//    GPIO_setPadConfig(SPIC_LCD_MOSI_GPIO20, GPIO_PIN_TYPE_PULLUP);
//    GPIO_setQualificationMode(SPIC_LCD_MOSI_GPIO20, GPIO_QUAL_ASYNC);

    GPIO_setDirectionMode(SPIC_LCD_MOSI_GPIO20, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(SPIC_LCD_MOSI_GPIO20, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(SPIC_LCD_MOSI_GPIO20, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(SPIC_LCD_MOSI_GPIO20, GPIO_QUAL_SYNC);
    GPIO_writePin(SPIC_LCD_MOSI_GPIO20, HIGH);

    //
    // GPIO21 is the SPISOMIC.
    //
//    GPIO_setMasterCore(SPIC_LCD_MISO_GPIO21, GPIO_CORE_CPU1);
//    GPIO_setPinConfig(GPIO_21_SPIC_SOMI);
//    GPIO_setPadConfig(SPIC_LCD_MISO_GPIO21, GPIO_PIN_TYPE_PULLUP);
//    GPIO_setQualificationMode(SPIC_LCD_MISO_GPIO21, GPIO_QUAL_ASYNC);

    GPIO_setDirectionMode(SPIC_LCD_MISO_GPIO21, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(SPIC_LCD_MISO_GPIO21, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(SPIC_LCD_MISO_GPIO21, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(SPIC_LCD_MISO_GPIO21, GPIO_QUAL_SYNC);


    //
    // GPIO22 is the SPICLKC.
    //
//    GPIO_setMasterCore(SPIC_LCD_SCLK_GPIO22, GPIO_CORE_CPU1);
//    GPIO_setPinConfig(GPIO_22_SPIC_CLK);
//    GPIO_setPadConfig(SPIC_LCD_SCLK_GPIO22, GPIO_PIN_TYPE_PULLUP);
//    GPIO_setQualificationMode(SPIC_LCD_SCLK_GPIO22, GPIO_QUAL_ASYNC);

    GPIO_setDirectionMode(SPIC_LCD_SCLK_GPIO22, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(SPIC_LCD_SCLK_GPIO22, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(SPIC_LCD_SCLK_GPIO22, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(SPIC_LCD_SCLK_GPIO22, GPIO_QUAL_SYNC);
    GPIO_writePin(SPIC_LCD_SCLK_GPIO22, HIGH);

    //
    // GPIO23 is the SPICS.
    //
    GPIO_setMasterCore(SPIC_LCD_CS_GPIO23, GPIO_CORE_CPU1);
    GPIO_setPadConfig(SPIC_LCD_CS_GPIO23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(SPIC_LCD_CS_GPIO23, GPIO_QUAL_ASYNC);
    GPIO_setDirectionMode(SPIC_LCD_CS_GPIO23, GPIO_DIR_MODE_OUT);
    //
     // Must put SPI into reset before configuring it.
     //
//     SPI_disableModule(SPIC_BASE);
//
//     SysCtl_setLowSpeedClock(SYSCTL_LSPCLK_PRESCALE_14);
//     //
//     // SPI configuration. Use a 2MHz SPICLK and 8-bit word size.
//     //
//     SPI_setConfig(SPIC_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL1PHA1,
//                   SPI_MODE_MASTER, 400000, 8);
//     SPI_setEmulationMode(SPIC_BASE, SPI_EMULATION_FREE_RUN);
//     //
//     // Configuration complete. Enable the module.
//     //
//     SPI_enableModule(SPIC_BASE);
////     LCD_CS_HIGH;
//     SPI_setSTESignalPolarity(SPIC_BASE,SPI_STE_ACTIVE_LOW);

}

void lcd_init()
{
//    DEVICE_DELAY_US(100);
    display_data_command(DISPLAY_COMMAND, 0x2A);  //function set (extended command set)
    display_data_command(DISPLAY_COMMAND,0x71);  //function selection A, disable internal Vdd regualtor
    display_data_command(DISPLAY_DATA,0x00);
    display_data_command(DISPLAY_COMMAND,0x28);  //function set (fundamental command set)
    display_data_command(DISPLAY_COMMAND,0x08);  //display off, cursor off, blinku off
    display_data_command(DISPLAY_COMMAND,0x2A);  //function set (extended command set)
    display_data_command(DISPLAY_COMMAND,0x79);  //OLED command set enabled
    display_data_command(DISPLAY_COMMAND,0xD5);  //set display clock divide ratio/oscillator frequency
    display_data_command(DISPLAY_COMMAND,0x70);  //set display clock divide ratio/oscillator frequency
    display_data_command(DISPLAY_COMMAND,0x78);  //OLED command set disabled
    display_data_command(DISPLAY_COMMAND,0x08);  //extended function set (4-lines)
    display_data_command(DISPLAY_COMMAND,0x06);  //COM SEG direction
    display_data_command(DISPLAY_COMMAND,0x72);  //function selection B, disable internal Vdd regualtor
    display_data_command(DISPLAY_DATA,0x00);     //ROM CGRAM selection
    display_data_command(DISPLAY_COMMAND,0x2A);  //function set (extended command set)
    display_data_command(DISPLAY_COMMAND,0x79);  //OLED command set enabled
    display_data_command(DISPLAY_COMMAND,0xDA);  //set SEG pins hardware configuration
    display_data_command(DISPLAY_COMMAND,0x00);  //set SEG pins hardware configuration   ////////////////////////////////////0x10 on other slim char OLEDs
    display_data_command(DISPLAY_COMMAND,0xDC);  //function selection C
    display_data_command(DISPLAY_COMMAND,0x00);  //function selection C
    display_data_command(DISPLAY_COMMAND,0x81);  //set contrast control
    display_data_command(DISPLAY_COMMAND,0x7F);  //set contrast control
    display_data_command(DISPLAY_COMMAND,0xD9);  //set phase length
    display_data_command(DISPLAY_COMMAND,0xF1);  //set phase length
    display_data_command(DISPLAY_COMMAND,0xDB);  //set VCOMH deselect level
    display_data_command(DISPLAY_COMMAND,0x40);  //set VCOMH deselect level    //done
    display_data_command(DISPLAY_COMMAND,0x78);  //OLED command set disabled
    display_data_command(DISPLAY_COMMAND,0x28);  //function set (fundamental command set)
    display_data_command(DISPLAY_COMMAND,0x01);  //clear display
    display_data_command(DISPLAY_COMMAND,0x80);  //set DDRAM address to 0x00
    display_data_command(DISPLAY_COMMAND,0x0C);  //display ON
//    DEVICE_DELAY_US(100);

}

//
// Function to configure SPI A in FIFO mode.
//
void initSPI()
{
    SPI0_Config();
    SPI1_LCD_Config();
}

//void i2cB_init()
//{
//    Interrupt_register(INT_I2CB, &I2C_FRAM_ISR);
//    Init_I2C_Master();
//    Interrupt_enable(INT_I2CB);
//
//}
