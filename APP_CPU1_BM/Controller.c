/*
 * Controller.c
 *
 *  Created on: 09-Mar-2022
 *      Author: prana
 */
#include "Includes.h"
#include "Controller.h"
#include "PD_ControllerStruct.h"
#include "ControllerMain.h"
uint8_t glFirmwareID[8] = { '6', '1', '2', '\0' };


uint8_t CmDataRxbuf[286];
uint8_t CmDataTxbuf[286];
uint8_t gRS485RxBuf[FW_PL_LENGTH];
uint8_t gPollingBuf[36];
uint8_t gFWOpcdeBuf[FW_PL_LENGTH];

uint16_t rUARTDataA[16];
uint16_t rUARTDataPointA;
uint16_t sDataA[2];

volatile uint8_t gindex;
volatile uint16_t gRS485RxIntrCount;
volatile uint16_t gRS485ByteCount;
volatile uint16_t gRS485ReInitDevCount;
volatile uint8_t gErrRetryCount;
volatile uint16_t gFWOpcodeBufLength;

uint32_t gVar[286];
static int gcount;
#pragma DATA_SECTION(gRS485RxBuf, "MSGRAM_CPU_TO_CM")
#pragma DATA_SECTION(gPollingBuf, "MSGRAM_CM_TO_CPU_IPC1")

extern float temp;

void PollingDataRxHandling(uint8_t *rBuffer)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

//    uint8_t CheckBit = (rBuffer[0] << 4);
//    uint8_t lRS485RxByteCount = (rBuffer[1] + 2);

    uint8_t Functionalcard_1_StatusInfo[BUF_SIZE_36BYTE] = {0};
    uint8_t TestCard_StatusInfo[40] = {0};

    Functionalcard_1_StatusInfo[0] = 0x01;//TC ID

    memcpy( &Functionalcard_1_StatusInfo[1], &rBuffer[3], 29);//Copying the received polling data from TC,20 actual BYtes of polling data will be received from TC

    TestCard_StatusInfo[0] = 34;//Total payload length inc. Header
    TestCard_StatusInfo[1] = 29;//Each TC Polling data length

    memcpy(&TestCard_StatusInfo[2], &Functionalcard_1_StatusInfo[0], 30);//20Bytes of polling data (20Bytes received from TC and 1 Bytes TC ID)

    TestCard_StatusInfo[31] = 0xAB;//Temp sensor keyword
    TestCard_StatusInfo[32] = (uint8_t) temp;//Temp sensor data, temp is a global temperature data variable

    rBuffer[0] = 0x1F;//Keyword for CM to identify polling command
    rBuffer[1] = 0x00;//Res// As part of 16 bit addressing mecahnism we cant neglect only1Byte in Cm,so 16 bit will be neglected

    memcpy(&rBuffer[2], &TestCard_StatusInfo[0], (TestCard_StatusInfo[0]+2));
}

//
// sciaTXFIFOISR - SCIA Transmit FIFO ISR
//
__interrupt void sciaTXFIFOISR(void)
{
    uint16_t i;

    SCI_writeCharArray(SCIA_BASE, sDataA, 2);


//     Increment send data for next cycle

    for(i = 0; i < 2; i++)
    {
        sDataA[i] = (sDataA[i] + 1) & 0x00FF;
    }

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_TXFF);

    //
    // Issue PIE ACK
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}

void FWOpcodeDataAppend(uint8_t * aFWOpcodeBuffer, uint8_t aFWPayloadLength)
{

    memcpy(&gFWOpcdeBuf[gFWOpcodeBufLength], aFWOpcodeBuffer, aFWPayloadLength);
    gFWOpcodeBufLength += aFWPayloadLength;

}

void FWRecvCmdHandler(uint8_t * aFWBuffer)
{

    switch(aFWBuffer[0])
    {
        case 0xC2://FW Streaming mode

                FWOpcodeDataAppend( &aFWBuffer[2], aFWBuffer[1] );

        break;

        default:
                switch(aFWBuffer[4])
                {
                    case 0x09://Flash Write

                        memset(gRS485RxBuf, 0x00, FW_PL_LENGTH);

                        memcpy(&gRS485RxBuf[0], aFWBuffer, (aFWBuffer[1]+2) );

                        memcpy(&gRS485RxBuf[ (aFWBuffer[1]+2) ], gFWOpcdeBuf, gFWOpcodeBufLength);


                        IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                                          IPC_CMD_READ_MEM, (uint32_t)gRS485RxBuf, ((gFWOpcodeBufLength+6)) );

                        IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);

                        memset(gFWOpcdeBuf, 0x00, FW_PL_LENGTH);
                        gFWOpcodeBufLength = 0;

                    break;

                    case 0x04:
                        SysCtl_resetDevice();
                        break;

                    default://Erase/PGM/BOOT cmds

                        if(aFWBuffer[4] == 0x02)//CM PGM
                            grllcddisplay(1, "CM-PGM MODE   ");

                        memcpy(&gRS485RxBuf[0], aFWBuffer, (aFWBuffer[1]+2) );

//                        grlRs485Rx_CmTx_DataHandler();
                        IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                                          IPC_CMD_READ_MEM, (uint32_t)gRS485RxBuf, (gRS485RxBuf[1] + 2 ) );

                        IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);

                        memset(gFWOpcdeBuf, 0x00, FW_PL_LENGTH);
                        gFWOpcodeBufLength = 0;
                        break;
                }
            break;
    }
}
//
// sciaRXFIFOISR - SCIA Receive FIFO ISR
//
__interrupt void sciaRXFIFOISR(void)
{
//    uint16_t i;

    SCI_readCharArray(SCIA_BASE, rUARTDataA, 16);

//    rUARTDataPointA = (rUARTDataPointA + 1) & 0x00FF;

    SCI_clearOverflowStatus(SCIA_BASE);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    //
    // Issue PIE ack
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    if( 0x02 == (rUARTDataA[0] & 0x0F) )
        FWRecvCmdHandler(rUARTDataA);

//    else if( 0xBB == (rUARTDataA[0] ) ) //API Rx Should be  0xBB, 0x01, 0xFF -> 0xFF for All registers read, Else Specific Register data only
//    {
 //       IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
 //                         IPC_CMD_READ_MEM, (uint32_t)rUARTDataA, (rUARTDataA[1] + 2 ) );
//
//        IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);
//    }
    else
        RecvDataHandler(rUARTDataA);

}

//
//Pranay,25march'22, Sending received data from CPU1 to CM and making RS485Bus Flag IDLe for handling polling
//This function needs to be called if we want any data to be pushed from CPU2 to CM,
// Data that needs to be transferred shall reside in gRS485RxBuf
//
void grlRs485Rx_CmTx_DataHandler()
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();


    switch( gRS485RxBuf[2] )
    {

        case 0x0C:

            PollingDataRxHandling(gRS485RxBuf);

//            memcpy(gPollingBuf,gRS485RxBuf,36);

            IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                              IPC_CMD_READ_MEM, (uint32_t)gRS485RxBuf, gRS485RxBuf[2]+2);

            IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG1);

            break;

        default:
            IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                              IPC_CMD_READ_MEM, (uint32_t)gRS485RxBuf, gRS485RxBuf[1]);

            IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);
            break;

    }

#ifdef POLLING
    gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;
    gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag = RUNTIME_API_HANDLED;
#endif

}
/**
* Function to send the polling command to get the PDC status information.
* @author Harsha
* @param lDataType Character variable which represent the
* @param lPortID Character variable holds the particular card number.
* @date 22Jan'20
*/

void GetPDCStatus(uint8_t lDataType, uint8_t lPortID)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    gControlStruct_t = (gControlStruct *)GetStructPtr();
    uint8_t FuncardInfo[] = {0x0F,0x01,0x0C};    //Get the Test card PDC,LED status & Eload data.
    uint8_t lRS485TxByteCount = (FuncardInfo[1] + 2);
    FuncardInfo[0] |=(lPortID << 4); //transfer port_id (the port_id is stored in the 0th index from 7:4(higher nibble))

    gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_BUSY;

    RS485SpiTransfer (1,lRS485TxByteCount, FuncardInfo, NULL, WRITE);

}

void InitFWInstances()
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    if(GPIO_readPin(TC_BOARD_DET_GPIO14) == true)
    {
        TURN_ON_TC;
        gControlStruct_t->gTestHandle_t.gTestControl_t.TestCard_PowerState = TC_CONNECTED;
    }

    GPIO_writePin(TC_PMOD_CONTROL_GPIO65, 0);//Making PMOD pin of TC to low

    DEVICE_DELAY_US(10000);

    RS485DeviceInit();

#ifdef POLLING
    gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = true;//Turning On Polling By default

    MsgTimerStart(50000, Timer0_Polling_Expiry_Handler, TIMER0);
#else
    gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = false;
#endif
#ifndef POLLING
//    MsgTimerStart(2000, Timer0_TempDataRead, TIMER0);
#endif
        MsgTimerStart(2000, Timer2_TempDataRead, TIMER2);

}
__interrupt void gpioInterruptCbHandler()
{

    gcount++;
    uint8_t lReadBuf[2] = {0};
    RS485SpiTransfer(1, NULL, NULL, lReadBuf, READ);


    GPIO_togglePin(RS485_IRQ_GPIO25);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

}

__interrupt void TesterCardDetection()
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    if(GPIO_readPin(TC_BOARD_DET_GPIO14) == true)
    {
        TURN_ON_TC;
        gControlStruct_t->gTestHandle_t.gTestControl_t.TestCard_PowerState = TC_CONNECTED;
    }
    else
    {
        TURN_OFF_TC;
        gControlStruct_t->gTestHandle_t.gTestControl_t.TestCard_PowerState = TC_DISCONNECTED;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

void FanControlHandler(uint8_t aVar)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    switch(aVar)
    {
        case 0: //ON ALL

            pwmSignal.dutyValB = 1.0;
            EPWM_configureSignal(EPWM2_BASE, &pwmSignal);
        break;

        case 1: //OFF ALL
            pwmSignal.dutyValB = 0.5;
            EPWM_configureSignal(EPWM2_BASE, &pwmSignal);

        break;

        case 2:
            pwmSignal.dutyValB = 0.0;
            EPWM_configureSignal(EPWM2_BASE, &pwmSignal);

        break;
    }

}

void GetDataCmdHandler(uint8_t * aRxBuffer)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    uint8_t lcmdType = aRxBuffer[2];
    uint8_t lRxByteCnt = aRxBuffer[1] + 2;

    memset(CmDataTxbuf, 0x00, 128);
    memset(gRS485RxBuf, 0x00, 256);

    gcount = 0;

    switch(lcmdType)
    {
    case 0x01://FW Versions fetch
        if(aRxBuffer[3] == 0x01)//CC FW v fetch
        {

            memcpy(&gRS485RxBuf[0], aRxBuffer, 3);//Appending received data

            memcpy(&gRS485RxBuf[3], glFirmwareID, 3);//FW version

            gRS485RxBuf[1] = (aRxBuffer[1] + 4 + HEADER_BYTE_CNT);

            grlRs485Rx_CmTx_DataHandler();

        }
        else
        {
            //Pranay,18Oct'22, Sending error command word incase if response is not received for FW version fetch aswell, not handled in earlier release so handling now
            SPI0ReConfig();
            gRS485RxIntrCount = 0;//Resetting the required variables used while reading.
            gRS485ByteCount = 255;
            MsgTimerStop(TIMER0);//Stopping the timers
            MsgTimerStop(TIMER1);

            RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);
            MsgTimerStart(400, Timer0_Read_Expiry_Timer, TIMER0);

        }
        break;
    case 0x84://Get temp data

        memcpy(&gRS485RxBuf[0], aRxBuffer, 3);//Appending received data

//        memcpy(&gRS485RxBuf[3], (uint8_t)temp, 1);//FW version

        gRS485RxBuf[3] = (uint8_t) temp ;

        gRS485RxBuf[1] = (aRxBuffer[1] + 2 + HEADER_BYTE_CNT);

        grlRs485Rx_CmTx_DataHandler();

        break;

    default://APIs related to TC will all be handled here //Any get command related to TC is expected to hit here

        SPI0ReConfig();
        gRS485RxIntrCount = 0;//Resetting the required variables used while reading.
        gRS485ByteCount = 255;
        MsgTimerStop(TIMER0);//Stopping the timers
        MsgTimerStop(TIMER1);

        RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);
        MsgTimerStart(400, Timer0_Read_Expiry_Timer, TIMER0);

        break;
    }

}

//Pranay,02Sept'22,To ensure IP address is getting resetted everytime, need to make ip_add as volatile so that no optimizations shall be done by MCU
volatile  uint8_t ip_add[17];

uint8_t* ip_addrss_formation(uint32_t ip)
{
   uint8_t num=0,i,j,k,temp=0;
   int8_t l;
   for(l=24,k=0;l>=0;l=l-8)
   {
    num=ip >> l;
    num=num&0xff;
    if (num==0)
    {
        ip_add[k++]='0';
    }
    for(i=0;num!=0;i++)
    {
        j=num%10;
        num=num/10;
        ip_add[k++]=j+'0';

    }
    if(num==0 && i>1)
           {
               if(i==3)
               {
                   temp=ip_add[k-3];
                   ip_add[k-3]=ip_add[k-1];
                   ip_add[k-1]=temp;
               }
               else if(i==2)
               {
                   temp=ip_add[k-2];
                  ip_add[k-2]=ip_add[k-1];
                  ip_add[k-1]=temp;
               }

           }

    if(l!=0){
    ip_add[k++]='.';
    }
   }
return ip_add;
}

uint8_t* sys_id_formation(uint8_t * aRxBuffer)
{
    static uint8_t sysid[16];
    uint16_t year=0,sys_id=0,temp=0,k=6,j=0,i=0;
    memset(sysid, '0', 16);
          sysid[0]='V';
          sysid[1]='D';
          sysid[2]='P';
          sysid[3]='W';
          sysid[4]='R';
          sysid[5]='-';
          year= aRxBuffer[3]<<8 |aRxBuffer[4];
          sys_id = aRxBuffer[5]<<8 |aRxBuffer[6];
          while(year!=0)
          {
              j=year%10;
              year=year/10;
              sysid[k++]=j+'0';
          }
            temp=sysid[k-4];
            sysid[k-4]=sysid[k-1];
            sysid[k-1]=temp;
            temp=sysid[k-3];
            sysid[k-3]=sysid[k-2];
            sysid[k-2]=temp;

        while(sys_id!=0)
        {
            j=sys_id%10;
            i++;
            sys_id=sys_id/10;
            sysid[k++]=j+'0';
        }
          if(i<4)
          {
              if(i==1)
              {
                 sysid[k+2] =sysid[k-1];
                 sysid[k+1]='0';
                 sysid[k]='0';
                 sysid[k-1]='0';
              }
              else if(i==2)
              {
                  sysid[k+1] =sysid[k-2];
                  sysid[k]=sysid[k-1];
                  sysid[k-1]='0';
                  sysid[k-2]='0';

              }
              else if(i==3)
              {
                  temp=sysid[k-1];
                  sysid[k]=sysid[k-3];
                  sysid[k-1]=sysid[k-2];
                  sysid[k-2]=temp;
                  sysid[k-3]='0';
              }
          }
          sysid[14]='\0';
return sysid;

}

void SetDataCmd(uint8_t * aRxBuffer)
{
    uint8_t lcmdType = aRxBuffer[2];
    uint8_t lRxByteCnt = aRxBuffer[1] + 2;

    if((aRxBuffer[3] == 0x02) || (aRxBuffer[3] == 0x0A)) // checking whether the particular command is related to eload or not (if --> true we need to control the FAN)
    {
        if (aRxBuffer[8] == 0x00)
            pwmSignal.dutyValB = 0.5;
        else /*if( (aRxBuffer[8] & 0x01) == 0x00)*/
            pwmSignal.dutyValB = 1.0;

        EPWM_configureSignal(EPWM2_BASE, &pwmSignal);
    }
    RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);

}

void Ti_CPU1BootModeSelection(uint8_t * aRxBuffer)
{
    uint8_t lBootMode = aRxBuffer[4];

    if(lBootMode == 0x02) //Program mode received -> Enter in to SSBL
    {
        grlCPU1PGMmodeSetHandler(aRxBuffer);
//        SysCtl_resetDevice();
    }
    else
    {

    }

}
void is_TiFWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lCpuSelection = aRxBuffer[3];

    switch(lCpuSelection)
    {
    case TI_CM://0x00
        SysCtl_controlCMReset(SYSCTL_CORE_ACTIVE);
//        MsgTimerStart(TIMER_1, 2000);
        break;
    case TI_CPU1_CONTROLCARD://0x01

        Ti_CPU1BootModeSelection(aRxBuffer);

        break;

    case TI_CPU2_PPS://0x02

        break;
    }

}
void isAPI_FWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lFwController = aRxBuffer[2];
    uint16_t lRxByteCnt = aRxBuffer[1] + 2;

    switch(lFwController)
    {
	//Pranay,20Oct'22,for handling the SSBL FW update of TC, handling in case specific manner 
        case TC_PGM_MODE_SEL://TC Program mode selection

            switch(aRxBuffer[3])
            {
                default:
                case BOOT_MODE_SELECTION:
                case PROGRAMMING_MODE:
                case VIA_2ND_STAGE_BL:
                    RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);
                    break;
                case PMOD_SELECTION_ON:
                        GPIO_writePin(TC_PMOD_CONTROL_GPIO65, 1);//Making PMOD pin of TC to low
                    break;
                case PMOD_SELECTION_OFF:
                       GPIO_writePin(TC_PMOD_CONTROL_GPIO65, 0);//Making PMOD pin of TC to low
                   break;
            }

            break;
        case TC_PROGRAMMING_FW_PAYLOAD_WRITE://TC FX3 Fw updates
        case CCG3PA_PGM_MODE_SELECTION:
        case CCG3PA_PROGRAMMING_FW_PAYLOAD_WRITE:

            RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);

            break;
        case TI_FW_UPDATES ://Ti
            is_TiFWUpdate(aRxBuffer);
            break;
        default: // ELOAD
            RS485SpiTransfer(1, lRxByteCnt, aRxBuffer, NULL, WRITE);

            break;

    }

}
void grlIPCDataRxHandler(uint8_t * aRxBuffer)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    uint8_t lRxCmdType = (aRxBuffer[0] & 0x0F);


    switch(lRxCmdType)
    {
        case RX_API_IS_SET:

            SetDataCmd(aRxBuffer);

#ifdef POLLING
            gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;//Making RS485 Bus flag as IDLE after pushing data to  TC
            gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag = RUNTIME_API_HANDLED;
#endif
            break;

        case RX_API_IS_PGM:

            //Pranay,03Sept'22, For FW updates we need to replace the upper nibble of 0th Byte i.e., slot ID to 1 always else commands will be ignored
            aRxBuffer[0] &= 0x0F;
            aRxBuffer[0] |= 0x10;

            //If PGM mode is going on we can stop polling till FW updates are done
            gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = false;
            isAPI_FWUpdate(aRxBuffer);

#ifdef POLLING
            gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;//Making RS485 Bus flag as IDLE after pushing data to  TC
            gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag = RUNTIME_API_HANDLED;
#endif
            break;

        case Rx_API_IS_GET:

             GetDataCmdHandler(aRxBuffer);// Making Rs485 bus as IDLE incase of read command is done after receiving data back from TC in grlRs485Rx_CmTx_DataHandler function

//             MsgTimerStart(70, Timer0_Read_Expiry_Timer, TIMER0);

            break;
    }

}

__interrupt void CPU2_CPU1_IPC_ISR1()
{
    uint32_t command, addr, data;
//
    IPC_readCommand(IPC_CPU1_L_CPU2_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);
    IPC_ackFlagRtoL(IPC_CPU1_L_CPU2_R, IPC_FLAG2);

    int i =0,index = 0;
    uint32_t lVar[286] = {0};
    uint16_t lTxBuf[4] = {0};

    memset(gRS485RxBuf, 0x00, MAX_DATA_TX_SIZE);

    for (i = 0; i < (data); i++)
    {
        lVar[0+i] = (*((uint32_t *)addr + i) );
    }
    for (i = 0; i < ( ( (data/4) + 1) * 2); i++)
    {

        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        gRS485RxBuf[index++] = lTxBuf[0] ;

        gRS485RxBuf[index++] = (lTxBuf[2]) ;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    grlRs485Rx_CmTx_DataHandler();

}

void ControlCommandhandler(uint8_t * aRxBuffer)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    uint8_t lcmdType = aRxBuffer[2];

    uint32_t ip_address;
    uint8_t *ip,*sysid;

    uint8_t lRxCmdType = (aRxBuffer[0] & 0x0F);

    switch(lRxCmdType)
    {

    case RX_API_IS_SET:

        switch(lcmdType)
        {
            case 0x01://Control card

               switch(aRxBuffer[3])
               {
                   case 0x01: //TC reset handling

                       if(aRxBuffer[4] == 0x01)
                           TURN_ON_TC;
                       else
                           TURN_OFF_TC;

                   break;

                   case 0x03://Polling control
#ifdef POLLING
                       if(aRxBuffer[4] == 0x00)//stop
                       {
                           //If PGM mode is going on we can stop polling till FW updates are done
                           gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = false;
                           MsgTimerStop(TIMER0);
                       }
                       else if(aRxBuffer[4] == 0x01)//Start
                       {
                           gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = true;//Turning On Polling By default

                           MsgTimerStart(10000, Timer0_Polling_Expiry_Handler, TIMER0);
                       }
#endif
                       break;
                   case 0x04://Control card reset

                           SysCtl_resetDevice();

                       break;

                   case 0x06:

                       gRS485RxIntrCount = 0;
                       gRS485ByteCount = 255;
                       gRS485ReInitDevCount = 0;
                       RS485DeviceInit();

                       break;

                   case 0x07:

                       SPI0ReConfig();

                       break;
               }
               break;

           case 0x05://Related to display

               aRxBuffer[3]=aRxBuffer[3]<<8|aRxBuffer[4];
               aRxBuffer[5]=aRxBuffer[5]<<8|aRxBuffer[6];
               ip_address=aRxBuffer[3];
               ip_address=ip_address<<16;
               ip_address=ip_address | aRxBuffer[5];

               ip = ip_addrss_formation(ip_address);

               //Pranay,02Sept'22,Clearing existing IP display before updating IP address
               grllcddisplay(2, "                ");

               grllcddisplay(2, ip);

               break;

           case 0x06://Related to display

               sysid=  sys_id_formation(aRxBuffer);
               grllcddisplay(1, sysid);

           break;
           }

        break;

    case RX_API_IS_PGM:
        //Pranay,03Sept'22, For FW updates we need to replace the upper nibble of 0th Byte i.e., slot ID to 1 always else commands will be ignored
        aRxBuffer[0] &= 0x0F;
        aRxBuffer[0] |= 0x10;

        //If PGM mode is going on we can stop polling till FW updates are done
        gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled = false;
        isAPI_FWUpdate(aRxBuffer);

#ifdef POLLING
        gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;//Making RS485 Bus flag as IDLE after pushing data to  TC
        gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag = RUNTIME_API_HANDLED;
#endif
        break;

    case Rx_API_IS_GET:

        GetDataCmdHandler(aRxBuffer);

        break;

    }

}

void RecvDataHandler(uint8_t * aRxBuffer)
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    if( (aRxBuffer[2] == 0x01) || (aRxBuffer[2] == 0x05) || (aRxBuffer[2] == 0x06) ) //If commands are related to control card data, handle directly
    {
//        grlIPCDataRxHandler(aRxBuffer);

        ControlCommandhandler(aRxBuffer);
    }
    else
    {

#ifdef POLLING

        gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag = RUNTIME_API_RECEIVED;
        if(gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus == RS485_BUSY)
        {
            MsgTimerStart(55,Timer1_RunTime_CmdHandler, TIMER1); //if the index value reach max value reset the index and start the polling timer.
        }
        else
        {
            gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_BUSY;
            grlIPCDataRxHandler(CmDataRxbuf);
        }
#else
        grlIPCDataRxHandler(CmDataRxbuf);

#endif
    }


}
__interrupt void CM_CPU1_IPC_ISR1()
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    uint32_t command, addr, data;

    IPC_readCommand(IPC_CPU1_L_CM_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);
    IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1);

    int i =0,index = 0;

    for (i = 0; i < (data); i++)
    {
        gVar[0+i] = (*((uint32_t *)addr + i) );
    }
    for (i = 0; i < (data/4)+1; i++)
    {
        CmDataRxbuf[index++] = (gVar[i] & 0xFF);
        CmDataRxbuf[index++] = (gVar[i] & 0xFF00) >> 8;
        CmDataRxbuf[index++] = (gVar[i] & 0xFF0000) >> 16;
        CmDataRxbuf[index++] = (gVar[i] & 0xFF000000) >> 24;
    }
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    if( (CmDataRxbuf[0] & 0xFF)== PPS_PGM_MODE)
    {
        grllcddisplay(1, "PPS-PGM MODE  ");
    }
    else if( (CmDataRxbuf[0] & 0xFF)== PPS_FW_UPD)
    {
        grllcddisplay(1, "PPS- FW UPD...");
    }
    else if((CmDataRxbuf[0] & 0xFF)== CM_PGM_MODE)
    {
        grllcddisplay(1, "CM-PGM MODE ");
    }
    else if((CmDataRxbuf[0] & 0xFF)== 0x0d)
    {
        sDataA[0]=CmDataRxbuf[0] & 0xFF;
        sDataA[1]=CmDataRxbuf[1] & 0xFF ;
        SCI_writeCharArray(SCIA_BASE, sDataA, 10);
    }


//    CM_IPC_DataReceived = true;
    RecvDataHandler(CmDataRxbuf);


}

