/*
 * Grl_PD_Comm.c
 *
 *  Created on: 01-Feb-2022
 *      Author: pranay
 */

#include <GrlPDComm.h>
#include "flash_programming_f2838x_cm.h"
#include "F021_F2838x_CM.h"

#pragma DATA_SECTION(buf_tx_CM_to_CPU2, "MSGRAM_CM_TO_CPU2")

u8_t buf_tx_CM_to_CPU2[286];

#define SYSCTL_STATUS_CMSYSRESETREQ 0x0004U

void DecodeSystemDetails(uint16_t * aBuffer)
{
    gSystemID =  ( (aBuffer[2] & 0xFF) << 8)  | ( aBuffer[1] >> 8 );
    gMFDMonth = aBuffer[2] >> 8;
    gMFDYear = aBuffer[3];
}



void CM_reset()
{
    uint32_t *lRegVal;

    volatile uint32_t lAircr_reg = AIRCR_REG;


    lRegVal = &lAircr_reg;

    *lRegVal |= SYSCTL_STATUS_CMSYSRESETREQ;

}

void Tx_cpu2_intr(uint8_t acmd)
{
    buf_tx_CM_to_CPU2[0]=acmd;
    IPC_sendCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,IPC_CMD_READ_MEM, (uint32_t)buf_tx_CM_to_CPU2, 2);
    IPC_waitForAck(IPC_CM_L_CPU2_R, IPC_FLAG3);
}
void Tx_cpu1_intr(uint8_t acmd)
{
    buf_rx[0]=acmd;
    IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,IPC_CMD_READ_MEM, (uint32_t)buf_rx, 2);
     IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);
}
//*****************************************************************************
//  Example_EraseSector
//
//  Example function to Erase data of a sector in Flash.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_EraseSector, ".TI.ramfunc");
#endif
bool Example_EraseSector(uint32 Flash_sector)
{
    Fapi_StatusType  oReturnCheck;
//    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 200);

    if (oReturnCheck != Fapi_Status_Success)
    {
//      return FALSE;
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if (oReturnCheck != Fapi_Status_Success)
    {

//      return FALSE;
    }
    //
    // Erase the sector that is programmed in the above example
    // Erase Sector6
    //
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Flash_sector);

    //
    // Wait until FSM is done with erase sector operation
    //
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors
        //
//        Example_Error(oReturnCheck);
//        return false;

    }

    //
    // Verify that Sector6 is erased
    //
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Flash_sector,
                                     Sector16KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for error info
        //
//        Example_Error(oReturnCheck);
//        return false;

    }
    return true;
}

#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(grlFlashBOOTPGMSectorWrite, ".TI.ramfunc");
#endif
bool grlFlashBOOTPGMSectorWrite(uint32_t Flash_sector, uint8_t *Buffer, uint8_t words)
{

    Fapi_StatusType oReturnCheck;
    volatile Fapi_FlashStatusType oFlashStatus;
//    Fapi_FlashStatusWordType oFlashStatusWord;
    //
    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency(in MHz).
    // This function is required to initialize the Flash API based on System
    // frequency before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is
    // changed.
    //
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,
                                      200);

    if(oReturnCheck != Fapi_Status_Success)
    {
        //
        // Check Flash API documentation for possible errors
        //
//        return false;

//        Example_Error(oReturnCheck);
    }

    //
    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for
    // further Flash operations to be performed on the banks.
    //
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {
//        return false;

        //
        // Check Flash API documentation for possible errors
        //
//        Example_Error(oReturnCheck);
    }
    oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)Flash_sector,Buffer,
                                                words, 0, 0, Fapi_DataOnly);

    //
    // Wait until the Flash program operation is over
    //
    while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

    if(oReturnCheck != Fapi_Status_Success)
    {
//        return false;

        //
        // Check Flash API documentation for possible errors
        //
//        Example_Error(oReturnCheck);
    }

    //
    // Read FMSTAT register contents to know the status of FSM after
    // program command to see if there are any program operation related
    // errors
    //
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {
        return false;

        //
        // Check FMSTAT and debug accordingly
        //
//        FMSTAT_Fail();
    }


    return true;
}
uint8_t lPgmData[8] =  {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xAA, 0x0B};

void tcpRxdataHandler(u8_t *atcpRxBuf)
{

    u8_t lRxCmdType = (atcpRxBuf[0] & 0x0F);

    u16_t lAPILength = atcpRxBuf[1] + HEADER_BYTE_CNT;

//    uint32_t lFlashData =  0xFFFFCCDD;


    Fapi_FlashStatusWordType oFlashStatusWord;

    switch(lRxCmdType)
    {
        case RX_API_IS_SET://0x01

            isEchobackReq = true;
#if 0
            if(atcpRxBuf[2] == 0x03)//CPU2
            {
                //We cant use same buffer for transferring data to CPU1 and CPU2, So copying received data here to CPU2 Tx buffer and pusing to IPC
                memcpy(buf_tx_CM_to_CPU2, atcpRxBuf, lAPILength);

                IPC_sendCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)buf_tx_CM_to_CPU2, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU2_R, IPC_FLAG3);

            }
#endif
//            else

            //Pranay,12Sept'22, As per raja inputs inorder to avoid data loss of Set commands if any rogue Hub is used for connecting Eth cable to test PC,
            //As CM FW pushes the received API back to SW (Echo back), and if SW validate the received command and confirms that command has been sent to tester safely....
//            grlTcpDataTx(atcpRxBuf, (atcpRxBuf[1]+HEADER_BYTE_CNT) );//Sending data back to client

            if( (atcpRxBuf[2] == 0x01) && (atcpRxBuf[3] == 0x0F)) //if FRAM DATA Write
            {
                //We cant use same buffer for transferring data to CPU1 and CPU2, So copying received data here to CPU2 Tx buffer and pushing to IPC
                memcpy(buf_tx_CM_to_CPU2, atcpRxBuf, lAPILength);

                IPC_sendCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)buf_tx_CM_to_CPU2, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU2_R, IPC_FLAG3);
            }
            else//For any other Setting just pass through
            {
                IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)atcpRxBuf, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);
            }
            break;

        case RX_API_IS_PGM: //0x02

            isEchobackReq = true;

            if(atcpRxBuf[2] == Ti_FW_UPDATE)//For Ti Cores FW update
            {
                 //no. of Bytes actually present in the FW payload received will be indicated in this Bytes, So for transfering over IPC adding actual header to the received FW payload Count
                  if(atcpRxBuf[4] == 0x09)//Flash write
                  {
                      //Need to increase the APi length only for Flash writing
                     lAPILength = ((atcpRxBuf[6] << 8 )| atcpRxBuf[5] );
                     lAPILength += (FW_UPDATE_HEADER_BYTECNT + HEADER_BYTE_CNT);

                     //Pranay,01Sep'22,As per discussions with bala for handling data loss during FWupdate so to handle data loss echoeing back the received data to app for validation
//                     grlTcpDataTx(atcpRxBuf,lAPILength);//Sending back data to client
                  }
                  else
                  {
                      //Pranay,01Sep'22,As per discussions with bala for handling data loss during FWupdate so to handle data loss echoeing back the received data to app for validation
//                        grlTcpDataTx(atcpRxBuf, (atcpRxBuf[1]+HEADER_BYTE_CNT) );//Sending back data to client
                  }

                  /**Pranay,03Sept'22, Handling the echoback and msg ID sequence during FW udpates, If prev and present MSG ID is same then ignore received FW packet*/
//                  gFWupdPresentRxMsgID = ( (atcpRxBuf[0] & 0xF0) >> 4);
//
//                  if(gFWupdPrevMsgID == gFWupdPresentRxMsgID)
//                      return ;
//                  else
//                      gFWupdPrevMsgID = gFWupdPresentRxMsgID;

                  if(atcpRxBuf[3] == CPU1)//CPU1 FW update
                  {
                    IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                                     IPC_CMD_READ_MEM, (uint32_t)atcpRxBuf, lAPILength);


                    IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);
                  }

                  else if(atcpRxBuf[3] == CPU2 )//CPU2 FW update
                  {
                    //We cant use same buffer for transferring data to CPU1 and CPU2, So copying received data here to CPU2 Tx buffer and pusing to IPC
                    memcpy(buf_tx_CM_to_CPU2, atcpRxBuf, lAPILength);

                    IPC_sendCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                                       IPC_CMD_READ_MEM, (uint32_t)buf_tx_CM_to_CPU2, lAPILength);

                    IPC_waitForAck(IPC_CM_L_CPU2_R, IPC_FLAG3);

                    if(atcpRxBuf[4]==0x0A)
                    {
                       Tx_cpu1_intr(0xEE);
//                       Tx_cpu2_intr(0xAB);
                    }
                    if(atcpRxBuf[4]==0x02)
                       Tx_cpu1_intr(0xDD);

                  }
                  else if(atcpRxBuf[3] == CONNECTIVITY_MANAGER )
                  {
                      if(atcpRxBuf[4]==0x02)
                       {
                          Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);
//                          delay();

                          Example_EraseSector(Bzero_Sector13_start);
//                          Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
//                                         (uint32 *)Bzero_Sector13_start);
//                          delay();

                          grlFlashBOOTPGMSectorWrite(Bzero_Sector13_start, lPgmData, 8);
//                          delay();
//                          delay();

//                          Fapi_doBlankCheck((uint32_t *)0x0027C000, 1, &oFlashStatusWord);
//                           if((oFlashStatusWord.au32StatusWord[1] & 0xFFFFFFFF) == PGM_MODE_CD_WORD)
//                           {
//                               Tx_cpu1_intr(0xE1);
//                           }
//                           else
//                           {
//                               Tx_cpu1_intr(0xE2);
//                           }
                           Flash_releasePumpSemaphore();

//                           Tx_cpu1_intr(0xAA);

                       }
                  }
            }
            else//Any other controller FW update push directly to CPU1 and then on to Rs485
            {

                //Pranay,01Sep'22,As per discussions with bala for handling data loss during FWupdate so to handle data loss echoeing back the received data to app for validation
//                grlTcpDataTx(atcpRxBuf, (atcpRxBuf[1]+HEADER_BYTE_CNT) );//Sending data back to client

                /**Pranay,03Sept'22, Handling the echoback and msg ID sequence during FW udpates, If prev and present MSG ID is same then ignore received FW packet*/
//                gFWupdPresentRxMsgID = ( (atcpRxBuf[0] & 0xF0) >> 4);

//                if(gFWupdPrevMsgID == gFWupdPresentRxMsgID)
//                    return ;
//                else
//                    gFWupdPrevMsgID = gFWupdPresentRxMsgID;

                //Pranay,03Sept'22, For FW updates we need to replace the upper nibble of 0th Byte i.e., slot ID to 1 always else commands will be ignored
                atcpRxBuf[0] &= 0x0F;
                atcpRxBuf[0] |= 0x10;

                IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)atcpRxBuf, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);
            }
            break;

        case Rx_API_IS_GET://0x07

            isEchobackReq = false;

//            GPIO_writePin(37, 1);

            //
            // Make this flag true if data to be sent is expecting any response(i.e., get commands)
            // This is not required as of now, but going forward if we need to perform any operation with the received data without pushing to application, this flag can be used
            //
            if(atcpRxBuf[2] == 0x80)//GET FRAM DATA
            {
                gReadAPI = true;

                //We cant use same buffer for transferring data to CPU1 and CPU2, So copying received data here to CPU2 Tx buffer and pusing to IPC
                memcpy(buf_tx_CM_to_CPU2, atcpRxBuf, lAPILength);

                IPC_sendCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)buf_tx_CM_to_CPU2, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU2_R, IPC_FLAG3);
            }
            else if(atcpRxBuf[2] == 0x83)//For polling data read
            {
                gReadAPI = true;

//                gReadPollingData = true;
            }
            else if( (atcpRxBuf[2] == 0x01) && (atcpRxBuf[3] == 0x07) )//CM core FW version read
            {
                TxBuf[0] = 0x0617;
                TxBuf[1] = 0x0601;
                TxBuf[2] = 0x0701;
                TxBuf[3] = 0x0906;//FW Version 6.0
//                TxBuf[4] = 0x04;//FW Version
//                TxBuf[5] = 0x00;
//                memcpy( &TxBuf[4], glFirmwareID, 3);

                grlTcpDataTx(TxBuf, 10);

            }
            else
            {
                gReadAPI = true;

                IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                                   IPC_CMD_READ_MEM, (uint32_t)atcpRxBuf, lAPILength);

                IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);
            }
            break;

        default:

            break;
    }

}

void grlAsciitoHexConvertion(u8_t *aSrcRxBuf, uint32_t *aDestRxBuf, u16_t aPayLoadLength)
{
    u16_t i =0, j = 0;

    for(i = 0; aSrcRxBuf[i] != '\n' ; i++)
        {
            switch(aSrcRxBuf[i])
            {
                case '0':
                    aSrcRxBuf[i] = 0x00;
                    break;
                case '1':
                    aSrcRxBuf[i] = 0x01;
                    break;
                case '2':
                    aSrcRxBuf[i] = 0x02;
                    break;
                case '3':
                    aSrcRxBuf[i] = 0x03;
                    break;
                case '4':
                    aSrcRxBuf[i] = 0x04;
                    break;
                case '5':
                    aSrcRxBuf[i] = 0x05;
                    break;
                case '6':
                    aSrcRxBuf[i] = 0x06;
                    break;
                case '7':
                    aSrcRxBuf[i] = 0x07;
                    break;
                case '8':
                    aSrcRxBuf[i] = 0x08;
                    break;
                case '9':
                    aSrcRxBuf[i] = 0x09;
                    break;
                case 'A':
                    aSrcRxBuf[i] = 0x0A;
                    break;
                case 'B':
                    aSrcRxBuf[i] = 0x0B;
                    break;
                case 'C':
                    aSrcRxBuf[i] = 0x0C;
                    break;
                case 'D':
                    aSrcRxBuf[i] = 0x0D;
                    break;
                case 'E':
                    aSrcRxBuf[i] = 0x0E;
                    break;
                case 'F':
                    aSrcRxBuf[i] = 0x0F;
                    break;
                default:

                    break;
            }
            if(aSrcRxBuf[i] == ' ')
            {
                aDestRxBuf[j++] = ( (aSrcRxBuf[i-2] << 4 ) | (aSrcRxBuf[i-1]) );
            }
            else if(aSrcRxBuf[i+1] == '\n')//LAst Byte handling
                aDestRxBuf[j++] = ( (aSrcRxBuf[i-2] << 4 ) | (aSrcRxBuf[i-1]) );
        }
}

void grl_tcp_recv_data_pop(struct tcp_pcb *tpcb,
                          struct pbuf *p)
{
    char *cad_rx;
    uint16_t long_actual = 0;
    uint16_t long_UDP_complete = 0;
    uint16_t long_total = 0;
    uint8_t cnt_lee = 0;

    memset(buf_rx, 0x00, MAX_DATA_TX_SIZE);

    long_total = p->tot_len;

    cont_rx_udp++;

//    if(tcp_connc_established)
//    {
        while ((long_UDP_complete < long_total)
                   && (long_UDP_complete < 1800) || (cnt_lee == 0))
           {
               cnt_lee++;
               long_actual = p->len;

               cad_rx = p->payload;

               int i = 0;
               for (i = 0; i < long_actual; i++)
               {

                   buf_rx[0 + i + long_UDP_complete] = *cad_rx++;
               }

               long_UDP_complete = long_actual + long_UDP_complete;
               buf_rx[long_UDP_complete + 1] = '\n';
       //        buf_rx[long_UDP_complete ] = '\n';//GRL-EDIT

               if (long_UDP_complete == long_total)
               {
//                   pbuf_free(p); /* don't leak the pbuf!*/
               }
               else
               {
                   if ( p->next != NULL)
                   p = p->next;
               }

           }

        unsigned short int prtNo = 5003;
        if(tpcb->local_port == prtNo)
        {
            Rxtcb_5003 = tpcb;
        }
        else
        {
            Rxtcb = tpcb;
        }
        //
        // Pranay,11March'22
        // Handling GRL Application related activity here, mostly differentiating the
        // data either for CPU1/CPU2 and Read/Write/Pgm and further pushing to respective core via IPC
        //


        tcpRxdataHandler(buf_rx);

        return;
}
