//#############################################################################
//
// FILE:   flashapi_ex1_programming.c
//
// TITLE:  Flash programming example
//
//! \addtogroup driver_example_list
//! <h1> Flash Programming with AutoECC, DataAndECC, DataOnly and EccOnly </h1>
//!
//! This example demonstrates how to program Flash using API's following options
//! 1. AutoEcc generation
//! 2. DataOnly and EccOnly
//! 3. DataAndECC
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
//!
//
//#############################################################################
// $TI Release: F2838x Support Library v3.04.00.00 $
// $Release Date: Fri Feb 12 19:08:49 IST 2021 $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include <CPU2_SSBL_Main.h>
#include "driverlib.h"
#include "device.h"
#include "string.h"
#include "f2838x_device.h"
#include "f2838x_flash.h"
#include "FRAM.h"
//
// Include Flash API include file
//
#include "F021_F2838x_C28x.h"

void Example_Error(Fapi_StatusType status);
void Example_Done(void);
bool Flash_data_program_8(uint32 Flash_sector, uint16 *Buffer, uint16 words);
void FMSTAT_Fail(void);
void ECC_Fail(void);
bool Example_EraseSector(uint32_t);



//
// Length (in 16-bit words) of data buffer used for program
//
#define DVK
#define ETHERNET
#define  WORDS_IN_FLASH_BUFFER    0x100
#define IPC_CMD_READ_MEM   0x1001
#define DATASIZE        286
#define APP_DATA         0x08
#define BOOT_MODE        0x01
#define WRITE_FLASH      0x09
#define ERASE_SECTOR     0x0A
#define WRITE_STATUS     0x07

#pragma DATA_SECTION(grlRxbuf, "MSGRAM_CPU_TO_CM")
#pragma DATA_SECTION(gVar, "IPC_RX_DECODE_BUF");
#pragma DATA_SECTION(CPU2DataRxbuf, "MSGRAM_CPU2_TO_CPU1")

uint8_t grlRxbuf[DATASIZE] = {0};

uint32_t gVar[DATASIZE] = {0};

uint8_t write_status[8]={0,0,0,0,0,0,0,0};

uint32_t sys_id;
uint32_t mfd_year;
uint8_t mfd_month;
//uint8_t CMDataRxbuf[286];
uint8_t is_program;

uint32_t entry_point;
struct HEADER
{
    int16_t BlockSize;
    uint32_t DestAddr;
} BlockHeader;
static uint16_t mp = 0;
uint8_t iteration_cnt=0;

#define FLASH_ADDRESSES_CNT         12

uint8_t CPU2DataRxbuf[286];

void Example_ProgramUsingAutoECC(void);

/*flash sector addresses*/
uint32 flash_address[FLASH_ADDRESSES_CNT] = { 0x86000, 0x88000,
                             0x90000, 0x98000, 0xA0000, 0xA8000, 0xB0000,
                             0xB8000, 0xBA000, 0xBC000 };

#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(grlFlashBOOTPGMSectorWrite, ".TI.ramfunc");
#endif
bool grlFlashBOOTPGMSectorWrite(uint32_t Flash_sector, uint16_t *Buffer, uint16_t words)
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
        return false;

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
        return false;

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
        return false;

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
/**
 * @brief: this function it will handle the given hex data as per boot structure of TI controller
 *         it will validate key value and then write hex data as given block size and block address.
 * @author <19 FEB 2021 ; Basweshwar >
 * @return after successful completion return TRUE otherwise FALSE.
 */

bool copy_data(uint8_t * aBuffer)
{
    bool state;
    uint32 k,j;
    int16_t i;
  static  uint16_t size, D_address;
    uint32_t offset;

    if (mp == 0)
    {
        /* key verification */
        if ((aBuffer[0]) != 0x08AA)
        {
         write_status[3]=1;
         write_status[6]=iteration_cnt;
        }
        /*getting entry point address*/
        entry_point = (aBuffer[9]);
        entry_point = entry_point << 16 | aBuffer[10];
        BlockHeader.BlockSize = aBuffer[11];
        BlockHeader.DestAddr = aBuffer[12];
        BlockHeader.DestAddr = (BlockHeader.DestAddr << 16) | aBuffer[13];
        //k+=14;
        for (i = 14; i < 128;)
        {
            if (BlockHeader.BlockSize == 0)
            {
                BlockHeader.BlockSize = aBuffer[i++];
                BlockHeader.DestAddr = aBuffer[i++];
                BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                        | aBuffer[i++];

            }

            if (BlockHeader.BlockSize < 8)
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i,
                                             BlockHeader.BlockSize);
//                Example_CallFlashAPI();
                if (state != TRUE)
                {
                    write_status[1]=1;

                    // return FALSE;
                }
                i += BlockHeader.BlockSize;
                BlockHeader.DestAddr += BlockHeader.BlockSize;
                BlockHeader.BlockSize -= BlockHeader.BlockSize;
                //BlockHeader.DestAddr += BlockHeader.BlockSize;
            }
            if ((128 - i) < 8)
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i, (128 - i));
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                              aBuffer + i, (128 - i));
                if (state != TRUE)
                {
                    write_status[1]=1;

                    //Example_Done();
                }

                BlockHeader.DestAddr += 128 - i;
                BlockHeader.BlockSize -= 128 - i;
                i += 128 - i;
                //BlockHeader.DestAddr += 128 - i;
                break;

            }
            if (BlockHeader.BlockSize != 0)
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             (aBuffer + i), 8);
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                                             (aBuffer + i), 8);
                if (state != TRUE)
                {
                    write_status[1]=1;

                    //Example_Done();
                }
                i += 8;
                BlockHeader.BlockSize -= 8;
                BlockHeader.DestAddr += 8;
            }
        }
        mp = 1;
        // point = 0;
        //    bpoint = 0;
//        memset(I2C_RX_Buffer, 0, 16);
        memset(aBuffer, 0, DATASIZE);
    }
    else
    {

        for (i = 0; i < 128;)
        {
            offset = BlockHeader.DestAddr & 0x0000000f;   //address offset check

            if (BlockHeader.BlockSize == 0)
            {
                if (BlockHeader.DestAddr == 0xfffffffa)
                {
                    BlockHeader.BlockSize = size;
                    BlockHeader.DestAddr = D_address;
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                            | aBuffer[i++];

                }
                else if (BlockHeader.DestAddr == 0xfffffffb)
                {
                    BlockHeader.BlockSize = size;
                    BlockHeader.DestAddr = aBuffer[i++];
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                            | aBuffer[i++];

                }
                else if (i > 125)
                {
                    if (i == 126)
                    {
                        size = aBuffer[i++];
                        if (size == 0)
                        {
                            // ReleaseFlashPump();
                            // Flash_sector_erase(boot_address);
                            //  state = bootmode_update(BOOT_MODE);
                            if (state != TRUE)
                            {
                                //  Example_Done();
                            }
                            //Soft_reset();
                        }
                        D_address = aBuffer[i++];
                        BlockHeader.DestAddr = 0xfffffffa;

                    }
                    if (i == 127)
                    {
                        size = aBuffer[i++];
                        if (size == 0)
                        {                          //ReleaseFlashPump();
                                                   //Flash_sector_erase(boot_address);
                                                   //  state = bootmode_update(BOOT_MODE);
                            if (state != TRUE)
                            {
                                // Example_Done();
                            }
                            // Soft_reset();
                        }
                        BlockHeader.DestAddr = 0xfffffffb;
                    }

                }
                else
                {
                    BlockHeader.BlockSize = aBuffer[i++];
                    BlockHeader.DestAddr = aBuffer[i++];
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                            | aBuffer[i++];
                    if (BlockHeader.BlockSize == 0)
                    {                          //ReleaseFlashPump();
                                               //Flash_sector_erase(boot_address);
                                               //   state = bootmode_update(BOOT_MODE);
                        if (state != TRUE)
                        {

                            // Example_Done();
                        }
                        //   Soft_reset();
                    }
                }

                if( (BlockHeader.BlockSize == 0xffff) && (BlockHeader.DestAddr= 0xffffffff))
                         {
                             BlockHeader.BlockSize = 0x0;
                             BlockHeader.DestAddr = 0x0;
                             break;
                         }


            }
            else if ((offset % 8) != 0)        //(offset != 8 || offset != 0)
            {
                if (offset > 8)
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 (aBuffer + i),
                                                 16 - offset);
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                                   (aBuffer + i),
                                                                   16 - offset);
                    if (state != TRUE)
                    {
                        write_status[1]=1;

                        //Example_Done();
                    }
                    i += (16 - offset);
                    BlockHeader.BlockSize -= 16 - offset;
                    BlockHeader.DestAddr += 16 - offset;
                }
                else if (offset < 8)
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 (aBuffer + i),
                                                 8 - offset);
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                                     (aBuffer + i),
                                                                     8 - offset);
                    if (state != TRUE)
                    {
                        write_status[1]=1;

                        //Example_Done();
                    }
                    i += (8 - offset);
                    BlockHeader.BlockSize -= 8 - offset;
                    BlockHeader.DestAddr += 8 - offset;

                }

            }
            else if (BlockHeader.BlockSize < 8)
            {
                if (((128 - i) - BlockHeader.BlockSize) < 0)
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 aBuffer + i, 128 - i);
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                                     aBuffer + i, 128 - i);
                    if (state != TRUE)
                    {
                        write_status[1]=1;

                        // Example_Done();
                        //return FALSE;
                    }
                    //  i += 128-i;
                    BlockHeader.DestAddr += (128 - i);
                    BlockHeader.BlockSize -= (128 - i);
                    i += (128 - i);
                }
                else
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 aBuffer + i,
                                                 BlockHeader.BlockSize);
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                                     aBuffer + i,
                                                                     BlockHeader.BlockSize);
                    if (state != TRUE)
                    {
                        write_status[1]=1;

                        // Example_Done();
                        //return FALSE;
                    }
                    i += BlockHeader.BlockSize;
                    BlockHeader.DestAddr += BlockHeader.BlockSize;
                    BlockHeader.BlockSize -= BlockHeader.BlockSize;
                }

            }
            else if (((128 - i) != 0) && ((128 - i) < 8))
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i, (128 - i));
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                                            aBuffer + i, (128 - i));
                if (state != TRUE)
                {
                    write_status[1]=1;

                    //Example_Done();
                }

                BlockHeader.DestAddr += (128 - i);
                BlockHeader.BlockSize -= (128 - i);
                i += (128 - i);
                break;

            }
            else
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             (aBuffer + i), 8);
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                                       (aBuffer + i), 8);
                if (state != TRUE)
                {
                    write_status[1]=1;

                    //Example_Done();
                }
                i += 8;
                BlockHeader.BlockSize -= 8;
                BlockHeader.DestAddr += 8;

            }

        }
        // point = 0;
        //fpoint = 0;
//        memset(I2C_RX_Buffer, 0, 16);
        memset(aBuffer, 0, DATASIZE);
    }
//    GPIO_writePin(166, 1);


    return TRUE;
}

void TxCPU1IPC(uint16_t aCmd)
{
    CPU2DataRxbuf[0] = aCmd;
     IPC_sendCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                     IPC_CMD_READ_MEM, (uint32_t)CPU2DataRxbuf, 2);

     IPC_waitForAck(IPC_CPU2_L_CPU1_R, IPC_FLAG2);
}

void cm_ipc_intr_write_status(uint8_t status)
{
    memset(grlRxbuf,0, DATASIZE);
    grlRxbuf[0]= 0x0d;
    grlRxbuf[1]=status;
    grlRxbuf[2]= write_status[4];
    grlRxbuf[3]=write_status[5];
    grlRxbuf[4]=write_status[6];

    IPC_sendCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                    IPC_CMD_READ_MEM, (uint32_t)grlRxbuf,2);

    IPC_waitForAck(IPC_CPU2_L_CM_R, IPC_FLAG3);

}


bool DataRxHandler(uint8_t * aBuffer)
{

    bool lRetStatus = false;
    uint8_t lrxCmd = aBuffer[4];
    uint8_t app_status=0;
//    uint8_t lFwController = aRxBuffer[2];
    uint32_t i,j,state=0;        //mode;
    uint32_t sector_add = 0;
    uint8_t lBuffer[286] = {0};
    uint16_t lFlashData =  BOOT_MODE_CD_WORD;
    uint16_t lFWPayloadLength = 0;
    uint16_t Word_Point = 0;



    switch(lrxCmd)
    {
    case BOOT_MODE://0x01


//        Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);

//        Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
//                       (uint32 *)Grl_Boot_Pgm_Sector);
        Example_EraseSector(Bzero_Sector13_start);

        grlFlashBOOTPGMSectorWrite(Bzero_Sector13_start, &lFlashData, 1);

//        Flash_releasePumpSemaphore();

        break;
    case WRITE_FLASH://0x09
        lFWPayloadLength = ((aBuffer[6] << 8 )| aBuffer[5] );
//        Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);

        iteration_cnt++;
        for (i = 7, Word_Point = 0; i < lFWPayloadLength+7; i += 2)
        {
            lBuffer[Word_Point++] = (aBuffer[i] | (aBuffer[i + 1] << 8));

        }

        copy_data(lBuffer);



//        Flash_releasePumpSemaphore();

        break;

    case ERASE_SECTOR://0x0A

//        Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);

        for( i =0 ; i < FLASH_ADDRESSES_CNT; i++)
        {
           sector_add = flash_address[i];

           state = Example_EraseSector(sector_add);
           DEVICE_DELAY_US(60000);

           if (state == false)
           {
        //            Example_Done();
           }
        }

        break;
    case WRITE_STATUS:
        if((write_status[1]!=1) ||(write_status[2]!=1) ||(write_status[3]!=1))
               {
                   app_status = WRITE_SUCESS;
               }
               else
               {
                  if(write_status[1]==1)
                      app_status |= ADDRESS_ERROR;
                  if(write_status[2]==1)
                      app_status |= VERIFY_ERROR;
                  if(write_status[3]==1)
                      app_status |= KEY_ERROR;
               }
               cm_ipc_intr_write_status(app_status);
               memset(write_status, 0, 8);
               iteration_cnt=0;

    }

    return lRetStatus;

}

void is_TiFWUpdate(uint8_t* aRxBuffer)
{
    uint8_t lCpuSelection = aRxBuffer[3];

    switch(lCpuSelection)
    {
    case TI_CM://0x00

        break;
    case TI_CPU1_CONTROLCARD://0x01

//            Ti_CPU1BootModeSelection(aRxBuffer);
        DataRxHandler(aRxBuffer);

        break;

    case TI_CPU2_PPS://0x02
        DataRxHandler(aRxBuffer);

        break;

    case 0x03:
        CPU2DataRxbuf[0]=0x01;
        CPU2DataRxbuf[1]=0;
        CPU2DataRxbuf[2]=0x08;
        CPU2DataRxbuf[3]=0xdd;
        IPC_sendCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                        IPC_CMD_READ_MEM, (uint32_t)CPU2DataRxbuf, CPU2DataRxbuf[1]+4);

        IPC_waitForAck(IPC_CPU2_L_CPU1_R, IPC_FLAG2);
    }
}

void isAPI_FWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lFwController = aRxBuffer[2];

    switch(lFwController)
    {
        case 0x06 ://Ti
                is_TiFWUpdate(aRxBuffer);
            break;
    }
}


__interrupt void CM_CPU2_IPC_ISR1()
{
    uint32_t command, addr, data;

       //
       // Read the command
       //
       IPC_readCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                       &command, &addr, &data);

       IPC_ackFlagRtoL(IPC_CPU2_L_CM_R, IPC_FLAG3);

       Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

       memset(CPU2DataRxbuf, 0x00, 128);

       int i =0,index = 0;

       for (i = 0; i < (data); i++)
       {
           gVar[0+i] = (*((uint32_t *)addr + i) );
   //        buf_rx1[0 + i ] = (*((uint32_t *)addr + i) );
       }
       for (i = 0; i < (data/4)+1; i++)
       {
           CPU2DataRxbuf[index++] = (gVar[i] & 0xFF);
           CPU2DataRxbuf[index++] = (gVar[i] & 0xFF00) >> 8;
           CPU2DataRxbuf[index++] = (gVar[i] & 0xFF0000) >> 16;
           CPU2DataRxbuf[index++] = (gVar[i] & 0xFF000000) >> 24;
       }

//       if(CPU2DataRxbuf[0]& 0xff == 0xAB)
//       {
//           Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);
//       }

    isAPI_FWUpdate(CPU2DataRxbuf);

}


//
// IPC ISR for Flag 0.
// C28x core sends data without message queue using Flag 0
//
__interrupt void IPC_ISR0()
{
    uint32_t lVar[64];
//    int i;
    uint32_t command, addr, data;
//    uint16_t aVal = 0;
//    bool status = false;
    uint16_t lTxBuf[4] = {0};

    //
    // Read the command
    //
    IPC_readCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CPU2_L_CPU1_R, IPC_FLAG2);


    int i =0,index = 0;

    for (i = 0; i < (data); i++)
    {
        lVar[0+i] = (*((uint32_t *)addr + i) );
//        buf_rx1[0 + i ] = (*((uint32_t *)addr + i) );
    }
    for (i = 0; i < ((data/4) + 1 * 2); i++)
    {

        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        CPU2DataRxbuf[index++] = (lTxBuf[0] | (lTxBuf[1] << 8)) ;
        CPU2DataRxbuf[index++] = (lTxBuf[2] | (lTxBuf[3] << 8)) ;
    }

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    IPC_sendCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                    IPC_CMD_READ_MEM, (uint32_t)CPU2DataRxbuf, CPU2DataRxbuf[1]+2);

    IPC_waitForAck(IPC_CPU2_L_CPU1_R, IPC_FLAG2);




}

void CM_EthernetIOMuxHandler()
{
#ifdef DVK
#ifdef ETHERNET
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

#endif

#else/**DVK*/
#ifdef ETHERNET
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
//    GPIO_setPinConfig(GPIO_105_ENET_MDIO_CLK);
//    GPIO_setPinConfig(GPIO_106_ENET_MDIO_DATA);

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
#endif/**DVK*/
}

void i2cB_init()
{
    Interrupt_register(INT_I2CB, &I2C_FRAM_ISR);
    Init_I2C_Master();
    Interrupt_enable(INT_I2CB);

}

void TxSystemDetailstoCM()
{

    memset(grlRxbuf, 0x00, 12);

    grlRxbuf[0] = 0xC1;
    grlRxbuf[1] = 0xC2;
    grlRxbuf[2] = 0x0A;//No of Bytes payload length

    grlRxbuf[3] = (sys_id & 0xFF);
    grlRxbuf[4] = (sys_id >> 16);

    grlRxbuf[5] = (mfd_month& 0xFF);

    grlRxbuf[6] = (mfd_year & 0xFF);
    grlRxbuf[7] = (mfd_year >> 16);


    grlRxbuf[8] = 0xFD;//Checksum Keyword

    IPC_sendCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                    IPC_CMD_READ_MEM, (uint32_t)grlRxbuf, grlRxbuf[2]);

    IPC_waitForAck(IPC_CPU2_L_CM_R, IPC_FLAG3);

}
//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    // Copy the Flash initialization code from Flash to RAM
    // Copy the Flash API from Flash to RAM
    // Configure Flash wait-states, fall back power mode, performance features
    // and ECC
    //

    uint32 i,j = 0;
    Device_init();

    //
    // Initialize GPIO
    //
//    Device_initGPIO();

    i2cB_init();
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enable IPC interrupts
    //
    IPC_registerInterrupt(IPC_CPU2_L_CM_R, IPC_INT3, CM_CPU2_IPC_ISR1);
    //
    // Clear any IPC flags if set already
    //p
    IPC_clearFlagLtoR(IPC_CPU2_L_CM_R, IPC_FLAG_ALL);

    //IPC sync for Sync between cores
    IPC_sync(IPC_CPU2_L_CM_R, IPC_FLAG29);

    //looping to ensure CM boot first and waiting for IPC data of System ID details to come from CPU2
    while(j++ < 50)
        for(i = 0; i < 5000; i++)
            ;
//
// Enable IPC interrupts
//
//       IPC_registerInterrupt(IPC_CPU2_L_CPU1_R, IPC_INT2, IPC_ISR0);
//       //
//       // Clear any IPC flags if set already
//       //
//       IPC_clearFlagLtoR(IPC_CPU2_L_CPU1_R, IPC_FLAG_ALL);


    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);

    Fapi_setActiveFlashBank(Fapi_FlashBank0);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    Flash_disableECC(FLASH0ECC_BASE);

    FRAM_I2C_read(2, 0x08, &sys_id);
    FRAM_I2C_read(1, 77, &mfd_month);
    FRAM_I2C_read(2, 78, &mfd_year);

    TxSystemDetailstoCM();

    //IPC Sync for IP address syncing, Making CM wait till it receive System details
    IPC_sync(IPC_CPU2_L_CM_R, IPC_FLAG30);

    Fapi_FlashStatusWordType oFlashStatusWord;

    Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);

    while(1)
    {

        Fapi_doBlankCheck((uint32_t *)Bzero_Sector13_start,
                              1,
                              &oFlashStatusWord);

        if((oFlashStatusWord.au32StatusWord[1] & 0x0000FFFF) == BOOT_MODE_CD_WORD)
        {
            Flash_releasePumpSemaphore();
            __asm(" LB 0x86000 ");     /*jumping to entry point*/
        }
        else
        {
            ;
        }
    }
}


//*****************************************************************************
//  Example_CallFlashAPI
//
//  This function will interface to the flash API.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Flash_data_program_8, ".TI.ramfunc");
#endif
bool Flash_data_program_8(uint32 Flash_sector, uint16 *Buffer, uint16 words)
{
//    uint16 i = 0;
    uint32 u32Index = 0;
    //  uint16 i = 0;
    uint32 *Buffer32 = (uint32 *) Buffer;
    Fapi_StatusType oReturnCheck;
    volatile Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;
    //
    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency(in MHz).
    // This function is required to initialize the Flash API based on System
    // frequency before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is
    // changed.
    //
    EALLOW;
    //Error handling here, if address for flash writing is not in required ranges, than dont handle respective APi
    if((Flash_sector >= 0x086000) && (Flash_sector <= 0x0BE000) )
    {

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
//        return false;

        //
        // Check FMSTAT and debug accordingly
        //
//        FMSTAT_Fail();
    }


    //
    // Verify the values programmed.  The Program step itself does a verify
    // as it goes.  This verify is a 2nd verification that can be done.
    //
//    oReturnCheck = Fapi_doVerify((uint32 *) Flash_sector, words / 2,
//                                     Buffer, &oFlashStatusWord);
//
//    if (oReturnCheck != Fapi_Status_Success)
//    {
//        return FALSE;
//        //   Example_Error(oReturnCheck);
//    }
    oReturnCheck=Fapi_doVerifyBy16bits((uint32 *) Flash_sector,  words, Buffer, &oFlashStatusWord);
    if (oReturnCheck != Fapi_Status_Success)
       {
        write_status[2]=1;
        write_status[5]=iteration_cnt;
         return FALSE;
       }
    return true;
    }
    write_status[4]=iteration_cnt;
    return false;
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

    EALLOW;
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 200);

    if (oReturnCheck != Fapi_Status_Success)
    {
      return FALSE;
    }

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if (oReturnCheck != Fapi_Status_Success)
    {

      return FALSE;
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
        return false;

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
        return false;

    }
    return true;
}
#if 0
//*****************************************************************************
//  Example_ProgramUsingAutoECC
//
//  Example function to Program data in Flash using "AutoEccGeneration" option.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_ProgramUsingAutoECC, ".TI.ramfunc");
#endif
void Example_ProgramUsingAutoECC(void)
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // A data buffer of max 8 16-bit words can be supplied to the program
    // function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 4 or 8.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration option is used, Flash API calculates ECC for the
    // given 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify
    // reasons.
    //
    // Monitor ECC address for Sector6 while programming with AutoEcc mode.
    //
    // In this example, 0x100 bytes are programmed in Flash Sector6
    // along with auto-generated ECC.
    //
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 200);

    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);


    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER));
       i+= 8, u32Index+= 8)
    {
        oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index,Buffer+i,
                                               8, 0, 0, Fapi_DataOnly);

        //
        // Wait until the Flash program operation is over
        //
        while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
//            Example_Error(oReturnCheck);
        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {
            //
            // Check FMSTAT and debug accordingly
            //
//            FMSTAT_Fail();
        }

        //
        // Verify the programmed values.  Check for any ECC errors.
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32+(i/2),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {
            //
            // Check Flash API documentation for possible errors
            //
//            Example_Error(oReturnCheck);
        }
    }
}
#endif
//******************************************************************************
// For this example, just stop here if an API error is found
//******************************************************************************
void Example_Error(Fapi_StatusType status)
{
    //
    //  Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}

//******************************************************************************
//  For this example, once we are done just stop here
//******************************************************************************
void Example_Done(void)
{
    __asm("    ESTOP0");
}

//******************************************************************************
// For this example, just stop here if FMSTAT fail occurs
//******************************************************************************
void FMSTAT_Fail(void)
{
    __asm("    ESTOP0");
}

//******************************************************************************
// For this example, just stop here if ECC fail occurs
//******************************************************************************
void ECC_Fail(void)
{
    __asm("    ESTOP0");
}

//
// End of File
//
