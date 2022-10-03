//###########################################################################
//
// FILE:   enet_lwip.c
//
// TITLE:  lwIP based Ethernet Example.
//
// Example to demonstrate UDP socket (for daikin customer)
// buf_rx,buf_tx are the watch variables which can be used or updated in the
// main application based on the requirement.
//###########################################################################
// $TI Release:   $
// $Release Date:   $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.co/
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
//###########################################################################


#include <GrlPDComm.h>

#include "driverlib_cm.h"
#include "cm.h"

#pragma DATA_SECTION(buf_rx, "MSGRAM_CM_TO_CPU1")

uint16_t cont_rx_udp;

volatile uint32_t msTime=0;

uint32_t buf_rx_cm[150] ;
uint8_t CPU1RxBuf[1040];
uint8_t buf_rx[PAYLOAD];
uint8_t buf_tx[PAYLOAD];


#define MAX_DATA_TX_SIZE        1024


extern void start_application(unsigned long app_link_location){    asm(" ldr sp, [r0,#0]");    asm(" ldr pc, [r0,#4]");}

//
// Defines
//

#define TEST_PASS          0x5555
#define TEST_FAIL          0xAAAA

//*****************************************************************************
//
//! \addtogroup master_example_list
//! <h1>Ethernet with lwIP (enet_lwip)</h1>
//!
//! This example application demonstrates the operation of the F2838x
//! microcontroller Ethernet controller using the lwIP TCP/IP Stack. Once
//! programmed, the device sits endlessly waiting for ICMP ping requests. It
//! has a static IP address. To ping the device, the sender has to be in the
//! same network. The stack also supports ARP.
//!
//! For additional details on lwIP, refer to the lwIP web page at:
//! http://savannah.nongnu.org/projects/lwip/
//
//*****************************************************************************

// These are defined by the linker (see device linker command file)


#define DEVICE_FLASH_WAITSTATES 2

//*****************************************************************************
//
// Driver specific initialization code and macro.
//
//*****************************************************************************





uint32_t releaseTxCount = 0;
uint32_t genericISRCustomcount = 0;
uint32_t genericISRCustomRBUcount = 0;
uint32_t genericISRCustomROVcount = 0;
uint32_t genericISRCustomRIcount = 0;

uint32_t systickPeriodValue = 125000; //15000000;



uint32_t sendPacketFailedCount = 0;
uint32_t entryAddress;



// Pranay, 11March,22
// IPC ISR for communication between CM and CPU1,
// THis will get executed when CM receives data from CPU1
//

__interrupt void IPC_CPU1_CM_ISR0()
{
    uint32_t command, addr, data;

    IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);

    memset(CPU1RxBuf, 0xFF, 1040);


    uint32_t i =0,index = 0,j=0;

    uint32_t lVar[600] = {0};
//    uint16_t lTxBuf[4] = {0};

    //
    //Pranay,11March'22
    //Data will be received in 32bit format,
    // i.e., If we're sending AA,BB,CC,DD from Remote core, it will be received as
    // 00BB00AA, 00DD00CC,
    //So receiving data here in a 32 bit variable
    //
    for (i = 0; i < ((data/2)+1); i++)
    {
      lVar[0 + i] = (*((uint32_t *)addr + i) );
    }
    //
    //Pranay,11March'22
    // Here in Ti we only have a min of 16bit addressing, there is not 8 bit addressing even though you decalre as uint8, it will create a 16 bit variable
    // Once the whole data is received in to lVar buffer further go for decoding
    // First we're decoding Each byte data from lVar and storing into lTxBuf
    // then further forming a pair of Byte(1 word) with 2 bytes LSB first and MSB next
    // we have other alternate way of writing this logic aswell, in whichwe can track only 0th Byte and 2nd Byte directly into CPU2RxBuf
    //
    // in looping condition,
    // +1 for rounding off ending condition,
    // * 2 when zeroes included as MSB ByteCount increases by 2
    //
    index = 0;

    j = (data % 4) ;

    if(j > 0)
    {
        data = ((data/4) + 1);
    }
    else
        data = ((data/4));

    for (i = 0; i < (data*2); i++)//+1 for rounding off, *2 when zeroes included as MSB ByteCount increases
    {

//        lTxBuf[0] = lVar[i] & 0xFF;
//        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
//        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
//        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        CPU1RxBuf[index++] = lVar[i] & 0xFF;
//        CPU1RxBuf[index++] = (lVar[i] & 0xFF00) >> 8;
        CPU1RxBuf[index++] = (lVar[i] & 0xFF0000) >> 16;
//        CPU1RxBuf[index++] = (lVar[i] & 0xFF000000) >> 24;

        //Here instead of forming a word with neighbouring bytes, we're forming a word with bytes in which data is actually present
//        CPU1RxBuf[index++] = (lTxBuf[0] | (lTxBuf[2] << 8)) ;

    }

    cm_handler(CPU1RxBuf);

}

void initFlashSectors(void){
    Fapi_StatusType  oReturnCheck;

   //
   // Initialize the Flash API by providing the Flash register base address
   // and operating frequency(in MHz).
   // This function is required to initialize the Flash API based on CM
   // operating frequency before any other Flash API operation can be performed.
   // This function must also be called whenever System frequency or RWAIT is
   // changed.
   //
   oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS,
                                     CM_CLK_FREQ/1000000U);

   if(oReturnCheck != Fapi_Status_Success)
   {
       //
       // Check Flash API documentation for possible errors
       //
//       exampleError(oReturnCheck);
   }

   //
   // Initialize the Flash banks and FMC for erase and program operations.
   // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for
   // further Flash operations to be performed on the banks.
   //
   oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

   if(oReturnCheck != Fapi_Status_Success)
   {
       //
       // Check Flash API documentation for possible errors
       //
//       exampleError(oReturnCheck);
   }

}


//void exit(uint32_t address)
//{
//    //
//    // Jump to entry address
//    //
//    __asm(" bx r0");
//}

void enter(uint32_t address)
{
    __asm(" bx r0");

}

#define JumpToApp (void (*)(void))0x23d71c

int
main(void)
{

    Fapi_FlashStatusWordType oFlashStatusWord;
//    int i=0;


    //  ////////////////////////////////////////
    // Initializing the CM. Loading the required functions to SRAM.
    //
    CM_init();

    //
    // Enable processor interrupts.
    //
    Interrupt_enableInProcessor();

    //CM- CPU1
    IPC_clearFlagLtoR(IPC_CM_L_CPU1_R, IPC_FLAG_ALL);
    IPC_registerInterrupt(IPC_CM_L_CPU1_R, IPC_INT0, IPC_CPU1_CM_ISR0);


    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 2);
    initFlashSectors();

    Flash_disableECC(FLASH0ECC_BASE);

    Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);


    while (1)
    {
//
        Fapi_doBlankCheck((uint32_t *)0x0027C000, 1, &oFlashStatusWord);

          if((oFlashStatusWord.au32StatusWord[1] & 0x0000FFFF) == BOOT_MODE_CD_WORD)
          {

              IPC_sync(IPC_CM_L_CPU1_R, IPC_FLAG28);
              IPC_sync(IPC_CM_L_CPU2_R, IPC_FLAG29);

              Flash_releasePumpSemaphore();

              entryAddress = 0x210001;
              ( (void (*)(void))entryAddress)();

          }
          else
          {
              ;
          }

    }
}


