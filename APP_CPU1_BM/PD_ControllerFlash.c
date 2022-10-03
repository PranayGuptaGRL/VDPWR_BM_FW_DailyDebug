/*
 * PD_ControllerFlash.c
 *
 *  Created on: 17-Feb-2022
 *      Author: pranay
 */
#include <PD_ControllerFlash.h>
#include "F021_F2838x_C28x.h"

#define WORDS_IN_FLASH_BUFFER   0x100
#pragma  DATA_SECTION(Buffer,"DataBufferSection");
uint16_t   Buffer[WORDS_IN_FLASH_BUFFER];
uint32_t   *Buffer32 = (uint32_t *)Buffer;

void grlFlash_InitModule()
{
    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);

    Fapi_setActiveFlashBank(Fapi_FlashBank0);
}
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


    for(i=0, u32Index = 0x080004;
       (u32Index < (0x080004 + WORDS_IN_FLASH_BUFFER));
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

        }

        //
        // Read FMSTAT register contents to know the status of FSM after
        // program command to see if there are any program operation related
        // errors
        //
        oFlashStatus = Fapi_getFsmStatus();
        if(oFlashStatus != 0)
        {

        }

        //
        // Verify the programmed values.  Check for any ECC errors.
        //
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32+(i/2),
                                     &oFlashStatusWord);

        if(oReturnCheck != Fapi_Status_Success)
        {

        }
    }
}

void grlCPU1PGMmodeSetHandler(uint8_t * aRxBuffer)
{
    uint32_t flash_address = BOOTPGM_FLASH_SECTOR_ADDR;
    uint16_t lFlashData =  PGM_MODE_CD_WORD;


    Flash_claimPumpSemaphore(FLASH_CPU1_WRAPPER);

    Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)flash_address);

    grlFlashBOOTPGMSectorWrite(flash_address, &lFlashData, 1);


    Flash_releasePumpSemaphore();

}

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
    }

    return true;
}
