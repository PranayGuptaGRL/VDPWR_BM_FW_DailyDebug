/*
 * Grl_PD_Comm.c
 *
 *  Created on: 01-Feb-2022
 *      Author: pranay
 */

#include <GrlPDComm.h>

#define SYSCTL_STATUS_CMSYSRESETREQ 0x0004U
#define DATASIZE        286

struct HEADER
{
    uint16_t BlockSize;
    uint32_t DestAddr;
} BlockHeader;
static uint16_t mp = 0;

uint8_t iteration_cnt=0;
uint32_t data_size=0;

uint8_t write_status[8]={0,0,0,0,0,0,0,0};
//uint32_t entry_point;

/*flash sector addresses*/
uint32 flash_address[FLASH_ADDRESSES_CNT] = {  0x00210000,0x00220000,
          0x00230000, 0x00240000, 0x00250000, 0x00260000, 0x00270000,0x00274000, 0x00278000 };



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
bool Flash_data_program_8(uint32 Flash_sector, uint8 *Buffer, uint8 words)
{
//    uint16 i = 0;
    uint32 u32Index = 0;
    //  uint16 i = 0;
    uint32 *Buffer32 = (uint32 *) Buffer;
    Fapi_StatusType oReturnCheck;
    volatile Fapi_FlashStatusType oFlashStatus;
    Fapi_FlashStatusWordType oFlashStatusWord;


    if((Flash_sector >= 0x00210000) && (Flash_sector <= 0x00278000) )
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
//    oReturnCheck = Fapi_doVerify((uint32 *) Flash_sector, words / 4,
//                                     Buffer, &oFlashStatusWord);
//
//    if (oReturnCheck != Fapi_Status_Success)
//    {
//        return FALSE;
//        //   Example_Error(oReturnCheck);
//    }
    oReturnCheck = Fapi_doVerifyByByte((uint32_t *)Flash_sector, words, Buffer, &oFlashStatusWord);
        if (oReturnCheck != Fapi_Status_Success)
        {
            write_status[2]=1;
            write_status[5] =iteration_cnt;
                return false;
        }

    return true;
    }
    write_status[4] = iteration_cnt;
    return false;

}


//void CM_reset()
//{
//    uint32_t *lRegVal;
//
//    volatile uint32_t lAircr_reg = AIRCR_REG;
//
//
//    lRegVal = &lAircr_reg;
//
//    *lRegVal |= SYSCTL_STATUS_CMSYSRESETREQ;
//
//}



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
//    return true;
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
                                      CM_CLK_FREQ/1000000U);

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


    return true;
}



#ifdef 128_byte
/**
 * @brief: this function it will handle the given hex data as per boot structure of TI controller
 *         it will validate key value and then write hex data as given block size and block address.
 * @author <19 FEB 2021 ; Basweshwar >
 * @return after successful completion return TRUE otherwise FALSE.
 */
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(copy_data, ".TI.ramfunc");
#endif
bool copy_data(uint8_t * aBuffer)
{
    bool state;
    uint32 k,j;
    uint16_t i=0;
  static  uint16_t size, D_address;
    uint32_t offset;

    if (mp == 0)
    {
        /* key verification */
        if ((aBuffer[0] << 8 |aBuffer[1]) != 0x08AA)
        {
//            Example_Done();
            // return FALSE;
        }
        /*getting entry point address*/
        entry_point = (aBuffer[18] << 8 )|aBuffer[19];
        entry_point = entry_point << 16 | ((aBuffer[20]<<8) |aBuffer [21]);
        BlockHeader.BlockSize = aBuffer[22] << 8 |aBuffer [23];
        BlockHeader.DestAddr =  (aBuffer[24] << 8 )|aBuffer[25];
        BlockHeader.DestAddr = (BlockHeader.DestAddr << 16) | ((aBuffer[26]<<8) |aBuffer [27]);
        //k+=14;
        for (i = 28; i < 128;)
        {
            if (BlockHeader.BlockSize == 0)
            {
                BlockHeader.BlockSize =(aBuffer[i++] << 8 )|aBuffer[i++];
                BlockHeader.DestAddr = (aBuffer[i++] << 8 )|aBuffer[i++];
                BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                        |((aBuffer[i++]<<8 )|aBuffer [i++]);

            }

            if (BlockHeader.BlockSize < 8)
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i,
                                            ( BlockHeader.BlockSize ));
//                Example_CallFlashAPI();
                if (state != TRUE)
                {
                    write_status =1;
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
                if (state != TRUE)
                {
                    write_status =1;
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
                if (state != TRUE)
                {write_status =1;
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
        memset(aBuffer, 0, 128);
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
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)  |((aBuffer[i++] << 8 )|aBuffer [i++]);;

                }
                else if (BlockHeader.DestAddr == 0xfffffffb)
                {
                    BlockHeader.BlockSize = size;
                    BlockHeader.DestAddr = ((aBuffer[i++] << 8 )|aBuffer [i++]);
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)| ((aBuffer[i++] << 8 )|aBuffer [i++]);

                }
                else if (i > 123)
                {
                    if (i == 124)
                    {
                        size = (aBuffer[i++] << 8 )|aBuffer[i++];;
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
                        D_address =((aBuffer[i++] << 8 )|aBuffer [i++]);
                        BlockHeader.DestAddr = 0xfffffffa;

                    }
                    if (i == 126)
                    {
                        size = ((aBuffer[i++] << 8 )|aBuffer [i++]);
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
                    BlockHeader.BlockSize = (aBuffer[i++] << 8 )|aBuffer[i++];;
                    BlockHeader.DestAddr = (aBuffer[i++] << 8 )|aBuffer[i++];
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)|((aBuffer[i++]<<8 )|aBuffer [i++]);
                    if (BlockHeader.BlockSize == 0)
                    {                          //ReleaseFlashPump();
                                               //Flash_sector_erase(boot_address);
                                               //   state = bootmode_update(BOOT_MODE);
                        if (state != TRUE)
                        {
                            write_status =1;
                            // Example_Done();
                        }
                        //   Soft_reset();
                    }
                }

            }
            else if ((offset % 8) != 0)        //(offset != 8 || offset != 0)
            {
                if (offset > 8)
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 (aBuffer + i),
                                                 (16 - offset) );
                    if (state != TRUE)
                    {
                        write_status =1;
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
                                                 (8 - offset));
                    if (state != TRUE)
                    {
                        write_status =1;
                        //Example_Done();
                    }
                    i += (8 - offset);
                    BlockHeader.BlockSize -= 8 - offset;
                    BlockHeader.DestAddr += 8 - offset;

                }

            }
//            else if (BlockHeader.BlockSize < 8)
//            {
//                if (((128 - i) - BlockHeader.BlockSize) < 0)
//                {
//                    state = Flash_data_program_8(BlockHeader.DestAddr,
//                                                 aBuffer + i, (128 - i));
//                    if (state != TRUE)
//                    {
//                        write_status =1;
//                        // Example_Done();
//                        //return FALSE;
//                    }
//                    //  i += 128-i;
//                    BlockHeader.DestAddr += (128 - i);
//                    BlockHeader.BlockSize -= (128 - i);
//                    i += (128 - i);
//                }
//                else
//                {
//                    state = Flash_data_program_8(BlockHeader.DestAddr,
//                                                 aBuffer + i,
//                                                 (BlockHeader.BlockSize ));
//                    if (state != TRUE)
//                    {
//                        write_status =1;
//                        // Example_Done();
//                        //return FALSE;
//                    }
//                    i += BlockHeader.BlockSize;
//                    BlockHeader.DestAddr += BlockHeader.BlockSize;
//                    BlockHeader.BlockSize -= BlockHeader.BlockSize;
//                }
//
//            }
            else if (((128 - i) != 0) && ((128 - i) < 8))
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i, (128 - i));
                if (state != TRUE)
                {
                    write_status =1;
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
                if (state != TRUE)
                {
                    write_status =1;
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
        memset(aBuffer, 0, 128);
    }
//    GPIO_writePin(166, 1);
    return TRUE;
}
#endif



/**
 * @brief: this function it will handle the given hex data as per boot structure of TI controller
 *         it will validate key value and then write hex data as given block size and block address.
 * @author <19 FEB 2021 ; Basweshwar >
 * @return after successful completion return TRUE otherwise FALSE.
 */
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(copy_data, ".TI.ramfunc");
#endif
bool copy_data(uint8_t * aBuffer)
{
    bool state;
    uint32 k,j;
    uint16_t i=0;
  static  uint16_t size, D_address;
    uint32_t offset;

    if (mp == 0)
    {
        /* key verification */
        if ((aBuffer[0] << 8 |aBuffer[1]) != 0x08AA)
        {
            write_status[3] = 1;
            write_status[6] =iteration_cnt;
        }
        /*getting entry point address*/
        entry_point = (aBuffer[18] << 8 )|aBuffer[19];
        entry_point = entry_point << 16 | ((aBuffer[20]<<8) |aBuffer [21]);
        BlockHeader.BlockSize = aBuffer[22] << 8 |aBuffer [23];
        BlockHeader.DestAddr =  (aBuffer[24] << 8 )|aBuffer[25];
        BlockHeader.DestAddr = (BlockHeader.DestAddr << 16) | ((aBuffer[26]<<8) |aBuffer [27]);
        //k+=14;
        for (i = 28; i < 1024;)
              {
                  if (BlockHeader.BlockSize == 0)
                  {
                      BlockHeader.BlockSize =(aBuffer[i++] << 8 )|aBuffer[i++];
                      BlockHeader.DestAddr = (aBuffer[i++] << 8 )|aBuffer[i++];
                      BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)
                              |((aBuffer[i++]<<8 )|aBuffer [i++]);

                  }

                  if (BlockHeader.BlockSize < 8)
                  {
                      state = Flash_data_program_8(BlockHeader.DestAddr,
                                                   aBuffer + i,
                                                  ( BlockHeader.BlockSize ));
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
                  if ((1024 - i) < 8)
                  {
                      state = Flash_data_program_8(BlockHeader.DestAddr,
                                                   aBuffer + i, (1024 - i));
                      if (state != TRUE)
                      {
                          write_status[1]=1;
                          //Example_Done();
                      }

                      BlockHeader.DestAddr += 1024 - i;
                      BlockHeader.BlockSize -= 1024 - i;
                      i +=( 1024 - i);
                      //BlockHeader.DestAddr += 128 - i;
                      break;

                  }
                  if (BlockHeader.BlockSize != 0)
                  {
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
        memset(aBuffer, 0, 1024);
    }
    else
    {

        for (i = 0; i < data_size;)
        {
            offset = BlockHeader.DestAddr & 0x0000000f;   //address offset check

            if (BlockHeader.BlockSize == 0)
            {
                if (BlockHeader.DestAddr == 0xfffffffa)
                {
                    BlockHeader.BlockSize = size;
                    BlockHeader.DestAddr = D_address;
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)  |((aBuffer[i++] << 8 )|aBuffer [i++]);;

                }
                else if (BlockHeader.DestAddr == 0xfffffffb)
                {
                    BlockHeader.BlockSize = size;
                    BlockHeader.DestAddr = ((aBuffer[i++] << 8 )|aBuffer [i++]);
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)| ((aBuffer[i++] << 8 )|aBuffer [i++]);

                }
                else if (i > 1019)
                {
                    if (i == 1020)
                    {
                        size = (aBuffer[i++] << 8 )|aBuffer[i++];;
                        if (size == 0)
                        {
                            // ReleaseFlashPump();
                            // Flash_sector_erase(boot_address);
                            //  state = bootmode_update(BOOT_MODE);
                            if (state != TRUE)
                            {
                                write_status[1]= 1;  //2822
                                //  Example_Done();
                            }
                            //Soft_reset();
                        }
                        D_address =((aBuffer[i++] << 8 )|aBuffer [i++]);
                        BlockHeader.DestAddr = 0xfffffffa;

                    }
                    if (i == 1022)
                    {
                        size = ((aBuffer[i++] << 8 )|aBuffer [i++]);
                        if (size == 0)
                        {                          //ReleaseFlashPump();
                                                   //Flash_sector_erase(boot_address);
                                                   //  state = bootmode_update(BOOT_MODE);
                            if (state != TRUE)
                            {
                                write_status[1]= 1; //2822
                                // Example_Done();
                            }
                            // Soft_reset();
                        }
                        BlockHeader.DestAddr = 0xfffffffb;
                    }

                }
                else
                {
                    BlockHeader.BlockSize = (aBuffer[i++] << 8 )|aBuffer[i++];
                    BlockHeader.DestAddr = (aBuffer[i++] << 8 )|aBuffer[i++];
                    BlockHeader.DestAddr = (BlockHeader.DestAddr << 16)|((aBuffer[i++]<<8 )|aBuffer [i++]);
                    if (BlockHeader.BlockSize == 0)
                    {                          //ReleaseFlashPump();
                                               //Flash_sector_erase(boot_address);
                                               //   state = bootmode_update(BOOT_MODE);
                        if (state != TRUE)
                        {
                            write_status[1]=1;
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
                                                 (16 - offset) );
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
                                                 (8 - offset));
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
//                if (((1024 - i) - BlockHeader.BlockSize) < 0)
//                {
//                    state = Flash_data_program_8(BlockHeader.DestAddr,
//                                                 aBuffer + i, (128 - i));
//                    if (state != TRUE)
//                    {
//                        write_status =1;
//                        // Example_Done();
//                        //return FALSE;
//                    }
//                    //  i += 128-i;
//                    BlockHeader.DestAddr += (128 - i);
//                    BlockHeader.BlockSize -= (128 - i);
//                    i += (128 - i);
//                }
//                else
                {
                    state = Flash_data_program_8(BlockHeader.DestAddr,
                                                 aBuffer + i,
                                                 (BlockHeader.BlockSize ));
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
            else if (((1024 - i) != 0) && ((1024 - i) < 8))
            {
                state = Flash_data_program_8(BlockHeader.DestAddr,
                                             aBuffer + i, (1024 - i));
                if (state != TRUE)
                {
                    write_status[1]=1;
                    //Example_Done();
                }

                BlockHeader.DestAddr += (1024 - i);
                BlockHeader.BlockSize -= (1024 - i);
                i += (1024 - i);
                break;

            }
            else
            {
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
        memset(aBuffer, 0, 1024);
    }
//    GPIO_writePin(166, 1);
    return TRUE;
}

void cpu1_ipc_intr_write_status(uint8_t status)
{
    memset(buf_rx,0, PAYLOAD);
    buf_rx[0]= 0x0d;
    buf_rx[1]=status;
    buf_rx[2]= write_status[4];
    buf_rx[3]= write_status[5];
    buf_rx[4]= write_status[6];

    IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,IPC_CMD_READ_MEM, (uint32_t)buf_rx, 5);
        IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);

}

//static bool gCount;

#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(cm_handler, ".TI.ramfunc");
#endif
bool cm_handler(uint8_t * aBuffer)
{

    bool lRetStatus = false;
    uint8_t app_status=0;

    uint8_t lrxCmd = aBuffer[4];
//    uint8_t lFwController = aRxBuffer[2];
    uint32_t i,j,state=0;        //mode;
    uint32_t sector_add = 0;
    uint16_t lFlashData =  BOOT_MODE_CD_WORD;
    uint16_t lFWPayloadLength = 0;
    uint16_t Word_Point = 0;

    data_size =0;
    switch(lrxCmd)
    {
    case BOOT_MODE://0x01

//        Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);

        Example_EraseSector(Bzero_Sector13_start);

        grlFlashBOOTPGMSectorWrite(Bzero_Sector13_start, &lFlashData, 2);

//        Flash_releasePumpSemaphore();

        break;
    case WRITE_FLASH://0x09

        data_size= (aBuffer[1] <<8 |aBuffer[2])-6;
//        Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);
        memmove(&aBuffer[0], &aBuffer[6], data_size);
//        memset(&aBuffer[data_size],0,(1040-data_size));

        iteration_cnt++;
//        for(j=0;j<1024;j+=128)
//        {
        copy_data(aBuffer);
//        Tx_cpu1_intr(0x11);
//        }
//        Flash_releasePumpSemaphore();

//        if(gCount == true)
//        {
//            Tx_cpu1_intr(0xBC);//Write flash
//            gCount = false;
//        }

        break;

    case ERASE_SECTOR://0x0A
//        gCount = true;
//        Flash_claimPumpSemaphore(FLASH_CM_WRAPPER);
        for( i =0 ; i < FLASH_ADDRESSES_CNT; i++)
        {
           sector_add = flash_address[i];

           state = Example_EraseSector(sector_add);
//           DEVICE_DELAY_US(60000);
           if (state == false)
           {
        //            Example_Done();
           }
        }
//        Flash_releasePumpSemaphore();

//        Tx_cpu1_intr(0xBA);// Erase write

        break;

    case FLASH_WRITE_STATUS:
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
              cpu1_ipc_intr_write_status(app_status);
              memset(write_status, 0, 8);
              iteration_cnt=0;
        break;

    }

    return lRetStatus;

}






