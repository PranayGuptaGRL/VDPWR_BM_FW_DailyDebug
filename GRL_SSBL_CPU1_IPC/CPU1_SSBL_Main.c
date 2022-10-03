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
#include <CPU1_SSBL_Main.h>


#pragma DATA_SECTION(grlRxbuf, "MSGRAM_CPU_TO_CM")
#pragma DATA_SECTION(gVar, "IPC_RX_DECODE_BUF");

struct HEADER
{
    int16_t BlockSize;
    uint32_t DestAddr;
} BlockHeader;

uint8_t iteration_cnt=0;
static uint16_t mp = 0;
uint8_t write_status[8]={0,0,0,0,0,0,0,0};


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
//    uint32 j;
    int16_t i;
static uint16_t size, D_address;
    uint32_t offset;

    if (mp == 0)
    {
        /* key verification */
        if ((aBuffer[0]) != 0x08AA)
        {
            write_status[3]=1;
            write_status[6]=iteration_cnt;
             return FALSE;
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
//                     return FALSE;
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

void cm_ipc_intr_write_status(uint8_t status)
{
    memset(grlRxbuf,0, FW_PL_LENGTH);
    grlRxbuf[0]= 0x0d;
    grlRxbuf[1]=status;
    grlRxbuf[2]= write_status[4];
    grlRxbuf[3]= write_status[5];
    grlRxbuf[4]= write_status[6];
    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                                       IPC_CMD_READ_MEM, (uint32_t)grlRxbuf, 2 );
    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);

}

uint8_t* ip_addrss_formation(uint32_t ip)
{
   uint8_t num=0,i,j,k,temp=0;
   int8_t l;
   static  uint8_t ip_add[17];
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

bool DataRxHandler(uint8_t * aBuffer)
{

    bool lRetStatus = false;
    uint8_t app_status=0;
    uint8_t lrxCmd = aBuffer[4];
//    uint8_t lFwController = aRxBuffer[2];
    uint32_t i,state=0;        //mode;
    uint32_t sector_add = 0;
    uint8_t lBuffer[286] = {0};
    uint16_t lFlashData =  BOOT_MODE_CD_WORD;
    uint16_t lFWPayloadLength = 0;
    uint16_t Word_Point = 0;

    switch(lrxCmd)
    {
    case BOOT_MODE://0x01
//        Flash_claimPumpSemaphore(FLASH_CPU1_WRAPPER);

//        Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
//                       (uint32 *)Grl_Boot_Pgm_Sector);
        Example_EraseSector(Bzero_Sector13_start);

        grlFlashBOOTPGMSectorWrite(Bzero_Sector13_start, &lFlashData, 1);

//        Flash_releasePumpSemaphore();

        break;
    case WRITE_FLASH://0x09
        lFWPayloadLength = ((aBuffer[6] << 8 )| aBuffer[5] );
//        Flash_claimPumpSemaphore(FLASH_CPU1_WRAPPER);

//        for(i=0; i < WORDS_IN_FLASH_BUFFER; i++)
//        {
//            Buffer[i] = i;
//        }
//        Example_ProgramUsingAutoECC();
//        memcpy(&lBuffer[0], &aBuffer[7], lFWPayloadLength);

        iteration_cnt++;
        for (i = 7, Word_Point = 0; i < lFWPayloadLength+7; i += 2)
        {
            lBuffer[Word_Point++] = (aBuffer[i] | (aBuffer[i + 1] << 8));

        }

        copy_data(lBuffer);

//        Flash_releasePumpSemaphore();

        break;

    case ERASE_SECTOR://0x0A

        grllcddisplay(1, "CC-FW UPD...");
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
    case 0x0B:
        Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
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

void is_TiFWUpdate(uint8_t * aRxBuffer)
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

        break;
    }
}

void isAPI_FWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lFwController = aRxBuffer[2];
    uint32_t ip_address;
    uint8_t *ip;

    switch(lFwController)
    {
       case 0x01:
           if(aRxBuffer[3]==0x04)
               SysCtl_resetDevice();

        case 0x05:
            aRxBuffer[3]=aRxBuffer[3]<<8|aRxBuffer[4];
            aRxBuffer[5]=aRxBuffer[5]<<8|aRxBuffer[6];
            ip_address=aRxBuffer[3];
            ip_address=ip_address<<16;
            ip_address=ip_address | aRxBuffer[5];

           ip = ip_addrss_formation(ip_address);
           grllcddisplay(2, ip);


            break;
        case 0x06 ://Ti
                is_TiFWUpdate(aRxBuffer);
            break;
    }
}

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

void lcd_init()
{
//    DEVICE_DELAY_US(100);
    display_data_command(DISPLAY_COMMAND, 0x2A);  //function set (extended command set)
    display_data_command(DISPLAY_COMMAND,0x71);  //function selection A, disable internal Vdd regualtor
    display_data_command(DISPLAY_DATA,0x00);
    display_data_command(DISPLAY_COMMAND,0x28);  //function set (fundamental command set)
    display_data_command(DISPLAY_COMMAND,0x08);  //display off, cursor off, blinku off
    display_data_command(DISPLAY_COMMAND,0x2A);  //fnction set (extended command set)
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
void initSCIAFIFO()
{
    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);

    SCI_enableModule(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_enableInterrupt(SCIA_BASE, (SCI_INT_RXFF ));
    SCI_disableInterrupt(SCIA_BASE, (SCI_INT_RXERR | SCI_INT_TXFF) );

    //
    // The transmit FIFO generates an interrupt when FIFO status
    // bits are less than or equal to 2 out of 16 words
    // The receive FIFO generates an interrupt when FIFO status
    // bits are greater than equal to 2 out of 16 words
    //
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX1, SCI_FIFO_RX3);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);

}

void UART_FIFO_init()
{
    //
     // Interrupts that are used in this example are re-mapped to
     // ISR functions found within this file.
     //
     Interrupt_register(INT_SCIA_RX, sciaRXFIFOISR);
     Interrupt_register(INT_SCIA_TX, sciaTXFIFOISR);

     initSCIAFIFO();

     Interrupt_enable(INT_SCIA_RX);
     Interrupt_enable(INT_SCIA_TX);
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}
void FWOpcodeDataAppend(uint8_t * aFWOpcodeBuffer, uint8_t aFWPayloadLength)
{

    memcpy(&gFWOpcdeBuf[gFWOpcodeBufLength], aFWOpcodeBuffer, aFWPayloadLength);
    gFWOpcodeBufLength += aFWPayloadLength;

}



void FWRecvCmdHandler(uint8_t * aFWBuffer)
{
//    static uint8_t i = 0;
    switch(aFWBuffer[0])
    {
        case 0xC2://FW Streaming mode
            if(gFWOpcodeBufLength ==0)
                memset(gFWOpcdeBuf, 0x00, FW_PL_LENGTH);

                FWOpcodeDataAppend( &aFWBuffer[2], aFWBuffer[1] );

        break;

        default:
            switch(aFWBuffer[4])
            {
                case 0x09://Flash Write

                    memset(grlRxbuf, 0x00, FW_PL_LENGTH);

                    memcpy(&grlRxbuf[0], aFWBuffer, (aFWBuffer[1]+2) );

                    memcpy(&grlRxbuf[ (aFWBuffer[1]+2) ], gFWOpcdeBuf, gFWOpcodeBufLength);


                    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                                      IPC_CMD_READ_MEM, (uint32_t)grlRxbuf, ((gFWOpcodeBufLength+6)) );

                    IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);

                    memset(gFWOpcdeBuf, 0x00, FW_PL_LENGTH);
                    gFWOpcodeBufLength = 0;

                break;

                case 0x04:
                    SysCtl_resetDevice();
                    break;


                default://Erase/PGM/BOOT cmds

                    if(aFWBuffer[4] == 0x02)//CM PGM
                        grllcddisplay(1, "CM-PGM MODE");
                    else if(aFWBuffer[4] == 0x0A)//CM ERASE
                       grllcddisplay(1, "CM-FW UPD ...");
                    memset(grlRxbuf, 0x00,FW_PL_LENGTH );
                    memcpy(&grlRxbuf[0], aFWBuffer, (aFWBuffer[1]+2) );

//                        grlRs485Rx_CmTx_DataHandler();
                    IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                                      IPC_CMD_READ_MEM, (uint32_t)grlRxbuf, (grlRxbuf[1] + 2 ) );

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
    {
        FWRecvCmdHandler(rUARTDataA);
    }

//    else
//        RecvDataHandler(rUARTDataA);

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
void UART_Init()
{
    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO85_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO84_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO84_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO84_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO84_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO84_PIN_SCITXDA, GPIO_QUAL_ASYNC);

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

__interrupt void CM_CPU1_IPC_ISR1()
{
    uint32_t command, addr, data;
//    uint32_t cad_rx = 0;

    IPC_readCommand(IPC_CPU1_L_CM_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);
    IPC_ackFlagRtoL(IPC_CPU1_L_CM_R, IPC_FLAG1);

    int i =0,index = 0;

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    memset(gVar, 0x00, DATASIZE);
    memset(grlRxbuf, 0x00, DATASIZE);

    for (i = 0; i < (data); i++)
    {
        gVar[0+i] = (*((uint32_t *)addr + i) );
    }

    for (i = 0; i <= (data/4); i++)
    {
        grlRxbuf[index++] = (gVar[i] & 0xFF);
        grlRxbuf[index++] = (gVar[i] & 0xFF00) >> 8;
        grlRxbuf[index++] = (gVar[i] & 0xFF0000) >> 16;
        grlRxbuf[index++] = (gVar[i] & 0xFF000000) >> 24;
    }

//    memcpy(App_Data_Buf, grlRxbuf, DATASIZE);

    if( ( (grlRxbuf[0] & 0x0F) == 0x01) || ( (grlRxbuf[0] & 0x0F) == 0x07) || ( (grlRxbuf[0] & 0x0F) == 0x02)  )
    {
        isAPI_FWUpdate(grlRxbuf);

    }
    else //if related to display only one byte will be pushed
    {
        if( (grlRxbuf[0] & 0xFF)== PPS_PGM_MODE)
        {
            grllcddisplay(1, "PPS-PGM MODE  ");
        }
        else if( (grlRxbuf[0] & 0xFF)== PPS_FW_UPD)
        {
            grllcddisplay(1, "PPS- FW UPD...");
        }
        else if((grlRxbuf[0] & 0xFF)== 0x0d)
        {
            sDataA[0]=grlRxbuf[0] & 0xFF;
            sDataA[1]=grlRxbuf[1] & 0xFF ;
            sDataA[2]=grlRxbuf[2] & 0xFF ;
            sDataA[3]=grlRxbuf[3] & 0xFF ;
            sDataA[4]=grlRxbuf[4] & 0xFF ;
            SCI_writeCharArray(SCIA_BASE, sDataA, 10);
        }



//        else if( (grlRxbuf[0] & 0xFF)== CM_ERASE_FLASH)
//        {
//            grllcddisplay(1, "CM-Flash-Ers...");
//        }
//        else if( (grlRxbuf[0] & 0xFF)== CM_FLASH_WRITE)
//         {
//             grllcddisplay(1, "CM-Flash-WRI...");
//         }
    }

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
void LEDsInit()
{

    GPIO_setDirectionMode(DBG_GREEN_LED_GPIO36, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DBG_GREEN_LED_GPIO36, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(DBG_GREEN_LED_GPIO36, GPIO_CORE_CM);
    GPIO_setQualificationMode(DBG_GREEN_LED_GPIO36, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_36_GPIO36);

    GPIO_setDirectionMode(DBG_BLUE_LED_GPIO37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DBG_BLUE_LED_GPIO37, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(DBG_BLUE_LED_GPIO37, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(DBG_BLUE_LED_GPIO37, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_37_GPIO37);

    GPIO_setDirectionMode(OCP_RED_LED_GPIO38, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(OCP_RED_LED_GPIO38, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(OCP_RED_LED_GPIO38, GPIO_CORE_CPU2);
    GPIO_setQualificationMode(OCP_RED_LED_GPIO38, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_38_GPIO38);
}
//
// Main
//
void main(void)
{

//    uint32 i,j;
    Fapi_FlashStatusWordType oFlashStatusWord;
    Device_init();

#ifdef _FLASH
    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif

    DEVICE_DELAY_US(20000);//20mS
    // Boot CPU2 core
    //
#ifdef _FLASH
//    Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
#endif

    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    SPI1_LCD_Config();

    UART_Init();
    LEDsInit();
    //I2C B initialization to CPU2
    GPIO_setPinConfig(GPIO_34_I2CB_SDA);
    GPIO_setPinConfig(GPIO_35_I2CB_SCL);

    GPIO_setPadConfig(GPIO34_PIN_SDAB, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPadConfig(GPIO35_PIN_SCLB, GPIO_PIN_TYPE_PULLUP);

    GPIO_setQualificationMode(GPIO34_PIN_SDAB, GPIO_QUAL_ASYNC);
    GPIO_setQualificationMode(GPIO35_PIN_SCLB, GPIO_QUAL_ASYNC);

    GPIO_setMasterCore(GPIO34_PIN_SDAB, GPIO_CORE_CPU2);
    GPIO_setMasterCore(GPIO35_PIN_SCLB, GPIO_CORE_CPU2);

    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL7_I2C, 2, SYSCTL_CPUSEL_CPU2 );

    CM_EthernetIOMuxHandler();
//    MemCfg_setGSRAMMasterSel( (MEMCFG_SECT_GS6 | MEMCFG_SECT_GS7 | MEMCFG_SECT_GS8 | MEMCFG_SECT_GS9 | MEMCFG_SECT_GS10  ),MEMCFG_GSRAMMASTER_CPU2);
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT1, CM_CPU1_IPC_ISR1);

    IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);

    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, 3);

    Fapi_setActiveFlashBank(Fapi_FlashBank0);

    //UART init
    UART_FIFO_init();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    lcd_init();

    Flash_disableECC(FLASH0ECC_BASE);
//    Flash_releasePumpSemaphore();


    grllcddisplay(1, "CC-PGM MODE");

    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG28);

    while(1)
    {
        Fapi_doBlankCheck((uint32_t *)Bzero_Sector13_start,1, &oFlashStatusWord);

        if((oFlashStatusWord.au32StatusWord[1] & 0x0000FFFF) == BOOT_MODE_CD_WORD)
        {
            Flash_releasePumpSemaphore();
//            Flash_claimPumpSemaphore(FLASH_CPU2_WRAPPER);
            __asm(" LB 0x86000 ");     /*jumping to entry point*/
        }
        else if(((oFlashStatusWord.au32StatusWord[1] & 0x0000FFFF) != BOOT_MODE_CD_WORD) && !is_program)
        {
            Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
            is_program=1;
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
