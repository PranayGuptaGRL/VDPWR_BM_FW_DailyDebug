//###########################################################################
//
// FILE:   flash_programming_f2838x_c28x.h
//
// TITLE:  A set of Constant Values for the F2838x C28x Family.
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

#ifndef FLASH_PROGRAMMING_F2838X_C28X_H
#define FLASH_PROGRAMMING_F2838X_C28X_H

#include "driverlib.h"
#include "device.h"
#include "string.h"
#include "f2838x_device.h"
#include "f2838x_flash.h"
#include "F021_F2838x_C28x.h"
//
// Bank0 Sector start addresses
//
#define FlashStartAddress           0x80000
#define Grl_Boot_Pgm_Sector         0x80002
#define Bzero_Sector0_start         0x80004
#define Bzero_Sector1_start	        0x82000
#define Bzero_Sector2_start	        0x84000
#define Bzero_Sector3_start	        0x86000
#define Bzero_Sector4_start	        0x88000
#define Bzero_Sector5_start	        0x90000
#define Bzero_Sector6_start	        0x98000
#define Bzero_Sector7_start	        0xA0000
#define Bzero_Sector8_start	        0xA8000
#define Bzero_Sector9_start	        0xB0000
#define Bzero_Sector10_start	    0xB8000
#define Bzero_Sector11_start	    0xBA000
#define Bzero_Sector12_start	    0xBC000
#define Bzero_Sector13_start	    0xBE000
#define FlashEndAddress             0xBFFFF

//
// Sector length in number of 32bits
//
#define Sector16KB_u32length   0x1000
#define Sector64KB_u32length   0x4000

#define PGM_MODE_CD_WORD    0x0BAA
#define BOOT_MODE_CD_WORD   0x0B0B
#define SPIC_LCD_MOSI_GPIO20            20
#define SPIC_LCD_MISO_GPIO21            21
#define SPIC_LCD_SCLK_GPIO22            22
#define SPIC_LCD_CS_GPIO23              23
#define LOW 0
#define HIGH 1

#define GPIO34_PIN_SDAB             34
#define GPIO35_PIN_SCLB             35


#define  PPS_PGM_MODE    0xDD
#define  PPS_FW_UPD      0xEE
#define CM_PGM_MODE      0xAA
#define CM_FW_UPD      0xBB
#define GOT_IP          0xCC
//#define CM_ERASE_FLASH     0xBA
//#define CM_FLASH_WRITE     0xBC

#define DISPLAY_COMMAND   2
#define DISPLAY_DATA      1
#define START_DATA        0xFA
#define START_COMMAND     0xF8

#define DEVICE_GPIO85_PIN_SCIRXDA     85U             // GPIO number for SCI RX
#define DEVICE_GPIO84_PIN_SCITXDA     84U             // GPIO number for SCI TX
#define DEVICE_GPIO85_CFG_SCIRXDA     GPIO_85_SCIA_RX // "pinConfig" for SCI RX
#define DEVICE_GPIO84_CFG_SCITXDA     GPIO_84_SCIA_TX // "pinConfig" for SCI TX
#define FW_PL_LENGTH        1052


#define ETHERNET
#define  WORDS_IN_FLASH_BUFFER    0x100
#define IPC_CMD_READ_MEM   0x1001
#define DATASIZE        286
#define APP_DATA         0x08
#define BOOT_MODE        0x01
#define WRITE_FLASH      0x09
#define ERASE_SECTOR     0x0A
#define WRITE_STATUS     0x07
#define FLASH_ADDRESSES_CNT         12
#define DBG_GREEN_LED_GPIO36            36
#define DBG_BLUE_LED_GPIO37            37
#define OCP_RED_LED_GPIO38              38




#define WRITE_SUCESS  0X01
#define ADDRESS_ERROR 0X02
#define VERIFY_ERROR  0X04
#define KEY_ERROR     0x08     //(refer as invalid file)


__interrupt void sciaRXFIFOISR(void);
__interrupt void sciaTXFIFOISR(void);
void Example_ProgramUsingAutoECC(void);
void grllcddisplay(uint8_t line,char *display_data);
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
bool Flash_data_program_8(uint32 Flash_sector, uint16 *Buffer, uint16 words);
void FMSTAT_Fail(void);
void ECC_Fail(void);
bool Example_EraseSector(uint32_t);


typedef enum
{
    TI_CM   = 0,
    TI_CPU1_CONTROLCARD = 1,
    TI_CPU2_PPS = 2
}Ti_Cores;

uint16_t rUARTDataA[16];
uint16_t rUARTDataPointA;
uint16_t sDataA[10];
uint8_t gFWOpcdeBuf[FW_PL_LENGTH];
uint8_t grlRxbuf[FW_PL_LENGTH] = {0};
uint32_t gVar[DATASIZE] = {0};
uint8_t is_program;
uint32_t entry_point;
volatile uint16_t gFWOpcodeBufLength;

/*flash sector addresses*/
uint32 flash_address[FLASH_ADDRESSES_CNT] = { 0x86000, 0x88000,
                             0x90000, 0x98000, 0xA0000, 0xA8000, 0xB0000,
                             0xB8000, 0xBA000, 0xBC000 };


#endif /* FLASH_PROGRAMMING_F2838X_C28X_H */
