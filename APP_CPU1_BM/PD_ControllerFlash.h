/*
 * PD_ControllerFlash.h
 *
 *  Created on: 17-Feb-2022
 *      Author: pranay
 */

#ifndef PD_CONTROLLERFLASH_H_
#define PD_CONTROLLERFLASH_H_

#include <Includes.h>

#define PGM_MODE_CD_WORD    0x0BAA
#define BOOT_MODE_CD_WORD   0x0B0B
#define BOOTPGM_FLASH_SECTOR_ADDR   0x0BE000


void grlFlash_InitModule();
void grlCPU1PGMmodeSetHandler(uint8_t * aRxBuffer);


bool grlFlashBOOTPGMSectorWrite(uint32_t Flash_sector, uint16_t *Buffer, uint16_t words);

#endif /* PD_CONTROLLERFLASH_H_ */
