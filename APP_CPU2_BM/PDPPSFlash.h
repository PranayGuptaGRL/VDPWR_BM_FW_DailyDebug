/*
 * PD_ControllerFlash.h
 *
 *  Created on: 17-Feb-2022
 *      Author: pranay
 */

#ifndef PDPPSFLASH_H_
#define PDPPSFLASH_H_

#include <PDPPSManuIncludes.h>

#define PGM_MODE_CD_WORD    0x0BAA
#define BOOT_MODE_CD_WORD   0x0B0B
#define BOOTPGM_FLASH_SECTOR_ADDR   0x0BE000


void grlFlash_InitModule();

void grlCPU2PGMmodeSetHandler(uint8_t * aRxBuffer);

bool grlFlashBOOTPGMSectorWrite(uint32_t Flash_sector, uint16_t *Buffer, uint16_t words);

#endif /* PDPPSFLASH_H_ */
