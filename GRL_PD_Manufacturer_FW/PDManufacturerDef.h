/*
 * GrlManufacturerDef.h
 *
 *  Created on: Jun 12, 2019
 *      Author: Prasanna
 */

#ifndef GRLMANUFACTURERDEF_H_
#define GRLMANUFACTURERDEF_H_

#define LOOPBACK_EVENT 1
#define TIMER_THREAD 1
//#define IDLE_THREAD
#define ERR_QUEUE
//#define WD_TIMER

#define WRITE 0
#define READ 1

#define ATTACH 1
#define DETACH 0

#define VBUS_READ_CCG3PA

#define FUNCTION_CARD_ID_MASK		0xF0

#define CCG3PA_FW_UPDATE_CHECK

#define CLR_3P5DB_DEEMP_MASK		0xFFFFFFC0
#define CLR_6DB_DEEMP_MASK			0xFFFFE07F
#define CLR_TX_SWING_FULL_MASK		0xFFE03FFF
#define CLR_TX_SWING_LOW_MASK		0xF01FFFFF
#define LNK_PHY_TX_TRIM_REG         0xE003303C
#define PHY_CONF_REG                0xE003100C

#define SPI_INTR		0xE0000C08
#define SPI_INTR_MASK	0xE0000C0C
#define SPI_STATUS		0xE0000C04

#define CONFIG_3P5DB_DE_EMP	1
#define CONFIG_6DB_DE_EMP	2
#define CONFIG_FULLSWING	1
#define CONFIG_LOWSWING	    2

#define USB2_0_PRE_EMP_ENABLE  1
#define USB2_0_PRE_EMP_DISABLE 2

#define CLR_USB2_0_PRE_EMP_MASK    0xFFBFFFFF
#define READ_RS485_DATA_REG_SUCCESS_VAL		0x40

#define BIT_SHIFT_0  0
#define BIT_SHIFT_1  1
#define BIT_SHIFT_2  2
#define BIT_SHIFT_3  3
#define BIT_SHIFT_4  4
#define BIT_SHIFT_5  5
#define BIT_SHIFT_6  6
#define BIT_SHIFT_7  7
#define BIT_SHIFT_8  8
#define BIT_SHIFT_12  12
#define BIT_SHIFT_16  16
#define BIT_SHIFT_17  17
#define BIT_SHIFT_20  20
#define BIT_SHIFT_22  22
#define BIT_SHIFT_24  24
#define BIT_SHIFT_28  28
#define BIT_SHIFT_32  32

#define BIT_MASK_1 0x00000001
#define BIT_MASK_2 0x00000003
#define BIT_MASK_3 0x00000007
#define BIT_MASK_4 0x0000000F
#define BIT_MASK_5 0x0000001F
#define BIT_MASK_6 0x0000003F
#define BIT_MASK_7 0x0000007F
#define BIT_MASK_8 0x000000FF
#define BIT_MASK_12 0x00000FFF
#define BIT_MASK_16 0x0000FFFF
#define BIT_MASK_20 0x000FFFFF
#define BIT_MASK_24 0x00FFFFFF
#define BIT_MASK_28 0x0FFFFFFF
#define BIT_MASK_32 0xFFFFFFFF

#define BYTE_1  1
#define BYTE_2  2
#define BYTE_3  3
#define BYTE_4  4
#define BYTE_5  5
#define BYTE_6  6
#define BYTE_7  7
#define BYTE_8  8
#define BYTE_9  9
#define BYTE_12  12
#define BYTE_13  13
#define BYTE_14  14
#define BYTE_15  15
#define BYTE_16  16
#define BYTE_17  17
#define BYTE_18  18

#define BUF_SIZE_1BYTE  1
#define BUF_SIZE_2BYTE  2
#define BUF_SIZE_4BYTE  4
#define BUF_SIZE_5BYTE  5
#define BUF_SIZE_8BYTE  8
#define BUF_SIZE_12BYTE 12
#define BUF_SIZE_16BYTE 16
#define BUF_SIZE_20BYTE 20
#define BUF_SIZE_26BYTE 26
#define BUF_SIZE_32BYTE 32
#define BUF_SIZE_48BYTE 48

#define BUF_SIZE_64BYTE 64
#define BUF_SIZE_128BYTE 128
#define BUF_SIZE_255BYTE 255
#define BUF_SIZE_256BYTE 256
#define BUF_SIZE_280BYTE 280
#define BUF_SIZE_496BYTE 496
#define BUF_SIZE_512BYTE 512
#define BUF_SIZE_1K_BYTE 1024
#define BUF_SIZE_2K_BYTE 2048

#define MAX_RTOS_TIMER_TICKS_IN_MIN	60000
#define MAX_NO_OF_PDO 7
#define DEBUG_FW_GPIO_TOGGLE 	1
#define SET          1
#define RESET        0

#define REG_GPIO_SET_BIT_1 0x00000001
#define REG_GPIO_CLR_BIT_1 0xFFFFFFFE
#define REG_GPIO     1

#define DBG1 1
#define DBG2 2
//#define DBGLOG	0

#define TI_BOOT_MODE   1
#define TI_PGM_MODE    0


void CyFxGpioIntrCb (uint8_t );
void RS485ErrHandler();

void DEBUG_LOG(uint8_t, uint8_t, uint32_t);

#endif /* GRLMANUFACTURERDEF_H_ */
