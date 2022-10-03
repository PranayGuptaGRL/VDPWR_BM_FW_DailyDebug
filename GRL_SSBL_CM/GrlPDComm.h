/*
 * GrlPDComm.h
 *
 *  Created on: 01-Feb-2022
 *      Author: pranay
 */

#ifndef GRLPDCOMM_H_
#define GRLPDCOMM_H_

#include <string.h>
//
//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
//#include "inc/hw_types.h"
//#include "inc/hw_sysctl.h"
//
#include "driverlib_cm/gpio.h"
#include "driverlib_cm/interrupt.h"
#include "driverlib_cm/flash.h"
#include "driverlib_cm/cpu.h"
#include "driverlib_cm.h"
//#include "cm.h"
#include "driverlib_cm/ipc.h"
//#include "driverlib_cm/sysctl.h"
#include "driverlib_cm/systick.h"

//#include "board_drivers/pinout.h"
#include "flash_programming_f2838x_cm.h"
#include "F021_F2838x_CM.h"

//#define WRITE_STATUS     0x07             //2822

uint32_t entry_point;


#define IPC_CMD_READ_MEM   0x1001
#define PAYLOAD 20
#define HEADER_BYTE_CNT 2
#define FLASH_ADDRESSES_CNT         9

extern uint32_t buf_rx_cm[150] ;
extern uint8_t buf_rx[PAYLOAD];
extern uint8_t buf_tx[PAYLOAD];

struct tcp_pcb *Rxtcb;
#define BOOT_MODE        0x01
#define WRITE_FLASH      0x09
#define ERASE_SECTOR     0x0A
#define FLASH_WRITE_STATUS     0x07

#define PGM_MODE_CD_WORD    0x0BAA
#define BOOT_MODE_CD_WORD   0x0B0B


#define WRITE_SUCESS  0X01
#define ADDRESS_ERROR 0X02
#define VERIFY_ERROR  0X04
#define KEY_ERROR     0x08     //(refer as invalid file)

//char * buf_tx_start_msg = "Something to show UDP is working \n";
//extern uint32_t buf_tx_start_msg_count = 35;

extern uint16_t cont_rx_udp;

struct pbuf *pbuf1_tx;

#define AIRCR_REG   0xD0C

typedef enum
{
    RX_API_IS_SET = 0x01,
    RX_API_IS_PGM = 0x02,
    Rx_API_IS_GET = 0x07,

}rxApiType;

bool cm_handler(uint8_t * aBuffer);


void grlAsciitoHexConvertion(uint8_t *aSrcRxBuf, uint32_t *aDestRxBuf, uint16_t aPayLoadLength);




#endif /* GRLPDCOMM_H_ */
