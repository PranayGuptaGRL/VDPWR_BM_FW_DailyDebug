/*
 * GrlPDComm.h
 *
 *  Created on: 01-Feb-2022
 *      Author: pranay
 */

#ifndef GRLPDCOMM_H_
#define GRLPDCOMM_H_

#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_emac.h"

#include "driverlib_cm/ethernet.h"
#include "driverlib_cm/gpio.h"
#include "driverlib_cm/interrupt.h"
#include "driverlib_cm/flash.h"

#include "driverlib_cm/sysctl.h"
#include "driverlib_cm/systick.h"

#include "utils/lwiplib.h"
#include "board_drivers/pinout.h"
#include "lwipopts.h"
#include "lwip/dhcp.h"
#include "Grl_CM_timer.h"

//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************

extern unsigned long IPAddr; //=  0xC0A80004; // 0xC0A80004; //192.168.0.4
extern unsigned long IPAddr_dup; //=  0x0400A8C0; //192.168.0.4
extern unsigned long NetMask; //= 0xFFFFFF00;
extern unsigned long GWAddr; //= 0x00000000;
extern unsigned char pucMACArray[8];

#define IPC_CMD_READ_MEM   0x2002
#define PAYLOAD 1088

#define HEADER_BYTE_CNT         2
#define MAX_DATA_TX_SIZE        286

#define CPU1_RXBUFSIZE          MAX_DATA_TX_SIZE
#define CPU2_RXBUFSIZE          MAX_DATA_TX_SIZE
#define FW_UPDATE_MAX_DATA_TX    286
#define FW_UPDATE_HEADER_BYTECNT    5 // FW payload will be indicated in byte 5 and Byte 6, so FW header count is 5

//#define GRL_MAC_0      0x00D5B370
//#define GRL_MAC_1      0x00009074

#define GRL_MAC_0      0x0070B3D5
#define GRL_MAC_1      0x00749000

#define PROGRAM   0xFFFF
#define BOOT      0xAAAA


#define SYS_ID_PAYLOAD_LENGHT   18 //MAx of 18 Bytes being filled into sysID buffer when requested for GetSysID details - buffer being filled is GRL-VDPWR-XXXXYYYY(XXXX: MFD YEAR,YYYY:SYS ID)
#define SYSID_HEADER_LENGTH     1 //1Byte of Header is being pushed . header will be total payload lenght of system serial ID
#define SYS_ID_MAXLENGTH    SYS_ID_PAYLOAD_LENGHT + SYSID_HEADER_LENGTH //system serial number details length includes payload lenght and header length

extern uint32_t buf_rx_cm[150] ;
extern u8_t buf_rx[PAYLOAD];
extern u8_t buf_tx[PAYLOAD];
//extern u8_t buf_rx_CPU2[150];

struct tcp_pcb *Rxtcb;
struct tcp_pcb *Rxtcb_5003;
static uint8_t gSysSnoLength;
volatile bool gReadAPI;
volatile bool isEchobackReq;
//volatile bool gReadPollingData;

/**Pranay,03Sept'22, Handling the echoback and msg ID sequence during FW udpates, If prev and present MSG ID is same then ignore received FW packet*/
volatile u8_t gFWupdPresentRxMsgID;
volatile u8_t gFWupdPrevMsgID;

uint16_t gSystemID;
uint16_t gMFDYear;
uint8_t gMFDMonth;
//char * buf_tx_start_msg = "Something to show UDP is working \n";
//extern uint32_t buf_tx_start_msg_count = 35;

extern uint16_t cont_rx_udp;

struct pbuf *pbuf1_tx;

#define AIRCR_REG   0xD0C
#define PGM_MODE_CD_WORD    0xFFFF0BAA
#define BOOT_MODE_CD_WORD   0x0B0B
typedef enum
{
    RX_API_IS_SET = 0x01,
    RX_API_IS_PGM = 0x02,
    Rx_API_IS_GET = 0x07,

}rxApiType;

//extern uint16_t CPU1RxBuf[CPU1_RXBUFSIZE];
////uint8_t CPU2RxBuf[CPU2_RXBUFSIZE];
//extern uint16_t CPU2RxBuf[CPU2_RXBUFSIZE];
uint16_t TxBuf[24];

void grl_tcp_recv_data_pop(struct tcp_pcb *tpcb, struct pbuf *p);
void grlAsciitoHexConvertion(u8_t *aSrcRxBuf, uint32_t *aDestRxBuf, u16_t aPayLoadLength);
void tcpRxdataHandler(u8_t *atcpRxBuf);
void DecodeSystemDetails(uint16_t * aBuffer);
void sys_id();
void TxIPAddr();
void delay();
uint8_t* sys_id_formation(uint8_t * aRxBuffer);

err_t grlTcpDataTx(uint16_t * aTxBuf, uint16_t aDataLength);


typedef enum
{
    CONNECTIVITY_MANAGER = 0,
    CPU1 = 1,
    CPU2 = 2,
}Ti_CORE;

typedef enum
{
    Ti_FW_UPDATE = 6,
}fw_update_cmd;

#endif /* GRLPDCOMM_H_ */
