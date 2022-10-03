/*
 * Controller.h
 *
 *  Created on: 09-Mar-2022
 *      Author: prana
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "PD_ControllerFlash.h"
#include "PD_ControllerTimer.h"
__interrupt void CM_CPU1_IPC_ISR1();
__interrupt void CPU2_CPU1_IPC_ISR1();

__interrupt void gpioInterruptCbHandler();
__interrupt void TesterCardDetection();
void SetDataCmd(uint8_t * aRxBuffer);
void GetDataCmdHandler(uint8_t * aRxBuffer);
void grlRs485Rx_CmTx_DataHandler();
void grlIPCDataRxHandler(uint8_t * aRxBuffer);
void GetPDCStatus(uint8_t lDataType, uint8_t lPortID);
void RecvDataHandler(uint8_t * aRxBuffer);
__interrupt void sciaRXFIFOISR(void);
__interrupt void sciaTXFIFOISR(void);

void InitFWInstances();

typedef enum
{
    TI_CM   = 0,
    TI_CPU1_CONTROLCARD = 1,
    TI_CPU2_PPS = 2
}Ti_Cores;

typedef enum
{
    RX_API_IS_SET = 0x01,
    RX_API_IS_PGM = 0x02,
    Rx_API_IS_GET = 0x07,

}rxApiType;

#define  PPS_PGM_MODE    0xDD
#define  PPS_FW_UPD      0xEE
#define CM_PGM_MODE      0xE1

#endif /* CONTROLLER_H_ */
