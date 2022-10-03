/*
 * Includes.h
 *
 *  Created on: 09-Mar-2022
 *      Author: prana
 */

#ifndef INCLUDES_H_
#define INCLUDES_H_

#include <stdlib.h>

#include "driverlib.h"
#include "device.h"
#include "Peripheral.h"
#include "Controller.h"
#include "stdio.h"
#include "string.h"

#define IPC_CMD_READ_MEM   0x1001

extern uint32_t gVar[286];
extern uint8_t CmDataRxbuf[286];
extern uint8_t CmDataTxbuf[286];
extern uint8_t gRS485RxBuf[FW_PL_LENGTH];
extern uint8_t gPollingBuf[36];
extern uint8_t gFWOpcdeBuf[FW_PL_LENGTH];

volatile extern uint8_t gindex;
volatile extern uint16_t gRS485RxIntrCount;
volatile extern uint16_t gRS485ByteCount;
volatile extern uint16_t gRS485ReInitDevCount;
volatile extern uint8_t gErrRetryCount;
volatile extern uint16_t gFWOpcodeBufLength;

#endif /* INCLUDES_H_ */
