/*
 * PDManufacturerTimer.h
 *
 *  Created on: Jun 26, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERTIMER_H_
#define PDMANUFACTURERTIMER_H_

#define DEFAULT_TIMER_VALUE 15

CyU3PTimer gMsgTimer0;
CyU3PTimer gMsgTimer1;
CyU3PTimer gMsgTimer2;
CyU3PTimer gMsgTimer3;
CyU3PTimer gMsgTimer4;

CyU3PTimer *pMsgTimer;
char *pTimername;

CyU3PReturnStatus_t  MsgTimerStart(uint16_t, uint8_t);
uint32_t GetMsgTimer0Count(uint16_t);
uint32_t GetMsgTimer1Count(uint16_t);
uint32_t GetMsgTimer2Count(uint16_t);
uint32_t GetMsgTimer3Count(uint16_t );
uint32_t GetMsgTimer4Count(uint16_t );

void SetCurrentRunningTimer(uint16_t, uint32_t, uint8_t);
uint16_t GetCurrentRunningTimer(uint8_t );
CyU3PReturnStatus_t MsgTimerStop(uint8_t);
CyU3PReturnStatus_t MsgTimerModify(uint32_t, uint8_t);
void MsgTimer0Handling (uint32_t);
void MsgTimer1Handling (uint32_t);
CyU3PReturnStatus_t MsgTimer0ExpiredHandle (uint8_t);
CyU3PReturnStatus_t MsgTimer1ExpiredHandle (uint8_t);
CyU3PReturnStatus_t MsgTimer2ExpiredHandle (uint8_t );
CyU3PReturnStatus_t MsgTimer3ExpiredHandle (uint8_t );
CyU3PReturnStatus_t MsgTimer4ExpiredHandle (uint8_t );

void updateLEDToggleStatus(uint8_t );

#endif /* PDMANUFACTURERTIMER_H_ */
