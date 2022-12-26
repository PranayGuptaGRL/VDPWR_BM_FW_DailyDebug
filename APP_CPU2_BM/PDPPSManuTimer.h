/*
 * PDPPSManuTimer.h
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#ifndef PDPPSMANUTIMER_H_
#define PDPPSMANUTIMER_H_
#include "PDPPSManuIncludes.h"

#define TIMER_1             (unsigned short)0x0001
#define TIMER_2             (unsigned short)0x0002
#define TIMER_3             (unsigned short)0x0004
#define TIMER_16            (unsigned short)0x8000


/* Global variable for reference */
static unsigned short gTimer;


void MsgTimerStart(uint32_t, uint8_t, uint8_t );

__interrupt void MsgTimer1ExpiryHandler( void );
__interrupt void MsgTimer0ExpiryHandler( void );
void MsgTimerStop(uint8_t aMsgTimer);

 void setupMsgTimer1( void );
 void setupMsgTimer0( void );
 void INIT_CPU_TIMERS(void);
 void Config_CPU_TIMER(uint32_t cpuTimer, float freq, float period);

#endif /* PDPPSMANUTIMER_H_ */
