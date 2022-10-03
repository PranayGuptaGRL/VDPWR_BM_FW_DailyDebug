/*
 * PDPPSManuTimer.h
 *
 *  Created on: 10-Nov-2021
 *      Author: GRL
 */

#ifndef PD_CONTROLLERTIMER_H_
#define PD_CONTROLLERTIMER_H_

#include "Includes.h"
#include "Controller.h"

void MsgTimerStart(uint32_t, uint8_t, uint8_t );

__interrupt void MsgTimer1ExpiryHandler( void );
__interrupt void MsgTimer0ExpiryHandler( void );
void MsgTimerStop(uint8_t aMsgTimer);


 void setupMsgTimer1( void );
 void setupMsgTimer0( void );
 void setupMsgTimer2( void );

 void INIT_CPU_TIMERS(void);
 void Config_CPU_TIMER(uint32_t cpuTimer, float freq, float period);
 void ADC_Data_read();

#endif /* PD_CONTROLLERTIMER_H_ */
