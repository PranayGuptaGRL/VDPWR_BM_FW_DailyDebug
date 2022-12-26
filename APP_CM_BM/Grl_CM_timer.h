/*
 * Grl_CM_timer.h
 *
 *  Created on: 08-Apr-2022
 *      Author: prana
 */

#ifndef GRL_CM_TIMER_H_
#define GRL_CM_TIMER_H_

#include "driverlib_cm.h"
#include "cm.h"
#include "GrlPDComm.h"
extern struct netif g_sNetIF;
extern uint8_t is_dhcp;
extern bool isIPSent;
extern uint32_t IP_address;
extern uint16_t CPU2_MODE;
extern uint32_t IPAddr_dhcp;
uint8_t gTimer0Var;
uint8_t gTimer1Var;
typedef enum
{
    TIMER0 = 0,
    TIMER1,
}MsgTimers_t;

void MsgTimerStart(uint32_t, uint8_t, uint8_t );

__interrupt void MsgTimer1ExpiryHandler( void );
__interrupt void MsgTimer0ExpiryHandler( void );
void MsgTimerStop(uint8_t aMsgTimer);


 void setupMsgTimer1( void );
 void setupMsgTimer0( void );
 void INIT_CPU_TIMERS(void);
 void Config_CPU_TIMER(uint32_t cpuTimer, uint32_t period);

#endif /* GRL_CM_TIMER_H_ */
