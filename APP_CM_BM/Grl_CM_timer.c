/*
 * Grl_CM_timer.c
 *
 *  Created on: 08-Apr-2022
 *      Author: prana
 */
#include <Grl_CM_timer.h>
struct dhcp *dhcp = NULL;
u16_t lDHCPVar ;

void INIT_CPU_TIMERS(void)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    //
    // Reload all counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
    //
    // Reset interrupt counter

}
void ConfigCPUTimerCount(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;
    // Initialize timer period:
    temp = (uint32_t) (freq / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, temp);
}
//
// Config_CPU_TIMER - This function initializes the selected timer to the
// period specified by the "freq" and "period" parameters. The "freq" is
// entered as Hz and the period in uSeconds. The timer is held in the stopped
// state after configuration.
//
void Config_CPU_TIMER(uint32_t cpuTimer, uint32_t period)
{
//    uint32_t temp;
//    temp = (uint32_t) (freq / 1000000 * period);
//    // Initialize timer period:
//    CPUTimer_setPeriod(cpuTimer, temp);
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    CPUTimer_setPreScaler(cpuTimer, 0);

    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);

}

void MsgTimerStart(uint32_t aTimerVal, uint8_t aTimerVar, uint8_t aMsgTimer)
{

    switch(aMsgTimer)
    {

    case TIMER0:

#ifdef RTOSTIMER
        ConfigCpuTimer(&CpuTimer0,
                       configCPU_CLOCK_HZ / 1000000,  // CPU clock in MHz
                       aTimerVal );                    // Timer period in uS
        CpuTimer0Regs.TCR.all = 0x4000;               // Enable interrupt and start timer.

        IER |= M_INT12;
#else
        //
        // Starts CPU-Timer 0, CPU-Timer 1
        //
        gTimer0Var = aTimerVar;

        aTimerVal *= 1000;

        ConfigCPUTimerCount( CPUTIMER0_BASE,  CM_CLK_FREQ,  aTimerVal);
        CPUTimer_enableInterrupt(CPUTIMER0_BASE);
        CPUTimer_startTimer(CPUTIMER0_BASE);
#endif
        break;

    case TIMER1:

#ifdef RTOSTIMER
        ConfigCpuTimer(&CpuTimer1,
                       configCPU_CLOCK_HZ / 1000000,  // CPU clock in MHz
                       aTimerVal );                    // Timer period in uS
        CpuTimer1Regs.TCR.all = 0x4000;               // Enable interrupt and start timer.

        IER |= M_INT13;

#else        //
        // Starts CPU-Timer 0, CPU-Timer 1
        //
        gTimer1Var = aTimerVar;
        aTimerVal *= 1000;

        ConfigCPUTimerCount( CPUTIMER1_BASE,  CM_CLK_FREQ,  aTimerVal);
        CPUTimer_enableInterrupt(CPUTIMER1_BASE);
        CPUTimer_startTimer(CPUTIMER1_BASE);
#endif
        break;

    }

}

void MsgTimerStop(uint8_t aMsgTimer)
{
    switch(aMsgTimer)
    {
        case TIMER0:

#ifdef RTOSTIMER
            StopCpuTimer(&CpuTimer0);
#else
            CPUTimer_disableInterrupt(CPUTIMER0_BASE);
//            Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
#endif
            break;

        case TIMER1:

#ifdef RTOSTIMER
                StopCpuTimer(&CpuTimer1);
#else
                CPUTimer_disableInterrupt(CPUTIMER1_BASE);
#endif

            break;
    }
}

__interrupt void MsgTimer0ExpiryHandler( void )
{

    MsgTimerStop(0);

    switch(gTimer0Var)
    {
        case 0:

            lDHCPVar = 0;

//            if(!netif_is_link_up(&g_sNetIF));
//            {
//                etharp_request(&g_sNetIF, &IPAddr_dhcp);
//            }
//
//            dhcp_supplied_address(&g_sNetIF);
//            sys_id();
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);

            MsgTimerStart(1000, 1, TIMER0);

        break;
        case 1:
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);

            TxIPAddr();
            MsgTimerStart(1000, 2, TIMER0);

            break;
        case 2:
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);

            dhcp = netif_dhcp_data(&g_sNetIF);

            if( (dhcp->offered_ip_addr.addr != 0)
                    /*&& (g_sNetIF.ip_addr.addr != dhcp->offered_ip_addr.addr)*/ && (!isIPSent) )
            {
                IP_address=dhcp->offered_ip_addr.addr ;
//                sys_id();
//                TxIPAddr();
                isIPSent = 1;
                MsgTimerStart(1000, 3, TIMER0);

            }
            else if( lDHCPVar++ < 20)
            {
    //            lDHCPVar = 0;
               if(!netif_is_link_up(&g_sNetIF));
               {
                   etharp_request(&g_sNetIF, &IPAddr_dhcp);
               }

               dhcp_supplied_address(&g_sNetIF);

               MsgTimerStart(1000, 2, TIMER0);
            }
            else
            {
                MsgTimerStart(1000, 3, TIMER0);
                GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);
            }
            break;
        case 3:
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);


//            sys_id();
            MsgTimerStart(1000, 4, TIMER0);
            break;
        case 4:
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);


            TxIPAddr();
            MsgTimerStart(1000, 5, TIMER0);

            break;

        case 5:
//            togglegpio(36);
            GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);


            dhcp_supplied_address(&g_sNetIF);

            MsgTimerStart(2000, 5, TIMER0);

            break;
    }

}
__interrupt void MsgTimer1ExpiryHandler( void )
{

    MsgTimerStop(1);

//    switch()
    {


    }
}

void setupMsgTimer0( void )
{
#ifdef RTOSTIMER

    /**Timer 0*/
    EALLOW;

    PieVectTable.TIMER0_INT = &MsgTimer0ExpiryHandler;

    EDIS;

    IER |= M_INT12;
#else
    Interrupt_registerHandler(INT_TIMER0, &MsgTimer0ExpiryHandler);

//     To ensure precise timing, use write-only instructions to write to the
//     entire register. Therefore, if any of the configuration bits are changed
//     in Config_CPU_TIMER and INIT_CPU_TIMERS, the below settings must also
//     be updated.

    Config_CPU_TIMER(CPUTIMER0_BASE, 1000000);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
//     Enables CPU int1, int13, and int14 which are connected to CPU-Timer 0,
//     CPU-Timer 1, and CPU-Timer 2 respectively.
//     Enable TINT0 in the PIE: Group 1 interrupt 7

    Interrupt_enable(INT_TIMER0);
#endif
}
//-------------------------------------------------------------------------------------------------
void setupMsgTimer1( void )
{
#ifdef RTOSTIMER

    /**Timer 1*/
    EALLOW;

    PieVectTable.TIMER1_INT = &MsgTimer1ExpiryHandler;

    EDIS;

    IER |= M_INT13;

#else
    Interrupt_registerHandler(INT_TIMER1, &MsgTimer1ExpiryHandler);
//    //
//     // To ensure precise timing, use write-only instructions to write to the
//     // entire register. Therefore, if any of the configuration bits are changed
//     // in Config_CPU_TIMER and INIT_CPU_TIMERS, the below settings must also
//     // be updated.
//     //
    Config_CPU_TIMER(CPUTIMER1_BASE, 1000000);
//
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);
//
//    // Enables CPU int1, int13, and int14 which are connected to CPU-Timer 0,
//    // CPU-Timer 1, and CPU-Timer 2 respectively.
//    // Enable TINT0 in the PIE: Group 1 interrupt 7
//    //
    Interrupt_enable(INT_TIMER1);
#endif

}



