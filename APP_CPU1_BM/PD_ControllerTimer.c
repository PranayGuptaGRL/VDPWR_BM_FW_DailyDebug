/*
 * PDPPSManuTimer.c
 *
 *  Created on: 10-Nov-2021
 *      Author: GRL
 */

#include <PD_ControllerTimer.h>

#include "ControllerMain.h"
#include "PD_ControllerStruct.h"
#include "math.h"
#ifndef RTOSTIMER
float Vout;
float temp;
float ADCReading;
//Initialize cpu timers

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
void Config_CPU_TIMER(uint32_t cpuTimer, float freq, float period)
{
//    uint32_t temp;
//    // Initialize timer period:
//    temp = (uint32_t) (freq / 1000000 * period);
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
#endif

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

        aTimerVal *= mS;

        ConfigCPUTimerCount( CPUTIMER0_BASE,  DEVICE_SYSCLK_FREQ,  aTimerVal);
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
        aTimerVal *= mS;

        ConfigCPUTimerCount( CPUTIMER1_BASE,  DEVICE_SYSCLK_FREQ,  aTimerVal);
        CPUTimer_enableInterrupt(CPUTIMER1_BASE);
        CPUTimer_startTimer(CPUTIMER1_BASE);
#endif
        break;
    case TIMER2:

        gTimer1Var = aTimerVar;
        aTimerVal *= mS;

        ConfigCPUTimerCount( CPUTIMER2_BASE,  DEVICE_SYSCLK_FREQ,  aTimerVal);
        CPUTimer_enableInterrupt(CPUTIMER2_BASE);
        CPUTimer_startTimer(CPUTIMER2_BASE);

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
            Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
#endif
            break;

        case TIMER1:

#ifdef RTOSTIMER
                StopCpuTimer(&CpuTimer1);
#else
                CPUTimer_disableInterrupt(CPUTIMER1_BASE);
#endif

            break;
        case TIMER2:
            CPUTimer_disableInterrupt(CPUTIMER2_BASE);

            break;
    }
}

void ledToggle(uint8_t aLED)
{
    static uint32_t counter = 0;

    counter++;
    GPIO_writePin(aLED, counter & 1);

}

//gControlStruct * gControlStruct_t;
//uint8_t lindex = 0;
void ADC_Data_read()
{
    uint8_t i = 0;
    ADCReading = 0;
    Vout = 0;
    temp = 0;
//    HeatSinkTemp = 0;

//    ADC_forceMultipleSOC(ADCB_BASE, ADC_FORCE_SOC2 );

    ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER2 );

    //
    // Wait for ADCD to complete, then acknowledge flag
    //
    while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2) == false)
    {
    }
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);
    for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
    {
        adcHeatSinkTemp2SensorDataBuffer[i] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2 );
        DEVICE_DELAY_US(100);
        ADCReading += adcHeatSinkTemp2SensorDataBuffer[i];
    }

    ADCReading /= ((float) RESULTS_BUFFER_SIZE);

//    float tempC = 0.0, tempF = 0.0;

    Vout = (3.0 / 4096) * ADCReading ;


    temp = ( (Vout - 1.875928) / (-12.0432) ) * 1000.00;

//    temp = (1.8663 - 3.3*HeatSinkTemp)/0.01169;
//    temp /= 2;
//    temp = ( (temp - 1.8528) / (-11.79) ) * 1000.00;

//    tempC = ( -1481.96 + sqrt( ( 2.1962 * pow(10,6) ) + ( (1.8639 - temp) / 3.88 * pow(10,-6) ) ) );

//    tempC = (1.8663 - temp) / 0.01169;
//    tempF = 1.8 * tempC + 32.0;
//    HeatSinkTemp = temp;

//    HeatSinkTemp = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2 );

}
__interrupt void MsgTimer0ExpiryHandler( void )
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    MsgTimerStop(TIMER0);

    switch(gTimer0Var)
    {
        case Timer0_Init_Polling_CmdHandler:

//            if( (gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled == true)
//                        && (gControlStruct_t->gTestHandle_t.gTestControl_t.TestCard_PowerState == TC_CONNECTED) )
            {
                GetPDCStatus(Timer0_Init_Polling_CmdHandler,0x01); //sending commands to test cards which are connected.
                ADC_Data_read();
                MsgTimerStart(55,Timer0_Polling_Expiry_Handler,TIMER0); //Start polling expire timer.

            }
//            else
//            {
//                gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;
//                MsgTimerStart(55,Timer0_Polling_Expiry_Handler,TIMER0); //Start polling expire timer.
//            }

            break;

        case Timer0_Start_Polling_Handler:

            break;

        case Timer0_Polling_Expiry_Handler:

            if(gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag == RUNTIME_API_RECEIVED) //check if there is any run time command in queue (True -- make RS485 bus idle)
            {
                gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus = RS485_IDLE;
                MsgTimerStart(55,Timer0_Polling_Expiry_Handler,TIMER0); //Start polling expire timer.

            }
            else if( (gControlStruct_t->gTestHandle_t.gTestControl_t.gRuntimeCmd_Flag == RUNTIME_API_HANDLED)
                        && (gControlStruct_t->gTestHandle_t.gTestControl_t.TestCard_PowerState == TC_CONNECTED)
                            && (gControlStruct_t->gTestHandle_t.gTestControl_t.gIsPollingEnabled == true) )
            {

                MsgTimerStart(2,Timer0_Init_Polling_CmdHandler,TIMER0); //Start polling expire timer.
            }

            break;
        case Timer0_TempDataRead:

            ledToggle(DBG_BLUE_LED_GPIO37);

            ADC_Data_read();

            MsgTimerStart(2000,Timer0_TempDataRead,TIMER0);

            break;
        case Timer0_Read_Expiry_Timer:

            //Pranay,20May'22, Resetting RS485 Bus incase if there is no response from TC
            gRS485RxIntrCount = 0;
            gRS485ByteCount = 255;
            gRS485ReInitDevCount = 0;

            RS485DeviceInit();
            SPI0ReConfig();

            gRS485RxBuf[0] = 0xAA;
            gRS485RxBuf[1] = 0xAA;
            gRS485RxBuf[2] = 0xAA;
            gRS485RxBuf[3] = 0xAA;

            IPC_sendCommand(IPC_CPU1_L_CM_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                              IPC_CMD_READ_MEM, (uint32_t)gRS485RxBuf, 0x04);

            IPC_waitForAck(IPC_CPU1_L_CM_R, IPC_FLAG0);
            break;
    }

}
__interrupt void MsgTimer1ExpiryHandler( void )
{
    gControlStruct_t = (gControlStruct *)GetStructPtr();

    MsgTimerStop(TIMER1);

    switch(gTimer1Var)
    {
#ifdef POLLING

    case Timer1_RunTime_CmdHandler:

        if(gControlStruct_t->gTestHandle_t.gTestControl_t.gRS485BusStatus == RS485_IDLE)
        {
            grlIPCDataRxHandler(CmDataRxbuf);
        }
        else
        {
            MsgTimerStart(20,Timer1_RunTime_CmdHandler, TIMER1); //if the index value reach max value reset the index and start the polling timer.
        }
        break;
#endif
    case Timer1_RS485DevInit:

        gRS485RxIntrCount = 0;
        gRS485ByteCount = 255;
        gRS485ReInitDevCount = 0;
        RS485DeviceInit();

        break;
    }
}

__interrupt void MsgTimer2ExpiryHandler( void )
{
    MsgTimerStop(TIMER2);

    switch(gTimer2Var)
    {
        case Timer2_TempDataRead:

            ledToggle(DBG_BLUE_LED_GPIO37);

            ADC_Data_read();

            MsgTimerStart(2000,Timer2_TempDataRead,TIMER2);
        break;
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
    Interrupt_register(INT_TIMER0, &MsgTimer0ExpiryHandler);

//     To ensure precise timing, use write-only instructions to write to the
//     entire register. Therefore, if any of the configuration bits are changed
//     in Config_CPU_TIMER and INIT_CPU_TIMERS, the below settings must also
//     be updated.

    Config_CPU_TIMER(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000000);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
//     Enables CPU int1, int13, and int14 which are connected to CPU-Timer 0,
//     CPU-Timer 1, and CPU-Timer 2 respectively.
//     Enable TINT0 in the PIE: Group 1 interrupt 7

    Interrupt_enable(INT_TIMER0);
#endif
}
//-------------------------------------------------------------------------------------------------
void setupMsgTimer2( void )
{
    Interrupt_register(INT_TIMER2, &MsgTimer2ExpiryHandler);
//    //
//     // To ensure precise timing, use write-only instructions to write to the
//     // entire register. Therefore, if any of the configuration bits are changed
//     // in Config_CPU_TIMER and INIT_CPU_TIMERS, the below settings must also
//     // be updated.
//     //
    Config_CPU_TIMER(CPUTIMER2_BASE, DEVICE_SYSCLK_FREQ, 1000000);
//
    CPUTimer_enableInterrupt(CPUTIMER2_BASE);
//
//    // Enables CPU int1, int13, and int14 which are connected to CPU-Timer 0,
//    // CPU-Timer 1, and CPU-Timer 2 respectively.
//    // Enable TINT0 in the PIE: Group 1 interrupt 7
//    //
    Interrupt_enable(INT_TIMER2);
}
void setupMsgTimer1( void )
{
#ifdef RTOSTIMER

    /**Timer 1*/
    EALLOW;

    PieVectTable.TIMER1_INT = &MsgTimer1ExpiryHandler;

    EDIS;

    IER |= M_INT13;

#else
    Interrupt_register(INT_TIMER1, &MsgTimer1ExpiryHandler);
//    //
//     // To ensure precise timing, use write-only instructions to write to the
//     // entire register. Therefore, if any of the configuration bits are changed
//     // in Config_CPU_TIMER and INIT_CPU_TIMERS, the below settings must also
//     // be updated.
//     //
    Config_CPU_TIMER(CPUTIMER1_BASE, DEVICE_SYSCLK_FREQ, 1000000);
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
