/*
 * PDPPSManuTimer.c
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#include "PDPPSManuTimer.h"
#include "PDPPSManufacturerMain.h"
#include "FRAM.h"
gPPSStruct *pStructPtr;
static uint32_t gLedBlinkCount;
volatile uint8_t gOCP_check_count;//Variable that checks how many times in a single request current drawing has been crossed the OCP set limit


void Config_CPU_TIMER(uint32_t cpuTimer, float freq, float period)
{
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

void MsgTimerStop(uint8_t aMsgTimer)
{
    switch(aMsgTimer)
    {
        case 0:

#ifdef RTOSTIMER
            StopCpuTimer(&CpuTimer0);
#else
            CPUTimer_disableInterrupt(CPUTIMER0_BASE);
            Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
#endif
            break;

        case 1:

#ifdef RTOSTIMER
                StopCpuTimer(&CpuTimer1);
#else
                CPUTimer_disableInterrupt(CPUTIMER1_BASE);
#endif

            break;
    }
}

void FRAM_DataConfig()
{
    pStructPtr->gMiscStruct.gFramByteCount = 0;
    pStructPtr->gMiscStruct.gFramCurrentByteAddress = 0;
    memset(FRAM_Misc_buf, 0x00, 16);

//    CyFxUsbI2cTransfer(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);
    FRAM_I2C_read(8, pStructPtr->gMiscStruct.gFramCurrentByteAddress, FRAM_Misc_buf);

    pStructPtr->gMiscStruct.gFramByteCount = ((FRAM_Misc_buf[2]) | (FRAM_Misc_buf[3]<<8));
//    EloadFRAM_DataWrite();
}

void EloadFRAM_DataWrite()
{
//    gFunctStruct_t = (gFunctStruct *)GetStructPtr();

    memset(FRAM_Misc_buf, 0x00, 8);

//    CyFxUsbI2cTransfer(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);     /** Reading from FRAM SLAVE ADRRESS 8bytes of data in every iteration*/
    FRAM_I2C_read(8, pStructPtr->gMiscStruct.gFramCurrentByteAddress, FRAM_Misc_buf);
    PPSCalibDataRxHandler(FRAM_Misc_buf);

    if(pStructPtr->gMiscStruct.gFramCurrentByteAddress <= pStructPtr->gMiscStruct.gFramByteCount)
    {
//        Config_Eload(Eload_BootUpSync, gI2CRxBuf);
        MsgTimerStart(30000, I2C_FRAM_CALIB_DATA_HANDLER, TIMER0);//
        pStructPtr->gMiscStruct.gFramCurrentByteAddress += 8;

//        if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevsion_index == 1)
//            gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision = gI2CRxBuf[2]; //get the function card board revision for CC switch enable/disable.

//        gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevsion_index++;
    }
    else
    {
//      TxPPSCalibDataCalcAPI();/**After writing total Calib data to PPS, Send an API to make TI PPS start calculating Slope and gradients*/
//        RaSelection(0); /** By default their will be no Ra Assertion & and which will work for normal type-c cable*/
//        PD_Attach_Detach(1); //attach command after power on of function card.
        pStructPtr->gMiscStruct.gFramCurrentByteAddress = 0;

        MsgTimerStart(10000, CALIB_DATA_POP, TIMER0);

    }
}
void ledToggle(uint8_t aLED)
{
    static uint32_t counter = 0;

    counter++;
    GPIO_writePin(aLED, counter & 1);

}

/*Venkat 14'NOV 2022 , Function that checks for OCP and triggers respective GPIO to Fx3*/
void OCPCheckMechanism()
{
    if( gOCP_check_count++ >= OCP_LIMIT_BREACH) //check for constant 10mS OCP limit breach,200uS * 50times = 10mS
    {
        OCP_TRIGGER(GPIO_RESET);
        pStructPtr->gVbusHandler.gOCP_trigger_indicator = true;
        gOCP_check_count = 0;
    }
}

__interrupt void MsgTimer0ExpiryHandler( void )
{

    MsgTimerStop(TIMER0);
    switch(gTimer0Var)
    {
    case ADC_DATA_READ:
        Task_ADC_DataRead();

        if(gLedBlinkCount++ >= LED_BLINK_ITER_COUNT)//Venkat,15NOv'22, To make the LED blinking visible, so toggling it for every 500mS
        {
            GPIO_togglePin(38);
            gLedBlinkCount = 0;
        }
        MsgTimerStart(ADC_CONVERSION_TIME, ADC_DATA_READ, TIMER0);

        break;
    case I2C_FRAM_BYTECOUNT_DECODE:

        FRAM_DataConfig();

        MsgTimerStart(10000, I2C_FRAM_CALIB_DATA_HANDLER, TIMER0);//

        break;
    case I2C_FRAM_CALIB_DATA_HANDLER:

         EloadFRAM_DataWrite();

//        FRAM_I2C_read(aRxBuffer[3], (aRxBuffer[4] | aRxBuffer[5] << 8), CPU2DataRxbuf);

        break;
    case CALIB_DATA_POP:

//        memcpy(&PPSCalibRxData[gCalibRxBufIndex], &aBuffer[2], 8);

        GetDataFromCalibBuf();
        MsgTimerStart(10000, ADC_DATA_READ, TIMER0);//


        break;

    }

}

__interrupt void MsgTimer1ExpiryHandler( void )
{

    MsgTimerStop(TIMER1);

    switch(gTimer1Var)
    {
    case TIMER1_VBUS_INC://Used incase of Calibration
        gCurrentVbusSetting += VBUS_STEP_INC_DEC_SIZE;

          if(gCurrentVbusSetting < gRxVbusVoltage)
          {
              SetDACVoltage(gCurrentVbusSetting);
              MsgTimerStart(100, TIMER1_VBUS_INC, TIMER1);//100uS
          }
          else if(gCurrentVbusSetting > gRxVbusVoltage)
          {
              SetDACVoltage(gRxVbusVoltage);//Final voltage setting after step wise incrementing
              pStructPtr->gMiscStruct.gIsRxVbusSet = true;
          }

          break;

    case TIMER1_VBUS_DEC://Used incase of calibration

        gCurrentVbusSetting -= VBUS_STEP_INC_DEC_SIZE;

        if(gCurrentVbusSetting <= gRxVbusVoltage)
        {
            SetDACVoltage(gRxVbusVoltage);//Final voltage setting after setp wise decrementing
            pStructPtr->gMiscStruct.gIsRxVbusSet = true;

        }
        else
        {
            SetDACVoltage(gCurrentVbusSetting);
            MsgTimerStart(100, TIMER1_VBUS_DEC, TIMER1);//100uS
        }

        break;
    case TIMER1_VBUS_ZERO:
        gCurrentVbusSetting -= VBUS_STEP_INC_DEC_SIZE;

        SetDACVoltage(gCurrentVbusSetting);
        gLastDacValueWritten = gCurrentVbusSetting;

        if(gCurrentVbusSetting <= VBUS_5V)
        {
            DEVICE_DELAY_US(5);//5us delay for DAC to act
            HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
            if( gBootTime )
                gBootTime = false;
        }
        else
            MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_ZERO, TIMER1);//100uS

        break;

    case TIMER1_BOOTUP_VBUSHANDLER:
        gBootTime = true;
        HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
        SetVbustoZero();

        break;

    case TIMER1_VBUS_FB_TRIGGER:

        if( (pStructPtr->gMiscStruct.gIsVfbEnabled) && (pStructPtr->gMiscStruct.gIsRxVbusSet) && ( gI2CCmdStatus == I2C_CMD_HANDLED) )
        {
              if( abs(pStructPtr->gVbusHandler.gTypeCendVbusVoltage - gRxVbusVoltage) > VBUS_TOLR_VALUE )
              {
                  VfbHandler();
                  if( !pStructPtr->gVbusHandler.gOCP_trigger_indicator )
                  {
                      if(pStructPtr->gVbusHandler.gADCLiveVbusCurrent + OCP_TOLERANCE >= pStructPtr->gVbusHandler.gRxVbus_i_OCPLimitVal )  //Ignoring OCP check if current level is less than  the expected OCP Limit
                      {
                          OCPCheckMechanism();
                      }
                      else
                      {
                          gOCP_check_count = 0;
                      }
                  }
                  else
                  {
                        OCP_TRIGGER(GPIO_SET);
                  }


                  MsgTimerStart(VFB_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//2000uS

              }
              else
              {
                  if( !pStructPtr->gVbusHandler.gOCP_trigger_indicator )
                  {
                      if(pStructPtr->gVbusHandler.gADCLiveVbusCurrent + OCP_TOLERANCE >= pStructPtr->gVbusHandler.gRxVbus_i_OCPLimitVal )  //Ignoring OCP check if current level is less than  the expected OCP Limit
                      {
                          OCPCheckMechanism();
                      }
                      else
                      {
                          gOCP_check_count = 0;
                      }
                  }
                  else
                  {
                        OCP_TRIGGER(GPIO_SET);
                  }

                  MsgTimerStart(VFB_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//2000uS

              }
        }
        else
        {
           //Do nothing if Vfb not enabled or any API command is received or if received Vbus is not yet set
        }
        break;
#if 0/**Pranay,14Dec'22, Not using these cases anymore so commented out*/
    case TIMER1_VBUS_DAC_STEP_INC:
        pStructPtr->gVbusHandler.gReqVbusStartingDACCount -= VBUS_INCDEC_STEPSIZE;

        DACSetShadowValue(pStructPtr->gVbusHandler.gReqVbusStartingDACCount);
        DEVICE_DELAY_US(2);
        HANDLE_VBUS_CTRL_SWITCH(TURN_ON);

        if(pStructPtr->gVbusHandler.gReqVbusStartingDACCount > pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount)
        {
            MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_DAC_STEP_INC, TIMER1);//100uS
        }
        else //if(pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount >= pStructPtr->gVbusHandler.gReqVbusStartingDACCount)
        {
            gLastDacValueWritten = pStructPtr->gVbusHandler.gReqVbusStartingDACCount;
            if(pStructPtr->gMiscStruct.gIsVfbEnabled)
            {
                gI2CCmdStatus = I2C_CMD_HANDLED;
                pStructPtr->gMiscStruct.gIsRxVbusSet = true;
//                MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//100uS
            }
            else
            {
                DACSetShadowValue(pStructPtr->gVbusHandler.gReqVbusFinalDACCount);
                DEVICE_DELAY_US(2);
                HANDLE_VBUS_CTRL_SWITCH(TURN_ON);
                pStructPtr->gMiscStruct.gIsRxVbusSet = true;
                gLastDacValueWritten = pStructPtr->gVbusHandler.gReqVbusFinalDACCount;

            }

        }
        break;
    case TIMER1_VBUS_DAC_STEP_DEC:

        pStructPtr->gVbusHandler.gReqVbusStartingDACCount += VBUS_INCDEC_STEPSIZE;
        DACSetShadowValue(pStructPtr->gVbusHandler.gReqVbusStartingDACCount);
        DEVICE_DELAY_US(2);
        HANDLE_VBUS_CTRL_SWITCH(TURN_ON);

        if(pStructPtr->gVbusHandler.gReqVbusStartingDACCount < pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount)
        {
            MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_DAC_STEP_DEC, TIMER1);//100uS
        }
        else
        {
            gLastDacValueWritten = pStructPtr->gVbusHandler.gReqVbusStartingDACCount;

            if(pStructPtr->gMiscStruct.gIsVfbEnabled)
            {
                gI2CCmdStatus = I2C_CMD_HANDLED;
                pStructPtr->gMiscStruct.gIsRxVbusSet = true;

//                MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//100uS
            }
            else
            {
                DACSetShadowValue(pStructPtr->gVbusHandler.gReqVbusFinalDACCount);
                DEVICE_DELAY_US(2);
                HANDLE_VBUS_CTRL_SWITCH(TURN_ON);
                pStructPtr->gMiscStruct.gIsRxVbusSet = true;
                gLastDacValueWritten = pStructPtr->gVbusHandler.gReqVbusFinalDACCount;
            }
        }
        break;
#endif
    case TIMER1_VBUS_VOL_STEP_INC:

        gCurrentVbusSetting += VBUS_STEP_INC_DEC_SIZE;

          if(gCurrentVbusSetting < pStructPtr->gVbusHandler.gPresentRxVbusCount)
          {
              gLastDacValueWritten = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, gCurrentVbusSetting);
//              DACSetShadowValue( SetLiveValue(PPS_DAC_VBUS_VOLTAGE, gCurrentVbusSetting) );
              DACSetShadowValue(gLastDacValueWritten);
              HANDLE_VBUS_CTRL_SWITCH(TURN_ON);

              MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_VOL_STEP_INC, TIMER1);//10uS
          }
          else if(gCurrentVbusSetting > pStructPtr->gVbusHandler.gPresentRxVbusCount)//Final voltage setting
          {
              gLastDacValueWritten =  SetLiveValue(PPS_DAC_VBUS_VOLTAGE, pStructPtr->gVbusHandler.gPresentRxVbusCount);
//              DACSetShadowValue( SetLiveValue(PPS_DAC_VBUS_VOLTAGE, gRxVbusVoltage) );

              DACSetShadowValue( gLastDacValueWritten );
              HANDLE_VBUS_CTRL_SWITCH(TURN_ON);
              gI2CCmdStatus = I2C_CMD_HANDLED;
              pStructPtr->gMiscStruct.gIsRxVbusSet = true;

//              DEVICE_DELAY_US(VBUS_INCDEC_STEP_TIME);
//
//              TriggerVbusVfb();

              MsgTimerStart(VFB_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//200uS
          }

        break;
    case TIMER1_VBUS_VOL_STEP_DEC:

        gCurrentVbusSetting -= VBUS_STEP_INC_DEC_SIZE;

            if(gCurrentVbusSetting <= pStructPtr->gVbusHandler.gPresentRxVbusCount)//Final voltage setting
            {
                gLastDacValueWritten = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, pStructPtr->gVbusHandler.gPresentRxVbusCount);
                DACSetShadowValue( gLastDacValueWritten );
                HANDLE_VBUS_CTRL_SWITCH(TURN_ON);

                gI2CCmdStatus = I2C_CMD_HANDLED;
                pStructPtr->gMiscStruct.gIsRxVbusSet = true;

                MsgTimerStart(VFB_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);//200uS

            }
            else
            {
                gLastDacValueWritten =  SetLiveValue(PPS_DAC_VBUS_VOLTAGE, gCurrentVbusSetting);
//                DACSetShadowValue( SetLiveValue(PPS_DAC_VBUS_VOLTAGE, gCurrentVbusSetting) );
                DACSetShadowValue( gLastDacValueWritten );
                HANDLE_VBUS_CTRL_SWITCH(TURN_ON);

                MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_VOL_STEP_DEC, TIMER1);//200uS
            }

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

        ConfigCPUTimerCount( CPUTIMER1_BASE,  DEVICE_SYSCLK_FREQ,  aTimerVal);
        CPUTimer_enableInterrupt(CPUTIMER1_BASE);
        CPUTimer_startTimer(CPUTIMER1_BASE);
#endif
        break;

    }

}

