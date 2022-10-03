/*******************************************************************************
* File Name: PWMI_C.c
* Version 2.10
*
* Description:
*  This file provides the source code to the API for the PWMI_C
*  component
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PWMI_C.h"

uint8 PWMI_C_initVar = 0u;


/*******************************************************************************
* Function Name: PWMI_C_Init
********************************************************************************
*
* Summary:
*  Initialize/Restore default PWMI_C configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_Init(void)
{

    /* Set values from customizer to CTRL */
    #if (PWMI_C__QUAD == PWMI_C_CONFIG)
        PWMI_C_CONTROL_REG = PWMI_C_CTRL_QUAD_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        PWMI_C_TRIG_CONTROL1_REG  = PWMI_C_QUAD_SIGNALS_MODES;

        /* Set values from customizer to INTR */
        PWMI_C_SetInterruptMode(PWMI_C_QUAD_INTERRUPT_MASK);
        
         /* Set other values */
        PWMI_C_SetCounterMode(PWMI_C_COUNT_DOWN);
        PWMI_C_WritePeriod(PWMI_C_QUAD_PERIOD_INIT_VALUE);
        PWMI_C_WriteCounter(PWMI_C_QUAD_PERIOD_INIT_VALUE);
    #endif  /* (PWMI_C__QUAD == PWMI_C_CONFIG) */

    #if (PWMI_C__TIMER == PWMI_C_CONFIG)
        PWMI_C_CONTROL_REG = PWMI_C_CTRL_TIMER_BASE_CONFIG;
        
        /* Set values from customizer to CTRL1 */
        PWMI_C_TRIG_CONTROL1_REG  = PWMI_C_TIMER_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        PWMI_C_SetInterruptMode(PWMI_C_TC_INTERRUPT_MASK);
        
        /* Set other values from customizer */
        PWMI_C_WritePeriod(PWMI_C_TC_PERIOD_VALUE );

        #if (PWMI_C__COMPARE == PWMI_C_TC_COMP_CAP_MODE)
            PWMI_C_WriteCompare(PWMI_C_TC_COMPARE_VALUE);

            #if (1u == PWMI_C_TC_COMPARE_SWAP)
                PWMI_C_SetCompareSwap(1u);
                PWMI_C_WriteCompareBuf(PWMI_C_TC_COMPARE_BUF_VALUE);
            #endif  /* (1u == PWMI_C_TC_COMPARE_SWAP) */
        #endif  /* (PWMI_C__COMPARE == PWMI_C_TC_COMP_CAP_MODE) */

        /* Initialize counter value */
        #if (PWMI_C_CY_TCPWM_V2 && PWMI_C_TIMER_UPDOWN_CNT_USED && !PWMI_C_CY_TCPWM_4000)
            PWMI_C_WriteCounter(1u);
        #elif(PWMI_C__COUNT_DOWN == PWMI_C_TC_COUNTER_MODE)
            PWMI_C_WriteCounter(PWMI_C_TC_PERIOD_VALUE);
        #else
            PWMI_C_WriteCounter(0u);
        #endif /* (PWMI_C_CY_TCPWM_V2 && PWMI_C_TIMER_UPDOWN_CNT_USED && !PWMI_C_CY_TCPWM_4000) */
    #endif  /* (PWMI_C__TIMER == PWMI_C_CONFIG) */

    #if (PWMI_C__PWM_SEL == PWMI_C_CONFIG)
        PWMI_C_CONTROL_REG = PWMI_C_CTRL_PWM_BASE_CONFIG;

        #if (PWMI_C__PWM_PR == PWMI_C_PWM_MODE)
            PWMI_C_CONTROL_REG |= PWMI_C_CTRL_PWM_RUN_MODE;
            PWMI_C_WriteCounter(PWMI_C_PWM_PR_INIT_VALUE);
        #else
            PWMI_C_CONTROL_REG |= PWMI_C_CTRL_PWM_ALIGN | PWMI_C_CTRL_PWM_KILL_EVENT;
            
            /* Initialize counter value */
            #if (PWMI_C_CY_TCPWM_V2 && PWMI_C_PWM_UPDOWN_CNT_USED && !PWMI_C_CY_TCPWM_4000)
                PWMI_C_WriteCounter(1u);
            #elif (PWMI_C__RIGHT == PWMI_C_PWM_ALIGN)
                PWMI_C_WriteCounter(PWMI_C_PWM_PERIOD_VALUE);
            #else 
                PWMI_C_WriteCounter(0u);
            #endif  /* (PWMI_C_CY_TCPWM_V2 && PWMI_C_PWM_UPDOWN_CNT_USED && !PWMI_C_CY_TCPWM_4000) */
        #endif  /* (PWMI_C__PWM_PR == PWMI_C_PWM_MODE) */

        #if (PWMI_C__PWM_DT == PWMI_C_PWM_MODE)
            PWMI_C_CONTROL_REG |= PWMI_C_CTRL_PWM_DEAD_TIME_CYCLE;
        #endif  /* (PWMI_C__PWM_DT == PWMI_C_PWM_MODE) */

        #if (PWMI_C__PWM == PWMI_C_PWM_MODE)
            PWMI_C_CONTROL_REG |= PWMI_C_CTRL_PWM_PRESCALER;
        #endif  /* (PWMI_C__PWM == PWMI_C_PWM_MODE) */

        /* Set values from customizer to CTRL1 */
        PWMI_C_TRIG_CONTROL1_REG  = PWMI_C_PWM_SIGNALS_MODES;
    
        /* Set values from customizer to INTR */
        PWMI_C_SetInterruptMode(PWMI_C_PWM_INTERRUPT_MASK);

        /* Set values from customizer to CTRL2 */
        #if (PWMI_C__PWM_PR == PWMI_C_PWM_MODE)
            PWMI_C_TRIG_CONTROL2_REG =
                    (PWMI_C_CC_MATCH_NO_CHANGE    |
                    PWMI_C_OVERLOW_NO_CHANGE      |
                    PWMI_C_UNDERFLOW_NO_CHANGE);
        #else
            #if (PWMI_C__LEFT == PWMI_C_PWM_ALIGN)
                PWMI_C_TRIG_CONTROL2_REG = PWMI_C_PWM_MODE_LEFT;
            #endif  /* ( PWMI_C_PWM_LEFT == PWMI_C_PWM_ALIGN) */

            #if (PWMI_C__RIGHT == PWMI_C_PWM_ALIGN)
                PWMI_C_TRIG_CONTROL2_REG = PWMI_C_PWM_MODE_RIGHT;
            #endif  /* ( PWMI_C_PWM_RIGHT == PWMI_C_PWM_ALIGN) */

            #if (PWMI_C__CENTER == PWMI_C_PWM_ALIGN)
                PWMI_C_TRIG_CONTROL2_REG = PWMI_C_PWM_MODE_CENTER;
            #endif  /* ( PWMI_C_PWM_CENTER == PWMI_C_PWM_ALIGN) */

            #if (PWMI_C__ASYMMETRIC == PWMI_C_PWM_ALIGN)
                PWMI_C_TRIG_CONTROL2_REG = PWMI_C_PWM_MODE_ASYM;
            #endif  /* (PWMI_C__ASYMMETRIC == PWMI_C_PWM_ALIGN) */
        #endif  /* (PWMI_C__PWM_PR == PWMI_C_PWM_MODE) */

        /* Set other values from customizer */
        PWMI_C_WritePeriod(PWMI_C_PWM_PERIOD_VALUE );
        PWMI_C_WriteCompare(PWMI_C_PWM_COMPARE_VALUE);

        #if (1u == PWMI_C_PWM_COMPARE_SWAP)
            PWMI_C_SetCompareSwap(1u);
            PWMI_C_WriteCompareBuf(PWMI_C_PWM_COMPARE_BUF_VALUE);
        #endif  /* (1u == PWMI_C_PWM_COMPARE_SWAP) */

        #if (1u == PWMI_C_PWM_PERIOD_SWAP)
            PWMI_C_SetPeriodSwap(1u);
            PWMI_C_WritePeriodBuf(PWMI_C_PWM_PERIOD_BUF_VALUE);
        #endif  /* (1u == PWMI_C_PWM_PERIOD_SWAP) */
    #endif  /* (PWMI_C__PWM_SEL == PWMI_C_CONFIG) */
    
}


/*******************************************************************************
* Function Name: PWMI_C_Enable
********************************************************************************
*
* Summary:
*  Enables the PWMI_C.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_Enable(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    PWMI_C_BLOCK_CONTROL_REG |= PWMI_C_MASK;
    CyExitCriticalSection(enableInterrupts);

    /* Start Timer or PWM if start input is absent */
    #if (PWMI_C__PWM_SEL == PWMI_C_CONFIG)
        #if (0u == PWMI_C_PWM_START_SIGNAL_PRESENT)
            PWMI_C_TriggerCommand(PWMI_C_MASK, PWMI_C_CMD_START);
        #endif /* (0u == PWMI_C_PWM_START_SIGNAL_PRESENT) */
    #endif /* (PWMI_C__PWM_SEL == PWMI_C_CONFIG) */

    #if (PWMI_C__TIMER == PWMI_C_CONFIG)
        #if (0u == PWMI_C_TC_START_SIGNAL_PRESENT)
            PWMI_C_TriggerCommand(PWMI_C_MASK, PWMI_C_CMD_START);
        #endif /* (0u == PWMI_C_TC_START_SIGNAL_PRESENT) */
    #endif /* (PWMI_C__TIMER == PWMI_C_CONFIG) */
    
    #if (PWMI_C__QUAD == PWMI_C_CONFIG)
        #if (0u != PWMI_C_QUAD_AUTO_START)
            PWMI_C_TriggerCommand(PWMI_C_MASK, PWMI_C_CMD_RELOAD);
        #endif /* (0u != PWMI_C_QUAD_AUTO_START) */
    #endif  /* (PWMI_C__QUAD == PWMI_C_CONFIG) */
}


/*******************************************************************************
* Function Name: PWMI_C_Start
********************************************************************************
*
* Summary:
*  Initializes the PWMI_C with default customizer
*  values when called the first time and enables the PWMI_C.
*  For subsequent calls the configuration is left unchanged and the component is
*  just enabled.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  PWMI_C_initVar: global variable is used to indicate initial
*  configuration of this component.  The variable is initialized to zero and set
*  to 1 the first time PWMI_C_Start() is called. This allows
*  enabling/disabling a component without re-initialization in all subsequent
*  calls to the PWMI_C_Start() routine.
*
*******************************************************************************/
void PWMI_C_Start(void)
{
    if (0u == PWMI_C_initVar)
    {
        PWMI_C_Init();
        PWMI_C_initVar = 1u;
    }

    PWMI_C_Enable();
}


/*******************************************************************************
* Function Name: PWMI_C_Stop
********************************************************************************
*
* Summary:
*  Disables the PWMI_C.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_Stop(void)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_BLOCK_CONTROL_REG &= (uint32)~PWMI_C_MASK;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetMode
********************************************************************************
*
* Summary:
*  Sets the operation mode of the PWMI_C. This function is used when
*  configured as a generic PWMI_C and the actual mode of operation is
*  set at runtime. The mode must be set while the component is disabled.
*
* Parameters:
*  mode: Mode for the PWMI_C to operate in
*   Values:
*   - PWMI_C_MODE_TIMER_COMPARE - Timer / Counter with
*                                                 compare capability
*         - PWMI_C_MODE_TIMER_CAPTURE - Timer / Counter with
*                                                 capture capability
*         - PWMI_C_MODE_QUAD - Quadrature decoder
*         - PWMI_C_MODE_PWM - PWM
*         - PWMI_C_MODE_PWM_DT - PWM with dead time
*         - PWMI_C_MODE_PWM_PR - PWM with pseudo random capability
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetMode(uint32 mode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_MODE_MASK;
    PWMI_C_CONTROL_REG |= mode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetQDMode
********************************************************************************
*
* Summary:
*  Sets the the Quadrature Decoder to one of the 3 supported modes.
*  Its functionality is only applicable to Quadrature Decoder operation.
*
* Parameters:
*  qdMode: Quadrature Decoder mode
*   Values:
*         - PWMI_C_MODE_X1 - Counts on phi 1 rising
*         - PWMI_C_MODE_X2 - Counts on both edges of phi1 (2x faster)
*         - PWMI_C_MODE_X4 - Counts on both edges of phi1 and phi2
*                                        (4x faster)
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetQDMode(uint32 qdMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_QUAD_MODE_MASK;
    PWMI_C_CONTROL_REG |= qdMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPrescaler
********************************************************************************
*
* Summary:
*  Sets the prescaler value that is applied to the clock input.  Not applicable
*  to a PWM with the dead time mode or Quadrature Decoder mode.
*
* Parameters:
*  prescaler: Prescaler divider value
*   Values:
*         - PWMI_C_PRESCALE_DIVBY1    - Divide by 1 (no prescaling)
*         - PWMI_C_PRESCALE_DIVBY2    - Divide by 2
*         - PWMI_C_PRESCALE_DIVBY4    - Divide by 4
*         - PWMI_C_PRESCALE_DIVBY8    - Divide by 8
*         - PWMI_C_PRESCALE_DIVBY16   - Divide by 16
*         - PWMI_C_PRESCALE_DIVBY32   - Divide by 32
*         - PWMI_C_PRESCALE_DIVBY64   - Divide by 64
*         - PWMI_C_PRESCALE_DIVBY128  - Divide by 128
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPrescaler(uint32 prescaler)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_PRESCALER_MASK;
    PWMI_C_CONTROL_REG |= prescaler;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetOneShot
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the PWMI_C runs
*  continuously or stops when terminal count is reached.  By default the
*  PWMI_C operates in the continuous mode.
*
* Parameters:
*  oneShotEnable
*   Values:
*     - 0 - Continuous
*     - 1 - Enable One Shot
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetOneShot(uint32 oneShotEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_ONESHOT_MASK;
    PWMI_C_CONTROL_REG |= ((uint32)((oneShotEnable & PWMI_C_1BIT_MASK) <<
                                                               PWMI_C_ONESHOT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPWMMode
********************************************************************************
*
* Summary:
*  Writes the control register that determines what mode of operation the PWM
*  output lines are driven in.  There is a setting for what to do on a
*  comparison match (CC_MATCH), on an overflow (OVERFLOW) and on an underflow
*  (UNDERFLOW).  The value for each of the three must be ORed together to form
*  the mode.
*
* Parameters:
*  modeMask: A combination of three mode settings.  Mask must include a value
*  for each of the three or use one of the preconfigured PWM settings.
*   Values:
*     - CC_MATCH_SET        - Set on comparison match
*     - CC_MATCH_CLEAR      - Clear on comparison match
*     - CC_MATCH_INVERT     - Invert on comparison match
*     - CC_MATCH_NO_CHANGE  - No change on comparison match
*     - OVERLOW_SET         - Set on overflow
*     - OVERLOW_CLEAR       - Clear on  overflow
*     - OVERLOW_INVERT      - Invert on overflow
*     - OVERLOW_NO_CHANGE   - No change on overflow
*     - UNDERFLOW_SET       - Set on underflow
*     - UNDERFLOW_CLEAR     - Clear on underflow
*     - UNDERFLOW_INVERT    - Invert on underflow
*     - UNDERFLOW_NO_CHANGE - No change on underflow
*     - PWM_MODE_LEFT       - Setting for left aligned PWM.  Should be combined
*                             with up counting mode
*     - PWM_MODE_RIGHT      - Setting for right aligned PWM.  Should be combined
*                             with down counting mode
*     - PWM_MODE_CENTER     - Setting for center aligned PWM.  Should be
*                             combined with up/down 0 mode
*     - PWM_MODE_ASYM       - Setting for asymmetric PWM.  Should be combined
*                             with up/down 1 mode
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPWMMode(uint32 modeMask)
{
    PWMI_C_TRIG_CONTROL2_REG = (modeMask & PWMI_C_6BIT_MASK);
}



/*******************************************************************************
* Function Name: PWMI_C_SetPWMSyncKill
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the PWM kill signal (stop input)
*  causes asynchronous or synchronous kill operation.  By default the kill
*  operation is asynchronous.  This functionality is only applicable to the PWM
*  and PWM with dead time modes.
*
*  For Synchronous mode the kill signal disables both the line and line_n
*  signals until the next terminal count.
*
*  For Asynchronous mode the kill signal disables both the line and line_n
*  signals when the kill signal is present.  This mode should only be used
*  when the kill signal (stop input) is configured in the pass through mode
*  (Level sensitive signal).

*
* Parameters:
*  syncKillEnable
*   Values:
*     - 0 - Asynchronous
*     - 1 - Synchronous
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPWMSyncKill(uint32 syncKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_PWM_SYNC_KILL_MASK;
    PWMI_C_CONTROL_REG |= ((uint32)((syncKillEnable & PWMI_C_1BIT_MASK)  <<
                                               PWMI_C_PWM_SYNC_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPWMStopOnKill
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the PWM kill signal (stop input)
*  causes the PWM counter to stop.  By default the kill operation does not stop
*  the counter.  This functionality is only applicable to the three PWM modes.
*
*
* Parameters:
*  stopOnKillEnable
*   Values:
*     - 0 - Don't stop
*     - 1 - Stop
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPWMStopOnKill(uint32 stopOnKillEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_PWM_STOP_KILL_MASK;
    PWMI_C_CONTROL_REG |= ((uint32)((stopOnKillEnable & PWMI_C_1BIT_MASK)  <<
                                                         PWMI_C_PWM_STOP_KILL_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPWMDeadTime
********************************************************************************
*
* Summary:
*  Writes the dead time control value.  This value delays the rising edge of
*  both the line and line_n signals the designated number of cycles resulting
*  in both signals being inactive for that many cycles.  This functionality is
*  only applicable to the PWM in the dead time mode.

*
* Parameters:
*  Dead time to insert
*   Values: 0 to 255
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPWMDeadTime(uint32 deadTime)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_PRESCALER_MASK;
    PWMI_C_CONTROL_REG |= ((uint32)((deadTime & PWMI_C_8BIT_MASK) <<
                                                          PWMI_C_PRESCALER_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPWMInvert
********************************************************************************
*
* Summary:
*  Writes the bits that control whether the line and line_n outputs are
*  inverted from their normal output values.  This functionality is only
*  applicable to the three PWM modes.
*
* Parameters:
*  mask: Mask of outputs to invert.
*   Values:
*         - PWMI_C_INVERT_LINE   - Inverts the line output
*         - PWMI_C_INVERT_LINE_N - Inverts the line_n output
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPWMInvert(uint32 mask)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_INV_OUT_MASK;
    PWMI_C_CONTROL_REG |= mask;

    CyExitCriticalSection(enableInterrupts);
}



/*******************************************************************************
* Function Name: PWMI_C_WriteCounter
********************************************************************************
*
* Summary:
*  Writes a new 16bit counter value directly into the counter register, thus
*  setting the counter (not the period) to the value written. It is not
*  advised to write to this field when the counter is running.
*
* Parameters:
*  count: value to write
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_WriteCounter(uint32 count)
{
    PWMI_C_COUNTER_REG = (count & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadCounter
********************************************************************************
*
* Summary:
*  Reads the current counter value.
*
* Parameters:
*  None
*
* Return:
*  Current counter value
*
*******************************************************************************/
uint32 PWMI_C_ReadCounter(void)
{
    return (PWMI_C_COUNTER_REG & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_SetCounterMode
********************************************************************************
*
* Summary:
*  Sets the counter mode.  Applicable to all modes except Quadrature Decoder
*  and the PWM with a pseudo random output.
*
* Parameters:
*  counterMode: Enumerated counter type values
*   Values:
*     - PWMI_C_COUNT_UP       - Counts up
*     - PWMI_C_COUNT_DOWN     - Counts down
*     - PWMI_C_COUNT_UPDOWN0  - Counts up and down. Terminal count
*                                         generated when counter reaches 0
*     - PWMI_C_COUNT_UPDOWN1  - Counts up and down. Terminal count
*                                         generated both when counter reaches 0
*                                         and period
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetCounterMode(uint32 counterMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_UPDOWN_MASK;
    PWMI_C_CONTROL_REG |= counterMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_WritePeriod
********************************************************************************
*
* Summary:
*  Writes the 16 bit period register with the new period value.
*  To cause the counter to count for N cycles this register should be written
*  with N-1 (counts from 0 to period inclusive).
*
* Parameters:
*  period: Period value
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_WritePeriod(uint32 period)
{
    PWMI_C_PERIOD_REG = (period & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadPeriod
********************************************************************************
*
* Summary:
*  Reads the 16 bit period register.
*
* Parameters:
*  None
*
* Return:
*  Period value
*
*******************************************************************************/
uint32 PWMI_C_ReadPeriod(void)
{
    return (PWMI_C_PERIOD_REG & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_SetCompareSwap
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the compare registers are
*  swapped. When enabled in the Timer/Counter mode(without capture) the swap
*  occurs at a TC event. In the PWM mode the swap occurs at the next TC event
*  following a hardware switch event.
*
* Parameters:
*  swapEnable
*   Values:
*     - 0 - Disable swap
*     - 1 - Enable swap
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetCompareSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_RELOAD_CC_MASK;
    PWMI_C_CONTROL_REG |= (swapEnable & PWMI_C_1BIT_MASK);

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_WritePeriodBuf
********************************************************************************
*
* Summary:
*  Writes the 16 bit period buf register with the new period value.
*
* Parameters:
*  periodBuf: Period value
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_WritePeriodBuf(uint32 periodBuf)
{
    PWMI_C_PERIOD_BUF_REG = (periodBuf & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadPeriodBuf
********************************************************************************
*
* Summary:
*  Reads the 16 bit period buf register.
*
* Parameters:
*  None
*
* Return:
*  Period value
*
*******************************************************************************/
uint32 PWMI_C_ReadPeriodBuf(void)
{
    return (PWMI_C_PERIOD_BUF_REG & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_SetPeriodSwap
********************************************************************************
*
* Summary:
*  Writes the register that controls whether the period registers are
*  swapped. When enabled in Timer/Counter mode the swap occurs at a TC event.
*  In the PWM mode the swap occurs at the next TC event following a hardware
*  switch event.
*
* Parameters:
*  swapEnable
*   Values:
*     - 0 - Disable swap
*     - 1 - Enable swap
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetPeriodSwap(uint32 swapEnable)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_CONTROL_REG &= (uint32)~PWMI_C_RELOAD_PERIOD_MASK;
    PWMI_C_CONTROL_REG |= ((uint32)((swapEnable & PWMI_C_1BIT_MASK) <<
                                                            PWMI_C_RELOAD_PERIOD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_WriteCompare
********************************************************************************
*
* Summary:
*  Writes the 16 bit compare register with the new compare value. Not
*  applicable for Timer/Counter with Capture or in Quadrature Decoder modes.
*
* Parameters:
*  compare: Compare value
*
* Return:
*  None
*
* Note:
*  It is not recommended to use the value equal to "0" or equal to 
*  "period value" in Center or Asymmetric align PWM modes on the 
*  PSoC 4100/PSoC 4200 devices.
*  PSoC 4000 devices write the 16 bit compare register with the decremented 
*  compare value in the Up counting mode (except 0x0u), and the incremented 
*  compare value in the Down counting mode (except 0xFFFFu).
*
*******************************************************************************/
void PWMI_C_WriteCompare(uint32 compare)
{
    #if (PWMI_C_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (PWMI_C_CY_TCPWM_4000) */

    #if (PWMI_C_CY_TCPWM_4000)
        currentMode = ((PWMI_C_CONTROL_REG & PWMI_C_UPDOWN_MASK) >> PWMI_C_UPDOWN_SHIFT);

        if (((uint32)PWMI_C__COUNT_DOWN == currentMode) && (0xFFFFu != compare))
        {
            compare++;
        }
        else if (((uint32)PWMI_C__COUNT_UP == currentMode) && (0u != compare))
        {
            compare--;
        }
        else
        {
        }
        
    
    #endif /* (PWMI_C_CY_TCPWM_4000) */
    
    PWMI_C_COMP_CAP_REG = (compare & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadCompare
********************************************************************************
*
* Summary:
*  Reads the compare register. Not applicable for Timer/Counter with Capture
*  or in Quadrature Decoder modes.
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
* Parameters:
*  None
*
* Return:
*  Compare value
*
* Note:
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
*******************************************************************************/
uint32 PWMI_C_ReadCompare(void)
{
    #if (PWMI_C_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (PWMI_C_CY_TCPWM_4000) */

    #if (PWMI_C_CY_TCPWM_4000)
        currentMode = ((PWMI_C_CONTROL_REG & PWMI_C_UPDOWN_MASK) >> PWMI_C_UPDOWN_SHIFT);
        
        regVal = PWMI_C_COMP_CAP_REG;
        
        if (((uint32)PWMI_C__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)PWMI_C__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & PWMI_C_16BIT_MASK);
    #else
        return (PWMI_C_COMP_CAP_REG & PWMI_C_16BIT_MASK);
    #endif /* (PWMI_C_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: PWMI_C_WriteCompareBuf
********************************************************************************
*
* Summary:
*  Writes the 16 bit compare buffer register with the new compare value. Not
*  applicable for Timer/Counter with Capture or in Quadrature Decoder modes.
*
* Parameters:
*  compareBuf: Compare value
*
* Return:
*  None
*
* Note:
*  It is not recommended to use the value equal to "0" or equal to 
*  "period value" in Center or Asymmetric align PWM modes on the 
*  PSoC 4100/PSoC 4200 devices.
*  PSoC 4000 devices write the 16 bit compare register with the decremented 
*  compare value in the Up counting mode (except 0x0u), and the incremented 
*  compare value in the Down counting mode (except 0xFFFFu).
*
*******************************************************************************/
void PWMI_C_WriteCompareBuf(uint32 compareBuf)
{
    #if (PWMI_C_CY_TCPWM_4000)
        uint32 currentMode;
    #endif /* (PWMI_C_CY_TCPWM_4000) */

    #if (PWMI_C_CY_TCPWM_4000)
        currentMode = ((PWMI_C_CONTROL_REG & PWMI_C_UPDOWN_MASK) >> PWMI_C_UPDOWN_SHIFT);

        if (((uint32)PWMI_C__COUNT_DOWN == currentMode) && (0xFFFFu != compareBuf))
        {
            compareBuf++;
        }
        else if (((uint32)PWMI_C__COUNT_UP == currentMode) && (0u != compareBuf))
        {
            compareBuf --;
        }
        else
        {
        }
    #endif /* (PWMI_C_CY_TCPWM_4000) */
    
    PWMI_C_COMP_CAP_BUF_REG = (compareBuf & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadCompareBuf
********************************************************************************
*
* Summary:
*  Reads the compare buffer register. Not applicable for Timer/Counter with
*  Capture or in Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Compare buffer value
*
* Note:
*  PSoC 4000 devices read the incremented compare register value in the 
*  Up counting mode (except 0xFFFFu), and the decremented value in the 
*  Down counting mode (except 0x0u).
*
*******************************************************************************/
uint32 PWMI_C_ReadCompareBuf(void)
{
    #if (PWMI_C_CY_TCPWM_4000)
        uint32 currentMode;
        uint32 regVal;
    #endif /* (PWMI_C_CY_TCPWM_4000) */

    #if (PWMI_C_CY_TCPWM_4000)
        currentMode = ((PWMI_C_CONTROL_REG & PWMI_C_UPDOWN_MASK) >> PWMI_C_UPDOWN_SHIFT);

        regVal = PWMI_C_COMP_CAP_BUF_REG;
        
        if (((uint32)PWMI_C__COUNT_DOWN == currentMode) && (0u != regVal))
        {
            regVal--;
        }
        else if (((uint32)PWMI_C__COUNT_UP == currentMode) && (0xFFFFu != regVal))
        {
            regVal++;
        }
        else
        {
        }

        return (regVal & PWMI_C_16BIT_MASK);
    #else
        return (PWMI_C_COMP_CAP_BUF_REG & PWMI_C_16BIT_MASK);
    #endif /* (PWMI_C_CY_TCPWM_4000) */
}


/*******************************************************************************
* Function Name: PWMI_C_ReadCapture
********************************************************************************
*
* Summary:
*  Reads the captured counter value. This API is applicable only for
*  Timer/Counter with the capture mode and Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Capture value
*
*******************************************************************************/
uint32 PWMI_C_ReadCapture(void)
{
    return (PWMI_C_COMP_CAP_REG & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadCaptureBuf
********************************************************************************
*
* Summary:
*  Reads the capture buffer register. This API is applicable only for
*  Timer/Counter with the capture mode and Quadrature Decoder modes.
*
* Parameters:
*  None
*
* Return:
*  Capture buffer value
*
*******************************************************************************/
uint32 PWMI_C_ReadCaptureBuf(void)
{
    return (PWMI_C_COMP_CAP_BUF_REG & PWMI_C_16BIT_MASK);
}


/*******************************************************************************
* Function Name: PWMI_C_SetCaptureMode
********************************************************************************
*
* Summary:
*  Sets the capture trigger mode. For PWM mode this is the switch input.
*  This input is not applicable to the Timer/Counter without Capture and
*  Quadrature Decoder modes.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - PWMI_C_TRIG_LEVEL     - Level
*     - PWMI_C_TRIG_RISING    - Rising edge
*     - PWMI_C_TRIG_FALLING   - Falling edge
*     - PWMI_C_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetCaptureMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_TRIG_CONTROL1_REG &= (uint32)~PWMI_C_CAPTURE_MASK;
    PWMI_C_TRIG_CONTROL1_REG |= triggerMode;

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetReloadMode
********************************************************************************
*
* Summary:
*  Sets the reload trigger mode. For Quadrature Decoder mode this is the index
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - PWMI_C_TRIG_LEVEL     - Level
*     - PWMI_C_TRIG_RISING    - Rising edge
*     - PWMI_C_TRIG_FALLING   - Falling edge
*     - PWMI_C_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetReloadMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_TRIG_CONTROL1_REG &= (uint32)~PWMI_C_RELOAD_MASK;
    PWMI_C_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << PWMI_C_RELOAD_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetStartMode
********************************************************************************
*
* Summary:
*  Sets the start trigger mode. For Quadrature Decoder mode this is the
*  phiB input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - PWMI_C_TRIG_LEVEL     - Level
*     - PWMI_C_TRIG_RISING    - Rising edge
*     - PWMI_C_TRIG_FALLING   - Falling edge
*     - PWMI_C_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetStartMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_TRIG_CONTROL1_REG &= (uint32)~PWMI_C_START_MASK;
    PWMI_C_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << PWMI_C_START_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetStopMode
********************************************************************************
*
* Summary:
*  Sets the stop trigger mode. For PWM mode this is the kill input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - PWMI_C_TRIG_LEVEL     - Level
*     - PWMI_C_TRIG_RISING    - Rising edge
*     - PWMI_C_TRIG_FALLING   - Falling edge
*     - PWMI_C_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetStopMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_TRIG_CONTROL1_REG &= (uint32)~PWMI_C_STOP_MASK;
    PWMI_C_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << PWMI_C_STOP_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_SetCountMode
********************************************************************************
*
* Summary:
*  Sets the count trigger mode. For Quadrature Decoder mode this is the phiA
*  input.
*
* Parameters:
*  triggerMode: Enumerated trigger mode value
*   Values:
*     - PWMI_C_TRIG_LEVEL     - Level
*     - PWMI_C_TRIG_RISING    - Rising edge
*     - PWMI_C_TRIG_FALLING   - Falling edge
*     - PWMI_C_TRIG_BOTH      - Both rising and falling edge
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetCountMode(uint32 triggerMode)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_TRIG_CONTROL1_REG &= (uint32)~PWMI_C_COUNT_MASK;
    PWMI_C_TRIG_CONTROL1_REG |= ((uint32)(triggerMode << PWMI_C_COUNT_SHIFT));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_TriggerCommand
********************************************************************************
*
* Summary:
*  Triggers the designated command to occur on the designated TCPWM instances.
*  The mask can be used to apply this command simultaneously to more than one
*  instance.  This allows multiple TCPWM instances to be synchronized.
*
* Parameters:
*  mask: A combination of mask bits for each instance of the TCPWM that the
*        command should apply to.  This function from one instance can be used
*        to apply the command to any of the instances in the design.
*        The mask value for a specific instance is available with the MASK
*        define.
*  command: Enumerated command values. Capture command only applicable for
*           Timer/Counter with Capture and PWM modes.
*   Values:
*     - PWMI_C_CMD_CAPTURE    - Trigger Capture/Switch command
*     - PWMI_C_CMD_RELOAD     - Trigger Reload/Index command
*     - PWMI_C_CMD_STOP       - Trigger Stop/Kill command
*     - PWMI_C_CMD_START      - Trigger Start/phiB command
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_TriggerCommand(uint32 mask, uint32 command)
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();

    PWMI_C_COMMAND_REG = ((uint32)(mask << command));

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PWMI_C_ReadStatus
********************************************************************************
*
* Summary:
*  Reads the status of the PWMI_C.
*
* Parameters:
*  None
*
* Return:
*  Status
*   Values:
*     - PWMI_C_STATUS_DOWN    - Set if counting down
*     - PWMI_C_STATUS_RUNNING - Set if counter is running
*
*******************************************************************************/
uint32 PWMI_C_ReadStatus(void)
{
    return ((PWMI_C_STATUS_REG >> PWMI_C_RUNNING_STATUS_SHIFT) |
            (PWMI_C_STATUS_REG & PWMI_C_STATUS_DOWN));
}


/*******************************************************************************
* Function Name: PWMI_C_SetInterruptMode
********************************************************************************
*
* Summary:
*  Sets the interrupt mask to control which interrupt
*  requests generate the interrupt signal.
*
* Parameters:
*   interruptMask: Mask of bits to be enabled
*   Values:
*     - PWMI_C_INTR_MASK_TC       - Terminal count mask
*     - PWMI_C_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetInterruptMode(uint32 interruptMask)
{
    PWMI_C_INTERRUPT_MASK_REG =  interruptMask;
}


/*******************************************************************************
* Function Name: PWMI_C_GetInterruptSourceMasked
********************************************************************************
*
* Summary:
*  Gets the interrupt requests masked by the interrupt mask.
*
* Parameters:
*   None
*
* Return:
*  Masked interrupt source
*   Values:
*     - PWMI_C_INTR_MASK_TC       - Terminal count mask
*     - PWMI_C_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 PWMI_C_GetInterruptSourceMasked(void)
{
    return (PWMI_C_INTERRUPT_MASKED_REG);
}


/*******************************************************************************
* Function Name: PWMI_C_GetInterruptSource
********************************************************************************
*
* Summary:
*  Gets the interrupt requests (without masking).
*
* Parameters:
*  None
*
* Return:
*  Interrupt request value
*   Values:
*     - PWMI_C_INTR_MASK_TC       - Terminal count mask
*     - PWMI_C_INTR_MASK_CC_MATCH - Compare count / capture mask
*
*******************************************************************************/
uint32 PWMI_C_GetInterruptSource(void)
{
    return (PWMI_C_INTERRUPT_REQ_REG);
}


/*******************************************************************************
* Function Name: PWMI_C_ClearInterrupt
********************************************************************************
*
* Summary:
*  Clears the interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to clear
*   Values:
*     - PWMI_C_INTR_MASK_TC       - Terminal count mask
*     - PWMI_C_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_ClearInterrupt(uint32 interruptMask)
{
    PWMI_C_INTERRUPT_REQ_REG = interruptMask;
}


/*******************************************************************************
* Function Name: PWMI_C_SetInterrupt
********************************************************************************
*
* Summary:
*  Sets a software interrupt request.
*
* Parameters:
*   interruptMask: Mask of interrupts to set
*   Values:
*     - PWMI_C_INTR_MASK_TC       - Terminal count mask
*     - PWMI_C_INTR_MASK_CC_MATCH - Compare count / capture mask
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SetInterrupt(uint32 interruptMask)
{
    PWMI_C_INTERRUPT_SET_REG = interruptMask;
}


/* [] END OF FILE */
