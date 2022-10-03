/*******************************************************************************
* File Name: PWMI_C.h
* Version 2.10
*
* Description:
*  This file provides constants and parameter values for the PWMI_C
*  component.
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

#if !defined(CY_TCPWM_PWMI_C_H)
#define CY_TCPWM_PWMI_C_H


#include "CyLib.h"
#include "cytypes.h"
#include "cyfitter.h"


/*******************************************************************************
* Internal Type defines
*******************************************************************************/

/* Structure to save state before go to sleep */
typedef struct
{
    uint8  enableState;
} PWMI_C_BACKUP_STRUCT;


/*******************************************************************************
* Variables
*******************************************************************************/
extern uint8  PWMI_C_initVar;


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define PWMI_C_CY_TCPWM_V2                    (CYIPBLOCK_m0s8tcpwm_VERSION == 2u)
#define PWMI_C_CY_TCPWM_4000                  (CY_PSOC4_4000)

/* TCPWM Configuration */
#define PWMI_C_CONFIG                         (7lu)

/* Quad Mode */
/* Parameters */
#define PWMI_C_QUAD_ENCODING_MODES            (0lu)
#define PWMI_C_QUAD_AUTO_START                (1lu)

/* Signal modes */
#define PWMI_C_QUAD_INDEX_SIGNAL_MODE         (0lu)
#define PWMI_C_QUAD_PHIA_SIGNAL_MODE          (3lu)
#define PWMI_C_QUAD_PHIB_SIGNAL_MODE          (3lu)
#define PWMI_C_QUAD_STOP_SIGNAL_MODE          (0lu)

/* Signal present */
#define PWMI_C_QUAD_INDEX_SIGNAL_PRESENT      (0lu)
#define PWMI_C_QUAD_STOP_SIGNAL_PRESENT       (0lu)

/* Interrupt Mask */
#define PWMI_C_QUAD_INTERRUPT_MASK            (1lu)

/* Timer/Counter Mode */
/* Parameters */
#define PWMI_C_TC_RUN_MODE                    (0lu)
#define PWMI_C_TC_COUNTER_MODE                (0lu)
#define PWMI_C_TC_COMP_CAP_MODE               (2lu)
#define PWMI_C_TC_PRESCALER                   (0lu)

/* Signal modes */
#define PWMI_C_TC_RELOAD_SIGNAL_MODE          (0lu)
#define PWMI_C_TC_COUNT_SIGNAL_MODE           (3lu)
#define PWMI_C_TC_START_SIGNAL_MODE           (0lu)
#define PWMI_C_TC_STOP_SIGNAL_MODE            (0lu)
#define PWMI_C_TC_CAPTURE_SIGNAL_MODE         (0lu)

/* Signal present */
#define PWMI_C_TC_RELOAD_SIGNAL_PRESENT       (0lu)
#define PWMI_C_TC_COUNT_SIGNAL_PRESENT        (0lu)
#define PWMI_C_TC_START_SIGNAL_PRESENT        (0lu)
#define PWMI_C_TC_STOP_SIGNAL_PRESENT         (0lu)
#define PWMI_C_TC_CAPTURE_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define PWMI_C_TC_INTERRUPT_MASK              (1lu)

/* PWM Mode */
/* Parameters */
#define PWMI_C_PWM_KILL_EVENT                 (0lu)
#define PWMI_C_PWM_STOP_EVENT                 (0lu)
#define PWMI_C_PWM_MODE                       (4lu)
#define PWMI_C_PWM_OUT_N_INVERT               (0lu)
#define PWMI_C_PWM_OUT_INVERT                 (0lu)
#define PWMI_C_PWM_ALIGN                      (0lu)
#define PWMI_C_PWM_RUN_MODE                   (0lu)
#define PWMI_C_PWM_DEAD_TIME_CYCLE            (0lu)
#define PWMI_C_PWM_PRESCALER                  (0lu)

/* Signal modes */
#define PWMI_C_PWM_RELOAD_SIGNAL_MODE         (0lu)
#define PWMI_C_PWM_COUNT_SIGNAL_MODE          (3lu)
#define PWMI_C_PWM_START_SIGNAL_MODE          (0lu)
#define PWMI_C_PWM_STOP_SIGNAL_MODE           (0lu)
#define PWMI_C_PWM_SWITCH_SIGNAL_MODE         (0lu)

/* Signal present */
#define PWMI_C_PWM_RELOAD_SIGNAL_PRESENT      (0lu)
#define PWMI_C_PWM_COUNT_SIGNAL_PRESENT       (0lu)
#define PWMI_C_PWM_START_SIGNAL_PRESENT       (0lu)
#define PWMI_C_PWM_STOP_SIGNAL_PRESENT        (0lu)
#define PWMI_C_PWM_SWITCH_SIGNAL_PRESENT      (0lu)

/* Interrupt Mask */
#define PWMI_C_PWM_INTERRUPT_MASK             (0lu)


/***************************************
*    Initial Parameter Constants
***************************************/

/* Timer/Counter Mode */
#define PWMI_C_TC_PERIOD_VALUE                (65535lu)
#define PWMI_C_TC_COMPARE_VALUE               (65535lu)
#define PWMI_C_TC_COMPARE_BUF_VALUE           (65535lu)
#define PWMI_C_TC_COMPARE_SWAP                (0lu)

/* PWM Mode */
#define PWMI_C_PWM_PERIOD_VALUE               (500lu)
#define PWMI_C_PWM_PERIOD_BUF_VALUE           (65535lu)
#define PWMI_C_PWM_PERIOD_SWAP                (0lu)
#define PWMI_C_PWM_COMPARE_VALUE              (1lu)
#define PWMI_C_PWM_COMPARE_BUF_VALUE          (65535lu)
#define PWMI_C_PWM_COMPARE_SWAP               (0lu)


/***************************************
*    Enumerated Types and Parameters
***************************************/

#define PWMI_C__LEFT 0
#define PWMI_C__RIGHT 1
#define PWMI_C__CENTER 2
#define PWMI_C__ASYMMETRIC 3

#define PWMI_C__X1 0
#define PWMI_C__X2 1
#define PWMI_C__X4 2

#define PWMI_C__PWM 4
#define PWMI_C__PWM_DT 5
#define PWMI_C__PWM_PR 6

#define PWMI_C__INVERSE 1
#define PWMI_C__DIRECT 0

#define PWMI_C__CAPTURE 2
#define PWMI_C__COMPARE 0

#define PWMI_C__TRIG_LEVEL 3
#define PWMI_C__TRIG_RISING 0
#define PWMI_C__TRIG_FALLING 1
#define PWMI_C__TRIG_BOTH 2

#define PWMI_C__INTR_MASK_TC 1
#define PWMI_C__INTR_MASK_CC_MATCH 2
#define PWMI_C__INTR_MASK_NONE 0
#define PWMI_C__INTR_MASK_TC_CC 3

#define PWMI_C__UNCONFIG 8
#define PWMI_C__TIMER 1
#define PWMI_C__QUAD 3
#define PWMI_C__PWM_SEL 7

#define PWMI_C__COUNT_UP 0
#define PWMI_C__COUNT_DOWN 1
#define PWMI_C__COUNT_UPDOWN0 2
#define PWMI_C__COUNT_UPDOWN1 3


/* Prescaler */
#define PWMI_C_PRESCALE_DIVBY1                ((uint32)(0u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY2                ((uint32)(1u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY4                ((uint32)(2u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY8                ((uint32)(3u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY16               ((uint32)(4u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY32               ((uint32)(5u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY64               ((uint32)(6u << PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_PRESCALE_DIVBY128              ((uint32)(7u << PWMI_C_PRESCALER_SHIFT))

/* TCPWM set modes */
#define PWMI_C_MODE_TIMER_COMPARE             ((uint32)(PWMI_C__COMPARE         <<  \
                                                                  PWMI_C_MODE_SHIFT))
#define PWMI_C_MODE_TIMER_CAPTURE             ((uint32)(PWMI_C__CAPTURE         <<  \
                                                                  PWMI_C_MODE_SHIFT))
#define PWMI_C_MODE_QUAD                      ((uint32)(PWMI_C__QUAD            <<  \
                                                                  PWMI_C_MODE_SHIFT))
#define PWMI_C_MODE_PWM                       ((uint32)(PWMI_C__PWM             <<  \
                                                                  PWMI_C_MODE_SHIFT))
#define PWMI_C_MODE_PWM_DT                    ((uint32)(PWMI_C__PWM_DT          <<  \
                                                                  PWMI_C_MODE_SHIFT))
#define PWMI_C_MODE_PWM_PR                    ((uint32)(PWMI_C__PWM_PR          <<  \
                                                                  PWMI_C_MODE_SHIFT))

/* Quad Modes */
#define PWMI_C_MODE_X1                        ((uint32)(PWMI_C__X1              <<  \
                                                                  PWMI_C_QUAD_MODE_SHIFT))
#define PWMI_C_MODE_X2                        ((uint32)(PWMI_C__X2              <<  \
                                                                  PWMI_C_QUAD_MODE_SHIFT))
#define PWMI_C_MODE_X4                        ((uint32)(PWMI_C__X4              <<  \
                                                                  PWMI_C_QUAD_MODE_SHIFT))

/* Counter modes */
#define PWMI_C_COUNT_UP                       ((uint32)(PWMI_C__COUNT_UP        <<  \
                                                                  PWMI_C_UPDOWN_SHIFT))
#define PWMI_C_COUNT_DOWN                     ((uint32)(PWMI_C__COUNT_DOWN      <<  \
                                                                  PWMI_C_UPDOWN_SHIFT))
#define PWMI_C_COUNT_UPDOWN0                  ((uint32)(PWMI_C__COUNT_UPDOWN0   <<  \
                                                                  PWMI_C_UPDOWN_SHIFT))
#define PWMI_C_COUNT_UPDOWN1                  ((uint32)(PWMI_C__COUNT_UPDOWN1   <<  \
                                                                  PWMI_C_UPDOWN_SHIFT))

/* PWM output invert */
#define PWMI_C_INVERT_LINE                    ((uint32)(PWMI_C__INVERSE         <<  \
                                                                  PWMI_C_INV_OUT_SHIFT))
#define PWMI_C_INVERT_LINE_N                  ((uint32)(PWMI_C__INVERSE         <<  \
                                                                  PWMI_C_INV_COMPL_OUT_SHIFT))

/* Trigger modes */
#define PWMI_C_TRIG_RISING                    ((uint32)PWMI_C__TRIG_RISING)
#define PWMI_C_TRIG_FALLING                   ((uint32)PWMI_C__TRIG_FALLING)
#define PWMI_C_TRIG_BOTH                      ((uint32)PWMI_C__TRIG_BOTH)
#define PWMI_C_TRIG_LEVEL                     ((uint32)PWMI_C__TRIG_LEVEL)

/* Interrupt mask */
#define PWMI_C_INTR_MASK_TC                   ((uint32)PWMI_C__INTR_MASK_TC)
#define PWMI_C_INTR_MASK_CC_MATCH             ((uint32)PWMI_C__INTR_MASK_CC_MATCH)

/* PWM Output Controls */
#define PWMI_C_CC_MATCH_SET                   (0x00u)
#define PWMI_C_CC_MATCH_CLEAR                 (0x01u)
#define PWMI_C_CC_MATCH_INVERT                (0x02u)
#define PWMI_C_CC_MATCH_NO_CHANGE             (0x03u)
#define PWMI_C_OVERLOW_SET                    (0x00u)
#define PWMI_C_OVERLOW_CLEAR                  (0x04u)
#define PWMI_C_OVERLOW_INVERT                 (0x08u)
#define PWMI_C_OVERLOW_NO_CHANGE              (0x0Cu)
#define PWMI_C_UNDERFLOW_SET                  (0x00u)
#define PWMI_C_UNDERFLOW_CLEAR                (0x10u)
#define PWMI_C_UNDERFLOW_INVERT               (0x20u)
#define PWMI_C_UNDERFLOW_NO_CHANGE            (0x30u)

/* PWM Align */
#define PWMI_C_PWM_MODE_LEFT                  (PWMI_C_CC_MATCH_CLEAR        |   \
                                                         PWMI_C_OVERLOW_SET           |   \
                                                         PWMI_C_UNDERFLOW_NO_CHANGE)
#define PWMI_C_PWM_MODE_RIGHT                 (PWMI_C_CC_MATCH_SET          |   \
                                                         PWMI_C_OVERLOW_NO_CHANGE     |   \
                                                         PWMI_C_UNDERFLOW_CLEAR)
#define PWMI_C_PWM_MODE_ASYM                  (PWMI_C_CC_MATCH_INVERT       |   \
                                                         PWMI_C_OVERLOW_SET           |   \
                                                         PWMI_C_UNDERFLOW_CLEAR)

#if (PWMI_C_CY_TCPWM_V2)
    #if(PWMI_C_CY_TCPWM_4000)
        #define PWMI_C_PWM_MODE_CENTER                (PWMI_C_CC_MATCH_INVERT       |   \
                                                                 PWMI_C_OVERLOW_NO_CHANGE     |   \
                                                                 PWMI_C_UNDERFLOW_CLEAR)
    #else
        #define PWMI_C_PWM_MODE_CENTER                (PWMI_C_CC_MATCH_INVERT       |   \
                                                                 PWMI_C_OVERLOW_SET           |   \
                                                                 PWMI_C_UNDERFLOW_CLEAR)
    #endif /* (PWMI_C_CY_TCPWM_4000) */
#else
    #define PWMI_C_PWM_MODE_CENTER                (PWMI_C_CC_MATCH_INVERT       |   \
                                                             PWMI_C_OVERLOW_NO_CHANGE     |   \
                                                             PWMI_C_UNDERFLOW_CLEAR)
#endif /* (PWMI_C_CY_TCPWM_NEW) */

/* Command operations without condition */
#define PWMI_C_CMD_CAPTURE                    (0u)
#define PWMI_C_CMD_RELOAD                     (8u)
#define PWMI_C_CMD_STOP                       (16u)
#define PWMI_C_CMD_START                      (24u)

/* Status */
#define PWMI_C_STATUS_DOWN                    (1u)
#define PWMI_C_STATUS_RUNNING                 (2u)


/***************************************
*        Function Prototypes
****************************************/

void   PWMI_C_Init(void);
void   PWMI_C_Enable(void);
void   PWMI_C_Start(void);
void   PWMI_C_Stop(void);

void   PWMI_C_SetMode(uint32 mode);
void   PWMI_C_SetCounterMode(uint32 counterMode);
void   PWMI_C_SetPWMMode(uint32 modeMask);
void   PWMI_C_SetQDMode(uint32 qdMode);

void   PWMI_C_SetPrescaler(uint32 prescaler);
void   PWMI_C_TriggerCommand(uint32 mask, uint32 command);
void   PWMI_C_SetOneShot(uint32 oneShotEnable);
uint32 PWMI_C_ReadStatus(void);

void   PWMI_C_SetPWMSyncKill(uint32 syncKillEnable);
void   PWMI_C_SetPWMStopOnKill(uint32 stopOnKillEnable);
void   PWMI_C_SetPWMDeadTime(uint32 deadTime);
void   PWMI_C_SetPWMInvert(uint32 mask);

void   PWMI_C_SetInterruptMode(uint32 interruptMask);
uint32 PWMI_C_GetInterruptSourceMasked(void);
uint32 PWMI_C_GetInterruptSource(void);
void   PWMI_C_ClearInterrupt(uint32 interruptMask);
void   PWMI_C_SetInterrupt(uint32 interruptMask);

void   PWMI_C_WriteCounter(uint32 count);
uint32 PWMI_C_ReadCounter(void);

uint32 PWMI_C_ReadCapture(void);
uint32 PWMI_C_ReadCaptureBuf(void);

void   PWMI_C_WritePeriod(uint32 period);
uint32 PWMI_C_ReadPeriod(void);
void   PWMI_C_WritePeriodBuf(uint32 periodBuf);
uint32 PWMI_C_ReadPeriodBuf(void);

void   PWMI_C_WriteCompare(uint32 compare);
uint32 PWMI_C_ReadCompare(void);
void   PWMI_C_WriteCompareBuf(uint32 compareBuf);
uint32 PWMI_C_ReadCompareBuf(void);

void   PWMI_C_SetPeriodSwap(uint32 swapEnable);
void   PWMI_C_SetCompareSwap(uint32 swapEnable);

void   PWMI_C_SetCaptureMode(uint32 triggerMode);
void   PWMI_C_SetReloadMode(uint32 triggerMode);
void   PWMI_C_SetStartMode(uint32 triggerMode);
void   PWMI_C_SetStopMode(uint32 triggerMode);
void   PWMI_C_SetCountMode(uint32 triggerMode);

void   PWMI_C_SaveConfig(void);
void   PWMI_C_RestoreConfig(void);
void   PWMI_C_Sleep(void);
void   PWMI_C_Wakeup(void);


/***************************************
*             Registers
***************************************/

#define PWMI_C_BLOCK_CONTROL_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define PWMI_C_BLOCK_CONTROL_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_CTRL )
#define PWMI_C_COMMAND_REG                    (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define PWMI_C_COMMAND_PTR                    ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_CMD )
#define PWMI_C_INTRRUPT_CAUSE_REG             (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define PWMI_C_INTRRUPT_CAUSE_PTR             ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TCPWM_INTR_CAUSE )
#define PWMI_C_CONTROL_REG                    (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CTRL )
#define PWMI_C_CONTROL_PTR                    ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CTRL )
#define PWMI_C_STATUS_REG                     (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__STATUS )
#define PWMI_C_STATUS_PTR                     ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__STATUS )
#define PWMI_C_COUNTER_REG                    (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__COUNTER )
#define PWMI_C_COUNTER_PTR                    ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__COUNTER )
#define PWMI_C_COMP_CAP_REG                   (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CC )
#define PWMI_C_COMP_CAP_PTR                   ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CC )
#define PWMI_C_COMP_CAP_BUF_REG               (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CC_BUFF )
#define PWMI_C_COMP_CAP_BUF_PTR               ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__CC_BUFF )
#define PWMI_C_PERIOD_REG                     (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__PERIOD )
#define PWMI_C_PERIOD_PTR                     ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__PERIOD )
#define PWMI_C_PERIOD_BUF_REG                 (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define PWMI_C_PERIOD_BUF_PTR                 ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__PERIOD_BUFF )
#define PWMI_C_TRIG_CONTROL0_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define PWMI_C_TRIG_CONTROL0_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL0 )
#define PWMI_C_TRIG_CONTROL1_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define PWMI_C_TRIG_CONTROL1_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL1 )
#define PWMI_C_TRIG_CONTROL2_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define PWMI_C_TRIG_CONTROL2_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__TR_CTRL2 )
#define PWMI_C_INTERRUPT_REQ_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR )
#define PWMI_C_INTERRUPT_REQ_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR )
#define PWMI_C_INTERRUPT_SET_REG              (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_SET )
#define PWMI_C_INTERRUPT_SET_PTR              ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_SET )
#define PWMI_C_INTERRUPT_MASK_REG             (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_MASK )
#define PWMI_C_INTERRUPT_MASK_PTR             ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_MASK )
#define PWMI_C_INTERRUPT_MASKED_REG           (*(reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_MASKED )
#define PWMI_C_INTERRUPT_MASKED_PTR           ( (reg32 *) PWMI_C_cy_m0s8_tcpwm_1__INTR_MASKED )


/***************************************
*       Registers Constants
***************************************/

/* Mask */
#define PWMI_C_MASK                           ((uint32)PWMI_C_cy_m0s8_tcpwm_1__TCPWM_CTRL_MASK)

/* Shift constants for control register */
#define PWMI_C_RELOAD_CC_SHIFT                (0u)
#define PWMI_C_RELOAD_PERIOD_SHIFT            (1u)
#define PWMI_C_PWM_SYNC_KILL_SHIFT            (2u)
#define PWMI_C_PWM_STOP_KILL_SHIFT            (3u)
#define PWMI_C_PRESCALER_SHIFT                (8u)
#define PWMI_C_UPDOWN_SHIFT                   (16u)
#define PWMI_C_ONESHOT_SHIFT                  (18u)
#define PWMI_C_QUAD_MODE_SHIFT                (20u)
#define PWMI_C_INV_OUT_SHIFT                  (20u)
#define PWMI_C_INV_COMPL_OUT_SHIFT            (21u)
#define PWMI_C_MODE_SHIFT                     (24u)

/* Mask constants for control register */
#define PWMI_C_RELOAD_CC_MASK                 ((uint32)(PWMI_C_1BIT_MASK        <<  \
                                                                            PWMI_C_RELOAD_CC_SHIFT))
#define PWMI_C_RELOAD_PERIOD_MASK             ((uint32)(PWMI_C_1BIT_MASK        <<  \
                                                                            PWMI_C_RELOAD_PERIOD_SHIFT))
#define PWMI_C_PWM_SYNC_KILL_MASK             ((uint32)(PWMI_C_1BIT_MASK        <<  \
                                                                            PWMI_C_PWM_SYNC_KILL_SHIFT))
#define PWMI_C_PWM_STOP_KILL_MASK             ((uint32)(PWMI_C_1BIT_MASK        <<  \
                                                                            PWMI_C_PWM_STOP_KILL_SHIFT))
#define PWMI_C_PRESCALER_MASK                 ((uint32)(PWMI_C_8BIT_MASK        <<  \
                                                                            PWMI_C_PRESCALER_SHIFT))
#define PWMI_C_UPDOWN_MASK                    ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                            PWMI_C_UPDOWN_SHIFT))
#define PWMI_C_ONESHOT_MASK                   ((uint32)(PWMI_C_1BIT_MASK        <<  \
                                                                            PWMI_C_ONESHOT_SHIFT))
#define PWMI_C_QUAD_MODE_MASK                 ((uint32)(PWMI_C_3BIT_MASK        <<  \
                                                                            PWMI_C_QUAD_MODE_SHIFT))
#define PWMI_C_INV_OUT_MASK                   ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                            PWMI_C_INV_OUT_SHIFT))
#define PWMI_C_MODE_MASK                      ((uint32)(PWMI_C_3BIT_MASK        <<  \
                                                                            PWMI_C_MODE_SHIFT))

/* Shift constants for trigger control register 1 */
#define PWMI_C_CAPTURE_SHIFT                  (0u)
#define PWMI_C_COUNT_SHIFT                    (2u)
#define PWMI_C_RELOAD_SHIFT                   (4u)
#define PWMI_C_STOP_SHIFT                     (6u)
#define PWMI_C_START_SHIFT                    (8u)

/* Mask constants for trigger control register 1 */
#define PWMI_C_CAPTURE_MASK                   ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                  PWMI_C_CAPTURE_SHIFT))
#define PWMI_C_COUNT_MASK                     ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                  PWMI_C_COUNT_SHIFT))
#define PWMI_C_RELOAD_MASK                    ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                  PWMI_C_RELOAD_SHIFT))
#define PWMI_C_STOP_MASK                      ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                  PWMI_C_STOP_SHIFT))
#define PWMI_C_START_MASK                     ((uint32)(PWMI_C_2BIT_MASK        <<  \
                                                                  PWMI_C_START_SHIFT))

/* MASK */
#define PWMI_C_1BIT_MASK                      ((uint32)0x01u)
#define PWMI_C_2BIT_MASK                      ((uint32)0x03u)
#define PWMI_C_3BIT_MASK                      ((uint32)0x07u)
#define PWMI_C_6BIT_MASK                      ((uint32)0x3Fu)
#define PWMI_C_8BIT_MASK                      ((uint32)0xFFu)
#define PWMI_C_16BIT_MASK                     ((uint32)0xFFFFu)

/* Shift constant for status register */
#define PWMI_C_RUNNING_STATUS_SHIFT           (30u)


/***************************************
*    Initial Constants
***************************************/

#define PWMI_C_CTRL_QUAD_BASE_CONFIG                                                          \
        (((uint32)(PWMI_C_QUAD_ENCODING_MODES     << PWMI_C_QUAD_MODE_SHIFT))       |\
         ((uint32)(PWMI_C_CONFIG                  << PWMI_C_MODE_SHIFT)))

#define PWMI_C_CTRL_PWM_BASE_CONFIG                                                           \
        (((uint32)(PWMI_C_PWM_STOP_EVENT          << PWMI_C_PWM_STOP_KILL_SHIFT))   |\
         ((uint32)(PWMI_C_PWM_OUT_INVERT          << PWMI_C_INV_OUT_SHIFT))         |\
         ((uint32)(PWMI_C_PWM_OUT_N_INVERT        << PWMI_C_INV_COMPL_OUT_SHIFT))   |\
         ((uint32)(PWMI_C_PWM_MODE                << PWMI_C_MODE_SHIFT)))

#define PWMI_C_CTRL_PWM_RUN_MODE                                                              \
            ((uint32)(PWMI_C_PWM_RUN_MODE         << PWMI_C_ONESHOT_SHIFT))
            
#define PWMI_C_CTRL_PWM_ALIGN                                                                 \
            ((uint32)(PWMI_C_PWM_ALIGN            << PWMI_C_UPDOWN_SHIFT))

#define PWMI_C_CTRL_PWM_KILL_EVENT                                                            \
             ((uint32)(PWMI_C_PWM_KILL_EVENT      << PWMI_C_PWM_SYNC_KILL_SHIFT))

#define PWMI_C_CTRL_PWM_DEAD_TIME_CYCLE                                                       \
            ((uint32)(PWMI_C_PWM_DEAD_TIME_CYCLE  << PWMI_C_PRESCALER_SHIFT))

#define PWMI_C_CTRL_PWM_PRESCALER                                                             \
            ((uint32)(PWMI_C_PWM_PRESCALER        << PWMI_C_PRESCALER_SHIFT))

#define PWMI_C_CTRL_TIMER_BASE_CONFIG                                                         \
        (((uint32)(PWMI_C_TC_PRESCALER            << PWMI_C_PRESCALER_SHIFT))       |\
         ((uint32)(PWMI_C_TC_COUNTER_MODE         << PWMI_C_UPDOWN_SHIFT))          |\
         ((uint32)(PWMI_C_TC_RUN_MODE             << PWMI_C_ONESHOT_SHIFT))         |\
         ((uint32)(PWMI_C_TC_COMP_CAP_MODE        << PWMI_C_MODE_SHIFT)))
        
#define PWMI_C_QUAD_SIGNALS_MODES                                                             \
        (((uint32)(PWMI_C_QUAD_PHIA_SIGNAL_MODE   << PWMI_C_COUNT_SHIFT))           |\
         ((uint32)(PWMI_C_QUAD_INDEX_SIGNAL_MODE  << PWMI_C_RELOAD_SHIFT))          |\
         ((uint32)(PWMI_C_QUAD_STOP_SIGNAL_MODE   << PWMI_C_STOP_SHIFT))            |\
         ((uint32)(PWMI_C_QUAD_PHIB_SIGNAL_MODE   << PWMI_C_START_SHIFT)))

#define PWMI_C_PWM_SIGNALS_MODES                                                              \
        (((uint32)(PWMI_C_PWM_SWITCH_SIGNAL_MODE  << PWMI_C_CAPTURE_SHIFT))         |\
         ((uint32)(PWMI_C_PWM_COUNT_SIGNAL_MODE   << PWMI_C_COUNT_SHIFT))           |\
         ((uint32)(PWMI_C_PWM_RELOAD_SIGNAL_MODE  << PWMI_C_RELOAD_SHIFT))          |\
         ((uint32)(PWMI_C_PWM_STOP_SIGNAL_MODE    << PWMI_C_STOP_SHIFT))            |\
         ((uint32)(PWMI_C_PWM_START_SIGNAL_MODE   << PWMI_C_START_SHIFT)))

#define PWMI_C_TIMER_SIGNALS_MODES                                                            \
        (((uint32)(PWMI_C_TC_CAPTURE_SIGNAL_MODE  << PWMI_C_CAPTURE_SHIFT))         |\
         ((uint32)(PWMI_C_TC_COUNT_SIGNAL_MODE    << PWMI_C_COUNT_SHIFT))           |\
         ((uint32)(PWMI_C_TC_RELOAD_SIGNAL_MODE   << PWMI_C_RELOAD_SHIFT))          |\
         ((uint32)(PWMI_C_TC_STOP_SIGNAL_MODE     << PWMI_C_STOP_SHIFT))            |\
         ((uint32)(PWMI_C_TC_START_SIGNAL_MODE    << PWMI_C_START_SHIFT)))
        
#define PWMI_C_TIMER_UPDOWN_CNT_USED                                                          \
                ((PWMI_C__COUNT_UPDOWN0 == PWMI_C_TC_COUNTER_MODE)                  ||\
                 (PWMI_C__COUNT_UPDOWN1 == PWMI_C_TC_COUNTER_MODE))

#define PWMI_C_PWM_UPDOWN_CNT_USED                                                            \
                ((PWMI_C__CENTER == PWMI_C_PWM_ALIGN)                               ||\
                 (PWMI_C__ASYMMETRIC == PWMI_C_PWM_ALIGN))               
        
#define PWMI_C_PWM_PR_INIT_VALUE              (1u)
#define PWMI_C_QUAD_PERIOD_INIT_VALUE         (0x8000u)



#endif /* End CY_TCPWM_PWMI_C_H */

/* [] END OF FILE */
