/**
 * @file config.h
 *
 * @brief Internal config header file used to generate PD stack library.
 */

/*
 * @copyright
 *
 * Copyright (2014-2016), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */
#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#define TYPEC_PORT_0_IDX        (0)
#define TYPEC_PORT_1_IDX        (1)

/*******************************************************************************
 * Timer Module Configuration
 ******************************************************************************/

/*
 * The timer module provides software timer support. It provides multiple
 * timers running off a single hardware timer and has a general accuracy of 5%.
 * This module can generate callbacks at a granularity of 1ms. It provides
 * various backend implementations selectable via the following compile time
 * options. The various options can be selected by selecting the required value
 * for implemenation macro TIMER_TYPE.
 */

/*
 * SYS_TICK based timer backend, interrupting the system every 1ms.
 * This implementation requires the SYS_TICK timer to be reserved for the use
 * of this module. This implementation shall not function in DEEP_SLEEP mode
 * and the user should ensure that the timer is shut-off before entering
 * DEEP_SLEEP. To shut off the timer, just ensure that all soft-timers are
 * stopped or have expired.
 */
#define TIMER_TYPE_SYSTICK                      (1)

/*
 * WDT based timer backend.
 * This requires user to reserve both WDT and SYS_TICK timers for the use of this.
 * The WDT timer runs off ILO which is highly inaccurate. The SYS_TICK timer is
 * used to calibrate the WDT to match IMO accuracy. The WDT based
 * implementation works across DEEP_SLEEP.
 */
#define TIMER_TYPE_WDT                          (2)

/*
 * Timer implementation selection.
 * TIMER_TYPE_WDT should be used if deep sleep entry for power saving is
 * being used.
 */
#define TIMER_TYPE                              (TIMER_TYPE_WDT)

/*
 * In addition to the hardware timer options, the module also provides a
 * TICKLESS timer implementation. The TICKLESS implementation is currently
 * available only for WDT based timer backend. The TICKLESS timer interrupts
 * the system only at the timer expiry (instead of every 1ms). Since this
 * involves a more complex algorithm, it requires more code space (about 200
 * bytes more). This implementation allows the same timer to be used in ACTIVE
 * as well as DEEP_SLEEP modes, due to the WDT backend. It also gives maximum
 * power savings as well as faster execution due to less number of interrupts.
 */
#define TIMER_TICKLESS_ENABLE                   (1)

/*
 * Timer module supports multiple software instances based on a single hardware
 * timer. The number of instances is defined based on the PD port count.
 */
#define TIMER_NUM_INSTANCES                     (NO_OF_TYPEC_PORTS)

/*****************************************************************************/
/*           Declarations for functions provided by Creator.                 */
/*****************************************************************************/

#define CY_ISR(FuncName)        void FuncName (void)
#define CY_ISR_PROTO(FuncName)  void FuncName (void)
typedef void (* cyisraddress)(void);

void     CyDelayUs(uint16_t microseconds);
uint8_t  CyEnterCriticalSection(void);
void     CyExitCriticalSection(uint8_t savedIntrStatus);
void     CyIntSetPriority(uint8_t number, uint8_t priority);
uint8_t  CyIntGetPriority(uint8_t number);
void     CyIntEnable(uint8_t number);
uint8_t  CyIntGetState(uint8_t number);
void     CyIntDisable(uint8_t number);
void     CyIntSetPending(uint8_t number);
void     CyIntClearPending(uint8_t number);
uint32_t CyDisableInts(void);
void     CyEnableInts(uint32_t mask);
cyisraddress CyIntSetSysVector(uint8_t number, cyisraddress address);
cyisraddress CyIntGetSysVector(uint8_t number);
cyisraddress CyIntSetVector(uint8_t number, cyisraddress address);
cyisraddress CyIntGetVector(uint8_t number);

#define CyGlobalIntEnable           do                          \
{                                                               \
    __asm("CPSIE   i");                                         \
} while ( 0 )

#define CyGlobalIntDisable          do                          \
{                                                               \
    __asm("CPSID   i");                                         \
} while ( 0 )

#define I2C_CFG_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
#define I2C_CFG_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
#define I2C_CFG_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
#define I2C_CFG_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
#define I2C_CFG_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
#define I2C_CFG_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
#define I2C_CFG_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
#define I2C_CFG_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */

void I2C_CFG_SetDriveMode(uint8_t mode);
void I2C_CFG_Write(uint8_t value);
uint8_t I2C_CFG_Read(void);
void EC_INT_Write(uint8_t value);
void CySoftwareReset(void);

#endif /* _CONFIG_H_ */

/* End of file */

