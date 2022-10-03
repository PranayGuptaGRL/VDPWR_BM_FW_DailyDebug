/**
 * @file stack_params.h
 *
 * @brief @{Header file that defines parameters to configure the CCGx Firmware
 * Stack. The current definitions for these values are optimized for the CCG3PA
 * Port Controller implementation and should not be changed.
 *
 * Please contact Cypress for details of possible customizations in these
 * settings.@}
 */

/*
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
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
#ifndef _STACK_PARAMS_H_
#define _STACK_PARAMS_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>

/*******************************************************************************
 * CCG Device Selection.
 ******************************************************************************/

/*
 * Select target device family. This definition is used to implement the PD
 * block driver.
 */
#define CCG3PA

/* Select target silicon ID for CYPD3171-24LQXQ. */
#define CCG_DEV_SILICON_ID                      (0x2003)
#define CCG_DEV_FAMILY_ID                       (0x11B0)

/* No. of USB-PD ports supported. CYPD3171-24LQXQ supports one port. */
#define NO_OF_TYPEC_PORTS                       (1u)

#define TYPEC_PORT_0_IDX                        (0u)
#define TYPEC_PORT_1_IDX                        (1u)

/* Enable TYPE-A support. */
#define CCG_TYPE_A_PORT_ENABLE                  (0u)

#if CCG_TYPE_A_PORT_ENABLE

/* TYPE-A port ID. */
#define TYPE_A_PORT_ID                          (1u)

/* Dual regulator support for TYPE-A VBUS. */
#define TYPE_A_DUAL_REG_ENABLE                  (1u)

#endif /* CCG_TYPE_A_PORT_ENABLE */

#if (NO_OF_TYPEC_PORTS >= 2) || (CCG_TYPE_A_PORT_ENABLE == 1)
    /* Set this flag to enable the second PD port. */
    #define CCG_PD_DUALPORT_ENABLE              (1u)
#else
    #define CCG_PD_DUALPORT_ENABLE              (0u)
#endif

/*******************************************************************************
 * Solution workarounds.
 ******************************************************************************/
/*
 * FET control lines for producer and consumer may be reversed or used differently
 * for certain packages or applications.
 */
#define CCG_FLIPPED_FET_CTRL                    (1u)
#define CCG_SRC_FET                             (0u)
#define CCG_SNK_FET                             (0u)

/*
 * Disable Pseudo-metadata handling in flashing sequence.
 * This definition should be left enabled for CCG4 solutions.
 */
#define CCG_PSEUDO_METADATA_DISABLE             (1u)

/*******************************************************************************
 * Enable PD spec Rev 3 support
 ******************************************************************************/
#define CCG_PD_REV3_ENABLE                        (1u)

#if CCG_PD_REV3_ENABLE
    #define CCG_FRS_RX_ENABLE                     (0u)
    #define CCG_FRS_TX_ENABLE                     (0u)
    #define CCG_PPS_SRC_ENABLE                    (1u)
#endif /* CCG_PD_REV3_ENABLE */

#define CCG_PROG_SOURCE_ENABLE                    (1u)

#if CCG_PPS_SRC_ENABLE
    #define CCG_REV3_HANDLE_BAD_SINK              (1u)
#endif /* CCG_PPS_SRC_ENABLE */

/*******************************************************************************
 * Enable Battery Charging support
 ******************************************************************************/
#define  BATTERY_CHARGING_ENABLE                   (0u)
#if CCG_TYPE_A_PORT_ENABLE
#define  NO_OF_BC_PORTS                            (2u)
#else /* !CCG_TYPE_A_PORT_ENABLE */
#define  NO_OF_BC_PORTS                            (1u)
#endif /* CCG_TYPE_A_PORT_ENABLE */
#define  BC_PORT_0_IDX                             (0u)
#define  BC_PORT_1_IDX                             (1u)

#if BATTERY_CHARGING_ENABLE
/* Enable this option if PD and legacy state machines shall run in parallel. */
#define LEGACY_PD_PARALLEL_OPER                    (1u)

/*
 * Enable / disable external Apple source termination via external resistor 
 * combination.
 *
 * This is useful only when legacy BC 1.2 and Apple source protocols need to 
 * co-exist. This compile time option also requires the solution to implement 
 * external Apple 2.4A terminations. The sln_apply_apple_src_term() and 
 * sln_remove_apple_src_term() functions shall be implemented to match the
 * solution hardware and requirement.
 *
 * This macro is port independant. If only one port needs support for external 
 * termination, even then the solution specific functions need to be implemented.
 *
 * To support parallel detection logic, an external resistance(s) and a dedicated
 * GPIO per port is required. Each port can be configured differently. If the 
 * specific port does not have external resistance connected, then it should not
 * be configured for parallel operation. In this case, the solution handler can
 * invoke the HAL function directly to use the internal terminations. Refer to
 * description of sln_apply_apple_src_term() function for details of hardware 
 * connection and usage.
 */
#define LEGACY_APPLE_SRC_EXT_TERM_ENABLE            (0u)
#endif /* BATTERY_CHARGING_ENABLE */

/*******************************************************************************
 * High level firmware feature selection.
 ******************************************************************************/

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
#if CCG_TYPE_A_PORT_ENABLE
#define TIMER_NUM_INSTANCES                     (2)
#else
#define TIMER_NUM_INSTANCES                     (NO_OF_TYPEC_PORTS)
#endif /* CCG_TYPE_A_PORT_ENABLE */

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD             (250u)

/* 
 * Period (in ms) between every step change of vbus. This is not used for 
 * this device family.
 */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD     (1u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD         (5u)

/*
 * Time (in ms) for which the VBus_Discharge path will be enabled when turning
 * power source OFF.
 */
#define APP_PSOURCE_DIS_TIMER_PERIOD            (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD    (1u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * voltage is rising above 5V.
 */
#define VBUS_CTRL_ABOVE_5V_MAX_STEP             (20000u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * voltage is falling below 5V.
 */
#define VBUS_CTRL_BELOW_5V_MAX_STEP             (5000u)

/* 
 * Enable this macro if the interval between steps need to be done in micro-
 * seconds. This may be required if the regulator is very sensitive to feedback
 * node change. This will allow for smaller feedback voltage step size. When
 * enabling this feature, a TCPWM timer should be added with TC interrupt 
 * configured for one-shot timer interrupting with the required interval as
 * period. The minimum interval allowed is 200us. Use this only if you need step
 * interval between 200us and 1ms. If this is disabled, the step size is assumed
 * to be 1ms.
 *
 * Most direct feedback based regulators are fast and does not require this. So
 * this is left disabled by default.
 */
#define VBUS_CTRL_STEP_US_ENABLE                (0u)

/* 
 * Period (in ms) to debounce the VBUS stable detection. VBUS change is detected
 * using ADC over the specified duration.
 */
#define VBUS_CTRL_SLOPE_DEBOUNCE_PERIOD         (5u)

/* 
 * Minimum period (in ms) for vbus to settle after reaching stable slope. This
 * is imposed to ensure that we provide a minimum time before indicating ready.
 */
#define VBUS_CTRL_SETTLE_TIME_PERIOD            (15u)

/*
 * Enable this macro to allow the VBUS voltage to be corrected based on ADC
 * readings. This allows for better accuracy in direct feedback systems. This
 * should not be enabled for opto-isolator designs.
 */
#define VBUS_CTRL_ADC_ADJUST_ENABLE             (1u)

#if CCG_PPS_SRC_ENABLE
#define VBUS_CF_EN                              (1u)     
#endif /* CCG_PPS_SRC_ENABLE */

/*******************************************************************************
 * VBus monitor configuration.
 ******************************************************************************/

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                     (-20)

/* Allowed VBus valid margin (as percentage of expected voltage) before detach detection is triggered. */
#define VBUS_TURN_OFF_MARGIN                    (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                   (20)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN             (10)

/* VBUS in discharge enable */
#define VBUS_IN_DISCHARGE_EN                    (1u)

/* CCG3PA discharge drive strength settings. */
#if VBUS_IN_DISCHARGE_EN
#define CCG3PA_DISCHG_DS_VBUS_C_0A              (4u)
#else /* !VBUS_IN_DISCHARGE_EN */
#define CCG3PA_DISCHG_DS_VBUS_C_0A              (8u)
#endif
#define CCG3PA_DISCHG_DS_VBUS_IN_0A             (4u)

/*******************************************************************************
 * VBus Monitor connection configuration for Port 1.
 ******************************************************************************/
//GRL-EDIT, below 3 were uncommented
/* CCG IO port to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
//#define APP_VBUS_MON_PORT_NO_P1                 (VBUS_MON_P1__PORT)

/* CCG IO pin to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
//#define APP_VBUS_MON_PIN_NO_P1                  (VBUS_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VBUS_MON_P1 pin. */
#define APP_VBUS_MON_PORT_PIN_P1                ((VBUS_MON_P1__PORT << 4) | VBUS_MON_P1__SHIFT)

/*
 * IO setting to connect VBUS_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VBUS_MON_AMUX_INPUT_P1              (6)

/*******************************************************************************
 * VConn Monitor connection configuration for Port 1.
 * This section is optional as VConn monitoring is not enabled in the stack.
 ******************************************************************************/

/* CCG IO port to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PORT_NO_P1                (VCONN_MON_P1__PORT)

/* CCG IO pin to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PIN_NO_P1                 (VCONN_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VCONN_MON_P1 pin. */
#define APP_VCONN_MON_PORT_PIN_P1               ((VCONN_MON_P1__PORT << 4) | VCONN_MON_P1__SHIFT)

/*
 * IO setting to connect VCONN_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VCONN_MON_AMUX_INPUT_P1             (7)

/*******************************************************************************
 * VBus OCP fault GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PORT_NO_P1           (OCP_FAULT_P1__PORT)

/* CCG pin to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PIN_NO_P1            (OCP_FAULT_P1__SHIFT)

/* Combined Port+Pin representation for the OCP_FAULT_P1 pin. */
#define APP_VBUS_OCP_PORT_PIN_P1                ((OCP_FAULT_P1__PORT << 4) | OCP_FAULT_P1__SHIFT)

/*******************************************************************************
 * VBUS_OVP_TRIP GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PORT_NO_P1            (VBUS_OVP_TRIP_P1__PORT)

/* CCG pin to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PIN_NO_P1             (VBUS_OVP_TRIP_P1__SHIFT)

/* Combined Port+Pin representation of the VBUS_OVP_TRIP_P1 pin. */
#define APP_VBUS_OVP_TRIP_PORT_PIN_P1           ((VBUS_OVP_TRIP_P1__PORT << 4) | VBUS_OVP_TRIP_P1__SHIFT)

/* CCG IO mode corresponding to the VBUS_OVP_TRIP functionality. This should be set to 12. */
#define APP_VBUS_OVP_TRIP_HSIOM_P1              (12)

/*******************************************************************************
 * CCG3 VBus FET control staggering. Ranges from 0 to 0xF LF clock cycles.
 ******************************************************************************/
#define NGDO_TURN_OFF_SPACING                   (0xA)

/*******************************************************************************
 * ADC selection for various functions.
 ******************************************************************************/
#define APP_VBUS_POLL_ADC_ID                    (PD_ADC_ID_0)
#define APP_VBUS_POLL_ADC_INPUT                 (PD_ADC_INPUT_AMUX_B)

/*******************************************************************************
 * Types of Data path switches supported.
 ******************************************************************************/

/* CCG controlled switch for DisplayPort and USB lines. */
#define DP_MUX                                  (0u)

/* Data path switching handled by Alpine Ridge. */
#define RIDGE_MUX                               (1u)

/* This firmware supports only CCG controlled DP/USB switch operation. */
#define MUX_TYPE                                3

/*******************************************************************************
 * VBUS Slow Discharge Feature.
 ******************************************************************************/

/*
 * Enable / disable VBUS Slow Discharge Feature. When this feature is enabled,
 * the discharge drive strength shall be increased by steps every ms until the
 * selected top drive strength is achieved. Similarly, the drive strength is
 * decreased in steps while stopping the discharge.
 */
#define VBUS_SLOW_DISCHARGE_EN                  (0u)

/*
 * Macro defines the time in milliseconds for which source cap starting will
 * be delayed.
 */
#define DELAY_SRC_CAP_START_MS                  (100u)

#endif /* _STACK_PARAMS_H_ */

/* End of file */
