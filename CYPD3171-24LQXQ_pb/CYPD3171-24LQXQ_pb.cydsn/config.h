/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG firmware features.
 *
 * This file also provides mapping to the implementation for hardware dependent
 * functions like FET control, voltage selection etc.
 *
 * This current implementation matches the CY4531 EVK from Cypress. This can be
 * updated by users to match their hardware implementation.@}
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
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <project.h>
#include <stack_params.h>
#include <power_bank.h>
#include <vbus_ctrl.h>

//#define POWER_BANK                                  (0)

/*******************************************************************************
 * VBus Control Type
 ******************************************************************************/

#define VBUS_CTRL_NONE                              (0)

/* VBUS Control using PWM. */
#define VBUS_CTRL_PWM                               (1)

/* VBUS Control using Direct Feedback. */
#define VBUS_CTRL_DIR_FB                            (2)

/* VBUS Control using Opto Feedback. */
#define VBUS_CTRL_OPTO_FB                           (3)

/* This is the VBUS Control type used by the application for P1. */
#define VBUS_CTRL_TYPE_P1                           (VBUS_CTRL_DIR_FB)
//#define VBUS_CTRL_TYPE_P1                           (VBUS_CTRL_NONE)//GRL-EDIT
    
/* This is the VBUS Control type used by the application for P2 (TYPE-A port). */
#define VBUS_CTRL_TYPE_P2                           (VBUS_CTRL_PWM)

/*******************************************************************************
 * PSOURCE controls for PD port 1.
 ******************************************************************************/

#define REGULATOR_REQUIRE_STABLE_ON_TIME           (1)
#define REGULATOR_ENABLE()                          BUCK_BOOST_EN_C_Write(0)
#define REGULATOR_DISABLE()                         BUCK_BOOST_EN_C_Write(1)

/*
 * CCTRL is used for controlling the SRC direction as well. So using the
 * corresponding FET configuration. In source mode operation, OCP is controlled
 * by firmware and should not be controlled via PWM. So keep it disabled.
 */
#define APP_VBUS_SRC_FET_ON_P1()                    \
{                                                   \
    PWMI_C_Stop();                                  \
    PWMI_OUT_C_SetDriveMode                         \
    (PWMI_OUT_C_DM_ALG_HIZ);                        \
    DIR_CTRL_C_Write(1);                            \
    pd_internal_cfet_on(0, false);                  \
    BUCK_BOOST_EN_C_Write(0);                       \
}

#define APP_VBUS_SET_VOLT_P1(volt_mV)               vbus_ctrl_fb_set_volt(TYPEC_PORT_0_IDX, volt_mV)

#define APP_VBUS_SRC_FET_OFF_P1()                   \
{                                                   \
    BUCK_BOOST_EN_C_Write(1);                       \
    pd_internal_cfet_off(0, false);                 \
}

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P1()                   pd_internal_vbus_discharge_on(0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 OFF. */
#define APP_DISCHARGE_FET_OFF_P1()                  pd_internal_vbus_discharge_off(0)

/*******************************************************************************
 * PSOURCE controls for Port 2 (TYPE A port).
 ******************************************************************************/

#define APP_VBUS_SRC_FET_ON_P2()                    vbus_ctrl_pwm_turn_on(TYPEC_PORT_1_IDX)

#define APP_VBUS_SET_VOLT_P2(volt_mV)               vbus_ctrl_pwm_set_volt(TYPEC_PORT_1_IDX, volt_mV)

#define APP_VBUS_SRC_FET_OFF_P2()                   vbus_ctrl_pwm_turn_off(TYPEC_PORT_1_IDX)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P2()                   ((void)0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P2 OFF. */
#define APP_DISCHARGE_FET_OFF_P2()                  ((void)0)

/*******************************************************************************
 * Power Sink (PSINK) controls for PD port 1.
 ******************************************************************************/

/* Function/Macro to turn consumer FET for P1 ON. */

/* Enable consumer direction, turn on Consumer FETs and enable Buck Boost converter. */
#define APP_VBUS_SNK_FET_ON_P1()                    \
{                                                   \
    DIR_CTRL_C_Write(0);                            \
    pd_internal_cfet_on(0, false);                  \
    BUCK_BOOST_EN_C_Write(0);                       \
}

/* Function/Macro to turn consumer FET for P1 OFF. */

/*
 * Disbale Buck Boost converter, turn off Consumer FETs, set direction to provider mode
 * and turn off PWM I which limits current consumption.
 */
#define APP_VBUS_SNK_FET_OFF_P1()                   \
{                                                   \
    BUCK_BOOST_EN_C_Write(1);                       \
    pd_internal_cfet_off(0, false);                 \
    DIR_CTRL_C_Write(1);                            \
    PWMI_C_WriteCompare(1);                         \
    PWMI_C_Stop();                                  \
    PWMI_OUT_C_SetDriveMode                         \
    (PWMI_OUT_C_DM_ALG_HIZ);                        \
}

#define APP_SINK_SET_CURRENT_P1(cur_10mA)           pb_typec_set_current(cur_10mA)

/*******************************************************************************
 * Power Sink (PSINK) controls for Port 2 (TYPE A port).
 ******************************************************************************/

#define APP_VBUS_SNK_FET_OFF_P2()                  ((void)0)

#define APP_VBUS_SNK_FET_ON_P2()                   ((void)0)

#define APP_SINK_SET_CURRENT_P2(cur_10mA)          ((void)0)

/*******************************************************************************
 * VBus Monitoring Controls for detach detection.
 ******************************************************************************/

/* Division factor applied between VBus and the voltage on VBUS_MON input. */
#define VBUS_MON_DIVIDER                            (11u)

/*******************************************************************************
 * VBus Over-Current Protection Configuration.
 *
 * The VBus OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/

/*
 * VBus OCP feature enable. This can be enabled on CY4531 and on other boards
 * that have the load switch.
 */
//#define VBUS_OCP_ENABLE                             (1u)
#define VBUS_OCP_ENABLE                             (0u)//GRL-EDIT

/*
 * Software OCP mode.
 * 0 - External OCP hardware.
 * 1 - Internal OCP with neither software debounce nor automatic FET control.
 * 2 - Internal OCP with automatic FET control by hardware when an OCP event is
 *     detected.
 * 3 - Internal OCP with software debounce using delay in milliseconds from the
 *     config table.
 */
#define VBUS_OCP_MODE                               (3u)

/*
 * Total Rsense as seen at the CSA input pin in units of 0.1mOhm (100 uOhm). 
 * This is the sum of the actual resistor value along with any compensation 
 * required for the board layout. This is to avoid inaccuracies due to trace
 * resistance.
 */
#define VBUS_CSA_RSENSE                             (50u)

/*******************************************************************************
 * VBus Over-Voltage Protection Configuration.
 *
 * The VBus OVP feature uses an internal ADC in the CCG to measure the voltage
 * on the VBUS_MON input and uses the ADC output to detect over-voltage
 * conditions.
 *
 * The default implementation of OVP uses firmware ISRs to turn off the FETs
 * when OVP is detected. If quicker response is desired, there is the option of
 * using a direct OVP_TRIP output derived from a hardware comparator associated
 * with the ADC.
 ******************************************************************************/

/* VBus OVP enable setting. */
//#define VBUS_OVP_ENABLE                             (1u)
#define VBUS_OVP_ENABLE                             (1u)//GRL-EDIT

/*
 * OVP mode selection
 * 0 - OVP using ADC comparator.
 * 1 - OVP using dedicated comparator. Firmware detects trip interrupt and turns off the FETs.
 * 2 - OVP using dedicated comparator. Hardware detects trip interrupt and turns off the FETs.
 */
#define VBUS_OVP_MODE                               (2u)

/*******************************************************************************
 * VBus Under-Voltage Protection Configuration.
 ******************************************************************************/

/* VBus UVP enable setting. */
//#define VBUS_UVP_ENABLE                             (1u)
#define VBUS_UVP_ENABLE                             (0u)//GRL-EDIT

/*
 * UVP mode selection
 * 0 - UVP using ADC comparator.
 * 1 - UVP using dedicated comparator. Firmware detects trip interrupt and turns off the FETs.
 * 2 - UVP using dedicated comparator. Hardware detects trip interrupt and turns off the FETs.
 */
#define VBUS_UVP_MODE                               (2u)

/*******************************************************************************
 * VBus Short Circuit Protection Configuration.
 ******************************************************************************/

/*
 * VBus SCP feature enable.
 */
#define VBUS_SCP_ENABLE                             (1u)

/* SCP mode: AUTO Control. */
#define VBUS_SCP_MODE                               (2u)

/*******************************************************************************
 * VConn Over-Current Protection Configuration.
 *
 * The VConn OCP feature is implemented based on an external load switch and
 * only uses a fault indicator GPIO coming into the CCG device.
 ******************************************************************************/

/* VConn OCP enable setting. */
#define VCONN_OCP_ENABLE                            (0u)

/*******************************************************************************
 * VBUS offset voltage configuration.
 *
 * Offset voltage value is a configuration table parameter.
 ******************************************************************************/
 /* VBUS offset voltage enable setting. */
#define VBUS_OFFSET_VOLTAGE_ENABLE                  (1u)

/*******************************************************************************
 * Firmware feature configuration.
 ******************************************************************************/

/* Set to 1 if building a debug enabled binary with no boot-loader dependency. */
#define CCG_FIRMWARE_APP_ONLY                       (0u)

/* Single firmware Image */
#define CCG_DUALAPP_DISABLE                         (1u)

/* Enable CCG deep sleep to save power. */
#define SYS_DEEPSLEEP_ENABLE                        (0u)

/* Enable hardware based DRP toggle for additional power saving. */
#define CCG_HW_DRP_TOGGLE_ENABLE                    (1u)

/* Enable Alternate Mode support when CCG is DFP. */
#define DFP_ALT_MODE_SUPP                           (0u)

/* Enable DisplayPort Source support as DFP. */
#define DP_DFP_SUPP                                 (0u)

/* Enable Alt mode as UFP */
#define UFP_ALT_MODE_SUPP                           (0u)

/* Enable saving only SVIDs which are supported by CCG. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/* Enable HPI support. */
#define CCG_HPI_ENABLE                              (0u)

/* Enable PD policy registers in HPI. */
#define CCG_HPI_PD_ENABLE                           (0u)

/*
 * Index of SCB used for HPI interface. This should be set based on
 * the pin selection in the project schematic.
 */
#define HPI_SCB_INDEX                               (2u)

/* Enable image selection based on APP Priority Field. */
#define APP_PRIORITY_FEATURE_ENABLE                 (0u)

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                           (0u)

/* Enable Internal UVP Comparator to be used as VBUS divider. */
#define VBUS_MON_INTERNAL                           (0u)//GRL-EDIT 1 was here

/*
 * Select CCG3 GPIO to be used as Activity Indication. This should be set to a
 * valid value if APP_FW_LED_ENABLE is non-zero.
 */
#define FW_LED_GPIO_PORT_PIN                        (GPIO_PORT_2_PIN_0)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                                (0xC0)
/*
 * The LED toggle period.
 */
#define LED_TIMER_PERIOD                            (1000)

/* Timer used to ensure I2C transfers to the MUX complete on time. */
#define MUX_I2C_TIMER                               (0xC1)
/* The MUX transfer timeout is set to 10 ms timeout period. */
#define MUX_I2C_TIMER_PERIOD                        (10u)

/* Enabling flashing of the device via PD interface. */
#define FLASHING_MODE_PD_ENABLE                     (0u)

/***********************************************************************************/

/* Enable watchdog hardware reset for CPU lock-up recovery */
#define WATCHDOG_HARDWARE_RESET_ENABLE              (1u)

/* Disable CCG device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/* Enable reset reporting through HPI. */
#define HPI_WATCHDOG_RESET_ENABLE                   (0u)

/* Watchdog reset timer id. */
#define WATCHDOG_TIMER_ID                           (0xC2u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Enable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/* Set this to 1 to Shutdown the SNK FET in the application layer. */
#define SNK_FET_SHUTDOWN_ENABLE                     (1u)

/***********************************************************************************/

/*
 * The following macro enables cable compensation logic. The feature is 
 * applicable only for Type-C port of CCG3PA/CCG3PA2 designs which uses the
 * internal feedback based voltage control.
 */
#define CCG_CABLE_COMP_ENABLE                       (0u)//GRL_EDIT 1 was here

/* 
 * The following macro disables the cable compensation logic for
 * PPS contracts.
 */
#define CCG_CABLE_COMP_IN_PPS_DISABLE               (0u)

/*
 * The following macro defines whether we will handle extended   
 * message in solution space. 
 */    
#define CCG_SLN_EXTN_MSG_HANDLER_ENABLE             (1u)
    
/* Valid battery_status response when source */
    
#define CCG_EXT_MSG_VALID_BAT_STAT_SRC              (0xffff0600)

/* Valid battery_status response when sink */
    
#define CCG_EXT_MSG_VALID_BAT_STAT_SNK              (0xffff0200)
    
/* Invalid battery_status response */
    
#define CCG_EXT_MSG_INVALID_BAT_REF                 (0xffff0100)

/* This macro defines the number of batteries */   
#define CCG_PB_NO_OF_BATT                           (1u)   

/* This macro defines the VID-PID for Power Bank */   
#define CCG_PB_VID_PID                              (0xf56504b4)

/* This macro defines the battery design capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Design capacity unknown
 */     
#define CCG_PB_BAT_DES_CAP                          (0x0000)//GRL-EDIT 0xFFFF was here
    
/* This macro defines the battery last full charge capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Last full charge capacity unknown
 */
#define CCG_PB_BAT_FUL_CHG_CAP                      (0x0000)//GRL-EDIT 0xFFFF was here
    
#endif /* _CONFIG_H_ */

/* End of file */
