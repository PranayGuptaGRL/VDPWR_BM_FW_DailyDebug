/**
 * @file app.c
 *
 * @brief @{PD application handler source file.@}
 *
 *******************************************************************************
 *
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <pdo.h>
#include <swap.h>
#include <vdm.h>
#include <app.h>
#include <vdm_task_mngr.h>
#include <timer.h>
#include <hpi.h>
#include <alt_mode_hw.h>
#include <alt_modes_mngr.h>
#include <hal_ccgx.h>
#include <gpio.h>
#include <system.h>
#if CCG_LOAD_SHARING_ENABLE
#include <loadsharing.h>
#endif /* CCG_LOAD_SHARING_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
#include <sensor_check.h>
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
#if CCG_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CCG_CABLE_COMP_ENABLE */

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#include <intel_ridge.h>
#endif /* RIDGE_SLAVE_ENABLE */
#if DP_UFP_SUPP
#include <dp_sid.h>    
#endif /* DP_UFP_SUPP */    
#if (CCG_BB_ENABLE != 0)
#include <billboard.h>
#endif /* (CCG_BB_ENABLE != 0) */

#if DP_UFP_SUPP
#include <hpd.h>
#endif /* DP_UFP_SUPP */

#if BATTERY_CHARGING_ENABLE
#include <battery_charging.h>
#endif /* BATTERY_CHARGING_ENABLE */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
#include <type_a.h>
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if CCG_UCSI_ENABLE
#include <ucsi.h>
#endif /* CCG_UCSI_ENABLE */

#if APP_PPS_SINK_SUPPORT
uint8_t hpi_user_reg_handler(uint16_t addr, uint8_t size, uint8_t *data);
void app_pps_sink_disable(uint8_t port);
#endif /* APP_PPS_SINK_SUPPORT */

ovp_settings_t* pd_get_ptr_ovp_tbl(uint8_t port)
{
    /* Update the OVP settings from the configuration table. */
    return ((ovp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ovp_tbl_offset));
}

ocp_settings_t* pd_get_ptr_ocp_tbl(uint8_t port)
{
    /* Update the VBus OCP settings from the configuration table */
    return ((ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->ocp_tbl_offset));
}

rcp_settings_t* pd_get_ptr_rcp_tbl(uint8_t port)
{
    /* Update the VBus RCP settings from the configuration table */
    return ((rcp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->rcp_tbl_offset));
}

uvp_settings_t* pd_get_ptr_uvp_tbl(uint8_t port)
{
    /* Update the VBus UVP settings from the configuration table */
    return ((uvp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->uvp_tbl_offset));
}

scp_settings_t* pd_get_ptr_scp_tbl(uint8_t port)
{
    /* Update the VBus SCP settings from the configuration table */
    return ((scp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->scp_tbl_offset));
}

vconn_ocp_settings_t* pd_get_ptr_vconn_ocp_tbl(uint8_t port)
{
    /* Update the Vcon OCP settings from the configuration table */
    return ((vconn_ocp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->vconn_ocp_tbl_offset));
}

otp_settings_t* pd_get_ptr_otp_tbl(uint8_t port)
{
    /* Update the OTP settings from the configuration table */
    return ((otp_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->otp_tbl_offset));
}

pwr_params_t* pd_get_ptr_pwr_tbl(uint8_t port)
{
    /* Update the power parameters from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->pwr_tbl_offset));
}

chg_cfg_params_t* pd_get_ptr_chg_cfg_tbl(uint8_t port)
{
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Return parameters for TYPE-A port. */
        return pd_get_ptr_type_a_chg_cfg_tbl (0);
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    /* Update the legacy charging parameters from the configuration table */
    return ((chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
        get_pd_port_config(port)->chg_cfg_tbl_offset));
}

bat_chg_params_t* pd_get_ptr_bat_chg_tbl(uint8_t port)
{
    /* Update the battery charging parameterss from the configuration table */
    return ((bat_chg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bat_chg_tbl_offset));
}

pwr_params_t* pd_get_ptr_type_a_pwr_tbl(uint8_t port)
{
    /* Update the power parameters of Type-A port from the configuration table */
    return ((pwr_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_pwr_tbl_offset));
}

typeA_chg_cfg_params_t* pd_get_ptr_type_a_chg_cfg_tbl(uint8_t port)
{
    /* Update the legacy charging parameters from the configuration table */
    return ((typeA_chg_cfg_params_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->type_a_chg_tbl_offset));
}

bb_settings_t* pd_get_ptr_bb_tbl(uint8_t port)
{
    /* Update the Billboard settings from the configuration table*/
    return ((bb_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->bb_tbl_offset));
}

auto_cfg_settings_t* pd_get_ptr_auto_cfg_tbl(uint8_t port)
{
    /* Update the Automotive charger settings from the configuration table*/
    return ((auto_cfg_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->auto_cfg_tbl_offset));
}

tbthost_cfg_settings_t* pd_get_ptr_tbthost_cfg_tbl(uint8_t port)
{
    /* Retrieve the thunderbolt host config parameters. */
    return ((tbthost_cfg_settings_t *)((uint8_t *)(get_pd_config ()) +
                get_pd_port_config(port)->tbthost_cfg_tbl_offset));
}

#if (CCG_TYPE_A_PORT_ENABLE)
app_status_t app_status[2];
#else
app_status_t app_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_TYPE_A_PORT_ENABLE */

enum {
    FAULT_TYPE_VBUS_OVP = 0,    /* 0 */
    FAULT_TYPE_VBUS_UVP,        /* 1 */
    FAULT_TYPE_VBUS_OCP,        /* 2 */
    FAULT_TYPE_VBUS_SCP,        /* 3 */
    FAULT_TYPE_CC_OVP,          /* 4 */
    FAULT_TYPE_VCONN_OCP,       /* 5 */
    FAULT_TYPE_SBU_OVP,         /* 6 */
    FAULT_TYPE_OTP,             /* 7 */
    FAULT_TYPE_VBUS_RCP,        /* 8 */
    FAULT_TYPE_COUNT            /* 9 */
};

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
/* Number of retries defined by user for each fault type. */
static uint8_t gl_app_fault_retry_limit[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] = {
    0
};
#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

/* Number of times each fault condition has been detected during current connection. */
static volatile uint8_t gl_app_fault_count[NO_OF_TYPEC_PORTS][FAULT_TYPE_COUNT] = {
    0
};

/* Flag to indicate that activity timer timed out. */
static volatile bool ccg_activity_timer_timeout = false;

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timeout status */
volatile uint8_t gl_bad_sink_timeout_status[NO_OF_TYPEC_PORTS];
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
extern bool gl_power_throttle_cmd_pending[NO_OF_TYPEC_PORTS];
extern bool gl_power_throttle_renegotiation_complete[NO_OF_TYPEC_PORTS];
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)

/* Check whether any fault count has exceeded limit for the specified PD port. */
static bool app_port_fault_count_exceeded(uint8_t port)
{
    uint32_t i;
    bool     retval = false;

    /*
     * Check whether the count for any fault type has exceeded the limit specified.
     */
    for (i = 0; i < FAULT_TYPE_COUNT; i++)
    {
        if (gl_app_fault_count[port][i] > gl_app_fault_retry_limit[port][i])
        {
            retval = true;
            break;
        }
    }

    return (retval);
}

/* Clear all fault counters associated with the specified port. */
static void app_clear_fault_counters(uint8_t port)
{
    /* Clear all fault counters on disconnect. */
    memset ((uint8_t *)gl_app_fault_count[port], 0, FAULT_TYPE_COUNT);
}

/* Generic routine that notifies the stack about recovery actions for a fault. */
static void app_handle_fault(uint8_t port, uint32_t fault_type)
{
    uint8_t reason = PD_HARDRES_REASON_VBUS_OVP;

    if (fault_type == FAULT_TYPE_VBUS_OCP)
    {
        reason = PD_HARDRES_REASON_VBUS_OCP;
    }

    /* Not checking for validity of port or fault_type as all calls to this function are internal. */
    dpm_set_fault_active(port);

    /* Update the fault count. */
#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
    /*
     * If current foldback mode is enabled, then we should recover from the
     * failure as the behaviour is expected. But we should still continue to
     * handle the fault with hard reset. So, we do not let the fault count 
     * to be incremented.
     */
    if ((fault_type != FAULT_TYPE_VBUS_UVP) ||
            (app_status[port].cur_fb_enabled == false))
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    {
        gl_app_fault_count[port][fault_type]++;
    }

    if (gl_app_fault_count[port][fault_type] < (gl_app_fault_retry_limit[port][fault_type] + 1))
    {
        dpm_clear_hard_reset_count(port);

        /*
         * Try a Hard Reset to recover from fault.
         * If not successful (not in PD contract), try Type-C error recovery.
         */
        if (dpm_send_hard_reset (port, reason) != CCG_STAT_SUCCESS)
        {
            dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        }
    }
    else
    {
        app_conf_for_faulty_dev_removal(port);
    }
}

#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

bool app_validate_configtable_offsets()
{
    uint8_t  port;

    for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if (defined(CCG3PA) || defined(CCG3PA2))
        if (get_pd_port_config(port)->pwr_tbl_offset == 0)
        {
            return false;
        }

        if(VBUS_CTRL_TYPE_P1 != pd_get_ptr_pwr_tbl(port)->fb_type)
        {
            return false;
        }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
        if (get_pd_port_config(port)->auto_cfg_tbl_offset == 0)
        {
            return false;
        }
        /* No retries for Auto Table. */
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

#if VBUS_OVP_ENABLE
        if (get_pd_port_config(port)->ovp_tbl_offset == 0)
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OVP] = pd_get_ptr_ovp_tbl(port)->retry_cnt;
#endif /* VBUS_OVP_ENABLE */

#if VBUS_OCP_ENABLE
        if (get_pd_port_config(port)->ocp_tbl_offset == 0)
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_OCP] = pd_get_ptr_ocp_tbl(port)->retry_cnt;
#endif /* VBUS_OCP_ENABLE */

#if VBUS_RCP_ENABLE
        if (get_pd_port_config(port)->rcp_tbl_offset == 0)
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_RCP] = pd_get_ptr_rcp_tbl(port)->retry_cnt;
#endif /* VBUS_OCP_ENABLE */

#if VBUS_UVP_ENABLE
        if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
        {
            if (get_pd_port_config(port)->uvp_tbl_offset == 0)
            {
                return false;
            }

            gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_UVP] = pd_get_ptr_uvp_tbl(port)->retry_cnt;
        }
#endif /* VBUS_UVP_ENABLE */

#if VBUS_SCP_ENABLE
        if (get_pd_port_config(port)->scp_tbl_offset == 0)
        {
            return false;
        }

        gl_app_fault_retry_limit[port][FAULT_TYPE_VBUS_SCP] = pd_get_ptr_scp_tbl(port)->retry_cnt;
#endif /* VBUS_SCP_ENABLE */

#if VCONN_OCP_ENABLE
        if (get_pd_port_config(port)->vconn_ocp_tbl_offset == 0)
        {
            return false;
        }

        /* No retries for VCONN OCP. */
#endif /* VCONN_OCP_ENABLE */

#if OTP_ENABLE
        if (get_pd_port_config(port)->otp_tbl_offset == 0)
        {
            return false;
        }

        /* No retries for OTP. */
#endif /* VBUS_OTP_ENABLE */

#if BATTERY_CHARGING_ENABLE
#if CCG_TYPE_A_PORT_ENABLE
        if (port == TYPE_A_PORT_ID)
        {
            if (get_pd_port_config(0)->type_a_chg_tbl_offset == 0)
            {
                return false;
            }
        }
        else
#endif /* CCG_TYPE_A_PORT_ENABLE */
        {
            if (get_pd_port_config(port)->chg_cfg_tbl_offset == 0)
            {
                return false;
            }
        }
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
        if (get_pd_port_config(port)->bat_chg_tbl_offset == 0)
        {
            return false;
        }
#endif /* POWER_BANK */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(port)->type_a_enable)
        {
            if (get_pd_port_config(port)->type_a_pwr_tbl_offset == 0)
            {
                return false;
            }
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (CCG_BB_ENABLE != 0)
        if (get_pd_port_config(port)->bb_tbl_offset == 0)
        {
            return false;
        }
#endif /* (CCG_BB_ENABLE != 0) */
    }

    return true;
}

#if OTP_ENABLE

/* Globals to keep track of OTP condition. */
static uint8_t g1_otp_therm_type[NO_OF_TYPEC_PORTS] = {APP_THERMISTOR_TYPE_ERROR};
static uint8_t gl_otp_debounce_count[NO_OF_TYPEC_PORTS] = {0};
static bool    gl_otp_debounce_active[NO_OF_TYPEC_PORTS] = {false};
static bool    gl_otp_port_disable[NO_OF_TYPEC_PORTS] = {false};

void app_otp_enable(uint8_t port)
{
    /*
     * If port is < NO_OF_TYPEC_PORTS, individual port specific data structure
     * instances are updated. Port = NO_OF_TYPEC_PORTS is used for updating
     * data strutures for alll port at 1 shot. This is used during
     * initialization.
     */
    if(NO_OF_TYPEC_PORTS == port)
    {
        /* This has been called from initialization. We need to do the data
           structure initialization for all port instances. */
        for(port = TYPEC_PORT_0_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            gl_otp_port_disable[port]    = false;
            gl_otp_debounce_active[port] = false;
            gl_otp_debounce_count[port]  = 0;
            g1_otp_therm_type[port]      = APP_THERMISTOR_TYPE_ERROR;
        }
    }
    else
    {
        /* Initialization for port specific instance. */
        gl_otp_port_disable[port]    = false;
        gl_otp_debounce_active[port] = false;
        gl_otp_debounce_count[port]  = 0;
        g1_otp_therm_type[port]      = APP_THERMISTOR_TYPE_ERROR;
    }
}

uint16_t app_otp_get_therm_volt (uint8_t port)
{
    uint8_t level = 0;
    uint16_t therm_volt = 0;

    if (0x0 == (pd_get_ptr_otp_tbl(port)->therm_type))
    {
        /* NTC Thermistor */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_NTC;
    }
    else if(0x01 == ((pd_get_ptr_otp_tbl(port)->therm_type) & 0x01))
    {
        /* PTC Thermistor */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_PTC;
    }
    else
    {
        /* Error in configuration */
        g1_otp_therm_type[port] = APP_THERMISTOR_TYPE_ERROR;
    }

    /* Configure GPIO. */
    hsiom_set_config(OTP_THERM_GPIO, HSIOM_MODE_AMUXA);
    /* Take ADC sample. */
    level = pd_adc_sample (port, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);
    therm_volt = pd_adc_level_to_volt (port, PD_ADC_ID_1, level);
    return therm_volt;
}

static void otp_debounce_cb(uint8_t port, timer_id_t id)
{
    uint32_t therm_volt;

    (void)id;

    /* Get thermistor voltage. */
    therm_volt = app_otp_get_therm_volt(port);

    /* If OT still exists. */
    if (
            ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
             (therm_volt <= pd_get_ptr_otp_tbl(port)->cutoff_volt)) ||
            (((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
              (therm_volt >= pd_get_ptr_otp_tbl(port)->cutoff_volt)))
       )
    {
        if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
        {
            /* OT detected. Check if it has crossed configured debounce time */
            gl_otp_debounce_count[port]++;
            if(gl_otp_debounce_count[port] > pd_get_ptr_otp_tbl(port)->debounce)
            {
                /* Valid OT detected. Disable the port now. */
                dpm_stop (port);
                gl_otp_port_disable[port] = true;
                gl_otp_debounce_active[port] = false;
            }
            else
            {
                /* Start debounce timer again. */
                timer_start (port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
        else
        {
            /* OTP feature can get disabled while debouncing is in progress.
             * There is no point continuing OTP checking in this case.
             * This is equivalent to no OTP condition.
             */
            app_otp_enable (port);
        }
    }
    else
    {
        /* OT condition doesn't exist anymore. Restart OTP detection. */
        app_otp_enable (port);
    }
}

void app_otp_check_temp(uint8_t port)
{
    uint16_t therm_volt;

    /*
     * This function will be called in a polling fashion after every expiry of
     * activity timer. Proceed and do necessary steps only if OTP protection
     * is enabled for this PD port.
     */
    if ((get_pd_port_config(port)->protection_enable & CFG_TABLE_OTP_EN_MASK))
    {
        /* Check thermistor voltage and see if it is below cut off voltage. */
        therm_volt = app_otp_get_therm_volt(port);

        /* If port is disabled then look for restart voltage. */
        if (gl_otp_port_disable[port] == true)
        {
            if (
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
                     (therm_volt >= pd_get_ptr_otp_tbl(port)->restart_volt)) ||
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
                     (therm_volt <= pd_get_ptr_otp_tbl(port)->restart_volt))
               )
            {
                /* OT condition doesn't exist anymore. Re-enable the port. */
                dpm_start(port);
                /* Reset the port specific data structures. */
                app_otp_enable(port);
            }
        }
        /* If port not disabled, look for cut off voltage. */
        else if (gl_otp_debounce_active[port] == false)
        {
            /*
             * Compare therm volt and cut off voltage and start OT debounce,
             * if required. Thermistor type value will decide the comparison
             * algorithm.
             */
            if (
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_NTC) &&
                     (therm_volt <= pd_get_ptr_otp_tbl(port)->cutoff_volt)) ||
                    ((g1_otp_therm_type[port] == APP_THERMISTOR_TYPE_PTC) &&
                     (therm_volt >= pd_get_ptr_otp_tbl(port)->cutoff_volt))
               )
            {
                gl_otp_debounce_active[port] = true;
                gl_otp_debounce_count[port] = 0;
                /* Start OTP debounce timer. */
                timer_start (port, OTP_DEBOUNCE_TIMER_ID, OTP_DEBOUNCE_PERIOD, otp_debounce_cb);
            }
        }
    }
}
#endif /* OTP_ENABLE */

#ifdef CCG3PA

bool ccg_app_is_idle(void)
{
    /*
     * If activity timer timeout event is not pending, CCG is idle and system can
     * enter low power mode.
     */
    return !ccg_activity_timer_timeout;
}

void ccg_activity_timer_cb(uint8_t instance, timer_id_t id)
{
    (void)instance;
    (void)id;
    /*
     * Activity timer expired. Generate an event so that CCG periodic checks
     * can run.
     */
    ccg_activity_timer_timeout = true;
}
#if (CCG_LOAD_SHARING_ENABLE)
    extern uint8_t gl_stop_ls[NO_OF_TYPEC_PORTS];
#endif
void ccg_app_task(uint8_t port)
{
#if CCG_CABLE_COMP_ENABLE
    ccg_cable_comp_task(port);
#endif /* CCG_CABLE_COMP_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
    ccg_sensor_debounce_task();
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
    if (false == gl_stop_ls[port])
    {
        ccg_ls_task(port);
    }
#endif /* CCG_LOAD_SHARING_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
    ccg_power_throttle_task(port);
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

    /* Check VBATT, OTP and TYPE-A current consumption if activity timer has timed out. */
    if (ccg_activity_timer_timeout == true)
    {
#if (POWER_BANK == 1)
        pb_bat_monitor ();
#endif /* (POWER_BANK == 1) */

#if (CCG_TYPE_A_PORT_ENABLE == 1)
        if (get_pd_port_config(0)->type_a_enable)
        {
            type_a_detect_disconnect ();
        }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (OTP_ENABLE == 1)
        app_otp_check_temp (port);
#endif /* OTP_ENABLE */

#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) || (CCG_LOAD_SHARING_ENABLE == 1))
        ccg_activity_timer_timeout = false;
        timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
                ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ ENABLE == 1) */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
        ccg_sensor_check();
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */
    }
}

void ccg_app_task_init(void)
{
#if (POWER_BANK == 1)
    pb_task_init ();
#endif /* (POWER_BANK == 1) */

#if OTP_ENABLE
    /* Enable OTP. */
    app_otp_enable (NO_OF_TYPEC_PORTS);
#endif /* OTP_ENABLE */

    /*
     * Start CCG activity timer. This timer is periodically used to monitor
     * battrey voltage (in power bank application), TYPE-A current consumption
     * and OTP.
     */
#if ((OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) || \
    (CCG_TEMP_BASED_VOLTAGE_THROTTLING == 1) || (CCG_VIN_BASED_VOLTAGE_THROTTLING == 1) || (CCG_LOAD_SHARING_ENABLE == 1))
    ccg_activity_timer_timeout = false;
    timer_start (0, CCG_ACTIVITY_TIMER_ID, CCG_ACTIVITY_TIMER_PERIOD,
            ccg_activity_timer_cb);
#endif /* (OTP_ENABLE == 1) || (POWER_BANK == 1) || (CCG_TYPE_A_PORT_ENABLE == 1) */
}

#endif /* CCG3PA */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
static bool app_is_vdm_task_ready(uint8_t port)
{
    /* Assume cable discovery finished when device is UFP. */
    bool retval = true;

#if DFP_ALT_MODE_SUPP

    const dpm_status_t *dpm_stat = dpm_get_info (port);

    /* This check only makes sense for DFP. */
    if (gl_dpm_port_type[port] != PRT_TYPE_UFP)
    {

#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
        /* Don't proceed with alternate mode if DR_SWAP is pending. */
        if (app_status[port].dr_swap_pending)
        {
            return false;
        }
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */
        /*
         * Set the cable discovered flag if:
         * 1. Cable discovery is disabled.
         * 2. EMCA present flag in DPM is set.
         */
        if ((dpm_stat->cbl_dsc == false) || (dpm_stat->emca_present != false))
        {
            app_status[port].cbl_disc_id_finished = true;
        }

        /* Return the status of Cable discovered flag. */
        retval = app_status[port].cbl_disc_id_finished;
    }

#endif /* DFP_ALT_MODE_SUPP */

    return retval;
}
#endif /* ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP)) */

/* Timer used to re-enable the PD port after a fault. */
static void fault_recovery_timer_cb(uint8_t port, timer_id_t id)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        if ((app_status[port].fault_status & APP_PORT_VBUS_DROP_WAIT_ACTIVE) != 0)
        {
            app_status[port].fault_status &= ~APP_PORT_VBUS_DROP_WAIT_ACTIVE;

            /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
            pd_typec_rd_enable (port);
            period = APP_FAULT_RECOVERY_MAX_WAIT;
        }
        else
        {
            /*
             * If VBus is not detected, we can re-enable the PD port.
             */
            app_status[port].fault_status &= ~APP_PORT_DISABLE_IN_PROGRESS;
            dpm_clear_fault_active(port);

            pd_typec_dis_rd(port, CC_CHANNEL_1);
            pd_typec_dis_rd(port, CC_CHANNEL_2);
            dpm_start(port);

            /* Return without restarting the timer. */
            return;
        }
    }

    /* Restart the timer to check VBus and Rp status again. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

/* Callback used to get notification that PD port disable has been completed. */
static void app_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    uint16_t period = APP_FAULT_RECOVERY_TIMER_PERIOD;

    if (
            (vbus_is_present(port, VSAFE_0V, 0) == false)
       )
    {
        /* VBus has already been removed. Enable the Rd termination to check for physical detach. */
        pd_typec_rd_enable (port);
        period = APP_FAULT_RECOVERY_MAX_WAIT;
    }
    else
    {
        /* VBus has not been removed. Start a task which waits for VBus removal. */
        app_status[port].fault_status |= APP_PORT_VBUS_DROP_WAIT_ACTIVE;
    }

    /* Provide a delay to allow VBus turn-on by port partner and then enable the port. */
    timer_start (port, APP_FAULT_RECOVERY_TIMER, period, fault_recovery_timer_cb);
}

ccg_status_t app_disable_pd_port(uint8_t port, dpm_typec_cmd_cbk_t cbk)
{
    ccg_status_t retval = CCG_STAT_SUCCESS;

    if (timer_is_running (port, APP_FAULT_RECOVERY_TIMER))
    {
        /* If the HPI Master is asking us to disable the port, make sure all fault protection state is cleared. */
        app_status[port].fault_status &= ~(
                APP_PORT_VBUS_DROP_WAIT_ACTIVE | APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS |
                APP_PORT_VCONN_FAULT_ACTIVE | APP_PORT_V5V_SUPPLY_LOST);
        timer_stop(port, APP_FAULT_RECOVERY_TIMER);
        pd_typec_dis_rd(port, CC_CHANNEL_1);
        pd_typec_dis_rd(port, CC_CHANNEL_2);
        cbk(port, DPM_RESP_SUCCESS);
    }
    else
    {
        /* Just pass the call on-to the stack. */
        if (dpm_get_info(port)->dpm_enabled)
        {
            retval = dpm_typec_command(port, DPM_CMD_PORT_DISABLE, cbk);
        }
        else
        {
            cbk(port, DPM_RESP_SUCCESS);
        }
    }

    return retval;
}

#if CCG_REV3_HANDLE_BAD_SINK
/* Bad sink timer callback */
void app_bad_sink_timeout_cbk(uint8_t port, timer_id_t id)
{
    /* Do a HARD_RESET if PD connected device */
    if(dpm_get_info(port)->pd_connected == true)
    {
        gl_bad_sink_timeout_status[port] = true;
        /*
         * Try a Hard Reset to recover from fault.
         * If not successful (not in PD contract), try Type-C error recovery.
         */
        if (dpm_send_hard_reset (port, PD_HARDRES_REASON_CONTRACT_ERROR) != CCG_STAT_SUCCESS)
        {
            dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
        }
    }
}
#endif /* CCG_REV3_HANDLE_BAD_SINK */

uint8_t app_task(uint8_t port)
{
    /*
     * If SINK fault handling is pending, queue a port disable command.
     */
    if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
    {
        if (dpm_typec_command (port, DPM_CMD_PORT_DISABLE, app_port_disable_cb) != CCG_STAT_BUSY)
        {
            app_status[port].fault_status &= ~APP_PORT_SINK_FAULT_ACTIVE;
            app_status[port].fault_status |= APP_PORT_DISABLE_IN_PROGRESS;
        }
    }

#if CCG_REV3_HANDLE_BAD_SINK
    /* Check for bad sink timeout.
     * If the PD contract was not completed for APP_BAD_SINK_TIMEOUT_TIMER_PERIOD,
     * a hard reset was sent. Downgrade the PD revision from REV3 to REV2
     * which is expected by few sinks to complete the contracts. */
    if((gl_bad_sink_timeout_status[port] == true) && (dpm_get_info(port)->pe_fsm_state == PE_FSM_SRC_STARTUP)) 
    {
        gl_bad_sink_timeout_status[port] = false;
        dpm_downgrade_pd_port_rev(port);
    }
#endif /* CCG_REV3_HANDLE_BAD_SINK */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* If VDM processing is allowed */
    if (app_status[port].vdm_task_en != false)
    {
        /* Wait for cable discovery completion before going on Alt. Modes. */
        if (app_is_vdm_task_ready (port))
        {
            vdm_task_mngr (port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if NCP_CLIND_ENABLE
    if(port == TYPEC_PORT_0_IDX)
    {
        Clind_OCP_Check(port);
    }
#endif /* NCP_CLIND_ENABLE */

#if (CCG_BB_ENABLE != 0)
    if (bb_is_present(port) != false)
    {
        bb_task(port);
    }
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
    uint8_t i;
    for (i=0; i < NO_OF_BC_PORTS; i++)
    {
        bc_fsm (i);
    }
#endif /* BATTERY_CHARGING_ENABLE */

#if RIDGE_SLAVE_ENABLE
    ridge_slave_task();
#endif /* RIDGE_SLAVE_ENABLE */

#ifdef CCG3PA
    /* Run polling tasks of CCG. */
    ccg_app_task (port);
#endif /* CCG3PA */

    return true;
}

#define AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN     (0x02u);

void app_type_c_enter_sleep(uint8_t port)
{
#if ((defined(CCG3PA)) || (defined(CCG3PA2)))
    /*
     * Configure Refgen block to use Deepsleep Reference input instead of Bandgap
     * reference which is not available in deep sleep.
     */
    PDSS->refgen_0_ctrl &= ~(PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
        PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);

    /* Switch to using refgen_2_ctrl SEL7 for EA shunt regulator reference. */
    PDSS->amux_ctrl |= AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN;
#endif /* ((defined(CCG3PA)) || (defined(CCG3PA2))) */
}

void app_type_c_wakeup()
{
#if ((defined(CCG3PA)) || (defined(CCG3PA2)))
    /* Switch to using bandgap for EA shunt regulator reference. */
    PDSS->amux_ctrl &= ~AMUX_CTRL_EA_TOP_REFGEN_SEL7_EN;
    /*
     * Configure Refgen block to use Bandgap Reference input instead of Deep sleep
     * reference.
     */
    PDSS->refgen_0_ctrl |= (PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
        PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);
#endif /* ((defined(CCG3PA)) || (defined(CCG3PA2))) */
}

bool app_type_c_sleep_allowed(void)
{
    bool out = true;
    uint8_t i = 0;

#if (defined(CCG3PA) || defined(CCG3PA2))
    const dpm_status_t *dpm;

    /*
     * Deepsleep mode operation can only be supported when un-attached.
     * When attached, the references and the CSA block requires to be
     * active.
     */
    for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
    {
        dpm = dpm_get_info(i);

        if (dpm->attach == true)
        {
            out = false;
            break;
        }
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    if (out == true)
    {
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
        {
            app_type_c_enter_sleep (i);
        }
    }

    return out;
}

bool app_sleep(void)
{
    bool stat = true;
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_BB_ENABLE
        if (!bb_enter_deep_sleep(port))
        {
            stat = false;
            break;
        }
#endif /* CCG_BB_ENABLE */

#ifdef CCG3PA
        /*
         * Check if CCG polling tasks are not pending to be serviced and system can enter
         * low power mode.
         */
        if (ccg_app_is_idle () == false)
        {
            stat = false;
            break;
        }
#endif /* CCG3PA */

        /* Don't go to sleep while CC/SBU fault handling is pending. */
        if ((app_status[port].fault_status & APP_PORT_SINK_FAULT_ACTIVE) != 0)
        {
            stat = false;
            break;
        }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
        if (!is_vdm_task_idle(port))
        {
            stat = false;
            break;
        }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (DP_UFP_SUPP) && (CCG_HPD_RX_ENABLE)
        /* CDT 245126 workaround: Check if HPD RX Activity timer is running.
         * If yes, don't enter deep sleep. */
        if (!is_hpd_rx_state_idle (port))
        {
            stat = false;
            break;
        }
#endif /* DP_UFP_SUPP && CCG_HPD_RX_ENABLE */
    }

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    if (stat)
    {
        for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
        {
            /* Prepare for deep-sleep entry. */
            alt_mode_mngr_sleep(port);
        }
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

    return stat;
}

void app_wakeup(void)
{
#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        alt_mode_mngr_wakeup (port);
    }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

#if (CCG_BB_ENABLE != 0)
/* Alternate mode entry timeout callback function. */
static void ame_tmr_cbk(uint8_t port, timer_id_t id)
{
    (void)id;

    /* Alternate modes are reset in vdm_task_mngr_deinit(). */
    bb_enable(port, BB_CAUSE_AME_TIMEOUT);
}
#endif /* (CCG_BB_ENABLE != 0) */

#if CCG_PD_REV3_ENABLE

void extd_msg_cb(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr)
{
    static pd_ams_type ams_type[NO_OF_TYPEC_PORTS];
    (void)pkt_ptr;
    if(resp == RES_RCVD){
        dpm_set_chunk_transfer_running(port, ams_type[port]);
    }
    if(resp == CMD_SENT){
        ams_type[port] = dpm_get_info(port)->non_intr_response;
    }
}

/* Global variable used as dummy data buffer to send Chunk Request messages. */
static uint32_t gl_extd_dummy_data;

static bool app_extd_msg_handler(uint8_t port, pd_packet_extd_t *pd_pkt_p)
{
    /* If this is a chunked message which is not complete, send another chunk request. */
    if ((pd_pkt_p->hdr.hdr.chunked == true) && (pd_pkt_p->hdr.hdr.data_size >
               ((pd_pkt_p->hdr.hdr.chunk_no + 1) * MAX_EXTD_MSG_LEGACY_LEN)))
    {
        dpm_pd_cmd_buf_t extd_dpm_buf;

        extd_dpm_buf.cmd_sop = pd_pkt_p->sop;
        extd_dpm_buf.extd_type = pd_pkt_p->msg;
        extd_dpm_buf.extd_hdr.val = 0;
        extd_dpm_buf.extd_hdr.extd.chunked = true;
        extd_dpm_buf.extd_hdr.extd.request = true;
        extd_dpm_buf.extd_hdr.extd.chunk_no = pd_pkt_p->hdr.hdr.chunk_no + 1;
        extd_dpm_buf.dat_ptr = (uint8_t*)&gl_extd_dummy_data;
        extd_dpm_buf.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;

        /* Send next chunk request */
        dpm_pd_command_ec(port, DPM_CMD_SEND_EXTENDED,
                &extd_dpm_buf, extd_msg_cb);
    }
    else
    {
       
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        /* If macro is enabled - allow handling the requests from solution space. */
        return false;
#else        
        /*
         * Don't send any response to response messages. Handling here instead of in the stack so that
         * these messages can be used for PD authentication implementation.
         */
        if ((pd_pkt_p->msg != EXTD_MSG_SECURITY_RESP) && (pd_pkt_p->msg != EXTD_MSG_FW_UPDATE_RESP))
        {
            /* Send Not supported message */
            dpm_pd_command_ec(port, DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
        }
#endif /* CCG_HANDLE_EXT_MSG_IN_SOL */
    }
    
    return true;
}
#endif /* CCG_PD_REV3_ENABLE */

/* This function stops PD operation and configures type c
 * to look for detach of faulty device */
void app_conf_for_faulty_dev_removal(uint8_t port)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    if ((!dpm_stat->attach) || (dpm_stat->cur_port_role == PRT_ROLE_SINK))
    {
        /* Set flag to trigger port disable sequence. */
        app_status[port].fault_status |= APP_PORT_SINK_FAULT_ACTIVE;
    }

    /* Stop PE */
    dpm_pe_stop(port);

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
    /* Make sure any alternate mode related state is cleared. */
    vdm_task_mngr_deinit (port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
}

void app_update_bc_src_support(uint8_t port, uint8_t enable)
{
#if BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6))
    app_status[port].bc_12_src_disabled = (bool)(!enable);
    if (!enable)
    {
        bc_stop(port);
    }
#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6)) */
}

void app_update_sys_pwr_state(uint8_t state)
{
#if BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6))
    uint8_t i = 0;

    /*
     * If the port is currently functioning as DCP and system is going into S0 state; go through Type-C error
     * recovery. If the port is currently functioning as CDP, we do not change current state of the port.
     * Port will function as DCP on next Type-C connection.
     */
    if (
            (state == 0) && (state != hpi_get_sys_pwr_state()) &&
            (app_status[i].bc_12_src_disabled == 0)
       )
    {
#if CCG_PD_DUALPORT_ENABLE
        for (i = 0; i < NO_OF_TYPEC_PORTS; i++)
#endif /* CCG_PD_DUALPORT_ENABLE */
        {
            if (
                    (dpm_get_info(i)->attach) &&
                    (dpm_get_info(i)->cur_port_role == PRT_ROLE_SOURCE) &&
                    ((alt_mode_get_status(i) & 0x7F) == 0)
               )
            {
                dpm_typec_command (i, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
            }
        }
    }
#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5) || defined(CCG6)) */
}

#if BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5))

/* Function to start the CCG5 BC 1.2 source state machine. */
void app_bc_12_sm_start(uint8_t port)
{
    /* Enable CDP is system power state is S0, otherwise enable DCP. */
#if CCG_HPI_ENABLE
    if (hpi_get_sys_pwr_state () == 0)
#endif /* CCG_HPI_ENABLE */
    {
        ccg_bc_cdp_en(port);
    }
#if CCG_HPI_ENABLE
    else
    {
        ccg_bc_dcp_en(port);
    }
#endif /* CCG_HPI_ENABLE */
}

#endif /* BATTERY_CHARGING_ENABLE && (defined(CCG5C) || defined(CCG5)) */

uint32_t get_bat_status[NO_OF_TYPEC_PORTS];

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
static void src_disable_cbk(uint8_t port)
{
    /* Dummy callback used to ensure VBus discharge happens on CC/SBU OVP. */
}
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if  ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))

/* Variable storing current preference for data role. */
volatile uint8_t app_pref_data_role[NO_OF_TYPEC_PORTS];

#if PR_SWAP_ENABLE
/* Variable storing current preference for power role. */
volatile uint8_t app_pref_power_role[NO_OF_TYPEC_PORTS];

static void app_initiate_pr_swap (
        uint8_t port, 
        timer_id_t id);

static void app_drp_pr_swap_resp_cb (
        uint8_t            port,
        resp_status_t      resp,
        const pd_packet_t *pkt_ptr)
{
    app_status_t *app_stat = &app_status[port];
    if(resp == RES_RCVD)
    {
        if (pkt_ptr->hdr.hdr.msg_type == CTRL_MSG_WAIT)
        {
            app_stat->pr_swap_count++;
            if (app_stat->pr_swap_count < APP_MAX_SWAP_ATTEMPT_COUNT)
            {
                timer_start(port, APP_INITIATE_PR_SWAP_TIMER, APP_INITIATE_PR_SWAP_TIMER_PERIOD, app_initiate_pr_swap);
            }
            else
            {
                app_stat->pr_swap_pending = false;
            }
        }
        else
        {
            app_stat->pr_swap_pending = false;
        }
    }
    else if ((resp == CMD_FAILED) || (resp == SEQ_ABORTED) || (resp == RES_TIMEOUT))
    {
        if (
                (dpm_get_info(port)->attach) &&
                (dpm_get_info(port)->cur_port_role != app_pref_power_role[port]) &&
                (app_pref_power_role[port] < PRT_DUAL)
           )
        {
            timer_start(port, APP_INITIATE_PR_SWAP_TIMER, APP_INITIATE_PR_SWAP_TIMER_PERIOD, app_initiate_pr_swap);
        }
    }
}

static void app_initiate_pr_swap (
        uint8_t port, 
        timer_id_t id)
{
    dpm_pd_cmd_buf_t pd_cmd_buf;
    pd_cmd_buf.cmd_sop = SOP;

    if (
            (dpm_get_info(port)->cur_port_role != app_pref_power_role[port]) &&
            (app_pref_power_role[port] < PRT_DUAL) &&
            (app_status[port].pr_swap_pending)
       )
    {
        if(dpm_pd_command(port, DPM_CMD_SEND_PR_SWAP, &pd_cmd_buf, app_drp_pr_swap_resp_cb) != CCG_STAT_SUCCESS)
        {
            timer_start(port, APP_INITIATE_PR_SWAP_TIMER, APP_INITIATE_PR_SWAP_TIMER_PERIOD, app_initiate_pr_swap);
        }
    }
}
#endif /* PR_SWAP_ENABLE */

static void app_initiate_dr_swap (
        uint8_t port, 
        timer_id_t id);

static void app_drp_dr_swap_resp_cb (
        uint8_t            port,
        resp_status_t      resp,
        const pd_packet_t *pkt_ptr)
{
    app_status_t *app_stat = &app_status[port];

    if (resp == RES_RCVD)
    {
        if (pkt_ptr->hdr.hdr.msg_type == CTRL_MSG_WAIT)
        {
            /* Set a limit on SWAP attempts made when the port partner is
             * responding with WAIT. */
            app_stat->dr_swap_count++;
            if (app_stat->dr_swap_count < APP_MAX_SWAP_ATTEMPT_COUNT)
            {
                timer_start(port, APP_INITIATE_DR_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_dr_swap);
            }
            else
            {
                app_stat->dr_swap_pending = false;
            }
        }
        else
        {
            app_stat->dr_swap_pending = false;
        }
    }
    else if ((resp == CMD_FAILED) || (resp == SEQ_ABORTED) || (resp == RES_TIMEOUT))
    {
        if (
                (dpm_get_info(port)->attach) &&
                (dpm_get_info(port)->cur_port_type != app_pref_data_role[port]) &&
                (app_pref_data_role[port] < PRT_TYPE_DRP)
           )
        {
            timer_start(port, APP_INITIATE_DR_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_dr_swap);
        }
    }
}

static void app_initiate_dr_swap (
        uint8_t port, 
        timer_id_t id)
{
    dpm_pd_cmd_buf_t pd_cmd_buf;

    pd_cmd_buf.cmd_sop = SOP;
    if (
            (dpm_get_info(port)->cur_port_type != app_pref_data_role[port]) &&
            (app_pref_data_role[port] < PRT_TYPE_DRP) &&
            (app_status[port].dr_swap_pending) &&
            (app_status[port].alt_mode_entered == false)
       )
    {
        if (dpm_pd_command(port, DPM_CMD_SEND_DR_SWAP, &pd_cmd_buf, app_drp_dr_swap_resp_cb) != CCG_STAT_SUCCESS)
            timer_start(port, APP_INITIATE_DR_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_dr_swap);
    }
}

static void app_contract_handler (
        uint8_t port)
{
    app_status_t *app_stat = &app_status[port];
#if PR_SWAP_ENABLE
    if (
            (dpm_get_info(port)->cur_port_role != app_pref_power_role[port]) &&
            (app_pref_power_role[port] < PRT_DUAL)
       )
    {
        timer_start(port, APP_INITIATE_PR_SWAP_TIMER, APP_INITIATE_PR_SWAP_TIMER_PERIOD, app_initiate_pr_swap);
    }
    else
    {
        app_stat->pr_swap_pending = false;
    }
#endif /* PR_SWAP_ENABLE */
    if (
            (dpm_get_info(port)->cur_port_type != app_pref_data_role[port]) &&
            (app_pref_data_role[port] < PRT_TYPE_DRP)
       )
    {
        timer_start(port, APP_INITIATE_DR_SWAP_TIMER, APP_INITIATE_DR_SWAP_TIMER_PERIOD, app_initiate_dr_swap);
    }
    else
    {
        app_stat->dr_swap_pending = false;
    }
}
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */

void g_SrcCapsDecode(grlSrcCapsStructVar_t * aSrcCaps)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
    
    aSrcCaps->gNoOfPDOs = dpm_stat->src_cap_p->len ;
    
    for(uint8_t i = 0; i < aSrcCaps->gNoOfPDOs; ++i)
    {
        aSrcCaps->PDO[i].val = dpm_stat->src_cap_p->dat[i].val;      
        /** TBD : If this doesnt work than need to decode each and every PDOs current and voltage values for comparing*/    
    }

}
static uint8_t gl_app_previous_polarity[NO_OF_TYPEC_PORTS];

void app_event_handler(uint8_t port, app_evt_t evt, const void* dat)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    pd_packet_extd_t *pd_pkt_p;
    const app_req_status_t* result;
    const pd_contract_info_t* contract_status;
    bool  skip_soln_cb = false;
    bool  hardreset_cplt = false;
    bool  typec_only = false;
    const dpm_status_t *dpm_stat = dpm_get_info(port);
#if CCG_PD_REV3_ENABLE
    pd_do_t alert_ado;
#endif /* CCG_PD_REV3_ENABLE */
    //gBufLog(false,evt);
    switch(evt)
    {
        case APP_EVT_TYPEC_STARTED:
#if (defined(CCG3PA) || defined(CCG3PA2))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD (i.e not
             * in dead battery), disable internal VBUS regulator.
             */
            if (dpm_stat->dead_bat == false)
            {
                pd_hal_disable_vreg (port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

            /* Initialize the MUX to its default settings (isolate). */
            mux_ctrl_init (port);
            app_status[port].vdm_prcs_failed = false;
            break;

        case APP_EVT_TYPEC_ATTACH:
#if CCG_REV3_HANDLE_BAD_SINK
            /* Start bad sink timer */
            timer_start(port, APP_BAD_SINK_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, app_bad_sink_timeout_cbk);
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            /* This will also enable the USB (DP/DM) MUX where required. */
            set_mux (port, MUX_CONFIG_SS_ONLY, 0);

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
            /* Clear all fault counters if we have seen a change in polarity from previous connection. */
            if (dpm_stat->polarity != gl_app_previous_polarity[port])
            {
                app_clear_fault_counters(port);
            }
#endif /* (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE) */
            gl_app_previous_polarity[port] = dpm_stat->polarity;
            isAttachInterrupt = true;  
            break;

        case APP_EVT_CONNECT:
            app_status[port].vdm_prcs_failed = false;
            app_status[port].cbl_disc_id_finished = false;

#if (CCG_BB_ENABLE != 0)
            /* Enable the AME timer on attach if in sink mode. */
            if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
            {
                timer_start(port, APP_AME_TIMEOUT_TIMER, APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
            }
#endif /* (CCG_BB_ENABLE != 0) */

#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
            /*
             * Assume that PR_SWAP and DR_SWAP are pending. The actual status
             * will be updated on contract completion.
             */
#if PR_SWAP_ENABLE
            app_status[port].pr_swap_pending = true;
            app_status[port].pr_swap_count   = 0;
#endif /* PR_SWAP_ENABLE */
            app_status[port].dr_swap_pending = true;
            app_status[port].dr_swap_count   = 0;
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */
            break;

        case APP_EVT_HARD_RESET_COMPLETE:
            app_status[port].cbl_disc_id_finished = false;
            hardreset_cplt = true;
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_SENT:
        case APP_EVT_PE_DISABLED:
            typec_only = ((dpm_stat->pd_connected == false) || (evt == APP_EVT_PE_DISABLED));
            /* Intentional fall-through. */

        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_VBUS_PORT_DISABLE:
        case APP_EVT_DISCONNECT:
        case APP_EVT_TYPE_C_ERROR_RECOVERY:
            
            i2cBuf[2] = 0x00;
            i2cBuf[3]  = 0x00;
            //i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
            //i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
            //i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);
            
            PD_BC_i2cBufHandler(INTR_BUF_CLR);
            PD_BC_i2cBufHandler(INTR_DETACH);
            g_PdssGPIOIntrHandler(INTR_SET);
            
#if CCG_REV3_HANDLE_BAD_SINK
            if (evt == APP_EVT_DISCONNECT)
            {
                /* Stop bad sink timer */
                timer_stop(port, APP_BAD_SINK_TIMEOUT_TIMER);
            }
#endif /* CCG_REV3_HANDLE_BAD_SINK */
#if (defined(CCG3PA) || defined(CCG3PA2))
#if (POWER_BANK == 1)
            /*
             * In power bank case, if device is powered by external VDDD, VDDD gets
             * shorted to VBUS_IN line. This shall result connecting VDDD to the
             * Type-C VBUS line. This also includes cases where we start as dead
             * dead battery device and then get charged. So if any time VBUS has to
             * be removed in course of PD / Type-C state machine, ensure that internal
             * VBUS regulator is disabled. In event of dead battery, this shall lead
             * to device reset. This is the safest recovery path. CDT 276535.
             *
             * This code can be removed if the VBATT monitoring can be done
             * continously. But this code can still be in place to avoid any
             * corner case handling.
             */

            /* Do this only on disconnect and type-C error recovery. */
            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_TYPE_C_ERROR_RECOVERY))
            {
                pd_hal_disable_vreg(port);
            }
#endif /* (POWER_BANK == 1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
            vdm_task_mngr_deinit (port);

#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
            timer_stop (port, APP_CBL_DISC_TRIGGER_TIMER);
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */

#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

            /*
             * Re-enable MUX in USB mode if hard reset has been completed.
             */
            if (hardreset_cplt)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            else
            {
                /*
                 * Isolate the data lines if this is a PD connection.
                 */
                if (!typec_only)
                {
                    set_mux (port, MUX_CONFIG_ISOLATE, 0);
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                }
            }

#if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
            if(evt == APP_EVT_TYPE_C_ERROR_RECOVERY)
            {
                /* Clear port-in-fault flag if all fault counts are within limits. */
                if (!app_port_fault_count_exceeded(port))
                {
                    if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                    {
                        dpm_clear_fault_active(port);
                    }
                }
            }
#endif /* VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE */

            if ((evt == APP_EVT_DISCONNECT) || (evt == APP_EVT_VBUS_PORT_DISABLE))
            {
#if REGULATOR_REQUIRE_STABLE_ON_TIME
                /* Disable the regulator on port disconnect */
                REGULATOR_DISABLE();
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

                /* Cleanup the PD block states on disconnect. */
                pd_hal_cleanup(port);

                if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                {
                    dpm_clear_fault_active(port);
                }

        #if VCONN_OCP_ENABLE
                /* Clear the VConn fault status. */
                app_status[port].fault_status &= ~APP_PORT_VCONN_FAULT_ACTIVE;
        #endif /* VCONN_OCP_ENABLE */

        #if (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE)
                app_clear_fault_counters(port);
        #endif /* (VBUS_OCP_ENABLE | VBUS_SCP_ENABLE | VBUS_OVP_ENABLE| VBUS_UVP_ENABLE) */

        #if RIDGE_SLAVE_ENABLE
                /* Clear the error status. */
                ridge_slave_update_ocp_status(port, false);
        #endif /* RIDGE_SLAVE_ENABLE */
            }

            /* Disconnect and Port Disable events are handled above. */
            if (evt == APP_EVT_HARD_RESET_SENT)
            {
                /* Ensure Fault active condition is cleared, if one was detected. */
                if ((app_status[port].fault_status & APP_PORT_DISABLE_IN_PROGRESS) == 0)
                {
                    dpm_clear_fault_active(port);
                }
            }

#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
            if (
                    (evt == APP_EVT_HARD_RESET_COMPLETE) ||
                    (evt == APP_EVT_TYPE_C_ERROR_RECOVERY) ||
                    (evt == APP_EVT_DISCONNECT)
               )
            {
                /* Stop the DR-Swap and PR-Swap trigger timers.  Assume that
                 * PR_SWAP and DR_SWAP are pending. The actual status will be
                 * updated on contract completion.
                 */
#if PR_SWAP_ENABLE
                timer_stop (port, APP_INITIATE_PR_SWAP_TIMER);
                app_status[port].pr_swap_pending = true;
                app_status[port].pr_swap_count   = 0;
#endif /* PR_SWAP_ENABLE */

                timer_stop (port, APP_INITIATE_DR_SWAP_TIMER);
                app_status[port].dr_swap_pending = true;
                app_status[port].dr_swap_count   = 0;
            }
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */

#if APP_PPS_SINK_SUPPORT
            /* Make sure the PPS re-negotiation task is stopped. */
            app_pps_sink_disable (port);
#endif /* APP_PPS_SINK_SUPPORT */

            AUG_TIMER_Stop();/**Turning off timer if we get detach/hardreset */
            timer_stop(0,GRL_APP_SOP1_TIMER); /**Stopping SOP1 timer in detach event*/
            /** Pranay,27Sept'19,to resolve issue i.e., after requesting PDOx if we send Attach/Detach, 
            data msg request packet was being sent with previous requested PDOx so Added this flag here to 
            avoid requesting PDOx after Detach/Attach*/
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = false;
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            
#if ONLY_PD_SNK_FUNC_EN            
            /**Reverting back the Message ID to default 0 in detach as SOP1 sequence will start from zero again after detach*/
            g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID = 0;
            g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
#endif/*ONLY_PD_SNK_FUNC_EN*/           
            /** Configuring to Default PDO incase of manual Attach/Detach*/
            if(g_Struct_Ptr->RequestPacketConfig.gCustomConfig != gAptivDetachOverride)
            {
                g_Struct_Ptr->RequestPacketConfig.gPDO_Index = 1;
                g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = 0x32;
                g_Struct_Ptr->RequestPacketConfig.gOperating_I = 0x0A;
                g_Struct_Ptr->RequestPacketConfig.gPDO_Type = PDO_FIXED_SUPPLY;
            }
            break;

        case APP_EVT_EMCA_DETECTED:
        case APP_EVT_EMCA_NOT_DETECTED:
            app_status[port].cbl_disc_id_finished = true;
            app_status[port].vdm_prcs_failed = false;

            /*
               Update the MUX settings with new cable information once EMCA detection is completed, only
               if we are still in USB mode.
             */
            if (get_mux_state(port) == MUX_CONFIG_SS_ONLY)
            {
                set_mux (port, MUX_CONFIG_SS_ONLY, 0);
            }
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            result = (const app_req_status_t*)dat ;
            if(*result == REQ_ACCEPT)
            {
#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
                app_status[port].dr_swap_pending = false;
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /* Device data role changed. Reset alternate mode layer. */
                alt_mode_layer_reset(port);
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */

#if (CCG_BB_ENABLE != 0)
                /* Start tAME Timer to enable BB functionality */
                if (gl_dpm_port_type[port] == PRT_TYPE_UFP)
                {
                    timer_start(port, APP_AME_TIMEOUT_TIMER , APP_AME_TIMEOUT_TIMER_PERIOD, ame_tmr_cbk);
                }
                else
                {
                    timer_stop (port, APP_AME_TIMEOUT_TIMER);
                }
#endif /* (CCG_BB_ENABLE != 0) */
            }
            break;

        case APP_EVT_VENDOR_RESPONSE_TIMEOUT:
            /* If the APP layer is going to retry the VDM, do not send the event. */
            if (app_status[port].vdm_retry_pending)
                skip_soln_cb = true;
            break;

        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
        
            PD_BC_i2cBufHandler(INTR_BUF_CLR);
           
            contract_status = (pd_contract_info_t*)dat;
           
#if CCG_REV3_HANDLE_BAD_SINK
            if(contract_status->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL)
            {
                /* Stop bad sink timer */
                timer_stop(port, APP_BAD_SINK_TIMEOUT_TIMER);
            }
#endif /* CCG_REV3_HANDLE_BAD_SINK */
            /* Set VDM version based on active PD revision. */
#if CCG_PD_REV3_ENABLE
            if (dpm_stat->spec_rev_sop_live >= PD_REV3)
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV3;
            }
            else
#endif /* CCG_PD_REV3_ENABLE */
            {
                app_status[port].vdm_version = STD_VDM_VERSION_REV2;
            }

            if ((contract_status->status == PD_CONTRACT_NEGOTIATION_SUCCESSFUL) ||
                    (contract_status->status == PD_CONTRACT_CAP_MISMATCH_DETECTED))
            {

#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
                app_contract_handler (port);
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */

#if ((DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP))
                /*
                 * Contract established.  Enable VDM task manager for Alt. Mode support.
                 * This function will have no effect if the Alt. Modes are already running.
                 */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_UFP) ||
                        (app_status[port].vdm_prcs_failed == false)
                   )
                {
                    enable_vdm_task_mngr(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) || (UFP_ALT_MODE_SUPP) */
            }

#if (CCG_BB_ENABLE != 0)
            if (
                    (contract_status->status != PD_CONTRACT_NEGOTIATION_SUCCESSFUL) &&
                    (dpm_get_info(port)->cur_port_role == PRT_ROLE_SINK) &&
                    (gl_dpm_port_type[port] == PRT_TYPE_UFP)
               )
            {
                bb_enable(port, BB_CAUSE_PWR_FAILURE);
            }
#endif /* (CCG_BB_ENABLE != 0) */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
            if(true == gl_power_throttle_cmd_pending[port])
            {
                 gl_power_throttle_renegotiation_complete[port] = true;
            }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
            
            i2cBuf[2] = 0x00;
            i2cBuf[2] |= (g_Struct_Ptr->RequestPacketConfig.gDUTSpecRev << 1);//[b2:1]
            i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
            i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
            i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);

            g_PdssGPIOIntrHandler(INTR_SET);
            PD_BC_i2cBufHandler(INTR_PD);

            /** DUT fall back flag will be TRUE only after every Ps_Rdy and will be false each time when we send API.*/
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = true;
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = false;
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = false;
            /***/
            if(g_Struct_Ptr->gLogData.isOCPOccurred)
            {
                schedule_task(10, GRL_INIT_OCP_ALERT);
            }
            
            if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
                g_SrcCapsDecode(&g_Struct_Ptr->gPrev_SRCCaps);
#if ONLY_PD_SNK_FUNC_EN            
	    /**Pranay,15Jul'21,By default Initiating SOP1 DiscID immediately after Every PDC unless configured otherwise
            * also if cable data is ready no need to explicitly initiate SOP1 again and again (ex:in case of APDO request PDC success will be for every 8sec so no need to init SOP1 for every PDC)
            */           
            if( /*g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady == false  &&*/
                    g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC)
            {
                schedule_task(1000, GRL_APP_SOP1_TIMER);
            }
#endif
            break;

        case APP_EVT_VBUS_OCP_FAULT:
#if VBUS_OCP_ENABLE

    #if RIDGE_SLAVE_ENABLE
            /* Update the OCP status. */
            ridge_slave_update_ocp_status(port, true);
    #endif /* RIDGE_SLAVE_ENABLE */

            app_handle_fault(port, FAULT_TYPE_VBUS_OCP);
#endif /* VBUS_OCP_ENABLE */
            break;

        case APP_EVT_VBUS_SCP_FAULT:
#if VBUS_SCP_ENABLE
            app_handle_fault(port, FAULT_TYPE_VBUS_SCP);
#endif /* VBUS_SCP_ENABLE */
            break;

        case APP_EVT_VBUS_RCP_FAULT:
#if VBUS_RCP_ENABLE
            app_handle_fault(port, FAULT_TYPE_VBUS_RCP);
#endif /* VBUS_RCP_ENABLE */
            break;

        case APP_EVT_VBUS_OVP_FAULT:
#if VBUS_OVP_ENABLE
            app_handle_fault(port, FAULT_TYPE_VBUS_OVP);
#endif /* VBUS_OVP_ENABLE */
            break;

        case APP_EVT_VBUS_UVP_FAULT:
#if VBUS_UVP_ENABLE
            app_handle_fault(port, FAULT_TYPE_VBUS_UVP);
#endif /* VBUS_UVP_ENABLE */
            break;

        case APP_EVT_VCONN_OCP_FAULT:
#if VCONN_OCP_ENABLE
            /* Store the VConn fault status. */
            app_status[port].fault_status |= APP_PORT_VCONN_FAULT_ACTIVE;

#if (DFP_ALT_MODE_SUPP)
            /* Exit any active alternate modes. */
            if (gl_dpm_port_type[port] == PRT_TYPE_DFP)
            {
                vdm_task_mngr_deinit(port);
            }
#endif /* (DFP_ALT_MODE_SUPP) */
#endif /* VCONN_OCP_ENABLE */
            break;

#if CCG_PD_REV3_ENABLE
        case APP_EVT_HANDLE_EXTENDED_MSG:
#if (CCG_HPI_PD_ENABLE)
            /* Handle the extended message locally if forwarding to EC is not enabled. */
            if (hpi_is_extd_msg_ec_ctrl_enabled (port) == false)
#endif
            {
                if(!(app_extd_msg_handler(port, (pd_packet_extd_t *)dat)))
                {
                    skip_soln_cb  = false;
                }
                else
                {
                    skip_soln_cb  = true;
                }
            }
            break;

        case APP_EVT_ALERT_RECEIVED:
            /* Respond to ALERT message only if there is the number of object is one. */
            if (((pd_packet_t*)dat)->len == 1)
            {
                alert_ado = ((pd_packet_t*)dat)->dat[0];
                if(alert_ado.ado_alert.bat_status_change == false)
                {
                    dpm_pd_command(port, DPM_CMD_GET_STATUS, NULL, NULL);
                }
                else
                {
                    uint8_t i = alert_ado.ado_alert.fixed_bats |
                        (alert_ado.ado_alert.hot_swap_bats << 4);
                    dpm_pd_cmd_buf_t cmd;

                    /* Identify the first battery for which the change is intended. */
                    get_bat_status[port] = 0;
                    while ((i != 0) && ((i & 0x01) == 0))
                    {
                        get_bat_status[port]++;
                        i >>= 1;
                    }

                    cmd.cmd_sop = SOP;
                    cmd.extd_hdr.val = 0x1;
                    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
                    cmd.extd_type = EXTD_MSG_GET_BAT_STATUS;
                    cmd.dat_ptr = (uint8_t*)&get_bat_status[port];
                    dpm_pd_command(port, DPM_CMD_SEND_EXTENDED, &cmd, NULL);
                }
            }
            break;
#endif /* CCG_PD_REV3_ENABLE */

#if REGULATOR_REQUIRE_STABLE_ON_TIME
    case APP_EVT_TYPEC_ATTACH_WAIT:
        REGULATOR_ENABLE();
        break;
    case APP_EVT_TYPEC_ATTACH_WAIT_TO_UNATTACHED:
        REGULATOR_DISABLE();
        
        PD_BC_i2cBufHandler(INTR_BUF_CLR);
        i2cBuf[2] = 0x00;
        i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
        i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
        i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);
        PD_BC_i2cBufHandler(INTR_DETACH);
        g_PdssGPIOIntrHandler(INTR_SET);
        
        break;
#endif /* REGULATOR_REQUIRE_STABLE_ON_TIME */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    case APP_EVT_CC_OVP:
    case APP_EVT_SBU_OVP:
        {
            /* Make sure SOURCE/SINK FETs and VConn supply are turned OFF. */
            vconn_disable(port, dpm_stat->rev_pol);
            if ((dpm_stat->attach) && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
            {
                /* Remove the Rp termination and notify the HAL that OVP is pending. */
                pd_typec_dis_rp(port, dpm_stat->polarity);
                pd_hal_set_cc_ovp_pending(port);

                psrc_disable(port, src_disable_cbk);
            }
            else
            {
#if CCG_HW_DRP_TOGGLE_ENABLE
                /* Abort auto toggle if enabled. */
                pd_hal_abort_auto_toggle(port);
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */
#if (!(CCG_SOURCE_ONLY))
                psnk_disable(port, 0);
#endif /* (!(CCG_SOURCE_ONLY)) */
            }

#if VBUS_OVP_ENABLE
            /* No need to take new action as long as previous fault handling is still pending. */
            if ((app_status[port].fault_status & (APP_PORT_SINK_FAULT_ACTIVE | APP_PORT_DISABLE_IN_PROGRESS)) == 0)
            {
                app_handle_fault(port, FAULT_TYPE_CC_OVP);
            }
            else
#endif /* VBUS_OVP_ENABLE */
            {
                skip_soln_cb = true;
            }
        }
        break;
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
        
        case APP_EVT_PKT_RCVD:

            if(g_Struct_Ptr->gLogData.gCustomConfig == GET_BATTERY_SOC_TEMP_DETAILS)//If initiated as part of polling data
            {
                pd_pkt_p = (pd_packet_extd_t *)dat;
                
                if(pd_pkt_p->hdr.hdr.extd == 0x01)//If extended message means Battery capabilities message
                {
                    if(pd_pkt_p->msg == EXTD_MSG_BAT_CAP)//Response for Get_Battery_capabilities
                    {
                        //VID
                        gBattStatBuf[4] = (pd_pkt_p->dat[0].val & 0x00FF);
                        gBattStatBuf[5] = (pd_pkt_p->dat[0].val & 0xFF00) >> 8;
                        //PID
                        gBattStatBuf[6] = (pd_pkt_p->dat[0].val & 0xFF0000) >> 16;
                        gBattStatBuf[7] = (pd_pkt_p->dat[0].val & 0xFF000000) >> 24;
                        //Battery Design Capacity
                        gBattStatBuf[8] = (pd_pkt_p->dat[1].val & 0x000000FF);
                        gBattStatBuf[9] = (pd_pkt_p->dat[1].val & 0x0000FF00) >> 8;
                        //Battery Last Full Charge Capacity
                        gBattStatBuf[10] = (pd_pkt_p->dat[1].val & 0x00FF0000) >> 16;
                        gBattStatBuf[11] = (pd_pkt_p->dat[1].val & 0xFF000000) >> 24;
                        //Battery Type
                        gBattStatBuf[12] = (pd_pkt_p->dat[2].val & 0x000000FF);
                        
                        schedule_task(4, GRL_INIT_GET_BATT_STATUS);//Pranay,06Jun'22
                        //g_Struct_Ptr->gLogData.gCustomConfig = NA//Pranay,06Jun'22
                    }
                    else if(pd_pkt_p->msg == EXTD_MSG_STATUS)
                    {
                        //Temperature status 2 bits
                        gBattStatBuf[2] |= ( (pd_pkt_p->dat[1].val & 0x06) << 1);//[b3:2]
                        //Internal Temperature - 1Byte
                        gBattStatBuf[3] =  (pd_pkt_p->dat[0].val & 0xFF);//Internal Temperature
                        
                        schedule_task(6, GRL_INIT_GET_BATT_CAPS);
                    }
                }
                else
                {
                    if(pd_pkt_p->msg == DATA_MSG_BAT_STATUS)//Response for Get_Battery_Status
                    {
                        //Byte[1:0] -- SoC
                        //Byte[2]b[1:0] -- Battery charging status
                        //       b[3:2] -- Temperature status
                        // Byte3 -- Internal Temperature
                        //SoC-2Bytes
                        gBattStatBuf[0] = (pd_pkt_p->dat[0].val & 0x00FF0000) >> 16;
                        gBattStatBuf[1] = (pd_pkt_p->dat[0].val & 0xFF000000) >> 24;
                        //Battery charging status-2Bits
                        gBattStatBuf[2] =  (pd_pkt_p->dat[0].val & 0x0C00) >> 10;//b[1:0] 
                        g_Struct_Ptr->gLogData.gCustomConfig = NA;//Pranay,06Jun'22
                        //schedule_task(2, GRL_INIT_GET_STATUS);//Pranay,06Jun'22
                    }
                }
            }
            if(g_Struct_Ptr->gLogData.isGetBattStatusInited)//For Runtime APIs handling
            {
                pd_pkt_p = (pd_packet_extd_t *)dat;
                if(pd_pkt_p->msg == EXTD_MSG_BAT_CAP)
                {
                    g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[0] = pd_pkt_p->dat[0].val;
                    g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[1] = pd_pkt_p->dat[0].val >> 8;
                    g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[2] = pd_pkt_p->dat[0].val >> 16;
                    g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[3] = pd_pkt_p->dat[0].val >> 24;
                }
                
                g_Struct_Ptr->gLogData.isGetBattStatusInited = false;
            }
            if(g_Struct_Ptr->gLogData.isGetStatusInited)//For Runtime APIs handling
            {
                pd_pkt_p = (pd_packet_extd_t *)dat;
                if(pd_pkt_p->msg == EXTD_MSG_STATUS)
                {
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[0] = pd_pkt_p->dat[0].val;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[1] = pd_pkt_p->dat[0].val >> 8;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[2] = pd_pkt_p->dat[0].val >> 16;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[3] = pd_pkt_p->dat[0].val >> 24;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[4] = pd_pkt_p->dat[1].val;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[5] = pd_pkt_p->dat[2].val >> 8;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[6] = pd_pkt_p->dat[3].val >> 16;
                    g_Struct_Ptr->gCustomConfig.GetStatusBuf[7] = pd_pkt_p->dat[4].val >> 24;
                   
                }
                g_Struct_Ptr->gLogData.isGetStatusInited = false;
            }
#if ONLY_PD_SNK_FUNC_EN
            if(g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit == true)
            {
                g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs = (((pd_packet_t*)dat)->len);
                
                for(uint8_t i = 0; i < g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs; i++)
                {
                    g_Struct_Ptr->gPDSSConfigCtrl.gVdmSVID[i] = (((pd_packet_t*)dat)->dat[i].val);
                }
                g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit = false;
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_IGNORE;
            }
#endif /*ONLY_PD_SNK_FUNC_EN*/

            /**If APDO request is sent and SRC Rejected APDO reject, DOnt again re-send APDO, rather send the last Successfull requested PDO */
            if( (((pd_packet_t*)dat)->msg == CTRL_MSG_REJECT) && (g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag) && 
            (dpm_stat -> snk_sel_pdo.src_gen.supply_type == PDO_AUGMENTED))
            { 
                g_Struct_Ptr->RequestPacketConfig.gPDO_Index = dpm_stat->snk_rdo.rdo_gen.obj_pos;
                g_Struct_Ptr->RequestPacketConfig.gPDO_Type = dpm_stat -> snk_sel_pdo.src_gen.supply_type;
                g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = (dpm_stat->contract.max_volt)/20;
                g_Struct_Ptr->RequestPacketConfig.gOperating_I = (dpm_stat -> contract.cur_pwr)/50;
                
                g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = true;
                g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
                
            }
            break;
#if 0
        /* Default handlers are sufficient for these cases. */
        case APP_EVT_UNEXPECTED_VOLTAGE_ON_VBUS:
        case APP_EVT_RP_CHANGE:
        
        case APP_EVT_VCONN_SWAP_COMPLETE:
        case APP_EVT_SENDER_RESPONSE_TIMEOUT:
        case APP_EVT_SOFT_RESET_SENT:
        case APP_EVT_CBL_RESET_SENT:
#endif

        case APP_EVT_PR_SWAP_COMPLETE:
#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
#if PR_SWAP_ENABLE
            app_status[port].pr_swap_pending = false;
#endif /* PR_SWAP_ENABLE */
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */
            break;
        default:
            /* Nothing to do. */
            break;
    }

#if BATTERY_CHARGING_ENABLE
    if(g_Struct_Ptr->gPDSSConfigCtrl.isBC12_Ctrl_Enabled)   
        bc_pd_event_handler(port,evt);
#endif /* BATTERY_CHARGING_ENABLE */

#if POWER_BANK
    pb_event_handler (port, evt);
#endif /* POWER_BANK */

    if (!skip_soln_cb)
    {
        /* Send notifications to the solution */
        sln_pd_event_handler(port, evt, dat);
    }
    
    g_PdssGPIOIntrHandler(INTR_CLR);
}

void g_Init_BattCapsMsg()
{            
    dpm_pd_cmd_buf_t cmd;
    cmd.no_of_cmd_do = 0x01;//Pranay,16Aug'22,DataObjects are supposed to be 1
    cmd.cmd_sop = SOP;
    cmd.extd_hdr.val = 0x8001;//Pranay,16Aug'22,Making chunked field as 1 (as per capture with c2)
    cmd.extd_hdr.extd.chunked = 0x01;
    cmd.extd_hdr.extd.data_size = 0x01;
    //Pranay,16Aug'22,As per testing analysis with c2
    cmd.extd_hdr.extd.chunk_no = 0x00;
    cmd.extd_hdr.extd.request = 0x00;
    
    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
    cmd.extd_type = EXTD_MSG_GET_BAT_CAP;
    get_bat_status[0] = 0x00000000;//Pranay,16Aug'22,As per testing analysis with c2
    
    cmd.dat_ptr = (uint8_t*)&get_bat_status[0];
    
    dpm_pd_command(0, DPM_CMD_SEND_EXTENDED, &cmd, NULL);
}

void g_Init_AlertDataMsg()
{
    static dpm_pd_cmd_buf_t Alert_cmd_Buf ;
    Alert_cmd_Buf.cmd_sop = SOP;
    Alert_cmd_Buf.timeout = 120;
    Alert_cmd_Buf.no_of_cmd_do = 1;
    
//    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR;
//    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.cmd = VDM_CMD_DSC_IDENTITY;
//    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.vdm_ver = 0x02;
//    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.vdm_type = VDM_TYPE_UNSTRUCTURED;
    
    Alert_cmd_Buf.cmd_do[0].ado_alert.ocp = 0x01;
    Alert_cmd_Buf.cmd_do[0].ado_alert.hot_swap_bats = 0x00;
    Alert_cmd_Buf.cmd_do[0].ado_alert.fixed_bats = 0x00;
    Alert_cmd_Buf.cmd_do[0].val = 0x04000000;
    //vdm_cmd_buf.cmd_do[0].val = 0x04B40002;/** 04B4:VID, VDM_GET_DEVICE_VERSION code word = 0x02*/
    
    dpm_pd_command(G_PORT0, DPM_CMD_SEND_ALERT,&Alert_cmd_Buf, NULL);
    
}

void g_PdssGPIOIntrHandler(uint8_t aVar)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(aVar)
    {
        case INTR_SET:
        gpio_set_value (GPIO_PORT_1_PIN_2,0);//GPIO LOW
        break;
        case INTR_CLR:
        gpio_set_value (GPIO_PORT_1_PIN_2,1);//GPIO High
        break;
    }
}
void PD_BC_i2cBufHandler(uint8_t Var)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(Var)
        {
        case INTR_BUF_CLR:
            for(uint8_t i = 0; i < 12; i++)
            {
                i2cBuf[i] = 0x00;
            }
            //mem_set(i2cBuf,0x00,12);
        break;
        case INTR_PD://PD
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = false;
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x05;
        break;
        case INTR_BC12://BC
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x06;
        break;
        case INTR_CAPMISMATCH://CapMismatch
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x07; 
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = false;
        break;
        case INTR_ATTACHSTATE:
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x08; 
            break;
        case INTR_REQ_STATE:
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x09; 
            break;
        case INTR_DETACH:
            i2cBuf[0] = 0xFB;
            i2cBuf[1] = 0x0A; 
            break;
    }
}
#if ONLY_PD_SNK_FUNC_EN
void gPrepareSOP1DiscID(uint8_t *aBuffer)
{   
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    static dpm_pd_cmd_buf_t vdm_cmd_buf ;
    uint8_t VdmType = (aBuffer[0] & 0x0F);/**VdmType; 0 :: Unstructured, 1 :: structured **/
    
    vdm_cmd_buf.cmd_sop = (aBuffer[0] & 0xF0) >> 4;/*SOP TYPE*/
    
    uint8_t lDataObjectsCnt = aBuffer[1];/*No of Dataobjects*/
    
    vdm_cmd_buf.no_of_cmd_do = lDataObjectsCnt;
    vdm_cmd_buf.timeout = 120;

    uint8_t lBufIndex = 2;
    uint32_t vdoVal;
 
    switch(VdmType)
    {
        case VDM_TYPE_UNSTRUCTURED :
        
        break;
    
        case VDM_TYPE_STRUCTURED:
            
            if(vdm_cmd_buf.cmd_sop != SOP)
            {
                g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader = (aBuffer[lBufIndex]);lBufIndex++;
                g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader |= ((aBuffer[lBufIndex] << 8)); lBufIndex++;
            }
            for(uint8_t index = 0; index < lDataObjectsCnt; ++index)
            {
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR;
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.cmd = VDM_CMD_DSC_IDENTITY;
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.vdm_type = VDM_TYPE_STRUCTURED;
                vdm_cmd_buf.cmd_do[index].std_vdm_hdr.obj_pos = index;
             
                vdoVal = 0;

                for(uint8_t i = 0,j = 0 ; i < 4; ++i,++lBufIndex)
                {
                    vdoVal |= (aBuffer[lBufIndex] << j); 
                    vdm_cmd_buf.cmd_do[index].val = vdoVal;
                    j += 8;
                }
            }
            if(vdm_cmd_buf.cmd_sop != SOP)
            {
                g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_CABLE_TESTER;
                
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_ACK;
                
                pd_phy_load_msg(0, vdm_cmd_buf.cmd_sop ,0,vdm_cmd_buf.no_of_cmd_do, g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader , false, (uint32_t *)vdm_cmd_buf.cmd_do);
                CyDelayUs(1000);
                pd_phy_send_msg(0);
                pd_phy_refresh_roles(0);/**Should call this if not we'll not receive any SOP1 responses*/
            }
            else
            {
                g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit = true;
                dpm_pd_command(G_PORT0, DPM_CMD_SEND_VDM,&vdm_cmd_buf, NULL);
            }
        break;
    }
}

void gInitSOP1DiscID()
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    uint8_t lBuffer[] = {0x11, 0x01, 0x4F, 0x10, 0x01, 0xA0, 0x00, 0xFF};/**[SOPType|vdmtype] [Dataobjects count][3:2 Header][4:7 Data object[0]]**/
                        
    ++g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID;
    
    /**Configuring message ID based on received and prev. initiated count, Header'ss [11:9] bits indicates Message ID*/
    lBuffer[3] |= (g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID << 1);
    
    gPrepareSOP1DiscID(lBuffer);
}
bool PrepareAckPkt()
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    static pd_packet_extd_t* rx_pd_packet ;
    rx_pd_packet = pd_phy_get_rx_packet(0);
    uint32_t CmdType = rx_pd_packet->dat[0].val;
    uint8_t SopType = rx_pd_packet -> sop;
    
    if((CmdType & 0x1F) != VDM_CMD_DSC_IDENTITY)/**IF received SVID/Modes, Send NAK*/
    {
        static dpm_pd_cmd_buf_t SVID_cmd_buf ;
        
        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.vdm_type = VDM_TYPE_STRUCTURED;

        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.obj_pos = 0;
        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.st_ver = 0;
        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.svid = 0xFF00;
        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.cmd = VDM_CMD_DSC_SVIDS;
        SVID_cmd_buf.cmd_sop = SopType;
        
        SVID_cmd_buf.cmd_do[0].std_vdm_hdr.cmd_type = CMD_TYPE_RESP_NAK;
        SVID_cmd_buf.no_of_cmd_do = 1;
        
        pd_phy_load_msg(0,SopType,  100, SVID_cmd_buf.no_of_cmd_do, 0x114F , false, (uint32_t *)SVID_cmd_buf.cmd_do);
    }
    else
    {
        vdm_cmd_buf.cmd_sop  = SopType;
        vdm_cmd_buf.no_of_cmd_do = g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs;
        pd_phy_load_msg(0,SopType,  100, g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs, g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader , false, (uint32_t *)vdm_cmd_buf.cmd_do);
    }
    
    return true;
}
#endif /*ONLY_PD_SNK_FUNC_EN*/

#ifdef EVT_LOG
void gBufLog(bool isReset,uint8_t aLogVar)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    if(isReset)
    {
        memset(gEventlogBuffer,0x00,EVNT_LOG_BUF_SIZE);
       
        gEventlogBufIndex = 0x00;
        
        g_Struct_Ptr->gLogData.gFromBufIndex = 0x00;
    }
    else
    {
        /***If event logger Buffer overflowed, than reset Buf index(used for filling event log) and base buf index(used for responding to user API)*/
        if(gEventlogBufIndex == EVNT_LOG_BUF_SIZE)
        {
            gEventlogBufIndex = 0;
            g_Struct_Ptr->gLogData.gFromBufIndex = 0;
        }
        gEventlogBuffer[gEventlogBufIndex] = aLogVar;
        ++gEventlogBufIndex;
    }
}
#endif

app_resp_t* app_get_resp_buf(uint8_t port)
{
    return &app_status[port].app_resp;
}

app_status_t* app_get_status(uint8_t port)
{
    return &app_status[port];
}

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
bool app_is_port_enabled(uint8_t port)
{
    bool ret = true;

    if (port > NO_OF_TYPEC_PORTS)
        ret = false;

#if ((CCG_HPI_ENABLE) && (!CCG_LOAD_SHARING_ENABLE))
    /* Check if the port has been disabled through HPI. */
    if ((hpi_get_port_enable() & (1 << port)) == 0)
        ret = false;
#endif /* CCG_HPI_ENABLE */

    return ret;
}

/* Callback used to receive fault notification from the HAL and to pass it on to the event handler. */
static void app_cc_sbu_fault_handler(uint8_t port, bool fault_type)
{
    app_event_handler(port, (fault_type ? APP_EVT_SBU_OVP : APP_EVT_CC_OVP), 0);
}

#if CCGX_V5V_CHANGE_DETECT

static void app_cbl_dsc_timer_cb (uint8_t port, timer_id_t id);

static void app_cbl_dsc_callback (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    /* Keep repeating the DPM command until we succeed. */
    if (resp == SEQ_ABORTED)
    {
        timer_start (port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}

static void app_cbl_dsc_timer_cb (uint8_t port, timer_id_t id)
{
    if (dpm_pd_command (port, DPM_CMD_INITIATE_CBL_DISCOVERY, NULL, app_cbl_dsc_callback) != CCG_STAT_SUCCESS)
    {
        timer_start (port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
    }
}

#if DP_UFP_SUPP
static void app_ufp_5v_recov_cb(uint8_t port, timer_id_t id)
{
    (void)id;
    
    /* Create app alt mode command to initiate DP related Vconn Swap procedure */
    uint8_t dp_cmd[4]  = {0xA, 0x00, 0x01, 0xFF};
    uint8_t dp_data[4] = {0x00, 0x00, 0x00, DP_APP_VCONN_SWAP_CFG_CMD};
    
    if (eval_app_alt_mode_cmd(port, dp_cmd, dp_data) == false)
    {
        /* Init Hard reset if DP alt mode not entered */
        dpm_pd_command (port, DPM_CMD_SEND_HARD_RESET, NULL, NULL);
    }
}
#endif /* #if DP_UFP_SUPP */
#endif /* CCGX_V5V_CHANGE_DETECT */

/* Callback that will be called when there is any change to the V5V or VSYS supplies. */
void app_ccg5_supply_change_cb(uint8_t port, ccg_supply_t supply_id, bool present)
{
#if CCGX_V5V_CHANGE_DETECT
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /*
     * Currently we only handle V5V changes:
     * If V5V is removed, we exit active alternate modes if there is a cable which requires VConn.
     * If V5V is re-applied after being removed, we restart the alternate mode state machine.
     */
    if (supply_id == CCG_SUPPLY_V5V)
    {
        if (!present)
        {
            app_status[port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;

            if (vconn_is_present (port))
            {
                /* Ensure that the VConn switch is turned off. */
                vconn_disable (port, dpm_stat->rev_pol);

#if (DFP_ALT_MODE_SUPP)
                /* If we are the DFP, the cable requires VConn and Alt. Modes are Active, exit alt. modes. */
                if (
                        (gl_dpm_port_type[port] == PRT_TYPE_DFP) &&
                        (dpm_stat->cbl_vdo.std_cbl_vdo.cbl_term != CBL_TERM_BOTH_PAS_VCONN_NOT_REQ) &&
                        (app_status[port].alt_mode_entered != 0)
                   )
                {
                    vdm_task_mngr_deinit(port);
                }
#endif /* (DFP_ALT_MODE_SUPP) */
            }
        }
        else
        {
            if ((app_status[port].fault_status & APP_PORT_V5V_SUPPLY_LOST) != 0)
            {
                app_status[port].fault_status &= ~APP_PORT_V5V_SUPPLY_LOST;

#if (DFP_ALT_MODE_SUPP)
                /*
                 * Alt. Mode operation was previously suspended due to V5V not being present.
                 * We can restart the alt. mode state machine so that mode discovery and operation
                 * can take place.
                 */
                if ((dpm_stat->contract_exist) && (gl_dpm_port_type[port] == PRT_TYPE_DFP))
                {
                    /* Ask PD stack to trigger cable discovery. */
                    if (dpm_pd_command(port, DPM_CMD_INITIATE_CBL_DISCOVERY,
                                NULL, app_cbl_dsc_callback) != CCG_STAT_SUCCESS)
                    {
                        timer_start(port, APP_CBL_DISC_TRIGGER_TIMER, APP_CBL_DISC_TIMER_PERIOD, app_cbl_dsc_timer_cb);
                    }

                    /*
                     * Reset the alternate mode state machine. The cable discovery complete flag is also cleared so
                     * that alternate mode state machine can be started at the end of cable discovery.
                     */
                    alt_mode_layer_reset(port);
                    app_status[port].cbl_disc_id_finished = false;
                }
#endif /* (DFP_ALT_MODE_SUPP) */
#if DP_UFP_SUPP
                if ((dpm_stat->contract_exist) && (gl_dpm_port_type[port] == PRT_TYPE_UFP))
                {
                    timer_start(port, APP_UFP_RECOV_VCONN_SWAP_TIMER, APP_UFP_RECOV_VCONN_SWAP_TIMER_PERIOD, app_ufp_5v_recov_cb);
                }
#endif /* DP_UFP_SUPP */  
            }
        }
    }
#endif /* CCGX_V5V_CHANGE_DETECT */
}

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

void app_init(void)
{
    uint8_t port;

    /* For now, only the VDM handlers require an init call. */
    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        vdm_data_init(port);

#if (CCG_BB_ENABLE != 0)
        /*
         * Initialize the billboard interface. The billboard
         * interface shall not get initialized if it is not
         * enabled in configuration table.
         */
        bb_init(port);
#endif /* (CCG_BB_ENABLE != 0) */

#if BATTERY_CHARGING_ENABLE
        bc_init(port);
#endif /* BATTERY_CHARGING_ENABLE */

#if ((TBT_DFP_SUPP) | (TBT_UFP_SUPP))
        if (get_pd_port_config(port)->tbthost_cfg_tbl_offset != 0)
        {
#if PR_SWAP_ENABLE
            app_pref_power_role[port] = (pd_get_ptr_tbthost_cfg_tbl(port)->pref_pwr_role);
#endif /* PR_SWAP_ENABLE */
            app_pref_data_role[port] = (pd_get_ptr_tbthost_cfg_tbl(port)->pref_data_role);
        }
        else
        {
            /* Initialize with no preference by default. */
#if PR_SWAP_ENABLE
            app_pref_power_role[port] = PRT_DUAL;
#endif /* PR_SWAP_ENABLE */
            app_pref_data_role[port] = PRT_TYPE_DRP;
        }
#endif /* ((TBT_DFP_SUPP) | (TBT_UFP_SUPP)) */
    }

#if (CCG_TYPE_A_PORT_ENABLE == 1)
    /* For systems with TYPEA port. */
    if (get_pd_port_config(0)->type_a_enable)
    {
        type_a_port_enable();
    }
    else
    {
        /* Ensure that the state machine variables are initialized correctly. */
        type_a_port_disable();
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE == 1) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Register a callback for notification of CC/SBU faults. */
    ccg_set_fault_cb(app_cc_sbu_fault_handler);

    /* Register a handler that will be notified when there is any change in V5V or VSYS state. */
    pd_hal_set_supply_change_evt_cb(app_ccg5_supply_change_cb);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if APP_PPS_SINK_SUPPORT
    hpi_set_userdef_write_handler (hpi_user_reg_handler);
#endif /* APP_PPS_SINK_SUPPORT */
}

/* Implements CCG deep sleep functionality for power saving. */
bool system_sleep(void)
{
    uint8_t intr_state;
    bool dpm_slept = false;
    bool app_slept = false;
    bool app_type_c_slept = false;
    bool retval = false;
#if BATTERY_CHARGING_ENABLE
    bool bc_slept = false;
#endif /* BATTERY_CHARGING_ENABLE */

    intr_state = CyEnterCriticalSection();

    /*
     * We have to check the application layer, HPI and the Device Policy
     * Manager (DPM) to see if all of these modules are ready for sleep.
     * CCG can only enter deep sleep if all of these blocks are in an idle
     * state.
     *
     * Note: The respective sleep functions might be performing some
     * state updates as part of the idle check function; and therefore
     * the corresponding wakeup function needs to be called if they have
     * returned true to indicate that sleep is allowed.
     */
    if (app_sleep())
    {
        app_slept = true;

#if BATTERY_CHARGING_ENABLE
        if(bc_sleep() == true)
        {
            bc_slept = true;
#endif /* BATTERY_CHARGING_ENABLE */

            if (
#if CCG_HPI_ENABLE
                    (hpi_sleep_allowed()) &&
#endif /* CCG_HPI_ENABLE */

#if CCG_UCSI_ENABLE
                    (ucsi_sleep_allowed()) &&
#endif /* CCG_UCSI_ENABLE */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING)
                    ccg_sensor_is_idle() &&
#endif /*(CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING) */

#if CCG_LOAD_SHARING_ENABLE
                    ccg_ls_is_idle(0) &&
#endif /* CCG_LOAD_SHARING_ENABLE */
#if CCG_CABLE_COMP_ENABLE
                    ccg_cable_comp_is_idle(0) &&
#endif /* CCG_CABLE_COMP_ENABLE */
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
                    ccg_power_throttle_sleep_allowed() &&
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
                    (dpm_deepsleep())
               )
            {
                dpm_slept = true;
                /*
                 * Check connection status of TYPE-A and TYPE-C ports to determine if deepsleep
                 * entry is allowed. If not,enter sleep mode to save power.
                 */
                if (
    #if CCG_TYPE_A_PORT_ENABLE
                    (type_a_is_idle() == true) &&
    #endif /* CCG_TYPE_A_PORT_ENABLE */
                    (app_type_c_sleep_allowed() == true)
                    )
                {
                    app_type_c_slept = true;
                    timer_enter_sleep();

                    /*
                     * CDT 224642: The I2C IDLE check needs to be done as the last step
                     * before device enters into sleep. Otherwise, the device may fail
                     * to wake up when there is an address match on the I2C interface.
                     */
#if ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE))
                    if (
    #if CCG_HPI_ENABLE
                            (hpi_sleep())
    #else
                            (1)
    #endif /* CCG_HPI_ENABLE */
                            &&
    #if RIDGE_SLAVE_ENABLE
                            (ridge_slave_sleep())
    #else
                            (1)
    #endif /* RIDGE_SLAVE_ENABLE */
                       )
#endif /* ((CCG_HPI_ENABLE) || (RIDGE_SLAVE_ENABLE)) */
                    {
                        /* Device sleep entry. */
                        CySysPmDeepSleep();
                        retval = true;
                    }
                }
                else
                {
#if (defined(CCG3PA) || defined(CCG3PA2))
                    /* Enter Sleep mode to save power. */
                    CySysPmSleep();
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
                }
            }
#if BATTERY_CHARGING_ENABLE
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

    CyExitCriticalSection(intr_state);

    if (app_type_c_slept)
    {
        app_type_c_wakeup();
    }

    /* Call dpm_wakeup() if dpm_sleep() had returned true. */
    if(dpm_slept)
    {
        dpm_wakeup();
    }

    /* Call app_wakeup() if app_sleep() had returned true. */
    if(app_slept)
    {
        app_wakeup();
    }

#if BATTERY_CHARGING_ENABLE
    if(bc_slept)
    {
        bc_wakeup();
    }
#endif /* BATTERY_CHARGING_ENABLE */

    return retval;
}

#if VCONN_OCP_ENABLE
void app_vconn_ocp_cbk(uint8_t port, bool comp_out)
{
    /* Disable VConn since we hit a fault. */
    vconn_disable(port, dpm_get_info(port)->rev_pol);

    /* Notify application layer about fault. */
    app_event_handler(port, APP_EVT_VCONN_OCP_FAULT, NULL);
}
#endif /* VCONN_OCP_ENABLE */

bool vconn_enable(uint8_t port, uint8_t channel)
{
#if VCONN_OCP_ENABLE

    /* Do not attempt to enable VConn if fault was detected in present connection. */
    if ((app_status[port].fault_status & APP_PORT_VCONN_FAULT_ACTIVE) != 0)
    {
        return false;
    }

#if CCG_VCONN_MON_WITH_ADC

    /* If Vconn current is being monitored through ADC, configure the associated IOs. */
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P1,APP_VCONN_MON_AMUX_INPUT_P1);
#if CCG_PD_DUALPORT_ENABLE
    hsiom_set_config (APP_VCONN_MON_PORT_PIN_P2,APP_VCONN_MON_AMUX_INPUT_P2);
#endif /* CCG_PD_DUALPORT_ENABLE */

    /*
     * 120us delay required as a settling time after HSIOM config to get a stable
     * ADC reading.
     */
    CyDelayUs(120);

#endif /* CCG_VCONN_MON_WITH_ADC */

    system_vconn_ocp_en(port, app_vconn_ocp_cbk);
#endif /* VCONN_OCP_ENABLE */

    /* Reset RX Protocol for cable */
    dpm_prot_reset_rx(port, SOP_PRIME);
    dpm_prot_reset_rx(port, SOP_DPRIME);

    if (pd_vconn_enable(port, channel) != CCG_STAT_SUCCESS)
    {
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
        app_status[port].fault_status |= APP_PORT_V5V_SUPPLY_LOST;
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */
        return false;
    }
    return true;
}

void vconn_disable(uint8_t port, uint8_t channel)
{
    pd_vconn_disable(port, channel);

#if VCONN_OCP_ENABLE
    system_vconn_ocp_dis(port);
#endif /* VCONN_OCP_ENABLE */
}

bool vconn_is_present(uint8_t port)
{
    return pd_is_vconn_present( port, dpm_get_info(port)->rev_pol);
}

bool vbus_is_present(uint8_t port, uint16_t volt, int8 per)
{
    uint8_t level;
    uint8_t retVal;

    const dpm_status_t *dpm_stat = dpm_get_info(port);

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling.
     * To avoid false output on OVP Trip pin when VBUS is polled
     * OVP trip pin is disconnected from OVP comp output and last
     * value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4*/

    /*
     * If we are in contract as a sink with a PPS source and default Type-C detach detection settings are being
     * used, change the settings to match the actual PDO.
     */
    if (
            (dpm_stat->contract_exist) &&
            (dpm_stat->cur_port_role == PRT_ROLE_SINK) &&
            /*(dpm_stat->snk_sel_pdo.pps_src.supply_type == PDO_AUGMENTED) &&*/
            (volt == VSAFE_5V) &&
            (per == VSAFE_5V_SNK_TURN_OFF_MARGIN)
       )
    {
        volt = 3000/*dpm_stat->contract.min_volt*/;
        per = VSAFE_SNK_TURN_OFF_MARGIN;
    }

    /*
     * Re-run calibration every time to ensure that VDDD or the measurement
     * does not break.
     */
    pd_adc_calibrate (port, APP_VBUS_POLL_ADC_ID);
    level = pd_get_vbus_adc_level(port, APP_VBUS_POLL_ADC_ID, volt, per);
    retVal = pd_adc_comparator_sample (port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT, level);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

uint16_t vbus_get_value(uint8_t port)
{
    uint16_t retVal;

#ifdef CCG4
    /*
     * OVP comparator is multiplexed with VBUS polling. To avoid false output
     * on OVP Trip pin when VBUS is polled OVP trip pin is disconnected from
     * OVP comp output and last value of OVP comp output is driven via GPIO.
     */
    system_disconnect_ovp_trip(port);
#endif /* CCG4 */

    /* Measure the actual VBUS voltage. */
    retVal = pd_hal_measure_vbus(port);

#ifdef CCG4
    /* Connect OVP trip pin back to comparator output */
    system_connect_ovp_trip(port);
#endif /* CCG4*/

    return retVal;
}

#if VCONN_OCP_ENABLE
bool system_vconn_ocp_en(uint8_t port, PD_ADC_CB_T cbk)
{
    if (cbk == NULL)
    {
        return false;
    }

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    /* Request the HAL to enable OCP detection with the appropriate debounce period. */
    pd_hal_vconn_ocp_enable(port, pd_get_ptr_vconn_ocp_tbl(port)->debounce, cbk);

#else /* !defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl |= PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr_1_cfg |= CYVAL_USBPD_V5V_CFG_POS_EDG_DIS_NEG_EDG_EN << PDSS_INTR_1_CFG_V5V_CFG_POS;
    pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Configure ADC to detect VConn Over-Current condition. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, APP_VCONN_OCP_ADC_INPUT,
            APP_VCONN_TRIGGER_LEVEL, PD_ADC_INT_RISING, cbk);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
    return true;
}

void system_vconn_ocp_dis(uint8_t port)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    /* Disable VConn OCP detection in the HAL. */
    pd_hal_vconn_ocp_disable(port);

#else /* !defined (CCG5) || defined(CCG6) || defined(CCG5C) */
#if CCG4_DOCK
    PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
    {
        PDSS0
#if CCG_PD_DUALPORT_ENABLE
            ,
        PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
    };
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->pfet300_ctrl &= ~PDSS_PFET300_CTRL_EN_COMP;
    pd->intr_1_cfg &= ~PDSS_INTR_1_CFG_V5V_CFG_MASK;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#else /* !CCG4_DOCK */
    /* Disable the ADC used for VConn OCP detection. */
    pd_adc_comparator_ctrl(port, APP_VCONN_OCP_ADC_ID, 0, 0, 0, NULL);
#endif /* CCG4_DOCK */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
}

#endif /* VCONN_OCP_ENABLE */

#if VBUS_OVP_ENABLE
/* Configure Over-Voltage Protection checks based on parameters in config table. */
void app_ovp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T ovp_cb)
{
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
    uint8_t level;
    uint8_t threshold;
#else
    uint8_t debounce = 0;
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

    uint8_t intr_state;

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection();

#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        threshold = pd_get_ptr_ovp_tbl(port)->threshold;

#if ADC_FALSE_OVP_FIX
        /* Make sure threshold is set to a suitable value at low voltages to avoid false OVP trips. */
        if ((volt_mV + ((volt_mV / 100) * threshold)) < ADC_VBUS_MIN_OVP_LEVEL)
        {
            volt_mV   = ADC_VBUS_MIN_OVP_LEVEL;
            threshold = 0;
        }
#endif /* ADC_FALSE_OVP_FIX */

        /* Set OVP threshold. */
        level = pd_get_vbus_adc_level(port, APP_OVP_ADC_ID, volt_mV, threshold);
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, APP_OVP_ADC_INPUT, level, PD_ADC_INT_FALLING, ovp_cb);

#else /* (VBUS_OVP_MODE != VBUS_OVP_MODE_ADC) */

        /* Convert debounce period from us to clock cycles assuming a 500 KHz filter clock. */
        debounce = pd_get_ptr_ovp_tbl(port)->debounce;
        debounce = (debounce > 0x40) ? 0x20 : ((debounce + 1) >> 1);

#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) || defined(CCG6) || defined(CCG5C))
        pd_internal_vbus_ovp_en(port, volt_mV, pd_get_ptr_ovp_tbl(port)->threshold, ovp_cb,
                pfet, VBUS_OVP_MODE, debounce);
#endif /* defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) || defined(CCG6) || defined(CCG5C) */

#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */

        CyExitCriticalSection(intr_state);
    }
}

void app_ovp_disable(uint8_t port, bool pfet)
{
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OVP_EN_MASK)
    {
        /* Disable OVP. */
#if (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC)
        pd_adc_comparator_ctrl(port, APP_OVP_ADC_ID, 0, 0, 0, NULL);
#else
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) || defined(CCG6) || defined(CCG5C))
        pd_internal_vbus_ovp_dis(port, pfet);
#endif /* defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) || defined(CCG6) || defined(CCG5C) */
#endif /* (VBUS_OVP_MODE == VBUS_OVP_MODE_ADC) */
    }
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE

#if CCG4_DOCK
extern void dock_uvp_check (uint8_t port, timer_id_t id);
#endif /* CCG4_DOCK */

/* Configure Under-Voltage Protection checks based on parameters in config table. */
void app_uvp_enable(uint8_t port, uint16_t volt_mV, bool pfet, PD_ADC_CB_T uvp_cb)
{
    uint8_t intr_state;
    uint8_t debounce;

    /* Assuming 500 KHz filter clock for debounce. */
    debounce = pd_get_ptr_uvp_tbl(port)->debounce;
    debounce = (debounce > 0x40) ? 0x20 : ((debounce+1) >> 1);

    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
        intr_state = CyEnterCriticalSection ();

#if CCG_PPS_SRC_ENABLE
        if (dpm_get_info(port)->src_sel_pdo.fixed_src.supply_type == PDO_AUGMENTED)
        {
            /*
             * In PPS mode operation, UVP is not an unrecoverable event. It
             * needs to be dealt with a simple hardreset. Configure for non-
             * hardware cutoff operation. NOTE: The threshold for operation
             * can be overridden to set the cut-off based on system requirement.
             * Currently using the lowest UVP point for this.
             */
            volt_mV = dpm_get_info(port)->src_sel_pdo.pps_src.min_volt * 100;
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE_INT_COMP, debounce);
        }
        else
#endif /* CCG_PPS_SRC_ENABLE */
        {
#if CCG4_DOCK
            timer_start(port, VBUS_UVP_TIMER_ID, VBUS_UVP_TIMER_PERIOD, dock_uvp_check);
#else /* CCG4_DOCK */
            pd_internal_vbus_uvp_en (port, volt_mV, pd_get_ptr_uvp_tbl(port)->threshold,
                    uvp_cb, pfet, VBUS_UVP_MODE, debounce);
#endif /* CCG4_DOCK */
        }

        CyExitCriticalSection (intr_state);
    }
}

void app_uvp_disable(uint8_t port, bool pfet)
{
    /* Disable UVP. */
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
    {
#if CCG4_DOCK
        timer_stop(port, VBUS_UVP_TIMER_ID);
#else /* CCG4_DOCK */
        pd_internal_vbus_uvp_dis (port, pfet);
#endif /* CCG4_DOCK */
    }
}

#endif /* VBUS_UVP_ENABLE */

/* End of File */
