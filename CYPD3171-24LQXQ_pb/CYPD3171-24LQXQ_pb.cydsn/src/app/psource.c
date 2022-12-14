/**
 * @file psource.c
 *
 * @brief @{Power source (Provider) manager source file.@}
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

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <app.h>
#include <timer.h>
#include <hal_ccgx.h>
#include <pdss_hal.h>
#include <gpio.h>
#include <battery_charging.h>
#include <system.h>
#include <grlapp.h>
#if CCG_CABLE_COMP_ENABLE
#include <cable_comp.h>
#endif /* CCG_CABLE_COMP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))
#include <vbus_ctrl.h>
#endif /* #if (defined(CCG3PA) || defined(CCG3PA2)) */

/* Type-C current levels in 10mA units. */
#define CUR_LEVEL_3A    300
#define CUR_LEVEL_1_5A  150
#define CUR_LEVEL_DEF   90

/* VBUS absolute maximum voltage in mV units */
#define VBUS_MAX_VOLTAGE  (21500u)

static void psrc_dis_ovp(uint8_t port);
void psrc_en_ovp(uint8_t port);
void psrc_en_uvp(uint8_t port);
static void psrc_dis_uvp(uint8_t port);
void psrc_en_rcp(uint8_t port);
static void psrc_dis_ocp(uint8_t port);
static void psrc_shutdown(uint8_t port, bool discharge_dis);

void app_psrc_tmr_cbk(uint8_t port, timer_id_t id);
void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out);
void app_psrc_vbus_ocp_cbk(uint8_t port);
void app_psrc_vbus_scp_cbk(uint8_t port);
void app_psrc_vbus_rcp_cbk(uint8_t port);

void psrc_select_voltage(uint8_t port);

#if NCP_MANAGEMENT
extern void ncp_manage_cbk(uint8_t port, timer_id_t id);
#endif /* NCP_MANAGEMENT */

#if CCG_CABLE_COMP_ENABLE
extern volatile uint8_t gl_cable_comp_start[NO_OF_TYPEC_PORTS];
#endif /* CCG_CABLE_COMP_ENABLE */

static void vbus_fet_on(uint8_t port)
{
    if(app_get_status(port)->is_vbus_on == false)
    {
        /* If fet is already On then no need to enable it again */
        app_get_status(port)->is_vbus_on = true;
#if NCP_POWER_SAVE
#if NCP_MANAGEMENT
        timer_start(port,NCP_ENABLE_DELAY_ID,NCP_ENABLE_DELAY_PERIOD,ncp_manage_cbk);
#else
        PD_CTRL_EN(port);
#endif /* NCP_MANAGEMENT */
#endif /* NCP_POWER_SAVE */
        if(port == TYPEC_PORT_0_IDX)
        {
            /*
             * In case of REGULATOR_REQUIRE_STABLE_ON_TIME, the regulator is
             * already turned on. POWER_BANK implementation, uses a single
             * regulator and FET control for both source and sink operation.
             * Turning OFF sink FET here will cause the regulator to get wrongly
             * shut down. We should not disable sink here in this case.
             */
#if (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK)))
            APP_VBUS_SNK_FET_OFF_P1();
            CyDelayUs(10);
#endif /* (!((REGULATOR_REQUIRE_STABLE_ON_TIME) && (POWER_BANK))) */
            APP_VBUS_SRC_FET_ON_P1();
        }
#if CCG_PD_DUALPORT_ENABLE
        if(port == TYPEC_PORT_1_IDX)
        {
            APP_VBUS_SNK_FET_OFF_P2();
            CyDelayUs(10);
            APP_VBUS_SRC_FET_ON_P2();
        }
#endif /* CCG_PD_DUALPORT_ENABLE */
    }
}

static void vbus_fet_off(uint8_t port)
{
    app_get_status(port)->is_vbus_on = false;
#if NCP_POWER_SAVE
    PD_CTRL_DIS(port);
#endif /* NCP_POWER_SAVE */
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_VBUS_SRC_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
}

void vbus_discharge_on(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_ON_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_ON_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

void vbus_discharge_off(uint8_t port)
{
    if(port == TYPEC_PORT_0_IDX)
    {
        APP_DISCHARGE_FET_OFF_P1();
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        APP_DISCHARGE_FET_OFF_P2();
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

}

#if CCG_PPS_SRC_ENABLE

static void psrc_en_cf(uint8_t port)
{
#if VBUS_CF_EN
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    if (dpm_stat->src_last_rdo.val != dpm_stat->src_cur_rdo.val)
    {
        uint32_t op_cur = dpm_stat->src_cur_rdo.rdo_pps.op_cur * 5; /* In 10mA units */

        /* Minimum supported limit is 1A. */
        if(op_cur < I_1A)
        {
            op_cur = I_1A;
        }

        /*
         * The PDO is power limited, then we should limit the current to the
         * maximum allowed by PDP limit. The PDP information is retrieved from
         * the extended source cap information.
         */
        if (dpm_stat->src_sel_pdo.pps_src.pps_pwr_limited)
        {
            uint32_t limit = (dpm_stat->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX] * 1000);

            /*
             * To improve arithematic accuracy, the limit is calculated by
             * (mW * 100) / (mV) to get current in 10mA units.
             */
            limit = ((limit * 100) / app_get_status(port)->psrc_volt);
            op_cur = GET_MIN(op_cur, limit);
        }

        pd_cf_enable(port, op_cur, NULL);
        app_get_status(port)->cur_fb_enabled = true;
    }
#endif /* VBUS_CF_EN */
}

static void psrc_dis_cf(uint8_t port)
{
#if VBUS_CF_EN
    app_status_t *app_stat = app_get_status(port);

    if (app_stat->cur_fb_enabled)
    {
        pd_cf_disable(port);
        dpm_set_cf(port, false);
        app_stat->cur_fb_enabled = false;
        timer_stop(port, APP_PSOURCE_CF_TIMER);
    }
#endif /* VBUS_CF_EN */
}
#endif /* CCG_PPS_SRC_ENABLE */

static void call_psrc_ready_cbk(uint8_t port)
{
    app_status_t* app_stat = app_get_status(port);

    if (app_stat->pwr_ready_cbk != NULL)
    {
        app_stat->pwr_ready_cbk (port);
        app_stat->pwr_ready_cbk = NULL;
    }
#if CCG_CABLE_COMP_ENABLE
    /* 
     * Enable cable compensation comparators for the first time when 
     * PS_RDY is sent.
     * Cable compensation task will take care of re-adjusting the comparators 
     * as and when comparator interrupts are received.
     */
    gl_cable_comp_start[port] = true;
#endif /* CCG_CABLE_COMP_ENABLE */
}

/*Timer Callback*/
void app_psrc_tmr_cbk(uint8_t port, timer_id_t id)
{
    app_status_t* app_stat = app_get_status(port);

    switch(id)
    {
        case APP_PSOURCE_EN_TIMER:
            /* Supply did not reach expected level. Turn off power and do error recovery. */
            timer_stop_range(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_HYS_TIMER);
            app_stat->psrc_volt_old = VSAFE_0V;
            psrc_shutdown(port, true);

#if (VBUS_UVP_ENABLE)
            /*
             *If the VBUS does not reach VSAFE5V, then we need to treat this as an
             * under voltage condition. Since the UVP hardware detection cannot be
             * enabled when turning on the VBUS, this has to be manually triggered
             * from here by invoking the callback directly. Do this only if UVP is
             * enabled from the configuration table.
             */
            if (get_pd_port_config(port)->protection_enable & CFG_TABLE_UVP_EN_MASK)
            {
                app_psrc_vbus_ovp_cbk(port, false);
            }
#endif /* (VBUS_UVP_ENABLE) */
            break;

#if ((!defined(CCG3PA)) && (!defined(CCG3PA2)))
        case APP_PSOURCE_EN_MONITOR_TIMER:
            if (((app_stat->psrc_rising == true) &&
                        (vbus_is_present(port, app_stat->psrc_volt, VBUS_TURN_ON_MARGIN) == true)) ||
                    ((app_stat->psrc_rising == false) &&
                     (vbus_is_present(port, app_stat->psrc_volt, VBUS_DISCHARGE_MARGIN) == false)))
            {
                /* Start Source enable hysteresis Timer */
                timer_start(port, APP_PSOURCE_EN_HYS_TIMER,
                        APP_PSOURCE_EN_HYS_TIMER_PERIOD, app_psrc_tmr_cbk);
                break;
            }

            /* Start Monitor Timer again */
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
            break;
#endif /* ((!defined(CCG3PA)) && (!defined(CCG3PA2))) */

        case APP_PSOURCE_EN_HYS_TIMER:
            timer_stop(port, APP_PSOURCE_EN_TIMER);
            app_stat->psrc_volt_old = app_stat->psrc_volt;
            vbus_discharge_off(port);

            if(app_stat->psrc_rising == false)
            {
                /* VBus voltage has stabilized at the new lower level. Update the OVP and RCP limits. */
                psrc_en_ovp (port);
                psrc_en_rcp (port);
            }
            else
            {
                psrc_en_uvp (port);
            }

#if CCG_PPS_SRC_ENABLE
            if (app_stat->cur_fb_enabled)
            {
                timer_start(port, APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD, app_psrc_tmr_cbk);
            }
#endif /* CCG_PPS_SRC_ENABLE */
            call_psrc_ready_cbk(port);
            break;

        case APP_PSOURCE_DIS_TIMER:
            /* Discharge operation timed out. */
            timer_stop(port, APP_PSOURCE_DIS_MONITOR_TIMER);
            psrc_shutdown(port, true);
            break;

        case APP_PSOURCE_DIS_MONITOR_TIMER:
            if (vbus_is_present(port, VSAFE_5V, VBUS_DISCHARGE_TO_5V_MARGIN) == false)
            {
                /* Voltage has dropped below 5 V. We can now turn off the FET and continue discharge. */
                psrc_shutdown(port, false);
            }

            if (vbus_is_present(port, VSAFE_0V, VBUS_TURN_ON_MARGIN) == false)
            {
                /* Start Extra discharge to allow proper discharge below Vsafe0V */
                timer_start(port, APP_PSOURCE_DIS_EXT_DIS_TIMER,
                        APP_PSOURCE_DIS_EXT_DIS_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            else
            {
                /* Start Monitor Timer again */
                timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER,
                        APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD,
                        app_psrc_tmr_cbk);
            }
            break;

        case APP_PSOURCE_DIS_EXT_DIS_TIMER:
            timer_stop(port, APP_PSOURCE_DIS_TIMER);
            vbus_discharge_off(port);

            /* Notify the caller that psrc_disable is complete. */
            call_psrc_ready_cbk(port);
            break;

#if CCG_PPS_SRC_ENABLE
        case APP_PSOURCE_CF_TIMER:
            if (app_stat->cur_fb_enabled)
            {
                /*
                 * This task needs to be invoked every 100ms when in PPS mode
                 * operation for the state machine to function correctly. Since
                 * the function expects the VBUS to have stabilized before
                 * calling, it is being called from this timer interrupt handler.
                 */
                dpm_pps_task(port);
                timer_start(port, APP_PSOURCE_CF_TIMER,
                        APP_PSOURCE_CF_TIMER_PERIOD, app_psrc_tmr_cbk);
            }
            break;
#endif /* CCG_PPS_SRC_ENABLE */

        default:
            break;
    }
}

#if VBUS_OCP_ENABLE

#if HX3PD_DS1_OCP_PIN_WORKAROUND
CY_ISR (pwren_intr_handler)
{
    if(gpio_get_intr(HX3PD_DS1_OCP_PIN))
    {
        CyIntDisable (HX3PD_DS1_OCP_PIN >> 4);
        gpio_clear_intr (HX3PD_DS1_OCP_PIN);
        dpm_clear_fault_active(TYPEC_PORT_1_IDX);
        dpm_start(TYPEC_PORT_1_IDX);
    }
}
#endif /* HX3PD_DS1_OCP_PIN_WORKAROUND */

void app_psrc_vbus_ocp_cbk(uint8_t port)
{
    timer_stop_range(port, APP_PSOURCE_EN_MONITOR_TIMER, APP_PSOURCE_EN_HYS_TIMER);

#if HX3PD_DS1_OCP_PIN_WORKAROUND
    if(gpio_read_value(HX3PD_DS1_OCP_PIN))
    {
        /* Configure input from PWREN to handle events. */
        CyIntDisable (HX3PD_DS1_OCP_PIN >> 4);
        gpio_clear_intr (HX3PD_DS1_OCP_PIN);
        gpio_int_set_config (HX3PD_DS1_OCP_PIN, GPIO_INTR_FALLING);
        CyIntSetVector (HX3PD_DS1_OCP_PIN >> 4, pwren_intr_handler);
        CyIntEnable (HX3PD_DS1_OCP_PIN >> 4);
    }
#endif /* HX3PD_DS1_OCP_PIN_WORKAROUND */

    call_psrc_ready_cbk(port);

#ifdef CCG6
    /* TODO: See if this is really required. */
    PDSS->pgdo_pu_1_cfg |= PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
#endif /* CCG6 */

    /* OCP fault. */
    psrc_shutdown(port, true);

    /* Set alert message */
    pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = true;
    dpm_set_alert(port, alert);

    /* Enqueue HPI OVP fault event. */
    app_event_handler(port, APP_EVT_VBUS_OCP_FAULT, NULL);

}
#endif /* VBUS_OCP_ENABLE */


#if VBUS_SCP_ENABLE
void app_psrc_vbus_scp_cbk(uint8_t port)
{
    /* SCP fault. */
    psrc_shutdown(port, true);

    /* Set alert message */
    pd_do_t alert;
    alert.val = 0;
    alert.ado_alert.ocp = true;
    dpm_set_alert(port, alert);

    /* Enqueue HPI SCP fault event. */
    app_event_handler(port, APP_EVT_VBUS_SCP_FAULT, NULL);
}
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
void app_psrc_vbus_rcp_cbk(uint8_t port)
{
    /* RCP fault. */
    psrc_shutdown(port, true);

    /* Enqueue HPI RCP fault event. */
    app_event_handler(port, APP_EVT_VBUS_RCP_FAULT, NULL);
}
#endif /* VBUS_RCP_ENABLE */

#if ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE))

static void ovp_pwr_ready_cbk(uint8_t port)
{
    /* Dummy callback to allow vbus discharge */
}

void app_psrc_vbus_ovp_cbk(uint8_t port, bool comp_out)
{
    app_status_t *app_stat = app_get_status(port);

#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
    bool delay_shutdown = false;

    /*
     * If this is UVP and we are in PPS mode, we should not increment the
     * fault count. Also, we should be disabling the port before sending the
     * hard reset.
     */
    if ((app_stat->cur_fb_enabled) && (comp_out == false))
    {
        delay_shutdown = true;
    }

    /* Drop source voltage to VSAFE_0V. */
    if (delay_shutdown == false)
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    {
        app_stat->psrc_volt = VSAFE_0V;
        psrc_select_voltage(port);

        /*OVP fault*/
        psrc_shutdown(port, true);
    }

#ifndef CCG4
    /* Enqueue HPI fault event */
    if (comp_out == true)
#endif /* CCG4 */
    {
        /* OVP */
        app_event_handler(port, APP_EVT_VBUS_OVP_FAULT, NULL);
        psrc_disable(port, ovp_pwr_ready_cbk);
    }
#ifndef CCG4
    else
    {
        /* UVP */
        app_event_handler(port, APP_EVT_VBUS_UVP_FAULT, NULL);
#if ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE))
        if (delay_shutdown == true)
        {
            psrc_shutdown(port, true);
        }
#endif /* ((VBUS_UVP_ENABLE) && (CCG_PPS_SRC_ENABLE)) */
    }
#endif /* CCG4 */
}
#endif /* ((VBUS_OVP_ENABLE) || (VBUS_UVP_ENABLE)) */

void psrc_select_voltage(uint8_t port)
{
    app_status_t *app_stat = app_get_status(port);
#if VBUS_OFFSET_VOLTAGE_ENABLE
    pwr_params_t *ptr = pd_get_ptr_pwr_tbl(port);
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */

#if CCG_PROG_SOURCE_ENABLE
    uint32_t select_volt = app_stat->psrc_volt;

    /* Don't drop voltage below 5 V. */
    if (app_stat->psrc_volt == VSAFE_0V)
    {
        app_stat->psrc_volt = VSAFE_5V;
    }

#if VBUS_OFFSET_VOLTAGE_ENABLE
    /*
     * Add Vbus offset voltage.
     * Since this is a new parameters in the configuratiopn table, verify if the
     * length matches to include new field.
     * And Vbus offset is not applicable for PPS supply type.
     */
    if(ptr->table_len == sizeof(pwr_params_t))
    {
        if(dpm_get_info(port)->src_sel_pdo.src_gen.supply_type != PDO_AUGMENTED)
        {
            select_volt += ptr->vbus_offset_volt ;
        }
    }
#endif /* VBUS_OFFSET_VOLTAGE_ENABLE */
    
#if CCG_CABLE_COMP_ENABLE
    if (cable_comp_is_enabled(port) == true)
    {
        select_volt += cable_comp_get_voltage(port);
    }
#endif /* CCG_CABLE_COMP_ENABLE */

    /* 
     * Cap the selected voltage to the absolute maximum voltage that can be
     * applied to the cable. 
     */ 
    if (select_volt > VBUS_MAX_VOLTAGE)
    {
        select_volt = VBUS_MAX_VOLTAGE;
    }

    if(port == TYPEC_PORT_0_IDX)
    {
#if NCP_MANAGEMENT
        timer_start(port,NCP_VOLTCHANGE_DELAY_ID,NCP_VOLTCHANGE_DELAY_PERIOD,ncp_manage_cbk);
#else
        APP_VBUS_SET_VOLT_P1(select_volt);
#endif /* NCP_MANAGEMENT */
    }
#if CCG_PD_DUALPORT_ENABLE
    else
    {
        APP_VBUS_SET_VOLT_P2(select_volt);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

#else /* CCG_PROG_SOURCE_ENABLE */

    uint8_t intr_state = CyEnterCriticalSection();

    if (port == TYPEC_PORT_0_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case VSAFE_9V:
                APP_VBUS_SET_9V_P1();
                break;
            case VSAFE_12V:
                APP_VBUS_SET_12V_P1();
                break;
            case VSAFE_13V:
                APP_VBUS_SET_13V_P1();
                break;
            case VSAFE_15V:
                APP_VBUS_SET_15V_P1();
                break;
            case VSAFE_19V:
                APP_VBUS_SET_19V_P1();
                break;
            case VSAFE_20V:
                APP_VBUS_SET_20V_P1();
                break;
            default:
                app_stat->psrc_volt = VSAFE_5V;
                APP_VBUS_SET_5V_P1();
                break;
        }
    }
#if CCG_PD_DUALPORT_ENABLE
    if(port == TYPEC_PORT_1_IDX)
    {
        switch (app_stat->psrc_volt)
        {
            case VSAFE_9V:
                APP_VBUS_SET_9V_P2();
                break;
            case VSAFE_12V:
                APP_VBUS_SET_12V_P2();
                break;
            case VSAFE_13V:
                APP_VBUS_SET_13V_P2();
                break;
            case VSAFE_15V:
                APP_VBUS_SET_15V_P2();
                break;
            case VSAFE_19V:
                APP_VBUS_SET_19V_P2();
                break;
            case VSAFE_20V:
                APP_VBUS_SET_20V_P2();
                break;
            default:
                app_stat->psrc_volt = VSAFE_5V;
                APP_VBUS_SET_5V_P2();
                break;
        }
    }
#endif /* CCG_PD_DUALPORT_ENABLE */
    CyExitCriticalSection(intr_state);
#endif  /* CCG_PROG_SOURCE_ENABLE */
}

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
void app_psrc_ld_sw_cbk(uint8_t port, bool state)
{
    uint16_t volt_mV = app_get_status(port)->psrc_volt;

    /* We need to ensure that we are still active to filter out race conditions. */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * The load is above the threshold. Disable the load switch and configure
         * SR comparator to trigger on falling edge.
         */
        if (state)
        {
            APP_PSRC_LD_SW_OFF(port);
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        }
        else
        {
            APP_PSRC_LD_SW_ON(port);
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_EN_NEG_DIS, app_psrc_ld_sw_cbk);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        }
    }
    else
    {
#if (!CCG_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_get_status(port)->ld_sw_ctrl = false;
    }
}

void psrc_ld_sw_ctrl(uint8_t port, uint16_t volt_mV)
{
    uint8_t intr_state;
    app_status_t *app_stat = app_get_status(port);

    intr_state = CyEnterCriticalSection();
    /*
     * Turn ON the load switch if the requested voltage and current are below
     * the threshold specified. Since this voltage is always expected to be
     * below 5V, it is possible only when PPS is active.
     */
    if ((volt_mV < APP_VBUS_LD_SW_VOLT_MIN) && (volt_mV > VSAFE_0V))
    {
        /*
         * If the load switch is already active, then do nothing. If not, configure
         * the SR comparator to interrupt when the current goes below the threshold.
         */
        if (app_stat->ld_sw_ctrl == false)
        {
#if (!CCG_CABLE_COMP_ENABLE)
            pd_set_sr_comp(port, (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES),
                    FILTER_CFG_POS_DIS_NEG_EN, app_psrc_ld_sw_cbk);
#else
            /* 
             * Use Vbus current change from cable compensation callback.
             * And enable load switch to add initial load on the Vbus.
             */
            APP_PSRC_LD_SW_ON(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
            app_stat->ld_sw_ctrl = true;
        }
    }
    else
    {
        /*
         * Disable the SR comparator and disable the load switch.
         */
#if (!CCG_CABLE_COMP_ENABLE)
        pd_stop_sr_comp(port);
#endif /* (!CCG_CABLE_COMP_ENABLE) */
        APP_PSRC_LD_SW_OFF(port);
        app_stat->ld_sw_ctrl = false;
    }

    CyExitCriticalSection(intr_state);
}
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if (CCG_CABLE_COMP_ENABLE)
/* 
 * This function will be called on every CABLE_COMP_HYS change in Vbus current.
 * vbus_cur is in 10mA units.
 */
void app_psrc_vbus_cur_cbk(uint8_t port, uint16_t vbus_cur)
{
#if APP_VBUS_LOAD_SWITCH_ENABLE 
    app_status_t *app_stat = app_get_status(port);

    if (app_stat->ld_sw_ctrl == true)
    {
        if(vbus_cur > (APP_VBUS_LD_SW_CUR_MIN + APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, true);
        }
        else if(vbus_cur < (APP_VBUS_LD_SW_CUR_MIN - APP_VBUS_LD_SW_CUR_THRES))
        {
            app_psrc_ld_sw_cbk(port, false);
        }
    }
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */
}
#endif /* (CCG_CABLE_COMP_ENABLE) */

void psrc_set_voltage(uint8_t port, uint16_t volt_mV)
{
    app_status_t *app_stat = app_get_status(port);
    app_stat->psrc_volt = volt_mV;
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
       /**
        * Requirement is to Send REQUESTED VBUS VOLTAGE to FX3 details along with Keyword, Interrupt Status
        * So As Vbus details wont be available in request packet we need to track decode Vbus details from 
        * SrcCaps packets based on the Requested PDO INDEX,  
        * And one more thing is, Decoding Vbus details from respective Src caps packet,
        * Decoding will be different for different supply type so track SUPPLY_TYPE from Srccap packet and 
        * and based on supply type decode VBUS details and fill them in I2C buffer and generate interrupt.
        *
        */
        uint8_t i = 0;
        uint32_t lCurrentPDSSSrcCaps[7] = {0};//7PDOs each of 4Bytes so 28Bytes 2 Bytes Reserved
        uint32_t lPresentRequestPacket = 0;
        uint8_t lPrReqObjPos = 0;
        uint8_t lPrReqSupplyType = 0;
        
        uint16_t lPrReqMaxVoltage = 0, lPrReqBattMaxOpPower = 0;
        //uint16_t lPrReqMinVoltage;
        uint16_t lPrReqMaxCurrent = 0;
        PD_BC_i2cBufHandler(INTR_BUF_CLR);
        if( !isAttachInterrupt )
        {
            //gEventLogger[gEventLogIndex++] = dpm_stat->src_cur_rdo.val;/*GRL-EDIT, Tracking received Request*/
            
            lPresentRequestPacket = dpm_stat->src_cur_rdo.val;
            lPrReqObjPos = (lPresentRequestPacket & REQ_DO_EXTRACT) >> REQ_DO_POS_OFFSET;
            
            for( i = 0; i < MAX_NO_OF_PDO; i++)
            {   
                lCurrentPDSSSrcCaps[i] = dpm_stat->src_pdo[i].val;
            }
            lPrReqSupplyType = ((lCurrentPDSSSrcCaps[(lPrReqObjPos-1)] & SRCCAPS_SUPPLYTYPE_EXTRACT) >> SRCCPAS_DO_POS_OFFSET );
            g_Struct_Ptr->RequestPacketConfig.gReqSupplyType = lPrReqSupplyType;
            switch(lPrReqSupplyType)
            {
                case PDO_FIXED_SUPPLY:
                    
                    lPrReqMaxVoltage = ( (lCurrentPDSSSrcCaps[(lPrReqObjPos-1)] & (SRCCPAS_MAX_V_OFFSET << FIXED_PDO_MAX_V_OFFSET) ) >> FIXED_PDO_MAX_V_OFFSET );
                    lPrReqMaxVoltage *= 50;//1 uint = 50mV so converting it to 1u =1mV
                    
                    lPrReqMaxCurrent = (lPresentRequestPacket & REQ_MAX_I_OFFSET);
                    lPrReqMaxCurrent *= 10;//1 uint = 10mA so converting it to 1u =1mA
                    
                break;
                case PDO_BATTERY:
                    
                    lPrReqMaxVoltage = ( (lCurrentPDSSSrcCaps[(lPrReqObjPos-1)] & (SRCCPAS_MAX_V_OFFSET << BATT_PDO_MAX_V_OFFSET) ) >> BATT_PDO_MAX_V_OFFSET );
                    lPrReqMaxVoltage *= 50;//1 uint = 50mV so converting it to 1u =1mV
                    
                    lPrReqBattMaxOpPower = (lPresentRequestPacket & REQ_MAX_POWER_BATTERY_OFFSET);
                    lPrReqBattMaxOpPower *= 250;// 1 unit = 250 mW , so converting to 1unit = 1mW
                    //lPrReqBattMaxOpPower /= 1000; //mW to W
                   
                    lPrReqMaxCurrent = (lPrReqBattMaxOpPower / lPrReqMaxVoltage); // I = P/V; as we need to derive max current would be required
                    lPrReqMaxCurrent *= 1000;//derived current will be in Amps so converting to mAmps
                    break;
                case PDO_VARIABLE_SUPPLY:
                    
                    lPrReqMaxVoltage = ( (lCurrentPDSSSrcCaps[(lPrReqObjPos-1)] & (SRCCPAS_MAX_V_OFFSET << VAR_PDO_MAX_V_OFFSET) ) >> VAR_PDO_MAX_V_OFFSET );
                    lPrReqMaxVoltage *= 50;//1 uint = 50mV so converting it to 1u =1mV
                    
                    lPrReqMaxCurrent = (lPresentRequestPacket & REQ_MAX_I_OFFSET);
                    lPrReqMaxCurrent *= 10;//1 uint = 10mA so converting it to 1u =1mA
                    break;
                case PDO_AUGMENTED:  
                    
                    lPrReqMaxVoltage = ((lPresentRequestPacket & (APDO_REQ_OUT_V_BITSOFFSET << APDO_REQ_OUT_V_OFFSET)) >> APDO_REQ_OUT_V_OFFSET);
                    lPrReqMaxVoltage *= 20;//1unit = 20mV so converting to 1unit = 1mv
                    
                    lPrReqMaxCurrent = (lPresentRequestPacket & REQ_APDO_OP_I_OFFSET);
                    lPrReqMaxCurrent *= 50;//1 uint = 50mA so converting it to 1unit =1mA
                    
                    /**Pranay, 14Dec'22, 
                    *  Inorder not to generate an interrupt everytime when there is a 
                    *  APDO request from Sink for every 7-13Sec, 
                    *  tracking here if there is any change in previous APDO req Vbus v and i, 
                    *  only If there is any change in Vbus v or i, then request to be considered as new request and hence set 'isNewAPDORequest' flag 
                    *  and update gPrevReqAPDOVbusVoltage, gPrevReqAPDOVbusCurrent to the new vbus v and i values and then generate interrupt below
                    **/
                    if( (g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusVoltage != lPrReqMaxVoltage) || 
                            (g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusCurrent != lPrReqMaxCurrent) 
                        )
                    {
                        g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusVoltage = lPrReqMaxVoltage;
                        g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusCurrent = lPrReqMaxCurrent;
                        g_Struct_Ptr->RequestPacketConfig.isNewAPDORequest = true;
                    }
                    break;
            }
            
            i2cBuf[2] = isAttachInterrupt;//[b0]
            i2cBuf[2] |= (g_Struct_Ptr->RequestPacketConfig.gDUTSpecRev << 1);//[b2:1]
            i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
            i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
            i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);
            
            i2cBuf[4] = lPrReqMaxVoltage;
            i2cBuf[5] = lPrReqMaxVoltage >> 8;
            i2cBuf[6] = lPrReqMaxCurrent;
            i2cBuf[7] = lPrReqMaxCurrent >> 8;
            
            PD_BC_i2cBufHandler(INTR_REQ_STATE);
            schedule_task(50, GRL_SRC_PSRDY_TIMER);

        }
        else
        {
            i2cBuf[2] = isAttachInterrupt;
            i2cBuf[2] |= (g_Struct_Ptr->RequestPacketConfig.gDUTSpecRev << 1);//[b2:1]
            i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
            i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
            i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);
            i2cBuf[4] = 0x88;//5000mV
            i2cBuf[5] = 0x13;
            i2cBuf[6] = 0x64;//100mA- configured as suspended current but it shouldn't be more than 10mA
            i2cBuf[7] = 0x00;
            
            PD_BC_i2cBufHandler(INTR_ATTACHSTATE);
            isAttachInterrupt = false;
            schedule_task(20, GRL_SRC_ATTACH_INTR_TIMER);
        }
        /**Pranay,14Dec'22, Generating interrupt only if there is any change in prev requested and present request Vbus v/i incase of APDO*/
        if( (g_Struct_Ptr->RequestPacketConfig.gReqSupplyType == PDO_AUGMENTED) 
                && (g_Struct_Ptr->RequestPacketConfig.isNewAPDORequest) )
        {
            g_PdssGPIOIntrHandler(INTR_SET);
        }
        else if( g_Struct_Ptr->RequestPacketConfig.gReqSupplyType != PDO_AUGMENTED )
        {
            g_PdssGPIOIntrHandler(INTR_SET);
        }
            
    }
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        APP_VBUS_SET_VOLT_P2 (volt_mV);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Ensure that the feedback control logic is turned on. */
    vbus_ctrl_fb_enable(port);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if CCG_CABLE_COMP_ENABLE
    cable_comp_enable(port, app_psrc_vbus_cur_cbk);
    cable_comp_cfg(port, volt_mV);
#endif /* CCG_CABLE_COMP_ENABLE */

#if CCG_PPS_SRC_ENABLE
    if (dpm_get_info(port)->src_sel_pdo.fixed_src.supply_type == PDO_AUGMENTED)
    {
#if (CCG_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE)
        cable_comp_disable(port);
#endif /* (CCG_CABLE_COMP_ENABLE) && (CCG_CABLE_COMP_IN_PPS_DISABLE) */
        psrc_en_cf(port);
    }
    else
    {
        psrc_dis_cf(port);
    }
#endif /* CCG_PPS_SRC_ENABLE */

    /* Setup the load switch control. */
#if APP_VBUS_LOAD_SWITCH_ENABLE
    psrc_ld_sw_ctrl(port, volt_mV);
#endif /* APP_VBUS_LOAD_SWITCH_ENABLE */

#if VBUS_OCP_ENABLE
    /* Leave OCP detection disabled while doing the voltage transition. */
    psrc_dis_ocp (port);
#endif /* VBUS_OCP_ENABLE */

    if ((app_stat->psrc_volt >= app_stat->psrc_volt_old) && (volt_mV != VSAFE_0V))
    {
        /* If voltage is being increased, make sure that OVP and RCP limits are moved up. */
        psrc_en_ovp (port);
        psrc_en_rcp (port);
    }
    else if ((app_stat->psrc_volt < app_stat->psrc_volt_old) && (volt_mV != VSAFE_0V))
    {
        /*
         * Enable UVP only if port partner is attached. We need to ensure that
         * UVP does not get enabled if VBUS is not applied, like in case of
         * HARD_RESET.
         */
        if ((dpm_stat->attach == true) && (app_stat->is_vbus_on == true))
        {
           psrc_en_uvp (port);
        }
    }
    
    g_PdssGPIOIntrHandler(INTR_CLR);

    //psrc_select_voltage(port);//GRL-EDIT
}

uint32_t psrc_get_voltage (uint8_t port)
{
#if CCG_CABLE_COMP_ENABLE
    return app_get_status(port)->psrc_volt + cable_comp_get_voltage(port);
#else /* !CCG_CABLE_COMP_ENABLE */
    return app_get_status(port)->psrc_volt;
#endif /* CCG_CABLE_COMP_ENABLE */
}

#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
static const uint32_t cc_rp_to_cur_map[] = {
    CUR_LEVEL_DEF,
    CUR_LEVEL_1_5A,
    CUR_LEVEL_3A
};
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

void psrc_set_current (uint8_t port, uint16_t cur_10mA)
{
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
    /* Update the OCP/SCP thresholds when required. */
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint32_t ocp_cur;

#if VBUS_CF_EN
    app_status_t* app_stat = app_get_status(port);
    (void)app_stat;//GRL-EDIT
#endif /* VBUS_CF_EN */

    if (dpm_stat->contract_exist)
    {
#if CCG_PPS_SRC_ENABLE
        if (dpm_stat->src_sel_pdo.src_gen.supply_type == PDO_AUGMENTED)
        {
            /* Convert in 10mA units */
            ocp_cur = dpm_stat->src_sel_pdo.pps_src.max_cur * 5;
        }
        else
#endif /* CCG_PPS_SRC_ENABLE */
        {
            ocp_cur = dpm_stat->src_sel_pdo.src_gen.max_cur_power;
        }
    }
    else
    {
        ocp_cur = cc_rp_to_cur_map[dpm_stat->src_cur_level];

#if BATTERY_CHARGING_ENABLE
        /* Update current limit as per battery charging module */
        if (
                (bc_get_status(port)->cur_mode != BC_CHARGE_NONE) &&
                (bc_get_status(port)->cur_mode != BC_CHARGE_CDP)
           )
        {
            ocp_cur = bc_get_status(port)->cur_amp;
        }
#endif /* BATTERY_CHARGING_ENABLE */
    }

#if VBUS_OCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OCP_EN_MASK)
    {
#if VBUS_CF_EN
        /* Enable OCP only if not in current foldback mode. */
        if (app_stat->cur_fb_enabled == false)
#endif /* VBUS_CF_EN */
        {
            system_vbus_ocp_en(port, ocp_cur, app_psrc_vbus_ocp_cbk);
        }
    }
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
    /* Enable SCP. */
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_SCP_EN_MASK)
    {
        system_vbus_scp_en (port, ocp_cur, app_psrc_vbus_scp_cbk);
    }
#endif /* VBUS_SCP_ENABLE */

#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */
}

void psrc_enable (uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Turn on PSource FET for TYPE-A port. */
        vbus_fet_on(port);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    app_status_t* app_stat = app_get_status(port);
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    uint8_t       intr_state;

    intr_state = CyEnterCriticalSection();

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);

    /* Turn on FETs only if dpm is enabled and there is no active fault condition. */
    if ((dpm_stat->dpm_enabled) && (dpm_stat->fault_active == false))
    {
#if (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE)
        /* Make sure OCP/SCP is enabled where required. The current parameter is not used. */
        psrc_set_current (port, CUR_LEVEL_3A);
#endif /* (VBUS_OCP_ENABLE || VBUS_SCP_ENABLE) */

        /* Turn off VBus Discharge by default. */
        vbus_discharge_off(port);

        /* Turn on PSource FET */
        vbus_fet_on(port);

        if (pwr_ready_handler != NULL)
        {
            app_stat->psrc_rising = true;

            /* If the VBus voltage is dropping, turn the discharge path on. */
            if(app_stat->psrc_volt_old > app_stat->psrc_volt)
            {
                app_stat->psrc_rising = false;
                vbus_discharge_on(port);
            }
            app_stat->pwr_ready_cbk = pwr_ready_handler;

            /* Start Power source enable and monitor timer */
            timer_start(port, APP_PSOURCE_EN_TIMER,
                    APP_PSOURCE_EN_TIMER_PERIOD, app_psrc_tmr_cbk);
            /* For CCG3PA/CCG3PA2 APP_PSOURCE_EN_MONITOR_TIMER is not required */
#if ((!defined(CCG3PA)) && (!defined(CCG3PA2)))
            timer_start(port, APP_PSOURCE_EN_MONITOR_TIMER,
                    APP_PSOURCE_EN_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
#endif /* ((!defined(CCG3PA)) && (!defined(CCG3PA2))) */
        }
    }

    CyExitCriticalSection(intr_state);
}

void psrc_disable(uint8_t port, pwr_ready_cbk_t pwr_ready_handler)
{
    app_status_t* app_stat = app_get_status(port);
    uint8_t intr_state;

#if CCG_TYPE_A_PORT_ENABLE
    if (port == TYPE_A_PORT_ID)
    {
        /* Turn off PSource FET for TYPE-A port. */
        vbus_fet_off(port);
        return;
    }
#endif /* CCG_TYPE_A_PORT_ENABLE */

    timer_stop_range(port, APP_PSOURCE_EN_TIMER, APP_PSOURCE_DIS_EXT_DIS_TIMER);
#if CCG_PPS_SRC_ENABLE
    psrc_dis_cf(port);
#endif /* CCG_PPS_SRC_ENABLE */

#if VBUS_UVP_ENABLE
    psrc_dis_uvp(port);
#endif /* VBUS_UVP_ENABLE */

    if(app_stat->psrc_volt_old <= VSAFE_5V)
    {
        psrc_shutdown(port, false);
        CyDelayUs(20);
    }
    else
    {
        psrc_set_voltage(port, VSAFE_5V);
    }

    intr_state = CyEnterCriticalSection();
    app_stat->psrc_volt_old = VSAFE_0V;

    if ((pwr_ready_handler != NULL) && (dpm_get_info(port)->dpm_enabled))
    {
        /* Turn on discharge to get the voltage to drop faster. */
        vbus_discharge_on(port);
        app_stat->pwr_ready_cbk = pwr_ready_handler;

        /*Start Power source enable and monitor timer*/
        timer_start(port, APP_PSOURCE_DIS_TIMER,
                APP_PSOURCE_DIS_TIMER_PERIOD, app_psrc_tmr_cbk);
        timer_start(port, APP_PSOURCE_DIS_MONITOR_TIMER,
                APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD, app_psrc_tmr_cbk);
    }
    else
    {
        psrc_shutdown(port, true);
    }

    CyExitCriticalSection(intr_state);
}

static void psrc_dis_ovp(uint8_t port)
{
#if VBUS_OVP_ENABLE
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined (CCG5) || defined(CCG6) || defined(CCG5C))
    app_ovp_disable (port, CCG_SRC_FET);
#else /* CCG4 */
    app_ovp_disable (port, false);
#endif /* CCGx */
#endif /* VBUS_OVP_ENABLE */
}

static void psrc_dis_ocp(uint8_t port)
{
#if VBUS_OCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_OCP_EN_MASK)
    {
        system_vbus_ocp_dis(port);
    }
#endif /* VBUS_OCP_ENABLE */
}

static void psrc_dis_scp(uint8_t port)
{
#if VBUS_SCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_SCP_EN_MASK)
    {
        system_vbus_scp_dis (port);
    }
#endif /* VBUS_SCP_ENABLE */
}

static void psrc_dis_uvp(uint8_t port)
{
#if VBUS_UVP_ENABLE
#if CCG4_DOCK
    app_uvp_disable (port, true);
#else /* !CCG4_DOCK */
    app_uvp_disable (port, CCG_SRC_FET);
#endif /* CCG4_DOCK */
#endif /* VBUS_UVP_ENABLE */
}

static void psrc_dis_rcp(uint8_t port)
{
#if VBUS_RCP_ENABLE
    system_vbus_rcp_dis(port);
#endif /* VBUS_RCP_ENABLE */
}

static void psrc_shutdown(uint8_t port, bool discharge_dis)
{
    /*Turn Off Source FET*/
    vbus_fet_off(port);

    if(discharge_dis == true)
    {
        vbus_discharge_off(port);
    }

#if CCG_CABLE_COMP_ENABLE
    cable_comp_disable(port);
#endif /* CCG_CABLE_COMP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Ensure that the feedback control logic is turned off. */
    vbus_ctrl_fb_disable(port);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (APP_VBUS_LOAD_SWITCH_ENABLE)
    /* Ensure that the load switch is disabled. */
    psrc_ld_sw_ctrl(port, VSAFE_5V);
#endif /* (APP_VBUS_LOAD_SWITCH_ENABLE) */

    /* Disable OVP/OCP/UVP */
    psrc_dis_ovp(port);
    psrc_dis_uvp(port);
    psrc_dis_ocp(port);
    psrc_dis_scp(port);
    psrc_dis_rcp(port);

#if CCG_PPS_SRC_ENABLE
    psrc_dis_cf(port);
#endif /* CCG_PPS_SRC_ENABLE */
}

void psrc_en_ovp(uint8_t port)
{
#if (VBUS_OVP_ENABLE)
#if (defined(CCG3) || defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) || defined(CCG5C) || defined(CCG6))
    app_ovp_enable(port, app_get_status(port)->psrc_volt,
            CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#else /* CCG4 */
    app_ovp_enable(port, app_get_status(port)->psrc_volt,
            true, app_psrc_vbus_ovp_cbk);
#endif /* CCGx */
#endif /* (VBUS_OVP_ENABLE) */
}

void psrc_en_rcp(uint8_t port)
{
#if VBUS_RCP_ENABLE
    if (get_pd_port_config(port)->protection_enable & CFG_TABLE_RCP_EN_MASK)
    {
        system_vbus_rcp_en (port, app_psrc_vbus_rcp_cbk);
    }
#endif /* VBUS_RCP_ENABLE */
}

void psrc_en_uvp(uint8_t port)
{
#if VBUS_UVP_ENABLE
    /* Using the same callback as OVP as behavior is same. */
#if CCG4_DOCK
    app_uvp_enable(port, app_get_status(port)->psrc_volt, true, app_psrc_vbus_ovp_cbk);
#else
    app_uvp_enable(port, app_get_status(port)->psrc_volt, CCG_SRC_FET, app_psrc_vbus_ovp_cbk);
#endif /* CCG4_DOCK */
#endif /* VBUS_UVP_ENABLE */
}

/* End of File */
