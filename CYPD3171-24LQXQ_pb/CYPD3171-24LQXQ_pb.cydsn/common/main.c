/**
 * @file main.c
 *
 * @brief @{Main source file for CCG firmware implementation.@}
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

#include <project.h>
#include <flash_config.h>
#include <system.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
#include <psink.h>
#include <swap.h>
#include <vdm.h>
#include <pdo.h>
#include <app.h>
#include <hal_ccgx.h>
#include <timer.h>
#include <hpi.h>
#include <hpd.h>
#include <boot.h>
#include <flash.h>
#include <status.h>
#include <ccgx_version.h>
#include <app_version.h>
#include <utils.h>
#include <gpio.h>
#include <instrumentation.h>
#include <grlapp.h>
#include <pd_protocol.h>


uint16 AUG_TIMER_Count = 0;
uint16_t AUG_TIMER_Config_Idx = 17;

#if RIDGE_SLAVE_ENABLE
#include <ridge_slave.h>
#endif /* RIDGE_SLAVE_ENABLE */

#if (LEGACY_APPLE_SRC_EXT_TERM_ENABLE)
#include <chgb_hal.h>
#include <battery_charging.h>
#endif /* (LEGACY_APPLE_SRC_EXT_TERM_ENABLE) */

#define PORT_START_IDX (TYPEC_PORT_0_IDX)
/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
__attribute__ ((section(".customer_region"), used))
const uint32_t customer_info[8] = {0x00};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[5] = {0};

#if CCG_FIRMWARE_APP_ONLY
/*
   Provide a variable required by the HPI library. The real variable only
   exists if the boot-loadable component is included.
 */
volatile uint32_t cyBtldrRunType = 0;
#endif /* CCG_FIRMWARE_APP_ONLY */

#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE        

/* Global variable to hold the battery status */
uint32 gl_bat_stat = CCG_EXT_MSG_INVALID_BAT_REF;
/* Global variable to send the battery capability response */
static uint32 gl_bat_cap[3] = {
    CCG_PB_VID_PID,
    0x00000000,
    0x00000001
};

#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */       

/* Solution PD event handler */
void sln_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
    
#if (CCG_HPI_PD_ENABLE == 1) 
    /* Pass the callback to HPI */
    hpi_pd_event_handler(port, evt, data);
#else /* (CCG_HPI_PD_ENABLE == 0) */

    (void)port;
    
    if(evt == APP_EVT_HANDLE_EXTENDED_MSG) 
    {    
        pd_packet_extd_t * ext_mes = (pd_packet_extd_t * )data;
        if ((ext_mes->msg != EXTD_MSG_SECURITY_RESP) && (ext_mes->msg != EXTD_MSG_FW_UPDATE_RESP) 
#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE        
             &&(ext_mes->msg != EXTD_MSG_GET_BAT_STATUS) && (ext_mes->msg != EXTD_MSG_GET_BAT_CAP) 
#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */                   
           )
        {
            /* Send Not supported message */
            dpm_pd_command_ec(port, DPM_CMD_SEND_NOT_SUPPORTED, NULL, NULL);
        }

#if CCG_SLN_EXTN_MSG_HANDLER_ENABLE
        
        /* Handle the response messages here*/
        dpm_pd_cmd_buf_t cmd;
        const dpm_status_t * dpm_stat = dpm_get_info(port);
        if(ext_mes->msg == EXTD_MSG_GET_BAT_STATUS)
        {
            gl_bat_stat = CCG_EXT_MSG_INVALID_BAT_REF;
            /* Return the battery status
             * Here we only have a single fixed battery. So we compare with 0.
             * If there is more than one battery, this check has to be updated.
             */
            if(((ext_mes->dat[0].val) & 0xFFFF) == 0)
            {
                if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
                {
                    gl_bat_stat = CCG_EXT_MSG_VALID_BAT_STAT_SNK;
                }
                else
                {
                    gl_bat_stat = CCG_EXT_MSG_VALID_BAT_STAT_SRC;
                }
            }
         
            cmd.cmd_sop = SOP;
            cmd.cmd_do[0].val = gl_bat_stat;
            cmd.no_of_cmd_do = 1;
            cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
            dpm_pd_command_ec(port, DPM_CMD_SEND_BATT_STATUS, &cmd, NULL);
        }
        
        if(ext_mes->msg == EXTD_MSG_GET_BAT_CAP)
        {
            /* Initialize with invalid reference in request */
            gl_bat_cap[1] = 0x00;
            gl_bat_cap[2] = 0x01;
            if(((ext_mes->dat[0].val) & 0xFFFF) == 0)
            {
                /* Valid reference */
                gl_bat_cap[1] = (CCG_PB_BAT_FUL_CHG_CAP <<16) | CCG_PB_BAT_DES_CAP;
                gl_bat_cap[2] = 0x00;
            }
            
            cmd.cmd_sop = SOP;
            cmd.extd_hdr.val = 0x09;
            cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
            cmd.extd_type = EXTD_MSG_BAT_CAP;
            cmd.dat_ptr = (uint8 *)gl_bat_cap;
            dpm_pd_command_ec(port, DPM_CMD_SEND_EXTENDED,&cmd, NULL);
        }
#endif /* CCG_SLN_EXTN_MSG_HANDLER_ENABLE */         
}
#endif /* (CCG_HPI_PD_ENABLE) */
}

void pb_eval_src_cap(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler)
{
    /*
     * This function overrides default SRC CAP eval function. For, PB it's better to accept
     * whatever current source is offering.
     */

    /*
     * This fucntion assumes that PB has only 5V Fixed supply sink cap. We might have
     * update this if PB has higher voltage sink PDOs too.
     */

    pd_do_t snk_rdo;
    const dpm_status_t* dpm = dpm_get_info(port);

    snk_rdo.val = 0u;
    snk_rdo.rdo_gen.no_usb_suspend = dpm->snk_usb_susp_en;
    snk_rdo.rdo_gen.usb_comm_cap = dpm->snk_usb_comm_en;
    snk_rdo.rdo_gen.cap_mismatch = 0;
    snk_rdo.rdo_gen.obj_pos = 1;
    snk_rdo.rdo_gen.give_back_flag = 0;
    snk_rdo.rdo_gen.op_power_cur = src_cap->dat[0].fixed_src.max_current;
    snk_rdo.rdo_gen.min_max_power_cur = snk_rdo.rdo_gen.op_power_cur;

    app_get_resp_buf(port)->resp_do = snk_rdo;
    app_resp_handler(port, app_get_resp_buf(port));
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const app_cbk_t app_callback =
{
    app_event_handler,
    psrc_set_voltage,
    psrc_set_current,
    psrc_enable,
    psrc_disable,
    vconn_enable,
    vconn_disable,
    vconn_is_present,
    vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    psnk_set_voltage,
    psnk_set_current,
    psnk_enable,
    psnk_disable,
    eval_src_cap,
    eval_rdo,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
#if CCG_PD_REV3_ENABLE
    eval_fr_swap,
#else /* CCG_PD_REV3_ENABLE */
    NULL,
#endif /* CCG_PD_REV3_ENABLE */
    vbus_get_value,
    psrc_get_voltage
};

app_cbk_t* app_get_callback_ptr(uint8_t port)
{
    (void)port;
    /* Solution callback pointer is same for all ports */
    return ((app_cbk_t *)(&app_callback));
}

#if CCG_HPI_ENABLE
static void
update_hpi_regs (
        void)
{
#if CCG_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint8_t ver_invalid[8] = {0};

    /* Flash access is not allowed. */
    flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08;              /* FW2 is not valid. */
    hpi_set_mode_regs (mode, reason);
    hpi_update_versions (ver_invalid, (uint8_t *)&base_version, ver_invalid);
    hpi_update_fw_locations (0, CCG_LAST_FLASH_ROW_NUM);

#else /* !CCG_FIRMWARE_APP_ONLY */

    uint8_t mode, reason = 0x00;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0};

    /* Set mode variables and flash access limits based on the active firmware. */
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        mode = 0x81 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW2 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x08;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_IMG1_LAST_FLASH_ROW_NUM + 1, CCG_IMG2_LAST_FLASH_ROW_NUM,
                CCG_IMG2_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }
    else
    {
        mode = 0x82 | ((NO_OF_TYPEC_PORTS - 1) << 2) | ((CCG_FLASH_ROW_SHIFT_NUM - 7) << 4);

        /* Check if FW1 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR) != CCG_STAT_SUCCESS)
            reason = 0x04;

        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, gl_img2_fw_metadata->boot_last_row,
                CCG_IMG1_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
    }

    hpi_set_mode_regs (mode, reason);

    /* Calculate the version address from the firmware metadata. */
    if ((reason & 0x04) == 0)
    {
        fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
        fw1_ver = (((uint32_t)fw1_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw1_loc = fw1_md->boot_last_row + 1;
    }
    else
    {
        fw1_ver = (uint32_t)ver_invalid;
        fw1_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    if ((reason & 0x08) == 0)
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = (((uint32_t)fw2_md->boot_last_row + 1) << CCG_FLASH_ROW_SHIFT_NUM) + SYS_FW_VERSION_OFFSET;
        fw2_loc = fw2_md->boot_last_row + 1;
    }
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1;
    }

    /* Update version information in the HPI registers. */
    hpi_update_versions (
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );

    /* Update firmware location registers. */
    hpi_update_fw_locations (fw1_loc, fw2_loc);

#endif /* CCG_FIRMWARE_APP_ONLY */
}
#endif /* CCG_HPI_ENABLE */

#if APP_FW_LED_ENABLE

/* Blink the LED every LED_TIMER_PERIOD ms. This serves as an indication of the firmware running. */
void led_timer_cb (
    uint8_t port,
    timer_id_t id)
{
    (void)port;
    (void)id;

    gpio_set_value (FW_LED_GPIO_PORT_PIN, !(gpio_read_value (FW_LED_GPIO_PORT_PIN)));
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);
}

#endif /* APP_FW_LED_ENABLE */

#if (FLASHING_MODE_PD_ENABLE == 1)
/* Global to store Device Reset signature. */
uint32_t gl_reset_sig = 0;

static void pa_port_disable_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    (void)port;
    (void)resp;

    /* Disable all interrupts. */
    CyGlobalIntDisable;

#if !CCG_FIRMWARE_APP_ONLY
    /*
     * NOTE: Using Bootloader component provided variable
     * to store the signature. This makes sure that this value
     * is never overwritten by compiler and it ratains the value
     * through resets and jumps. We use lower two bytes of this
     * variable to store the siganture. */
    /* NOTE: Signature will be zero for RESET command. */
    cyBtldrRunType |= gl_reset_sig;
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* This delay will ensure that port partner sees a detach when CCG3PA is sink. */
    CyDelay (30);
    /* Call device reset routine. */
    CySoftwareReset ();
}

ccg_status_t uvdm_handle_device_reset(uint32_t reset_sig)
{
    /*
     * Notify PD stack to disable Port if in source mode or if in sink mode
     * and self powered. Else, just go through Reset without disabling port.
     */
    if ((dpm_get_info(0)->cur_port_role == PRT_ROLE_SOURCE) ||
        ((dpm_get_info(0)->cur_port_role == PRT_ROLE_SINK) &&
        ((PDSS->vreg_vsys_ctrl & PDSS_VREG_VSYS_CTRL_VREG_EN) == 0)))
    {
        if (dpm_typec_command (0, DPM_CMD_PORT_DISABLE, pa_port_disable_cb) ==
            CCG_STAT_SUCCESS)
        {
            /* Store Reset Signature. */
            gl_reset_sig = reset_sig;
            return CCG_STAT_NO_RESPONSE;
        }
        else
        {
            return CCG_STAT_FAILURE;
        }
    }
    else
    {
        gl_reset_sig = reset_sig;
        pa_port_disable_cb (0, 0);
        return CCG_STAT_NO_RESPONSE;
    }
}
#endif /*(FLASHING_MODE_PD_ENABLE == 1) */

#if (LEGACY_APPLE_SRC_EXT_TERM_ENABLE)
void sln_apply_apple_src_term(uint8_t cport, chgb_src_term_t charger_term)
{
    (void)charger_term;
    /*
     * This function assumes that there is at most one Type-C port and one Type-A
     * port in the system and is limited to CCG3PA device family use case.
     */
    if (cport == TYPEC_PORT_0_IDX)
    {
#ifdef CY_PINS_DP_APPLE_SRC_TERM_C_H
        chgb_remove_term(cport);
        chgb_apply_apple_src_dm(cport, CHGB_SRC_TERM_APPLE_2_4A);
        chgb_apply_src_term(cport, CHGB_SRC_TERM_DCP);
        chgb_apply_dp_pd(cport);
        DP_APPLE_SRC_TERM_C_Write(true);
        DP_APPLE_SRC_TERM_C_SetDriveMode(DP_APPLE_SRC_TERM_C_DM_STRONG);
#else /* (!CY_PINS_DP_APPLE_SRC_TERM_C_H) */
        /* Use the internal terminations. */
        chgb_apply_src_term(cport, charger_term);
#endif /* (CY_PINS_DP_APPLE_SRC_TERM_C_H) */
    }
#if (CCG_TYPE_A_PORT_ENABLE)
    else
    {
#ifdef CY_PINS_DP_APPLE_SRC_TERM_A_H
        chgb_remove_term(cport);
        chgb_apply_apple_src_dm(cport, CHGB_SRC_TERM_APPLE_2_4A);
        chgb_apply_src_term(cport, CHGB_SRC_TERM_DCP);
        chgb_apply_dp_pd(cport);
        DP_APPLE_SRC_TERM_A_Write(true);
        DP_APPLE_SRC_TERM_A_SetDriveMode(DP_APPLE_SRC_TERM_A_DM_STRONG);
#else /* (!CY_PINS_DP_APPLE_SRC_TERM_A_H) */
        /* Use the internal terminations. */
        chgb_apply_src_term(cport, charger_term);
#endif /* (CY_PINS_DP_APPLE_SRC_TERM_A_H) */
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE) */
}

void sln_remove_apple_src_term(uint8_t cport)
{
    chgb_remove_term(cport);

    if (cport == TYPEC_PORT_0_IDX)
    {
#ifdef CY_PINS_DP_APPLE_SRC_TERM_C_H
        DP_APPLE_SRC_TERM_C_SetDriveMode(DP_APPLE_SRC_TERM_C_DM_ALG_HIZ);
        DP_APPLE_SRC_TERM_C_Write(false);
#endif /* (CY_PINS_DP_APPLE_SRC_TERM_C_H) */
    }
#if (CCG_TYPE_A_PORT_ENABLE)
    else
    {
#ifdef CY_PINS_DP_APPLE_SRC_TERM_A_H
        DP_APPLE_SRC_TERM_A_SetDriveMode(DP_APPLE_SRC_TERM_A_DM_ALG_HIZ);
        DP_APPLE_SRC_TERM_A_Write(false);
#endif /* (CY_PINS_DP_APPLE_SRC_TERM_A_H) */
    }
#endif /* (CCG_TYPE_A_PORT_ENABLE) */
}
#endif /* (LEGACY_APPLE_SRC_EXT_TERM_ENABLE) */

CY_ISR(Timer_Int_Handler)
{
#ifdef GRL_PPS_TIMER_HANDLE 
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    AUG_TIMER_ClearInterrupt(AUG_TIMER_INTR_MASK_TC);
    
    AUG_TIMER_Count ++;
    if(AUG_TIMER_Count < AUG_TIMER_Config_Idx)
    {
        AUG_TIMER_WritePeriod(65535);
        AUG_TIMER_Start();
    }
    else
    {
        AUG_TIMER_Count = 0;
        AUG_TIMER_Stop();
        switch(gTimerVar)
        {
            case APDO_TIMER:
                g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = true;
                g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
                dpm_pd_command(0, DPM_CMD_GET_SRC_CAP, NULL, NULL);
            break;
            case ATTACH_INTR:
            case REQ_INTR:
                psrc_select_voltage(0);
            break;
        }


    }
 #endif/*GRL_PPS_TIMER_HANDLE*/
}

CY_ISR(MasterInputInt_ISR)
{
    MasterInput_ClearInterrupt();
    gI2cHandle_task();
}

void MsgTimerStart(uint16_t lTimerVal_mSec)
{
    AUG_TIMER_Config_Idx = (lTimerVal_mSec/680);
    AUG_TIMER_Count = 0;
    AUG_TIMER_WritePeriod(65535);
    AUG_TIMER_Start();
}

#if ONLY_PD_SNK_FUNC_EN
void grlPDSSAppHandling()
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    if(g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx)
    {
        switch(g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType)
        {
            case CCG_AS_CABLE_TESTER:
                timer_stop(0,GRL_APP_SOP1_TIMER);
                /**
                    Desc: As a cable tester, VUP shall initiate DiscID SOP1 packets and tracks response from Cable connected.
                    Assumptions: It is assumed that Vconn shall be sourced continuously by source DUT connected as port partner on other end of cable
                    TBD:Handling if cable IGNORES sent DISCID SOP1 packet 
                */
                g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
                g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = false;
                g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount = g_Decode_Rx_Ack();/**Global variable that tracks the received SOP1 ACK Buffer fill byte count**/
                g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = true;
                
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_IGNORE;
                pd_phy_refresh_roles(0);
            
            break;
            
            case CCG_AS_CABLE_AND_SNK:
            /**
            Desc: In this state machine if CCG gets an SOP1 DISC ID then It will be responding ACK/NAK/IGNORE based on the configuration from Application. 
                  For SOP1 SVID will respond with NAK(Hardcoded). By default response type will be ignore i.e., if received and DiscID sop1packets there will not be any response from CCG unless configured otherwise.
            Assumptions: As CCG doesnt have provision to read Vconn presence status, It will assume Vconn is present at the time of Disc ID reception.
                         - can also emualte as a cable if VConn pass through cable is used.
            TBD : Need to add Vconn fetching from Eload data from TC FX3 when interrupted by SOP1 DiscID.
            */
                if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
                {
                    PrepareAckPkt();
                    CyDelayUs(1000);
                    pd_phy_send_msg(0);
                    g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = false;
                    g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
                }
                
            break;
             
            default:
                break;
        }
#if 0        
	    /* Handle the device policy tasks for each PD port. */
        if((g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx) && (g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE))
        {
            PrepareAckPkt();
            CyDelayUs(1000);
            pd_phy_send_msg(0);
            g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = false;
            g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
            
        }
        if((g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType == CCG_AS_CABLE_TESTER) && (g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx))
        {
            /**TBD: Add decoding received ACK/NAK packet*/
            g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
        }
#endif
     }
}
#endif /**ONLY_PD_SNK_FUNC_EN*/
int main()
{
    uint32_t conf_addr;
    uint8_t  port;

    /* Remove internal feedback divider */
    //pd_remove_internal_fb_res_div();

#if CCG_HPI_ENABLE
    uint8_t i;
#endif /* CCG_HPI_ENABLE */

    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x= 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

#if CCG_FIRMWARE_APP_ONLY
    sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
#else /* !CCG_FIRMWARE_APP_ONLY */
    if ((uint32_t)&base_version < CCG_FW1_CONFTABLE_MAX_ADDR)
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_1);
#ifndef CCG_PSEUDO_METADATA_DISABLE
        /* Determine pseudo metadata table address for Image 1 based on last row of Image 1. */
        gl_img1_fw_pseudo_metadata = (sys_fw_metadata_t *)((CCG_IMG1_LAST_FLASH_ROW_NUM
                << CCG_FLASH_ROW_SHIFT_NUM) + (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE));
#endif /* CCG_PSEUDO_METADATA_DISABLE */
    }
    else
    {
        sys_set_device_mode (SYS_FW_MODE_FWIMAGE_2);
#ifndef CCG_PSEUDO_METADATA_DISABLE
        /* Determine pseudo metadata table address for Image 1 from the BOOT LAST row
         * parameter of Image 2. */
        gl_img1_fw_pseudo_metadata = (sys_fw_metadata_t *)((gl_img2_fw_metadata->boot_last_row
                << CCG_FLASH_ROW_SHIFT_NUM) + (CCG_FLASH_ROW_SIZE - CCG_METADATA_TABLE_SIZE));
#endif /* CCG_PSEUDO_METADATA_DISABLE */
    }
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Validate the signature and checksum of the configuration table. */
    conf_addr = (uint32_t)get_pd_config ();
    if ((boot_validate_configtable ((uint8_t *)conf_addr) != CCG_STAT_SUCCESS)
        || (!app_validate_configtable_offsets()))
    {
#if CCG_FIRMWARE_APP_ONLY
        /* Can't do anything if config table is not valid. */
        while (1);
#else /* !CCG_FIRMWARE_APP_ONLY */
        /* Erase the firmware metadata so that this binary stops loading. */
        if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
        {
            flash_row_clear (CCG_IMG1_METADATA_ROW_NUM);
        }
        else
        {
            flash_row_clear (CCG_IMG2_METADATA_ROW_NUM);
        }

        /* Now reset the device. */
        CySoftwareReset ();
#endif /* CCG_FIRMWARE_APP_ONLY */
    }

#ifndef CCG_PSEUDO_METADATA_DISABLE
    /* Check if Other FW image is pending to be validated. If other image is valid,
     * device will undergo reset and then jump to the other image. */
    boot_check_for_valid_fw ();
#endif /* CCG_PSEUDO_METADATA_DISABLE */

    
    
    system_init();

    /* Timer INIT has to be done first. */
    timer_init();

    pd_hal_disable_vreg (0);
    
    /* Enable global interrupts */
    CyGlobalIntEnable;

#if RIDGE_SLAVE_ENABLE
    /* Initialize the Alpine-Ridge slave interface. */
    ridge_slave_init (RIDGE_SLAVE_SCB_INDEX, 0);
#endif /* RIDGE_SLAVE_ENABLE */

#if CCG_HPI_ENABLE

#if CCG_FIRMWARE_APP_ONLY
    /* Configure HPI for no-boot operation. */
    hpi_set_no_boot_mode (true);
#endif /* CCG_FIRMWARE_APP_ONLY */

    /* Set the flash sizes and bootloader size limits. */
    hpi_set_flash_params (CCG_FLASH_SIZE, CCG_FLASH_ROW_SIZE, CCG_LAST_FLASH_ROW_NUM + 1, CCG_BOOT_LOADER_LAST_ROW);

    /* Initialize the HPI interface. */
    hpi_init (HPI_SCB_INDEX);

    /* Update HPI registers with mode and version information. */
    update_hpi_regs ();

#endif /* CCG_HPI_ENABLE */

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Update the VBUS RSENSE value for the application before starting the stack. */
    pd_hal_set_vbus_csa_rsense(VBUS_CSA_RSENSE);

    /* Perform application level initialization. */
    app_init();
    
    dpm_update_def_cable_cap(500);//GRL_EDIT, TO make src advertise more than 3A, shall call this only once before DPM initialised
    
    /* Initialize the Device Policy Manager for each PD port on the device. */
    for(port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
        dpm_init(port, app_get_callback_ptr(port));
    }

#if CCG_HPI_ENABLE

    /* Send a reset complete event to the EC. */
    hpi_send_fw_ready_event ();

    /* Wait until EC ready event has been received or 100 ms has elapsed. */
    for (i = 0; i < 100; i++)
    {
        hpi_task ();
        if (hpi_is_ec_ready ())
            break;

        CyDelay (1);
    }

#endif /* CCG_HPI_ENABLE */

    /* Start any timers or tasks associated with application instrumentation. */
    instrumentation_start();

    for (port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
    {
#if CCG_HPI_ENABLE
        /* Start the DPM for the port only if it is enabled at the HPI level. It is possible that the port
           got disabled before we got here.
         */
        if ((hpi_get_port_enable () & (1 << port)) != 0)
#endif /* CCG_HPI_ENABLE */
        {
            dpm_start(port);
        }
    }

#if APP_FW_LED_ENABLE

    /* Configure the LED control GPIO as an output. */
    gpio_hsiom_set_config (FW_LED_GPIO_PORT_PIN, HSIOM_MODE_GPIO, GPIO_DM_STRONG, true);
    /* Start a timer that will blink the FW ACTIVE LED, if required. */
    timer_start (0, LED_TIMER_ID, LED_TIMER_PERIOD, led_timer_cb);

#endif /* APP_FW_LED_ENABLE */

    /* Initialize CCG polling tasks. */
    ccg_app_task_init ();
    
    //dpm_set_delay_src_cap_start(0, DELAY_SRC_CAP_START_MS);
    dpm_set_delay_src_cap_start(0, 150);
    
    grlSystem_init();
 
    MasterInputInt_StartEx(MasterInputInt_ISR);
    
    AUG_ISR_StartEx(Timer_Int_Handler);  
    //gpio_set_value (GPIO_PORT_2_PIN_2,0);//GPIO LOW
    while (1)
    {
      /*venkat 6Jul'22
        TaskID - V-1-T314 - QC4.0 implementation
        This will stop PD policy engine,where PD state machine will be stopped,this is used when user need 
        to be switched from PD to QC mode.If its executed once it will go back to PD state machine again,
        it needs to be executed in contineous loop in order to be in Qc mode*/
        
        if((g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag != GRL_PD_MODE_SET) )
        {
            dpm_pe_stop(0);
        }
        
        /* Handle the device policy tasks for each PD port. */
        for (port = PORT_START_IDX ; port < NO_OF_TYPEC_PORTS; port++)
        {
            dpm_task(port);
            app_task(port);

#if CCG_HPI_ENABLE
            /* Handle any pending HPI commands. */
            hpi_task ();
#endif /* CCG_HPI_ENABLE */
            
            /* Perform tasks associated with instrumentation. */
            instrumentation_task();
        }
#if ONLY_PD_SNK_FUNC_EN
        /**GRL-PDSS Explicit requirements Handled here**/
#ifdef DBGMODE
       gI2cHandle_task();
#else
        grlPDSSAppHandling();
#endif
#endif /**ONLY_PD_SNK_FUNC_EN*/
#if SYS_DEEPSLEEP_ENABLE
        /* If possible, enter sleep mode for power saving. */
        system_sleep();
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* End of file */
