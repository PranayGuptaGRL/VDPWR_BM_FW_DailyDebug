/**
 * @file ucsi.c
 *
 * @brief @{USB Type-C Connector System Software Interface (UCSI) source file.@}
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
#include <stdio.h>
#include <string.h>

#include "i2c.h"
#include "ucsi_internal.h"
#include "ucsi.h"
#include "hpi.h"

#include "pd.h"
#include "hpd.h"
#include "dpm.h"
#include "app.h"
#include "psource.h"
#include "alt_mode_hw.h"
#include "alt_modes_mngr.h"
#include "typec_manager.h"


#include "timer.h"
#include "dpm_intern.h"

#if CCG_UCSI_ENABLE

bool modal_op_support = false;    
    
/* External functions declared here for SET_POWER_LEVEL */
extern pd_do_t* ucsi_change_pdo_power(uint8_t port, uint8_t power, uint8_t is_src, uint8_t *out_mask);    

/* 
 * Check PPM command for completion 
 */
#define ppm_cmd_requires_completion(cmdcode)    ((cmdcode != UCSI_CMD_PPM_RESET) && (cmdcode != UCSI_CMD_SET_NOTIFICATION_ENABLE))
    
/**
 * Enum to Hold status of DPM command
**/
typedef enum 
{ 
    NO_CMD,             /* 0 */
    CMD_QUEUED,         /* 1 */
    CMD_IN_PROGRESS,    /* 2 */
    CMD_SUCCESS,        /* 3 */
    CMD_ERROR           /* 4 */
    
} dpm_cmd_status_t;

/* Map PD command to desired Rp level. */
const dpm_typec_cmd_t ucsi_pdcmd_rp_map[] =
{
    DPM_CMD_SET_RP_3A,         /* PPM Defined Default */
    DPM_CMD_SET_RP_3A,         /* UCSI_PD_CMD_SET_RP_3A0 */
    DPM_CMD_SET_RP_1_5A,       /* UCSI_PD_CMD_SET_RP_1A5 */
    DPM_CMD_SET_RP_DFLT        /* UCSI_PD_CMD_SET_RP_DFLT */
};

void ucsi_set_status(ucsi_status_indicator_t status);

/**
 *  @brief UCSI timeout callback function.
 *  @param port port index
 *  @param timer_id_t timer id.
 *  @return NULL
 */
static void ucsi_tmr_cbk(uint8_t port, timer_id_t id);

ucsi_reg_space_t        ucsi_regs __attribute__ ((aligned (4))); /*UCSI Register space*/
#if CCG_UCSI_ENABLE_DIFF_ADDR
uint8_t                 ucsi_scb_idx = HPI_SCB_INDEX;
#endif /*CCG_UCSI_ENABLE_DIFF_ADDR*/    

extern uint8_t          gl_hpi_buffer[];                 /* Scratch buffer for recieving commands through I2C*/
bool                    gl_ucsi_read_pending = false;    /* Checks for command read pending */
volatile bool           gl_ucsi_cmd_available = false;   /* Checks for any command recevied from OPM */
ucsi_status_indicator_t gl_response = UCSI_STS_IGNORE;   /* Response to a command */

#if ENABLE_ST_SP_UCSI_CMD
/* Track who owns the I2C bus*/    
extern i2c_owner_t  gl_i2c_owner;
#endif /*ENABLE_ST_SP_UCSI_CMD*/

/* Check if the bus is already busy */
bool  gl_is_i2c_busy = false;

/**
 * @typedef ucsi_ppm_state_t
 * @brief PPM state enumeration.
 */
typedef enum {
    PPM_IDLE,                           /* PPM is waiting for a new command or notification */
    PPM_BUSY,                           /* PPM is waiting for a DPM command to complete */
    PPM_WAIT_FOR_ACK,                   /* PPM is waiting for the OPM to ACK the previous command */
    PPM_READ_PENDING,                   /* PPM is waiting for OPM to read the CCI register */
} ucsi_ppm_state_t;

static ucsi_ppm_state_t gl_ppm_state = PPM_IDLE;

/**
 * @static struct ucsi
 * @brief struct to hold UCSI status
 */
static struct {
    struct {
        uint16_t    notifications;      /* All notifications raised so far */
        uint16_t    event_filter;       /* Which notifications to not send to the OPM */
        bool        is_charging;        /* Whether the port is charging the system's battery */
    } ports[NO_OF_TYPEC_PORTS];
    dpm_pd_cmd_buf_t dpm_cmd_param;     /* Parameters associated with the active DPM command */
    uint16_t        event_mask;         /* Which notifications are enabled currently */
    union {                             /* Union to hold UCSI command parameters */
        struct {
            uint8_t start;              /* Start PDO index for GET_PDOS */
            uint8_t num;                /* Number of PDOs to ask in GET_PDOS */
        } get_pdos;
    } cmd_params;
    uint16_t         error_info;         /* UCSI error status */
    dpm_pd_cmd_t    dpm_cmd;            /* Active DPM command for each PD port. */
    bool            dpm_retry_pending;  /* Whether DPM command retry is pending. */
    uint8_t         dpm_retry_count;    /* Number of tries for this DPM command */
    uint8_t         dpm_port;           /* The port in which to execute the command */
    dpm_cmd_status_t dpm_cmd_status;    /* Status of the current DPM command */
    ucsi_cmd_t      active_cmd;         /* The command being processed now */
} gl_ucsi;

/**
 * @static struct gl_dev_capability
 * @brief data struct to device capability.
 */
static ucsi_capability_t gl_dev_capability = {
    .bmAttributes   = ALL_ATTRIBUTES_SUPPORTED,
    .bNumConnectors = NO_OF_TYPEC_PORTS,
    .bmOptionalFeatures = UCSI_OPTIONAL_FEATURES,
    .bNumAltModes = 1,
    .reserved = 0,
#if BATTERY_CHARGING_ENABLE    
    .bcdBCVersion = UCSI_BC_1_2_VERSION,
#else
    .bcdBCVersion = 0x0000,    
#endif /* BATTERY_CHARGING_ENABLE */    
#if CCG_PD_REV3_ENABLE    
    .bcdPDVersion = UCSI_USB_PD_3_0_VERSION,
#else
    .bcdPDVersion = UCSI_USB_PD_2_0_VERSION,    
#endif /*CCG_PD_REV3_ENABLE*/    
    .bcdUSBTypeCVersion = UCSI_USB_TYPE_C_1_3_VERSION
};

/* To hold connector capability*/
const static uint16_t connector_capability[NO_OF_TYPEC_PORTS] = {
    
#if (!CCG_UCSI_REV_1_1_ENABLE)  
    /* Port 0 */ CONN_CAP_DRP | CONN_CAP_USB_2_0_SUPP | CONN_CAP_USB_3_0_SUPP | CONN_CAP_ALT_MODE_SUPP | CONN_CAP_PWR_SRC_SUPP,
#if CCG_PD_DUALPORT_ENABLE    
    /* Port 1 */ CONN_CAP_DRP | CONN_CAP_USB_2_0_SUPP | CONN_CAP_USB_3_0_SUPP | CONN_CAP_ALT_MODE_SUPP | CONN_CAP_PWR_SRC_SUPP
#endif    

#else
    /* Port 0 */ CONN_CAP_DFP_ONLY | CONN_CAP_UFP_ONLY | CONN_CAP_DRP | CONN_CAP_USB_2_0_SUPP | CONN_CAP_USB_3_0_SUPP |\
                 CONN_CAP_ALT_MODE_SUPP | CONN_CAP_PWR_SRC_SUPP | CONN_CAP_PWR_SNK_SUPP | CONN_CAP_ANALOG_ACC_SUPP |\
                 CONN_CAP_DEBUG_ACC_SUPP | CONN_CAP_SWAP_TO_DFP | CONN_CAP_SWAP_TO_UFP | CONN_CAP_SWAP_TO_SRC |\
                 CONN_CAP_SWAP_TO_SNK,
#if CCG_PD_DUALPORT_ENABLE    
    /* Port 1 */ CONN_CAP_DFP_ONLY | CONN_CAP_UFP_ONLY | CONN_CAP_DRP | CONN_CAP_USB_2_0_SUPP | CONN_CAP_USB_3_0_SUPP |\
                 CONN_CAP_ALT_MODE_SUPP | CONN_CAP_PWR_SRC_SUPP | CONN_CAP_PWR_SNK_SUPP | CONN_CAP_ANALOG_ACC_SUPP |\
                 CONN_CAP_DEBUG_ACC_SUPP | CONN_CAP_SWAP_TO_DFP | CONN_CAP_SWAP_TO_UFP | CONN_CAP_SWAP_TO_SRC |\
                 CONN_CAP_SWAP_TO_SNK,
#endif      
    
#endif /* CCG_UCSI_REV_1_1_ENABLE */    
};

/* SVIDs received from the SOP* device */
static uint16_t gl_rx_svids[UCSI_MAX_SVIDS];

/* Modes received from the SOP* device */
static uint32_t gl_rx_modes[UCSI_MAX_SVIDS * 2];

/* SVID and mode index */
static uint8_t  gl_svid_idx = 0, gl_mode_idx = 0;

/* Variable to track the Silence mode of the UCSI interface. */
static uint8_t gl_ucsi_interface_silenced = false;

/* Mask used to  track the reception of the SET_CCOM command. */
static uint8_t gl_ccom_cmd_rcvd_mask = 0;
static uint8_t gl_typec_reset_cmd_completion = 0; 
static uint8_t gl_reset_pending = false;

/**
 * @brief Initialize the UCSI interface.
 * @return None
 */
void ucsi_init()
{
    if (gl_ucsi_interface_silenced == false)
    {
        memset((uint8_t*)&gl_ucsi, 0x00, sizeof(gl_ucsi));
        gl_ucsi.dpm_cmd = DPM_CMD_SEND_INVALID;
        gl_ucsi.dpm_retry_pending = false;
        gl_ucsi.dpm_retry_count = MAX_DPM_CMD_RETRY_COUNT;
        gl_ucsi.dpm_cmd_status = NO_CMD;
        gl_ucsi.event_mask = 0x00;    
        ucsi_reg_reset();
        gl_ucsi_cmd_available = false;
#if CCG_UCSI_ENABLE_DIFF_ADDR    
        ucsi_scb_idx = HPI_SCB_INDEX;
#endif /*CCG_UCSI_ENABLE_DIFF_ADDR*/    
        memset(&ucsi_regs, 0x00, sizeof(ucsi_regs));
        ucsi_regs.version = UCSI_VERSION;
        gl_ucsi.active_cmd = UCSI_CMD_RESERVED;
        gl_ppm_state = PPM_IDLE;

        gl_ccom_cmd_rcvd_mask = 0;
        gl_typec_reset_cmd_completion = 0; 
        gl_reset_pending = false;
        //    ucsi_set_status(UCSI_STS_RESET_COMPLETED);
    }
}

/**
 *  @brief To update UCSI notification.
 *  @param port port index
 *  @param UCSI notification 
 *  @return None
 */
void ucsi_notify(uint8_t port, uint16_t notification)
{
    if((gl_ucsi.event_mask & ~(gl_ucsi.ports[port].event_filter)) & notification)
        gl_ucsi.ports[port].notifications |= notification;
}
/**
 *  @brief Internal function to update the error status.
 *  @param port port index
 *  @param error_info Error information
 *  @return ucsi_status_indicator_t
 */
static ucsi_status_indicator_t ucsi_error(uint16_t error_info)
{
    gl_ucsi.error_info |= error_info;
    return UCSI_STS_ERROR;
}

/**
 *  @brief Internal function to update the UCSI Command Status.
 *  @param status UCSI Command Status
 *  @return None
 */
void ucsi_set_status(ucsi_status_indicator_t status)
{
    ucsi_regs.cci.indicators = status;
    gl_ucsi_read_pending = true;
    hpi_set_event (UCSI_READ_PENDING_EVENT);
}

/**
 *  @brief Handler for PD events reported from the stack.
 *  
 *  Internal function used to receive PD events from the stack and to update the UCSI registers. 
 *  
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that needs to be de-referenced
 * based on event type.
 *
 * @return None
 */
void ucsi_pd_event_handler(uint8_t port, app_evt_t evt, const void *data)
{
    dpm_status_t        *dpm_stat = (dpm_status_t *)dpm_get_info (port);
    alt_mode_app_evt_t  alt_mode_event = AM_NO_EVT;
    app_req_status_t    swap_status = REQ_REJECT;
    static bool         prev_charging_status = false;
    static bool         hard_reset_received = false;
    bool                system_is_charging = false;
    static bool         pr_swap_completed   = false;
    
    if ((dpm_stat->dpm_enabled == false) &&
        (evt != APP_EVT_VBUS_OVP_FAULT) && (evt != APP_EVT_VBUS_OCP_FAULT))
    {
        return;
    }
    
    switch (evt)
    {
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
            ucsi_notify(port, UCSI_EVT_PD_CONTRACT_CHANGED);
#if UCSI_RS5_HLK_FIX_CDT315271            
            ucsi_notify(port, UCSI_EVT_CAP_CHANGED);
#endif /*UCSI_RS5_HLK_FIX_CDT315271*/     
            if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
            {
                gl_ucsi.ports[port].is_charging = true;
                ucsi_notify(port, UCSI_EVT_BATT_CHARGING_STATUS_CHANGED);                
            }
            /* CDT295309 - PR SWAP notification should be sent only after the PS_RDY for contract completion happens. */           
            if (pr_swap_completed == true)
                ucsi_notify(port, UCSI_EVT_POWER_DIRECTION_CHANGED);
            pr_swap_completed = false;              
            break;
            
        case APP_EVT_CONNECT:
            pr_swap_completed = false;
            if(!hard_reset_received)
            {
                timer_start(port, UCSI_CONNECT_EVENT_TIMER, UCSI_CONNECT_EVENT_PERIOD, ucsi_tmr_cbk);
            }

            ucsi_notify(port, UCSI_EVT_CONNECT_CHANGED | UCSI_EVT_PARTNER_STATUS_CHANGED);
            ucsi_set_status_bit(UCSI_EVENT_PENDING);
            break;
            
        case APP_EVT_DISCONNECT:
            gl_ucsi.ports[port].is_charging = false;
            /* CDT295309 - Swap caused by SET PDR and UOR should be reset after a disconnect */
            dpm_update_swap_response(port, get_pd_port_config(port)->swap_response);            
            pr_swap_completed = false;
            hard_reset_received = false;            
            ucsi_notify(port, UCSI_EVT_CONNECT_CHANGED | UCSI_EVT_PARTNER_STATUS_CHANGED);
            ucsi_set_status_bit(UCSI_EVENT_PENDING);
            break;

        case APP_EVT_HARD_RESET_RCVD:
            hard_reset_received = true;
            break;
            
        case APP_EVT_HARD_RESET_COMPLETE:
            pr_swap_completed = false;
            if(hard_reset_received)
                ucsi_notify(port, UCSI_EVT_HARD_RESET_COMPLETE);
            hard_reset_received = false;
            break;

        case APP_EVT_ALT_MODE:
            alt_mode_event = ((alt_mode_evt_t *)data)->alt_mode_event.alt_mode_evt;
            switch(alt_mode_event)
            {
                case AM_EVT_ALT_MODE_SUPP:
                case AM_EVT_SUPP_ALT_MODE_CHNG:
                    ucsi_notify(port, UCSI_EVT_SUPPORTED_CAM_CHANGED);
#if CCG_PD_DUALPORT_ENABLE
#if UCSI_RS5_HLK_FIX    
                    ucsi_notify(TYPEC_PORT_1_IDX, UCSI_EVT_SUPPORTED_CAM_CHANGED);    
#endif /*UCSI_RS5_HLK_FIX*/                    
#endif /*CCG_PD_DUALPORT_ENABLE*/                    
                    break;
                case AM_EVT_ALT_MODE_ENTERED:
                case AM_EVT_ALT_MODE_EXITED:
                    ucsi_notify(port, UCSI_EVT_PARTNER_STATUS_CHANGED);
                    break;
                default:
                    break;
            }
            break;
                
        case APP_EVT_PR_SWAP_COMPLETE:
            swap_status = *((app_req_status_t *)data);
            if(swap_status == REQ_ACCEPT)
            {
                pr_swap_completed = true;
                ucsi_notify(port, UCSI_EVT_POWER_DIRECTION_CHANGED);
            }            
            break;

        case APP_EVT_DR_SWAP_COMPLETE:
            swap_status = *((app_req_status_t *)data);
            if(swap_status == REQ_ACCEPT)
            {
                ucsi_notify(port, UCSI_EVT_PARTNER_STATUS_CHANGED);
            }
            
        default:
            break;
    }  
    
    /* If either port is charging or both stop charging, notify the OPM */
#if CCG_PD_DUALPORT_ENABLE
    system_is_charging = gl_ucsi.ports[TYPEC_PORT_0_IDX].is_charging || gl_ucsi.ports[TYPEC_PORT_1_IDX].is_charging;
#else
    system_is_charging = gl_ucsi.ports[TYPEC_PORT_0_IDX].is_charging;
#endif
    if(system_is_charging != prev_charging_status)
        ucsi_notify(port, UCSI_EVT_EXT_SUPPLY_CHANGED);
    prev_charging_status = system_is_charging;
}

/**
 *  @brief Function to configure VDM for getting Alt Mode SVIDs & Modes
 *  @param cmd_sop Enum of the SOP (Start Of Frame) types.
 *  @param svid Standard ID / Vendor ID
 *  @param cmd DPM command
 *  @return None
 */
void ucsi_configure_send_vdm(sop_t cmd_sop, uint32_t svid, uint32_t cmd)
{
    gl_ucsi.dpm_cmd = DPM_CMD_SEND_VDM;
    gl_ucsi.dpm_cmd_param.cmd_sop = cmd_sop;
    gl_ucsi.dpm_cmd_param.timeout = PD_VDM_RESPONSE_TIMER_PERIOD;
    gl_ucsi.dpm_cmd_param.no_of_cmd_do = 1;
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.svid = svid;
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.vdm_type = VDM_TYPE_STRUCTURED;
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.st_ver = app_get_status(gl_ucsi.dpm_port)->vdm_version;
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.obj_pos = 0x00; 
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR; /*Initiator*/                    
    gl_ucsi.dpm_cmd_param.cmd_do->std_vdm_hdr.cmd = cmd; /*Discover Modes */    
}

/**
 *  @brief Function to handle UCSI Get Alternate Mode Command.
 *  @param port port index.
 *  @param pkt_ptr Struct to hold PD packet
 *  @return None
 */
static void ucsi_dpm_handle_alt_modes(uint8_t port, const pd_packet_t* pkt_ptr)
{
    ucsi_ctrl_details_t * ctrl_reg = &(ucsi_regs.control.details);
    uint8_t imsg_count = 0, i=0;
    sop_t rec_sop = SOP;

    /* Extract SOP*/
    if (ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_SOP_PRIME)
        rec_sop = SOP_PRIME;
    else if (ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_SOP_DPRIME)
        rec_sop = SOP_DPRIME;

    /* Verify that a valid packet was received */
    if(pkt_ptr->len >= 2)
    {   
        /* Discover SVID response */
        if(pkt_ptr->dat[0].std_vdm_hdr.cmd == VDM_DISCOVER_SVID)
        {
            memset(gl_rx_svids, 0x00, sizeof(gl_rx_svids));
            memset(gl_rx_modes, 0x00, sizeof(gl_rx_modes));

            /* Extract upto 4 SVIDs on this device */
            for(i = 0; i < UCSI_MAX_SVIDS; i++)
            {
                gl_rx_svids[i] = pkt_ptr->dat[1 + (i >> 1)].val >> (16 * (1 - (i & 0x01)));
                if(gl_rx_svids[i] == 0)
                    break;
            }

            gl_svid_idx = gl_mode_idx = 0;
            /* Setup Discover Modes for the first SVID on the list */
            ucsi_configure_send_vdm(rec_sop, gl_rx_svids[gl_svid_idx], VDM_DISCOVER_MODES);
            gl_ucsi.dpm_port = port;
            /* Queue the Disc Modes command to be sent. Reset retry state */
            gl_ucsi.dpm_cmd_status = CMD_QUEUED;
            gl_ucsi.dpm_retry_pending = false;
            gl_ucsi.dpm_retry_count = MAX_DPM_CMD_RETRY_COUNT; 
        }
        else
        {
            /*Extract modes of this SVID */
            gl_rx_modes[gl_mode_idx] = pkt_ptr->dat[1].val;
            gl_rx_modes[gl_mode_idx + 1] = (pkt_ptr->len >= 3) ? pkt_ptr->dat[2].val : 0;
            gl_mode_idx += 2;
            gl_svid_idx += 1;

            /* If there's another SVID for which modes weren't discovered, do it */
            if((gl_svid_idx < UCSI_MAX_SVIDS) && (gl_rx_svids[gl_svid_idx] != 0))
            {
                ucsi_configure_send_vdm(rec_sop, gl_rx_svids[gl_svid_idx], VDM_DISCOVER_MODES);
                gl_ucsi.dpm_port = port;
                /* Queue the Disc Modes command to be sent. Reset retry state */
                gl_ucsi.dpm_cmd_status = CMD_QUEUED;
                gl_ucsi.dpm_retry_pending = false;
                gl_ucsi.dpm_retry_count = MAX_DPM_CMD_RETRY_COUNT;                               
            }
            else
            {
                uint8_t offset = ctrl_reg->get_alternate_modes.alterate_mode_offset;
                uint8_t num_modes = ctrl_reg->get_alternate_modes.num_alternate_modes + 1;
                uint8_t mode_idx = 0, valid_mode = 0;

                imsg_count = 0;
                mode_idx = offset;

                for(mode_idx = 0; mode_idx < gl_mode_idx; mode_idx++)
                {
                    /* Skip invalid modes */
                    if(gl_rx_modes[mode_idx] == 0)
                        continue;
                    
                    /* A valid mode has been found. Skip until received offset */
                    if(valid_mode++ < offset)
                        continue;
                    
                    ucsi_regs.message_in[imsg_count++] = gl_rx_svids[mode_idx >> 1];
                    ucsi_regs.message_in[imsg_count++] = gl_rx_svids[mode_idx >> 1] >> 8;
                    ucsi_regs.message_in[imsg_count++] = gl_rx_modes[mode_idx];
                    ucsi_regs.message_in[imsg_count++] = gl_rx_modes[mode_idx] >> 8;
                    ucsi_regs.message_in[imsg_count++] = gl_rx_modes[mode_idx] >> 16;
                    ucsi_regs.message_in[imsg_count++] = gl_rx_modes[mode_idx] >> 24;
                    
                    num_modes -= 1;
                    
                    if((imsg_count >= UCSI_MAX_DATA_LENGTH) || (num_modes == 0))
                        break;
                }                
                
                ucsi_regs.cci.data_length = imsg_count;
                gl_ucsi.dpm_cmd = DPM_CMD_SEND_INVALID;
                gl_ucsi.dpm_cmd_status = CMD_SUCCESS;
            }
        }
    }
    else
    {
        ucsi_regs.cci.data_length = 0;
        gl_ucsi.dpm_cmd_status = CMD_SUCCESS;
    }
}

/**
 *  @brief UCSI DPM command callback
 *  @param port port index
 *  @param resp The response status to DPM commands
 *  @param pkt_ptr Struct to hold PD packet
 *  @return None
 */
static void ucsi_dpm_cmd_cb(uint8_t port, resp_status_t resp, const pd_packet_t* pkt_ptr)
{
    ucsi_ctrl_details_t * ctrl_reg = &(ucsi_regs.control.details);
//    const dpm_status_t *dpm_stat;    
    if (gl_ucsi.dpm_cmd == DPM_CMD_SEND_INVALID)
        return;

    if(resp < CMD_SENT)
    {
        gl_ucsi.dpm_cmd_status = CMD_ERROR;
        gl_ucsi.dpm_retry_pending = false;
        gl_ucsi.dpm_retry_count = 0;           
    }
    else if(resp == RES_RCVD)
    {
        switch(gl_ucsi.active_cmd)
        {
            case UCSI_CMD_GET_PDOS:
            {
  
                uint8_t pdo_requested =0;
                pdo_requested  = ctrl_reg->get_pdos.num_pdos+1;
                               
                if (pkt_ptr->len > ctrl_reg->get_pdos.pdo_offset)
                {
                    ucsi_regs.cci.data_length = UCSI_BOUND(sizeof(pd_do_t) * pdo_requested,
                            sizeof(pd_do_t) * (pkt_ptr->len - ctrl_reg->get_pdos.pdo_offset));
                    memcpy(ucsi_regs.message_in, (uint8_t *)&(pkt_ptr->dat[gl_ucsi.cmd_params.get_pdos.start]), ucsi_regs.cci.data_length);
                } 
                else
                {
                    ucsi_regs.cci.data_length = 0;
                }
                gl_ucsi.dpm_cmd_status = CMD_SUCCESS;
                break;                
            }
            case UCSI_CMD_GET_ALTERNATE_MODES:
            {
                ucsi_dpm_handle_alt_modes(port, pkt_ptr);
                break;
         
            }

            case UCSI_CMD_SET_UOR:
            case UCSI_CMD_SET_PDR:
                ucsi_regs.cci.data_length = 0;
                memset(ucsi_regs.message_in, 0x00, sizeof(ucsi_regs.message_in));
                if(pkt_ptr->msg == CTRL_MSG_ACCEPT)
                {
                    gl_ucsi.dpm_cmd_status = CMD_SUCCESS;
                }
                else
                {
#if CCG_UCSI_REV_1_1_ENABLE                   
                    gl_ucsi.error_info |= UCSI_ERR_PORT_PARTNER_REJECT_SWAP;
#endif /*CCG_UCSI_REV_1_1_ENABLE*/    
                    gl_ucsi.dpm_cmd_status = CMD_ERROR;                  
                    gl_ucsi.dpm_retry_pending = false;
                    gl_ucsi.dpm_retry_count = 0;
                }
                break;
           
            default:
                break;
        }
    }
}

/**
 *  @brief To get battery charging status.
 *  @param port port index
 *  @return Value of battery charging status.
 */
static uint8_t get_battery_charging_status(uint8_t port)
{
    const dpm_status_t *dpm_stat;
    uint8_t power_rate;
    dpm_stat = dpm_get_status(port);
    /*dpm_stat->contract.cur_pwr is in 10mA units*/    
    power_rate = ( (dpm_stat->contract.max_volt/1000) * (dpm_stat->contract.cur_pwr/100) );
    if(power_rate >= 45)
    {
        return BATT_STS_NOMINAL_CHARGING;
    }
    else if (power_rate >= 27) 
    {
        return BATT_STS_SLOW_CHARGING;
    }
    else if (power_rate >= 15)
    {
        return BATT_STS_VERY_SLOW_CHARGING;
    }
    else
    {
        return BATT_STS_NOT_CHARGING;
    }
    return BATT_STS_NOT_CHARGING;
}


/**
 *  @brief Function to get connector status of the connector
 *  @param port port index
 *  @param connector_status connector status of the connector
 *  @return ucsi_status_indicator_t return CCI status indicator
 */
static ucsi_status_indicator_t get_conn_status(uint8_t port, uint8_t * connector_status)
{
    const dpm_status_t *dpm_stat;
    volatile ucsi_connector_status_t *conn_status = (ucsi_connector_status_t *)connector_status;
    
    dpm_stat = dpm_get_info (port);
    
    memset(connector_status, 0x00, sizeof(ucsi_connector_status_t));  
    
    /* Push notifications to the OPM and clear it locally */
    conn_status->connector_status_change = gl_ucsi.ports[port].notifications;
    gl_ucsi.ports[port].notifications = 0;
    
    if (dpm_stat->attach == true)
    {
        conn_status->connect_status = 1;
        if(dpm_stat->contract_exist)
        {
            conn_status->power_op_mode = POM_PD;
            if (dpm_stat->cur_port_role == PRT_ROLE_SINK)
            {
                conn_status->battery_charging_status = BATT_STS_NOMINAL_CHARGING;
                conn_status->rdo = dpm_stat->snk_rdo.val;
            }
            else
            {
                conn_status->rdo = dpm_stat->src_rdo.val;
            }
        }
        else
        {
            if(dpm_stat->src_cur_level == SRC_CUR_LEVEL_DEF)
            {
                conn_status->power_op_mode = POM_CUR_LEVEL_DEF;
            }
            else if(dpm_stat->src_cur_level == SRC_CUR_LEVEL_1_5A)
            {
                conn_status->power_op_mode = POM_CUR_LEVEL_1_5A;
            }
            else if(dpm_stat->src_cur_level == SRC_CUR_LEVEL_3A)
            {
                conn_status->power_op_mode = POM_CUR_LEVEL_3A;
            }            
            if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
                conn_status->battery_charging_status = get_battery_charging_status(port);                 
        }  
        
        if(dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
            conn_status->power_direction = 1;
        
        /* Get partner type. */
        if (dpm_stat->attached_dev == DEV_DBG_ACC)
        {
            /* Debug acc */
            conn_status->connector_partner_type = CONN_PARTNER_DBG_ACC;
        }
        else if (dpm_stat->attached_dev == DEV_AUD_ACC)
        {
            /* Audio acc */
            conn_status->connector_partner_type = CONN_PARTNER_AUD_ACC;
        }
        else
        {
            /* DFP or UFP */
            if(dpm_stat->cur_port_type == PRT_TYPE_UFP)
                conn_status->connector_partner_type = CONN_PARTNER_DFP;
            else
                conn_status->connector_partner_type = CONN_PARTNER_UFP;        
        }        
        
        conn_status->connector_partner_flags = app_get_status(port)->alt_mode_entered ? 
                                                CONN_PARTNER_FLAG_ALT_MODE : CONN_PARTNER_FLAG_USB;
    }
    ucsi_clear_status_bit (UCSI_EVENT_PENDING);
    return UCSI_STS_CMD_COMPLETED;    
}

/**
 *  @brief Function to send PD commands
 *  @return dpm_cmd_status_t status of DPM command
 */
static void ucsi_send_dpm_cmd()
{
    /* We ran out of DPM command retries. Signal an error to the OPM */
    if(gl_ucsi.dpm_retry_pending && (gl_ucsi.dpm_retry_count == 0))
    {
        gl_ucsi.dpm_cmd_status = CMD_ERROR;
        gl_ucsi.dpm_retry_pending = false;           
        timer_stop(0, UCSI_DPM_RETRY_TIMER);
    }
    else
    {
        if(dpm_pd_command(gl_ucsi.dpm_port, gl_ucsi.dpm_cmd, 
                          &gl_ucsi.dpm_cmd_param, ucsi_dpm_cmd_cb) == CCG_STAT_SUCCESS)
        {
            gl_ucsi.dpm_retry_pending = false;
            gl_ucsi.dpm_retry_count = MAX_DPM_CMD_RETRY_COUNT;
            /* Mark command as being in progress */
            gl_ucsi.dpm_cmd_status = CMD_IN_PROGRESS;
            timer_stop(0, UCSI_DPM_RETRY_TIMER);
        }
        else /* DPM command couldn't be sent now. Try again later */
        {
            if(timer_is_running(0, UCSI_DPM_RETRY_TIMER) == false)
            {
                timer_start(0, UCSI_DPM_RETRY_TIMER, UCSI_DPM_RETRY_TIMER_PERIOD, NULL);
                gl_ucsi.dpm_retry_count--;
            }            
            gl_ucsi.dpm_retry_pending = true;
//            gl.ucsi.dpm_retry_count--;
            gl_ucsi.dpm_cmd_status = CMD_QUEUED;
        }
    }
}
/**
 *  @brief Function to handle UCSI Acknowledge Command
 *
 *  @return ucsi_status_indicator_t return CCI status indicator
 */
static ucsi_status_indicator_t ucsi_handle_ack_cmds (void)
{
   
    switch(ucsi_regs.control.command)
    {
        case UCSI_CMD_CANCEL:
            timer_stop(0,UCSI_DPM_RETRY_TIMER);
            gl_ucsi.active_cmd = UCSI_CMD_CANCEL;
            gl_ppm_state =PPM_IDLE;
            /*
            Once the control is transferred to PD/Type-C stack there is 
            no way to cancel it. So the current behavior of cancel command is 
            either it is completed before execution of cancel command or it is dropped.
            TBD : Need to address this in new event based architecture.
            Note : RS5 HLK doesn't implement test for this commands
            */
            return UCSI_STS_CMD_COMPLETED;

        case UCSI_CMD_ACK_CC_CI:
            gl_ucsi.active_cmd = UCSI_CMD_RESERVED;        
			if (ucsi_regs.control.details.ack_cc_ci.connector_change_ack)
			{
	            ucsi_regs.cci.connector_change = 0;
			}
            return UCSI_STS_ACK_CMD;
    }
    return UCSI_STS_IGNORE;
}

void ucsi_dpm_cb(uint8_t port, dpm_typec_cmd_resp_t resp)
{
    if (resp == DPM_RESP_SUCCESS)
    {
        if (gl_ccom_cmd_rcvd_mask & (1 << port))
        {
            gl_typec_reset_cmd_completion |= (1 << port);
        }
    }
    else
    {
        gl_ccom_cmd_rcvd_mask &= ~(1 << port);
    }
}

static void ucsi_cmd_ppm_reset_handler (
        void)
{
    uint8_t port = 0;

    ucsi_clear_status_bit(UCSI_EVENT_PENDING);
    memset((uint8_t*)&gl_ucsi, 0x00, sizeof(gl_ucsi));
    memset(&ucsi_regs, 0x00, sizeof(ucsi_regs));
    ucsi_regs.version = UCSI_VERSION;
    ucsi_clear_status_bit(UCSI_CMD_IN_PROGRESS);

    for(port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        /* Reset local notification list */
        gl_ucsi.ports[port].notifications = 0;

        /* Reset swap response */
        dpm_update_swap_response(port, get_pd_port_config(port)->swap_response);

        /* Reset source and sink caps */
        dpm_update_src_cap(port, get_pd_port_config(port)->src_pdo_cnt,
                (pd_do_t*)get_pd_port_config(port)->src_pdo_list);

        dpm_update_snk_cap(port, get_pd_port_config(port)->snk_pdo_cnt,
                (pd_do_t*)get_pd_port_config(port)->snk_pdo_list);

        if (gl_ccom_cmd_rcvd_mask & (1 << port))
        {
            if (dpm_typec_command(port, DPM_CMD_PORT_DISABLE, ucsi_dpm_cb) == CCG_STAT_SUCCESS)
            {
                gl_reset_pending = true;
            }
        }
        
        /* Reset Rp to original value from config table. */
       dpm_typec_command(port, (dpm_typec_cmd_t)(DPM_CMD_SET_RP_DFLT + get_pd_port_config(port)->current_level), NULL);
    }

    gl_ucsi_read_pending = false;
}

/**
 *  @brief Function to handle UCSI Reset Commands
 *
 *  @return ucsi_status_indicator_t return CCI status indicator
 */
static ucsi_status_indicator_t ucsi_handle_resets (void)
{
    ucsi_status_indicator_t ucsi_cmd_status = UCSI_STS_IGNORE;
    ucsi_ctrl_details_t * ctrl_reg = &(ucsi_regs.control.details);

    gl_ucsi.active_cmd = ucsi_regs.control.command;    

    switch(ucsi_regs.control.command)
    {
#if 0
        case UCSI_CMD_PPM_RESET:
            {
                ucsi_cmd_ppm_reset_handler ();
                ucsi_cmd_status = UCSI_STS_RESET_COMPLETED;
                break;
            }
#endif

        case UCSI_CMD_CONNECTOR_RESET:
            {
                const dpm_status_t *dpm_stat;
                uint8_t port = ctrl_reg->get_connector_capability.connector_number - 1;
                if (port >= NO_OF_TYPEC_PORTS)
                {
                    ucsi_cmd_status = ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
                    break;
                }

                dpm_stat = dpm_get_info (port);
                if (!(dpm_stat->contract_exist))
                {
                    /*TODO : UCSI - HLK Bug Failed if error bit is set.
                    ucsi_cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
                    */
                    ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;
                }                
                else
                {
#if (CCG_UCSI_REV_1_1_ENABLE)                
                    dpm_typec_command(port, DPM_CMD_TYPEC_ERR_RECOVERY, NULL);
#else
                    if (ctrl_reg->connector_reset.hard_reset)
                    {
                        gl_ucsi.dpm_cmd = DPM_CMD_SEND_HARD_RESET;
                    }
                    else
                    {
                        gl_ucsi.dpm_cmd = DPM_CMD_SEND_SOFT_RESET;
                    }

                    gl_ucsi.dpm_cmd_param.cmd_sop = SOP;
                    gl_ucsi.dpm_port = port;
                    gl_ucsi.ports[port].event_filter = UCSI_EVT_HARD_RESET_COMPLETE | UCSI_EVT_PD_CONTRACT_CHANGED | UCSI_EVT_EXT_SUPPLY_CHANGED;
                    ucsi_send_dpm_cmd();
#endif                 
                    ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;
                }
                break;
            }
    }

    return ucsi_cmd_status;
}

/**
 *  @brief Function to handle UCSI PPM Commands
 *
 *  @return ucsi_status_indicator_t return CCI status indicator
 */
static ucsi_status_indicator_t ucsi_handle_ppm_cmds (void)
{
    ucsi_ctrl_details_t * ctrl_reg = &(ucsi_regs.control.details);
    ucsi_status_indicator_t ucsi_cmd_status = UCSI_STS_IGNORE;
#if UCSI_HP_DRIVER      
    uint16_t prev_event_mask = 0;
#endif /* UCSI_HP_DRIVER */  

    gl_ucsi.active_cmd = ucsi_regs.control.command;

    switch(ucsi_regs.control.command)
    {
        case UCSI_CMD_SET_NOTIFICATION_ENABLE:
            {
#if UCSI_HP_DRIVER            
                prev_event_mask = ucsi.event_mask;
#endif /* UCSI_HP_DRIVER */    
                gl_ucsi.event_mask = ctrl_reg->set_notification_enable.notifications;

#if UCSI_HP_DRIVER        
                /* If the OPM enabled the "Connect Changed" event, trigger a
                 * notification so that the OPM can read the latest port state.
                 * This is a work-around for the UCSI driver not doing this on
                 * bootup */
                if((gl_ucsi.event_mask & UCSI_EVT_CONNECT_CHANGED)
                        && !(prev_event_mask & UCSI_EVT_CONNECT_CHANGED))
                {
                    ucsi_notify(TYPEC_PORT_0_IDX, UCSI_EVT_CONNECT_CHANGED);
#if CCG_PD_DUALPORT_ENABLE
                    ucsi_notify(TYPEC_PORT_1_IDX, UCSI_EVT_CONNECT_CHANGED);
#endif /* CCG_PD_DUALPORT_ENABLE */
                }
#endif /* UCSI_HP_DRIVER */

                /* If the event mask was updated, clear any pending notifications
                 * associated with the event */
                gl_ucsi.ports[TYPEC_PORT_0_IDX].notifications &= gl_ucsi.event_mask;
#if CCG_PD_DUALPORT_ENABLE
                gl_ucsi.ports[TYPEC_PORT_1_IDX].notifications &= gl_ucsi.event_mask;
#endif
                /* If the "Command Completed" notification was cleared, drop the response */
                if(gl_ucsi.event_mask & UCSI_EVT_CMD_COMPLETED)
                {
                    ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;
                }
                else
                {
                    gl_ucsi.active_cmd = UCSI_CMD_RESERVED;
                    ucsi_regs.cci.connector_change = 0;
                    ucsi_clear_status_bit(UCSI_EVENT_PENDING);
                }

                break;
            }
        
        case UCSI_CMD_GET_CAPABILITY:
            {
                uint8_t temp1 = 0;
#if CCG_PD_DUALPORT_ENABLE
                uint8_t temp2 = 0;
#endif /* CCG_PD_DUALPORT_ENABLE */

                ucsi_regs.cci.data_length = GET_CAPABILITY_STATUS_DATA_LENGTH;

                temp1 = get_alt_mode_numb(TYPEC_PORT_0_IDX);
#if CCG_PD_DUALPORT_ENABLE
                temp2 = get_alt_mode_numb(TYPEC_PORT_1_IDX);
                if (temp2 > temp1)
                {
                    temp1 = temp2;
                }
#endif /* CCG_PD_DUALPORT_ENABLE */

                gl_dev_capability.bNumAltModes = temp1;
                gl_dev_capability.bNumConnectors = NO_OF_TYPEC_PORTS;

                if ( 
                        (gl_dpm_port_type[TYPEC_PORT_0_IDX] != PRT_TYPE_DFP)
#if CCG_PD_DUALPORT_ENABLE 
                        || (gl_dpm_port_type[TYPEC_PORT_1_IDX] != PRT_TYPE_DFP)
#endif /* CCG_PD_DUALPORT_ENABLE */
                   )
                {
                    gl_dev_capability.bmAttributes |= PD_SUPPLY_SUPPORTED;
                }                

#if APP_AC_BARREL_CONNECTOR_SUPPPRTED
                gl_dev_capability.bmAttributes |= AC_SUPPLY_SUPPORTED;
#endif
                memcpy(ucsi_regs.message_in, &gl_dev_capability, sizeof(ucsi_capability_t));
                ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;
                break;
            }
         
        case UCSI_CMD_GET_CONNECTOR_CAPABILITY:
            {
                uint8_t port = ctrl_reg->get_connector_capability.connector_number - 1;
                if (port >= NO_OF_TYPEC_PORTS)
                {
                    ucsi_cmd_status = ucsi_error(UCSI_STS_CMD_COMPLETED | UCSI_ERR_UNKNOWN_CONNECTOR);
                }
                else
                {
                    ucsi_regs.cci.data_length = GET_CONNECTOR_CAPABILITY_DATA_LENGTH;
                    memcpy(ucsi_regs.message_in, &connector_capability[port], 2);
                    ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;
                }

                break;
            }

        case UCSI_CMD_GET_ERROR_STATUS:
            {
                memcpy(ucsi_regs.message_in, &gl_ucsi.error_info, 2);
                gl_ucsi.error_info = 0x00;
                ucsi_regs.cci.data_length = GET_ERROR_STATUS_DATA_LENGTH;
                ucsi_cmd_status = UCSI_STS_CMD_COMPLETED;

                break;
            }
    }

    return ucsi_cmd_status;
}

/**
 *  @brief Function to handle UCSI Port Commands
 *
 *  @return ucsi_status_indicator_t return CCI status indicator
 */
static ucsi_status_indicator_t ucsi_handle_port_cmd (void)
{
    ucsi_status_indicator_t cmd_status = UCSI_STS_CMD_COMPLETED;
    
    ucsi_ctrl_details_t * ctrl_reg = &(ucsi_regs.control.details);
    uint8_t port; // = (ctrl_reg->generic.connector_number) - 1;    

#if UCSI_SET_PWR_LEVEL_ENABLE
    uint8_t mask = 1;
    pd_do_t* pdos;    
#endif /*UCSI_SET_PWR_LEVEL_ENABLE*/
    
    
//    uint8_t cur_level;    
    const dpm_status_t *dpm_stat; 
    
    
#if UCSI_ALT_MODE_ENABLED
    uint8_t alt_mode_idx;
    uint8_t imsg_count;
    alt_mode_info_t    *am_info_p  = NULL;  

#endif /*UCSI_ALT_MODE_ENABLED*/    
    
    port_role_t port_role_map[5] = {0, PRT_ROLE_SOURCE, PRT_ROLE_SINK, 0, PRT_DUAL };
    
    gl_ucsi.active_cmd = ucsi_regs.control.command;    

       
    switch(ucsi_regs.control.command)
    {              
        case UCSI_CMD_RESERVED:
            cmd_status = UCSI_STS_NOT_SUPPORTED;
            break;            

        case UCSI_CMD_SET_CCOM:
        
            port = (ctrl_reg->set_uom.connector_number) - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR); 
            dpm_stat = dpm_get_info (port);

            if(dpm_stat->attach)
                cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
            else
            {
                gl_ccom_cmd_rcvd_mask |= (1 << port);
                port_role_t req_port_role = port_role_map[ctrl_reg->set_uom.usb_operating_mode];
                dpm_stop(port);
                dpm_update_port_config(port, req_port_role, PRT_ROLE_SOURCE, 1, 1);
                dpm_start(port); 
            }                        
            break;
        
        case UCSI_CMD_SET_UOR:
            
            port = (ctrl_reg->set_uor.connector_number) - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR); 
            dpm_stat = dpm_get_info (port);

            if(!(dpm_stat->contract_exist))
            {
                cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
            }
            else
            {
                port_type_t req_port_role;
                if (ctrl_reg->set_uor.operate_as_dfp)
                {
                    req_port_role = PRT_TYPE_DFP;
                }
                else if (ctrl_reg->set_uor.operate_as_ufp)
                {
                    req_port_role = PRT_TYPE_UFP;
                }
                else if (ctrl_reg->set_uor.accept_dr_swap)
                {
                    req_port_role = PRT_TYPE_DRP;
                }
                else
                {
                    cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
                    break;
                }
               
                if(req_port_role == PRT_TYPE_DRP)
                {
                    /* Accept DR SWAP. */
                    dpm_get_status(port)->swap_response &= ~0x03;
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }
                else if(dpm_stat->cur_port_type != req_port_role)
                {
                    if(app_get_status(port)->alt_mode_entered)
                    {
                        cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
                    }
                    else
                    {
                        gl_ucsi.dpm_cmd = DPM_CMD_SEND_DR_SWAP;
                        gl_ucsi.dpm_cmd_param.cmd_sop = SOP;
                        gl_ucsi.dpm_port = port;
                        gl_ucsi.ports[port].event_filter = UCSI_EVT_PARTNER_STATUS_CHANGED;
                        cmd_status = UCSI_STS_BUSY;
                    }
                }
                else
                {
#if 0                    
                    /* Reject DR SWAP. */
                    dpm_get_status(port)->swap_response &= ~0x03;
                    dpm_get_status(port)->swap_response |= 0x01;
#endif                    
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }
            }
            break;
            
#if (!CCG_UCSI_REV_1_1_ENABLE)
        case UCSI_CMD_SET_PDM:                       
            if(dpm_stat->attach)
                cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
            else
            {
                port_role_t req_port_role = port_role_map[ctrl_reg->set_pdm.power_direction_mode];
                dpm_stop(port);
                dpm_update_port_config(port, req_port_role, PRT_ROLE_SOURCE, 1, 1);
                dpm_start(port); 
            }            
            break;
#endif /* CCG_UCSI_REV_1_1_ENABLE*/


#if CCG_UCSI_REV_1_1_ENABLE
#if UCSI_SET_PWR_LEVEL_ENABLE
        case UCSI_CMD_SET_POWER_LEVEL:
            {
                port = ctrl_reg->set_power_level.connector_number - 1;
                if(port >= NO_OF_TYPEC_PORTS)
                {
                    return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
                }
                /* Call the solution layer function */
                pdos = ucsi_change_pdo_power(port, ctrl_reg->set_power_level.usb_pd_max_pwr, 
                        ctrl_reg->set_power_level.src_snk, &mask);

                if(pdos == NULL)
                {
                    cmd_status = ucsi_error(UCSI_STS_CMD_COMPLETED | UCSI_ERR_INVALID_PARAMS); 
                    break;
                }

                dpm_stat = dpm_get_info (port);
                /* Update the DPM */
                if(ctrl_reg->set_power_level.src_snk)
                {
                    dpm_update_src_cap(port, dpm_stat->src_pdo_count, pdos);
                    dpm_update_src_cap_mask(port, mask);
                    if(dpm_stat->contract_exist)
                    {
                        dpm_pd_command(port, DPM_CMD_SRC_CAP_CHNG, NULL, NULL);
                    }
                }
                else
                {
                    dpm_update_snk_cap(port, dpm_stat->snk_pdo_count, pdos);
                    dpm_update_snk_cap_mask(port, mask);
                    if(dpm_stat->contract_exist)
                    {
                        dpm_pd_command(port, DPM_CMD_SNK_CAP_CHNG, NULL, NULL);
                    }
                }

                /* Try to change Rp as well */
                switch(ctrl_reg->set_power_level.usb_typec_current)
                {
                    case UCSI_SET_RP_3A:
                    case UCSI_SET_RP_PPM_DEFAULT:
                        dpm_typec_command(port, DPM_CMD_SET_RP_3A, NULL);
                        break;

                    case UCSI_SET_RP_1_5A:
                        dpm_typec_command(port, DPM_CMD_SET_RP_1_5A, NULL);
                        break;

                    case UCSI_SET_RP_DEF:
                        dpm_typec_command(port, DPM_CMD_SET_RP_DFLT, NULL);
                        break;
                }

                break;
            }

#endif /* UCSI_SET_PWR_LEVEL_ENABLE */    
#endif /*CCG_UCSI_REV_1_1_ENABLE*/    

        case UCSI_CMD_SET_PDR:
            
#if 1            
            port = ctrl_reg->set_pdr.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
            dpm_stat = dpm_get_info (port);    
            if(!(dpm_stat->contract_exist))
                cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
            else
            {
#if 0                
                port_role_t req_port_role;
                if (ctrl_reg->set_pdr.operate_as_src)
                {
                    req_port_role = PRT_ROLE_SOURCE;
                }
                else if (ctrl_reg->set_pdr.operate_as_snk)
                {
                    req_port_role = PRT_ROLE_SINK;
                }
                else if (ctrl_reg->set_pdr.accept_pr_swap)
                {
                    req_port_role = PRT_DUAL;
                }
                else
                {
                    cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
                    break;
                }

                if (req_port_role == PRT_DUAL)
                {
                    /* Accept PR SWAP. */
                    dpm_get_status(port)->swap_response &= ~0x0C;
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }
                else if(dpm_stat->cur_port_role != req_port_role)
                {
                    gl_ucsi.dpm_cmd = DPM_CMD_SEND_PR_SWAP;
                    gl_ucsi.dpm_cmd_param.cmd_sop = SOP;
                    gl_ucsi.ports[port].event_filter = UCSI_EVT_PD_CONTRACT_CHANGED | UCSI_EVT_EXT_SUPPLY_CHANGED | 
                        UCSI_EVT_POM_CHANGED | UCSI_EVT_PARTNER_STATUS_CHANGED;
                    cmd_status = UCSI_STS_BUSY;
                }
                else
                {
                    /* Reject PR SWAP. */
                    dpm_get_status(port)->swap_response &= ~0x0C;
                    dpm_get_status(port)->swap_response |= 0x04;
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }
#endif
                if(ctrl_reg->set_pdr.accept_pr_swap)
                {
                        dpm_update_swap_response(port, 
                                (dpm_stat->swap_response & ~DPM_PR_SWAP_RESP_MASK) | (APP_RESP_ACCEPT << DPM_PR_SWAP_RESP_POS));
                }
                else
                {
                        dpm_update_swap_response(port,
                            (dpm_stat->swap_response & ~DPM_PR_SWAP_RESP_MASK) | (APP_RESP_REJECT << DPM_PR_SWAP_RESP_POS));
                }
                
                if((ctrl_reg->set_pdr.operate_as_snk && (dpm_stat->cur_port_role == PRT_ROLE_SOURCE))
                    || (ctrl_reg->set_pdr.operate_as_src && (dpm_stat->cur_port_role == PRT_ROLE_SINK)))
                {
                    gl_ucsi.dpm_cmd = DPM_CMD_SEND_PR_SWAP;
                    gl_ucsi.dpm_cmd_param.cmd_sop = SOP;
                    gl_ucsi.dpm_port = port;
                    gl_ucsi.ports[port].event_filter = UCSI_EVT_PD_CONTRACT_CHANGED | UCSI_EVT_EXT_SUPPLY_CHANGED | 
                        UCSI_EVT_POM_CHANGED | UCSI_EVT_PARTNER_STATUS_CHANGED;
                    cmd_status = UCSI_STS_BUSY;
                }
                
            }
#endif

            break;
#if UCSI_ALT_MODE_ENABLED            
        case UCSI_CMD_GET_ALTERNATE_MODES:
            {
                uint8_t req_am_offset = ctrl_reg->get_alternate_modes.alterate_mode_offset; 
                uint8_t req_am_num = ctrl_reg->get_alternate_modes.num_alternate_modes;
                uint8_t num_am_supp;
                    
                port = ctrl_reg->get_alternate_modes.connector_number - 1;            
                if(port >= NO_OF_TYPEC_PORTS)
                    return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);                 
                dpm_stat = dpm_get_info (port);
                num_am_supp = get_alt_mode_numb(port);

                if(port >= NO_OF_TYPEC_PORTS)
                    return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
                    
                if(ctrl_reg->get_alternate_modes.recipient > 3)
                    return ucsi_error(UCSI_ERR_INVALID_PARAMS);

                if(ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_CONNECTOR)
                {
                    if((req_am_num < 2) && (num_am_supp >= 1) && (req_am_offset < num_am_supp))
                    {
                        imsg_count = 0;
                        req_am_num += 1;
                        for(alt_mode_idx = req_am_offset; (req_am_num) && (alt_mode_idx < num_am_supp); alt_mode_idx++)
                        {
                            ucsi_regs.message_in[imsg_count++] = get_tbl_info(port, alt_mode_idx).svid;
                            ucsi_regs.message_in[imsg_count++] = get_tbl_info(port, alt_mode_idx).svid >> 8;
                            ucsi_regs.message_in[imsg_count++] = get_tbl_mode_info(port, alt_mode_idx);
                            ucsi_regs.message_in[imsg_count++] = get_tbl_mode_info(port, alt_mode_idx) >> 8;
                            ucsi_regs.message_in[imsg_count++] = get_tbl_mode_info(port, alt_mode_idx) >> 16;
                            ucsi_regs.message_in[imsg_count++] = get_tbl_mode_info(port, alt_mode_idx) >> 24;
                            req_am_num -= 1;
                        }

                        ucsi_regs.cci.data_length = imsg_count;
                        cmd_status = UCSI_STS_CMD_COMPLETED;
                    }
                    else
                    {
                        ucsi_regs.cci.data_length = 0;
                        cmd_status = UCSI_STS_CMD_COMPLETED;
                    }
                }
                else if (ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_SOP)
                {
                    if( (num_am_supp >= 1) && (req_am_offset <= (num_am_supp-1)) && (modal_op_support == true))
                    {
                        ucsi_configure_send_vdm(SOP, STD_SVID, VDM_DISCOVER_SVID);
                        gl_ucsi.dpm_port = port;
                        cmd_status = UCSI_STS_BUSY;
                    }
                    else
                    {
                        ucsi_regs.cci.data_length = 0;
                        cmd_status = UCSI_STS_CMD_COMPLETED;
                    }
                }
                else if (ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_SOP_PRIME)
                {
                    if((num_am_supp >= 1) && (req_am_offset <= (num_am_supp-1)) &&
                            (dpm_stat->emca_present != false) && (dpm_stat->cbl_mode_en != false))
                    {                    
                        ucsi_configure_send_vdm(SOP_PRIME, STD_SVID, VDM_DISCOVER_SVID);
                        gl_ucsi.dpm_port = port;
                        cmd_status = UCSI_STS_BUSY;
                    }
                    else
                    {
                        ucsi_regs.cci.data_length = 0;
                        cmd_status = UCSI_STS_CMD_COMPLETED;
                    }                    
                }
                else if (ctrl_reg->get_alternate_modes.recipient == ALT_MODES_RECIPIENT_SOP_DPRIME)
                {                                
                    if( (num_am_supp >= 1) && (req_am_offset <= (num_am_supp-1)) &&
                            (dpm_stat->emca_present != false) && (dpm_stat->cbl_mode_en != false))                    
                    {                    
                        ucsi_configure_send_vdm(SOP_DPRIME, STD_SVID, VDM_DISCOVER_SVID);
                        gl_ucsi.dpm_port = port;
                        cmd_status = UCSI_STS_BUSY; 
                    }
                    else
                    {
                        ucsi_regs.cci.data_length = 0;
                        cmd_status = UCSI_STS_CMD_COMPLETED;
                    }                    
                }
                else
                {
                    ucsi_regs.cci.data_length = 0;
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }                               
            }
         
            break;
            
        case UCSI_CMD_GET_CAM_SUPPORTED:
            port = ctrl_reg->get_cam_supported.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
            ucsi_regs.cci.data_length = (get_alt_mode_numb(port) >> 3) + 1;
            ucsi_regs.message_in[0] = get_supp_alt_modes(port);
            ucsi_regs.message_in[1] = get_supp_alt_modes(port) >> 8;
            ucsi_regs.message_in[2] = get_supp_alt_modes(port) >> 16;
            ucsi_regs.message_in[3] = get_supp_alt_modes(port) >> 24;
            cmd_status = UCSI_STS_CMD_COMPLETED;                          
            break;
            
        case UCSI_CMD_GET_CURRENT_CAM:
            port = ctrl_reg->get_current_cam.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
            if((get_alt_mode_numb(port) != 0) && app_get_status(port)->alt_mode_entered)
            {
                for(alt_mode_idx = 0; alt_mode_idx < get_alt_mode_numb(port); alt_mode_idx++)
                    if(get_tbl_info(port, alt_mode_idx).alt_mode_id == get_alt_mode_id(port))
                    {
                        ucsi_regs.message_in[0] = alt_mode_idx;
                        break;
                    }
                
                ucsi_regs.cci.data_length = 0x01;
                cmd_status = UCSI_STS_CMD_COMPLETED;    
            }
            else
            {
                
#if CCG_UCSI_REV_1_1_ENABLE
                ucsi_regs.message_in[0] = 0xFF;
                ucsi_regs.cci.data_length = 0x01;
                cmd_status = UCSI_STS_CMD_COMPLETED;                 
#else    
                ucsi_regs.cci.data_length = 0;
                ucsi_error(UCSI_ERR_INVALID_PARAMS);
                cmd_status = UCSI_STS_ERROR;    
#endif /* CCG_UCSI_REV_1_1_ENABLE*/                

            }            
            break;                     
                
        case UCSI_CMD_SET_NEW_CAM:
            port = ctrl_reg->set_new_cam.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);  
            dpm_stat = dpm_get_info (port);
            /* Cannot execute if port is not enabled. */                    
            if(!(dpm_stat->contract_exist))
            {
                cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);
            }
            else                
            {                
                for(alt_mode_idx = 0; alt_mode_idx < get_alt_mode_numb(port) ; alt_mode_idx++)
                {                   
                    if(alt_mode_idx == ctrl_reg->set_new_cam.new_cam)
                    {
                        if(get_mode_info(port,alt_mode_idx)->vdm_header.std_vdm_hdr.svid == (uint32_t) ctrl_reg->set_new_cam.alt_mode_specific)
                        {
                            am_info_p = get_mode_info(port,0);

                                                
                            if(ctrl_reg->set_new_cam.enter_or_exit == 0x01)
                            {
                                if (am_info_p->cbk != NULL)
                                {
                                    set_alt_mode_state(port, alt_mode_idx);
                                    /* Inits alt mode */
                                    am_info_p->mode_state = ALT_MODE_STATE_INIT;
                                    am_info_p->cbk(port);    
                                }
                            }
                            else
                            {
                                am_info_p->mode_state = ALT_MODE_STATE_EXIT;
                                if (am_info_p->cbk != NULL)
                                {
                                    am_info_p->cbk(port);
                                }                
                            }                                
                            
                        }
                    }
                    cmd_status = UCSI_STS_CMD_COMPLETED;
                }                                    
            }
            break;
#endif /*UCSI_ALT_MODE_ENABLED*/            
        
        case UCSI_CMD_GET_PDOS:
            port = ctrl_reg->get_pdos.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
            dpm_stat = dpm_get_info (port);
            if(ctrl_reg->get_pdos.partner_pdo)
            {
               if(dpm_stat->contract_exist)
               {       
                    gl_ucsi.cmd_params.get_pdos.start = ctrl_reg->get_pdos.pdo_offset;
                    gl_ucsi.cmd_params.get_pdos.num = ctrl_reg->get_pdos.num_pdos + 1;
                    if(dpm_stat->contract_exist)
                    {
                        gl_ucsi.dpm_cmd_param.cmd_sop = SOP;
                        gl_ucsi.dpm_port = port;                        
                        if(ctrl_reg->get_pdos.is_source_pdo)
                            gl_ucsi.dpm_cmd = DPM_CMD_GET_SRC_CAP;
                        else
                            gl_ucsi.dpm_cmd = DPM_CMD_GET_SNK_CAP;
                        gl_ucsi.ports[port].event_filter = UCSI_EVT_PD_CONTRACT_CHANGED | UCSI_EVT_EXT_SUPPLY_CHANGED;
                        cmd_status = UCSI_STS_BUSY;
                        gl_ucsi.cmd_params.get_pdos.start = ctrl_reg->get_pdos.pdo_offset;
                        gl_ucsi.cmd_params.get_pdos.num = ctrl_reg->get_pdos.num_pdos + 1;
                    }
                    else
                        ucsi_regs.cci.data_length = 0;
                }
                else
                {
                    cmd_status = ucsi_error(UCSI_ERR_PD_COMM_ERROR);   
                }
            }
            else /* Connector's PDOs */
            {
                if(ctrl_reg->get_pdos.is_source_pdo)
                {
                    switch(ctrl_reg->get_pdos.src_cap_type)
                    {
                        case UCSI_CUR_SUPPORTED_SRC_CAPS:
                        {      
                            uint8_t i = 0, pdo_num = 0, valid_pdo_offset = 0;
                            for(i = 0; i < dpm_stat->src_pdo_count; i++)
                            {
                                if ((dpm_stat->src_pdo_mask & (0x01 << i)) != 0)
                                {                                   
                                    if(valid_pdo_offset >= ctrl_reg->get_pdos.pdo_offset)
                                        memcpy(&ucsi_regs.message_in[(pdo_num++) * 4], &(dpm_stat->src_pdo[i]), sizeof(pd_do_t));
                                    valid_pdo_offset++;
                                    if (pdo_num >= (ctrl_reg->get_pdos.num_pdos + 1))
                                        break;
                                }
                            }
                            ucsi_regs.cci.data_length = sizeof(pd_do_t) * pdo_num;
                            break;                                                                                    
                        }

                        case UCSI_ADVERTISED_SRC_CAPS:
                        {                      
                            /* Get the number of PDOs requested by EC. */                            
                            uint8_t pdo_requested = ctrl_reg->get_pdos.num_pdos + 1;
                            uint8_t pdo_count = 0;
                              
                                
                            /* Ensure PDO offset doesn't cross the number of PDOs. */
                            if (dpm_stat->cur_src_pdo_count > ctrl_reg->get_pdos.pdo_offset)
                            {
                                /* Number of PDOs that can be sent in response. */
                                pdo_count = dpm_stat->cur_src_pdo_count - ctrl_reg->get_pdos.pdo_offset;
                                pdo_count = GET_MIN(pdo_count, pdo_requested);
                                ucsi_regs.cci.data_length = pdo_count << 2;
                                memcpy(ucsi_regs.message_in, &(dpm_stat->cur_src_pdo[ctrl_reg->get_pdos.pdo_offset]),
                                        ucsi_regs.cci.data_length);
                            }
                            else
                            {
                                ucsi_regs.cci.data_length = 0;
                            }
                            break;
                        }
                        
                        case UCSI_MAX_SUPPORTED_SRC_CAPS:
                        {
                            /* Get the number of PDOs requested by EC. */                            
                            uint8_t pdo_requested = ctrl_reg->get_pdos.num_pdos + 1; 
                            if (dpm_stat->src_pdo_count > ctrl_reg->get_pdos.pdo_offset)
                            {                            
                                ucsi_regs.cci.data_length = UCSI_BOUND(sizeof(pd_do_t) * pdo_requested,
                                                            sizeof(pd_do_t) * (dpm_stat->src_pdo_count - ctrl_reg->get_pdos.pdo_offset));
                                memcpy(ucsi_regs.message_in, &(dpm_stat->src_pdo[ctrl_reg->get_pdos.pdo_offset]),
                                        ucsi_regs.cci.data_length);
                            }
                            else
                            {
                                ucsi_regs.cci.data_length = 0;
                            }
                            break;
                        }
                        default:
                            cmd_status = ucsi_error(UCSI_ERR_UNRECOGNIZED_CMD);
                            ucsi_regs.cci.data_length = 0;
                            break;
                    }
                }
                else
                {
                    /* Get the number of PDOs requested by EC. */                            
                    uint8_t pdo_requested = ctrl_reg->get_pdos.num_pdos + 1;                    
                    if ((dpm_stat->snk_pdo_count > ctrl_reg->get_pdos.pdo_offset) && (pdo_requested != 0))
                    {                    
                        ucsi_regs.cci.data_length = UCSI_BOUND(sizeof(pd_do_t) * pdo_requested,
                                                    sizeof(pd_do_t) * (dpm_stat->snk_pdo_count - ctrl_reg->get_pdos.pdo_offset));
                        memcpy(ucsi_regs.message_in, &(dpm_stat->snk_pdo[ctrl_reg->get_pdos.pdo_offset]),
                                            ucsi_regs.cci.data_length);
                    }
                }
            }
            break;
            
        case UCSI_CMD_GET_CABLE_PROPERTY:
            {
                port = ctrl_reg->get_cable_property.connector_number - 1;            
                if(port >= NO_OF_TYPEC_PORTS)
                    return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);  
                    
                ucsi_cable_property_t * cable_prop = (ucsi_cable_property_t *)&(ucsi_regs.message_in[0]);
                const pd_do_t * cbl_vdo;
                /* Current Map 3A = 50*60, 5A = 50*100*/
                uint8_t current_map[4] = {0, CABLE_CURR_3A, CABLE_CURR_5A, 0};
                /* Speed Mapping 
                 * bmSpeedSupported = M * 10^(3*e)
                 * M = Mantissa, e=Exponent
                 * 
                 * 00 - USB2.0 only, 480 Mbps     = 0000 0111 1000 00 10
                 * 01 - USB 3.1 Gen 1, 5Gbps      = 0000 0000 0001 00 11
                 * 10 - USB 3.1 Gen 1 & 2, 10Gbps = 0000 0000 0010 10 11
                */
                uint16_t speed_map[3] = { CABLE_SPEED_480MBPS, CABLE_SPEED_5GBPS, CABLE_SPEED_10GBPS };
                
                dpm_stat = dpm_get_info (port);
                cbl_vdo = &(dpm_stat->cbl_vdo);

                cable_prop->bmSpeedSupported = speed_map[GET_MIN(cbl_vdo->std_cbl_vdo.usb_ss_sup, 0x02)];
                cable_prop->bCurrentCapability = current_map[cbl_vdo->std_cbl_vdo.vbus_cur];
                cable_prop->VBUSInCable = cbl_vdo->std_cbl_vdo.vbus_thru_cbl;
                cable_prop->CableType = cbl_vdo->std_cbl_vdo.cbl_term >> 1;
                if(cbl_vdo->val & CABLE_VDO_DIRECTION)
                    cable_prop->Directionality = 1;
                cable_prop->PlugEndType = cbl_vdo->std_cbl_vdo.typec_abc;
                cable_prop->ModeSupport = 0;
                cable_prop->Latency = cbl_vdo->std_cbl_vdo.cbl_latency;
                ucsi_regs.cci.data_length = GET_CABLE_PROPERTY_DATA_LENGTH;
            }
            break;
            
        case UCSI_CMD_GET_CONNECTOR_STATUS:
            port = ctrl_reg->get_connector_status.connector_number - 1;
            if(port >= NO_OF_TYPEC_PORTS)
                return ucsi_error(UCSI_ERR_UNKNOWN_CONNECTOR);
            cmd_status = get_conn_status(port, ucsi_regs.message_in);
            if(cmd_status == UCSI_STS_CMD_COMPLETED)
                ucsi_regs.cci.data_length = GET_CONNECTOR_STATUS_DATA_LENGTH;
            break;

        default:
            cmd_status = UCSI_STS_IGNORE;
            break;
    }
    
    /* If this command needs action on the PD port, attempt it now */
    if((cmd_status == UCSI_STS_BUSY) && (gl_ucsi.dpm_cmd != DPM_CMD_SEND_INVALID))
    {
        ucsi_send_dpm_cmd();
    }
    
    return cmd_status;
}

/**
 * App layer function to check if CCGx can be put to sleep
 */
bool ucsi_sleep_allowed()
{    
    if( (gl_ucsi_cmd_available) || (gl_ucsi.ports[TYPEC_PORT_0_IDX].notifications) 
#if CCG_PD_DUALPORT_ENABLE 
       || (gl_ucsi.ports[TYPEC_PORT_1_IDX].notifications)
#endif /*CCG_PD_DUALPORT_ENABLE*/   
    )
    {
        return false;
    }    
    return true;
}

static void update_dpm_config (uint8_t port)
{
    uint8_t role, dflt_role, toggle_en, try_src_snk_en;

    role = (get_pd_port_config(port)->port_role);
    dflt_role = (get_pd_port_config(port)->default_role);
    toggle_en = (get_pd_port_config(port)->drp_toggle_en);
    try_src_snk_en = (get_pd_port_config(port)->try_src_en);

    dpm_stop(port);
    dpm_update_port_config(port, role, dflt_role, toggle_en, try_src_snk_en);
    dpm_start(port);
}

static void ucsi_reset_dpm_config_handler(uint8_t port)
{
    if ((gl_ccom_cmd_rcvd_mask & (1 << port)) && (gl_typec_reset_cmd_completion & (1 << port)))
    {
        update_dpm_config (port);
        gl_ccom_cmd_rcvd_mask &= ~(1 << port);
        gl_typec_reset_cmd_completion &= ~(1 << port);
    }
}

/**
 * UCSI task handler.
 */
void ucsi_task()
{
    uint8_t port_idx, intr_status;

    if (gl_reset_pending)
    {
        if (gl_ccom_cmd_rcvd_mask)
        {
            ucsi_reset_dpm_config_handler (TYPEC_PORT_0_IDX);
#if CCG_PD_DUALPORT_ENABLE            
            ucsi_reset_dpm_config_handler (TYPEC_PORT_1_IDX);
#endif /*CCG_PD_DUALPORT_ENABLE*/            

            if (gl_ccom_cmd_rcvd_mask == 0)
            {
                gl_reset_pending = false;
                ucsi_set_status(UCSI_STS_RESET_COMPLETED);
            }
        }

        return;
    }

    /* Check if the PPM Reset command has been issued by the OPM. */
    if (gl_ucsi_cmd_available && (ucsi_regs.control.command == UCSI_CMD_PPM_RESET))
    {
        /* Handle the PPM Reset. */
        ucsi_cmd_ppm_reset_handler ();

        /* Reset the relevant variables to their default values. */
        gl_ucsi.dpm_cmd = DPM_CMD_SEND_INVALID;
        gl_ucsi.dpm_retry_pending = false;
        gl_ucsi.dpm_retry_count = MAX_DPM_CMD_RETRY_COUNT;
        gl_ucsi.dpm_cmd_status = NO_CMD;
        gl_ucsi.event_mask = 0x00;    
        gl_ucsi.active_cmd = UCSI_CMD_RESERVED;

        gl_ucsi_read_pending = false;
        gl_ucsi_cmd_available = false;

        /* Transition to PPM Idle state. */
        gl_ppm_state = PPM_IDLE;

        if (gl_reset_pending == false)
        {
            ucsi_set_status(UCSI_STS_RESET_COMPLETED);
            return;
        }
    }

    switch(gl_ppm_state)
    {
        case PPM_IDLE:
            {
                if(hpi_get_ucsi_control() == UCSI_SILENCE)
                {
                    gl_ucsi_interface_silenced = true;
                    break;
                }

                gl_ucsi_interface_silenced = false;

                /* If the OPM hasn't sent any command, then process notifications */ 
                if(gl_ucsi_cmd_available == false)
                {
                    /* If the I2C bus is busy (EC/BIOS) is writing a new command), don't send the
                     * notification now */
                    if(gl_is_i2c_busy)
                        break;

                    /* Notify only if the command completed event is set */
                    if(!(gl_ucsi.event_mask & UCSI_EVT_CMD_COMPLETED))
                        break;

                    /* If a notification is pending, wait until it's cleared */
                    if(ucsi_regs.cci.connector_change)
                        break;

                    for(port_idx = 0; port_idx < NO_OF_TYPEC_PORTS; port_idx++)
                    {
                        if(gl_ucsi.ports[port_idx].notifications)
                        {
                            intr_status = CyEnterCriticalSection();
                            /* If a transaction was just started, don't send the notification */
                            if(!gl_is_i2c_busy)
                            {
                                ucsi_regs.cci.data_length = 0;
                                ucsi_regs.cci.connector_change = port_idx + 1;
                                gl_ppm_state = PPM_READ_PENDING;
                                /*Update UCSI Status Register*/
                                ucsi_clear_status_bit(UCSI_CMD_IN_PROGRESS);
                                ucsi_set_status(gl_response);
                            }
                            CyExitCriticalSection(intr_status);
                            break;
                        }
                    }
                    break;
                }

                gl_ucsi_cmd_available = false;

                /* If the command completed notification isn't set, then fail this command */
                if(ppm_cmd_requires_completion(ucsi_regs.control.command) 
                        && !(gl_ucsi.event_mask & UCSI_EVT_CMD_COMPLETED))
                {
                    gl_response = ucsi_error(UCSI_ERR_UNRECOGNIZED_CMD);
                    memset(&ucsi_regs.cci, 0x00, sizeof(ucsi_regs.cci));
                    break;
                }
                else
                {
                    /* Clear the data memory first */
                    memset(ucsi_regs.message_in, 0x00, sizeof(ucsi_regs.message_in));
                    ucsi_regs.cci.data_length = 0;
#if UCSI_SET_PWR_LEVEL_ENABLE
                    if( (ucsi_regs.control.command > UCSI_CMD_SET_POWER_LEVEL)
#else
                    if( (ucsi_regs.control.command > UCSI_CMD_GET_ERROR_STATUS)
#endif /*UCSI_SET_PWR_LEVEL_ENABLE*/                    
#if CCG_UCSI_REV_1_1_ENABLE
                            || (ucsi_regs.control.command == UCSI_CMD_SET_PDM)
#endif /* CCG_UCSI_REV_1_1_ENABLE*/    

                      )
                    {
                        gl_response = UCSI_STS_NOT_SUPPORTED | UCSI_STS_CMD_COMPLETED;                    
                        ucsi_set_status(gl_response);
                        break;
                    }

                    gl_response = ucsi_handle_resets();

                    if(gl_response == UCSI_STS_IGNORE)
                    {
                        gl_response = ucsi_handle_ack_cmds();
                    }

                    if(gl_response == UCSI_STS_IGNORE)
                        gl_response = ucsi_handle_ppm_cmds();

                    if(gl_response == UCSI_STS_IGNORE)
                        gl_response = ucsi_handle_port_cmd();
                }

                /* If this command requires time to finish, then wait in the BUSY state */
                if(gl_response == UCSI_STS_BUSY)
                    gl_ppm_state = PPM_BUSY;
                else if(gl_response == UCSI_STS_IGNORE)
                {
                    /* If nobody processed the command, drop it and go back to the idle state */
                    return;
                }
                else if ((gl_response == UCSI_STS_RESET_COMPLETED) || (gl_response == UCSI_STS_ACK_CMD))
                {
                    /* Update the response. */
                }
                else
                {
                    gl_ppm_state = PPM_READ_PENDING;
                    /* Always set the command completed notification */
                    if((gl_response != UCSI_STS_RESET_COMPLETED) 
                            && (gl_response != UCSI_STS_ACK_CMD))
                        gl_response |= UCSI_STS_CMD_COMPLETED;
                }

                ucsi_set_status(gl_response);
                break;
            }

        case PPM_READ_PENDING:
            
            /* If the EC/BIOS started to write data here, then we're out of sequence.
             * Reset the state machine back to IDLE */
            if(gl_ucsi_cmd_available && (ucsi_regs.control.command != UCSI_CMD_PPM_RESET))
            {
                gl_ucsi.ports[TYPEC_PORT_0_IDX].event_filter = 0;
#if CCG_PD_DUALPORT_ENABLE
                gl_ucsi.ports[TYPEC_PORT_1_IDX].event_filter = 0;
#endif
                hpi_clear_event (UCSI_READ_PENDING_EVENT);
                ucsi_clear_status_bit (UCSI_EVENT_PENDING);
                ucsi_clear_status_bit(UCSI_CMD_IN_PROGRESS);
                gl_ppm_state = PPM_IDLE;
                break;
            }

            if(gl_ucsi_read_pending)
            {
                ucsi_set_status_bit(UCSI_CMD_IN_PROGRESS);                
                break;
            }

            /* ACK is required. Move to the respective state */
            if((gl_ucsi.active_cmd != UCSI_CMD_RESERVED) && (ucsi_regs.cci.indicators != UCSI_STS_BUSY))
            {
                //ucsi.active_cmd = UCSI_CMD_RESERVED;
                gl_ppm_state = PPM_WAIT_FOR_ACK;
            }
            else /* Command is done. Clear the notification fiter and move to IDLE */
            {
                gl_ucsi.ports[TYPEC_PORT_0_IDX].event_filter = 0;
#if CCG_PD_DUALPORT_ENABLE
                gl_ucsi.ports[TYPEC_PORT_1_IDX].event_filter = 0;
#endif
                gl_ppm_state = PPM_IDLE;
            }            
            break;

        case PPM_BUSY:
            /* If the DPM command wasn't sent, retry */
            if(gl_ucsi.dpm_cmd_status == CMD_QUEUED)
            {
                ucsi_send_dpm_cmd();
                break;
            }
                       
            /* If OPM hasn't read the data or if the DPM command is in progress, wait */
            if(gl_ucsi_read_pending || (gl_ucsi.dpm_cmd_status == CMD_IN_PROGRESS))
            {                              
            
                /* If the command completed notification isn't set, then fail this command */
                if(ppm_cmd_requires_completion(ucsi_regs.control.command) 
                    && !(gl_ucsi.event_mask & UCSI_EVT_CMD_COMPLETED))
                {
                    gl_response = ucsi_error(UCSI_ERR_UNRECOGNIZED_CMD);
                }
                else
                {
                    
                    if( (ucsi_regs.cci.indicators == UCSI_STS_BUSY) && (ucsi_regs.control.command == UCSI_CMD_CANCEL) )
                    {
                        /* Clear the data memory first */
                        memset(ucsi_regs.message_in, 0x00, sizeof(ucsi_regs.message_in));
                        ucsi_regs.cci.data_length = 0;
                        
                        gl_response = ucsi_handle_ack_cmds();
                    
                        /* If this command requires time to finish, then wait in the BUSY state */
                        if(gl_response == UCSI_STS_BUSY)
                            gl_ppm_state = PPM_BUSY;
                        else if(gl_response == UCSI_STS_IGNORE)
                        {
                            /* If nobody processed the command, drop it and go back to the idle state */
                            return;
                        }
                        else
                        {
                            gl_ppm_state = PPM_READ_PENDING;
                            /* Always set the command completed notification */
                            if((gl_response != UCSI_STS_RESET_COMPLETED) && (gl_response != UCSI_STS_ACK_CMD))
                                gl_response |= UCSI_STS_CMD_COMPLETED;
                        }

                        ucsi_set_status(gl_response);                        
                    }
                    else if(ucsi_regs.cci.indicators == UCSI_STS_BUSY)
                    {
#if (!(UCSI_HLK_SPECIFIC))
                        /*
                            As per spec when PPM is busy / Busy Indicator field is set to one, 
                            then no other bits in the UCSI data structure shall be set by the PPM.
                            However, GET_PDO RS5 HLK test fails due to EC delayed reading of UCSI INT.
                            In this case, CCG5 will clear data data length and EC will read zero length.
                        */
                        ucsi_regs.cci.data_length = 0;
                        ucsi_regs.cci.connector_change = 0;
#endif /*UCSI_HLK_SPECIFIC*/    
                        /* If this command requires time to finish, then wait in the BUSY state */
                        gl_ppm_state = PPM_BUSY;                        
                    }  
                }
                break;
            }

            /* DPM command is done. Send the result to the OPM */
            if(gl_ucsi.dpm_cmd_status == CMD_ERROR)
            {
                ucsi_set_status(UCSI_STS_ERROR | UCSI_STS_CMD_COMPLETED);
            }
            else
            {
                ucsi_set_status(UCSI_STS_CMD_COMPLETED);
            }
            gl_ucsi.dpm_cmd_status = NO_CMD;
            gl_ppm_state = PPM_READ_PENDING;
            break;

        case PPM_WAIT_FOR_ACK:
            if(!gl_ucsi_cmd_available)
                break;

            gl_ucsi_cmd_available = false;

            /* Clear the data memory first */
            memset(ucsi_regs.message_in, 0x00, sizeof(ucsi_regs.message_in));
            ucsi_regs.cci.data_length = 0;

            gl_response = ucsi_handle_ack_cmds();
            if(gl_response == UCSI_STS_IGNORE) /* Not an ACK or CANCEL */
                gl_response = ucsi_handle_resets();
            
            if(gl_response == UCSI_STS_IGNORE) /* Not a PPM_RESET */
                gl_response = ucsi_error(UCSI_ERR_UNRECOGNIZED_CMD) | UCSI_STS_CMD_COMPLETED;

            ucsi_set_status(gl_response);
            gl_ppm_state = PPM_READ_PENDING;
            break;
    }
}

#if CCG_UCSI_ENABLE_DIFF_ADDR
/* I2C command callback for the UCSI implementation */    
bool ucsi_i2c_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count)
{
    uint16_t size;
    uint8_t  num_bytes_read;
    static uint8_t read_offset = 0;
    bool status = false;
    
    (void)i2c_state;
    
    if (cmd == I2C_CB_SLAVE_ADDR_MATCH)
    {
        gl_is_i2c_busy = true;
        return true;
    }
    
    if (cmd == I2C_CB_CMD_TIMEOUT)
    {
        memset (gl_ucsi_buffer, 0, UCSI_BUFFER_SIZE);
        return true;
    }

    if (cmd == I2C_CB_CMD_WRITE)
    {
#if ENABLE_ST_SP_UCSI_CMD        
        if(gl_i2c_owner != I2C_OWNER_UCSI)
            return false;
#endif /*ENABLE_ST_SP_UCSI_CMD*/        
        
        if (count < UCSI_MIN_WRITE_SIZE)
        {
            read_offset = gl_ucsi_buffer[0];
            status = true;
        }
        else
        {
            if(gl_ucsi_buffer[0] >= UCSI_REG_MESSAGE_OUT)
            {
                memcpy(ucsi_regs.message_out + (gl_ucsi_buffer[0] - UCSI_REG_MESSAGE_OUT), &gl_ucsi_buffer[1], count - 1);
                status = true;
            }
            
            else if(gl_ucsi_buffer[0] == UCSI_REG_CONTROL)
            {
                memcpy(&ucsi_regs.control, &gl_ucsi_buffer[1], count - 1);
                gl_ucsi_cmd_available = true;
                status = true;
                gl_is_i2c_busy = false;
            }
        }
    }

    if (cmd == I2C_CB_CMD_READ)
    {
#if ENABLE_ST_SP_UCSI_CMD        
        if(gl_i2c_owner != I2C_OWNER_UCSI)
            return false;
#endif /*ENABLE_ST_SP_UCSI_CMD*/        

        size = GET_MIN (I2C_SCB_TX_FIFO_SIZE, (unsigned int)(sizeof(ucsi_regs) - read_offset));
        if (size != 0)
        {
            i2c_scb_write (ucsi_scb_idx, ((uint8_t *)&ucsi_regs) + read_offset, (uint8_t)size, &num_bytes_read);
            read_offset += num_bytes_read;
            status = true;
        }
    }
    
    if (cmd == I2C_CB_CMD_XFER_END)
    {
        gl_is_i2c_busy = false; /* Read done. End of transfer */
    }
    
    /* The buffer pointer will be reset to 0 by the SCB interrupt handler. */
    return status;
}
#endif /*CCG_UCSI_ENABLE_DIFF_ADDR*/

/**
 *  @brief UCSI timeout callback function.
 *  @param port port index
 *  @param timer_id_t timer id.
 *  @return NULL
 */
static void ucsi_tmr_cbk(uint8_t port, timer_id_t id)
{    
    ucsi_set_status_bit(UCSI_EVENT_PENDING);     
//    ucsi_notify(port, UCSI_EVT_CONNECT_CHANGED | UCSI_EVT_PARTNER_STATUS_CHANGED);    
}

/**
 *  @brief Check any change in port connection.
 *  @param port port index
 *  
 *  @return true if the port connect is changed, false otherwise.
 */
bool is_port_connect_changed(uint8_t port)
{
    if( gl_ucsi.ports[port].notifications & UCSI_EVT_CONNECT_CHANGED)
    {
        gl_ucsi.ports[port].notifications = 0;
        return 1;
    }
   
    return 0;
}

#endif /* CCG_UCSI_ENABLE */

/* [] END OF FILE */

