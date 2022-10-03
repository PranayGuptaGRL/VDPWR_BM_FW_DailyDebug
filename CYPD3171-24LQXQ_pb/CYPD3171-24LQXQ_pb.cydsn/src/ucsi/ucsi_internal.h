/**
 * @file hpi_internal.h
 *
 * @brief @{Host Processor Interface (HPI) stack internal header file.@}
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

#ifndef _UCSI_INTERNAL_H_
#define _UCSI_INTERNAL_H_

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "status.h"
#include "i2c.h"

#if CCG_HPI_PD_ENABLE
#include "pd.h"
#include "dpm.h"
#endif /* CCG_HPI_PD_ENABLE */

#define PACK_STRUCT                      __attribute__((packed, aligned(1)))

//#define UCSI_MIN_WRITE_SIZE             (3)
#define UCSI_MAX_DATA_LENGTH            (16)
#define UCSI_RESPONSE_TIMEOUT           (10)
#define UCSI_GET_ERROR_STS_DATA_LENGTH  (16)

/* Size of the UCSI Data section in HPI registers. */
#define UCSI_REG_PORTSECTION_SIZE       (48)

#define UCSI_MIN_TIME_TO_RESPOND_WITH_BUSY_ID       (0x01)
#define UCSI_MIN_TIME_TO_RESPOND_WITH_BUSY_PERIOD   (0x10)

/* UCSI Spec Version Info. */
#define UCSI_MAJOR_VERSION              (1)
#if CCG_UCSI_REV_1_1_ENABLE
#define UCSI_MINOR_VERSION              (1)
#else    
#define UCSI_MINOR_VERSION              (0)
#endif /*CCG_UCSI_REV_1_1_ENABLE*/    
#define UCSI_SUBMINOR_VERSION           (0)
#define UCSI_VERSION                    (UCSI_SUBMINOR_VERSION | (UCSI_MINOR_VERSION << 4) | (UCSI_MAJOR_VERSION << 8))

#define UCSI_BOUND(bound, lenval)       GET_MIN(GET_MIN(UCSI_MAX_DATA_LENGTH, lenval), bound)

/* Maximum number of SVIDs that the module stores and returns */
#define UCSI_MAX_SVIDS                          (4u)

/***** Internal timers *****/
#define UCSI_DPM_RETRY_TIMER                    (63u)
#define UCSI_DPM_RETRY_TIMER_PERIOD             (5u)

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/
/**
 * @typedef ucsi_ctrl_details_t
 * @brief Union to hold the Control Data Structure.
 */
typedef union {
    uint8_t data[6];
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1        : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT generic;
    struct {
        uint8_t connector_number : 7;
#if (!CCG_UCSI_REV_1_1_ENABLE)         
        uint8_t hard_reset       : 1;
#else    
        uint8_t reserved1 : 1;
#endif /* CCG_UCSI_REV_1_1_ENABLE */    
        uint8_t reserved2[5];
    } PACK_STRUCT connector_reset;
    struct {
        uint8_t connector_change_ack  : 1;
        uint8_t command_completed_ack : 1;
        uint8_t reserved1             : 6;
        uint8_t reserved2[5];
    } PACK_STRUCT ack_cc_ci;
    struct {
        uint16_t notifications;
        uint8_t  reserved[4];
    } PACK_STRUCT set_notification_enable;
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT get_connector_capability;
    struct {
        uint16_t connector_number   : 7;
        uint16_t usb_operating_mode : 3;
        uint16_t reserved1          : 6;
        uint8_t reserved2[4];
    } PACK_STRUCT set_uom;
    struct {
        uint16_t connector_number   : 7;
        uint16_t operate_as_dfp     : 1;
        uint16_t operate_as_ufp     : 1;
        uint16_t accept_dr_swap     : 1;
        uint16_t reserved1          : 6;
        uint8_t reserved2[4];
    } PACK_STRUCT set_uor;
    struct {
        uint16_t connector_number     : 7;
        uint16_t power_direction_mode : 3;
        uint16_t reserved1            : 6;
        uint8_t reserved2[4];
    } PACK_STRUCT set_pdm;
    struct {
        uint16_t connector_number     : 7;
        uint16_t operate_as_src     : 1;
        uint16_t operate_as_snk     : 1;
        uint16_t accept_pr_swap     : 1;
        uint16_t reserved1            : 6;
        uint8_t reserved2[4];
    } PACK_STRUCT set_pdr;
    struct {
        uint8_t recipient;
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t alterate_mode_offset;
        uint8_t num_alternate_modes;
        uint8_t reserved2[2];
    } PACK_STRUCT get_alternate_modes;
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT get_cam_supported;
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT get_current_cam;
    struct {
        uint8_t connector_number : 7;
        uint8_t enter_or_exit : 1;
        uint8_t new_cam;
        uint8_t alt_mode_specific[4];
    } PACK_STRUCT set_new_cam;
    struct {
        uint8_t connector_number : 7;
        uint8_t partner_pdo : 1;
        uint8_t pdo_offset;
        uint8_t num_pdos : 2;
        uint8_t is_source_pdo : 1;
        uint8_t src_cap_type : 2;
        uint8_t reserved8 : 3;
        uint8_t reserved[3];
    } PACK_STRUCT get_pdos;
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT get_cable_property;
    struct {
        uint8_t connector_number : 7;
        uint8_t reserved1 : 1;
        uint8_t reserved2[5];
    } PACK_STRUCT get_connector_status;
#if CCG_UCSI_REV_1_1_ENABLE
    struct {
        uint8_t connector_number : 7;
        uint8_t src_snk : 1;
        uint8_t usb_pd_max_pwr;
        uint8_t usb_typec_current : 2;
        uint8_t reserved1 : 6;
        uint8_t reserved2[3];
    } PACK_STRUCT set_power_level;
#endif /*CCG_UCSI_REV_1_1_ENABLE*/    
} PACK_STRUCT ucsi_ctrl_details_t;

/**
 * @typedef ucsi_reg_space_t
 * @The structure of the memory locations used to pass information between LPM/OPM/PPM.
 * See Sec 3 of UCSI specification.
 */
typedef struct {
    uint16_t version;
    uint16_t reserved;
    struct {
        uint8_t reserved1              : 1;
        uint8_t connector_change       : 7;
        uint8_t data_length;
        uint8_t reserved2;
        uint8_t indicators;
    } PACK_STRUCT cci;
    struct {
        uint8_t command;
        uint8_t data_length;
        ucsi_ctrl_details_t details;
    } control;
    uint8_t message_in[16];
    uint8_t message_out[16];
} ucsi_reg_space_t;

/* UCSI Register */
#define UCSI_REG_VERSION                    (0x00)
#define UCSI_REG_CCI                        (0x04)
#define UCSI_REG_CONTROL                    (0x08)
#define UCSI_REG_MESSAGE_IN                 (0x10)
#define UCSI_REG_MESSAGE_OUT                (0x20)

/* UCSI Data length for different commands */
#define MAX_DATA_LENGTH                         (0x10)
#define GET_CAPABILITY_STATUS_DATA_LENGTH       (0x10)
#define GET_ERROR_STATUS_DATA_LENGTH            (0x10)
#define GET_CONNECTOR_STATUS_DATA_LENGTH        (0x09)
#define GET_CABLE_PROPERTY_DATA_LENGTH          (0x05)
#define GET_CONNECTOR_CAPABILITY_DATA_LENGTH    (0x02)

/**
 * @typedef ucsi_connector_status_t
 * @Set of UCSI Connector Status Register. The structure of the memory locations 
 * used to pass information between LPM/OPM/PPM.
 * See Table 4.42 of UCSI specification.
 */
typedef struct {
    uint16_t connector_status_change;
    uint16_t power_op_mode : 3;
    uint16_t connect_status : 1;
    uint16_t power_direction : 1;
    uint16_t connector_partner_flags : 8;
    uint16_t connector_partner_type : 3;
    uint32_t rdo;
    uint8_t  battery_charging_status : 2;
    uint8_t  provider_caps_limited_reason : 4;
    uint8_t  reserved : 2;
} PACK_STRUCT ucsi_connector_status_t;

/**
 * @typedef Set of ucsI ucsi_evmask_t
 * @brief UCSI notification mask definitions.
 * This enumeration lists the various mask values that control the reporting of UCSI events.
 */
typedef enum {
    UCSI_EVT_CMD_COMPLETED                  = 0x0001u,
    UCSI_EVT_EXT_SUPPLY_CHANGED             = 0x0002u,
    UCSI_EVT_POM_CHANGED                    = 0x0004u,
    UCSI_EVT_RSVD1                          = 0x0008u,
    UCSI_EVT_RSVD2                          = 0x0010u,
    UCSI_EVT_CAP_CHANGED                    = 0x0020u,
    UCSI_EVT_PD_CONTRACT_CHANGED            = 0x0040u,
    UCSI_EVT_HARD_RESET_COMPLETE            = 0x0080u,
    UCSI_EVT_SUPPORTED_CAM_CHANGED          = 0x0100u,
    UCSI_EVT_BATT_CHARGING_STATUS_CHANGED   = 0x0200u,
    UCSI_EVT_RSVD3                          = 0x0400u,
    UCSI_EVT_PARTNER_STATUS_CHANGED         = 0x0800u,
    UCSI_EVT_POWER_DIRECTION_CHANGED        = 0x1000u,
    UCSI_EVT_RSVD4                          = 0x2000u,
    UCSI_EVT_CONNECT_CHANGED                = 0x4000u,
    UCSI_EVT_ERROR                          = 0x8000u
} ucsi_evmask_t;

/**
 * @typedef ucsi_status_indicator_t
 * @brief This enumeration holds CCI status indicator.
 */
typedef enum {
    UCSI_STS_IGNORE             = 0x00,
    UCSI_STS_RESERVED           = 0x01,
    UCSI_STS_NOT_SUPPORTED      = 0x02,
    UCSI_STS_CANCEL_COMPLETED   = 0x04,
    UCSI_STS_RESET_COMPLETED    = 0x08,
    UCSI_STS_BUSY               = 0x10,
    UCSI_STS_ACK_CMD            = 0x20,
    UCSI_STS_ERROR              = 0x40,
    UCSI_STS_CMD_COMPLETED      = 0x80,
} ucsi_status_indicator_t;

/**
 * @typedef ucsi_cmd_t
 * @brief This enumeration holds PPM Controller Commands.
 */
typedef enum
{
    UCSI_CMD_RESERVED = 0x00u,          /* Command Code = 0x00 */
    UCSI_CMD_PPM_RESET,                 /* Command Code = 0x01 */
    UCSI_CMD_CANCEL,                    /* Command Code = 0x02 */ 
    UCSI_CMD_CONNECTOR_RESET,           /* Command Code = 0x03 */
    UCSI_CMD_ACK_CC_CI,                 /* Command Code = 0x04 */
    UCSI_CMD_SET_NOTIFICATION_ENABLE,   /* Command Code = 0x05 */
    UCSI_CMD_GET_CAPABILITY,            /* Command Code = 0x06 */
    UCSI_CMD_GET_CONNECTOR_CAPABILITY,  /* Command Code = 0x07 */
    UCSI_CMD_SET_CCOM,                   /* Command Code = 0x08 */
    UCSI_CMD_SET_UOR,                   /* Command Code = 0x09 */
    UCSI_CMD_SET_PDM,                   /* Command Code = 0x0A */
    UCSI_CMD_SET_PDR,                   /* Command Code = 0x0B */
    UCSI_CMD_GET_ALTERNATE_MODES,       /* Command Code = 0x0C */
    UCSI_CMD_GET_CAM_SUPPORTED,         /* Command Code = 0x0D */
    UCSI_CMD_GET_CURRENT_CAM,           /* Command Code = 0x0E */
    UCSI_CMD_SET_NEW_CAM,               /* Command Code = 0x0F */
    UCSI_CMD_GET_PDOS,                  /* Command Code = 0x10 */
    UCSI_CMD_GET_CABLE_PROPERTY,        /* Command Code = 0x11 */
    UCSI_CMD_GET_CONNECTOR_STATUS,      /* Command Code = 0x12 */
    UCSI_CMD_GET_ERROR_STATUS,          /* Command Code = 0x13 */
    UCSI_CMD_SET_POWER_LEVEL            /* Command Code = 0x14 */
} ucsi_cmd_t;

/**
 * @typedef ucsi_capability_t
 * @brief Struct to hold UCSI Get Capability Data.
 */
typedef struct {
    uint32_t bmAttributes;
    uint8_t  bNumConnectors;
    uint32_t bmOptionalFeatures : 24;
    uint32_t bNumAltModes : 8;
    uint8_t  reserved;
    uint16_t bcdBCVersion;
    uint16_t bcdPDVersion;
    uint16_t bcdUSBTypeCVersion;
} ucsi_capability_t;

/**
 * @typedef ucsi_capability_t
 * @brief Struct to hold UCSI Get Cable Property Data.
 */
typedef struct {
    uint16_t bmSpeedSupported;
    uint8_t  bCurrentCapability;
    uint8_t  VBUSInCable : 1;
    uint8_t  CableType : 1;
    uint8_t  Directionality : 1;
    uint8_t  PlugEndType : 2;
    uint8_t  ModeSupport : 1;
    uint8_t  Reserved : 2;
    uint8_t  Latency : 4;
    uint8_t  Reserved2 : 4;
} ucsi_cable_property_t;

/* bmAttributes (Get Capability Cmd) macros.*/
#define TYPEC_DISABLED_SUPPORTED            (0x0001)

#if BATTERY_CHARGING_ENABLE
#define BC_SUPPORTED                        (0x0002)
#else
#define BC_SUPPORTED                        (0x0000)
#endif /* BATTERY_CHARGING_ENABLE */

#define USB_PD_SUPPORTED                    (0x0004)
#define TYPE_C_RP_SUPPORTED                 (0x0040)

/* The Battery Charging Specification Release Number supported by the PPM in
 * BCD format. */
#define UCSI_BC_1_2_VERSION                 (0x0120)

/* The USB Power Delivery Specification 3.0 Release Number in BCD format. */
#define UCSI_USB_PD_3_0_VERSION             (0x0300)

/* The USB Power Delivery Specification 2.0 Release Number in BCD format. */
#define UCSI_USB_PD_2_0_VERSION             (0x0200)

/* The USB Type C 1.3 Release Number in BCD format. */
#define UCSI_USB_TYPE_C_1_3_VERSION         (0x0130)

/* The USB Type C 1.2 Release Number in BCD format. */
#define UCSI_USB_TYPE_C_1_2_VERSION         (0x0120)

/* bmPowerSource (Get Capability Cmd) macros */
#define AC_SUPPLY_SUPPORTED                 (0x0100)
#define OTHER_SUPPLY_SUPPORTED              (0x0400)
#define PD_SUPPLY_SUPPORTED                 (0x4000)
 
#define ALL_ATTRIBUTES_SUPPORTED            (TYPEC_DISABLED_SUPPORTED | BC_SUPPORTED | USB_PD_SUPPORTED |\
                                             TYPE_C_RP_SUPPORTED | AC_SUPPLY_SUPPORTED | OTHER_SUPPLY_SUPPORTED )

/* Optional features macros.*/
#define SET_UOM_SUPPORTED                   (0x01)
#define SET_PDM_SUPPORTED                   (0x02)
#define ALT_MODE_DETAILS_SUPPORTED          (0x04)
#define ALT_MODE_OVERRIDE_SUPPORTED         (0x08)
#define PDO_DETAILS_SUPPORTED               (0x10)
#define CABLE_DETAILS_SUPPORTED             (0x20)
#define EXT_SUPPLY_NOTFN_SUPPORTED          (0x40)
#define HARD_RESET_NOTFN_SUPPORTED          (0x80)

#if  CCG_UCSI_REV_1_1_ENABLE
#define UCSI_OPTIONAL_FEATURES                (ALT_MODE_DETAILS_SUPPORTED | ALT_MODE_OVERRIDE_SUPPORTED|\
                                             PDO_DETAILS_SUPPORTED | CABLE_DETAILS_SUPPORTED | CABLE_DETAILS_SUPPORTED |\
                                             EXT_SUPPLY_NOTFN_SUPPORTED | HARD_RESET_NOTFN_SUPPORTED | SET_UOM_SUPPORTED )         
#else    
#define v1_OPTIONAL_FEATURES                (ALT_MODE_DETAILS_SUPPORTED | ALT_MODE_OVERRIDE_SUPPORTED|\
                                             PDO_DETAILS_SUPPORTED | CABLE_DETAILS_SUPPORTED | CABLE_DETAILS_SUPPORTED |\
                                             EXT_SUPPLY_NOTFN_SUPPORTED | HARD_RESET_NOTFN_SUPPORTED | SET_UOM_SUPPORTED | SET_PDM_SUPPORTED )  
#endif /*CCG_UCSI_REV_1_1_ENABLE*/ 

#define UCSI_ALL_OPTIONAL_FEATURES               (UCSI_OPTIONAL_FEATURES | SET_UOM_SUPPORTED | ALT_MODE_OVERRIDE_SUPPORTED)

/* Error Information macros.*/
#define UCSI_ERR_UNRECOGNIZED_CMD           (0x0001)
#define UCSI_ERR_UNKNOWN_CONNECTOR          (0x0002)
#define UCSI_ERR_INVALID_PARAMS             (0x0004)
#define UCSI_ERR_INCOMPAT_PARTNER           (0x0008)
#define UCSI_ERR_PD_COMM_ERROR              (0x0010)
#define UCSI_ERR_BATTERY_DEAD               (0x0020)
#define UCSI_ERR_CONTRACT_NEG_FAILURE       (0x0040)

#if CCG_UCSI_REV_1_1_ENABLE
#define UCSI_ERR_OVERCURRENT                (0x0080)
#define UCSI_ERR_UNDEFINED                  (0x0100)
#define UCSI_ERR_PORT_PARTNER_REJECT_SWAP   (0x0200)
#define UCSI_ERR_HARD_RESET                 (0x0400)
#define UCSI_ERR_PPM_POLICY_CONFLICT        (0x0800)
#define UCSI_ERR_SWAP_REJECTED              (0x1000)
#endif /*CCG_UCSI_REV_1_1_ENABLE*/

/* Source Capabilities Type macros*/
#define UCSI_CUR_SUPPORTED_SRC_CAPS         (0x00)
#define UCSI_ADVERTISED_SRC_CAPS            (0x01)
#define UCSI_MAX_SUPPORTED_SRC_CAPS         (0x02)
#define UCSI_UNKNOWN_SRC_CAPS               (0x03)

/* Battery Charging Status macros*/
#define BATT_STS_NOT_CHARGING               (0)
#define BATT_STS_NOMINAL_CHARGING           (1)
#define BATT_STS_SLOW_CHARGING              (2)
#define BATT_STS_VERY_SLOW_CHARGING         (3)

/***** GET_CONNECTOR_STATUS 'Power Operation Mode' types *****/
#define PWR_OP_MODE_NO_CONSUMER             (0x00)
#define PWR_OP_MODE_TYPE_C_DEFAULT          (0x01)
#define PWR_OP_MODE_BC_1_2                  (0x02)
#define PWR_OP_MODE_USB_PD                  (0x03)
#define PWR_OP_MODE_TYPE_C_1_5A             (0x04)
#define PWR_OP_MODE_TYPE_C_3_0A             (0x05)

/* Operation Mode (Get Connector Capability) macros*/
#define CONN_CAP_DFP_ONLY                   (0x0001)
#define CONN_CAP_UFP_ONLY                   (0x0002)
#define CONN_CAP_DRP                        (0x0004)
#define CONN_CAP_ANALOG_ACC_SUPP            (0x0008)
#define CONN_CAP_DEBUG_ACC_SUPP             (0x0010)
#define CONN_CAP_USB_2_0_SUPP               (0x0020)
#define CONN_CAP_USB_3_0_SUPP               (0x0040)
#define CONN_CAP_ALT_MODE_SUPP              (0x0080)
#define CONN_CAP_PWR_SRC_SUPP               (0x0100)
#define CONN_CAP_PWR_SNK_SUPP               (0x0200)
#if CCG_UCSI_REV_1_1_ENABLE
#define CONN_CAP_SWAP_TO_DFP                (0x0400)    
#define CONN_CAP_SWAP_TO_UFP                (0x0800)    
#define CONN_CAP_SWAP_TO_SRC                (0x1000)    
#define CONN_CAP_SWAP_TO_SNK                (0x2000)    
#endif /* CCG_UCSI_REV_1_1_ENABLE */    

#define ALT_MODE_SVID_SIZE                  (2u)
#define ALT_MODE_MID_SIZE                   (4u)

/* Connector Partner Type (Get Connector Status) macros*/
#define CONN_PARTNER_DFP                    (0x01)
#define CONN_PARTNER_UFP                    (0x02)
#define CONN_PARTNER_PWR_CBL_NO_UFP         (0x03)
#define CONN_PARTNER_PWR_CBL_UFP            (0x04)
#define CONN_PARTNER_DBG_ACC                (0x05)
#define CONN_PARTNER_AUD_ACC                (0x06)

/* Connector Partner Flags macros*/
#define CONN_PARTNER_FLAG_USB               (0x01)
#define CONN_PARTNER_FLAG_ALT_MODE          (0x02)

/* Device Policy Manager swap command macros.*/
#define DPM_DR_SWAP_RESP_MASK               (0x03)
#define DPM_DR_SWAP_RESP_POS                (0x00)
#define DPM_PR_SWAP_RESP_MASK               (0x0C)
#define DPM_PR_SWAP_RESP_POS                (0x02)

#define PWR_LEVEL_SINK                      (0x00)
#define PWR_LEVEL_SOURCE                    (0x01)

#endif /* _UCSI_INTERNAL_H_ */

/* [] END OF FILE */
