/**
 * @file pdss_mx_hal.c
 *
 * @brief @{CCG MX-PD PHY driver module source file (CCG5, CCG3PA families).@}
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
#include <hal_ccgx.h>
#include <pd.h>
#include <pd_protocol.h>
#include <dpm.h>
#include <dpm_intern.h>
#include <typec_manager.h>
#include <pdss_hal.h>
#include <ccgx_regs.h>
#include <status.h>
#include <gpio.h>
#include <hpd.h>
#include <timer.h>
#include <utils.h>
#include <chgb_hal.h>
#include <battery_charging.h>
#include <system.h>
#include <app.h>
#include <srom_vars.h>
#include <grlapp.h>
/**
 * Type C voltage thresholds (in mV) as per Section 4.11.3 of Type C
 * specification Rev1.
 */
const uint8_t thresholds[4][4] =
{
    {PD_CMP_VSEL_0_2_V, PD_CMP_VSEL_1_77_V, 0, 0}, /* Rp USB default row. */
    {PD_CMP_VSEL_0_4_V, PD_CMP_VSEL_1_77_V, 0, 0}, /* Rp 1.5A row. */
    {PD_CMP_VSEL_0_8_V, PD_CMP_VSEL_2_6_V, 0, 0}, /* Rp 3A row. */
    {PD_CMP_VSEL_0_2_V, PD_CMP_VSEL_0_655_V, PD_CMP_VSEL_1_235_V, PD_CMP_VSEL_2_6_V} /* RD row. */
};

/* Ordered sets for transmission. */
const uint32_t os_table[SOP_INVALID] =
{
    0x8E318u,      /**< SOP Default. */
    0x31B18u,      /**< SOP Prime. */
    0x360D8u,      /**< SOP Double Prime. */
    0x34F98u,      /**< SOP Prime Debug. */
    0x89A78u,      /**< SOP Double Prime Debug. */
    0xE7393u,      /**< Hard Reset. */
    0xE0F8Cu       /**< Cable Reset. */
};

#define VBUS_C_20_PER_DIV                   (5)       /* 20% */
#define VBUS_C_10_PER_DIV                   (10)      /* 10% */
#define VBUS_C_8_PER_DIV                    (12)      /* APPROX 8% */

/* On CCG5, AMUX_NHV[4] enables connection of VBus divider to ADC_IN_2. */
#define AMUX_ADC_CCG5_VBUS_DIV_EN_POS       (4)

/* Selecting 8% divider for connection of VBus divider to ADC. */
#define AMUX_ADC_CCG5_VBUS_DIV_8PC_POS      (3)

/* CCG3PA enable VBUS_IN resistor divider for TYPE-C VBUS monitoring using ADC. */
#define AMUX_ADC_CCG3PA_VBUS_IN_8P_EN_POS   (7)
#define AMUX_ADC_CCG3PA_VBUS_IN_20P_EN_POS  (4)

/* CCG3PA switch to 20% divider option at the final MUX. */
#define AMUX_ADC_CCG3PA_VBUS_DIV_2OP_EN_POS (14)

/* CCG3PA discharge drive strength settings. */
#define CCG3PA_DISCHG_DS_VBUS_C_00          (1)
#define CCG3PA_DISCHG_DS_VBUS_IN_00         (1)

/* Location of SFLASH flag that indicates validity of VConn OCP trim setting. */
#define SFLASH_VCONN_TRIM_ENABLE_ADDR       (0x0FFFF400)

#if VBUS_SLOW_DISCHARGE_EN

/* Placeholder for VBUS discharge strength for DISCHG_SHV_CTRL[0]*/
static uint8_t gl_vbus_discharge_ds_0;

/* Placeholder for VBUS discharge strength for DISCHG_SHV_CTRL[1]*/
static uint8_t gl_vbus_discharge_ds_1;

/* Flag determines if VBUS discharge ON request is ongoing */
static uint8_t gl_vbus_is_slow_discharge_on;

/* Flag determines if VBUS discharge OFF request is ongoing */
static uint8_t gl_vbus_is_slow_discharge_off;

/* Time interval in ms between every steps of discharge strength setting */
#define VBUS_SLOW_DISCHARGE_TIME_MS          (1)

/* Minimum VBUS_C Discharge Drive strength control value */
#define CCG3PA_DISCHG_DS_VBUS_C_MIN          (1)

/* Minimum VBUS_IN Discharge Drive strength control value */
#define CCG3PA_DISCHG_DS_VBUS_IN_MIN         (1)

/* Function Prototype for Slow Discharge callback */
static void vbus_slow_discharge_cbk(uint8_t port, timer_id_t id);

#endif /* VBUS_SLOW_DISCHARGE */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6))

/* Time period for soft turn-on */
#define SOFT_START_PERIOD                   (5u)

#if (defined(CCG3PA) || defined(CCG3PA2))
/* Starting ISnk Counter value */
#define ISNK_COUNTER_STARTING_VALUE         (5u)
#else /* CCG6 */
#define ISNK_COUNTER_STARTING_VALUE         (20u)
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

static uint16_t gl_isnk_counter_value[NO_OF_TYPEC_PORTS] = {ISNK_COUNTER_STARTING_VALUE};
static bool     gl_isnk_counter_hold[NO_OF_TYPEC_PORTS] = {false};

#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6)) */

static uint8_t  gl_ccgx_pfet_on[NO_OF_TYPEC_PORTS] = {false};
static uint8_t  gl_ccgx_cfet_on[NO_OF_TYPEC_PORTS] = {false};

/* LSCSA measured error for 15mV Vsense location in SFLASH. */
#define LSCSA_AMP1_ERR_SIGN     (*(volatile uint8_t *)(0x0ffff2a0))
#define LSCSA_AMP1_ERR_VAL      (*(volatile uint8_t *)(0x0ffff2a1))
#define LSCSA_AMP2_ERR_SIGN     (*(volatile uint8_t *)(0x0ffff2a2))
#define LSCSA_AMP2_ERR_VAL      (*(volatile uint8_t *)(0x0ffff2a3))

/* EA_IREF_GAIN TRIM setting location in SFLASH. */
#define EA_IREF_GAIN_PDAC       (*(volatile uint8_t *)(0x0ffff2a4))
#define EA_IREF_GAIN_NDAC       (*(volatile uint8_t *)(0x0ffff2a5))

/* Gain settings */
#define LSCSA_AV_SEL_150            (0x1Cu)
#define LSCSA_AV_SEL_125            (0x18u)
#define LSCSA_AV_SEL_35             (0x03u)

/* Range of VSense supported by the LSCSA */
#define LSCSA_VSENSE_MIN            (10u)
#define LSCSA_VSENSE_MAX            (600u)
#define LSCSA_GAIN_150_VSENSE_MAX   (140u)
#define LSCSA_GAIN_35_VSENSE_MIN    (160u)

/* Maximum voltage in mV that can be measured with 20% divider. */
#define VBUS_DIV_20_PER_MAX_VOLT    (9000)

/* Current threshold in 10mA units below which we detect it as CV mode. */
#define LSCSA_CF_CUR_THRES           (5)
#define LSCSA_CF_CUR_HIGH_LIMIT      (300)
#define LSCSA_CF_CUR_HIGH_THRES      (10)

/*QC related declerations*/
uint16_t gQCPrVBUS;
#define PULSE_VOLTAGE           (0xC8)   /*QC3.0 pulse voltage increment*/
#define TEST_pulse              (0x1900)
#if VBUS_OCP_ENABLE

/* VBus OCP mode. */
static uint8_t gl_vbus_ocp_mode[NO_OF_TYPEC_PORTS];

/* VBus OCP software debounce in ms. */
static uint8_t gl_ocp_sw_db_ms[NO_OF_TYPEC_PORTS];

/* PGDO type for VBUS OCP. */
bool gl_vbus_ocp_pgdo_type[NO_OF_TYPEC_PORTS];

/* AMUX Control bit to select reference voltage for OCP comparator. */
#define VBUS_OCP_AMUX_CTRL_REF_SEL_BIT_POS      (3)
#endif /* VBUS_OCP_ENABLE */

#if VBUS_OVP_ENABLE

#define AMUX_OV_VBUS_SEL_MASK           (0x00000208)
#if (defined(CCG3PA) || defined(CCG3PA2))
#define AMUX_OV_DIV_SEL_BIT_POS         (13)
#else /* CCG5 */
#define AMUX_OV_DIV_SEL_BIT_POS         (2)
#define CCG5_P1_OV_DIV_SEL_BIT_POS      (1)
#define CCG5_P1_UV_DIV_SEL_BIT_POS      (0)
#endif /* CCGx */

/* OVP callback pointer storage. */
PD_ADC_CB_T gl_ovp_cb[NO_OF_TYPEC_PORTS];

/* VBus OVP mode. */
static vbus_ovp_mode_t gl_vbus_ovp_mode[NO_OF_TYPEC_PORTS];

/* PGDO type for VBUS OVP. */
bool gl_vbus_ovp_pgdo_type[NO_OF_TYPEC_PORTS];

uint32_t gl_vbus_ovp_filter_id[NO_OF_TYPEC_PORTS];

#endif /* VBUS_OVP_ENABLE */

#if VCONN_OCP_ENABLE

/* Debounce period for VConn OCP condition. */
static uint8_t     gl_vconn_ocp_debounce[NO_OF_TYPEC_PORTS];

/* Callback to be invoked when VConn OCP is detected. */
static PD_ADC_CB_T gl_vconn_ocp_cb[NO_OF_TYPEC_PORTS];

/* CC channel on which VConn has been applied. */
static uint8_t     gl_vconn_channel[NO_OF_TYPEC_PORTS];

#endif /* VCONN_OCP_ENABLE */

#if VBUS_UVP_ENABLE

#define AMUX_UV_VBUS_SEL_MASK           (0x00000044)
#define AMUX_UV_DIV_SEL_BIT_POS         (12)

/* VBus UVP mode. */
static vbus_uvp_mode_t gl_vbus_uvp_mode;

/* UVP callback pointer storage. */
PD_ADC_CB_T gl_uvp_cb;

/* PGDO type for VBUS UVP. */
bool gl_vbus_uvp_pgdo_type;

#endif /* VBUS_UVP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))

#if VBUS_SCP_ENABLE
/* VBus SCP mode. */
static uint8_t gl_vbus_scp_mode;

/* PGDO type for VBUS SVP. */
bool gl_vbus_scp_pgdo_type;

/* AMUX Control bit to select reference voltage for SCP comparator. */
#define VBUS_SCP_AMUX_CTRL_REF_SEL_BIT_POS      (2)
#endif /* VBUS_SCP_ENABLE */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))
/* Callback used for notification of CC/SBU over-voltage conditions. */
PD_ADC_CB_T gl_ccg_fault_cb = NULL;

#if CCGX_V5V_CHANGE_DETECT
/* Status variable that stores the live status of V5V supply for each port. */
static volatile bool ccg5_v5v_supply_present[NO_OF_TYPEC_PORTS] = {
    true
#if CCG_PD_DUALPORT_ENABLE
    ,
    true
#endif /* CCG_PD_DUALPORT_ENABLE */
};
#endif /* CCGX_V5V_CHANGE_DETECT */

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

/* Callback used for notification of any input supply change. */
pd_supply_change_cbk_t gl_ccg_supply_changed_cb = NULL;

/**
 * @struct pdss_status_t
 * @brief Structure to hold PDSS IP status.
 */
typedef struct
{
    /** PD phy callback. */
    pd_phy_cbk_t pd_phy_cbk;

    /** The received PD packet. */
    pd_packet_extd_t rx_pkt;

    /** The tx data pointer. */
    uint32_t* tx_dat_ptr;

    /** ADC block variables. */
    volatile uint16_t adc_vddd_mv[PD_ADC_NUM_ADC];

    /** ADC callback. */
    PD_ADC_CB_T adc_cb[PD_ADC_NUM_ADC];

    /** The tx data count. */
    uint8_t tx_dobj_count;

    /** The tx data pointer. */
    uint8_t volatile tx_obj_sent;

    /* Holds current transmission is unchunked or not */
    uint8_t volatile tx_unchunked;

    /** Holds retry count. */
    int8_t volatile retry_cnt;

    /**
     * Flag to indicate a message has been transmitted and we're waiting for
     * GoodCRC.
     */
    uint8_t volatile tx_done;

    /**
     * Flag to indicate currently received message is unchunked extended message
     */
    bool volatile rx_unchunked;

    /**
     * Length of currently being received extended unchunked messages in 32 bits units
     */
    uint8_t volatile rx_unchunk_len;

    /**
     * Count in 32 bits units of no of words read from rx memory for extended unchunked
     * message.
     */
    uint8_t volatile rx_unchunk_count;

    /**
     * Read memory location where the HAL should read the next portion of data from.
     */
    uint8_t volatile rx_read_location;
/** BC phy callback. */
bc_phy_cbk_t bc_phy_cbk;
#if BATTERY_CHARGING_ENABLE
    /** BC phy callback. */
    bc_phy_cbk_t bc_phy_cbk;

#if (!QC_AFC_CHARGING_DISABLED)
    /** QC3.0 net pulse count (+ve = D+ pulse, -ve = D- pulse).*/
    int volatile bc_qc_pulse_count;

    /** AFC TX buffer */
    uint8_t bc_afc_tx_buf[16];

    /** AFC RX buffer */
    uint8_t bc_afc_rx_buf[16];

    /** AFC RX buffer index */
    uint8_t bc_afc_rx_idx;

    /** AFC TX buffer index */
    uint8_t bc_afc_tx_idx;

    /** AFC TX buffer size */
    uint8_t bc_afc_tx_size;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

#endif /* BATTERY_CHARGING_ENABLE */

#if VBUS_CF_EN
    uint32_t cf_cur;
    vbus_cf_cbk_t cf_cbk;
#endif /* VBUS_CF_EN */

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd_cmp_cbk_t sr_cmp_cbk;
    pd_cmp_cbk_t pfc_cmp_cbk;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    /** Auto toggle enable from stack. */
    uint8_t volatile auto_toggle_en;

    /** Auto toggle active state. */
    uint8_t volatile auto_toggle_act;

    /** Type-C state machine re-enable pending. */
    uint8_t volatile typec_start_pending;

    /** OVP fault pending on CC line. */
    uint8_t volatile cc_ovp_pending;

} pdss_status_t;

#ifdef CCG5
/** Pointer array for HW IP register structure. */
static PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] =
{
    PDSS0
#if CCG_PD_DUALPORT_ENABLE
    ,
    PDSS1
#endif /* CCG_PD_DUALPORT_ENABLE */
};

#else /* CCG3PA, CCG3PA2, CCG5C, CCG6 */

#if (CCG_TYPE_A_PORT_ENABLE == 0)
static PPDSS_REGS_T gl_pdss[NO_OF_TYPEC_PORTS] = {PDSS};
#else /* (CCG_TYPE_A_PORT_ENABLE) */
/*
 * This is a hack to allow CCG3PA/CCG3PA2 devices to support additional
 * type-A port on the same PD block. This allows the code to use the same
 * gl_pdss array to get to the port (simplifies the code) which can now be
 * re-used across CCG5, CCG6.
 */
static PPDSS_REGS_T gl_pdss[2] = {PDSS, PDSS};
#endif /* (CCG_TYPE_A_PORT_ENABLE) */
#endif /* CCGx */

#if ((BATTERY_CHARGING_ENABLE) && ((defined CCG5C) || (defined CCG6)))
/* On CCG5C and CCG6, we use TCPWM blocks to detect disconnect of BC 1.2 sink device. */
static PCNT_REGS_T gl_cnt[2] =
{
     CNT0,
     CNT1
};
#endif /* ((BATTERY_CHARGING_ENABLE) && ((defined CCG5C) || (defined CCG6))) */

/** PDSS status. */
#if (CCG_TYPE_A_PORT_ENABLE == 0)
static pdss_status_t gl_pdss_status[NO_OF_TYPEC_PORTS];
#else
static pdss_status_t gl_pdss_status[2];
#endif /* (CCG_TYPE_A_PORT_ENABLE == 0) */

void pdss_intr0_handler(uint8_t port);
CY_ISR_PROTO(pdss_port0_intr0_handler);

void pdss_intr1_handler(uint8_t port);
CY_ISR_PROTO(pdss_port0_intr1_handler);

#if CCG_PD_DUALPORT_ENABLE
CY_ISR_PROTO(pdss_port1_intr0_handler);
CY_ISR_PROTO(pdss_port1_intr1_handler);
#endif /* CCG_PD_DUALPORT_ENABLE */

/* HPD transmit enable per PD port. */
#if (CCG_PD_DUALPORT_ENABLE == 0)
    static bool hpd_transmit_enable[NO_OF_TYPEC_PORTS] =
#else
    static bool hpd_transmit_enable[2] =
#endif /* (CCG_PD_DUALPORT_ENABLE == 0) */
{
    false
#if CCG_PD_DUALPORT_ENABLE
        ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* HPD receive enable per PD port. */
#if (CCG_PD_DUALPORT_ENABLE == 0)
    static bool hpd_receive_enable[NO_OF_TYPEC_PORTS] =
#else
    static bool hpd_receive_enable[2] =
#endif /* (CCG_PD_DUALPORT_ENABLE == 0) */
{
    false
#if CCG_PD_DUALPORT_ENABLE
        ,
    false
#endif
};

/* HPD event callback per PD port. */
#if (CCG_PD_DUALPORT_ENABLE == 0)
    static hpd_event_cbk_t hpd_cbks[NO_OF_TYPEC_PORTS] =
#else
    static hpd_event_cbk_t hpd_cbks[2] =
#endif /* (CCG_PD_DUALPORT_ENABLE == 0) */
{
    NULL
#if CCG_PD_DUALPORT_ENABLE
        ,
    NULL
#endif /* CCG_PD_DUALPORT_ENABLE */
};

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5) || defined(CCG6) || defined(CCG5C))
/*
 * Choose fractional (0.5) divider support.
 * This is required when we are using 8% of VBus for ADC measurement.
 */
#define CCG_AMUX_DIV_2X_ENABLE                  (1u)
#endif /* (defined(CCG5) || defined(CCG3PA) || defined(CCG3PA2)) */

/* Configuration provided from solution. */
#if (defined(CCG3PA) || defined(CCG3PA2))
static PD_ADC_INPUT_T   pd_vbus_detach_adc_input    = PD_ADC_INPUT_AMUX_B;
static PD_ADC_ID_T      pd_vbus_detach_adc_id       = PD_ADC_ID_1;
#else
static PD_ADC_INPUT_T   pd_vbus_detach_adc_input    = PD_ADC_INPUT_AMUX_A;
static PD_ADC_ID_T      pd_vbus_detach_adc_id       = PD_ADC_ID_1;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

static bool             pd_vbus_mon_internal        = false;

#if CCG_AMUX_DIV_2X_ENABLE
#define VBUS_MON_DIV_20P_VAL                    (10u)
#define VBUS_MON_DIV_8P_VAL                     (25u)
#else /* !CCG_AMUX_DIV_2X_ENABLE */
#define VBUS_MON_DIV_20P_VAL                    (5u)
#define VBUS_MON_DIV_8P_VAL                     (12u)
#endif /* CCG_AMUX_DIV_2X_ENABLE */

static uint8_t          pd_vbus_mon_divider         = VBUS_MON_DIV_8P_VAL;

typedef enum
{
    SWAPR_SRC_ADC1,             /* 0 */
    SWAPR_SRC_ADC2,
    SWAPR_SRC_VBUS_MON,
#if (defined(CCG3PA) || defined(CCG3PA2))
    SWAPR_SRC_VSRC_NEW_P,       /* 3 */
    SWAPR_SRC_VSRC_NEW_M,
    SWAPR_SRC_UV,               /* 5 */
    SWAPR_SRC_OV,
    SWAPR_SRC_DISCHG,
    SWAPR_SRC_SCP,
    SWAPR_SRC_OCP,
    SWAPR_SRC_SCP2,             /* 10 */
    SWAPR_SRC_SR
#elif (defined(CCG5) || defined(CCG6) || defined(CCG5C))
    SWAPR_SRC_VSYS_DET,         /* 3 - Only for port 0. */
    SWAPR_SRC_UV = 5,
    SWAPR_SRC_OV,
    SWAPR_SRC_OCP = 9
#endif /* CCGx */
} swapr_src_t;

/* The FRS comparator threshold should be set to 0.52V = 2. */
#define CMP_FS_VSEL_VALUE       (2)

/*
 * Swap CTRL default settings for FRS receive. This settings are based on 5Mhz clock
 * to the block.
 */
#define FRS_RX_SWAP_CTRL1_DFLT_VAL      ((175u << PDSS_SWAP_CTRL1_PULSE_MIN_POS)| \
                                         (650u << PDSS_SWAP_CTRL1_PULSE_MAX_POS)|\
                                         (PDSS_SWAP_CTRL1_RESET_SWAP_STATE))

#define FRS_RX_SWAP_CTRL2_DFLT_VAL      ((50u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_LOW_POS) | \
                                         (1u << PDSS_SWAP_CTRL2_GLITCH_WIDTH_HIGH_POS))

#define FRS_RX_SWAP_CTRL3_DFLT_VAL      (160u << PDSS_SWAP_CTRL3_STABLE_LOW_POS)

#define FRS_RX_SWAP_CTRL5_DFLT_VAL      (750u << PDSS_SWAP_CTRL5_LONG_LOW_POS)

/* AUTO_MODE control mask for PGDO_1_CFG and PGDO_PU_1_CFG registers. */
#define PGDO_1_CFG_AUTO_SEL_MASK       (PDSS_PGDO_1_CFG_SEL_CSA_OC |\
                                        PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_FILT2_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_CC1_OCP |\
                                        PDSS_PGDO_1_CFG_SEL_CC2_OCP |\
                                        PDSS_PGDO_1_CFG_SEL_CC1_OVP |\
                                        PDSS_PGDO_1_CFG_SEL_CC2_OVP |\
                                        PDSS_PGDO_1_CFG_SEL_SBU1_OVP_MASK |\
                                        PDSS_PGDO_1_CFG_SEL_SBU2_OVP_MASK)

#define PGDO_PU_1_CFG_AUTO_SEL_MASK     (PGDO_1_CFG_AUTO_SEL_MASK)

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))

/*
 * Trim bits for CC Pull-up current source
 * Firmware must read SFlash and set this value for each Rp value
 *
 * 0FFF_F536 : Port 0 - Default
 * 0FFF_F537 : Port 0 - 1p5
 * 0FFF_F538 : Port 0 - 3p0
 * 0FFF_F542 : Port 1 - Default
 * 0FFF_F543 : Port 1 - 1p5
 * 0FFF_F544 : Port 1 - 3p0
 */
#define SFLASH_PDSS_PORT0_TRIM1_BASE_REV1       (0x0FFFF536u)
#define SFLASH_PDSS_PORT1_TRIM1_BASE_REV1       (0x0FFFF542u)

static const uint8_t *pdss_rp_trim_db_0 = (const uint8_t *)SFLASH_PDSS_PORT0_TRIM1_BASE_REV1;
#if CCG_PD_DUALPORT_ENABLE
static const uint8_t *pdss_rp_trim_db_1 = (const uint8_t *)SFLASH_PDSS_PORT1_TRIM1_BASE_REV1;
#endif /* CCG_PD_DUALPORT_ENABLE */

#endif /* (defined(CCG5) || defined(CCG6) || defined(CCG5C)) */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6))

/* Timer callback used to strongly enable the gate driver. */
static void soft_start_timer_cb (uint8_t port, timer_id_t id)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    (void)port;
    (void)id;

    if (gl_isnk_counter_hold[port])
    {
        pd->pgdo_pd_isnk_cfg |= PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
        gl_isnk_counter_hold[port] = false;
        return;
    }

    /*
     * While VBus stays below 4.5 V, keep increasing the current sink strength slowly every ms.
     * Once VBus reaches 4.5 V, keep the current sink strength at that level for 5 ms and then do a strong
     * pull-down.
     */
    if (pd_hal_measure_vbus(port) < 4500)
    {
        timer_start(port, APP_FET_SOFT_START_TIMER_ID, 1, soft_start_timer_cb);
    }
    else
    {
        timer_start(port, APP_FET_SOFT_START_TIMER_ID, 5, soft_start_timer_cb);
        gl_isnk_counter_hold[port] = true;
    }

    gl_isnk_counter_value[port] += 2;

#if (defined(CCG6))
    pd->pgdo_pd_isnk_cfg = (
            ((gl_isnk_counter_value[port] & PGDO_PD_ISINK_TERMINAL_VAL) << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            (((gl_isnk_counter_value[port] >> 6) & PGDO_PD_ISINK_TERMINAL_VAL_1) << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS) |
            PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK
            );
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd->pgdo_pd_isnk_cfg = (
            ((gl_isnk_counter_value[port] & PGDO_PD_ISINK_TERMINAL_VAL) << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK
            );
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
}

void pd_internal_pfet_soft_start_on (uint8_t port, uint8_t fet_status)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval = 0;

    if (fet_status == true)
    {
        /* If the FET is already ON, just do a strong enable. */
#if (defined(CCG6))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            (PGDO_PD_ISINK_TERMINAL_VAL_1 << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS);
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

        regval &= ~PDSS_PGDO_PD_ISNK_CFG_GO_ISNK;
        regval |= PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK | PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
    }
    else
    {
        /* Clear the ISNK counter first. */
        pd->pgdo_pd_isnk_cfg = 0;
        CyDelayUs (50);
        pd->pgdo_pd_isnk_cfg = PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE;
        CyDelayUs (100);
        while ((pd->pgdo_pd_isnk_cfg & PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE) != 0);

        gl_isnk_counter_value[port] = ISNK_COUNTER_STARTING_VALUE;
        gl_isnk_counter_hold[port]  = false;

#if (defined(CCG6))
        regval = (
                ((gl_isnk_counter_value[port] & PGDO_PD_ISINK_TERMINAL_VAL) << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
                (((gl_isnk_counter_value[port] >> 6) & PGDO_PD_ISINK_TERMINAL_VAL_1) << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS) |
                PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK
                );
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
        regval = (
                ((gl_isnk_counter_value[port] & PGDO_PD_ISINK_TERMINAL_VAL) << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
                PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK
                );
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

        /* Start a timer which will measure VBus and update the current sink strength. */
        timer_start (port, APP_FET_SOFT_START_TIMER_ID, 1, soft_start_timer_cb);
    }

    pd->pgdo_pd_isnk_cfg = regval;
}

void pd_internal_pfet_soft_start_off (uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

#if (defined(CCG6))
    pd->pgdo_pd_isnk_cfg = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
        (PGDO_PD_ISINK_TERMINAL_VAL_1 << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS) | PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK;

    timer_stop(port, APP_FET_SOFT_START_TIMER_ID);
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd->pgdo_pd_isnk_cfg = PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
}

void  pd_internal_cfet_soft_start_on (uint8_t port, uint8_t fet_status)
{
    uint32_t regval = 0;
    PPDSS_REGS_T pd = gl_pdss[port];

    if (fet_status == true)
    {
        /* If the FET is already ON, just do a strong enable. */
#if (defined(CCG6))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            (PGDO_PD_ISINK_TERMINAL_VAL_1 << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS);
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

        regval &= ~PDSS_PGDO_PD_ISNK_CFG_GO_ISNK;
        regval |= PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK | PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
    }
    else
    {
        /* Clear the ISNK counter first. */
        pd->pgdo_pd_isnk_cfg = 0;
        CyDelayUs (50);
        pd->pgdo_pd_isnk_cfg = PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE;
        CyDelayUs (100);
        while ((pd->pgdo_pd_isnk_cfg & PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE) != 0);

        /* Configure PGDO_PD_ISINK for soft start. FET will be turned ON slowly for 5 ms,
         * and then enabled for hard turn-on. */
#if (defined(CCG6))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            (PGDO_PD_ISINK_TERMINAL_VAL_1 << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS) |
            PDSS_PGDO_PD_ISNK_CFG_GO_ISNK;
#endif /* (defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))
        regval = (PGDO_PD_ISINK_TERMINAL_VAL << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
            PDSS_PGDO_PD_ISNK_CFG_GO_ISNK;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

        gl_isnk_counter_hold[port] = true;
        timer_start (port, APP_FET_SOFT_START_TIMER_ID, 15, soft_start_timer_cb);
    }

    pd->pgdo_pd_isnk_cfg = regval;
}

void  pd_internal_cfet_soft_start_off (uint8_t port)
{
    timer_stop (port, APP_FET_SOFT_START_TIMER_ID);
    gl_isnk_counter_hold[port] = false;
}

#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2))

/*
 * Trim bits for CC Pull-up current source
 * Correct SFLASH addresses for CCG3PA and CCG3PA2 silicon.
 * Firmware must read SFlash and set this value for each Rp value
 *
 * 0FFF_F288 : Port 0 - Default
 * 0FFF_F289 : Port 0 - 1p5
 * 0FFF_F28A : Port 0 - 3p0
 */
#define SFLASH_PDSS_PORT0_TRIM1_BASE            (0x0FFFF288u)

static const uint8_t *pdss_rp_trim_db_0 = (const uint8_t *)SFLASH_PDSS_PORT0_TRIM1_BASE;

/* Default RSENSE value is 5mOhm. */
#define LSCSA_DEF_RSENSE        (50)

static uint8_t gl_vbus_csa_rsense = LSCSA_DEF_RSENSE;

void pd_hal_set_vbus_csa_rsense(uint8_t rsense)
{
    gl_vbus_csa_rsense = rsense;
}

uint8_t pd_hal_get_vbus_csa_rsense(void)
{
    return gl_vbus_csa_rsense;
}

#if CCG_CABLE_COMP_ENABLE
/*
 * TODO: Remove this compile time check option as this is required for meeting
 * accuracy.
 */
#define LSCSA_VSENSE_ADJ_ENABLE     (1)

#if LSCSA_VSENSE_ADJ_ENABLE

/*
 * The table contains the error adjusted Vsense value in 10uV units. The table
 * is done for every 0.5mV Vsense and the table entry contains the corresponding
 * Vsense adjusted for error in 100uV units (100X scaled for accuracy). The first
 * 64 entries are error adjusted for AMP1 and second set of 64 entries are error
 * adjusted for AMP2.
 */
uint16_t gl_lscsa_table[128];

/*
 * The function prepares the LSCSA error table for error adjusted VSense value
 * for reverse look-up. This table shall be used by the VBUS current measurement
 * function to match the measured Vsense to actual Vsense and then calculating
 * the current. The function is expected to be invoked only once, at HAL init.
 */
static void lscsa_prep_table(void)
{
    uint8_t i, err_sign;
    uint32_t vsense, err_target;

    vsense = 5;
    for (i = 0; i < 128; i++)
    {
        /*
         * For vsense < 15mV, the spec error is (20 - vsense); for vsense between 15mV
         * and 20mV, the error is 5%; for vsense between 20mV and 30mV, the error is
         * (7 - 0.1 * vsense)%; for vsense between 30mV and 50mV, the error is specd
         * to (5.5 - 0.05*vsense)%.
         *
         * The err_target is converted to 10X value to allow for division of 10 as int.
         * Since the error variation is very small post 30mV, it does not warrant
         * calculating for division with 20.
         */
        if (vsense < 150)
        {
            err_target = (200 - vsense);
        }
        else if (vsense < 200)
        {
            err_target = 50;
        }
        else if (vsense < 300)
        {
            err_target = (70 - (vsense / 10));
        }
        else
        {
            err_target = 55 - (vsense / 20);
        }

        if (i < 64)
        {
            err_target *= (LSCSA_AMP1_ERR_VAL);
            err_sign = LSCSA_AMP1_ERR_SIGN;
        }
        else
        {
            err_target *= (LSCSA_AMP2_ERR_VAL);
            err_sign = LSCSA_AMP2_ERR_SIGN;
        }

        if (err_sign)
        {
            err_target *= -1;
        }

        /*
         * To make the calculations better, the 5 (spec error at 15mV) and the divider
         * for % (100) are taken upto the numerator and divided at a higher value.
         * Also, an additional 10X is done to accomodate for the err_target being
         * made 10X in previous calculation.
         */
        err_target = (((50000 - err_target) * vsense) / 5000);

        gl_lscsa_table[i] = (uint16_t)err_target;

        if (i == 63)
        {
            vsense = 5;
        }
        else
        {
            vsense += 5;
        }
    }
}

/*
 * The function takes the ADC measured OUT_EA value and the gain of the LSCSA EA
 * amplifer and returns the Vsense value in 10uV (100X) units adjusted for LSCSA
 * error.
 * @vref The voltage in mV measured at OUT_EA.
 * @gain The gain of the LSCSA amplifier.
 *
 * @return The Vsense value adjusted for LSCSA error in 10uV (100X) units.
 */
static uint16_t lscsa_vsense_adj(uint32_t vref, uint8_t gain)
{
    uint8_t i, offset;
    uint32_t val;
    uint16_t retval;

    val = (vref * 100) / gain;

    if (gain == 35)
    {
        offset = 0;
    }
    else
    {
        offset = 64;
    }

    for (i = 0; i < 64; i++)
    {
        if (gl_lscsa_table[i + offset] >= val)
        {
            break;
        }
    }

    /*
     * Perform round off operation. Since Vsense in the table is discrete with
     * 0.5mV step size (stored in 100X), we need to adjust for this. Since the
     * LSCSA accuracy is not very off, we can assume the relative Vsense to be
     * accurate. So, the delta from measured to last step in the table can be
     * added together to get the variation within the 0.5mV step size. Since
     * the step for each entry has an additional 10X scale when returning the
     * data, we need to multiply this with 10 for relative increase.
     */
    if (i > 0)
    {
        retval = (i - 1) * 500;
        retval += ((val - gl_lscsa_table[i + offset - 1]) * 10);
    }
    else
    {
        retval = val * 10;
    }

    return retval;
}
#endif /* LSCSA_VSENSE_ADJ_ENABLE */

#endif /* CCG_CABLE_COMP_ENABLE */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))

uint32_t ccg_get_vsys_status(uint8_t port)
{
    return gl_pdss[port]->vreg_vsys_ctrl;
}

/* This control is only supported on CCG5, CCG5C and CCG6. */
void pd_hal_set_cc_ovp_pending(uint8_t port)
{
    gl_pdss_status[port].cc_ovp_pending = true;
}

#endif /* (defined(CCG5) || defined(CCG6) || defined(CCG5C)) */

uint16_t pd_hal_measure_vbus(uint8_t port)
{
    /* Sample the VBus voltage using ADC. */
    uint8_t level;
    uint16_t retval;

    level = pd_adc_sample(port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);
    retval = pd_adc_get_vbus_voltage(port, APP_VBUS_POLL_ADC_ID, level);

#if (defined(CCG3PA) || defined(CCG3PA2))
    /*
     * For low voltages, CCG3PA(2) requires higher accuracy. This can be done
     * by moving to a lower divider value of 20%. Do this only if required.
     */
    if (retval < VBUS_DIV_20_PER_MAX_VOLT)
    {
        uint8_t state;
        PPDSS_REGS_T pd = gl_pdss[port];

        state = CyEnterCriticalSection();

        /* First set the AMUX to use 20% divider. */
        pd->amux_nhv_ctrl |= (1 << AMUX_ADC_CCG3PA_VBUS_DIV_2OP_EN_POS);
        pd_hal_set_vbus_mon_divider(VBUS_MON_DIV_20P_VAL);
        CyDelayUs(10);
        /* Now measure the voltage. */
        level = pd_adc_sample(port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);
        retval = pd_adc_get_vbus_voltage(port, APP_VBUS_POLL_ADC_ID, level);

        /* Restore the divider value. */
        pd->amux_nhv_ctrl &= ~(1 << AMUX_ADC_CCG3PA_VBUS_DIV_2OP_EN_POS);
        pd_hal_set_vbus_mon_divider(VBUS_MON_DIV_8P_VAL);
        CyDelayUs(10);

        /*
         * TODO: Currently ADC interrupt is not enabled when running when
         * sampling the VBUS. If this changes, the code has to be extended to
         * save and restore the status to prevent falsely triggering interrupts.
         */

        CyExitCriticalSection(state);
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    return retval;
}

/* Sample the VBus current using ADC. */
uint16_t pd_hal_measure_vbus_cur(uint8_t port)
{
    uint16_t current = 0;

#if ((defined(CCG5C)) || (defined(CCG6)))
    /* Current measurement is only supported on CCG6 and CCG5C devices. */
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t level;

    /* Disconnect VBus divider from AMUX-B. */
    pd->amux_nhv_ctrl &= ~(1 << AMUX_ADC_CCG5_VBUS_DIV_EN_POS);
    CyDelayUs (50);

    /* Use ADFT to connect the CSA stage-1 output to AMUX Bus B. */
    pd->csa_scp_0_ctrl = ((pd->csa_scp_0_ctrl & ~PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL_MASK) |
            (15 << PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL_POS));
    CyDelayUs (100);

    /* Sample and log the ADC reading. */
    level = pd_adc_sample(port, PD_ADC_ID_0, PD_ADC_INPUT_AMUX_B);

    /* Disconnect the stage-1 CSA output from AMUX. */
    pd->csa_scp_0_ctrl &= ~PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL_MASK;
    CyDelayUs (50);

    /* Reconnect VBus divider to AMUX-B. */
    pd->amux_nhv_ctrl |= (1 << AMUX_ADC_CCG5_VBUS_DIV_EN_POS);

    /* Calculate the amplified sense voltage in mV from the ADC reading. */
    current = ((level * MX_PD_ADC_REF_VOLT_MV) / 256);
    if (pd_get_ptr_ocp_tbl(port)->sense_res == 10)
    {
        /*
           Convert to current in 10 mA units.
           We need to divide by gain (20) times the sense impedance (10 mOhm).
           */
        current = current / 2;
    }
    else
    {
        /*
           Convert to current in 10 mA units.
           We need to divide by gain (50) times the sense impedance (5 mOhm).
           */
        current = (current * 2) / 5;
    }
#endif /* ((defined(CCG5C)) || (defined(CCG6))) */

#if (defined(CCG3PA) || defined(CCG3PA2))
#if CCG_CABLE_COMP_ENABLE
    /* Sample the VBus current using ADC. */
    uint8_t state, gain;
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    state = CyEnterCriticalSection();

    /* Connect the OUT_EA_RC to ADC. */
    pd->amux_nhv_ctrl |= (1 << 15);
    CyDelayUs(10);

    regval = (PDSS->lscsa_0_ctrl >> PDSS_LSCSA_0_CTRL_AV_EA_POS) & 0x1F;
    if (regval == LSCSA_AV_SEL_150)
    {
        gain = 150;
    }
    else if (regval == LSCSA_AV_SEL_125)
    {
        gain = 125;
    }
    else /* LSCSA_AV_SEL_35 */
    {
        gain = 35;
    }

    /* Implement gain calculation. */
    regval = pd_adc_sample(port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);
    regval = pd_adc_level_to_volt(port, PD_ADC_ID_0, regval);

    /* TODO: Evaluate gain saturation in case of non-PPS contract. */

#if LSCSA_VSENSE_ADJ_ENABLE
    regval = lscsa_vsense_adj(regval, gain);
    regval = ((regval) / gl_vbus_csa_rsense);
#else /* !LSCSA_VSENSE_ADJ_ENABLE */
    regval = ((regval * 1000) / gl_vbus_csa_rsense / gain);
#endif /* LSCSA_VSENSE_ADJ_ENABLE */

    /* Revert the AMUX setting to connect to VBUS voltage. */
    pd->amux_nhv_ctrl &= ~(1 << 15);
    CyDelayUs(10);

    CyExitCriticalSection(state);

    current = (uint16_t)regval;
#endif /* CCG_CABLE_COMP_ENABLE */
#endif /* CCGx */

    return current;
}

void pd_hal_config_auto_toggle(uint8_t port, bool enable)
{
    /* Abort auto toggle if it is in progress. */
    pd_hal_abort_auto_toggle(port);
    gl_pdss_status[port].auto_toggle_en = enable;
}

bool pd_hal_is_auto_toggle_active(uint8_t port)
{
    /* Don't run through Type-C state machine if auto toggle is active or has just been stopped. */
    return (((bool)gl_pdss_status[port].auto_toggle_act) || ((bool)gl_pdss_status[port].typec_start_pending));
}

void pd_hal_abort_auto_toggle(uint8_t port)
{
    (void)port;
#if CCG_HW_DRP_TOGGLE_ENABLE
    PPDSS_REGS_T pd = gl_pdss[port];

    if (gl_pdss_status[port].auto_toggle_act)
    {
        /* Disable the RP-RD toggle and the attach interrupt. */
        pd->rp_rd_cfg1 &= ~(PDSS_RP_RD_CFG1_HW_RP_RD_AUTO_TOGGLE | PDSS_RP_RD_CFG1_START_TOGGLE);
        pd->intr1_mask &= ~PDSS_INTR1_DRP_ATTACHED_DETECTED;

        gl_pdss_status[port].auto_toggle_act     = false;
        gl_pdss_status[port].typec_start_pending = false;
    }
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */
}

PD_ADC_ID_T pd_hal_get_vbus_detach_adc(void)
{
    return pd_vbus_detach_adc_id;
}

PD_ADC_INPUT_T pd_hal_get_vbus_detach_input(void)
{
    return pd_vbus_detach_adc_input;
}

void pd_hal_set_vbus_detach_params(PD_ADC_ID_T adc_id, PD_ADC_INPUT_T adc_inp)
{
    pd_vbus_detach_adc_input = adc_inp;
    pd_vbus_detach_adc_id    = adc_id;
}

void pd_hal_set_vbus_mon_divider(uint8_t divider)
{
    pd_vbus_mon_divider = divider;
}

void pd_hal_enable_internal_vbus_mon(bool enable)
{
    pd_vbus_mon_internal = enable;
}

void pd_hal_set_fet_drive(pd_fet_dr_t pctrl_drive, pd_fet_dr_t cctrl_drive)
{
    (void)pctrl_drive;
    (void)cctrl_drive;
}

void pd_hal_dual_fet_config(bool dual_fet, uint8_t spacing)
{
    (void)dual_fet;
    (void)spacing;
}

/* Function definitions. */

CY_ISR(pdss_port0_intr0_handler)
{
    pdss_intr0_handler(TYPEC_PORT_0_IDX);
}

CY_ISR(pdss_port0_intr1_handler)
{
    pdss_intr1_handler(TYPEC_PORT_0_IDX);
}

#if CCG_PD_DUALPORT_ENABLE
CY_ISR(pdss_port1_intr0_handler)
{
    pdss_intr0_handler(TYPEC_PORT_1_IDX);
}

CY_ISR(pdss_port1_intr1_handler)
{
    pdss_intr1_handler(TYPEC_PORT_1_IDX);
}
#endif /* CCG_PD_DUALPORT_ENABLE */

#if (defined(CCG5))

#if CCG_PD_DUALPORT_ENABLE

/* Firmware has to copy trims for second PD block from SFLASH to registers. */

/* MMIO address of PD block trim registers. */
#define CCG5_USBPD1_MMIO_TRIM_ADDR   (0x400bff00)

/* Number of trim registers to be initialized. */
#define CCG5_USBPD_TRIM_REG_CNT      (29u)

/* SFLASH address where the trims are stored. Skip the count and address. */
#define CCG5_SFLASH_PD1_TRIM_ADDR    (0x0FFFF502)

static void pd_hal_init_ccg5_pd1_trims (void)
{
    uint32_t *trim_mmio_p   = (uint32_t *)CCG5_USBPD1_MMIO_TRIM_ADDR;
    uint8_t  *trim_sflash_p = (uint8_t *)CCG5_SFLASH_PD1_TRIM_ADDR;
    uint8_t   val, cnt;

    /* Read the trim values from SFLASH and copy into MMIO registers.
       Each register occupies one byte in SFLASH, but 4 bytes in MMIO.
     */
    for (cnt = 0; cnt < CCG5_USBPD_TRIM_REG_CNT; cnt++)
    {
        val = *trim_sflash_p;
        *trim_mmio_p = (uint32_t)val;

        trim_sflash_p++;
        trim_mmio_p++;
    }
}
#endif /* CCG_PD_DUALPORT_ENABLE */
#endif /* (defined(CCG5)) */

ccg_status_t pd_hal_init(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (port == TYPEC_PORT_0_IDX)
    {
        /* Register sync interrupt handler. */
        CyIntDisable(PD_PORT0_INTR0);
        (void)CyIntSetVector(PD_PORT0_INTR0, &pdss_port0_intr0_handler);
        CyIntEnable(PD_PORT0_INTR0);

        /* Register ganged interrupt handler. */
        CyIntDisable(PD_PORT0_INTR1);
        (void)CyIntSetVector(PD_PORT0_INTR1, &pdss_port0_intr1_handler);
        CyIntEnable(PD_PORT0_INTR1);
    }
#if CCG_PD_DUALPORT_ENABLE
    else
    {
        /* Register sync interrupt handler. */
        CyIntDisable(PD_PORT1_INTR0);
        (void)CyIntSetVector(PD_PORT1_INTR0, &pdss_port1_intr0_handler);
        CyIntEnable(PD_PORT1_INTR0);

        /* Register ganged interrupt handler. */
        CyIntDisable(PD_PORT1_INTR1);
        (void)CyIntSetVector(PD_PORT1_INTR1, &pdss_port1_intr1_handler);
        CyIntEnable(PD_PORT1_INTR1);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Enable the PD block. */
    pd->ctrl |= PDSS_CTRL_IP_ENABLED;

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Turn off PHY deepsleep. References require 100us to stabilize. */
    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;

    /*
     * Enable deep-sleep current reference outputs.
     */
    pd->dpslp_ref_ctrl |= PDSS_DPSLP_REF_CTRL_IGEN_EN;
    CyDelayUs(100);

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))
    /*
     * CCG5/CCG5C/CCG6:
     * The Deep Sleep voltage reference is used as input to the refgen block at all times.
     */
    pd->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    pd->refgen_1_ctrl  = 0x1C143616; /* SEL0=0x16(420 mV), SEL1=0x36(740 mV), SEL2=0x14(400 mV), SEL3=0x1C(480 mV) */
    pd->refgen_2_ctrl  = 0x003D6B5F; /* SEL4=0x5F(1.15 V), SEL5=0x6B(1.27 V), SEL6=0x3D(810 mV), SEL7=0x00(200 mV) */
    pd->refgen_3_ctrl  = 0x00003D14; /* SEL8=0x14(400 mV), SEL9=0x3D(810 mV), SEL10=0x00(200 mV) */
    pd->refgen_4_ctrl  = 0x00000900; /* SEL11=0x00(450 mV), SEL12=0x00(650 mV), SEL13=0x04(2.0 V), SEL14=0x04(1.12 V) */

    /* Give some delay for references to settle. */
    CyDelayUs (50);

    /* Enable the VSYS_DET comparator. Corresponding LF filter also needs to be enabled.
       Look for both positive and negative edge interrupts.
     */
    if (port == 0)
    {
#ifdef CCG5
        /* CDT 275510: Convert 5V pump trims from 8 bit to 9 bit. */
        PDSS_TRIMS0->trim_5vpump1_0 = (PDSS_TRIMS0->trim_5vpump1_0 & 0x3F) | ((PDSS_TRIMS0->trim_5vpump1_0 & 0xC0) << 1);
        PDSS_TRIMS0->trim_5vpump2_0 = (PDSS_TRIMS0->trim_5vpump2_0 & 0x3F) | ((PDSS_TRIMS0->trim_5vpump2_0 & 0xC0) << 1);
        PDSS_TRIMS0->trim_5vpump3_0 = (PDSS_TRIMS0->trim_5vpump3_0 & 0x3F) | ((PDSS_TRIMS0->trim_5vpump3_0 & 0xC0) << 1);

        /* Update the trims for VConn OCP to increase the current limit to the maximum. */
        PDSS_TRIMS0->trim_vconn20_2 = (PDSS_TRIMS0->trim_vconn20_2 & 0xF3) | 0x04;
#else
        /* CCG5C and CCG6 */

        /* Set the current reference used for VConn Over-Current detection. */
        pd->reg_control = ((pd->reg_control & ~PDSS_REG_CONTROL_T_REG_CONTROL_MASK) |
                    (2 << PDSS_REG_CONTROL_T_REG_CONTROL_POS));

        if (*((volatile uint8_t *)SFLASH_VCONN_TRIM_ENABLE_ADDR) == 0)
        {
            /* If optimal VConn OCP trims were not available in SFLASH, override with default value. */
            PDSS_TRIMS->trim_vconn20_2 = 0x08;
        }

#endif /* CCG5 */

        /* Enable interrupt on either positive or negative edge from the VSYS_DET comparator. */
        pd->comp_ctrl[COMP_ID_VSYS_DET] |= PDSS_COMP_CTRL_COMP_ISO_N;
        pd->comp_ctrl[COMP_ID_VSYS_DET] &= ~PDSS_COMP_CTRL_COMP_PD;
        pd->intr7_filter_cfg[1] = PDSS_INTR7_FILTER_CFG_FILT_EN | PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_RESET |
            (6 << PDSS_INTR7_FILTER_CFG_CLK_LF_FILT_SEL_POS) |
            (FILTER_CFG_POS_EN_NEG_EN << PDSS_INTR7_FILTER_CFG_FILT_CFG_POS);

#if (defined(CCG6) || defined(CCG5C))
        /* On Rev *A and later, we need to disable the bypass on VSYS good comparator. */
        if (ccg_get_si_revision() != 0)
        {
            pd->reg_control |= PDSS_REG_CONTROL_BYPASS_VSYS_GOOD_ACC;
        }
#endif /* (defined(CCG6) || defined(CCG5C)) */

        /* Wait for some time and check VSYS status. */
        CyDelay (1);
        if ((pd->intr7_status & (2 << PDSS_INTR7_STATUS_FILT_8_POS)) == 0)
        {
            /* Notify application layer about absence of VSYS supply. */
            if (gl_ccg_supply_changed_cb != NULL)
            {
                gl_ccg_supply_changed_cb (port, CCG_SUPPLY_VSYS, false);
            }
            CyDelay (1);

            /* VSYS is not present, disable VSYS switch. */
            pd->vreg_vsys_ctrl &= ~PDSS_VREG_VSYS_CTRL_ENABLE_VDDD_SWITCH;
        }
        else
        {
            /* Notify application about presence of VSYS supply. */
            if (gl_ccg_supply_changed_cb != NULL)
            {
                gl_ccg_supply_changed_cb (port, CCG_SUPPLY_VSYS, true);
            }
        }

        /* Enable interrupt for VSYS detection. */
        pd->intr7_mask |= (2 << PDSS_INTR7_CLK_LF_EDGE_CHANGED_POS);
    }
#if CCG_PD_DUALPORT_ENABLE
    else
    {
        pd_hal_init_ccg5_pd1_trims();

        /* CDT 275510: Convert 5V pump trims from 8 bit to 9 bit. */
        PDSS_TRIMS1->trim_5vpump1_0 = (PDSS_TRIMS1->trim_5vpump1_0 & 0x3F) | ((PDSS_TRIMS1->trim_5vpump1_0 & 0xC0) << 1);
        PDSS_TRIMS1->trim_5vpump2_0 = (PDSS_TRIMS1->trim_5vpump2_0 & 0x3F) | ((PDSS_TRIMS1->trim_5vpump2_0 & 0xC0) << 1);

        /* Update the trims for VConn OCP to increase the current limit to the maximum. */
        PDSS_TRIMS1->trim_vconn20_2 = (PDSS_TRIMS1->trim_vconn20_2 & 0xF3) | 0x04;
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

#endif /* defined(CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Enable HF Clk as source for Fault protection filters. */
    pd->ctrl |= PDSS_CTRL_SEL_CLK_FILTER;

#if(defined(CCG3PA) || defined(CCG3PA2))

#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_OPTO_FB)
    /* Enable shunt */
    pd->ea_ctrl |= PDSS_EA_CTRL_EN_SHNT;
#if VBUS_CF_EN
    /*
     * Constant current mode requires a compensation capacitor connected to
     * the EA block. Since the connection is not directly available, this has
     * to be done by using ADFT. The ADFT allows connecting the node to the
     * AMUX_A of the chip. A dedicated GPIO has to be used for this. The GPIO
     * IO matrix setting has to be updated to connect to AMUX_A. While using
     * this mode, the AMUX_A cannot be used for anything else. All other voltage
     * measurements should be done via AMUX_B.
     */
    PDSS->ea_ctrl |= (7 << PDSS_EA_CTRL_EA_ADFT_POS);
    CALL_IN_FUNCTION(gpio_hsiom_set_config)(APP_VBUS_EA_COMP_IO, HSIOM_MODE_AMUXA,
            GPIO_DM_HIZ_ANALOG, 0);
#endif /* VBUS_CF_EN */
#else /* VBUS_CTRL_TYPE_P1 == VBUS_CTRL_DIR_FB */
    pd->ea_ctrl |= PDSS_EA_CTRL_EN_CC_CMPN;
#endif /* VBUS_CTRL_TYPE_P1 */

    /*
     * Set the GM amplifier source current limit to maximum. This setting is
     * done based on feedback from the design team. Lower values can result
     * in saturation for higher voltage levels.
     */
    PDSS_TRIMS ->trim_ea1_1 = PDSS_TRIM_EA1_1_EA_IPD_CC_TRIM_MASK;

    /* Disable the shunt start opamp. */
    pd->ea_ctrl |= PDSS_EA_CTRL_SHNT_ST_OPAMP_ENB;
#if CCG_FLIPPED_FET_CTRL
    /*
     * In this case (applications like Power bank), VBUS_IN node of silicon is connected
     * to TYPE-C VBUS. Therefore, select VBUS_IN resistor divider to monitor TYPE-C VBUS
     */
    pd->amux_nhv_ctrl |= ((1 << AMUX_ADC_CCG3PA_VBUS_IN_8P_EN_POS) |
            (1 << AMUX_ADC_CCG3PA_VBUS_IN_20P_EN_POS));

#endif /* CCG_FLIPPED_FET_CTRL */

    /*
     * Silicon workaround. There seems to be a floating node for SR comparator
     * voltage reference and needs to be always turned ON and set AMP2 gain to
     * 150. These numbers are provided by the design team.
     */
    pd->lscsa_0_ctrl |= ((12 << PDSS_LSCSA_0_CTRL_AV_SR_ON_POS) |
            PDSS_LSCSA_0_CTRL_AV_SEL_SR_ON);
    if (ccg_get_si_revision() != 0)
    {
        /*
         * The LSCSA block has better accuracy when the chopper logic is enabled.
         * But this can be done only for *A revision of silicon or above.
         * Also load the discharge drive strength values.
         */
        pd->lscsa_0_ctrl |= (PDSS_LSCSA_0_CTRL_OS1_EN | PDSS_LSCSA_0_CTRL_OS2_EN);
        pd->refgen_0_ctrl |= (PDSS_REFGEN_0_CTRL_REFGEN_CLK_SEL);

#if VBUS_SLOW_DISCHARGE_EN
        /*
         * If slow discharge is enabled, we will load the minimumm
         * drive strength for both the dischg_shv_ctrl instances.
         * VBUS_C and VBUS_IN has dependency on CCG_FLIPPED_FET_CTRL value.
         *
         * Note: Slow Discharge control feature is only applicable for *A
         * revision of silicon or above.
         */
#if !CCG_FLIPPED_FET_CTRL
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_C_MIN << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_IN_MIN << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
#else /* !CCG_FLIPPED_FET_CTRL */
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_IN_MIN << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_C_MIN << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
#endif /* CCG_FLIPPED_FET_CTRL */

#else /* VBUS_SLOW_DISCHARGE_EN */
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_C_0A << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_IN_0A << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
#endif /* VBUS_SLOW_DISCHARGE_EN */
    }
    else
    {
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_C_00 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                (CCG3PA_DISCHG_DS_VBUS_IN_00 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
    }

#if CCG_CABLE_COMP_ENABLE
    /* Set the OUT_EA gain to 150 by default. */
    PDSS->lscsa_0_ctrl &= ~(PDSS_LSCSA_0_CTRL_AV_EA_MASK |
            PDSS_LSCSA_0_CTRL_AV_SEL_EA);
    PDSS->lscsa_0_ctrl |= (LSCSA_AV_SEL_35 << PDSS_LSCSA_0_CTRL_AV_EA_POS);
    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;
#if LSCSA_VSENSE_ADJ_ENABLE
    lscsa_prep_table();
#endif /* LSCSA_VSENSE_ADJ_ENABLE */
#endif /* CCG_CABLE_COMP_ENABLE */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))
    /* For CCG5, amux_nhv[4]/amux_nhv[3] should be set for proper connection.
       amux_nhv[3]/amux_nhv[2] is used to select 8% divider instead of 10% divider.
     */
    pd->amux_nhv_ctrl |= (1 << (AMUX_ADC_CCG5_VBUS_DIV_EN_POS - port)) |
        (1 << (AMUX_ADC_CCG5_VBUS_DIV_8PC_POS - port));

    /*
     * Connect 10% of VBUS_C to VBUS_MON comparator input. This has to be done at start to prevent errors on
     * the pct10 feedback lines.
     */
    pd->amux_denfet_ctrl |= PDSS_AMUX_DENFET_CTRL_SEL;
#endif /* defined(CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Initialize ADCs. */
    pd_adc_init(port, PD_ADC_ID_0);

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Only CCG3PA has two ADCs per port. */
    pd_adc_init(port, PD_ADC_ID_1);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if CCG_FLIPPED_FET_CTRL
    /*
     * Fix for CDT 286360: In CCG3PA Power bank design, on power-up, VBUS_P builds up to
     * around 10V. This happens because at init internal FB resistor divider is enabled
     * and VBUS_IN node is floating (or has some unknown voltage due to short to VDDD).
     * This results in a FB voltage which causes VBUS to build upto around 10V.
     * This VBUS_P discharge code ensures VBUS_P is discharged down to atleast 5V. If this is not
     * done and PB turns on provider FET immediately after power up due to connection detect,
     * TYPE_C VBUS can be more than 5V resulting in OV condition.
     * This is not required for PA applications because in those applications VBUS_IN
     * is always shorted to VBUS_P node and hence effective FB voltage is not very
     * much different from expected voltage resulting in ~5V VBUS_P.
     */
    pd_internal_vbus_in_discharge_on (port);
    /*
     * NOTE: This delay can be adjusted if it leads to timing failures.
     * Delay of 10ms if enough for default kit designs. Giving a margin
     * of extra 10ms.
     */
    CyDelay (20);
    pd_internal_vbus_in_discharge_off (port);
#endif /* CCG_FLIPPED_FET_CTRL */

    return CCG_STAT_SUCCESS;
}

void pd_hal_cleanup(uint8_t port)
{
    (void)port;
#if (defined(CCG3PA) || defined(CCG3PA2))
    PDSS->lscsa_1_ctrl |= PDSS_LSCSA_1_CTRL_LSCSA_PD;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
}

void pd_phy_vbus_detach_cbk(uint8_t port, bool comp_out)
{
    /* Do nothing. */
}

/* Function to register callbacks for input supply change. */
void pd_hal_set_supply_change_evt_cb(pd_supply_change_cbk_t cb)
{
    gl_ccg_supply_changed_cb = cb;
}

#if (defined(CCG5) || defined(CCG6) || defined(CCG5C))

#if (defined(CCG6))
/* Function to gradually modify a refgen output voltage so as to prevent perturbations on other outputs. */
void pd_set_refgen_voltage(uint8_t vref_sel, uint8_t vref_sel_pos)
{
    uint8_t vref_old_value = 0;
    uint8_t  pos = 0;
    uint32_t mask = 0xFF;
    volatile uint32_t *reg_addr = &PDSS->refgen_4_ctrl;
    uint8_t state;

    state = CyEnterCriticalSection ();
    switch (vref_sel_pos)
    {
        case 0:
        case 1:
        case 2:
        case 3:
            reg_addr = &(PDSS->refgen_1_ctrl);
            pos      = (vref_sel_pos * 8);
            break;

        case 4:
        case 5:
        case 6:
        case 7:
            reg_addr = &(PDSS->refgen_2_ctrl);
            pos      = (vref_sel_pos - 4) * 8;
            break;

        case 8:
        case 9:
        case 10:
            reg_addr = &(PDSS->refgen_3_ctrl);
            pos      = (vref_sel_pos - 8) * 8;
            break;

        default:
            /* References in REFGEN_4 are three bits wide. */
            pos      = (vref_sel_pos - 12) * 3;
            mask     = 0x07;
            break;
    }

    vref_old_value = (*reg_addr >> pos) & mask;
    if (vref_sel > vref_old_value)
    {
        while ((vref_sel - vref_old_value) > 20)
        {
            vref_old_value += 20;
            *reg_addr  = (*reg_addr & ~(mask << pos)) | ((vref_old_value << pos) & (mask << pos));
            CyDelayUs(10);
        }
    }
    else
    {
        while ((vref_old_value - vref_sel) > 20)
        {
            vref_old_value -= 20;
            *reg_addr  = (*reg_addr & ~(mask << pos)) | ((vref_old_value << pos) & (mask << pos));
            CyDelayUs(10);
        }
    }

    *reg_addr  = (*reg_addr & ~(mask << pos)) | ((vref_sel << pos) & (mask << pos));
    CyExitCriticalSection (state);
}
#endif /* (defined(CCG6)) */

/* Function to register callbacks for CC/SBU fault conditions. */
void ccg_set_fault_cb(PD_ADC_CB_T cb)
{
    gl_ccg_fault_cb = cb;
}

/* Helper function to disable pump on CCG5 device. */
static void ccg5_pump_disable(uint8_t port, uint8_t index)
{
    gl_pdss[port]->pump5v_ctrl[index] |= PDSS_PUMP5V_CTRL_PUMP5V_BYPASS_LV;
    CyDelayUs (10);
    gl_pdss[port]->pump5v_ctrl[index]  = PDSS_PUMP5V_CTRL_DEFAULT;
}

/* Helper function to enable pump on CCG5 device. */
static void ccg5_pump_enable(uint8_t port, uint8_t index)
{
    /* Enable the pump only if it is not already on. */
    if ((gl_pdss[port]->pump5v_ctrl[index] & PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN) == 0)
    {
        gl_pdss[port]->pump5v_ctrl[index] = PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN | PDSS_PUMP5V_CTRL_PUMP5V_BYPASS_LV;
        CyDelayUs (10);
        gl_pdss[port]->pump5v_ctrl[index] = PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN;
        CyDelayUs (40);
        gl_pdss[port]->pump5v_ctrl[index] = PDSS_PUMP5V_CTRL_PUMP5V_PUMP_EN | PDSS_PUMP5V_CTRL_PUMP5V_DEL_PUMP_EN;
    }
}
#endif /* defined(CCG5) || defined(CCG6) || defined(CCG5C) */

void pd_phy_detect_cc_rise(uint8_t port, bool rp_connected)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    const dpm_status_t *dpm_stat = dpm_get_info(port);

    /* Connect UP comparator to CC1 and down comparator to CC2 */
    pd->cc_ctrl_0 = ((pd->cc_ctrl_0 & ~PDSS_CC_CTRL_0_CMP_UP_CC1V2) |
            PDSS_CC_CTRL_0_CMP_DN_CC1V2);

    /* Set the Up comparator on CC1 and the Dn comparator on CC2 to detect rise of CC voltage. */
    pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);
    if (rp_connected)
    {
        pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
            ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
    }

    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_MASK |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS);
    pd->intr1_cfg_vcmp_up_down_ls |= (((PD_ADC_INT_RISING) << PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_POS) |
            ((PD_ADC_INT_RISING) << PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_POS) |
            (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN | PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN));

    CyDelayUs(10);
    pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED | PDSS_INTR1_VCMP_LA_CHANGED);
    pd->intr1_mask |= (PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED);

    /* If comparators have already triggered, then set the interrupts and return. */
    if (pd->intr1_status & (PDSS_INTR1_STATUS_VCMP_UP_STATUS | PDSS_INTR1_STATUS_VCMP_DN_STATUS))
    {
        pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
    }
}

bool pd_phy_deepsleep(uint8_t port)
{
#if SYS_DEEPSLEEP_ENABLE
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
#if (!(CCG_SOURCE_ONLY))
    uint8_t level;
#endif /* (!(CCG_SOURCE_ONLY)) */
    uint32_t volatile status;
    uint32_t regval;

    if (!(dpm_stat->dpm_enabled))
    {
        return true;
    }

    if (dpm_stat->connect == true)
    {
        /* Set LA comparator for wakeup. */
        pd->intr1 = PDSS_INTR1_VCMP_LA_CHANGED;
        pd->intr1_mask |= PDSS_INTR1_VCMP_LA_CHANGED;

        regval = pd->intr1_cfg_vcmp_up_down_ls;

        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            if (dpm_stat->attached_dev == DEV_AUD_ACC)
            {
                pd_phy_detect_cc_rise (port, true);
            }
            else
            {
                regval &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK |
                        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN);
                regval |= (PD_ADC_INT_RISING) << PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_POS;
                pd->intr1_cfg_vcmp_up_down_ls = regval;

                pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED;
                pd->intr1_mask |= PDSS_INTR1_VCMP_UP_CHANGED;

                /* If the comparator has already triggered, set the interrupt and return. */
                if (pd->intr1_status & PDSS_INTR1_STATUS_VCMP_UP_STATUS)
                {
                    pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                }
            }
        }
        else
        {
#if (!(CCG_SOURCE_ONLY))
            regval &= ~ (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN);
            /* Set up/down for both edges */
            regval |=  (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_MASK);
            pd->intr1_cfg_vcmp_up_down_ls = regval;

            pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED;
            pd->intr1_mask |= PDSS_INTR1_VCMP_UP_CHANGED | PDSS_INTR1_VCMP_DN_CHANGED;

            /* Check old cc status if diffrent then */
            if(dpm_stat->cc_old_status.state != pd_typec_get_cc_status(port).state)
            {
                /* Fire anyone interrupt to wakeup */
                pd->intr1_set |= PDSS_INTR1_VCMP_UP_CHANGED;
                return true;
            }

            if(dpm_stat->fr_rx_en_live == false)
            {
                /* Set the VBus detach comparator as per current detach threshold. */
                level = pd_get_vbus_adc_level(port, pd_vbus_detach_adc_id,
                        dpm_get_sink_detach_voltage(port), dpm_get_sink_detach_margin(port));

                /*
                 * The following call will also check if the comparator has
                 * triggered and set the interrupt.
                 * */
                pd_adc_comparator_ctrl(port, pd_vbus_detach_adc_id, pd_vbus_detach_adc_input,
                        level, PD_ADC_INT_RISING, pd_phy_vbus_detach_cbk);
            }
#endif /* (!(CCG_SOURCE_ONLY)) */
        }
    }
    else
    {
        uint8_t cc1_edge = PD_ADC_INT_RISING;
        uint8_t cc2_edge = PD_ADC_INT_RISING;

#if CCG_HW_DRP_TOGGLE_ENABLE
        /* Don't do anything if auto toggle is already running. */
        if (gl_pdss_status[port].auto_toggle_act)
        {
            return true;
        }
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */

        /* Connect UP comparator to CC1 and down comparator to CC2 */
        pd->cc_ctrl_0 = ((pd->cc_ctrl_0 & ~PDSS_CC_CTRL_0_CMP_UP_CC1V2) |
                PDSS_CC_CTRL_0_CMP_DN_CC1V2);

        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            /*
             * If Ra is present on only 1 CC line, then set the Up comparator
             * on the Ra line for a rising edge as per the Rp level. Check if
             * the comparator has already triggered, then set the interrupt and
             * return.
             *
             * Set the Dn comparator on the other line for a falling edge as
             * per the Rp level. Check if the comparator has already triggered,
             * then set the interrupt and return.
             *
             * Otherwise,
             *
             * Set the Up comparator on CC1 for a falling edge as per the Rp
             * level. Set the Dn comparator on CC2 for a falling edge as per
             * the Rp level. If the comparators have already triggered, then
             * set the respective interrupt and return.
             */

            /* Set threshold to Ra level to check if Ra is present on single CC line. */
            pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK);
            pd->cc_ctrl_0 |= ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS) |
                ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS);
            CyDelayUs(10);

            status = pd->intr1_status;

            /* Apply resistor based Rp and remove current source Rp */
            pd->cc_ctrl_1 |= PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
            pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_RP_CC1_EN | PDSS_CC_CTRL_0_RP_CC2_EN);

            if ((status & (PDSS_INTR1_STATUS_VCMP_UP_STATUS | PDSS_INTR1_STATUS_VCMP_DN_STATUS))
                    == PDSS_INTR1_STATUS_VCMP_DN_STATUS)
            {
                cc1_edge = PD_ADC_INT_FALLING;
                cc2_edge = PD_ADC_INT_RISING;
            }
            else if ((status & (PDSS_INTR1_STATUS_VCMP_UP_STATUS | PDSS_INTR1_STATUS_VCMP_DN_STATUS))
                    == PDSS_INTR1_STATUS_VCMP_UP_STATUS)
            {
                cc1_edge = PD_ADC_INT_RISING;
                cc2_edge = PD_ADC_INT_FALLING;
            }

            CyDelayUs(10);

            /* Configure and enable the CC1/CC2 detect filter. */
            pd->intr1_cfg_cc1_cc2_ls &= ~ (PDSS_INTR1_CFG_CC1_CC2_LS_CC1_CFG_MASK |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_CFG_MASK |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_BYPASS |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_BYPASS);
            pd->intr1_cfg_cc1_cc2_ls |= (((cc1_edge) << PDSS_INTR1_CFG_CC1_CC2_LS_CC1_CFG_POS) |
                        ((cc2_edge) << PDSS_INTR1_CFG_CC1_CC2_LS_CC2_CFG_POS) |
                    (PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN | PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN));

            pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED |
                    PDSS_INTR1_VCMP_DN_CHANGED |
                    PDSS_INTR1_VCMP_LA_CHANGED |
                    PDSS_INTR1_CC1_CHANGED |
                    PDSS_INTR1_CC2_CHANGED);
            pd->intr1_mask |= (PDSS_INTR1_CC1_CHANGED | PDSS_INTR1_CC2_CHANGED);
            status = pd->intr1_status;

            /* If the comparators have  already triggered, then set the interrupts and return. */
            if(((cc1_edge == PD_ADC_INT_RISING) && ((status & PDSS_INTR1_STATUS_CC1_STATUS) != 0)) ||
                    ((cc1_edge == PD_ADC_INT_FALLING) && ((status & PDSS_INTR1_STATUS_CC1_STATUS) == 0)) ||
                    ((cc2_edge == PD_ADC_INT_RISING) && ((status & PDSS_INTR1_STATUS_CC2_STATUS) != 0)) ||
                    ((cc2_edge == PD_ADC_INT_FALLING) && ((status & PDSS_INTR1_STATUS_CC2_STATUS) == 0)))
            {
                /* Fire anyone to wakeup*/
                pd->intr1_set |= PDSS_INTR1_CC1_CHANGED;
                return true;
            }
        }
        else
        {
            pd_phy_detect_cc_rise (port, false);

            /* Auto toggle cannot be started in Unattached.SNK state. */
            return true;
        }

#if CCG_HW_DRP_TOGGLE_ENABLE
        /* We can only use auto toggle when we don't already have an Ra connected. */
        if(
                (dpm_stat->toggle == true) &&
                (gl_pdss_status[port].auto_toggle_en != 0) &&
                ((cc1_edge == PD_ADC_INT_RISING) && (cc2_edge == PD_ADC_INT_RISING))
          )
        {
            /* Halt Type-C state machine. */
            timer_stop(port, TYPEC_GENERIC_TIMER1);
            dpm_stat->typec_evt &= ~TYPEC_EVT_TIMEOUT1;

            /* Make sure trimmed Rp/Rd are disabled. */
            pd->cc_ctrl_0 &= 0xFC300000;

            pd->intr1_mask &= ~(PDSS_INTR1_VCMP_UP_CHANGED |
                    PDSS_INTR1_VCMP_DN_CHANGED |
                    PDSS_INTR1_CC1_CHANGED |
                    PDSS_INTR1_CC2_CHANGED |
                    PDSS_INTR1_VCMP_LA_CHANGED);
            pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED |
                    PDSS_INTR1_VCMP_DN_CHANGED |
                    PDSS_INTR1_VCMP_LA_CHANGED |
                    PDSS_INTR1_CC1_CHANGED |
                    PDSS_INTR1_CC2_CHANGED);

            pd->rp_rd_cfg2 &= ~(PDSS_RP_RD_CFG2_OVERRIDE_HW_REF_CTRL |
                                PDSS_RP_RD_CFG2_VCMP_CC_OVERRIDE |
                                PDSS_RP_RD_CFG2_CC1_ATTACH_VALUE |
                                PDSS_RP_RD_CFG2_CC2_ATTACH_VALUE);
            /* Need to use firmware override of references to get expected voltage on the CC line. */
            pd->rp_rd_cfg2 |= PDSS_RP_RD_CFG2_VCMP_CC_OVERRIDE | PDSS_RP_RD_CFG2_OVERRIDE_HW_REF_CTRL;

            pd->rp_rd_cfg1 &= ~(PDSS_RP_RD_CFG1_TOGGLE_PERIOD_MASK |
                    PDSS_RP_RD_CFG1_BASE_HIGH_WIDTH_MASK |
                    PDSS_RP_RD_CFG1_CONTINUE_PREV);

#if (TIMER_TYPE == TIMER_TYPE_WDT)
            /* If WDT timer is being used, we can use a calibrated period setting. */
            pd->rp_rd_cfg1 |= (
                    ((PDSS_DRP_TOGGLE_PERIOD_MS * timer_get_multiplier()) << PDSS_RP_RD_CFG1_TOGGLE_PERIOD_POS) |
                    ((PDSS_DRP_HIGH_PERIOD_MS * timer_get_multiplier()) << PDSS_RP_RD_CFG1_BASE_HIGH_WIDTH_POS) |
                    PDSS_RP_RD_CFG1_HW_RP_RD_AUTO_TOGGLE |
                    PDSS_RP_RD_CFG1_RESET_COUNT
                    );
#else
            /* SYSTICK is used and LF CLK calibration is not done. */
            pd->rp_rd_cfg1 |= (
                    (PDSS_DRP_TOGGLE_PERIOD_VAL << PDSS_RP_RD_CFG1_TOGGLE_PERIOD_POS) |
                    (PDSS_DRP_HIGH_PERIOD_VAL << PDSS_RP_RD_CFG1_BASE_HIGH_WIDTH_POS) |
                    PDSS_RP_RD_CFG1_HW_RP_RD_AUTO_TOGGLE |
                    PDSS_RP_RD_CFG1_RESET_COUNT
                    );
#endif /* TIMER_TYPE */

            while(pd->rp_rd_cfg1 & PDSS_RP_RD_CFG1_RESET_COUNT);

            /* Setting cc1/cc2 filters */
            /* Configure filter clock cycles and disable the filter. */
            pd->intr1_cfg_cc1_cc2_ls &= ~(PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_SEL_MASK |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_SEL_MASK |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_RESET |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_RESET |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_BYPASS |
                    PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_BYPASS);

            pd->intr1_cfg_cc1_cc2_ls |= ((PDSS_CC_FILT_CYCLES << PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_SEL_POS) |
                    (PDSS_CC_FILT_CYCLES << PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_SEL_POS) |
                    (PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN | PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN));

            /*
             * Connect  up comparator to CC1 and down comparator to CC2. Also,
             * Set the up comparator on CC1 and the down comparator on CC2 for
             * rising threshold of 0.2V.
             */
            pd->cc_ctrl_0 = ((pd->cc_ctrl_0 & ~(PDSS_CC_CTRL_0_CMP_UP_CC1V2 |
                    PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK | PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK)) |
                    (PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_CMP_DN_CC1V2));

            /* Setting up/dn filters */
            pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_MASK |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_SEL_MASK |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_RESET |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_RESET |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS |
                    PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS);
            pd->intr1_cfg_vcmp_up_down_ls |= (
                    (PDSS_CC_FILT_CYCLES << PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_POS) |
                    (PDSS_CC_FILT_CYCLES << PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_SEL_POS) |
                    (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
                     PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN));

            /* Auto toggle is active. Prevent other changes from happening. */
            gl_pdss_status[port].auto_toggle_act = true;

            pd->intr1 = PDSS_INTR1_DRP_ATTACHED_DETECTED;
            pd->intr1_mask |= PDSS_INTR1_DRP_ATTACHED_DETECTED;

            pd->rp_rd_cfg1 |= PDSS_RP_RD_CFG1_START_TOGGLE;
        }
        else
        {
            /*
               Leave auto toggle disabled for now. It will get re-enabled when
               the Type-C state machine enters Unattached.SRC again.
             */
            gl_pdss_status[port].auto_toggle_en = false;
        }
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */
    }

    return true;
#else /* !SYS_DEEPSLEEP_ENABLE */
    return false;
#endif /* SYS_DEEPSLEEP_ENABLE */
}

#if SYS_DEEPSLEEP_ENABLE
extern void typec_gen_entry_event(uint8_t port, typec_fsm_state_t next_state);

static void pd_phy_wake_port(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);
    uint32_t regval;

    if (!(dpm_stat->dpm_enabled))
    {
        return;
    }

    /* Disable the deepsleep interrupts. */
    pd->intr1_mask &= ~(PDSS_INTR1_VCMP_UP_CHANGED |
            PDSS_INTR1_VCMP_DN_CHANGED |
            PDSS_INTR1_VCMP_LA_CHANGED |
            PDSS_INTR1_CC1_CHANGED |
            PDSS_INTR1_CC2_CHANGED);
    pd->intr1 = (PDSS_INTR1_VCMP_UP_CHANGED |
            PDSS_INTR1_VCMP_DN_CHANGED |
            PDSS_INTR1_VCMP_LA_CHANGED |
            PDSS_INTR1_CC1_CHANGED |
            PDSS_INTR1_CC2_CHANGED);

#if (!(CCG_SOURCE_ONLY))
    if ((dpm_stat->connect == true) && (dpm_stat->cur_port_role == PRT_ROLE_SINK))
    {
        if(dpm_stat->fr_rx_en_live == false)
        {
            /* Disable the detach detection comparator. */
            pd_adc_comparator_ctrl(port, pd_vbus_detach_adc_id, PD_ADC_INPUT_AMUX_A, 0, PD_ADC_INT_DISABLED, NULL);
        }
    }
#else
    if (0)
    {

    }
#endif /* (!(CCG_SOURCE_ONLY)) */
    else if(dpm_stat->connect == false)
    {
#if (!(CCG_SOURCE_ONLY))
        if (dpm_stat->toggle == true)
        {
#if CCG_HW_DRP_TOGGLE_ENABLE
            if ((gl_pdss_status[port].auto_toggle_en != 0) || (gl_pdss_status[port].typec_start_pending != 0))
            {
                if (gl_pdss_status[port].auto_toggle_act)
                {
                    /*
                       Auto toggle is active and attach has not been detected. Do nothing.
                     */
                    return;
                }

                /* Type-C restart will be done below if required. */
                gl_pdss_status[port].typec_start_pending = false;

                /*
                   Type-C state machine will not be up-to-date when DRP toggle is being done by hardware.
                   We need to jump to the right Type-C state to resume operation.
                 */
                if ((pd->rp_rd_cfg1 & PDSS_RP_RD_CFG1_HW_RP_RD_AUTO_TOGGLE) != 0)
                {
                    if (pd->status & PDSS_STATUS_RP_RD_STATUS)
                    {
                        pd_typec_en_rp(port, CC_CHANNEL_1, (rp_term_t)dpm_stat->src_cur_level_live);
                        pd_typec_en_rp(port, CC_CHANNEL_2, (rp_term_t)dpm_stat->src_cur_level_live);

                        /* Disable the deep sleep Rp. */
                        pd->cc_ctrl_1 &= ~PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;

                        dpm_stat->cur_port_role = PRT_ROLE_SOURCE;
                        dpm_stat->cur_port_type = PRT_TYPE_DFP;
                        gl_dpm_port_type[port]  = PRT_TYPE_DFP;
                        typec_gen_entry_event(port, TYPEC_FSM_UNATTACHED_SRC);
                    }
                    else
                    {
                        pd_typec_en_rd(port, CC_CHANNEL_1);
                        pd_typec_en_rd(port, CC_CHANNEL_2);

                        dpm_stat->cur_port_role = PRT_ROLE_SINK;
                        dpm_stat->cur_port_type = PRT_TYPE_UFP;
                        gl_dpm_port_type[port]  = PRT_TYPE_UFP;
                        typec_gen_entry_event(port, TYPEC_FSM_UNATTACHED_SNK);
                    }

                    /* Disable the RP-RD toggle. Rp or Rd needs to be applied before this. */
                    pd->rp_rd_cfg1 &= ~(PDSS_RP_RD_CFG1_HW_RP_RD_AUTO_TOGGLE | PDSS_RP_RD_CFG1_START_TOGGLE);
                    pd->intr1_mask &= ~PDSS_INTR1_DRP_ATTACHED_DETECTED;

                    /* Leave auto toggle disabled until enabled again by Type-C module. */
                    gl_pdss_status[port].auto_toggle_en = false;

                    /* typec_gen_entry_event disables scanning so reenable it */
                    dpm_stat->skip_scan = false;

                    /* Type-C start is required to enable RX. */
                    pd_typec_start(port);
                }
            }
#endif /* CCG_HW_DRP_TOGGLE_ENABLE */
        }
#endif /* (!(CCG_SOURCE_ONLY)) */

        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            /* Re-enable the trimmed Rp. */
            pd_typec_en_rp(port, CC_CHANNEL_1, (rp_term_t)dpm_stat->src_cur_level_live);
            pd_typec_en_rp(port, CC_CHANNEL_2, (rp_term_t)dpm_stat->src_cur_level_live);
        }

        /* Disable the deep sleep Rp. */
        pd->cc_ctrl_1 &= ~PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;

        /* Disable cc1/cc2/up/down filters */
        regval = pd->intr1_cfg_cc1_cc2_ls;
        regval &= ~(PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN |
                PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN);
        regval |= PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_BYPASS |
            PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_BYPASS;
        pd->intr1_cfg_cc1_cc2_ls = regval;


        regval = pd->intr1_cfg_vcmp_up_down_ls;
        regval &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
                PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN);
        regval |= (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS |
                PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS);
        pd->intr1_cfg_vcmp_up_down_ls = regval;

    }
}
#endif /* SYS_DEEPSLEEP_ENABLE */

void pd_hal_typec_sm_restart(uint8_t port)
{
#if SYS_DEEPSLEEP_ENABLE
    if (gl_pdss_status[port].typec_start_pending)
    {
        pd_phy_wake_port(port);
    }
#endif /* SYS_DEEPSLEEP_ENABLE */
}

bool pd_phy_wakeup(void)
{
#if SYS_DEEPSLEEP_ENABLE
    uint8_t port;

    for (port = 0; port < NO_OF_TYPEC_PORTS; port++)
    {
        pd_phy_wake_port(port);
    }
#endif /* SYS_DEEPSLEEP_ENABLE */

    return true;
}
static uint32_t pd_hal_mmio_reg_update_field(uint32_t origval, uint32_t fld, uint32_t mask, uint8_t pos)
{
    return ((origval & ~mask) | (fld << pos));
}
ccg_status_t pd_phy_init(uint8_t port, pd_phy_cbk_t cbk)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    pdss_stat->pd_phy_cbk = cbk;

#if CCG_PD_REV3_ENABLE
    /* Configure RX_SOP_GOOD_CRC_EN_CTRL */
    pd->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_REV3_CFG;
    /* Configure Extended Header Info register */
    pd->header_info = HEADER_INFO_CFG;
#else
    /* Configure RX_SOP_GOOD_CRC_EN_CTRL */
    pd->rx_sop_good_crc_en_ctrl = RX_SOP_GOOD_CRC_EN_CTRL_CFG;
#endif /* CCG_PD_REV3_ENABLE */

    /* Configure RX_CC reg */
    pd->rx_cc_0_cfg = RX_CC_CFG;

/***GRL-Edit*/
#ifndef  ONLY_PD_SNK_FUNC_EN
    /* Configure RX_ORDER_SET_CTRL */
    pd->rx_order_set_ctrl = RX_ORDER_SET_CTRL_CFG;
    pd->rx_order_set_ctrl |= EN_PRIME_SOP_DET_VAL; /**GRL-Edit*/
#else

    /**As per cypress input */
    /* Configure RX_ORDER_SET_CTRL pd->rx_order_set_ctrl|= EN_PRIME_SOP_DET_VAL;*/
    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
    {
        pd->rx_order_set_ctrl = RX_ORDER_SET_CTRL_CFG;
        pd->rx_order_set_ctrl |= EN_PRIME_SOP_DET_VAL;  // Include this line
        pd->rx_order_set_ctrl |= EN_DBL_SOP_DET_VAL;
    }
    else
        pd->rx_order_set_ctrl = RX_ORDER_SET_CTRL_CFG;
#endif    
    /* Configure CRC_COUNTER reg */
    pd->crc_counter = CRC_COUNTER_CFG;

    /* Configure INTER_PACKET_COUNTER reg */
    pd->inter_packet_counter = INTER_PACKET_COUNTER_CFG;

    /* Disable all PD interrupts */
    pd->intr0_mask &= ~PD_INTR_MASK;

    /* Configure DEBUG_CC_2 reg to disable cc monitoring during idle gap before
     * transmitting goodcrc and set expected goodrc message header mask */
    pd->debug_cc_2 = PDSS_DEBUG_CC_2_DIS_CC_MON_AUTO_CRC | pd_hal_mmio_reg_update_field (pd->debug_cc_2,
            EXPECTED_GOOD_CRC_HDR_MASK, PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_MASK,
            PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS);

#ifdef ONLY_PD_SNK_FUNC_EN
    /* Configure SOP_PRIME and SOP_DPRIME Auto Goodrc Header */
    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
        pd->tx_goodcrc_msg_order_set = (TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT|(1<<8)|(1<<24));
    else
        pd->tx_goodcrc_msg_order_set = TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT;
#else

    pd->debug_cc_2 &= ~PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_MASK;
    pd->debug_cc_2 |= (EXPECTED_GOOD_CRC_HDR_MASK << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS) |
                       PDSS_DEBUG_CC_2_DIS_CC_MON_AUTO_CRC;

    /* Configure SOP_PRIME and SOP_DPRIME Auto Goodrc Header */
    pd->tx_goodcrc_msg_order_set = TX_SOP_PRIME_DPRIME_GD_CRC_HDR_DFLT;
#endif

    return CCG_STAT_SUCCESS;
}

void pd_phy_refresh_roles(uint8_t port)
{
#if CCG_PD_REV3_ENABLE
    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);
    uint32_t temp;
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    temp = pd->rx_order_set_ctrl;
    temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
    temp |= (EN_DEFAULT_SOP_DET_VAL | EN_RX_HARD_RESET_DET_VAL );

    if (dpm_stat->spec_rev_sop_live >= PD_REV3)
    {
        /* Update goodcrc mask */
        pd->debug_cc_2 |= (EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3 << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS);

        /* Enable Extended RX */
        pd->header_info |= PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA;
    }
    else
    {
        /* Update goodcrc mask */
        pd->debug_cc_2 &= ~(EXPECTED_GOOD_CRC_HDR_DIFF_MASK_REV3 << PDSS_DEBUG_CC_2_EXPECTED_HEADER_MASK_POS);

        /* Disable Extended RX/TX */
        pd->header_info &= ~(PDSS_HEADER_INFO_EN_RX_EXTENDED_DATA | PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA);
    }

    /* Start off with cable communication disallowed. */
    temp &= ~(EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);

    /* If cable discovery is disabled, never allow SOP'/SOP'' communication. */
    if (dpm_stat->cbl_dsc != false)
    {
        if (dpm_stat->contract_exist == false)
        {
            /*
             * While in implicit contract, only source can talk to cable (SOP' only allowed).
             * Also, only VConn Source is allowed when the spec revision is PD 3.0.
             */
            if (
                    (dpm_stat->cur_port_role == PRT_ROLE_SOURCE) &&
                    ((dpm_stat->spec_rev_sop_live < PD_REV3) || (dpm_stat->vconn_logical != false))
               )
            {
                /* Enable SOP_PRIME only */
                temp |= EN_PRIME_SOP_DET_VAL;
            }
        }
        else
        {
            /*
             * Only VConn Source can talk to cable during a PD REV3 contract.
             * Only DFP can talk to cable during a PD 2.0 contract.
             */
            if (
                    ((dpm_stat->spec_rev_sop_live >= PD_REV3) && (dpm_stat->vconn_logical != false)) ||
                    ((dpm_stat->spec_rev_sop_live < PD_REV3) && (dpm_stat->cur_port_type == PRT_TYPE_DFP))
               )
            {
                /* Enable SOP_PRIME GoodCRC. */
                temp |= EN_PRIME_SOP_DET_VAL;

                /* If cable has been discovered, enabled SOP_DPRIME GoodCRC. */
                if (dpm_stat->emca_present)
                {
                    temp |= EN_DBL_SOP_DET_VAL;
                }
            }
        }
    }
#ifdef ONLY_PD_SNK_FUNC_EN
    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
    {
        temp |= EN_PRIME_SOP_DET_VAL;  //grl edit
        temp |= EN_DBL_SOP_DET_VAL;  
    }
#endif    
    pd->rx_order_set_ctrl = temp; 

    temp = pd->tx_ctrl;
    temp &= ~PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK;
    temp |= ( CTRL_MSG_GOOD_CRC | PD_DR_PR_ROLE(dpm_stat->cur_port_type, dpm_stat->cur_port_role));
    temp |= (PD_REV2 << PD_REV_POS);
    pd->tx_ctrl = temp;
    
#ifdef ONLY_PD_SNK_FUNC_EN    
    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
    {
        /* Configure SOP_PRIME and SOP_DPRIME Auto Goodrc Header */
        temp = CTRL_MSG_GOOD_CRC;
        temp |= (PD_REV2 << PD_REV_POS);
        pd->tx_goodcrc_msg_order_set = ((temp << 16)| temp |(1u<<8)|(1u<<24));
    }
    else
        pd->tx_goodcrc_msg_order_set = (temp << 16)| temp;
#else
    pd->tx_goodcrc_msg_order_set = (temp << 16)| temp;
#endif

#else

    PPDSS_REGS_T pd = gl_pdss[port];
    dpm_status_t* dpm_stat = dpm_get_status(port);
    uint32_t temp;

    temp = pd->rx_order_set_ctrl;
    temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
    temp |= (EN_DEFAULT_SOP_DET_VAL | EN_RX_HARD_RESET_DET_VAL);

    /* Start off with cable communication disallowed. */
    temp &= ~(EN_PRIME_SOP_DET_VAL | EN_DBL_SOP_DET_VAL);

    /* If cable discovery is disabled, never allow SOP'/SOP'' communication. */
    if (dpm_stat->cbl_dsc != false)
    {
        if (dpm_stat->contract_exist == false)
        {
            /* While in implicit contract, only source can talk to cable (SOP' only allowed). */
            if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
            {
                /* Enable SOP_PRIME only */
                temp |= EN_PRIME_SOP_DET_VAL;
            }
        }
        else
        {
            /* Only DFP can talk to cable during a PD 2.0 contract. */
            if (dpm_stat->cur_port_type == PRT_TYPE_DFP)
            {
                /* Enable SOP_PRIME GoodCRC. */
                temp |= EN_PRIME_SOP_DET_VAL;

                /* If cable has been discovered, enabled SOP_DPRIME GoodCRC. */
                if (dpm_stat->emca_present)
                {
                    temp |= EN_DBL_SOP_DET_VAL;
                }
            }
        }
    }

    pd->rx_order_set_ctrl = temp;

    temp = pd->tx_ctrl;
    temp &= ~PDSS_TX_CTRL_GOODCRC_MSG_BITS_MASK;
    temp |= (TX_SOP_GD_CRC_HDR_DFLT | PD_DR_PR_ROLE(dpm_stat->cur_port_type, dpm_stat->cur_port_role));
    pd->tx_ctrl = temp;
#endif /* CCG_PD_REV3_ENABLE */
}

void pd_phy_en_unchunked_tx(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->header_info |= PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
}

void pd_phy_dis_unchunked_tx(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->header_info &= ~PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
}

bool pd_phy_load_data_in_mem(uint8_t port, bool start)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint8_t i;
    uint8_t tx_ptr = pd->sram_ptr & PDSS_SRAM_PTR_TX_FUNC_RD_PTR_MASK;
    uint8_t start_idx = 0;
    uint8_t mem_size = 0;

    if(pdss_stat->tx_dobj_count == 0)
    {
        return false;
    }

    if(start == true)
    {
        mem_size = PDSS_MAX_TX_MEM_SIZE;
    }
    else
    {
        mem_size = PDSS_MAX_TX_MEM_HALF_SIZE;
        if(tx_ptr < PDSS_MAX_TX_MEM_HALF_SIZE)
        {
            start_idx = PDSS_MAX_TX_MEM_HALF_SIZE;
        }
    }

    /* Copy the data into the Tx memory. */
    for (i = start_idx; i < (start_idx+ mem_size); i++)
    {
        pd->tx_mem_data[i] = pdss_stat->tx_dat_ptr[pdss_stat->tx_obj_sent];
        pdss_stat->tx_obj_sent++;
        if(pdss_stat->tx_obj_sent >= pdss_stat->tx_dobj_count)
        {
            return false;
        }
    }

    return true;
}

#if CCG_PD_REV3_ENABLE
void pd_phy_read_data_from_mem(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint8_t i;
    uint8_t start_idx = pdss_stat->rx_read_location;
    uint8_t mem_size = PDSS_MAX_RX_MEM_HALF_SIZE;

    /* Copy the data from rx memory. */
    for (i = start_idx; i < (start_idx + mem_size); i++)
    {
        if(pdss_stat->rx_unchunk_count >= pdss_stat->rx_unchunk_len)
        {
            return;
        }
        pdss_stat->rx_pkt.dat[pdss_stat->rx_unchunk_count].val = pd->rx_mem_data[i];
        pdss_stat->rx_unchunk_count++;
    }

    /* Flip to the other half of the SRAM for the next read. */
    pdss_stat->rx_read_location = (pdss_stat->rx_read_location == 0) ?
        PDSS_MAX_RX_MEM_HALF_SIZE: 0;
}
#endif /* CCG_PD_REV3_ENABLE */

bool pd_phy_load_msg(uint8_t port, sop_t sop, uint8_t retries,
        uint8_t dobj_count, uint32_t header, bool unchunked, uint32_t* buf)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];
    uint16_t exp_hdr;

    pdss_stat->tx_dat_ptr = buf;
    pdss_stat->tx_dobj_count = dobj_count;
    pdss_stat->tx_unchunked = unchunked;

    /* Make sure GoodCRC response to SOP'' messages is enabled where required. */
    if (sop == SOP_DPRIME)
    {
        pd->rx_order_set_ctrl |= EN_DBL_SOP_DET_VAL;
    }

    /* Configure SOP ordered set. */
    pd->tx_sop_order_set = os_table[sop];

    /*
     * Configure the expected sop type and expected GoodCRC. Expected GoodCRC
     * mask was already set by pd_phy_init(). SOP type in hardware is sop + 1.
     */
    exp_hdr = header & (~EXPECTED_GOOD_CRC_CLEAR_MASK);
    exp_hdr |= CTRL_MSG_GOOD_CRC;
    pd->rx_expect_goodcrc_msg = exp_hdr | ((sop + 1) <<
            PDSS_RX_EXPECT_GOODCRC_MSG_EXPECTED_SOP_POS);

    /* Load the header in the Tx header register. */
    pd->tx_header = header;

    /* Save the number of requested retries. */
    pdss_stat->retry_cnt = (int8_t)retries;

    return true;
}

void pd_phy_reset_rx_tx_sm(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Stop any ongoing transmission */
    pd->debug_ctrl |= (PDSS_DEBUG_CTRL_RESET_TX | PDSS_DEBUG_CTRL_RESET_RX);
    CyDelayUs(5);
    pd->debug_ctrl &= ~(PDSS_DEBUG_CTRL_RESET_TX | PDSS_DEBUG_CTRL_RESET_RX);
}

bool pd_phy_send_msg(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    uint32_t rval;

    pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

    /* Clear Tx interrupts. */
    pd->intr0 = (TX_INTERRUPTS | PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE |
        PDSS_INTR0_TX_RETRY_ENABLE_CLRD | PDSS_INTR0_CC_NO_VALID_DATA_DETECTED |
        PDSS_INTR0_TX_SRAM_HALF_END);

    if (pdss_stat->retry_cnt < 0)
    {
        /* Create this interrupt to stop transmission. */
        pd->intr0_set |= PDSS_INTR0_CRC_RX_TIMER_EXP;
        return true;
    }

    if (pd->status & (PDSS_STATUS_RX_BUSY | PDSS_STATUS_SENDING_GOODCRC_MSG))
    {
        pdss_stat->retry_cnt--;

        pd->intr0_mask |= PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

        /*
         * Notify the protocol layer so that it can start a timer so as to
         * avoid an infinite wait on the channel going idle.
         */
        pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_COLLISION);

        return true;
    }

    pdss_stat->tx_obj_sent = 0;
    if(pd_phy_load_data_in_mem(port, true) == true)
    {
        /* Enable TX SRAM HALF END interrupt */
        pd->intr0_mask |= PDSS_INTR0_TX_SRAM_HALF_END;
    }
    /* Enable Tx interrupts. */
    pd->intr0_mask |= TX_INTERRUPTS;

    /* Checks if unchunked TX need to be enabled */
    if(pdss_stat->tx_unchunked == true)
    {
        pd->header_info |= PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
    }
    else
    {
        pd->header_info &= ~PDSS_HEADER_INFO_EN_TX_EXTENDED_DATA;
    }

    rval = pd->tx_ctrl;
    if (pdss_stat->retry_cnt != 0)
    {
        rval |= PDSS_TX_CTRL_TX_RETRY_ENABLE;
    }
    else
    {
        /* No retries. */
        rval &= ~PDSS_TX_CTRL_TX_RETRY_ENABLE;
    }
    rval |= PDSS_TX_CTRL_TX_GO;

    pdss_stat->tx_done = false;
    pdss_stat->retry_cnt--;

    /* Begin transmission. */
    pd->tx_ctrl = rval;

    return true;
}

ccg_status_t pd_prot_stop(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd_prot_rx_dis(port, false);

    pd->intr0_mask &= ~(TX_INTERRUPTS | RST_TX_INTERRUPTS |
                        PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
                        PDSS_INTR0_CC_NO_VALID_DATA_DETECTED|
                        PDSS_INTR0_TX_SRAM_HALF_END);
    pd->intr0 = (TX_INTERRUPTS | RST_TX_INTERRUPTS |
                 PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
                 PDSS_INTR0_CC_NO_VALID_DATA_DETECTED|
                 PDSS_INTR0_TX_SRAM_HALF_END);

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_prot_rx_en(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    pdss_stat->rx_unchunked = false;

    /* Clear and enable RX interrupts. */
    pd->intr0 = (RX_INTERRUPTS |RCV_INTR_MASK);

#if CCG_PD_REV3_ENABLE

    dpm_status_t* dpm_stat = dpm_get_status(port);
    pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET |PDSS_INTR2_RX_SRAM_OVER_FLOW;
    if(dpm_stat->spec_rev_sop_live >=PD_REV3)
    {
        pd->intr2_mask |= PDSS_INTR2_EXTENDED_MSG_DET;
    }

#endif /* CCG_PD_REV3_ENABLE */

    pd->intr0_mask |= RX_INTERRUPTS;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_prot_rx_dis(uint8_t port, uint8_t hard_reset_en)
{
    uint32_t temp;
    PPDSS_REGS_T pd = gl_pdss[port];

#if CCG_PD_REV3_ENABLE

    pd->intr2_mask &= ~(PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET);
    pd->intr2 = (PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET |
            PDSS_INTR2_RX_SRAM_OVER_FLOW);

#endif /* CCG_PD_REV3_ENABLE */

    if (hard_reset_en == false)
    {
        /* Disable Rx.*/
        pd->rx_order_set_ctrl &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;

        /* Disable and clear all Rx interrupts.*/
        pd->intr0_mask &= ~RX_INTERRUPTS;
    }

    if (hard_reset_en == true)
    {
        /* Enable only Hard Reset reception. */
        temp = pd->rx_order_set_ctrl;
        temp &= ~PDSS_RX_ORDER_SET_CTRL_SOP_RST_EN_MASK;
        temp |= EN_RX_HARD_RESET_DET_VAL;
        pd->rx_order_set_ctrl = temp;

        /* Enable only the Hard Reset received interrrupt. */
        temp = pd->intr0_mask;
        temp &= ~RX_INTERRUPTS;
        temp |=  PDSS_INTR0_RCV_RST;
        pd->intr0_mask = temp;
    }

    pd->intr0 = RX_INTERRUPTS;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_send_bist_cm2(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Enable Tx regulator. */
    pd->tx_ctrl |= PDSS_TX_CTRL_TX_REG_EN;

    /* Delay to let the Tx regulator turn on. */
    CyDelayUs(50);

    /* Start BIST CM2. */
    pd->tx_ctrl |= PDSS_TX_CTRL_EN_TX_BIST_CM2;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_abort_bist_cm2(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Stop BIST CM2. */
    pd->tx_ctrl &= ~PDSS_TX_CTRL_EN_TX_BIST_CM2;

    /* Disable Tx regulator. */
    pd->tx_ctrl &= ~PDSS_TX_CTRL_TX_REG_EN;

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_abort_tx_msg(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    pdss_stat->tx_done = false;

    pd->intr0_mask &= ~(TX_INTERRUPTS | PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
                        PDSS_INTR0_CC_NO_VALID_DATA_DETECTED | PDSS_INTR0_TX_SRAM_HALF_END);
    pd->intr0 = (TX_INTERRUPTS | PDSS_INTR0_TX_RETRY_ENABLE_CLRD |
                 PDSS_INTR0_CC_NO_VALID_DATA_DETECTED |PDSS_INTR0_TX_SRAM_HALF_END);

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_phy_send_reset(uint8_t port, sop_t sop)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t loops = 10;

    /* If this is a hard reset, we should reset the TX and RX state machine for this port. */
    if (sop == HARD_RESET)
    {
        pd_phy_reset_rx_tx_sm(port);
    }

    /* Send a Hard Reset or Cable Reset. */
    pd->intr0 = RST_TX_INTERRUPTS;
    pd->intr0_mask |= RST_TX_INTERRUPTS;

    pd->tx_hard_cable_order_set = os_table[sop];

    /* Wait while there is valid data on the CC line. */
    if (sop == HARD_RESET)
    {
        /* Wait while there is valid data on the CC line. */
        while ((loops > 0) &&
                ((pd->status & (
                                PDSS_STATUS_CC_DATA_VALID |
                                PDSS_STATUS_RX_BUSY |
                                PDSS_STATUS_TX_BUSY |
                                PDSS_STATUS_SENDING_GOODCRC_MSG
                               )
                 ) != 0)
              )
        {
            loops--;
            CyDelayUs (10);
        }

        if (
                (pd->status & (
                               PDSS_STATUS_CC_DATA_VALID |
                               PDSS_STATUS_RX_BUSY |
                               PDSS_STATUS_TX_BUSY |
                               PDSS_STATUS_SENDING_GOODCRC_MSG
                              )
                ) != 0)
        {
            /* Return busy so that the reset gets attempted again at a later time. */
            return CCG_STAT_BUSY;
        }
    }

    pd->tx_ctrl |= PDSS_TX_CTRL_TX_SEND_RST;
    return CCG_STAT_SUCCESS;
}

pd_packet_extd_t *pd_phy_get_rx_packet(uint8_t port)
{
    return &gl_pdss_status[port].rx_pkt;
}

bool pd_phy_is_busy(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (((pd->status & (PDSS_STATUS_CC_DATA_VALID |
                        PDSS_STATUS_RX_BUSY |
                        PDSS_STATUS_TX_BUSY |
                        PDSS_STATUS_SENDING_GOODCRC_MSG
                       )) != 0 ) ||
        (pd->intr0 & PDSS_INTR0_RCV_RST))
    {
        return true;
    }

    return false;
}
#if (0)                // 19'OCT'2022 Reducing Flash Size Debug
#if ((defined(CCG3PA) || defined(CCG3PA2)) && (BATTERY_CHARGING_ENABLE))
void pdss_qc3_handle_intr(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];

    /* Clear interrupts */
    PDSS->intr6 = QC3_PORT_0_DP_DM_PULSE_MASK << cport;

    /* Read count */
    PDSS->qc3_chrger_ctrl[cport] |= PDSS_QC3_CHRGER_CTRL_READ_DPDM_COUNT;
    while(PDSS->qc3_chrger_ctrl[cport] & PDSS_QC3_CHRGER_CTRL_READ_DPDM_COUNT);
    int dp_pulse = (PDSS->qc3_chrger_ctrl[cport] & PDSS_QC3_CHRGER_CTRL_DP_PULSE_COUNT_MASK)
                    >> PDSS_QC3_CHRGER_CTRL_DP_PULSE_COUNT_POS;
    int dm_pulse = (PDSS->qc3_chrger_ctrl[cport] & PDSS_QC3_CHRGER_CTRL_DM_PULSE_COUNT_MASK)
                    >> PDSS_QC3_CHRGER_CTRL_DM_PULSE_COUNT_POS;
    gl_pdss_status[cport].bc_qc_pulse_count += (dp_pulse - dm_pulse);

    /* Report an QC continuous event. */
    if (pdss_stat->bc_phy_cbk != NULL)
    {
        gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(cport, BC_EVT_QC_CONT);
    }
}

void chgb_afc_load_tx_data(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];

    if(pdss_stat->bc_afc_tx_idx < pdss_stat->bc_afc_tx_size)
    {
        PDSS->afc_ping_pong[cport] = pdss_stat->bc_afc_tx_buf[pdss_stat->bc_afc_tx_idx++];
    }
    if(pdss_stat->bc_afc_tx_idx < pdss_stat->bc_afc_tx_size)
    {
        PDSS->afc_ping_pong[cport] |= (pdss_stat->bc_afc_tx_buf[pdss_stat->bc_afc_tx_idx++]) << 8u;
    }
}

void pdss_afc_handle_ping_pong_intr(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];

    if(((PDSS->afc_sm_status[cport] & PDSS_AFC_SM_STATUS_CURR_OP_CODE_MASK)
         >> PDSS_AFC_SM_STATUS_CURR_OP_CODE_POS) == AFC_TX_DATA_S_OPCODE)
    {
        /* TX going on */
        chgb_afc_load_tx_data(cport);
    }
    else
    {
        /* RX going on */
        if(pdss_stat->bc_afc_rx_idx < AFC_MAX_BYTES)
        {
            pdss_stat->bc_afc_rx_buf[pdss_stat->bc_afc_rx_idx++] = DWORD_GET_BYTE0(PDSS->afc_ping_pong[cport]);
        }
        if(pdss_stat->bc_afc_rx_idx < AFC_MAX_BYTES)
        {
            pdss_stat->bc_afc_rx_buf[pdss_stat->bc_afc_rx_idx++] = DWORD_GET_BYTE1(PDSS->afc_ping_pong[cport]);
        }
        /* For AFC source, only 1 byte will be received so creating this event here.
         * Later when AFC sink is to be implemented then we need to put a check if source
         * only then create event */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(cport, BC_EVT_AFC_MSG_RCVD);
        }
    }
}

void pdss_afc_handle_idle_intr(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    uint32_t event;

    if(PDSS->intr4 & ((1 << (PDSS_INTR4_AFC_ERROR_POS + cport)) |
        (1 << (PDSS_INTR4_AFC_TIMEOUT_POS + cport))))
    {
        event = BC_EVT_AFC_MSG_SEND_FAIL;
    }
    else
    {
        event = BC_EVT_AFC_MSG_SENT;
    }
    if (pdss_stat->bc_phy_cbk != NULL)
    {
        gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(cport, event);
    }
}

void pdss_afc_handle_reset_intr(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    /* Report reset received event. */
    if (pdss_stat->bc_phy_cbk != NULL)
    {
        gl_pdss_status[BC_PORT_0_IDX].bc_phy_cbk(cport, BC_EVT_AFC_RESET_RCVD);
    }
}
#endif /* ((defined(CCG3PA) || defined(CCG3PA2)) && (BATTERY_CHARGING_ENABLE)) */

#endif/**#if 0 to reduce code size*/
void pdss_intr0_handler(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    uint32_t rval;
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
#if ((CCG_HPD_RX_ENABLE) || (!CCG_PD_REV3_ENABLE))
    uint32_t  i;
#endif /* ((CCG_HPD_RX_ENABLE) || (!CCG_PD_REV3_ENABLE)) */

#if CCG_PD_REV3_ENABLE
    dpm_status_t* dpm_stat = dpm_get_status(port);
#endif /* CCG_PD_REV3_ENABLE */

    if (pd->intr0_masked != 0)
    {
        /*
         * Receive interrupt handling.
         */
        if (pd->intr0_masked & PDSS_INTR0_RCV_RST)
        {
            pdss_stat->tx_done = false;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_RST);
            pd->intr0 = (PDSS_INTR0_RCV_RST | PDSS_INTR0_EOP_ERROR);
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_PACKET_DONE)
        {
            pd->intr0 = PDSS_INTR0_TX_PACKET_DONE;
            pdss_stat->tx_done = true;
        }

#if CCG_PD_REV3_ENABLE
        if (pd->intr2_masked & PDSS_INTR2_EXTENDED_MSG_DET)
        {
            if(pd->intr2 & PDSS_INTR2_CHUNK_DET)
            {
                pdss_stat->rx_unchunked = false;
            }
            else
            {
                pdss_stat->rx_unchunked = true;
                /* Store total byte count and initialize byte received */
                /* Length in 32 bit units */
                pdss_stat->rx_unchunk_len = DIV_ROUND_UP(((pd_hdr_t)pd->rx_header).hdr.data_size, 4);
                if(pdss_stat->rx_unchunk_len > MAX_EXTD_PKT_WORDS)
                {
                    pdss_stat->rx_unchunk_len = MAX_EXTD_PKT_WORDS;
                }

                pdss_stat->rx_unchunk_count = 0;
                pdss_stat->rx_read_location = 0;
            }
            /* Extended message detected */
            /* Clear interrupt */
            pd->intr2 = PDSS_INTR2_CHUNK_DET | PDSS_INTR2_EXTENDED_MSG_DET;
        }

        if(pd->intr0_masked & PDSS_INTR0_RX_SRAM_HALF_END)
        {
            /* Store data in extended buf and update count*/
            pd_phy_read_data_from_mem(port);
            pd->intr0 = PDSS_INTR0_RX_SRAM_HALF_END;
        }
#endif /* CCG_PD_REV3_ENABLE */

        if (pd->intr0_masked & PDSS_INTR0_RX_STATE_IDLE)
        {
            rval =  pd->intr0;

            if (rval & PDSS_INTR0_RCV_GOODCRC_MSG_COMPLETE)
            {
                if (((rval & PDSS_INTR0_RCV_EXPT_GOODCRC_MSG_COMPLETE) != 0) &&
                    ((rval & PDSS_INTR0_EOP_ERROR) == 0))
                {
                    if (pdss_stat->tx_done == true)
                    {
                        pdss_stat->tx_done = false;

                        /* Stop retries due to CRC countdown expiry. */
                        pd->rx_expect_goodcrc_msg |= PDSS_RX_EXPECT_GOODCRC_MSG_DISABLE_RX_CRC_TIMER;

                        pd->intr0_mask &= ~TX_INTERRUPTS;
                        pd->intr0 = TX_INTERRUPTS;

                        /* Successful transmission notification. */
                        pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_SUCCESS);
                    }
                }
            }

            if (rval & PDSS_INTR0_RCV_GOOD_PACKET_COMPLETE)
            {
                if ((rval & PDSS_INTR0_EOP_ERROR) == 0)
                {
                    pdss_stat->tx_done = false;

                    /* Disable and clear PDSS_INTR0_CC_NO_VALID_DATA_DETECTED. */
                    pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;
                    pd->intr0 = PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

                    /*
                     * Copy the received packet and analyze it. If the packet
                     * is valid with good msg id and if a Tx message is active,
                     * stop the Tx message and send a tx discard response to the
                     * upper layer. Also do not create a packet received event
                     * just now.
                     */
                    pdss_stat->rx_pkt.sop = (((pd->status & PDSS_STATUS_SOP_TYPE_DETECTED_MASK) >>
                                PDSS_STATUS_SOP_TYPE_DETECTED_POS) - 1);

                    /* Copy out the header from the PD hardware. */
                    pdss_stat->rx_pkt.hdr.val = pd->rx_header;

#if CCG_PD_REV3_ENABLE
                    if(dpm_stat->spec_rev_sop_live <= PD_REV2)
                    {
                        /* Ignore reserved bits */
                        pdss_stat->rx_pkt.hdr.val &= ~(PD_MSG_HDR_REV2_IGNORE_MASK);
                    }
#else
                    pdss_stat->rx_pkt.hdr.val &= ~(PD_MSG_HDR_REV2_IGNORE_MASK);
#endif /* CCG_PD_REV3_ENABLE */

                    /* Copy the data from the hardware buffer to the software buffer. */
                    pdss_stat->rx_pkt.len = pdss_stat->rx_pkt.hdr.hdr.len;
                    pdss_stat->rx_pkt.msg = pdss_stat->rx_pkt.hdr.hdr.msg_type;
                    pdss_stat->rx_pkt.data_role = pdss_stat->rx_pkt.hdr.hdr.data_role;

#if CCG_PD_REV3_ENABLE
                    if(pdss_stat->rx_unchunked == false)
                    {
                        mem_copy_word((uint32_t*)pdss_stat->rx_pkt.dat,
                                (uint32_t*)pd->rx_mem_data, pdss_stat->rx_pkt.len);
                    }
                    else
                    {
                        pd_phy_read_data_from_mem(port);
                    }
#else
                    for (i = 0; i < pdss_stat->rx_pkt.len; i++)
                    {
                        pdss_stat->rx_pkt.dat[i].val = pd->rx_mem_data[i];
                    }
#endif /* CCG_PD_REV3_ENABLE*/
#if 0 //grl-edit
                    pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG);
#endif
                    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig != RES_TYPE_IGNORE)
                    {
                        //Cypress input
              		    if(pdss_stat->rx_pkt.sop == SOP)
                        {
                            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG);
                        }
                        else
                        {
                            pdss_stat->tx_done = true;
                            /**Pranay,15Jul'21, tracking received Message ID so that it can be used for our next SOP1 DiscID packet init */
                            if( (pdss_stat->rx_pkt.hdr.hdr.msg_type == DATA_MSG_VDM) && 
                                (pdss_stat->rx_pkt.dat[0].std_vdm_hdr.cmd_type != CMD_TYPE_RESP_ACK)
                                )
                                    g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID = pdss_stat->rx_pkt.hdr.hdr.msg_id;
                           
                            /**Pranay,15Jul'21, custom Interrupt flag will be enabled only if VUP is in Cable tester mode and received is ACK**/
                            if( (g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType == CCG_AS_CABLE_TESTER) && 
                                (pdss_stat->rx_pkt.dat[0].std_vdm_hdr.cmd_type == CMD_TYPE_RESP_ACK)
                                )
                                    g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = true;
                                    
                            /***If acting as Cable+Sink and received any SOP1 DISC ID then enabling custom interrupt flag*/
                            else if(g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType == CCG_AS_CABLE_AND_SNK)
                                    g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = true;
                            
                            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG_CMPLT); 
                        }  
                    }
                    else
                        pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG);
        		    
                }
            }

            
            if((pdss_stat->rx_pkt.hdr.hdr.msg_type == DATA_MSG_REQUEST) || (pdss_stat->rx_pkt.hdr.hdr.msg_type == DATA_MSG_SRC_CAP))
            {
                g_Struct_Ptr->RequestPacketConfig.gDUTSpecRev = pdss_stat->rx_pkt.hdr.hdr.spec_rev;
            }
            
            pd->intr0 = RCV_INTR_MASK;

#if CCG_PD_REV3_ENABLE
            pdss_stat->rx_unchunked = false;
            pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET | PDSS_INTR2_RX_SRAM_OVER_FLOW;
#endif /* CCG_PD_REV3_ENABLE */
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_GOODCRC_MSG_DONE)
        {
            /* Create a packet received event. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG_CMPLT);

            /* Clear the interrupt. */
            pd->intr0 = PDSS_INTR0_TX_GOODCRC_MSG_DONE;
        }

        if (pd->intr0_masked & PDSS_INTR0_COLLISION_TYPE3)
        {
            /* Create a packet received event. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_RX_MSG_CMPLT);
            pd->intr0 = PDSS_INTR0_COLLISION_TYPE3;
        }

        if (pd->intr0_masked & PDSS_INTR0_CC_NO_VALID_DATA_DETECTED)
        {
            /* Disable the interrupt. */
            pd->intr0_mask &= ~PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;
            pd->intr0 = PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

            /* Notify the protocol layer to stop the phy busy max limit timer. */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_PHY_IDLE);
        }

        /*
         * Tx interrupt handling.
         */

        if(pd->intr0_masked & PDSS_INTR0_TX_SRAM_HALF_END)
        {
            if (pd_phy_load_data_in_mem(port, false) == false)
            {
                pd->intr0_mask &= ~PDSS_INTR0_TX_SRAM_HALF_END;
            }
            pd->intr0 = PDSS_INTR0_TX_SRAM_HALF_END;
        }

        if (pd->intr0_masked & PDSS_INTR0_CRC_RX_TIMER_EXP)
        {
            pdss_stat->tx_done = false;

            if (pdss_stat->retry_cnt < 0)
            {
                /* Transmission failed. */

                pd->intr0_mask &= ~TX_INTERRUPTS;
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_FAILED);
            }
            else
            {
                if (pdss_stat->retry_cnt != 0)
                {
                    /* Clear and enable the TX retry enable cleared interrupt if required */
                    pd->intr0 = PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
                    /* Delay to remove any race */
                    CyDelayUs(5);
                    if (pd->tx_ctrl & PDSS_TX_CTRL_TX_RETRY_ENABLE)
                    {
                        pd->intr0_mask |= PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
                    }
                    else
                    {
                        /* Delay so that IP works otherwise if clear and set is too fast
                         * retry can fail */
                        CyDelayUs(5);
                        pd->tx_ctrl |= PDSS_TX_CTRL_TX_RETRY_ENABLE;
                    }
                }
                pdss_stat->retry_cnt--;
            }
            pd->intr0 = PDSS_INTR0_CRC_RX_TIMER_EXP;
        }

        if (pd->intr0_masked & PDSS_INTR0_TX_RETRY_ENABLE_CLRD)
        {
            CyDelayUs(5);
            /* Enable retry. */
            pd->tx_ctrl |= PDSS_TX_CTRL_TX_RETRY_ENABLE;

            /* Disable interrupts. */
            pd->intr0_mask &= ~PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
            pd->intr0 = PDSS_INTR0_TX_RETRY_ENABLE_CLRD;
        }

        if (pd->intr0_masked & (PDSS_INTR0_COLLISION_TYPE1 | PDSS_INTR0_COLLISION_TYPE2))
        {
            /*
             * Notify the protocol layer so that it can start a timer so as to
             * avoid an infinite wait on the channel going idle.
             */
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_MSG_COLLISION);

            /* Clear interrupts and enable the channel idle interrupt. */
            pd->intr0 = (PDSS_INTR0_COLLISION_TYPE1 | PDSS_INTR0_COLLISION_TYPE2 |
                         PDSS_INTR0_CC_NO_VALID_DATA_DETECTED);
            pd->intr0_mask |= PDSS_INTR0_CC_NO_VALID_DATA_DETECTED;

        }

        /*
         * Reset interrupt handling.
         */
        if (pd->intr0_masked & PDSS_INTR0_TX_HARD_RST_DONE)
        {
            pdss_stat->tx_done = false;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_RST_SUCCESS);
            pd->intr0_mask &= ~RST_TX_INTERRUPTS;
            pd->intr0 = RST_TX_INTERRUPTS;
        }

        if (pd->intr0_masked & PDSS_INTR0_COLLISION_TYPE4)
        {
            pdss_stat->tx_done = false;
            pd->intr0_mask &= ~RST_TX_INTERRUPTS;
            pd->intr0 = RST_TX_INTERRUPTS;
            pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_TX_RST_COLLISION);
        }
    }

    if (pd->intr2_masked != 0)
    {
        /* Handle the queue interrupt, instead of specific plug/unplug/irq interrupts. */
        if ((pd->intr2_masked & PDSS_INTR2_HPD_QUEUE) != 0)
        {
            /* Clear the interrupt and send callbacks for all queued events. */
            pd->intr2 = PDSS_INTR2_HPD_QUEUE;

#if CCG_HPD_RX_ENABLE
            if (hpd_cbks[port] != NULL)
            {
                i = pd->hpd_queue;
                if (HPD_GET_EVENT_0(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_0(i));
                if (HPD_GET_EVENT_1(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_1(i));
                if (HPD_GET_EVENT_2(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_2(i));
                if (HPD_GET_EVENT_3(i) != HPD_EVENT_NONE)
                    hpd_cbks[port] (port, HPD_GET_EVENT_3(i));
            }
#endif /* CCG_HPD_RX_ENABLE */
        }

        if ((pd->intr2_masked & PDSS_INTR2_HPDT_COMMAND_DONE) != 0)
        {
            /* Clear the interrupt and send the callback. */
            pd->intr2 = PDSS_INTR2_HPDT_COMMAND_DONE;
            if (hpd_cbks[port] != NULL)
                hpd_cbks[port] (port, HPD_COMMAND_DONE);
        }
#if (!(CCG_SOURCE_ONLY))
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
        if (pd->intr2_masked & (PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT))
        {
            /* Disable frs receive interrupts as we cannot clear them here. */
            pd->intr2_mask &= ~(PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT);

            /* Disable the swap controller */
            pd->swap_ctrl1 |= PDSS_SWAP_CTRL1_RESET_SWAP_STATE;

            /* Stop any ongoing transmission */
            pd_phy_reset_rx_tx_sm(port);

            /* Clear pending rx interrupts */
            pd->intr0 = RCV_INTR_MASK | PDSS_INTR0_COLLISION_TYPE3 |PDSS_INTR0_TX_GOODCRC_MSG_DONE;

            pdss_stat->rx_unchunked = false;
            pd->intr2 = PDSS_INTR2_EXTENDED_MSG_DET | PDSS_INTR2_CHUNK_DET | PDSS_INTR2_RX_SRAM_OVER_FLOW;

            /* Turn off the consumer fet */
            dpm_stat->app_cbk->psnk_disable(port, NULL);
            dpm_stat->skip_scan = true;

            if(pdss_stat->pd_phy_cbk != NULL)
            {
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_FRS_SIG_RCVD);
            }

            /* Cannot clear interrupt here as this will cause auto fet turn on to assume no FRS signal */
            CyIntClearPending(PD_PORT0_INTR0 + port);

            /* Enable the SWAP_VBUS_LESS_5_DONE interrupt so that we can identify when the power swap is done. */
            pd->intr1_mask |= PDSS_INTR1_VSWAP_VBUS_LESS_5_DONE;
        }
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
        if(pd->intr2_masked & PDSS_INTR2_SWAP_COMMAND_DONE)
        {
            pd_frs_tx_disable(port);
            dpm_stat->fr_tx_en_live = false;

            pd_phy_reset_rx_tx_sm(port);

            /* Change Rp to allow sink to initiate AMS */
            typec_change_rp(port, RP_TERM_RP_CUR_3A);

            /* Turn On the sink fet */
            dpm_stat->app_cbk->psnk_enable(port);
            /* Stop sourcing power. */
            dpm_stat->app_cbk->psrc_disable(port, NULL);

            if(pdss_stat->pd_phy_cbk != NULL)
            {
                pdss_stat->pd_phy_cbk(port, PD_PHY_EVT_FRS_SIG_SENT);
            }
        }
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */
#endif /* (!(CCG_SOURCE_ONLY)) */

    }

#if ((defined(CCG3PA) || defined(CCG3PA2)) && (BATTERY_CHARGING_ENABLE))
#if (!QC_AFC_CHARGING_DISABLED)                   //19'OCT'2022 Venkata reducing flash size Debug
    if(pd->intr4_masked != 0)
    {
        if (pd->intr4_masked & (1 << PDSS_INTR4_UPDATE_PING_PONG_POS))
        {
            pdss_afc_handle_ping_pong_intr (BC_PORT_0_IDX);
            pd->intr4 = 1 << PDSS_INTR4_UPDATE_PING_PONG_POS;
        }

        if (pd->intr4_masked & (1 << (PDSS_INTR4_UPDATE_PING_PONG_POS + 1)))
        {
            pdss_afc_handle_ping_pong_intr(BC_PORT_1_IDX);
            pd->intr4 = 1 << (PDSS_INTR4_UPDATE_PING_PONG_POS + 1);
        }

        if (pd->intr4_masked & (1 << PDSS_INTR4_AFC_SM_IDLE_POS))
        {
            pdss_afc_handle_idle_intr(BC_PORT_0_IDX);
            pd->intr4 = 1 << PDSS_INTR4_AFC_SM_IDLE_POS;
        }

        if (pd->intr4_masked & (1 << (PDSS_INTR4_AFC_SM_IDLE_POS + 1)))
        {
            pdss_afc_handle_idle_intr(BC_PORT_1_IDX);
            pd->intr4 = 1 << (PDSS_INTR4_AFC_SM_IDLE_POS + 1);
        }

        if (pd->intr4_masked & (1 << PDSS_INTR4_AFC_RX_RESET_POS))
        {
            pdss_afc_handle_reset_intr(BC_PORT_0_IDX);
            pd->intr4 = 1 << PDSS_INTR4_AFC_RX_RESET_POS;
        }

        if (pd->intr4_masked & (1 << (PDSS_INTR4_AFC_RX_RESET_POS + 1)))
        {
            pdss_afc_handle_reset_intr(BC_PORT_1_IDX);
            pd->intr4 = 1 << (PDSS_INTR4_AFC_RX_RESET_POS + 1);
        }
    }

    if(pd->intr6_masked != 0)
    {
        if (pd->intr6_masked & QC3_PORT_0_DP_DM_PULSE_MASK)
        {
            pdss_qc3_handle_intr(BC_PORT_0_IDX);

        }
        if (pd->intr6_masked & QC3_PORT_1_DP_DM_PULSE_MASK)
        {
            pdss_qc3_handle_intr(BC_PORT_1_IDX);
        }
    }
#endif  /*#if (!QC_AFC_CHARGING_DISABLED)//19'OCT'2022 Venkata reducing flash size Debug**/
#endif /* ((defined(CCG3PA) || defined(CCG3PA2)) && (BATTERY_CHARGING_ENABLE)) */
}

/*
 * Type C functionality.
 */
void pd_typec_dis_up_dn_cmp_filter(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN |
                                       PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN);
}

void pd_typec_en_dp_dn_cmp_filter(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->intr1_cfg |= (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN |
                      PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN);
}

ccg_status_t pd_typec_init(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_HYST_MODE | PDSS_CC_CTRL_0_EN_HYST | PDSS_CC_CTRL_0_CMP_LA_VSEL_MASK);
    pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_CMP_LA_VSEL_CFG << PDSS_CC_CTRL_0_CMP_LA_VSEL_POS);

    /*
     * Up/Down comparators filter will only be enabled before going to
     * deepsleep and disabled after coming out of deepsleep.
     */
    pd_typec_dis_up_dn_cmp_filter(port);

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Disable filter on comparator 1 and comparator 2 outputs. */
    pd->intr3_cfg_adc_hs[0] &= ~(PDSS_INTR3_CFG_ADC_HS_FILT_EN);
    pd->intr3_cfg_adc_hs[1] &= ~(PDSS_INTR3_CFG_ADC_HS_FILT_EN);

    /* Always enable the pump. */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP | PDSS_PUMP_CTRL_BYPASS_LV);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Disable filter on comparator output. */
    pd->intr3_cfg_adc_hs &= ~(PDSS_INTR3_CFG_ADC_HS_FILT_EN);
    pd->intr3_cfg_adc_hs |= PDSS_INTR3_CFG_ADC_HS_FILT_BYPASS;

    /* Always enable the pump. PUMP enable is done through Port0 for both ports. */
    ccg5_pump_enable (0, 0);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}

#if CCG_PD_DUALPORT_ENABLE
static uint32_t pdss_active_ports = 0;
#endif /* CCG_PD_DUALPORT_ENABLE */

ccg_status_t pd_typec_start(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;
#if CCG_PD_DUALPORT_ENABLE
    if (port == 1)
    {
        gl_pdss[0]->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;
    }
#endif

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Enable PUMP */
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* PUMP enable is done through Port0 for both ports. */
    ccg5_pump_enable (0, 0);
#else
    /* Always enable the pump. */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP | PDSS_PUMP_CTRL_BYPASS_LV);
#endif /* (defined (CCG5) || defined(CCG6) || defined(CCG5C)) */

    CyDelayUs(50);

    pd->cc_ctrl_0  |= (PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_RX_EN);

    /* Use voltage reference from refgen to define CC tx_swing. */
    pd->cc_ctrl_1 |= PDSS_CC_CTRL_1_CC_VREF_1P1_SEL;

#if ((defined(CCG3PA) || defined(CCG3PA2)) && (VBUS_IN_DISCHARGE_EN) && (!CCG_FLIPPED_FET_CTRL))
    /* Set VBUS_IN discharge threshold to 5.5V */

    PDSS->dischg_shv_ctrl[1] |= PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG;

    /* Resistor div on VBUS IN already set to 10% in HW */
    /* Configure Reference for comparator. */
    regval = pd->refgen_4_ctrl;
    regval &= ~(PDSS_REFGEN_4_CTRL_SEL11_MASK);
    regval |= (PDSS_VBUS_IN_REF_SEL_VAL << PDSS_REFGEN_4_CTRL_SEL11_POS);
    pd->refgen_4_ctrl = regval;

    /* Enable Comparator. */
    pd->comp_ctrl[COMP_ID_VBUS_DISCHARGE] |= PDSS_COMP_CTRL_COMP_ISO_N;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) && (VBUS_IN_DISCHARGE_EN !=0) */

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd->refgen_0_ctrl &= ~PDSS_REFGEN_0_CTRL_REFGEN_PD;
    pd->refgen_0_ctrl  |= PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                          PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL;

    /* Set SEL7 reference (used by EA shunt regulator during deep sleep) to 1.2V */
    pd->refgen_2_ctrl = (pd->refgen_2_ctrl & PDSS_REFGEN_2_CTRL_SEL7_MASK) |
        (107 << PDSS_REFGEN_2_CTRL_SEL7_POS);

    /* Set SEL14 reference (used by cc_shvt block for tx_swing to 1.12 V. */
    pd->refgen_4_ctrl  = (pd->refgen_4_ctrl & ~PDSS_REFGEN_4_CTRL_SEL14_MASK) |
        (4 << PDSS_REFGEN_4_CTRL_SEL14_POS);

    /* Give some delay for references to settle. */
    CyDelayUs (50);

#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_OPTO_FB)
    PDSS_TRIMS->trim_ea1_0 = 0xA0;
    /* Use internal current for reference in case of opto feedback. */
    pd->ea_ctrl = ((pd->ea_ctrl & ~PDSS_EA_CTRL_IREF_BG_SEL_MASK) |
        (1u << PDSS_EA_CTRL_IREF_BG_SEL_POS));
#else /* (VBUS_CTRL_TYPE_P1 != VBUS_CTRL_OPTO_FB) */
    /*
     * For ** silicon, use this value. For *A silicon, the value is expected to
     * be loaded dynamically based on whether we are sourcing iDAC or sinking.
     * These TRIM settings are available only in *A silicon.
     */
    PDSS_TRIMS->trim_ea1_0 = 0x20;
     /* Use BG reference for direct feedback. */
    pd->ea_ctrl = ((pd->ea_ctrl & ~PDSS_EA_CTRL_IREF_BG_SEL_MASK) |
        (2u << PDSS_EA_CTRL_IREF_BG_SEL_POS));
#endif /* (VBUS_CTRL_TYPE_P1) */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    /* Disable all filters */
    regval = pd->intr1_cfg_cc1_cc2_ls;
    regval &= ~(PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_EN |
            PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_EN);
    regval |= PDSS_INTR1_CFG_CC1_CC2_LS_CC1_FILT_BYPASS |
                                PDSS_INTR1_CFG_CC1_CC2_LS_CC2_FILT_BYPASS;

    pd->intr1_cfg_cc1_cc2_ls = regval;

    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN);
    pd->intr1_cfg_vcmp_up_down_ls |= (PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS |
            PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS);

    /* Set LA config for wakeup */
    pd->intr1_cfg |= PDSS_INTR1_CFG_VCMP_LA_CFG_MASK;

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Enable the filter associated with CC1/2 OVP detection. */
    pd->intr1_cfg_cc12_ovp_hs = (
            (4 << PDSS_INTR1_CFG_CC12_OVP_HS_CC1_OVP_FILT_SEL_POS) |
            (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR1_CFG_CC12_OVP_HS_CC1_OVP_FILT_CFG_POS) |
            (PDSS_INTR1_CFG_CC12_OVP_HS_CC1_OVP_DPSLP_MODE) |
            (4 << PDSS_INTR1_CFG_CC12_OVP_HS_CC2_OVP_FILT_SEL_POS) |
            (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR1_CFG_CC12_OVP_HS_CC2_OVP_FILT_CFG_POS) |
            (PDSS_INTR1_CFG_CC12_OVP_HS_CC2_OVP_DPSLP_MODE)
            );
    pd->intr1_cfg_cc12_ovp_hs |= (
            PDSS_INTR1_CFG_CC12_OVP_HS_CC1_OVP_FILT_EN |
            PDSS_INTR1_CFG_CC12_OVP_HS_CC2_OVP_FILT_EN
            );

    /* Clear and Enable CC1/2 OVP change interrupt. */
    pd->intr1       = (PDSS_INTR1_CC1_OVP_CHANGED | PDSS_INTR1_CC2_OVP_CHANGED);
    pd->intr1_mask |= (PDSS_INTR1_MASK_CC1_OVP_CHANGED_MASK | PDSS_INTR1_MASK_CC2_OVP_CHANGED_MASK);

    /* If OVP condition is present, set the interrupt again. */
    if ((pd->intr1_status & PDSS_INTR1_STATUS_CC1_OVP_FILT) != 0)
        pd->intr1_set |= PDSS_INTR1_SET_CC1_OVP_CHANGED;
    if ((pd->intr1_status & PDSS_INTR1_STATUS_CC2_OVP_FILT) != 0)
        pd->intr1_set |= PDSS_INTR1_SET_CC2_OVP_CHANGED;

    /* Enable the SBU OVP filters. */
    pd->intr3_cfg_sbu20_ovp_hs = (
            (4 << PDSS_INTR3_CFG_SBU20_OVP_HS_SBU1_FILT_SEL_POS) |
            (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR3_CFG_SBU20_OVP_HS_SBU1_FILT_CFG_POS) |
            (PDSS_INTR3_CFG_SBU20_OVP_HS_SBU1_DPSLP_MODE) |
            (4 << PDSS_INTR3_CFG_SBU20_OVP_HS_SBU2_FILT_SEL_POS) |
            (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR3_CFG_SBU20_OVP_HS_SBU2_FILT_CFG_POS) |
            (PDSS_INTR3_CFG_SBU20_OVP_HS_SBU2_DPSLP_MODE)
            );
    pd->intr3_cfg_sbu20_ovp_hs |= (
            PDSS_INTR3_CFG_SBU20_OVP_HS_SBU1_FILT_EN |
            PDSS_INTR3_CFG_SBU20_OVP_HS_SBU2_FILT_EN
            );

    /* Enable OVP detection on the SBU pins. */
    pd->intr3       = PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK;
    pd->intr3_mask |= PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK;

#if CCGX_V5V_CHANGE_DETECT
    /* Enable the V5V detect comparator. */
    pd->vconn20_ctrl |= PDSS_VCONN20_CTRL_EN_COMP;

    /* Enable the V5V detect filter. */
    pd->intr1_cfg     = (pd->intr1_cfg & ~(PDSS_INTR1_CFG_V5V_FILT_EN |
                PDSS_INTR1_CFG_V5V_CFG_MASK | PDSS_INTR1_CFG_V5V_FILT_BYPASS));
    CyDelayUs (10);
    pd->intr1_cfg    |= PDSS_INTR1_CFG_V5V_CFG_MASK;
    CyDelayUs (10);
    pd->intr1_cfg    |= PDSS_INTR1_CFG_V5V_FILT_EN;
    CyDelayUs (10);

    /* Clear and enable the V5V change interrupt. */
    pd->intr1       = PDSS_INTR1_V5V_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
#endif /* CCGX_V5V_CHANGE_DETECT */

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if CCG_PD_DUALPORT_ENABLE
    pdss_active_ports |= (1 << port);
#endif

    return CCG_STAT_SUCCESS;
}

void pd_typec_rd_enable(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

    pd->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;
#if CCG_PD_DUALPORT_ENABLE
    if (port == 1)
    {
        gl_pdss[0]->dpslp_ref_ctrl &= ~PDSS_DPSLP_REF_CTRL_PD_DPSLP;
    }
#endif

    /* Power up the block. */
    pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_PWR_DISABLE;
    pd->cc_ctrl_0 |= (PDSS_CC_CTRL_0_CMP_EN | PDSS_CC_CTRL_0_RX_EN);

    /* Enable Rd on both CC lines. */
    temp = pd->cc_ctrl_0;
    temp |= (PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC1_DB_DIS);
    temp |= (PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);
    temp &= ~PDSS_CC_CTRL_0_DFP_EN;
    pd->cc_ctrl_0 = temp;
}

ccg_status_t pd_typec_stop(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if CCGX_V5V_CHANGE_DETECT
    /* Clear and disable the V5V change detect interrupt. */
    pd->intr1       = PDSS_INTR1_V5V_CHANGED;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_V5V_CHANGED_MASK;
    CyDelayUs (10);

    /* Disable and bypass the V5V detect filter. */
    pd->intr1_cfg = (pd->intr1_cfg & ~(PDSS_INTR1_CFG_V5V_FILT_EN |
                PDSS_INTR1_CFG_V5V_CFG_MASK | PDSS_INTR1_CFG_V5V_FILT_BYPASS));
    CyDelayUs (10);
    pd->intr1     = PDSS_INTR1_V5V_CHANGED;
#endif /* CCGX_V5V_CHANGE_DETECT */

    /* Clear and disable SBU OVP detect interrupts. */
    pd->intr3_mask &= ~(PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK);
    pd->intr3       = PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK;
    pd->intr3_cfg_sbu20_ovp_hs &= ~(
            PDSS_INTR3_CFG_SBU20_OVP_HS_SBU1_FILT_EN |
            PDSS_INTR3_CFG_SBU20_OVP_HS_SBU2_FILT_EN
            );

    /* Clear and disable CC1/2 OVP change interrupt. */
    pd->intr1       = (PDSS_INTR1_CC1_OVP_CHANGED | PDSS_INTR1_CC2_OVP_CHANGED);
    pd->intr1_mask &= ~(PDSS_INTR1_MASK_CC1_OVP_CHANGED_MASK | PDSS_INTR1_MASK_CC2_OVP_CHANGED_MASK);

    pd->intr1_cfg_cc12_ovp_hs &= ~(
            PDSS_INTR1_CFG_CC12_OVP_HS_CC1_OVP_FILT_EN |
            PDSS_INTR1_CFG_CC12_OVP_HS_CC2_OVP_FILT_EN
            );
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Power down the block. */
    pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_PWR_DISABLE;

    /* Disable PUMP */
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if CCG_PD_DUALPORT_ENABLE
    pdss_active_ports &= ~(1 << port);
    if (pdss_active_ports == 0)
#endif /* CCG_PD_DUALPORT_ENABLE */
    {
        ccg5_pump_disable (0, 0);
    }
#else /* CCG3PA, CCG3PA2 */
    pd->pump_ctrl |= (PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);
#endif /* CCGx */

#if (!CCG_PD_BLOCK_ALWAYS_ON)
#if (CCG_PD_DUALPORT_ENABLE)
    if (pdss_active_ports == 0)
#endif /* CCG_PD_DUALPORT_ENABLE */
    {
        /* Turn off references. */
        pd->dpslp_ref_ctrl |= PDSS_DPSLP_REF_CTRL_PD_DPSLP;
    }
#endif /* (!CCG_PD_BLOCK_ALWAYS_ON) */

    return CCG_STAT_SUCCESS;
}

#if (!(CCG_SOURCE_ONLY))
void pd_typec_snk_update_trim(uint8_t port)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    PPDSS_TRIMS_REGS_T trimregs;
    dpm_status_t *dpm_stat = dpm_get_status(port);

#if CCG_PD_DUALPORT_ENABLE
    if (port != 0)
        trimregs = PDSS_TRIMS1;
    else
        trimregs = PDSS_TRIMS0;
#else
#ifdef CCG5
    trimregs = PDSS_TRIMS0;
#else
    trimregs = PDSS_TRIMS;
#endif /* CCG5 */
#endif /* CCG_PD_DUALPORT_ENABLE */

    if (dpm_stat->cur_port_role == PRT_ROLE_SINK)
    {
        trimregs->trim_cc_0 &= ~PDSS_TRIM_CC_0_TX_TRIM_MASK;

        if (dpm_stat->snk_cur_level == RD_3A)
        {
            /* Use faster slew rate when 3A Rp is in use. */
            trimregs->trim_cc_0 |= (TRIM0_TX_TRIM_VALUE_3A << PDSS_TRIM_CC_0_TX_TRIM_POS);
        }
    }
#else /* CCG3PA/CCG3PA2 */
    (void)port;
    /* Do nothing for CCG3PA / CCG3PA2. */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
}
#endif /* (!(CCG_SOURCE_ONLY)) */

void pd_typec_en_rp(uint8_t port, uint8_t channel, rp_term_t rp_val)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t rp_mode;
    uint32_t temp;
    rp_mode = rp_val;

    PPDSS_TRIMS_REGS_T trimregs;
    uint8_t cc_trim = 0;

    /* Enable PUMP */
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* PUMP enable is done through Port0 for both ports. */
    ccg5_pump_enable (0, 0);
#else /* CCG3PA, CCG3PA2 */
    /* Always enable the pump. */
    pd->pump_ctrl &= ~(PDSS_PUMP_CTRL_PD_PUMP | PDSS_PUMP_CTRL_BYPASS_LV);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    if (port == TYPEC_PORT_0_IDX)
    {
        cc_trim = pdss_rp_trim_db_0[rp_val];

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5C) || defined(CCG6))
        trimregs = PDSS_TRIMS;
#elif defined(CCG5)
        trimregs = PDSS_TRIMS0;
#endif /* CCGx */
    }
#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
    else
    {
        cc_trim = pdss_rp_trim_db_1[rp_val];
        trimregs = PDSS_TRIMS1;
    }
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */

    /* Set cc trim from sflash */
    if (channel == CC_CHANNEL_1)
    {
        trimregs->trim_cc_1 = cc_trim;
    }
    else
    {
        trimregs->trim_cc_2 = cc_trim;
    }

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Do the TX trim only for CCG5. */
    trimregs->trim_cc_0 &= ~PDSS_TRIM_CC_0_TX_TRIM_MASK;
    if (rp_val == RP_TERM_RP_CUR_3A)
    {
        /* Actual value in HW register for 3A Rp is (RP_TERM_RP_CUR_3A + 1) */
        rp_mode++;

        /* Use faster slew rate when 3A Rp is in use. */
        trimregs->trim_cc_0 |= (TRIM0_TX_TRIM_VALUE_3A << PDSS_TRIM_CC_0_TX_TRIM_POS);
    }
#else /* (defined(CCG3PA) || defined(CCG3PA2)) */
    /* Trims are not required to be updated for CCG3PA and CCG3PA2. */
    if (rp_val == RP_TERM_RP_CUR_3A)
    {
        /* Actual value in HW register for 3A Rp is (RP_TERM_RP_CUR_3A + 1) */
        rp_mode++;
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Set Rp mode and enable references for source operation. */
    temp = pd->cc_ctrl_0;
    temp &= ~PDSS_CC_CTRL_0_RP_MODE_MASK;
    temp |= (rp_mode << PDSS_CC_CTRL_0_RP_MODE_POS) | PDSS_CC_CTRL_0_DFP_EN;

    if (channel == CC_CHANNEL_1)
    {
        temp |= PDSS_CC_CTRL_0_RP_CC1_EN;
    }
    else
    {
        temp |= PDSS_CC_CTRL_0_RP_CC2_EN;
    }
    pd->cc_ctrl_0 = temp;
}

void pd_typec_dis_rp(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (channel == CC_CHANNEL_1)
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RP_CC1_EN;
    }
    else
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_RP_CC2_EN;
    }
}

void pd_typec_en_dpslp_rp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_1 |= PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
}

void pd_typec_dis_dpslp_rp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->cc_ctrl_1 &= ~PDSS_CC_CTRL_1_DS_ATTACH_DET_EN;
}

void pd_typec_en_deadbat_rd(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

    temp = pd->cc_ctrl_0;

    /* Re-enable dead battery Rd */
    temp &= ~(PDSS_CC_CTRL_0_RD_CC1_DB_DIS | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);

    /* Remove trimmed Rd */
    temp &= ~(PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC2_EN);

    pd->cc_ctrl_0 = temp;
}

void pd_typec_en_rd(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t temp;

    /* Disable PUMP */
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if (!CCG_PD_DUALPORT_ENABLE)
    ccg5_pump_disable (0, 0);
#endif /* (!CCG_PD_DUALPORT_ENABLE) */
#else /* CCG3PA, CCG3PA2 */
    pd->pump_ctrl |= (PDSS_PUMP_CTRL_PD_PUMP  | PDSS_PUMP_CTRL_BYPASS_LV);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    temp = pd->cc_ctrl_0;
    if (channel == CC_CHANNEL_1)
    {
        temp |= (PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC1_DB_DIS);
    }
    else
    {
        temp |= (PDSS_CC_CTRL_0_RD_CC2_EN | PDSS_CC_CTRL_0_RD_CC2_DB_DIS);
    }

    temp &= ~PDSS_CC_CTRL_0_DFP_EN;
    pd->cc_ctrl_0 = temp;
}

void pd_typec_dis_rd(uint8_t port, uint8_t channel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval =  pd->cc_ctrl_0;

    if (channel == CC_CHANNEL_1)
    {
        regval &= ~PDSS_CC_CTRL_0_RD_CC1_EN;
        regval |= PDSS_CC_CTRL_0_RD_CC1_DB_DIS;
    }
    else
    {
        regval &= ~PDSS_CC_CTRL_0_RD_CC2_EN;
        regval |= PDSS_CC_CTRL_0_RD_CC2_DB_DIS;
    }
    pd->cc_ctrl_0 = regval;
}

/* Returns the current status on the CC line (rp_cc_status_t or rd_cc_status_t). */
static uint8_t pd_typec_get_rp_rd_status(uint8_t port, uint8_t channel, bool rd_idx)
{
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t rval = 0;
    uint32_t temp;
    uint8_t out;
    uint32_t status;
    bool change = false;

    /* Set default output. */
    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        out = RP_OPEN;
    }
    else
    {
        out = RD_RA + rd_idx;
    }

    /* Connect both the Up/Dn comparators to the active CC line. */
    if (channel == CC_CHANNEL_2)
    {
        rval = (PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);
    }
    temp = pd->cc_ctrl_0 & (PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);

    if ( temp != rval)
    {
        pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_DN_CC1V2 | PDSS_CC_CTRL_0_CMP_UP_CC1V2);
        pd->cc_ctrl_0 |= rval;

        change = true;
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        /*
         * Set the threshold of the Dn comparator to Ra level and the Up
         * comparator to Rp open level.
         */
        rval = ((thresholds[dpm_stat->src_cur_level_live][0]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS) |
            ((thresholds[dpm_stat->src_cur_level_live][1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS);
    }
    else
    {
        /* Set the Dn comparator to vRdUSB and the Up comparator to vRd1.5A. */
        rval = ((thresholds[RD_ROW_NO][rd_idx]) << PDSS_CC_CTRL_0_CMP_DN_VSEL_POS) |
            ((thresholds[RD_ROW_NO][rd_idx + 1]) << PDSS_CC_CTRL_0_CMP_UP_VSEL_POS);
    }



    temp = pd->cc_ctrl_0 & (PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK | PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK);

    if (temp != rval)
    {
        pd->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_CMP_DN_VSEL_MASK | PDSS_CC_CTRL_0_CMP_UP_VSEL_MASK);
        pd->cc_ctrl_0 |= rval;
        change = true;
    }

    if (change == true)
    {
        /* Delay to allow references to settle. */
        CyDelayUs (50);
    }

    status = pd->intr1_status;

    if (((status & PDSS_INTR1_STATUS_VCMP_DN_STATUS) != 0) && ((status & PDSS_INTR1_STATUS_VCMP_UP_STATUS) == 0))
    {
        if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
        {
            out = RP_RD;
        }
        else
        {
            out = RD_USB + rd_idx;
        }
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        if (((status & PDSS_INTR1_STATUS_VCMP_DN_STATUS) == 0) && ((status & PDSS_INTR1_STATUS_VCMP_UP_STATUS) == 0))
        {
            out = RP_RA;
        }
    }

    if (dpm_stat->cur_port_role == PRT_ROLE_SINK)
    {
        if (((status & PDSS_INTR1_STATUS_VCMP_DN_STATUS) != 0) && ((status & PDSS_INTR1_STATUS_VCMP_UP_STATUS) != 0))
        {
            out = RD_1_5A + rd_idx;
        }
    }

    return out;
}

cc_state_t pd_typec_get_cc_status(uint8_t port)
{
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t polarity = dpm_stat->polarity;
    cc_state_t new_state;
    uint8_t i;

    if (dpm_stat->cur_port_role == PRT_ROLE_SOURCE)
    {
        if ((dpm_stat->attach) && (pd->status & (PDSS_STATUS_TX_BUSY | PDSS_STATUS_CC_DATA_VALID)))
        {
            new_state = dpm_stat->cc_old_status ;
        }
        else
        {
            /* Scan both CC lines: the active CC line should be scanned last. */
            if (polarity)
            {
                new_state.cc[0] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_1, 0);
                new_state.cc[1] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_2, 0);
            }
            else
            {
                new_state.cc[1] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_2, 0);
                new_state.cc[0] = pd_typec_get_rp_rd_status(port, CC_CHANNEL_1, 0);
            }

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
            if (gl_pdss_status[port].cc_ovp_pending)
            {
                if (new_state.cc[polarity] == RP_OPEN)
                {
                    /* Keep returning RP_RD status while OVP is active. */
                    new_state.cc[polarity] = RP_RD;
                }
                else
                {
                    /* Re-enable the Rp termination and mark OVP not active. */
                    pd_typec_en_rp(port, polarity, (rp_term_t)dpm_stat->src_cur_level_live);
                    gl_pdss_status[port].cc_ovp_pending = false;
                }
            }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
        }
    }
    else
    {
        if (dpm_stat->attach == true)
        {
            new_state = dpm_stat->cc_status;

            /* Keep using previously debounced status while the CC TX/RX is busy. */
            if ((pd->status & (PDSS_STATUS_TX_BUSY | PDSS_STATUS_CC_DATA_VALID)) == 0)
            {
                new_state.cc[polarity] = pd_typec_get_rp_rd_status(port, polarity, 1);

                /* If CC line voltage is below the 1.5 A Rp threshold, do another check for presence of Rp. */
                if (new_state.cc[polarity] <= RD_USB)
                {
                    new_state.cc[polarity] = pd_typec_get_rp_rd_status(port, polarity, 0);
                }

                /* Only the active CC line needs to be scanned. */
                new_state.cc[dpm_stat->rev_pol] = RD_RA;
            }
        }
        else
        {
            for (i = 0; i < 2; i++)
            {
                /* Scan CC[i] with threshold vRa and vRdUsb. */
                new_state.cc[i] = pd_typec_get_rp_rd_status(port, i, 0);
                if (new_state.cc[i] != RD_RA)
                {
                    /* Scan CC[i] again with vRdusb and vRd1.5A to determine correct Rp value. */
                    new_state.cc[i] = pd_typec_get_rp_rd_status(port, i, 1);
                }
            }
        }
    }

    return new_state;
}

void pd_typec_set_polarity (uint8_t port, bool polarity)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (polarity == 0)
    {
        pd->cc_ctrl_0 &= ~PDSS_CC_CTRL_0_CC_1V2;
    }
    else
    {
        pd->cc_ctrl_0 |= PDSS_CC_CTRL_0_CC_1V2;
    }
}

bool pd_is_v5v_supply_on(uint8_t port)
{
    bool stat = true;

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if CCGX_V5V_CHANGE_DETECT
    PPDSS_REGS_T pd = gl_pdss[port];

    /* If V5V is not present, return error. */
    if ((pd->intr1_status & PDSS_INTR1_STATUS_V5V_STATUS) == 0)
        stat = false;
#endif /* CCGX_V5V_CHANGE_DETECT */
#endif /* (defined (CCG5) || defined(CCG6) || defined(CCG5C)) */

    return stat;
}

ccg_status_t pd_vconn_enable(uint8_t port, uint8_t channel)
{
#if (defined(CCG3PA) || defined(CCG3PA2))

    /* No VConn support in CCG3PA family as of now. */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

#if CCGX_V5V_CHANGE_DETECT
    /* If V5V is not present, return error. */
    if ((pd->intr1_status & PDSS_INTR1_STATUS_V5V_STATUS) == 0)
        return CCG_STAT_FAILURE;
#endif /* CCGX_V5V_CHANGE_DETECT */

    /* Check whether VConn has already been turned ON. */
    if (pd_is_vconn_present(port, channel))
        return CCG_STAT_SUCCESS;

    /* Turn on the VConn switch. */
    if (channel == CC_CHANNEL_1)
    {
        regval = pd->vconn20_cc1_switch_1_ctrl;
        regval |= PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_CC1_SWITCH_1_CTRL_EN_SWITCH_CC1_ON_VALUE |
            PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_CC1_OVP | PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_CC2_OVP;
        pd->vconn20_cc1_switch_1_ctrl = regval;

        /* Reset edge detector. */
        pd->vconn20_cc1_switch_1_ctrl |= PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc1_switch_1_ctrl &= ~PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
    }
    else
    {
        regval = pd->vconn20_cc2_switch_1_ctrl;
        regval |= PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_CC2_SWITCH_1_CTRL_EN_SWITCH_CC2_ON_VALUE |
            PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_CC1_OVP | PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_CC2_OVP;
        pd->vconn20_cc2_switch_1_ctrl = regval;

        /* Reset edge detector. */
        pd->vconn20_cc2_switch_1_ctrl |= PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc2_switch_1_ctrl &= ~PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
    }

    /* Turn on the VConn pump. */
    regval = pd->vconn20_pump_en_1_ctrl;
    regval |= PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_PUMP_EN_1_CTRL_EN_VCONN20_PUMP_ON_VALUE |
        PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC1_OVP | PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC2_OVP;
    pd->vconn20_pump_en_1_ctrl = regval;

#if DPM_DEBUG_SUPPORT
    dpm_get_status(port)->cc_stat[channel] = CCG_CC_STAT_VCONN_ACTIVE;
#endif /* DPM_DEBUG_SUPPORT */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t pd_vconn_disable(uint8_t port, uint8_t channel)
{
#if (defined(CCG3PA) || defined(CCG3PA2))

    /* No VConn support in CCG3PA family as of now. */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    /* Check whether VConn has already been turned OFF. */
    if (!pd_is_vconn_present(port, channel))
        return CCG_STAT_SUCCESS;

    /* Turn off the VConn pump. */
    regval = pd->vconn20_pump_en_1_ctrl;
    regval &= ~(PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_PUMP_EN_1_CTRL_EN_VCONN20_PUMP_ON_VALUE |
            PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC1_OVP | PDSS_VCONN20_PUMP_EN_1_CTRL_SEL_CC2_OVP);
    pd->vconn20_pump_en_1_ctrl = regval;

    /* Adding a small delay. */
    CyDelayUs (10);

    /* Turn off the VConn switch. */
    if (channel == CC_CHANNEL_1)
    {
        regval = pd->vconn20_cc1_switch_1_ctrl;
        regval &= ~(PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_CC1_SWITCH_1_CTRL_EN_SWITCH_CC1_ON_VALUE |
                PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_CC1_OVP);
        pd->vconn20_cc1_switch_1_ctrl = regval;

        /* Reset edge detector. */
        pd->vconn20_cc1_switch_1_ctrl |= PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc1_switch_1_ctrl &= ~PDSS_VCONN20_CC1_SWITCH_1_CTRL_RST_EDGE_DET;
    }
    else
    {
        regval = pd->vconn20_cc2_switch_1_ctrl;
        regval &= ~(PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_ON_OFF | PDSS_VCONN20_CC2_SWITCH_1_CTRL_EN_SWITCH_CC2_ON_VALUE |
                PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_CC2_OVP);
        pd->vconn20_cc2_switch_1_ctrl = regval;

        /* Reset edge detector. */
        pd->vconn20_cc2_switch_1_ctrl |= PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
        pd->vconn20_cc2_switch_1_ctrl &= ~PDSS_VCONN20_CC2_SWITCH_1_CTRL_RST_EDGE_DET;
    }

#if DPM_DEBUG_SUPPORT
    dpm_get_status(port)->cc_stat[channel] = CCG_CC_STAT_ZOPEN;
#endif /* DPM_DEBUG_SUPPORT */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}

bool pd_is_vconn_present(uint8_t port, uint8_t channel)
{
    bool retval = true;

#if (defined(CCG3PA) || defined(CCG3PA2))

    /* No VConn support in CCG3PA family as of now. */

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Check whether the V5V -> VConn Switch is ON. */
    if (channel == CC_CHANNEL_1)
    {
        retval = ((pd->vconn20_cc1_switch_1_ctrl & PDSS_VCONN20_CC1_SWITCH_1_CTRL_SEL_ON_OFF) != 0);
    }
    else
    {
        retval = ((pd->vconn20_cc2_switch_1_ctrl & PDSS_VCONN20_CC2_SWITCH_1_CTRL_SEL_ON_OFF) != 0);
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return retval;
}

ccg_status_t hpd_receive_init(uint8_t port, hpd_event_cbk_t cbk)
{
#if CCG_HPD_RX_ENABLE
    PPDSS_REGS_T pd;

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    if ((hpd_transmit_enable[port]) || (hpd_receive_enable[port]))
    {
        return CCG_STAT_BUSY;
    }

    /* PD block should have been enabled. */
    pd = gl_pdss[port];
    if ((pd->ctrl & PDSS_CTRL_IP_ENABLED) == 0)
    {
        return CCG_STAT_NOT_READY;
    }

    /* Store the callback pointer. */
    hpd_cbks[port] = cbk;
    hpd_receive_enable[port] = true;

    /* Configure the relevant GPIO for HPD functionality. */
    if (port == 0)
    {
        CALL_IN_FUNCTION(gpio_set_drv_mode)(HPD_P0_PORT_PIN, GPIO_DM_HIZ_DIGITAL);
        CALL_IN_FUNCTION(hsiom_set_config)(HPD_P0_PORT_PIN, HPD_HSIOM_SETTING);
    }
    else
    {
        CALL_IN_FUNCTION(gpio_set_drv_mode)(HPD_P1_PORT_PIN, GPIO_DM_HIZ_DIGITAL);
        CALL_IN_FUNCTION(hsiom_set_config)(HPD_P1_PORT_PIN, HPD_HSIOM_SETTING);
    }

    /* Set the default values for the HPD config settings. */
    pd->hpd_ctrl1 = PDSS_HPD_CTRL1_DEFAULT_VALUE;
    pd->hpd_ctrl2 = PDSS_HPD_CTRL2_DEFAULT;
    pd->hpd_ctrl3 = PDSS_HPD_CTRL3_DEFAULT_VALUE;
    pd->hpd_ctrl4 = PDSS_HPD_CTRL4_DEFAULT;
    pd->hpd_ctrl5 = PDSS_HPD_CTRL5_DEFAULT;

    pd->hpd_queue = pd->hpd_queue;

    /* Enable the HPD queue interrupt. */
    pd->intr2 = PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    pd->intr2_mask |= PDSS_INTR2_MASK_HPD_QUEUE_MASK;

    /*
     * CDT 245126 workaround.
     * Enable the HPDIN_CHANGE interrupt with both edge detection. As HPD module
     * can't detect HPD events across deep sleep, this interrupt is used to ensure
     * that device doesn't go back to deepsleep after HPD status changes.
     * HPD RX activity timer is started when this interrupt fires and device
     * doesn't go back to deep sleep until timer is running or a queue interrupt
     * fires.
     */
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    pd->intr1_cfg |= (3 << PDSS_INTR1_CFG_HPDIN_CFG_POS);
    /* Disable HPD IN filter. */
    pd->intr1_cfg &= ~(PDSS_INTR1_CFG_HPDIN_FILT_EN);
#else
    pd->intr_1_cfg |= (3 << PDSS_INTR_1_CFG_HPDIN_CFG_POS);
    /* Disable HPD IN filter. */
    pd->intr_1_cfg &= ~(PDSS_INTR_1_CFG_HPDIN_FILT_EN);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
    /* Clear and then enable the interrupt. */
    pd->intr1 = PDSS_INTR1_HPDIN_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_HPDIN_CHANGED;

    /* Enable the HPD function and the bring the HPD receiver out of reset. */
    pd->ctrl = (pd->ctrl & ~(PDSS_CTRL_HPD_DIRECTION | PDSS_CTRL_HPDT_ENABLED)) |
        PDSS_CTRL_HPD_ENABLED;
    pd->hpd_ctrl1 &= ~(PDSS_HPD_CTRL1_RESET_HPD_STATE);
#endif /* CCG_HPD_RX_ENABLE */

    return CCG_STAT_SUCCESS;
}

ccg_status_t hpd_transmit_init(uint8_t port, hpd_event_cbk_t cbk)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (hpd_transmit_enable[port])
    {
        return CCG_STAT_BUSY;
    }

    /* Store the callback pointer. */
    hpd_cbks[port] = cbk;
    hpd_transmit_enable[port] = true;

    /* Configure the relevant GPIO for HPD functionality. */
    if (port == 0)
    {
        CALL_IN_FUNCTION(gpio_set_drv_mode)(HPD_P0_PORT_PIN, GPIO_DM_STRONG);
        CALL_IN_FUNCTION(hsiom_set_config)((gpio_port_pin_t)HPD_P0_PORT_PIN, (hsiom_mode_t)HPD_HSIOM_SETTING);
    }
    else
    {
        CALL_IN_FUNCTION(gpio_set_drv_mode)(HPD_P1_PORT_PIN, GPIO_DM_STRONG);
        CALL_IN_FUNCTION(hsiom_set_config)((gpio_port_pin_t)HPD_P1_PORT_PIN, (hsiom_mode_t)HPD_HSIOM_SETTING);
    }

    /* Set the default values for the HPDT config settings. */
    pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT;
    pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;

    /* Enable the HPD queue interrupt. */
    pd->intr2 = PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK;
    pd->intr2_mask |= PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK;

    /* Enable the HPDT function and the bring the HPD transmitter out of reset. */
    pd->ctrl = (pd->ctrl & ~(PDSS_CTRL_HPD_ENABLED)) |
        PDSS_CTRL_HPDT_ENABLED | PDSS_CTRL_HPD_DIRECTION;
    pd->hpdt_ctrl1 &= ~(PDSS_HPDT_CTRL1_RESET_HPDT_STATE);

    return CCG_STAT_SUCCESS;
}

void hpd_sleep_entry(uint8_t port)
{
    /* Configure the relevant pin for GPIO functionality. */
    if (hpd_transmit_enable[port])
    {
        if (port == 0)
        {
            CALL_IN_FUNCTION(hsiom_set_config)(HPD_P0_PORT_PIN, HSIOM_MODE_GPIO);
        }
        else
        {
            CALL_IN_FUNCTION(hsiom_set_config)(HPD_P1_PORT_PIN, HSIOM_MODE_GPIO);
        }
    }
}

void hpd_wakeup(uint8_t port, bool value)
{
    PPDSS_REGS_T pd;

    /* PD block should be turned on already. */
    pd = gl_pdss[port];

    /* Configure the relevant GPIO for HPD functionality. */
    if (hpd_transmit_enable[port])
    {
        /* Set the default values for the HPDT config settings. */
        if (value)
        {
            /* Start HPD off in the high state and queue a PLUG event. */
            pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT | PDSS_HPDT_CTRL1_DEFAULT_LEVEL;
            pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;
        }
        else
        {
            /* Start HPD off in the low state. */
            pd->hpdt_ctrl1 = PDSS_HPDT_CTRL1_DEFAULT;
            pd->hpdt_ctrl2 = PDSS_HPDT_CTRL2_DEFAULT;
        }

        /*
         * Bring the HPDT block out of reset. We need a small delay here to
         * ensure that there is no glitch on the HPD line.
         */
        CyDelayUs(5);
        pd->hpdt_ctrl1 &= ~(PDSS_HPDT_CTRL1_RESET_HPDT_STATE);

        if (port == 0)
        {
            CALL_IN_FUNCTION(hsiom_set_config)((gpio_port_pin_t)HPD_P0_PORT_PIN, (hsiom_mode_t)HPD_HSIOM_SETTING);
        }
        else
        {
            CALL_IN_FUNCTION(hsiom_set_config)((gpio_port_pin_t)HPD_P1_PORT_PIN, (hsiom_mode_t)HPD_HSIOM_SETTING);
        }
    }
}

#if CCG_HPD_RX_ENABLE
/*
 * CDT 245126 workaround.
 * This routine implements the workaround before entering deep sleep.
 *
 * Details:
 * HPD RX module can't detect HPD High to Low and IRQ transitions across
 * deepsleep. Workaround is this:
 *
 * Before entering deepsleep, check if HPD is high. If yes, enable HPD
 * TX module with default value of HIGH. Enable HPD loopback. Configure HPD
 * RX to detect a high with minimum time possible (<50us). After this enter
 * deepsleep. Once device enters deep sleep, HPD RX module loses all memory.
 * Now, when HPD goes low, HPD IN wakeup interrupt will wake up the device.
 * HPD RX module starts with HPD RX status as LOW. Due to loopback enabled and HPD
 * TX driving high, HPD RX will see HPD Connect event. Then disable the loopback
 * and revert HPD RX settings. From this point, HPD RX will look at actual
 * HPD status and will capture HPD events correctly.
 *
 * FW also uses a HPD RX activity timer. Whenever HPD IN interrupt fires, start this
 * timer with maximum HPD event time period. This is currently set to 5ms and can be fine
 * tuned if required. This ensures that on any HPD activity, device remains active
 * till the HPD event is captured in HPD RX queue. Stop the timer once HPD Queue
 * interrupt fires.
 */

/*
 * This flag keeps track of HPD Connection status. It is used in HPD CHANGE wakeup
 * interrupt.
 */
bool gl_hpd_state = false;

void hpd_rx_sleep_entry(uint8_t port, bool hpd_state)
{
    PPDSS_REGS_T pd;
    pd = gl_pdss[port];

    /* Store HPD Connection status. */
    gl_hpd_state = hpd_state;

    if (hpd_receive_enable[port])
    {
        /* If HPD is connected, implement the CDT 245126 workaround. */
        if (hpd_state)
        {
            /* Ensure default level of HPD TX is high. */
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_DEFAULT_LEVEL;
            /* Enable HPD TX. */
            pd->ctrl |= PDSS_CTRL_HPDT_ENABLED;

            /* Enable HPD loop back. */
            pd->hpd_ctrl1 |= PDSS_HPD_CTRL1_LOOPBACK_EN;

            /* Update HPD RX Stable High minimum time to a very small value (<50us) */
            pd->hpd_ctrl3 = ((pd->hpd_ctrl3 & ~PDSS_HPD_CTRL3_STABLE_HIGH_MASK)
                | (1 << PDSS_HPD_CTRL3_STABLE_HIGH_POS));
            pd->hpd_ctrl5 = ((pd->hpd_ctrl5 & ~PDSS_HPD_CTRL5_LONG_HIGH_MASK)
                | (1 << PDSS_HPD_CTRL5_LONG_HIGH_POS));

            /*
             * Disable HPD Queue interrupt. There is no point in using Queue interrupt
             * for HPD high detection once device wakes up. We can poll for this.
             */
            pd->intr2 = PDSS_INTR2_HPD_QUEUE;
            pd->intr2_mask &= ~PDSS_INTR2_MASK_HPD_QUEUE_MASK;
        }
    }
}

/* CDT 245126 workaround: This routine implements the wakeup portion of workaround. */
void hpd_rx_wakeup(uint8_t port)
{
    PPDSS_REGS_T pd;
    pd = gl_pdss[port];
    uint8_t timeout = 0;

    if (hpd_receive_enable[port])
    {
        /* Revert the settings only if Loopback was enabled before entering deep sleep. */
        if (pd->hpd_ctrl1 & PDSS_HPD_CTRL1_LOOPBACK_EN)
        {
            /*
             * Wait for the HPD Queue connect event. This is the fake HPD Queue connect
             * event triggered due to CDT 245126 workaround. 20us wait is enough.
             */
            while ((HPD_GET_EVENT_0(pd->hpd_queue) != HPD_EVENT_PLUG) && (timeout < 20))
            {
                CyDelayUs (1);
                timeout++;
            }

            /* Ensure that HPD RX high time is reset back to default. */
            pd->hpd_ctrl3 = PDSS_HPD_CTRL3_DEFAULT_VALUE;
            pd->hpd_ctrl5 = PDSS_HPD_CTRL5_DEFAULT;
            /* Ensure that Loopback is disabled. */
            pd->hpd_ctrl1 &= ~PDSS_HPD_CTRL1_LOOPBACK_EN;
        }
        /* Enable the HPD Queue interrupt to capture true HPD interrupts from now on. */
        pd->intr2 = PDSS_INTR2_HPD_QUEUE;
        pd->intr2_mask |= PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    }
}

bool is_hpd_rx_state_idle(uint8_t port)
{
    /* If timer is running, HPD RX module is busy. */
    return (!timer_is_running (port, HPD_RX_ACTIVITY_TIMER_ID));
}

#endif /* CCG_HPD_RX_ENABLE */

ccg_status_t hpd_deinit(uint8_t port)
{
    PPDSS_REGS_T pd;

    if ((!hpd_transmit_enable[port]) && (!hpd_receive_enable[port]))
    {
        return CCG_STAT_FAILURE;
    }

    if (hpd_transmit_enable[port])
    {
        /* Make sure that the HPD signal is driven to zero. */
        if (port == 0)
        {
            CALL_IN_FUNCTION(gpio_set_value)(HPD_P0_PORT_PIN, 0);
        }
        else
        {
            CALL_IN_FUNCTION(gpio_set_value)(HPD_P1_PORT_PIN, 0);
        }
    }

    /* Disable all HPD related interrupts. */
    pd = gl_pdss[port];
    pd->intr2 = PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK | PDSS_INTR2_MASK_HPD_QUEUE_MASK;
    pd->intr2_mask &= ~(PDSS_INTR2_MASK_HPD_QUEUE_MASK | PDSS_INTR2_MASK_HPDT_COMMAND_DONE_MASK);

    pd->intr1 = PDSS_INTR1_HPDIN_CHANGED;
    pd->intr1_mask &= ~PDSS_INTR1_MASK_HPDIN_CHANGED_MASK;

    /* Disable both HPD transmit and receive blocks. */
    pd->ctrl &= ~(PDSS_CTRL_HPDT_ENABLED | PDSS_CTRL_HPD_ENABLED);

    hpd_cbks[port] = NULL;
    hpd_transmit_enable[port] = false;
    hpd_receive_enable[port] = false;

    if (port == 0)
    {
        CALL_IN_FUNCTION(hsiom_set_config)(HPD_P0_PORT_PIN, HSIOM_MODE_GPIO);
    }
    else
    {
        CALL_IN_FUNCTION(hsiom_set_config)(HPD_P1_PORT_PIN, HSIOM_MODE_GPIO);
    }

    return CCG_STAT_SUCCESS;
}

ccg_status_t hpd_transmit_sendevt(uint8_t port, hpd_event_type_t evtype,
        bool wait)
{
    PPDSS_REGS_T pd;

    /* Wait is currently not supported. */
    (void)wait;

    pd = gl_pdss[port];
    if ((pd->hpdt_ctrl1 & PDSS_HPDT_CTRL1_COMMAND_START) != 0)
    {
        return CCG_STAT_BUSY;
    }

    /* Update HPD-out as required. */
    switch (evtype)
    {
        case HPD_EVENT_UNPLUG:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        case HPD_EVENT_PLUG:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK) | (1 << PDSS_HPDT_CTRL1_COMMAND_POS);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        case HPD_EVENT_IRQ:
            pd->hpdt_ctrl1 = (pd->hpdt_ctrl1 & ~PDSS_HPDT_CTRL1_COMMAND_MASK) | (2 << PDSS_HPDT_CTRL1_COMMAND_POS);
            pd->hpdt_ctrl1 |= PDSS_HPDT_CTRL1_COMMAND_START;
            break;

        default:
            return CCG_STAT_BAD_PARAM;
    }

#ifdef CCGX_CDT225123_WORKAROUND
    /* CDT 225123 Workaround: Wait for 5 us and clear the command start bit. */
    CyDelayUs(5);
    pd->hpdt_ctrl1 &= ~PDSS_HPDT_CTRL1_COMMAND_START;
#endif /* CCGX_CDT225123_WORKAROUND */

    return CCG_STAT_SUCCESS;
}

/******************** PD block ADC functionality *****************************/

uint8_t pd_adc_volt_to_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt)
{
    uint32_t threshold;

    threshold = ((volt * PD_ADC_NUM_LEVELS) / MX_PD_ADC_REF_VOLT_MV);

    if (threshold < PD_ADC_LEVEL_MIN_THRESHOLD)
    {
        threshold = PD_ADC_LEVEL_MIN_THRESHOLD;
    }
    if (threshold > PD_ADC_LEVEL_MAX_THRESHOLD)
    {
        threshold = PD_ADC_LEVEL_MAX_THRESHOLD;
    }

    return (uint8_t)threshold;
}
uint16_t grl_pd_adc_level_to_3v3volt(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level)
{
    uint32_t threshold;

    threshold = ((level * 3300) / PD_ADC_NUM_LEVELS);
    //threshold = ((level * MX_PD_ADC_REF_VOLT_MV) / PD_ADC_NUM_LEVELS);
    return (uint16_t)threshold;
}

uint16_t pd_adc_level_to_volt(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level)
{
    uint32_t threshold;

    threshold = ((level * MX_PD_ADC_REF_VOLT_MV) / PD_ADC_NUM_LEVELS);

    return (uint16_t)threshold;
}

uint16_t pd_adc_get_vbus_voltage(uint8_t port, PD_ADC_ID_T adc_id, uint8_t level)
{
    uint32_t result;
    /* Unused in MX-IP */
    (void)port;
    (void)adc_id;

#ifdef CCG_AMUX_DIV_2X_ENABLE
    result = ((level * MX_PD_ADC_REF_VOLT_MV * pd_vbus_mon_divider)/(PD_ADC_NUM_LEVELS << 1u));
#else
    result = ((level * MX_PD_ADC_REF_VOLT_MV * pd_vbus_mon_divider)/PD_ADC_NUM_LEVELS);
#endif /* CCG_AMUX_DIV_2X_ENABLE */
    return result;
}


uint8_t pd_get_vbus_adc_level(uint8_t port, PD_ADC_ID_T adc_id, uint16_t volt,
        int8_t per)
{
    int32_t threshold;

    threshold = volt;
    threshold = (threshold + (threshold * per) / 100);

    /* Remove negative numbers. */
    if (threshold < 0)
    {
        threshold = 0;
    }

    /* Convert volts to ADC units. */
#ifdef CCG_AMUX_DIV_2X_ENABLE
    return pd_adc_volt_to_level(port, adc_id, (2 * threshold / pd_vbus_mon_divider));
#else
    return pd_adc_volt_to_level(port, adc_id, (threshold / pd_vbus_mon_divider));
#endif /* CCG_AMUX_DIV_2X_ENABLE */
}

ccg_status_t  pd_adc_free_run_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    /* Only one ADC supported by CCG5. */
    (void)adc_id;

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Disable interrupts. */
    pd->intr3_mask &= ~((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);
    pd->intr3 = ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);

    /* Configure ADC */
    pd->adc_ctrl[adc_id] = level | PDSS_ADC_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Disable interrupts. */
    pd->intr3_mask &= ~PDSS_INTR3_CMP_OUT_CHANGED;
    pd->intr3 = PDSS_INTR3_CMP_OUT_CHANGED;

    /* Configure ADC */
    pd->adc_ctrl = level | PDSS_ADC_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}


void pd_adc_comparator_ctrl(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level, PD_ADC_INT_T int_cfg, PD_ADC_CB_T cb)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    bool out;
    /* CCG5 has only one ADC. */
    (void)adc_id;

    state = CyEnterCriticalSection();
    gl_pdss_status[port].adc_cb[adc_id] = cb;

#if (defined(CCG3PA) || defined(CCG3PA2))
    if (cb != NULL)
    {
        pd->intr3_cfg_adc_hs[adc_id] &= ~PDSS_INTR3_CFG_ADC_HS_FILT_CFG_MASK;
        pd->intr3_cfg_adc_hs[adc_id] |= (int_cfg << PDSS_INTR3_CFG_ADC_HS_FILT_CFG_POS);
        pd->adc_ctrl[adc_id] = level | PDSS_ADC_CTRL_ADC_ISO_N |
                        ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
        /* Delay 10us for the input selection to stabilize. */
        CyDelayUs(10);

        /* Clear comparator interrupts. */
        pd->intr3 = ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);

        /* Enable comparator interrupts. */
        pd->intr3_mask |= ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);

        if (pd->adc_ctrl[adc_id] & PDSS_ADC_CTRL_CMP_OUT)
        {
            out = true;
        }
        else
        {
            out = false;
        }
        if (((int_cfg ==  PD_ADC_INT_FALLING) && (out == false)) ||
            ((int_cfg ==  PD_ADC_INT_RISING) && (out == true)))
        {
            /* Raise an interrupt. */
            pd->intr3_set |= (PDSS_INTR3_CMP_OUT_CHANGED_POS << adc_id);
        }
    }
    else
    {
        /* Revert register configuration. */
        pd->adc_ctrl[adc_id] = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;

        /* Disable interrupts. */
        pd->intr3_mask &= ~((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);
        pd->intr3 = ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);

    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    if (cb != NULL)
    {
        pd->intr3_cfg_adc_hs &= ~PDSS_INTR3_CFG_ADC_HS_FILT_CFG_MASK;
        pd->intr3_cfg_adc_hs |= (int_cfg << PDSS_INTR3_CFG_ADC_HS_FILT_CFG_POS);
        pd->adc_ctrl = level | PDSS_ADC_CTRL_ADC_ISO_N |
                        ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
        /* Delay 10us for the input selection to stabilize. */
        CyDelayUs(10);

        /* Clear comparator interrupts. */
        pd->intr3 = PDSS_INTR3_CMP_OUT_CHANGED;

        /* Enable comparator interrupts. */
        pd->intr3_mask |= PDSS_INTR3_CMP_OUT_CHANGED;

        if (pd->adc_ctrl & PDSS_ADC_CTRL_CMP_OUT)
        {
            out = true;
        }
        else
        {
            out = false;
        }
        if (((int_cfg ==  PD_ADC_INT_FALLING) && (out == false)) ||
            ((int_cfg ==  PD_ADC_INT_RISING) && (out == true)))
        {
            /* Raise an interrupt. */
            pd->intr3_set |= PDSS_INTR3_CMP_OUT_CHANGED;
        }
    }
    else
    {
        /* Revert register configuration. */
        pd->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;

        /* Disable interrupts. */
        pd->intr3_mask &= ~(PDSS_INTR3_CMP_OUT_CHANGED);
        pd->intr3 = PDSS_INTR3_CMP_OUT_CHANGED;

    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    CyExitCriticalSection(state);
}

static void pd_adc_restore_intr(uint8_t port, PD_ADC_ID_T adc_id, uint32_t reg_adc_ctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t rval;
    bool out;
    (void)adc_id;

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Revert register configuration. */
    pd->adc_ctrl[adc_id] = reg_adc_ctrl;
    CyDelayUs(10);

    pd->intr3 = (PDSS_INTR3_CMP_OUT_CHANGED_POS << adc_id);

    if (((pd->intr3_mask & ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id)) != 0) &&
        ((reg_adc_ctrl & PDSS_ADC_CTRL_PD_LV) == 0))
    {
        rval = (pd->intr3_cfg_adc_hs[adc_id] & PDSS_INTR3_CFG_ADC_HS_FILT_CFG_MASK) >>
            PDSS_INTR3_CFG_ADC_HS_FILT_CFG_POS;

        if (pd->adc_ctrl[adc_id] & PDSS_ADC_CTRL_CMP_OUT)
        {
            out = true;
        }
        else
        {
            out = false;
        }

        if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
            ((rval ==  PD_ADC_INT_RISING) && (out == true)))
        {
            /* Raise an interrupt. */
            pd->intr3_set |= ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << adc_id);
        }
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Revert register configuration. */
    pd->adc_ctrl = reg_adc_ctrl;
    CyDelayUs(10);

    pd->intr3 = PDSS_INTR3_CMP_OUT_CHANGED;

    if (((pd->intr3_mask & PDSS_INTR3_CMP_OUT_CHANGED) != 0) &&
        ((reg_adc_ctrl & PDSS_ADC_CTRL_PD_LV) == 0))
    {
        rval = (pd->intr3_cfg_adc_hs & PDSS_INTR3_CFG_ADC_HS_FILT_CFG_MASK) >>
            PDSS_INTR3_CFG_ADC_HS_FILT_CFG_POS;

        if (pd->adc_ctrl & PDSS_ADC_CTRL_CMP_OUT)
        {
            out = true;
        }
        else
        {
            out = false;
        }

        if (((rval ==  PD_ADC_INT_FALLING) && (out == false)) ||
            ((rval ==  PD_ADC_INT_RISING) && (out == true)))
        {
            /* Raise an interrupt. */
            pd->intr3_set |= PDSS_INTR3_CMP_OUT_CHANGED;
        }
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
}

bool pd_adc_comparator_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input,
        uint8_t level)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t reg_adc_ctrl;
    uint8_t state;
    uint8_t comp_out = true;
    (void)adc_id;

    /* Store previous configuration and disable interrupts. */
    state = CyEnterCriticalSection();

#if (defined(CCG3PA) || defined(CCG3PA2))
    reg_adc_ctrl = pd->adc_ctrl[adc_id];

    /* Configure the input and level. */
    pd->adc_ctrl[adc_id] = level | PDSS_ADC_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);

    /* Delay 10us for the input selection to stabilize. */
    CyDelayUs(10);

    if (pd->adc_ctrl[adc_id] & PDSS_ADC_CTRL_CMP_OUT)
    {
        comp_out = false;
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    reg_adc_ctrl = pd->adc_ctrl;

    /* Configure the input and level. */
    pd->adc_ctrl = level | PDSS_ADC_CTRL_ADC_ISO_N |
            ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);

    /* Delay 10us for the input selection to stabilize. */
    CyDelayUs(10);

    if (pd->adc_ctrl & PDSS_ADC_CTRL_CMP_OUT)
    {
        comp_out = false;
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    pd_adc_restore_intr(port, adc_id, reg_adc_ctrl);
    CyExitCriticalSection(state);

    return comp_out;
}

bool pd_adc_get_comparator_status(uint8_t port, PD_ADC_ID_T adc_id)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    (void)adc_id;

#if (defined(CCG3PA) || defined(CCG3PA2))
    if (pd->adc_ctrl[adc_id] & PDSS_ADC_CTRL_CMP_OUT)
    {
        return false;
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    if (pd->adc_ctrl & PDSS_ADC_CTRL_CMP_OUT)
    {
        return false;
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return true;
}

uint8_t pd_adc_sample(uint8_t port, PD_ADC_ID_T adc_id, PD_ADC_INPUT_T input)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t volatile timeout = 0;
    uint8_t level = 0;
    uint32_t reg_adc_ctrl;
    (void)adc_id;

    /* Store previous configuration and disable interrupts. */
    state = CyEnterCriticalSection();

#if (defined(CCG3PA) || defined(CCG3PA2))
    reg_adc_ctrl = pd->adc_ctrl[adc_id];

    /* Configure the input. */
    pd->adc_ctrl[adc_id] = PDSS_ADC_CTRL_ADC_ISO_N |
        ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
    pd->adc_sar_ctrl[adc_id] |= PDSS_ADC_SAR_CTRL_SAR_EN;

    /* Wait for sampling to complete or timeout. */
    while (((pd->adc_sar_ctrl[adc_id] & PDSS_ADC_SAR_CTRL_SAR_EN) != 0) && (timeout < PD_ADC_TIMEOUT_COUNT))
    {
        timeout++;
    }

    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    CyDelayUs(2);

    level = ((pd->adc_sar_ctrl[adc_id] & PDSS_ADC_SAR_CTRL_SAR_OUT_MASK) >>
            PDSS_ADC_SAR_CTRL_SAR_OUT_POS);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    reg_adc_ctrl = pd->adc_ctrl;

    /* Configure the input. */
    pd->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N |
        ((input << PDSS_ADC_CTRL_VSEL_POS) & PDSS_ADC_CTRL_VSEL_MASK);
    pd->adc_sar_ctrl |= PDSS_ADC_SAR_CTRL_SAR_EN;

    /* Wait for sampling to complete or timeout. */
    while (((pd->adc_sar_ctrl & PDSS_ADC_SAR_CTRL_SAR_EN) != 0) && (timeout < PD_ADC_TIMEOUT_COUNT))
    {
        timeout++;
    }

    /* Delay required between SAR_EN bit to be cleared and value to be loaded. */
    CyDelayUs(2);

    level = ((pd->adc_sar_ctrl & PDSS_ADC_SAR_CTRL_SAR_OUT_MASK) >>
            PDSS_ADC_SAR_CTRL_SAR_OUT_POS);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    pd_adc_restore_intr(port, adc_id, reg_adc_ctrl);
    CyExitCriticalSection(state);

    return level;
}

uint16_t pd_adc_calibrate(uint8_t port, PD_ADC_ID_T adc_id)
{
    /* Do nothing: Legacy API, MX-PD IP has bandgap based reference */
    (void)port;
    (void)adc_id;
    return 0;
}

ccg_status_t pd_adc_init(uint8_t port, PD_ADC_ID_T adc_id)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    (void)adc_id;

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Enable the ADC and power it down. */
    pd->adc_ctrl[adc_id] = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    /* Enable the ADC and power it down. */
    pd->adc_ctrl = PDSS_ADC_CTRL_ADC_ISO_N | PDSS_ADC_CTRL_PD_LV;
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}

bool pd_frs_rx_enable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

#if !(defined (CCG5) || defined(CCG6) || defined(CCG5C))
#error "FRS support is available only for CCG5/CCG6 on this IP."
#endif

    /* Set the VBus detach comparator threshold to Vsafe5V */
    uint8_t level = pd_get_vbus_adc_level (port, pd_hal_get_vbus_detach_adc(),
            VSAFE_5V, VSAFE_5V_FRS_SWAP_RX_MARGIN);

    /* Enable VSAFE5V comparator */
    pd_adc_free_run_ctrl(port, pd_hal_get_vbus_detach_adc(), pd_hal_get_vbus_detach_input(), level);

    /* Configure CC line voltage thresholds to detect frs signal */
    regval  = (pd->cc_ctrl_1 & ~(PDSS_CC_CTRL_1_CMP_FS_VSEL_MASK | PDSS_CC_CTRL_1_CMP_FS_CC1V2));
    regval |= (CMP_FS_VSEL_VALUE << PDSS_CC_CTRL_1_CMP_FS_VSEL_POS);

    /* See if cc polarity need update */
    if(dpm_stat->polarity == CC_CHANNEL_2)
    {
        regval |= PDSS_CC_CTRL_1_CMP_FS_CC1V2;
    }
    pd->cc_ctrl_1 = regval;

    /*
     * Set the vsafe5v comp signal source:
     * Using VBUS_MON. Also clear Rx Swap Done status.
     */
    pd->swap_ctrl0 = (pd_hal_get_vbus_detach_adc() << PDSS_SWAP_CTRL0_SWAPR_SOURCE_SEL_POS) |
        (1 << PDSS_SWAP_CTRL0_RX_SWAP_SOURCE_POS) |
        PDSS_SWAP_CTRL0_CLR_RX_SWAP_DONE;

    /* Now configure the Swap controller */
    pd->swap_ctrl1 = FRS_RX_SWAP_CTRL1_DFLT_VAL;
    pd->swap_ctrl2 = FRS_RX_SWAP_CTRL2_DFLT_VAL;
    pd->swap_ctrl3 = FRS_RX_SWAP_CTRL3_DFLT_VAL;
    pd->swap_ctrl5 = FRS_RX_SWAP_CTRL5_DFLT_VAL;

    /* Let thresholds settle */
    CyDelayUs(10);

    /* Take swap controller out of reset */
    pd->swap_ctrl1 &= ~PDSS_SWAP_CTRL1_RESET_SWAP_STATE;

    /* Enabling the FR pulse and disconnect interrupts. FET switching will only happen on pulse. */
    pd->intr2 = PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT;
    pd->intr2_mask |= PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT;

    pd->intr1 = PDSS_INTR1_VSWAP_VBUS_LESS_5_DONE;

    /* Enable the swap controller */
    pd->swap_ctrl0 &= ~PDSS_SWAP_CTRL0_CLR_RX_SWAP_DONE;
    pd->swap_ctrl0 |= PDSS_SWAP_CTRL0_SWAP_ENABLED;

    /* Ensure that the FET switching happens based on SWAP IRQ and VBUS Detect. */
    pd->debug_cc_0 |= PDSS_DEBUG_CC_0_VBUS_C_SWAP_SOURCE_SEL | PDSS_DEBUG_CC_0_VBUS_P_SWAP_SOURCE_SEL;

#if defined(CCG5)

    /* Set the sink fet OFF settings as per current HW, and enable LF filter 0 for auto FET switching. */
    regval = pd->pgdo_1_cfg[0];
    regval |= ((1 << PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_POS) | PDSS_PGDO_1_CFG_AUTO_MODE);
    pd->pgdo_1_cfg[0]  = regval;

    /* Ensure that source FET is off to start with, and then configure it for turn ON on FR trigger. */
    pd_internal_pfet_off (port, false);
    regval = pd->pgdo_1_cfg[1];
    regval |= (PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE |
            (1 << PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_POS) |
            PDSS_PGDO_1_CFG_AUTO_MODE);
    pd->pgdo_1_cfg[1] = regval;

#elif (defined(CCG5C) || defined (CCG6))

    /* Set the sink fet OFF settings as per current HW, and enable LF filter 0 for auto FET switching. */
    regval = pd->pgdo_1_cfg;
    regval |= ((1 << PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_POS) | PDSS_PGDO_1_CFG_AUTO_MODE);
    pd->pgdo_1_cfg  = regval;

    /* Ensure that source FET is off to start with, and then configure it for turn ON on FR trigger. */
    pd_internal_pfet_off (port, false);

    /* CCG6 uses PGDO_PU_1_CFG register for Provider FET */
    regval = pd->pgdo_pu_1_cfg;
    regval |= (PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE |
            (1 << PDSS_PGDO_PU_1_CFG_SEL_SWAP_VBUS_LESS_5_POS) |
            PDSS_PGDO_PU_1_CFG_AUTO_MODE);
    pd->pgdo_pu_1_cfg = regval;

#ifdef CCG6
    /* On CCG6, we can use the ISINK control of the gate driver to slowly enable the provider path instead of
     * doing an instantanous turn-on. Programming 0x120 value to the ID SINK counter in bypass mode. */

    /* First clear the current ISINK counter value. */
    pd->pgdo_pd_isnk_cfg &= ~(PDSS_PGDO_PD_ISNK_CFG_VALUE_MASK | PDSS_PGDO_PD_ISNK_CFG_VALUE_1_MASK |
            PDSS_PGDO_PD_ISNK_CFG_STRONG_EN);
    CyDelayUs (50);
    pd->pgdo_pd_isnk_cfg |= PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE;
    CyDelayUs (100);
    while ((pd->pgdo_pd_isnk_cfg & PDSS_PGDO_PD_ISNK_CFG_LOAD_INIT_VALUE) != 0);

    pd->pgdo_pd_isnk_cfg &= ~PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
    pd->pgdo_pd_isnk_cfg |= (0x20 << PDSS_PGDO_PD_ISNK_CFG_VALUE_POS) |
        (0x04 << PDSS_PGDO_PD_ISNK_CFG_VALUE_1_POS) |
        PDSS_PGDO_PD_ISNK_CFG_BYPASS_PD_ISNK;
#endif /* CCG6*/

#endif /* CCG5/CCG5C/CCG6 */
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

    return true;
}

bool pd_frs_rx_disable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
#if !(defined (CCG5) || defined(CCG6) || defined(CCG5C))
#error "FRS support is available only for CCG5/CCG6 on this IP."
#endif

    PPDSS_REGS_T pd = gl_pdss[port];

    pd_adc_comparator_ctrl (port, pd_hal_get_vbus_detach_adc(), PD_ADC_INPUT_AMUX_A, 0, PD_ADC_INT_DISABLED, NULL);

    /* Disable the swap controller */
    pd->swap_ctrl1 |= PDSS_SWAP_CTRL1_RESET_SWAP_STATE;
    pd->swap_ctrl0 &= ~PDSS_SWAP_CTRL0_SWAP_ENABLED;

    /* Disable and clear frs receive interrupts */
    pd->intr1_mask &= ~(PDSS_INTR1_VSWAP_VBUS_LESS_5_DONE);
    pd->intr1 = (PDSS_INTR1_VSWAP_VBUS_LESS_5_DONE);

    pd->intr2_mask &= ~(PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT);
    pd->intr2 = (PDSS_INTR2_SWAP_RCVD | PDSS_INTR2_SWAP_DISCONNECT);

#if defined(CCG5)
    /* Clear the switch on SWAP bits in the gate driver registers. */
    pd->pgdo_1_cfg[0] &= ~PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK;
    pd->pgdo_2_cfg[0] &= ~(1 << PDSS_PGDO_2_CFG_LS_SOURCE_SEL_POS);
    pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK;
    pd->pgdo_2_cfg[1] &= ~(1 << PDSS_PGDO_2_CFG_LS_SOURCE_SEL_POS);

    /* Make sure auto-mode is turned off on both the provider and consumer gate drivers. */
    if (((pd->pgdo_1_cfg[1] & PGDO_1_CFG_AUTO_SEL_MASK) == 0) && (pd->pgdo_2_cfg[1] == 0))
    {
        pd_reset_edge_det(port, true);
        pd->pgdo_1_cfg[1] &= ~(PDSS_PGDO_1_CFG_AUTO_MODE);
    }

    if (((pd->pgdo_1_cfg[0] & PGDO_1_CFG_AUTO_SEL_MASK) == 0) && (pd->pgdo_2_cfg[0] == 0))
    {
        pd_reset_edge_det(port, false);
        pd->pgdo_1_cfg[0] &= ~(PDSS_PGDO_1_CFG_AUTO_MODE);
    }
#elif (defined(CCG5C) || defined (CCG6))
    /* Clear the switch on SWAP bits in the gate driver registers. */
    pd->pgdo_1_cfg &= ~PDSS_PGDO_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK;
    pd->pgdo_2_cfg &= ~(1 << PDSS_PGDO_2_CFG_LS_SOURCE_SEL_POS);

    /* CCG5C/CCG6 uses PGDO_PU_1_CFG register for Provider FET */
    pd->pgdo_pu_1_cfg &= ~PDSS_PGDO_PU_1_CFG_SEL_SWAP_VBUS_LESS_5_MASK;
    pd->pgdo_pu_2_cfg &= ~(1 << PDSS_PGDO_PU_2_CFG_LS_SOURCE_SEL_POS);

    /* Make sure auto-mode is turned off on both the provider and consumer gate drivers. */
    if (((pd->pgdo_pu_1_cfg & PGDO_PU_1_CFG_AUTO_SEL_MASK) == 0) && (pd->pgdo_pu_2_cfg == 0))
    {
        pd_reset_edge_det(port, true);
        pd->pgdo_pu_1_cfg &= ~(PDSS_PGDO_PU_1_CFG_AUTO_MODE);
    }

    if (((pd->pgdo_1_cfg & PGDO_1_CFG_AUTO_SEL_MASK) == 0) && (pd->pgdo_2_cfg == 0))
    {
        pd_reset_edge_det(port, false);
        pd->pgdo_1_cfg &= ~(PDSS_PGDO_1_CFG_AUTO_MODE);
    }
#endif /*CCG5/CCG5C/CCG6 */

    /* Restore FET switch condition to default. */
    pd->debug_cc_0 &= ~(PDSS_DEBUG_CC_0_VBUS_C_SWAP_SOURCE_SEL | PDSS_DEBUG_CC_0_VBUS_P_SWAP_SOURCE_SEL);

#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

    return true;
}

bool pd_frs_tx_enable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
    dpm_status_t* dpm_stat = dpm_get_status(port);
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    /* Configure FRS TX source */
    if(port == TYPEC_PORT_0_IDX)
    {
        /* Configuring a GPIO for trigering FRS signal */
        CALL_IN_FUNCTION(gpio_hsiom_set_config)(APP_FRS_TX_GPIO_PORT_PIN_P1, HSIOM_MODE_P0_SWAPT_IN,
                GPIO_DM_HIZ_DIGITAL, 0);
    }
#if CCG_PD_DUALPORT_ENABLE
    if (port == TYPEC_PORT_1_IDX)
    {
        /* Configuring a GPIO for trigering FRS signal */
        CALL_IN_FUNCTION(gpio_hsiom_set_config)(APP_FRS_TX_GPIO_PORT_PIN_P2, HSIOM_MODE_P1_SWAPT_IN,
                GPIO_DM_HIZ_DIGITAL, 0);
    }
#endif /* CCG_PD_DUALPORT_ENABLE */

    /* Configure for Auto FRS signal transmitting */
    regval = (pd->debug_cc_1 & ~(PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC1 |
                PDSS_DEBUG_CC_1_PFET300_PULLDN_EN_CC2 |
                PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN |
                PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN));

    /* Enable TX discard on swap */
    regval |= PDSS_TX_STOP_ON_SWAP_MASK;

     /* Set cc polarity for pulldowns */
    if(dpm_stat->polarity == CC_CHANNEL_2)
    {
        regval |= (PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN);
    }
    else
    {
        regval |= (PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN);
    }
    pd->debug_cc_1 = regval;

    pd->swap_ctrl0 = PDSS_SWAP_CTRL0_SWAP_ENABLED;
    pd->swap_ctrl0 |= FRS_TX_SOURCE_GPIO;

    pd->swap_ctrl0 |= PDSS_SWAP_CTRL0_SWAPT_POLARITY;
    pd->swapt_ctrl1 = FRS_TX_SWAP_CTRL1_DFLT_VAL;

    /* Enable necessary interrupts */
    pd->intr2 = PDSS_INTR2_SWAP_COMMAND_DONE;
    pd->intr2_mask |= PDSS_INTR2_SWAP_COMMAND_DONE;

    /* This delay is needed otherwise swap TX indefinitely short the cc line */
    CyDelayUs(10);

    /* Enable the swap tx  */
    pd->swapt_ctrl1 &= ~PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE;
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */

    return true;
}

bool pd_frs_tx_disable(uint8_t port)
{
#if (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE)
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable the swap controller */
    pd->swapt_ctrl1 = PDSS_SWAPT_CTRL1_RESET_SWAPT_STATE;
    pd->swap_ctrl0 &= ~PDSS_SWAP_CTRL0_SWAP_ENABLED;

    /* Disable frs receive interrupts */
    pd->intr2_mask &= ~PDSS_INTR2_SWAP_COMMAND_DONE;
    pd->intr2 = PDSS_INTR2_SWAP_COMMAND_DONE;

    /* Disable pulldown */
    pd->debug_cc_1 &= ~(PDSS_DEBUG_CC_1_SWAPT_TO_CC1_EN |
                PDSS_DEBUG_CC_1_SWAPT_TO_CC2_EN);
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_TX_ENABLE) */

    return true;
}

#if (VBUS_OCP_ENABLE)
static void ocp_handler_wrapper(uint8_t port, timer_id_t id)
{
    /* OCP debounced. Invoke callback. */
    vbus_ocp_handler(port);
}
#endif /* VBUS_OCP_ENABLE */

#if VCONN_OCP_ENABLE
static void vconn_ocp_timer_cb(uint8_t port, timer_id_t id)
{
    if (id == PD_VCONN_OCP_DEBOUNCE_TIMER)
    {
        /* VConn OCP debounced. Deliver callback to app layer. */
        if (gl_vconn_ocp_cb[port] != NULL)
        {
            gl_vconn_ocp_cb[port] (port, true);
        }
    }
}
#endif /* VCONN_OCP_ENABLE */

#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))
#if CCGX_V5V_CHANGE_DETECT
static void ccg5_v5v_supply_debounce_cb (uint8_t port, timer_id_t id)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    if (ccg5_v5v_supply_present[port])
    {
        if ((pd->intr1_status & PDSS_INTR1_STATUS_V5V_STATUS) != 0)
        {
            gl_ccg_supply_changed_cb (port, CCG_SUPPLY_V5V, true);
        }
        else
        {
            ccg5_v5v_supply_present[port] = false;
            timer_start (port, id, 500, ccg5_v5v_supply_debounce_cb);
        }
    }
    else
    {
        if ((pd->intr1_status & PDSS_INTR1_STATUS_V5V_STATUS) == 0)
        {
            gl_ccg_supply_changed_cb (port, CCG_SUPPLY_V5V, false);
        }
        else
        {
            ccg5_v5v_supply_present[port] = true;
            timer_start (port, id, 500, ccg5_v5v_supply_debounce_cb);
        }
    }
}
#endif /* CCGX_V5V_CHANGE_DETECT */
#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */

#if VBUS_OCP_ENABLE
/* Defining macros for registers and fields used in common OCP interrupt handler for CCG5, CCG5C and CCG6. */
#if ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6)))

#ifdef CCG5

#define CCG_OCP_INTR_REQ                PDSS_INTR3_CSA_OC_CHANGED
#define CCG_OCP_INTR_MASK               PDSS_INTR3_MASK_CSA_OC_CHANGED_MASK
#define CCG_OCP_INTR_SET                PDSS_INTR3_SET_CSA_OC_CHANGED
#define CCG_OCP_LIVE_STATUS             PDSS_INTR3_STATUS_0_CSA_OC_FILT
#define CCG_OCP_INTR_CFG_MASK           PDSS_INTR3_CFG_CSA_OC_HS_FILT_CFG_MASK
#define CCG_OCP_INTR_CFG_POS            PDSS_INTR3_CFG_CSA_OC_HS_FILT_CFG_POS

#define CCG_OCP_INT_ACT_REG(pd)         (pd)->intr3_masked
#define CCG_OCP_INT_REQ_REG(pd)         (pd)->intr3
#define CCG_OCP_INT_MSK_REG(pd)         (pd)->intr3_mask
#define CCG_OCP_INT_SET_REG(pd)         (pd)->intr3_set
#define CCG_OCP_STATUS_REG(pd)          (pd)->intr3_status_0
#define CCG_OCP_CFG_REG(pd)             (pd)->intr3_cfg_csa_oc_hs

#else /* (defined(CCG5C) || defined(CCG6)) */

#define CCG_OCP_INTR_REQ                PDSS_INTR13_CSA_OCP_CHANGED
#define CCG_OCP_INTR_MASK               PDSS_INTR13_MASK_CSA_OCP_CHANGED_MASK
#define CCG_OCP_INTR_SET                PDSS_INTR13_SET_CSA_OCP_CHANGED
#define CCG_OCP_LIVE_STATUS             PDSS_INTR13_STATUS_CSA_OCP_FILT
#define CCG_OCP_INTR_CFG_MASK           PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_CFG_MASK
#define CCG_OCP_INTR_CFG_POS            PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_CFG_POS

#define CCG_OCP_INT_ACT_REG(pd)         (pd)->intr13_masked
#define CCG_OCP_INT_REQ_REG(pd)         (pd)->intr13
#define CCG_OCP_INT_MSK_REG(pd)         (pd)->intr13_mask
#define CCG_OCP_INT_SET_REG(pd)         (pd)->intr13_set
#define CCG_OCP_STATUS_REG(pd)          (pd)->intr13_status
#define CCG_OCP_CFG_REG(pd)             (pd)->intr13_cfg_csa_scp_hs

#endif /* (defined(CCG5C) || defined(CCG6)) */

#endif /* ((defined(CCG5)) || (defined(CCG5C)) || (defined(CCG6))) */
#endif /* VBUS_OCP_ENABLE */

/*
 * Handle all wake-up interrupt sources.
 * INTR1/INTR3/INTR5 etc are mapped to the wake-up interrupt vector.
 */
void pdss_intr1_handler(uint8_t port)
{
    bool comp_out = true;
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];
    uint32_t intr1_cause = pd->intr1_masked;
    pd->intr1_mask &= ~intr1_cause;
    pd->intr1 = intr1_cause;

#ifdef CCG6

#if VBUS_SCP_ENABLE
    /*
     * SCP interrupt handling for CCG6.
     * Provider FET would have been turned off. We only need to clear the interrupt and start recovery.
     */
    if (pd->intr13_masked & (PDSS_INTR13_MASKED_CSA_SCP_CHANGED_MASKED))
    {
        /* Make sure P_CTRL does not get pulled up on VBus side. */
        pd->pgdo_pu_3_cfg = PDSS_PGDO_PU_3_CFG_PGDO_EN_SCP_LV;

        /* Clear and disable interrupt. */
        pd->intr13_mask &= ~(PDSS_INTR13_MASKED_CSA_SCP_CHANGED_MASKED);
        pd->intr13 = PDSS_INTR13_MASKED_CSA_SCP_CHANGED_MASKED;

        /* Reset PGDO edge detector logic. */
        pd_reset_edge_det (port, true);
        vbus_scp_handler (port);
    }
#endif /* VBUS_SCP_ENABLE */

#if VBUS_RCP_ENABLE
    /*
     * RCP interrupt handling for CCG6.
     * Provider FET would have been turned off. We only need to clear the interrupt and start recovery.
     */
    if (
            (pd->intr13_masked & (PDSS_INTR13_MASKED_CSA_OUT_CHANGED_MASKED)) ||
            (pd->intr13_masked & (PDSS_INTR13_MASKED_CSA_COMP_OUT_CHANGED_MASKED)) ||
            (pd->intr13_masked & (PDSS_INTR13_MASKED_CSA_VBUS_OVP_CHANGED_MASKED))
       )
    {
        /* Make sure P_CTRL does not get pulled up on Vin side. */
        pd->pgdo_pu_3_cfg = (PDSS_PGDO_PU_3_CFG_PGDO_EN_RCP_LV | PDSS_PGDO_PU_3_CFG_PGDO_EN_VBUS_OV_LV);

        /* Clear and disable all RCP related interrupts. */
        pd->intr13_mask &= ~(PDSS_INTR13_MASKED_CSA_OUT_CHANGED_MASKED |
                PDSS_INTR13_MASKED_CSA_COMP_OUT_CHANGED_MASKED |
                PDSS_INTR13_MASKED_CSA_VBUS_OVP_CHANGED_MASKED);
        pd->intr13 = (PDSS_INTR13_MASKED_CSA_OUT_CHANGED_MASKED |
                PDSS_INTR13_MASKED_CSA_COMP_OUT_CHANGED_MASKED |
                PDSS_INTR13_MASKED_CSA_VBUS_OVP_CHANGED_MASKED);

        /* Reset PGDO edge detector logic. */
        pd_reset_edge_det (port, true);
        vbus_rcp_handler (port);
    }
#endif /* VBUS_RCP_ENABLE */

#endif /* CCG6 */

    /*
     * This routine expects all interrupts which are triggered to be disabled
     * once they are fired, otherwise it can cause problems.
     */
    if (intr1_cause)
    {
        if (intr1_cause & PDSS_INTR1_HPDIN_CHANGED)
        {
#if CCG_HPD_RX_ENABLE
            /*
             * Start HPD ACTIVITY TIMER to prevent re-entry into deepsleep.
             * HPD RX hardware block can't detect HPD events if device enters
             * deepsleep while HPD state is changing (or if HPD is HIGH).
             * If HPD is not connected, timer period shall be 100ms
             * (this is the default minimum HPD Connect debounce time).
             * Otherwise, timer period is 5ms (this is large enough
             * to capture HPD LOW and IRQ events.
             */
            if (gl_hpd_state == false)
            {
                /* Start a timer of 100ms to prevent deep sleep entry. */
                timer_start_wocb(port, HPD_RX_ACTIVITY_TIMER_ID,
                        HPD_RX_ACTIVITY_TIMER_PERIOD_MAX);
            }
            else
            {
                timer_start_wocb(port, HPD_RX_ACTIVITY_TIMER_ID,
                        HPD_RX_ACTIVITY_TIMER_PERIOD_MIN);
            }
#endif /* CCG_HPD_RX_ENABLE */
        }

#if (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE)
        if (intr1_cause & PDSS_INTR1_VSWAP_VBUS_LESS_5_DONE)
        {
#if (defined (CCG5))
            /*
               Clear the AUTO mode on the FETs without affecting their current state. We can
               only do this by assuming that the consumer FET is OFF and the provider FET is ON.
               Please note that it is not possible to handle OV/OC errors while going through
               the Fast Role Transition.
             */
            pd->pgdo_1_cfg[0] &= ~(PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
            pd->pgdo_1_cfg[0] &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
            pd_reset_edge_det(port, false);

            pd->pgdo_1_cfg[1] |= (PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
            CyDelayUs (1);
            pd_reset_edge_det(port, true);

            pd->pgdo_1_cfg[1] |= PDSS_PGDO_1_CFG_SEL_ON_OFF;
            CyDelayUs (1);
            pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
            CyDelayUs (1);
            pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;

#elif (defined(CCG5C) || defined(CCG6))
            uint32_t regval;

            /*
               Clear the AUTO mode on the FETs without affecting their current state. We can
               only do this by assuming that the consumer FET is OFF and the provider FET is ON.
               Please note that it is not possible to handle OV/OC errors while going through
               the Fast Role Transition.
             */
            pd->pgdo_1_cfg &= ~(PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
            pd->pgdo_1_cfg &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
            pd_reset_edge_det(port, false);
            /* CCG6 uses PGDO_PU_1_CFG register for Provider FET */
            pd->pgdo_pu_1_cfg &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
            pd->pgdo_pu_1_cfg |= PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE;
            CyDelayUs (1);

            pd_reset_edge_det(port, true);

            /* CCG6 uses PGDO_PU_1_CFG register for Provider FET */
            regval = pd->pgdo_pu_1_cfg;
            regval &= ~(PDSS_PGDO_PU_1_CFG_AUTO_MODE | PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_ON_VALUE);
            regval |= PDSS_PGDO_PU_1_CFG_SEL_ON_OFF;
            pd->pgdo_pu_1_cfg = regval;

#endif /* (defined(CCG5C) || defined(CCG6)) */

            /*
             * Remember the fact that the provider FET is on so that a subsequent
             * soft start does not cause supply to be turned off.
             */
            gl_ccgx_pfet_on[port] = true;

            CyDelayUs (1);
            pd_frs_rx_disable(port);
        }
#endif /* (CCG_PD_REV3_ENABLE & CCG_FRS_RX_ENABLE) */

#if SYS_DEEPSLEEP_ENABLE
        if(intr1_cause & PDSS_INTR1_DRP_ATTACHED_DETECTED)
        {
            /* Auto toggle has been stopped. Also, mark Type-C restart pending. */
            gl_pdss_status[port].auto_toggle_act     = false;
            gl_pdss_status[port].typec_start_pending = true;
        }
#endif /* SYS_DEEPSLEEP_ENABLE */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
#if VCONN_OCP_ENABLE
        if (intr1_cause & (PDSS_INTR1_CC1_OCP_CHANGED | PDSS_INTR1_CC2_OCP_CHANGED))
        {
            uint32_t regval;
            uint32_t cfg_mask, cfg_pos;
            uint32_t stat_mask;

            /* Disable the interrupt to start with. */
            pd->intr1_mask &= ~(PDSS_INTR1_CC1_OCP_CHANGED | PDSS_INTR1_CC2_OCP_CHANGED);

            /* Identify the register fields of interest. */
            if (gl_vconn_channel[port])
            {
                cfg_mask  = PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_FILT_CFG_MASK;
                cfg_pos   = PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_FILT_CFG_POS;
                stat_mask = PDSS_INTR1_STATUS_CC2_OCP_FILT;
            }
            else
            {
                cfg_mask  = PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_FILT_CFG_MASK;
                cfg_pos   = PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_FILT_CFG_POS;
                stat_mask = PDSS_INTR1_STATUS_CC1_OCP_FILT;
            }

            /* If positive edge interrupt: Current exceeded positive threshold. */
            if (((pd->intr1_cfg_cc12_ocp_hs & cfg_mask) >> cfg_pos) == FILTER_CFG_POS_EN_NEG_DIS)
            {
                /* Look for negative edge of comparator. */
                regval = pd->intr1_cfg_cc12_ocp_hs;
                regval &= ~cfg_mask;
                regval |= FILTER_CFG_POS_DIS_NEG_EN << cfg_pos;
                pd->intr1_cfg_cc12_ocp_hs = regval;

                /* If the negative edge happened already, queue the interrupt. */
                if ((pd->intr1_status & stat_mask) == 0)
                {
                    pd->intr1_set = (gl_vconn_channel[port] ?
                            PDSS_INTR1_SET_CC2_OCP_CHANGED : PDSS_INTR1_SET_CC1_OCP_CHANGED);
                }

                /* Start the debounce timer. */
                timer_start(port, PD_VCONN_OCP_DEBOUNCE_TIMER, gl_vconn_ocp_debounce[port], vconn_ocp_timer_cb);
            }
            /* If negative edge interrupt: Current is back within limit. */
            else
            {
                if (timer_is_running (port, PD_VCONN_OCP_DEBOUNCE_TIMER))
                {
                    /* Stop the debounce timer. */
                    timer_stop(port, PD_VCONN_OCP_DEBOUNCE_TIMER);

                    /* Look for positive edge of comparator. */
                    regval = pd->intr1_cfg_cc12_ocp_hs;
                    regval &= ~cfg_mask;
                    regval |= FILTER_CFG_POS_EN_NEG_DIS << cfg_pos;
                    pd->intr1_cfg_cc12_ocp_hs = regval;

                    /* If the positive edge happened already, queue the interrupt. */
                    if ((pd->intr1_status & stat_mask) != 0)
                    {
                        pd->intr1_set = (gl_vconn_channel[port] ?
                                PDSS_INTR1_SET_CC2_OCP_CHANGED : PDSS_INTR1_SET_CC1_OCP_CHANGED);
                    }
                }
            }

            /* Enable interrupt. */
            pd->intr1_mask |= (gl_vconn_channel[port] ? PDSS_INTR1_CC2_OCP_CHANGED : PDSS_INTR1_CC1_OCP_CHANGED);
        }
#endif /* VCONN_OCP_ENABLE */

        /* CC1/2 OVP (VBus short) handler. */
        if ((intr1_cause & (PDSS_INTR1_CC1_OVP_CHANGED | PDSS_INTR1_CC2_OVP_CHANGED)) != 0)
        {
            /* No need to check current status as we only enable positive edge on the interrupt. */
            if (gl_ccg_fault_cb != NULL)
            {
                /* Passing false to indicate CC1/2 fault. */
                gl_ccg_fault_cb (port, false);
            }

            /* Re-enable the interrupts. */
            pd->intr1_mask |= (PDSS_INTR1_MASK_CC1_OVP_CHANGED_MASK | PDSS_INTR1_MASK_CC2_OVP_CHANGED_MASK);
        }

#if CCGX_V5V_CHANGE_DETECT
        if ((intr1_cause & PDSS_INTR1_MASK_V5V_CHANGED_MASK) != 0)
        {
            if (gl_ccg_supply_changed_cb != NULL)
            {
                /* Store the current status of the V5V supply and (re)start a timer for debounce. */
                if (pd->intr1_status & PDSS_INTR1_STATUS_V5V_STATUS)
                {
                    ccg5_v5v_supply_present[port] = true;
                }
                else
                {
                    ccg5_v5v_supply_present[port] = false;
                }

                timer_start (port, APP_V5V_CHANGE_DEBOUNCE_TIMER, 500, ccg5_v5v_supply_debounce_cb);
            }

            /* Re-enable the interrupt. */
            pd->intr1_mask |= PDSS_INTR1_MASK_V5V_CHANGED_MASK;
        }
#endif /* CCGX_V5V_CHANGE_DETECT */
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
    }

#if (defined(CCG3PA) || defined(CCG3PA2))
    if (pd->intr3_masked & (1 << PDSS_INTR3_CMP_OUT_CHANGED_POS))
    {
        pd->intr3_mask &= ~(1 << PDSS_INTR3_CMP_OUT_CHANGED_POS);
        pd->intr3       =  (1 << PDSS_INTR3_CMP_OUT_CHANGED_POS);

        /* Check status. */
        if (pd->adc_ctrl[PD_ADC_ID_0] & PDSS_ADC_CTRL_CMP_OUT)
        {
            comp_out = false;
        }

        /* Report status. */
        if (pdss_stat->adc_cb[PD_ADC_ID_0] != NULL)
        {
            pdss_stat->adc_cb[PD_ADC_ID_0](port, comp_out);
        }
    }

    if (pd->intr3_masked & ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << PD_ADC_ID_1))
    {
        pd->intr3_mask &= ~((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << PD_ADC_ID_1);
        pd->intr3       =  ((1 << PDSS_INTR3_CMP_OUT_CHANGED_POS) << PD_ADC_ID_1);

        /* Check status. */
        if (pd->adc_ctrl[PD_ADC_ID_1] & PDSS_ADC_CTRL_CMP_OUT)
        {
            comp_out = false;
        }

        /* Report status. */
        if (pdss_stat->adc_cb[PD_ADC_ID_1] != NULL)
        {
            pdss_stat->adc_cb[PD_ADC_ID_1](port, comp_out);
        }
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    if (pd->intr5_masked != 0)
    {
#if (defined(CCG3PA) || defined(CCG3PA2))
#if VBUS_OCP_ENABLE
        if (pd->intr5_masked & (1 << FILTER_ID_LSCSA_OCP))
        {
            /* If positive edge interrupt. */
            if (((pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] & PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK) >>
                PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) == FILTER_CFG_POS_EN_NEG_DIS)
            {
                /* Clear and disable interrupt. */
                pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_OCP);
                pd->intr5 = 1 << FILTER_ID_LSCSA_OCP;

                /* If No software debounce mode. */
                if ((gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT) ||
                    (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_AUTOCTRL))
                {
                    /* Invoke the callback. */
                    vbus_ocp_handler (port);
                }
                /* If software debounce mode. */
                else if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_SW_DB)
                {
                    uint32_t regval;
                    /*
                     * Look for negative edge of comparator. NOTE: Here we are using filter
                     * reset mechanism to simulate edge if the comparator status has already gone low.
                     * This assumes that the OCP comparator filter is being used during enable.
                     */
                    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] &= ~PDSS_INTR5_FILTER_CFG_FILT_EN;
                    regval = pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP];
                    regval &= ~(PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK);
                    regval |= FILTER_CFG_POS_DIS_NEG_EN << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS;
                    regval |= (PDSS_INTR5_FILTER_CFG_FILT_RESET | PDSS_INTR5_FILTER_CFG_FILT_EN);
                    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] = regval;

                    /* Enable interrupt. */
                    pd->intr5_mask |= 1 << FILTER_ID_LSCSA_OCP;
                    /* Start the debounce timer. */
                    timer_start(port, PD_OCP_DEBOUNCE_TIMER, gl_ocp_sw_db_ms[port], ocp_handler_wrapper);
                }
            }
            /* If negative edge interrupt. */
            else
            {
                /* Clear the active interrupt. */
                pd->intr5 = 1 << FILTER_ID_LSCSA_OCP;

                if ((timer_is_running (port, PD_OCP_DEBOUNCE_TIMER)))
                {
                    uint32_t regval;
                    /* Stop the debounce timer. */
                    timer_stop(port, PD_OCP_DEBOUNCE_TIMER);

                    /* Clear the interrupt. */
                    pd->intr5 = 1 << FILTER_ID_LSCSA_OCP;

                    /*
                     * Look for positive edge of comparator. NOTE: Here we are using filter
                     * reset mechanism to simulate edge if the comparator status has already gone low.
                     * This assumes that the OCP comparator filter is being used during enable.
                     */
                    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] &= ~PDSS_INTR5_FILTER_CFG_FILT_EN;
                    regval = pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP];
                    regval &= ~(PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_RESET);
                    regval |= FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS;
                    regval |= PDSS_INTR5_FILTER_CFG_FILT_EN;
                    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] = regval;
                }
                else
                {
                    /* Disable the interrupt as we are no longer interested in it. */
                    pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_OCP);
                }
            }
        }
#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
        if (pd->intr5_masked & (1 << FILTER_ID_LSCSA_SCP))
        {
            /* SCP interrupt handling. */
            /* Clear and disable interrupt. */
            pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_SCP);
            pd->intr5 = 1 << FILTER_ID_LSCSA_SCP;
            vbus_scp_handler (port);
        }
#endif /* VBUS_SCP_ENABLE */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if VBUS_OVP_ENABLE
        if (pd->intr5_masked & (1 << gl_vbus_ovp_filter_id[port]))
        {
            /* Disable and clear OV interrupt. */
            pd->intr5_mask &= ~(1 << gl_vbus_ovp_filter_id[port]);
            pd->intr5 = 1 << gl_vbus_ovp_filter_id[port];

            /* Invoke OVP callback. */
            if (gl_ovp_cb[port] != NULL)
            {
                gl_ovp_cb[port] (port, true);
            }
        }
#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE
        if (pd->intr5_masked & (1 << FILTER_ID_UV))
        {
            /* Disable and clear UV interrupt. */
            pd->intr5_mask &= ~(1 << FILTER_ID_UV);
            pd->intr5 = 1 << FILTER_ID_UV;

            /* Invoke UVP callback. */
            if (gl_uvp_cb != NULL)
            {
                gl_uvp_cb (port, false);
            }
        }
#endif /* VBUS_UVP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))
        if (pd->intr5_masked & (1 << FILTER_ID_LSCSA_PFC))
        {
            bool state = false;
            uint32_t regval = (pd->intr5_status_0 >> PDSS_INTR5_STATUS_0_FILT_12_POS);

            if (regval & (1 << FILTER_ID_LSCSA_PFC))
            {
                state = true;
            }

            pd->intr5 = 1 << FILTER_ID_LSCSA_PFC;

            /* Invoke callback. */
            if (pdss_stat->pfc_cmp_cbk != NULL)
            {
                pdss_stat->pfc_cmp_cbk(port, state);
            }
        }
        if (pd->intr5_masked & (1 << FILTER_ID_LSCSA_SR))
        {
            bool state = false;
            uint32_t regval = (pd->intr5_status_0 >> PDSS_INTR5_STATUS_0_FILT_12_POS);

            if (regval & (1 << FILTER_ID_LSCSA_SR))
            {
                state = true;
            }

            pd->intr5 = 1 << FILTER_ID_LSCSA_SR;

            /* Invoke callback. */
            if (pdss_stat->sr_cmp_cbk != NULL)
            {
                pdss_stat->sr_cmp_cbk(port, state);
            }
        }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
    }

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    if (pd->intr7_masked != 0)
    {
        /* Clear all interrupts. */
        pd->intr7 = pd->intr7;

        /* VSYS detection interrupt. */
        if ((pd->intr7_status & (2 << PDSS_INTR7_STATUS_FILT_8_POS)) != 0)
        {
            /* VSYS is now present: Enable switch and disable regulator. */
            pd->vreg_vsys_ctrl |= PDSS_VREG_VSYS_CTRL_ENABLE_VDDD_SWITCH;
            CyDelayUs (100);

            pd->vreg_vsys_ctrl &= ~PDSS_VREG_VSYS_CTRL_VREG20_1_EN;

#if CCG_PD_DUALPORT_ENABLE
            /* Disable regulator on the second port if applicable. */
            gl_pdss[1]->vreg_vsys_ctrl &= ~PDSS_VREG_VSYS_CTRL_VREG20_1_EN;
#endif /* CCG_PD_DUALPORT_ENABLE */

            /* Notify application about presence of VSYS supply. */
            if (gl_ccg_supply_changed_cb != NULL)
            {
                gl_ccg_supply_changed_cb (port, CCG_SUPPLY_VSYS, true);
            }
        }
        else
        {
            /* Disable the VSYS-VDDD Switch. */
            pd->vreg_vsys_ctrl &= ~PDSS_VREG_VSYS_CTRL_ENABLE_VDDD_SWITCH;

            /* Notify application about absence of VSYS supply. */
            if (gl_ccg_supply_changed_cb != NULL)
            {
                gl_ccg_supply_changed_cb (port, CCG_SUPPLY_VSYS, false);
            }
        }
    }
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

#if BATTERY_CHARGING_ENABLE
#if (defined(CCG3PA)) || (defined(CCG3PA2))
    if(pd->intr9_masked & (PDSS_INTR9_QCOM_RCVR_DM_CHANGED_MASK |
                PDSS_INTR9_QCOM_RCVR_DP_CHANGED_MASK))
    {
        pd->intr9_mask &= ~(PDSS_INTR9_QCOM_RCVR_DM_CHANGED_MASK |
                PDSS_INTR9_QCOM_RCVR_DP_CHANGED_MASK);
        pd->intr9 = PDSS_INTR9_QCOM_RCVR_DM_CHANGED_MASK |
            PDSS_INTR9_QCOM_RCVR_DP_CHANGED_MASK;
    }
#elif (defined(CCG5C) || defined(CCG6))
    if(pd->intr9_masked & (PDSS_INTR9_MASKED_QCOM_RCVR_DM_CHANGED_MASKED |
                PDSS_INTR9_MASKED_QCOM_RCVR_DP_CHANGED_MASKED))
    {
        pd->intr9_mask &= ~(PDSS_INTR9_MASK_QCOM_RCVR_DM_CHANGED_MASK |
                PDSS_INTR9_MASK_QCOM_RCVR_DP_CHANGED_MASK);
        pd->intr9 = PDSS_INTR9_QCOM_RCVR_DM_CHANGED |
            PDSS_INTR9_QCOM_RCVR_DP_CHANGED;
    }
#endif /* (defined(CCG5C) || defined(CCG6)) */

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5C) || defined(CCG6))
    if (pd->intr9_masked & BCH_PORT_0_CMP1_INTR_MASK)
    {
        /* Disable and clear interrupts. */
        pd->intr9_mask &= ~BCH_PORT_0_CMP1_INTR_MASK;
        pd->intr9 = BCH_PORT_0_CMP1_INTR_MASK;

        /* Report an CMP1 fire event. */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            pdss_stat->bc_phy_cbk(TYPEC_PORT_0_IDX, BC_EVT_CMP1_FIRE);
        }
    }

    if (pd->intr9_masked & BCH_PORT_0_CMP2_INTR_MASK)
    {
        /* Disable and clear interrupts. */
        pd->intr9_mask &= ~BCH_PORT_0_CMP2_INTR_MASK;
        pd->intr9 = BCH_PORT_0_CMP2_INTR_MASK;

        /* Report an CMP2 fire event. */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            pdss_stat->bc_phy_cbk(TYPEC_PORT_0_IDX, BC_EVT_CMP2_FIRE);
        }
    }

#if (NO_OF_BC_PORTS == 2)
    if (pd->intr9_masked & BCH_PORT_1_CMP1_INTR_MASK)
    {
        /* Disable and clear interrupts. */
        pd->intr9_mask &= ~BCH_PORT_1_CMP1_INTR_MASK;
        pd->intr9 = BCH_PORT_1_CMP1_INTR_MASK;

        /* Report an OVP trip event. */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            pdss_stat->bc_phy_cbk(TYPEC_PORT_1_IDX, BC_EVT_CMP1_FIRE);
        }
    }
    if (pd->intr9_masked & BCH_PORT_1_CMP2_INTR_MASK)
    {
        /* Disable and clear interrupts. */
        pd->intr9_mask &= ~BCH_PORT_1_CMP2_INTR_MASK;
        pd->intr9 = BCH_PORT_1_CMP2_INTR_MASK;

        /* Report an OVP trip event. */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            pdss_stat->bc_phy_cbk(TYPEC_PORT_1_IDX, BC_EVT_CMP2_FIRE);
        }
    }
#endif /* NO_OF_BC_PORTS == 2 */
#endif /* (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5C) || defined(CCG6)) */

#if (defined(CCG5))
    if (pd->intr3_masked & PDSS_INTR3_CHGDET_CHANGED)
    {
        /* Clear and disable the interrupt. Handling will be done as part of CDP state machine. */
        pd->intr3       = PDSS_INTR3_CHGDET_CHANGED;
        pd->intr3_mask &= ~PDSS_INTR3_MASK_CHGDET_CHANGED_MASK;
        /* Clear interrupt configuration. */
        pd->intr3_cfg_chgdet &= ~PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK;

        /* Notify the charger detect logic that the comparator match has occured. */
        if (pdss_stat->bc_phy_cbk != NULL)
        {
            pdss_stat->bc_phy_cbk(port, BC_EVT_CMP1_FIRE);
        }

    }
#endif /* (defined(CCG5) */
#endif /* BATTERY_CHARGING_ENABLE */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

    /* SBU1/2 OVP change interrupt. */
    if ((pd->intr3_masked & PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK) != 0)
    {
        /* Clear the interrupt. */
        pd->intr3 = PDSS_INTR3_SBU1_SBU2_OVP_CHANGED_MASK;

        /* We don't need to check status as only positive edge triggered interrupts are enabled. */
        if (gl_ccg_fault_cb != NULL)
        {
            /* Parameter true indicates SBU fault. */
            gl_ccg_fault_cb (port, true);
        }
    }

    /* Comparator (ADC) output change interrupt. */
    if (pd->intr3_masked & PDSS_INTR3_CMP_OUT_CHANGED)
    {
        pd->intr3_mask &= ~PDSS_INTR3_CMP_OUT_CHANGED;
        pd->intr3       =  PDSS_INTR3_CMP_OUT_CHANGED;

        /* Check status. */
        if (pd->adc_ctrl & PDSS_ADC_CTRL_CMP_OUT)
        {
            comp_out = false;
        }

        /* Report status. */
        if (pdss_stat->adc_cb[PD_ADC_ID_0] != NULL)
        {
            pdss_stat->adc_cb[PD_ADC_ID_0](port, comp_out);
        }
    }

#if VBUS_OCP_ENABLE
    /*
     * CCG5 uses INTR3.CSA_OC_CHANGED interrupt to indicate OCP condition.
     * CCG5C and CCG6 use INTR13.CSA_OCP_CHANGED interrupt to indicate OCP condition.
     */
    if ((CCG_OCP_INT_ACT_REG(pd) & CCG_OCP_INTR_REQ) != 0)
    {
        /* If positive edge interrupt. */
        if (((CCG_OCP_CFG_REG(pd) & CCG_OCP_INTR_CFG_MASK) >> CCG_OCP_INTR_CFG_POS) == FILTER_CFG_POS_EN_NEG_DIS)
        {
            /* Clear and disable interrupt. */
            CCG_OCP_INT_MSK_REG(pd) &= ~CCG_OCP_INTR_MASK;
            CCG_OCP_INT_REQ_REG(pd)  = CCG_OCP_INTR_REQ;

            /* If no software debounce mode. */
            if ((gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT) ||
                    (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_AUTOCTRL))
            {
                /* Invoke the callback. */
                vbus_ocp_handler (port);
            }
            /* If software debounce mode. */
            else if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_SW_DB)
            {
                uint32_t regval;

                /* Look for negative edge of comparator. */
                regval = CCG_OCP_CFG_REG(pd);
                regval &= ~(CCG_OCP_INTR_CFG_MASK);
                regval |= FILTER_CFG_POS_DIS_NEG_EN << CCG_OCP_INTR_CFG_POS;
                CCG_OCP_CFG_REG(pd) = regval;

                /* Enable interrupt. */
                CCG_OCP_INT_MSK_REG(pd) |= CCG_OCP_INTR_MASK;

                /* Start the debounce timer. */
                timer_start(port, PD_OCP_DEBOUNCE_TIMER, gl_ocp_sw_db_ms[port], ocp_handler_wrapper);

                /* If the negative edge has already happened, raise an interrupt. */
                if ((CCG_OCP_STATUS_REG(pd) & CCG_OCP_LIVE_STATUS) == 0)
                {
                    CCG_OCP_INT_SET_REG(pd) = CCG_OCP_INTR_SET;
                }
            }
        }
        /* If negative edge interrupt. */
        else
        {
            /* Clear the interrupt. */
            CCG_OCP_INT_REQ_REG(pd) = CCG_OCP_INTR_REQ;

            if (timer_is_running (port, PD_OCP_DEBOUNCE_TIMER))
            {
                uint32_t regval;

                /* Stop the debounce timer. */
                timer_stop (port, PD_OCP_DEBOUNCE_TIMER);

                /* Look for positive edge of comparator. */
                regval = CCG_OCP_CFG_REG(pd);
                regval &= ~(CCG_OCP_INTR_CFG_MASK);
                regval |= FILTER_CFG_POS_EN_NEG_DIS << CCG_OCP_INTR_CFG_POS;
                CCG_OCP_CFG_REG(pd) = regval;

                /* If the positive edge has already happened, raise an interrupt. */
                if ((CCG_OCP_STATUS_REG(pd) & CCG_OCP_LIVE_STATUS) != 0)
                {
                    CCG_OCP_INT_SET_REG(pd) = CCG_OCP_INTR_SET;
                }
            }
            else
            {
                /* Disable the interrupt here as it is no longer of interest. */
                CCG_OCP_INT_MSK_REG(pd) &= ~CCG_OCP_INTR_MASK;
            }
        }
    }
#endif /* VBUS_OCP_ENABLE */

#endif /* (defined (CCG5) || defined(CCG6) || defined(CCG5C)) */
}

void pd_reset_edge_det(uint8_t port, bool pgdo_type)
{
    PPDSS_REGS_T pd = gl_pdss[port];

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG5C) || defined(CCG6))
    /* True indicates PGDO with internal pull up. */
    if (pgdo_type)
    {
        /* Reset edge detector. */
        pd->pgdo_pu_1_cfg |= PDSS_PGDO_PU_1_CFG_RST_EDGE_DET;
        pd->pgdo_pu_1_cfg &= ~PDSS_PGDO_PU_1_CFG_RST_EDGE_DET;
    }
    else
    {
        /* Reset edge detector. */
        pd->pgdo_1_cfg |= PDSS_PGDO_1_CFG_RST_EDGE_DET;
        pd->pgdo_1_cfg &= ~PDSS_PGDO_1_CFG_RST_EDGE_DET;
    }
#elif defined(CCG5)
    if (pgdo_type)
    {
        /* Reset edge detector. */
        pd->pgdo_1_cfg[1] |= PDSS_PGDO_1_CFG_RST_EDGE_DET;
        pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_RST_EDGE_DET;
    }
    else
    {
        /* Reset edge detector. */
        pd->pgdo_1_cfg[0] |= PDSS_PGDO_1_CFG_RST_EDGE_DET;
        pd->pgdo_1_cfg[0] &= ~PDSS_PGDO_1_CFG_RST_EDGE_DET;
    }
#endif /* defined(CCGx) */
}

#if (defined(CCG3PA) || defined(CCG3PA2))

void pd_remove_internal_fb_res_div(void)
{
    PDSS->ea_ctrl |= PDSS_EA_CTRL_RES_DIV_BYPASS;
}

static void pd_pgdo_pu_en(void)
{
    uint32_t regval;

    /* Turn on the fet by setting ENABLE_ON bit and clearing IN_ON bit. */
    regval = PDSS->pgdo_pu_1_cfg;
    regval |= (PDSS_PGDO_PU_1_CFG_SEL_ON_OFF | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE);
    regval &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_ON_VALUE;
    PDSS->pgdo_pu_1_cfg |= regval;
}

static void pd_pgdo_pu_dis(void)
{
    /* Program PGDO_PU back to its default (OFF) state. */
    uint32_t regval = PDSS->pgdo_pu_1_cfg;

    regval &= ~(PDSS_PGDO_PU_1_CFG_SEL_ON_OFF
        | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE
        | PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_ON_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE);
    regval |= PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
    PDSS->pgdo_pu_1_cfg = regval;

}

static void pd_pgdo_en(void)
{
    uint32_t regval;
    regval = PDSS->pgdo_1_cfg;
    /* Turn on the fet by setting ENABLE_ON bit. */
    regval |= (PDSS_PGDO_1_CFG_SEL_ON_OFF | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
    PDSS->pgdo_1_cfg |= regval;
}

static void pd_pgdo_dis(void)
{
     /* Program PGDO_PU back to its default (OFF) state. */
    PDSS->pgdo_1_cfg &= ~(PDSS_PGDO_1_CFG_SEL_ON_OFF
        | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE);
}

void pd_internal_pfet_on(uint8_t port, bool turn_on_seq)
{
    (void)port;
    (void)turn_on_seq;

    pd_internal_pfet_soft_start_on (port, gl_ccgx_pfet_on[port]);
    gl_ccgx_pfet_on[port] = true;

    /*
     * The edge detector may have triggered later than previous fault.
     * In this case, this needs to be cleared before the FET can be
     * turned ON.
     */
    pd_reset_edge_det(port, true);
    pd_pgdo_pu_en();
}

void pd_internal_pfet_off(uint8_t port, bool turn_off_seq)
{
    (void)port;
    (void)turn_off_seq;

    timer_stop(port, APP_FET_SOFT_START_TIMER_ID);
    pd_pgdo_pu_dis();
    /*
     * The edge detector may have triggered later than previous fault.
     * In this case, this needs to be cleared.
     */
    pd_reset_edge_det(port, true);

    /* Make sure the soft start of the FET is disabled. */
    if (gl_ccgx_pfet_on[port])
    {
        pd_internal_pfet_soft_start_off (port);
        gl_ccgx_pfet_on[port] = false;
    }
}

void pd_internal_cfet_on(uint8_t port, bool turn_on_seq)
{
    (void)port;
    (void)turn_on_seq;
    
    /* We can turn source FET OFF when turning sink FET ON. */
    pd_internal_pfet_off(port, turn_on_seq);

    /* Configure the soft start for the consumer FET. */
    pd_internal_cfet_soft_start_on (port, gl_ccgx_cfet_on[port]);
    gl_ccgx_cfet_on[port] = true;

    /*
     * The edge detector may have triggered later than previous fault.
     * In this case, this needs to be cleared before the FET can be
     * turned ON.
     */
    pd_reset_edge_det(port, false);
    pd_pgdo_en();
}

void pd_internal_cfet_off(uint8_t port, bool turn_off_seq)
{
    (void)port;
    (void)turn_off_seq;

    timer_stop(port, APP_FET_SOFT_START_TIMER_ID);
    pd_pgdo_dis ();
    /*
     * The edge detector may have triggered later than previous fault.
     * In this case, this needs to be cleared.
     */
    pd_reset_edge_det(port, false);

    if (gl_ccgx_cfet_on[port])
    {
        PDSS->pgdo_pd_isnk_cfg = PDSS_PGDO_PD_ISNK_CFG_STRONG_EN;
        gl_ccgx_cfet_on[port] = false;
    }
}

#if VBUS_SLOW_DISCHARGE_EN

static void vbus_slow_discharge_on_process(uint8_t port)
{
    uint32_t reg_val = 0;
#if !CCG_FLIPPED_FET_CTRL

    /* We have obtained the current discharge strength in
     * pd_internal_vbus_discharge_on(). We should start
     * discharge from the next higher discharge strength value.
     */
    if (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_C_0A)
    {
        if (gl_vbus_discharge_ds_0 == 0)
        {
            gl_vbus_discharge_ds_0 = 1;
        }
        else
        {
            gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 << 1;
        }

        if (gl_vbus_discharge_ds_0 > CCG3PA_DISCHG_DS_VBUS_C_0A)
        {
            /* Set the discharge strength to maximum configured */
            gl_vbus_discharge_ds_0 = CCG3PA_DISCHG_DS_VBUS_C_0A;
        }

        /* Enable VBUS_C discharge with calculated drive strength */
        reg_val = PDSS->dischg_shv_ctrl[0];

        reg_val = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_0 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        PDSS->dischg_shv_ctrl[0] = reg_val;
    }

#if VBUS_IN_DISCHARGE_EN

    if (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_IN_0A)
    {
        if (gl_vbus_discharge_ds_1 == 0)
        {
            gl_vbus_discharge_ds_1 = 1;
        }
        else
        {
            gl_vbus_discharge_ds_1 = gl_vbus_discharge_ds_1 << 1;
        }
        if (gl_vbus_discharge_ds_1 > CCG3PA_DISCHG_DS_VBUS_IN_0A)
        {
            gl_vbus_discharge_ds_1 = CCG3PA_DISCHG_DS_VBUS_IN_0A;
        }

        /* This is VBUS_IN discharge path. Only discharge to 5V. */
        reg_val =  PDSS->dischg_shv_ctrl[1];
        reg_val = (reg_val & ~(PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG | PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK))
                    | (gl_vbus_discharge_ds_1 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        PDSS->dischg_shv_ctrl[1] = reg_val;

        /* Discharge enable driven by comparator. Enable the comparator. */
        PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] &= ~PDSS_COMP_CTRL_COMP_PD;
    }

#endif /* VBUS_IN_DISCHARGE_EN */

    if (
        (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_C_0A)
#if VBUS_IN_DISCHARGE_EN
        || (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_IN_0A)
#endif /* VBUS_IN_DISCHARGE_EN */
        )
    {
        timer_start(port, VBUS_DISCHARGE_SCHEDULE_TIMER,
            VBUS_SLOW_DISCHARGE_TIME_MS, vbus_slow_discharge_cbk);
    }
    else
    {
        gl_vbus_is_slow_discharge_on = false;
    }
#else /* !CCG_FLIPPED_FET_CTRL */
    if (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_C_0A)
    {
        if (gl_vbus_discharge_ds_1 == 0)
        {
            gl_vbus_discharge_ds_1 = 1;
        }
        else
        {
            gl_vbus_discharge_ds_1 = gl_vbus_discharge_ds_1 << 1;
        }
        if (gl_vbus_discharge_ds_1 > CCG3PA_DISCHG_DS_VBUS_C_0A)
        {
            gl_vbus_discharge_ds_1 = CCG3PA_DISCHG_DS_VBUS_C_0A;
        }
        reg_val = PDSS->dischg_shv_ctrl[1];

        reg_val = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_1 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        /* VBUS_C discharge enable with drive strength */
        PDSS->dischg_shv_ctrl[1] = reg_val;
    }

#if VBUS_IN_DISCHARGE_EN
    if (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_IN_0A)
    {
        if (gl_vbus_discharge_ds_0 == 0)
        {
            gl_vbus_discharge_ds_0 = 1;
        }
        else
        {
            gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 << 1;
        }
        if(gl_vbus_discharge_ds_0 > CCG3PA_DISCHG_DS_VBUS_IN_0A)
        {
            gl_vbus_discharge_ds_0 = CCG3PA_DISCHG_DS_VBUS_IN_0A;
        }
        reg_val = PDSS->dischg_shv_ctrl[0];

        reg_val = (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_0 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        PDSS->dischg_shv_ctrl[0] = reg_val;
    }
#endif /* VBUS_IN_DISCHARGE_EN */

    if (
        (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_C_0A)
#if VBUS_IN_DISCHARGE_EN
        || (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_IN_0A)
#endif /* VBUS_IN_DISCHARGE_EN */
        )
    {
        timer_start(port, VBUS_DISCHARGE_SCHEDULE_TIMER,
            VBUS_SLOW_DISCHARGE_TIME_MS, vbus_slow_discharge_cbk);
    }
    else
    {
        gl_vbus_is_slow_discharge_on = false;
    }

#endif /* CCG_FLIPPED_FET_CTRL */

}

static void vbus_slow_discharge_off_process(uint8_t port)
{
    uint32_t reg_val = 0;
#if !CCG_FLIPPED_FET_CTRL

    /* We have obtained the current discharge strength in
     * pd_internal_vbus_discharge_off(). We should start
     * discharge from the next lower discharge strength value.
     */
    if(gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_C_MIN)
    {
        gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 >> 1;

        if (gl_vbus_discharge_ds_0 < CCG3PA_DISCHG_DS_VBUS_C_MIN)
        {
            /* Set the discharge strength to minimum configured */
            gl_vbus_discharge_ds_0 = CCG3PA_DISCHG_DS_VBUS_C_MIN;
        }

        reg_val = PDSS->dischg_shv_ctrl[0];
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_0 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        /* Load calculated discharge drive strength */
        PDSS->dischg_shv_ctrl[0] = reg_val;
    }

#if VBUS_IN_DISCHARGE_EN

    if(gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_IN_MIN)
    {
        gl_vbus_discharge_ds_1 = gl_vbus_discharge_ds_1 >> 1;
        if (gl_vbus_discharge_ds_1 < CCG3PA_DISCHG_DS_VBUS_IN_MIN)
        {
            /* Set the discharge strength to minimum configured */
            gl_vbus_discharge_ds_1 = CCG3PA_DISCHG_DS_VBUS_IN_MIN;
        }

        reg_val = PDSS->dischg_shv_ctrl[1];
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_1 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        /* Load calculated discharge drive strength */
        PDSS->dischg_shv_ctrl[1] = reg_val;
    }

#endif /* VBUS_IN_DISCHARGE_EN */

    if (
        (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_C_MIN)
#if VBUS_IN_DISCHARGE_EN
        || (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_IN_MIN)
#endif /* VBUS_IN_DISCHARGE_EN */
        )
    {
        timer_start(port, VBUS_DISCHARGE_SCHEDULE_TIMER,
            VBUS_SLOW_DISCHARGE_TIME_MS, vbus_slow_discharge_cbk);
    }
    else
    {
        /*
         * Done with discharge. Now disable discharge accordingly.
         */
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#if VBUS_IN_DISCHARGE_EN
        PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] |= PDSS_COMP_CTRL_COMP_PD;
        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */

        gl_vbus_is_slow_discharge_off = false;
    }
#else
    if(gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_C_MIN)
    {
        gl_vbus_discharge_ds_1 = gl_vbus_discharge_ds_1 >> 1;

        if (gl_vbus_discharge_ds_1 < CCG3PA_DISCHG_DS_VBUS_C_MIN)
        {
            /* Set the discharge strength to minimum configured */
            gl_vbus_discharge_ds_1 = CCG3PA_DISCHG_DS_VBUS_C_MIN;
        }

        reg_val = PDSS->dischg_shv_ctrl[1];
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_1 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        /* Load calculated discharge drive strength */
        PDSS->dischg_shv_ctrl[1] = reg_val;
    }

#if VBUS_IN_DISCHARGE_EN

    if(gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_IN_MIN)
    {
        gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 >> 1;
        if (gl_vbus_discharge_ds_0 < CCG3PA_DISCHG_DS_VBUS_IN_MIN)
        {
            /* Set the discharge strength to minimum configured */
            gl_vbus_discharge_ds_0 = CCG3PA_DISCHG_DS_VBUS_IN_MIN;
        }

        reg_val = PDSS->dischg_shv_ctrl[0];
        reg_val = (reg_val & ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK) |
                    (gl_vbus_discharge_ds_0 << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS);

        /* Load calculated discharge drive strength */
        PDSS->dischg_shv_ctrl[0] = reg_val;
    }

#endif /* VBUS_IN_DISCHARGE_EN */

    if (
        (gl_vbus_discharge_ds_1 != CCG3PA_DISCHG_DS_VBUS_C_MIN)
#if VBUS_IN_DISCHARGE_EN
        || (gl_vbus_discharge_ds_0 != CCG3PA_DISCHG_DS_VBUS_IN_MIN)
#endif /* VBUS_IN_DISCHARGE_EN */
        )
    {
        timer_start(port, VBUS_DISCHARGE_SCHEDULE_TIMER,
            VBUS_SLOW_DISCHARGE_TIME_MS, vbus_slow_discharge_cbk);
    }
    else
    {
        /*
         * Done with discharge. Now disable discharge accordingly.
         */
#if VBUS_IN_DISCHARGE_EN
        PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* VBUS_IN_DISCHARGE_EN */

        PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
            PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);

        gl_vbus_is_slow_discharge_off = false;
    }
#endif /* CCG_FLIPPED_FET_CTRL */

}

static void vbus_slow_discharge_cbk(uint8_t port, timer_id_t id)
{
    if (VBUS_DISCHARGE_SCHEDULE_TIMER == id)
    {
        if (gl_vbus_is_slow_discharge_on == true)
        {
            /* Process Slow discharge ON. gl_vbus_is_slow_discharge_on will be
             * updated inside vbus_slow_discharge_on_process().
             */
            vbus_slow_discharge_on_process(port);
        } /* gl_vbus_slow_discharge_on == true */
        else if (gl_vbus_is_slow_discharge_off == true)
        {
            /* Process Slow discharge OFF. gl_vbus_is_slow_discharge_off will be
             * updated inside vbus_slow_discharge_off_process().
             */
            vbus_slow_discharge_off_process(port);
        }
    } /* VBUS_DISCHARGE_SCHEDULE_TIMER == id */
}
#endif /* VBUS_SLOW_DISCHARGE_EN */


void pd_internal_vbus_discharge_on(uint8_t port)
{
    (void)port;

#if VBUS_SLOW_DISCHARGE_EN
    if (gl_vbus_is_slow_discharge_on == true)
    {
        /* Last Discharge ON request ongoing. We will ignore this request */
    }
    else
    {
        /* Set the discharge ongoing flag */
        gl_vbus_is_slow_discharge_on = true;

        if (gl_vbus_is_slow_discharge_off == true)
        {
            /* Last Discharge OFF request ongoing. We will stop this and start
             * processing the current request */
            timer_stop(port,VBUS_DISCHARGE_SCHEDULE_TIMER);
            /* We will not be disabling the discharge flag here. */
        }

        /* Obtain the current drive strength values for VBUS_IN and VBUS_C.
         * In ideal case this will be 1. This might be of higher value if
         * the last discharge OFF was not completed when this discharge ON
         * request came.
         */
        gl_vbus_discharge_ds_0 = (PDSS->dischg_shv_ctrl[0] & PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK)
                        >> PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS;
        gl_vbus_discharge_ds_1 = (PDSS->dischg_shv_ctrl[1] & PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK)
                        >> PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS;

        if (gl_vbus_is_slow_discharge_off == false)
        {
            /*
             * Last Discharge completed completely before this request was
             * made. In thia case we should start discharging with the
             * minimum configured discharge strength.
             * We decrease the strength to the next lower power of 2 or 0,
             * whichever is lower here. In vbus_slow_discharge_on_process()
             * we start with the next power of 2 value. By this approach, there
             * is no skipping of minimum discharge strength configured for this
             * part.
             */
            if ((gl_vbus_discharge_ds_0 >> 1) != 0)
            {
                gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 >> 1;
            }
            else
            {
                gl_vbus_discharge_ds_0 = 0;
            }
        }
        else
        {
            /* Disable the discharge flag. We do not decrease the discharge
             * strength value, as we want to start from the next discharge
             * strength value in vbus_slow_discharge_on_process().
             */
            gl_vbus_is_slow_discharge_off = false;
        }
        /* Call the CB directly 1st time */
        vbus_slow_discharge_cbk(port, VBUS_DISCHARGE_SCHEDULE_TIMER);
    }
#else /* VBUS_SLOW_DISCHARGE_EN */

#if !CCG_FLIPPED_FET_CTRL
    /* This is VBUS_C discharge path. */
    PDSS->dischg_shv_ctrl[0] |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#else /* CCG_FLIPPED_FET_CTRL */
    PDSS->dischg_shv_ctrl[1] |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* CCG_FLIPPED_FET_CTRL */

#if VBUS_IN_DISCHARGE_EN
    pd_internal_vbus_in_discharge_on(port);
#endif /* VBUS_IN_DISCHARGE_EN */

#endif /* !VBUS_SLOW_DISCHARGE_EN */

}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    (void)port;

#if VBUS_SLOW_DISCHARGE_EN
    uint8_t count = 0;
    uint8_t reg_val_0 = 0;
    uint8_t reg_val_1 = 0;
    if (gl_vbus_is_slow_discharge_off == true)
    {
        /* Last Discharge OFF request ongoing. We will ignore this request */
    }
    else
    {
        /* Set the discharge off ongoing flag */
        gl_vbus_is_slow_discharge_off = true;

        if (gl_vbus_is_slow_discharge_on == true)
        {
            /* Last Discharge ON request ongoing. We will stop this and start
             * processing the current request */
            timer_stop(port,VBUS_DISCHARGE_SCHEDULE_TIMER);
            /* Disable the discharge */
            gl_vbus_is_slow_discharge_on = false;
        }

        /* Obtain the current drive strength values for VBUS_IN and VBUS_C.
         * In ideal case this will be maximum allowed. This might be of lesser
         * value if the last discharge ON was not completed when this discharge
         * OFF request came.
         */
        gl_vbus_discharge_ds_0 = (PDSS->dischg_shv_ctrl[0] & PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK)
                        >> PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS;
        gl_vbus_discharge_ds_1 = (PDSS->dischg_shv_ctrl[1] & PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK)
                        >> PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS;

        /* Keep a copy locally. Will be used later for comparison */
        reg_val_0 = gl_vbus_discharge_ds_0;
        reg_val_1 = gl_vbus_discharge_ds_1;

        /* Maximum discharge strength need not be power of 2. We will go to the
         * next higher power of 2. The value will be reduced in CB context.
         */
        while ((gl_vbus_discharge_ds_0 >> 1) != 0)
        {
            gl_vbus_discharge_ds_0 = gl_vbus_discharge_ds_0 >> 1;
            count = count + 1;
        }
        gl_vbus_discharge_ds_0 = 1 << (count + 1);

        count = 0;

        while ((gl_vbus_discharge_ds_1 >> 1) != 0)
        {
            gl_vbus_discharge_ds_1  = gl_vbus_discharge_ds_1 >> 1;
            count = count + 1;
        }
        gl_vbus_discharge_ds_1 = 1 << (count + 1);

        /*
         * For power of 2 discharge strength, we should not use the above
         * calculated value. We will restore the value to the original
         */
        if (reg_val_0 == (gl_vbus_discharge_ds_0 >> 1))
        {
            gl_vbus_discharge_ds_0 = reg_val_0;
        }
        if (reg_val_1 == (gl_vbus_discharge_ds_1 >> 1))
        {
            gl_vbus_discharge_ds_1 = reg_val_1;
        }

        /* Call the CB directly 1st time */
        vbus_slow_discharge_cbk(port, VBUS_DISCHARGE_SCHEDULE_TIMER);
    }
#else
#if !CCG_FLIPPED_FET_CTRL
    PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#else /* CCG_FLIPPED_FET_CTRL */
    PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* CCG_FLIPPED_FET_CTRL */

#if VBUS_IN_DISCHARGE_EN
    pd_internal_vbus_in_discharge_off(port);
#endif /* VBUS_IN_DISCHARGE_EN */

#endif /* !VBUS_SLOW_DISCHARGE_EN */

}

void pd_internal_vbus_in_discharge_on(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
#if !CCG_FLIPPED_FET_CTRL
    /* This is VBUS_IN discharge path. Only discharge to 5V. */
    PDSS->dischg_shv_ctrl[1] &= ~(PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
    /* Enable the comparator. */
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] &= ~PDSS_COMP_CTRL_COMP_PD;
#else /* CCG_FLIPPED_FET_CTRL */
    PDSS->dischg_shv_ctrl[0] |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* CCG_FLIPPED_FET_CTRL */
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_internal_vbus_in_discharge_off(uint8_t port)
{
    (void)port;
#if VBUS_IN_DISCHARGE_EN
#if !CCG_FLIPPED_FET_CTRL
    PDSS->comp_ctrl[COMP_ID_VBUS_DISCHARGE] |= PDSS_COMP_CTRL_COMP_PD;
    PDSS->dischg_shv_ctrl[1] = ((PDSS->dischg_shv_ctrl[1] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#else /* CCG_FLIPPED_FET_CTRL */
    PDSS->dischg_shv_ctrl[0] = ((PDSS->dischg_shv_ctrl[0] & ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN) |
        PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG);
#endif /* CCG_FLIPPED_FET_CTRL */
#endif /* VBUS_IN_DISCHARGE_EN */
}

void pd_hal_set_fb_dac(int16_t dac_value)
{
    uint32_t tmp;
    tmp = PDSS->ea_ctrl & ~(
            PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK |
            PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK |
            PDSS_EA_CTRL_ISRC_EN |
            PDSS_EA_CTRL_ISNK_EN);

    if(dac_value >= 0)
    {
        tmp |= ((dac_value << PDSS_EA_CTRL_ISNK_DAC_CTRL_POS) &
                PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);

        /* Enable only for non-zero value. */
        if(dac_value != 0)
        {
            tmp |= PDSS_EA_CTRL_ISNK_EN;
        }

#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_DIR_FB)
        if (ccg_get_si_revision() != 0)
        {
            PDSS_TRIMS->trim_ea1_0 = EA_IREF_GAIN_NDAC;
        }
#endif /* (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_DIR_FB) */
    }
    else
    {
        tmp |= ((((dac_value * -1) << PDSS_EA_CTRL_ISRC_DAC_CTRL_POS) &
                PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK) | PDSS_EA_CTRL_ISRC_EN);

#if (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_DIR_FB)
        if (ccg_get_si_revision() != 0)
        {
            PDSS_TRIMS->trim_ea1_0 = EA_IREF_GAIN_PDAC;
        }
#endif /* (VBUS_CTRL_TYPE_P1 == VBUS_CTRL_DIR_FB) */
    }

    PDSS->ea_ctrl = tmp;
}

int16_t pd_hal_get_fb_dac(void)
{
    int16_t src_dac, snk_dac;

    src_dac = (PDSS->ea_ctrl & PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK) >>
        PDSS_EA_CTRL_ISRC_DAC_CTRL_POS;
    snk_dac = (PDSS->ea_ctrl & PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK) >>
        PDSS_EA_CTRL_ISNK_DAC_CTRL_POS;

    return (snk_dac - src_dac);
}

void pd_hal_enable_cv(void)
{
    uint32_t regval;

    regval = PDSS->ea_ctrl & ~(PDSS_EA_CTRL_ISNK_EN | PDSS_EA_CTRL_ISRC_EN |
            PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK | PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);
    PDSS->ea_ctrl = (regval | PDSS_EA_CTRL_EN_CV);
}

void pd_hal_disable_cv(void)
{
    PDSS->ea_ctrl &= ~(PDSS_EA_CTRL_EN_CV | PDSS_EA_CTRL_ISRC_EN |
            PDSS_EA_CTRL_ISNK_EN | PDSS_EA_CTRL_ISRC_DAC_CTRL_MASK |
            PDSS_EA_CTRL_ISNK_DAC_CTRL_MASK);
}

void pd_hal_disable_vreg(uint8_t port)
{
    /*
     * If system has external VDDD source, internal VBUS regulator shall
     * be turned off.
     */
    PPDSS_REGS_T pd = gl_pdss[port];

    pd->vreg_vsys_ctrl &= ~(PDSS_VREG_VSYS_CTRL_VREG_EN);
}
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))

void pd_internal_pfet_on(uint8_t port, bool turn_on_seq)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval = 0;
    (void)turn_on_seq;

    /* We can turn sink FET OFF if source FET is being turned ON. */
    pd_internal_cfet_off(port, turn_on_seq);

    /* Reset the edge detector. */
    pd_reset_edge_det(port, true);

#ifdef CCG6
    pd_internal_pfet_soft_start_on (port, gl_ccgx_pfet_on[port]);
#endif /* CCG6 */

#ifdef CCG5C
    /* Enable the pull-down for the FET. */
    pd->pgdo_pd_cfg = PDSS_PGDO_PD_CFG_DEFAULT;
#endif /* CCG5C */

    gl_ccgx_pfet_on[port] = true;

#if (defined(CCG5C) || defined(CCG6))
    /* Turn on the fet by setting ENABLE_ON bit and clearing IN_ON bit. */
    regval = pd->pgdo_pu_1_cfg;
    regval |= (PDSS_PGDO_PU_1_CFG_SEL_ON_OFF | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE);
    regval &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_ON_VALUE;
    pd->pgdo_pu_1_cfg = regval;
#endif /* (defined(CCG5C) || defined(CCG6)) */

#ifdef CCG5
    /* Turn on the fet. */
    regval = pd->pgdo_1_cfg[1];
    regval |= (PDSS_PGDO_1_CFG_SEL_ON_OFF | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
    pd->pgdo_1_cfg[1] = regval;
#endif /* CCG5 */
}

void pd_internal_pfet_off(uint8_t port, bool turn_off_seq)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    /* Legacy parameter - Not used. */
    (void)turn_off_seq;

    if (gl_ccgx_pfet_on[port])
    {
#ifdef CCG6
        /* Make sure the soft start of the FET is disabled. */
        pd_internal_pfet_soft_start_off (port);
#endif

#if (defined(CCG5C))
        /* Disable the pull-down on the FET. */
        pd->pgdo_pd_cfg &= ~PDSS_PGDO_PD_CFG_PD_ENABLE;
#endif /* (defined(CCG5C)) */
    }

    gl_ccgx_pfet_on[port] = false;

#if (defined(CCG5C) || defined(CCG6))
    /* Disable the FET first. */
    pd->pgdo_pu_1_cfg &= ~(PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE | PDSS_PGDO_PU_1_CFG_SEL_ON_OFF |
            PDSS_PGDO_PU_1_CFG_AUTO_MODE);
    CyDelayUs (10);

    /* Disable all auto mode configuration. */
    pd->pgdo_pu_2_cfg  = 0;
    pd->pgdo_pu_1_cfg &= ~(PGDO_PU_1_CFG_AUTO_SEL_MASK | PDSS_PGDO_PU_1_CFG_AUTO_MODE);
    CyDelayUs (10);

    /* Program PGDO back to its default state. */
    pd->pgdo_pu_1_cfg &= ~(PDSS_PGDO_PU_1_CFG_SEL_ON_OFF
            | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_ON_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE
            | PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_ON_VALUE | PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE);
#endif /* (defined(CCG5C) || defined(CCG6)) */

#ifdef CCG5
    /* Disable all auto mode configuration. */
    pd->pgdo_2_cfg[1]  = 0;
    pd->pgdo_1_cfg[1] &= ~(PGDO_1_CFG_AUTO_SEL_MASK | PDSS_PGDO_1_CFG_AUTO_MODE);

    /* Program PGDO back to its default (OFF) state. */
    pd->pgdo_1_cfg[1] &= ~(PDSS_PGDO_1_CFG_SEL_ON_OFF |
            PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE);
#endif /* CCG5 */

    /*
     * The edge detector may have triggered later than previous fault.
     * In this case, this needs to be cleared.
     */
    pd_reset_edge_det(port, true);
}

void pd_internal_cfet_on(uint8_t port, bool turn_on_seq)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;
    uint8_t  mask;
    (void)turn_on_seq;

    /* We can turn source FET OFF when turning sink FET ON. */
    pd_internal_pfet_off(port, turn_on_seq);

    /* Reset the edge detector. */
    pd_reset_edge_det(port, false);

    mask = CyEnterCriticalSection ();

#if CCG_PD_DUALPORT_ENABLE
    /* Make sure both PD ports are idle before enabling the FET. */
    while (
            ((gl_pdss[0]->status & (PDSS_STATUS_RX_BUSY | PDSS_STATUS_CC_DATA_VALID)) != 0) ||
            ((gl_pdss[1]->status & (PDSS_STATUS_RX_BUSY | PDSS_STATUS_CC_DATA_VALID)) != 0)
          );
#else
    /* Make sure the PD port is idle before enabling the FET. */
    while ((pd->status & (PDSS_STATUS_RX_BUSY | PDSS_STATUS_CC_DATA_VALID)) != 0);
#endif /* CCG_PD_DUALPORT_ENABLE */

#ifdef CCG6
    /* Configure the soft start for the consumer FET. */
    pd_internal_cfet_soft_start_on (port, gl_ccgx_cfet_on[port]);
#endif /* CCG6 */

#ifdef CCG5C
    /* Enable the pull-down for the FET. */
    pd->pgdo_pd_cfg = PDSS_PGDO_PD_CFG_DEFAULT;
#endif /* CCG5C */

    gl_ccgx_cfet_on[port] = true;

#ifdef CCG5
    /* Turn on the fet by setting ENABLE_ON bit. */
    regval = pd->pgdo_1_cfg[0];
    regval |= (PDSS_PGDO_1_CFG_SEL_ON_OFF | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
    pd->pgdo_1_cfg[0] |= regval;
#else /* !CCG5 */
    /* Turn on the fet by setting ENABLE_ON bit. */
    regval = pd->pgdo_1_cfg;
    regval |= (PDSS_PGDO_1_CFG_SEL_ON_OFF | PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE);
    pd->pgdo_1_cfg |= regval;
#endif /* CCG5 */

    CyExitCriticalSection (mask);
}

void pd_internal_cfet_off(uint8_t port, bool turn_off_seq)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Legacy parameter - Not used. */
    (void)turn_off_seq;

#ifdef CCG5
    /* Disable all auto mode configuration. */
    pd->pgdo_2_cfg[0]  = 0;
    pd->pgdo_1_cfg[0] &= ~(PGDO_1_CFG_AUTO_SEL_MASK | PDSS_PGDO_1_CFG_AUTO_MODE);

    /* Program PGDO back to its default (OFF) state. */
    pd->pgdo_1_cfg[0] &= ~(PDSS_PGDO_1_CFG_SEL_ON_OFF |
           PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE);
#else /* !CCG5 */
    /* Disable all auto mode configuration. */
    pd->pgdo_2_cfg  = 0;
    pd->pgdo_1_cfg &= ~(PGDO_1_CFG_AUTO_SEL_MASK | PDSS_PGDO_1_CFG_AUTO_MODE);

    /* Program PGDO back to its default (OFF) state. */
    pd->pgdo_1_cfg &= ~(PDSS_PGDO_1_CFG_SEL_ON_OFF |
            PDSS_PGDO_1_CFG_PGDO_EN_LV_ON_VALUE | PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE);
#endif /* CCG5 */

    if (gl_ccgx_cfet_on[port])
    {
#ifdef CCG6
        /* Make sure the soft start of the FET is disabled. */
        pd_internal_cfet_soft_start_off (port);
#endif /* CCG6 */
    }

    gl_ccgx_cfet_on[port] = false;
}

/*
 * CCG5 VBus Discharge implementation:
 * -----------------------------------
 * We use a high resistance (~2 KOhms) when the voltage is above 6 V; and a lower resistance (~500 Ohms)
 * at lower voltages. Also, the circuit is left on continuously until turned off by the app. logic.
 */

/* Vbus discharge drive strength at high voltages for CCG5/CCG5C/CCG6 Silicon. */
#define DISCHG_DRIVE_STRENGTH_VBUS_HI_REVA      (0x03)

/* Vbus discharge drive strength at low voltages for CCG5/CCG5C/CCG6 Silicon. */
#define DISCHG_DRIVE_STRENGTH_VBUS_LO_REVA      (0x0F)

/* Vbus discharge monitoring period in ms. */
#define DISCHG_MONITOR_PERIOD                   (1u)

/*
 * Safe voltage level that can be allowed on VBus while discharge is in progress.
 * This value is equivalent to about 6 V assuming ADC reference voltage of 2 V.
 */
#define VBUS_SAFE_LEVEL                         (0x3D)

/*
 * This function implements a discharge monitoring task.
 * This is used to reduce the resistance on the discharge path once voltage has dropped to a safe region.
 */
void pd_discharge_monitor_cb(uint8_t port, timer_id_t id)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;
    uint8_t  vbus_val;

    /*
     * Measure the VBus voltage and set the discharge path strength based on the voltage.
     */
    vbus_val = pd_adc_sample(port, APP_VBUS_POLL_ADC_ID, APP_VBUS_POLL_ADC_INPUT);

    /*
     * Update discharge series resistance if voltage is in the SAFE range.
     */
    if (vbus_val < VBUS_SAFE_LEVEL)
    {
        regval = pd->dischg_shv_ctrl;
        regval &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK;

        regval |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG |
                  (DISCHG_DRIVE_STRENGTH_VBUS_LO_REVA << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));

        pd->dischg_shv_ctrl = regval;
    }
    else
    {
        /* Start ON timer so that discharge monitoring can be continued. */
        timer_start(port, id, DISCHG_MONITOR_PERIOD, pd_discharge_monitor_cb);
    }
}

void pd_internal_vbus_discharge_on(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t regval;

    /* Enable the VBus discharge circuit. */
    regval = pd->dischg_shv_ctrl;
    regval &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_DS_MASK;
    regval |= (PDSS_DISCHG_SHV_CTRL_DISCHG_EN | PDSS_DISCHG_SHV_CTRL_DISCHG_EN_CFG |
              (DISCHG_DRIVE_STRENGTH_VBUS_HI_REVA << PDSS_DISCHG_SHV_CTRL_DISCHG_DS_POS));
    pd->dischg_shv_ctrl = regval;

    /* Start a timer to monitor the VBus discharge progress. */
    timer_start(port, VBUS_DISCHARGE_SCHEDULE_TIMER, DISCHG_MONITOR_PERIOD, pd_discharge_monitor_cb);
}

void pd_internal_vbus_discharge_off(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Stop the discharge schedule update timer. */
    timer_stop(port, VBUS_DISCHARGE_SCHEDULE_TIMER);

    /* Disable the discharge circuit. */
    pd->dischg_shv_ctrl &= ~PDSS_DISCHG_SHV_CTRL_DISCHG_EN;
}

#if ((BATTERY_CHARGING_ENABLE) && (defined(CCG5)))

/* BC 1.2 source implementation for CCG5. */

static volatile bool cdp_sm_active[NO_OF_TYPEC_PORTS] = {
    false
#if CCG_PD_DUALPORT_ENABLE
    ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

static volatile bool cdp_vdmsrc_enabled[NO_OF_TYPEC_PORTS] = {
    false
#if CCG_PD_DUALPORT_ENABLE
    ,
    false
#endif /* CCG_PD_DUALPORT_ENABLE */
};

/* Check if CDP state machine is busy. */
bool ccg_is_cdp_sm_busy(uint8_t port)
{
    bool cdp_busy = false;

    if (cdp_sm_active[port])
    {
        if (cdp_vdmsrc_enabled[port])
            cdp_busy = true;
        else
        {
            /* Wake periodically to check voltage while the CDP state machine is running. */
            if (!timer_is_running (port, APP_BC_GENERIC_TIMER1))
            {
                timer_start_wocb (port, APP_BC_GENERIC_TIMER1, 15);
            }
        }
    }

    return (cdp_busy);
}

/* Enable BC 1.2 operation as DCP. */
void ccg_bc_dcp_en(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Enable charger detect isolation switch. */
    pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    CyDelayUs(10);

    pd->chgdet_0_ctrl = 0;
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_EN_CHGDET | PDSS_CHGDET_0_CTRL_DCP_EN;
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;
}

/* Disable BC 1.2 operation. */
void ccg_bc_dis(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable charger detect interrupt. */
    pd->intr3_mask &= ~PDSS_INTR3_MASK_CHGDET_CHANGED_MASK;

    /* Disable outputs from the CHGDET block. */
    pd->chgdet_1_ctrl = 0;

    /* Disable voltage source of D- pin. */
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_VDM_SRC_EN;

    /* Disable current sink on the D+ pin. */
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_IDP_SNK_EN;

    pd->chgdet_0_ctrl = 0;
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_PD;

    /* Make sure charger detect block is isolated from DP/DM connections. */
    pd->dpdm_ctrl &= ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK;

    /* Move all state machines to idle state. */
    cdp_sm_active[port]      = false;
    cdp_vdmsrc_enabled[port] = false;
}

/* BC 1.2 CDP state machine. */
bool ccg_bc_cdp_sm(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t val;

    if (cdp_sm_active[port] == false)
        return false;

    /* Clear any pending charger detect block interrupt. */
    pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;

    /* Check whether D+ voltage is in the 0.325 - 0.85 V window. */
    if ((pd->intr3_status_0 & PDSS_INTR3_STATUS_0_CHGDET_STATUS) != 0)
    {
        /* Voltage is above 0.325 V. */

        /* Change Vref to 0.8 V. */
        val = pd->chgdet_0_ctrl;
        val = (val & ~PDSS_CHGDET_0_CTRL_VREF_SEL_MASK) | (2 << PDSS_CHGDET_0_CTRL_VREF_SEL_POS);
        pd->chgdet_0_ctrl = val;
        CyDelayUs (1);

        if ((pd->intr3_status_0 & PDSS_INTR3_STATUS_0_CHGDET_STATUS) == 0)
        {
            /* D+ voltage is in the correct window. Enable voltage source on D-. */
            if (cdp_vdmsrc_enabled[port] == 0)
            {
                pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_VDM_SRC_EN;
                cdp_vdmsrc_enabled[port] = true;
            }
        }
        else
        {
            /* Voltage has gone beyond 0.8 V. Turn off Vdm_src, and return to idle state. */
            ccg_bc_dis (port);
            return false;
        }
    }
    else
    {
        /* Voltage has fallen below 0.325 V again. Disable voltage source on D-. */
        if (cdp_vdmsrc_enabled[port] != 0)
        {
            pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_VDM_SRC_EN;
            cdp_vdmsrc_enabled[port] = false;
        }
    }

    /* Change Vref back to 0.325 V. */
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_VREF_SEL_MASK;

    /* Configure the charger detect interrupt to fire on any change in voltage. */
    pd->intr3_cfg_chgdet |= (3 << PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_POS);
    pd->intr3_mask       |= PDSS_INTR3_MASK_CHGDET_CHANGED_MASK;

    /* State machine has more work to do. */
    return true;
}

#if (CCG5_CDP_WAIT_DURATION != 0)

static volatile uint32_t cdp_time_remaining[NO_OF_TYPEC_PORTS];

static void cdp_timeout_timer_cb (uint8_t port, timer_id_t id)
{
    /* Stop and disable the CDP state machine. */
    if (cdp_sm_active[port])
    {
        cdp_time_remaining[port]--;
        if (cdp_time_remaining[port] == 0)
        {
            ccg_bc_dis (port);
        }
        else
        {
            timer_start (port, id, 1000, cdp_timeout_timer_cb);
        }
    }
    else
    {
        cdp_time_remaining[port] = 0;
    }
}

#endif /* (CCG5_CDP_WAIT_DURATION != 0) */

/* Enable BC 1.2 operation as CDP. */
void ccg_bc_cdp_en(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t val;

    /* Enable charger detect isolation switch. */
    pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    CyDelayUs(10);

    /* Enable the charger detect block. */
    pd->chgdet_0_ctrl = 0;
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_EN_CHGDET;
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;

    /* Select 0.325V as VRef for comparison. */
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_VREF_SEL_MASK;

    /* Enable current sink on the D+ pin. */
    pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_IDP_SNK_EN;

    /* Disable the filtering of the chgdet comparator output and enable bypass. */
    pd->intr3_cfg_chgdet &= ~PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN;
    pd->intr3_cfg_chgdet |= PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_BYPASS;

    /* Connect D+ to positive input of comparator and Vref to negative input. */
    val = pd->chgdet_0_ctrl;
    val &= ~(PDSS_CHGDET_0_CTRL_CMP_INP_SEL_MASK | PDSS_CHGDET_0_CTRL_CMP_INN_SEL_MASK);
    val |= (2 << PDSS_CHGDET_0_CTRL_CMP_INP_SEL_POS) | (1 << PDSS_CHGDET_0_CTRL_CMP_INN_SEL_POS);
    pd->chgdet_0_ctrl = val;

    /* Enable comparator */
    pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_EN_COMP_CHGDET;

    cdp_sm_active[port] = true;
    CyDelayUs (1);
    ccg_bc_cdp_sm (port);

#if (CCG5_CDP_WAIT_DURATION != 0)
    /* Use a 1 second timer to count the user-defined CDP wait duration out. */
    cdp_time_remaining[port] = CCG5_CDP_WAIT_DURATION;
    timer_start (port, APP_BC_GENERIC_TIMER2, 1000, cdp_timeout_timer_cb);
#endif /* (CCG5_CDP_WAIT_DURATION != 0) */
}

#endif /* ((BATTERY_CHARGING_ENABLE) && (defined(CCG5))) */

static dpdm_mux_cfg_t gl_cur_dpdm_cfg = DPDM_MUX_CONN_NONE;

/* Configure DP/DM MUX. */
void ccg_config_dp_dm_mux(uint8_t port, dpdm_mux_cfg_t conf)
{
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Do nothing if the configuration is already correct. */
    if (gl_cur_dpdm_cfg == conf)
    {
        return;
    }

    gl_cur_dpdm_cfg = conf;

    if (conf == DPDM_MUX_CONN_NONE)
    {
        /* Disable pump first. PUMP[3] is for Port 0, and PUMP[2] is for Port 1. */
        ccg5_pump_disable (port, 2 - port);
        CyDelayUs (2);

        /* Turn off all switches and isolate outputs. */
        pd->dpdm_ctrl &= ((PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK)
#if (defined(CCG5C) || defined(CCG6))
                | (PDSS_DPDM_CTRL_DCP_SHORT_DPDM_TOP |
                    PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM)
#endif /* (defined(CCG5C) || defined(CCG6)) */
                );
    }
    else
    {
        /* Enable the output switch and configure required bits. */
        pd->dpdm_ctrl = (pd->dpdm_ctrl & PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) |
            ((uint32_t)conf | PDSS_DPDM_CTRL_DPDM_ISO_N);
        CyDelayUs (2);

        /* Now enable pump for slow ramp of signals. */
        ccg5_pump_enable (port, 2 - port);
    }
}

/* SBU1 and SBU2 switch state. */
sbu_switch_state_t gl_sbu1_state[NO_OF_TYPEC_PORTS];
sbu_switch_state_t gl_sbu2_state[NO_OF_TYPEC_PORTS];

/* AUX1 and AUX2 resistor configuration. */
aux_resistor_config_t gl_aux1_config[NO_OF_TYPEC_PORTS];
aux_resistor_config_t gl_aux2_config[NO_OF_TYPEC_PORTS];

ccg_status_t sbu_switch_configure(uint8_t port, sbu_switch_state_t sbu1_state, sbu_switch_state_t sbu2_state)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t sbu1val;
    uint32_t sbu2val;
    uint8_t  intstate;

    /* Check that state values are withing allowed range. */
    if ((sbu1_state >= SBU_MAX_STATE) || (sbu2_state >= SBU_MAX_STATE))
    {
        /* Don't service the request. */
        return CCG_STAT_INVALID_ARGUMENT;
    }

    intstate = CyEnterCriticalSection();
    if ((sbu1_state == SBU_NOT_CONNECTED) && (sbu2_state == SBU_NOT_CONNECTED))
    {
        /* Turn off pump first and then disable all switches. */
        ccg5_pump_disable (port, 1 - port);
        CyDelayUs (2);

        pd->sbu20_sbu1_en_1_ctrl = 0;
        pd->sbu20_sbu2_en_1_ctrl = 0;
    }
    else
    {
        /* SBU1 connection. */
        /* Turn the switch off when OVP is detected on any of the CC or SBU lines. */
        sbu1val = PDSS_SBU20_SBU1_EN_1_CTRL_SEL_ON_OFF |
            PDSS_SBU20_SBU1_EN_1_CTRL_SEL_CC1_OVP | PDSS_SBU20_SBU1_EN_1_CTRL_SEL_CC2_OVP |
            PDSS_SBU20_SBU1_EN_1_CTRL_SBU1_SEL_SBU1_OVP | PDSS_SBU20_SBU1_EN_1_CTRL_SBU1_SEL_SBU2_OVP;

        switch (sbu1_state)
        {
            case SBU_CONNECT_AUX1:
                sbu1val |= PDSS_SBU20_SBU1_EN_1_CTRL_AUXP_SBU1_EN_ON_VALUE;
                break;

            case SBU_CONNECT_AUX2:
                sbu1val |= PDSS_SBU20_SBU1_EN_1_CTRL_AUXN_SBU1_EN_ON_VALUE;
                break;

            case SBU_CONNECT_LSTX:
                sbu1val |= PDSS_SBU20_SBU1_EN_1_CTRL_LSTX_SBU1_EN_ON_VALUE;
                break;

            case SBU_CONNECT_LSRX:
                sbu1val |= PDSS_SBU20_SBU1_EN_1_CTRL_LSRX_SBU1_EN_ON_VALUE;
                break;

            default:
                sbu1val = 0;
                break;
        }

        /* SBU2 connection. */
        /* Turn the switch off when OVP is detected on any of the CC or SBU lines. */
        sbu2val = PDSS_SBU20_SBU2_EN_1_CTRL_SEL_ON_OFF |
            PDSS_SBU20_SBU2_EN_1_CTRL_SEL_CC1_OVP | PDSS_SBU20_SBU2_EN_1_CTRL_SEL_CC2_OVP |
            PDSS_SBU20_SBU2_EN_1_CTRL_SBU2_SEL_SBU1_OVP | PDSS_SBU20_SBU2_EN_1_CTRL_SBU2_SEL_SBU2_OVP;

        switch (sbu2_state)
        {
            case SBU_CONNECT_AUX1:
                sbu2val |= PDSS_SBU20_SBU2_EN_1_CTRL_AUXP_SBU2_EN_ON_VALUE;
                break;

            case SBU_CONNECT_AUX2:
                sbu2val |= PDSS_SBU20_SBU2_EN_1_CTRL_AUXN_SBU2_EN_ON_VALUE;
                break;

            case SBU_CONNECT_LSTX:
                sbu2val |= PDSS_SBU20_SBU2_EN_1_CTRL_LSTX_SBU2_EN_ON_VALUE;
                break;

            case SBU_CONNECT_LSRX:
                sbu2val |= PDSS_SBU20_SBU2_EN_1_CTRL_LSRX_SBU2_EN_ON_VALUE;
                break;

            default:
                sbu2val = 0;
                break;
        }

        pd->sbu20_sbu1_en_1_ctrl = sbu1val;
        pd->sbu20_sbu2_en_1_ctrl = sbu2val;

        /* Provide some delay and turn the pump on. */
        CyDelayUs (5);
        ccg5_pump_enable (port, 1 - port);
    }

    /* Store SBU1 and SBU2 states. */
    gl_sbu1_state[port] = sbu1_state;
    gl_sbu2_state[port] = sbu2_state;

    CyExitCriticalSection(intstate);
    return CCG_STAT_SUCCESS;
}

sbu_switch_state_t get_sbu1_switch_state(uint8_t port)
{
    return gl_sbu1_state[port];
}

sbu_switch_state_t get_sbu2_switch_state(uint8_t port)
{
    return gl_sbu2_state[port];
}

#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

//Venkat,18Nov'22, Inorder to optimize the size but to make QC functionalities work, 
//commenting whole bc statemachine but extracting only required functions as per our usecase so handling that using this macro
#if(!BC_BlOCK_SIZE_OPTIMIZE)
ccg_status_t chgb_init(uint8_t cport, bc_phy_cbk_t cbk)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    PPDSS_REGS_T pd = gl_pdss[cport];

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    pdss_stat->bc_phy_cbk = cbk;

#if (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || defined(CCG5C)
    pd->intr9_cfg_bch_det[cport] &= ~(PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN  |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_MASK);
#elif defined(CCG5)
    pd->intr3_cfg_chgdet &= ~(PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN|
                PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_SEL_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN  |
                PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_SEL_MASK);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || (defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}
ccg_status_t chgb_enable(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || (defined(CCG5C))
    /* Enable Charger detect block */
    pd->bch_det_0_ctrl[cport] = PDSS_BCH_DET_0_CTRL_EN_CHGDET;
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;

    /*
     * Clock to this block is required for wakeup functionality via
     * QCOM_RCVR_Dx_CHANGED interrupt. Enabling this at the beginning.
     */
#if (defined(CCG6) || defined(CCG5C))
    pd->ctrl |= (PDSS_CTRL_AFC_ENABLED);

    /* Connect the charger detect block to the D+/D- signals on the appropriate side of the Type-C connector. */
    if(dpm_get_status(cport)->polarity)
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (2 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    else
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
#else
    /* CCG3PA and CCG3PA2 */
    PDSS->ctrl |= (1 << (PDSS_CTRL_AFC_ENABLED_POS + cport));
#endif /*CCGx*/

#elif (defined(CCG5))
    /* Enable Charger detect block */
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_EN_CHGDET;
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;
#endif /* CCGx */

    CyDelayUs(50);
    return CCG_STAT_SUCCESS;
}
ccg_status_t chgb_remove_term(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_IDP_SNK_EN |
            PDSS_BCH_DET_0_CTRL_IDM_SNK_EN |
            PDSS_BCH_DET_0_CTRL_VDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
            PDSS_BCH_DET_0_CTRL_IDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_DCP_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN );
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;
#elif defined (CCG5)
    pd->chgdet_0_ctrl &= ~(PDSS_CHGDET_0_CTRL_IDP_SNK_EN |
            PDSS_CHGDET_0_CTRL_IDM_SNK_EN |
            PDSS_CHGDET_0_CTRL_VDP_SRC_EN |
            PDSS_CHGDET_0_CTRL_VDM_SRC_EN |
            PDSS_CHGDET_0_CTRL_IDP_SRC_EN |
            PDSS_CHGDET_0_CTRL_DCP_EN |
            PDSS_CHGDET_0_CTRL_RDM_PD_EN |
            PDSS_CHGDET_0_CTRL_RDM_PU_EN |
            PDSS_CHGDET_0_CTRL_RDP_PD_EN |
            PDSS_CHGDET_0_CTRL_RDP_PU_EN |
            PDSS_CHGDET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_CHGDET_0_CTRL_RDAT_LKG_DM_EN );
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;

#elif (defined (CCG5C) || defined (CCG6))
    pd->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_IDP_SNK_EN |
            PDSS_BCH_DET_0_CTRL_IDM_SNK_EN |
#ifdef CCG6
            PDSS_BCH_DET_0_CTRL_VDP_SRC_EN |
#endif /* CCG6 */
            PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
            PDSS_BCH_DET_0_CTRL_IDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_DCP_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN );
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;
    pd->dpdm_ctrl           &= ~(PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM | PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM);
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}
#endif /**(!BC_BlOCK_SIZE_OPTIMIZE)*/
/* venkat 6Jul'22
   TaskID - V-1-T314 - Set voltage for QC2.0,3.0,4.0 implementation, 
   GRL custom QC sink term application block where this api is specificaly ment to set voltage for QC mode operation
   this block consists of all Qc2.0,Qc.3.0,QC4.0 implementations by changing D+and D- lines voltage results 
   in 5v,9v,12v,20v from source depending on switch case choosen*/

ccg_status_t grl_chgb_apply_sink_term(uint8_t cport,grl_chgb_snkset_term_t testermodeterms,uint8_t * lAppBuffer)
{
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    switch(testermodeterms)
    {
        case GRL_SET_VOLTAGE://01
            if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag == QC2MODE)
            {
              gBufLog(false,0xD1);
              grl_qc2_sink(cport,lAppBuffer);  
            }
            else if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag == QC3MODE)
            {
              gBufLog(false,0xD2);

                grl_chgb_QC3(lAppBuffer);
            }
             
        break;
            
    }
    return CCG_STAT_SUCCESS;
}
#if BATTERY_CHARGING_ENABLE

ccg_status_t chgb_init(uint8_t cport, bc_phy_cbk_t cbk)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    PPDSS_REGS_T pd = gl_pdss[cport];

    if (cbk == NULL)
    {
        return CCG_STAT_BAD_PARAM;
    }

    pdss_stat->bc_phy_cbk = cbk;

#if (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || defined(CCG5C)
    pd->intr9_cfg_bch_det[cport] &= ~(PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN  |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_MASK |
               PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_MASK);
#elif defined(CCG5)
    pd->intr3_cfg_chgdet &= ~(PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN|
                PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_SEL_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN  |
                PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK |
                PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_SEL_MASK);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || (defined(CCG5C) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_enable(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || (defined(CCG5C))
    /* Enable Charger detect block */
    pd->bch_det_0_ctrl[cport] = PDSS_BCH_DET_0_CTRL_EN_CHGDET;
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;

    /*
     * Clock to this block is required for wakeup functionality via
     * QCOM_RCVR_Dx_CHANGED interrupt. Enabling this at the beginning.
     */
#if (defined(CCG6) || defined(CCG5C))
    pd->ctrl |= (PDSS_CTRL_AFC_ENABLED);

    /* Connect the charger detect block to the D+/D- signals on the appropriate side of the Type-C connector. */
    if(dpm_get_status(cport)->polarity)
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (2 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    else
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
#else
    /* CCG3PA and CCG3PA2 */
    PDSS->ctrl |= (1 << (PDSS_CTRL_AFC_ENABLED_POS + cport));
#endif /*CCGx*/

#elif (defined(CCG5))
    /* Enable Charger detect block */
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_EN_CHGDET;
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;
#endif /* CCGx */

    CyDelayUs(50);
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_disable(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || (defined(CCG6)) || (defined(CCG5C))
    pd->bch_det_1_ctrl[cport] = 0;
    pd->bch_det_0_ctrl[cport] = PDSS_BCH_DET_0_CTRL_PD;

    /* Disable all interrupts */
    pd->intr9_mask &= ~(BCH_PORT_0_CMP1_2_INTR_MASK << (cport << 1u));
    pd->intr9 = BCH_PORT_0_CMP1_2_INTR_MASK << (cport << 1u);

#if (!QC_AFC_CHARGING_DISABLED)
    chgb_qc_src_cont_mode_stop(cport);
    chgb_afc_src_stop(cport);
#endif /* (!QC_AFC_CHARGING_DISABLED) */

#if (defined(CCG6) || defined(CCG5C))
    pd->ctrl &= ~(PDSS_CTRL_AFC_ENABLED);
    /* Isolate the charge detect block from the DP DM lines */
    pd->dpdm_ctrl &= ~(PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK | PDSS_DPDM_CTRL_DCP_SHORT_DPDM_TOP | PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM);
#else
    PDSS->ctrl &= ~(1 << (PDSS_CTRL_AFC_ENABLED_POS + cport));
#endif /* (defined(CCG6) || defined(CCG5C)) */

#elif (defined(CCG5))
    pd->chgdet_1_ctrl = 0;
    pd->chgdet_0_ctrl = PDSS_CHGDET_0_CTRL_PD;

    /* Disable all interrupts. */
    pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;
    pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}

void chgb_deepsleep(uint8_t cport)
{
#if ((defined(CCG3PA)) || (defined (CCG3PA2)) || (defined (CCG5C)) || (defined (CCG6)))
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Configuring both edge detection. */
    pd->intr9_cfg_bch_det[cport] |= (PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DM_CFG_MASK |
            PDSS_INTR9_CFG_BCH_DET_QCOM_RCVR_DP_CFG_MASK);

#if (defined(CCG3PA)) || (defined (CCG3PA2))
    PDSS->intr9_mask |= (((1u << PDSS_INTR9_QCOM_RCVR_DM_CHANGED_POS) |
                (1u << PDSS_INTR9_QCOM_RCVR_DP_CHANGED_POS)) << cport);
#else
    pd->intr9_mask |= (PDSS_INTR9_QCOM_RCVR_DM_CHANGED |
            PDSS_INTR9_QCOM_RCVR_DP_CHANGED);
#endif /* (defined(CCG3PA)) || (defined (CCG3PA2)) */
#endif /* ((defined(CCG3PA)) || (defined (CCG3PA2)) || (defined (CCG5C)) || (defined (CCG6))) */

#if (defined(CCG5))
    /* TODO: Update this code for CCG5 support. */
#endif /* CCGx */
}

void chgb_wakeup(uint8_t cport)
{
    /* Nothing to do. */
}

#if (defined(CCG5C) || defined(CCG6))

/*
 * When the CCG5C or CCG6 is configured as a CDP source, we use comparators hooked up to
 * the D+ and D- lines to detect whether a BC 1.2 sink is connected at this stage.
 */

void chgb_config_cdp_comparators(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Configure DDFT mux to output the ORed value of the two comparators on TR_OUT[0] */
    pd->ddft_mux = (pd->ddft_mux & ~PDSS_DDFT_MUX_DDFT0_SEL_MASK) | ( 57<<PDSS_DDFT_MUX_DDFT0_SEL_POS );

    /* Vref = 130mV */
    pd->refgen_2_ctrl &= ~(PDSS_REFGEN_2_CTRL_SEL6_MASK);

    /* Turn on both comparators */
    pd->comp_ctrl[COMP_ID_DP_DETACH] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_DP_DETACH] &= ~PDSS_COMP_CTRL_COMP_PD;
    pd->comp_ctrl[COMP_ID_DM_DETACH] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_DM_DETACH] &= ~PDSS_COMP_CTRL_COMP_PD;
    CyDelayUs(10);
}

void chgb_disable_cdp_comparators(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Turn off both the comparators */
    pd->comp_ctrl[COMP_ID_DP_DETACH] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_DM_DETACH] &= ~PDSS_COMP_CTRL_COMP_ISO_N;

    pd->comp_ctrl[COMP_ID_DP_DETACH] |= PDSS_COMP_CTRL_COMP_PD;
    pd->comp_ctrl[COMP_ID_DM_DETACH] |= PDSS_COMP_CTRL_COMP_PD;
}

void chgb_counter_start(uint8_t counter_id)
{
    PCNT_REGS_T counter = gl_cnt[counter_id];

    /* Counter Init */
    counter->ctrl        = 1 << Timer_1_ONESHOT_SHIFT;

    /* Set the count sel line to tr_in[0] which is 0 + 2 = 2 */
    counter->ctrl        |= 2<<24 ;
    counter->tr_ctrl0   &= ~(0x0f << 4);
    counter->tr_ctrl0   |= (0x02 << 4);

    counter->tr_ctrl1   |= 3 << TCPWM_CNT_TR_CTRL_1_COUNT_EDGE_POS ;
    counter->period      = TCPWM_PERIOD;                                /* Terminal value               */
    Timer_1_SetMode(Timer_1_MODE_TIMER_CAPTURE);
    counter->counter     = 0;                                           /* Initialise counter value     */
    TCPWM->ctrl         |= TCPWM_CTRL_ENABLE_MASK;                      /* Enable the TCPWM block       */
    TCPWM->cmd          |= TCPWM_CMD_START_CMD_MASK;                    /* Start the TCPWM block        */
}

uint16_t chgb_get_counter_value(uint8_t counter_id)
{
    (void) counter_id;
    uint16_t count_val = Timer_1_ReadCounter();
    Timer_1_Stop();
    Timer_1_WriteCounter(0);
    return(count_val);
}

#endif /* (defined(CCG5C) || defined(CCG6)) */

ccg_status_t chgb_apply_src_term(uint8_t cport, chgb_src_term_t charger_term)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Remove any existing terminations. */
    chgb_remove_term(cport);

#if (defined(CCG3PA) || defined(CCG3PA2))
    switch(charger_term)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->bch_det_1_ctrl[cport] |= (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                                 (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                                 (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                                 (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;

#if (!QC_AFC_CHARGING_DISABLED)
        case CHGB_SRC_TERM_QC:
        case CHGB_SRC_TERM_AFC:
            PDSS->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
                PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN);
            break;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        case CHGB_SRC_TERM_DCP:
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_DCP_EN |
                                  PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN);
            break;
        case CHGB_SRC_TERM_SDP:
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
                PDSS_BCH_DET_0_CTRL_RDM_PD_EN);
            break;
        default:
            break;
    }
#elif (defined(CCG5C) || defined(CCG6))

    uint32_t regval = pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK;

    /* Make sure DP/DM connection to SYS side is disabled in all modes other than SDP/CDP. */
    if ((charger_term != CHGB_SRC_TERM_CDP) && (charger_term != CHGB_SRC_TERM_SDP))
    {
        regval &= ~(PDSS_DPDM_CTRL_DPDM_ISO_N | PDSS_DPDM_CTRL_DPDM_SWITCH_CTRL_MASK);
    }

    /* Make sure that charger detect block is connected to the correct set of D+/D- lines. */
    if(dpm_get_status(cport)->polarity)
    {
        pd->dpdm_ctrl = regval | (2 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    else
    {
        pd->dpdm_ctrl = regval | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    CyDelayUs(10);

    switch(charger_term)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->bch_det_1_ctrl[cport] |= (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;

        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;

        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;

#if (!QC_AFC_CHARGING_DISABLED)
        case CHGB_SRC_TERM_QC:
        case CHGB_SRC_TERM_AFC:
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
                PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN);
            break;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        case CHGB_SRC_TERM_DCP:
            if(dpm_get_status(cport)->polarity)
            {
                pd->dpdm_ctrl |= PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM;
            }
            else
            {
                pd->dpdm_ctrl |= PDSS_DPDM_CTRL_DCP_SHORT_DPDM_TOP;
            }
            break;

        case CHGB_SRC_TERM_SDP:
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
                    PDSS_BCH_DET_0_CTRL_RDM_PD_EN);
            break;

        case CHGB_SRC_TERM_CDP:
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_IDP_SNK_EN);
            break;

        default:
            break;
    }
#elif (defined(CCG5))
    switch(charger_term)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->chgdet_1_ctrl |= (1u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS)|
                (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                (1u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS) |
                (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_DCP:
            pd->chgdet_0_ctrl |= (PDSS_CHGDET_0_CTRL_DCP_EN |
                    PDSS_CHGDET_0_CTRL_RDAT_LKG_DP_EN |
                    PDSS_CHGDET_0_CTRL_RDAT_LKG_DM_EN);
            break;
        default:
            break;
    }
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}

#if (defined(CCG6) || defined(CCG5C))
ccg_status_t chgb_isolate_dpdm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

    /* Isolate charge detect block from DP and DM lines */
    pd->dpdm_ctrl &= ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK;
    return CCG_STAT_SUCCESS;
}
#endif /* (defined(CCG6) || defined(CCG5C)) */
ccg_status_t chgb_apply_dp_pd(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6) || defined(CCG5C))
    pd->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDP_PD_EN;
#elif (defined(CCG5))
    pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_RDP_PD_EN;
#endif /* CCGx */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_remove_dp_pd(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    pd->bch_det_0_ctrl[cport] &= ~PDSS_BCH_DET_0_CTRL_RDP_PD_EN;
#elif defined(CCG5)
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_RDP_PD_EN;
#endif /* CCGx */
    return CCG_STAT_SUCCESS;
}

void chgb_apply_rdat_lkg_dp(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    pd->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN;
#elif defined(CCG5)
    pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_RDAT_LKG_DP_EN;
#endif /* CCGx */
}

void chgb_apply_rdat_lkg_dm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    pd->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN;
#elif defined(CCG5)
    pd->chgdet_0_ctrl |= PDSS_CHGDET_0_CTRL_RDAT_LKG_DM_EN;
#endif /* CCGx */
}

void chgb_remove_rdat_lkg_dp(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    pd->bch_det_0_ctrl[cport] &= ~PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN;
#elif defined(CCG5)
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_RDAT_LKG_DP_EN;
#endif /* CCGx */
}

void chgb_remove_rdat_lkg_dm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    pd->bch_det_0_ctrl[cport] &= ~PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN;
#elif defined(CCG5)
    pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_RDAT_LKG_DM_EN;
#endif /* CCGx */
}

void chgb_apply_apple_src_dp(uint8_t cport, chgb_src_term_t apple_term_id)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    switch(apple_term_id)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->bch_det_1_ctrl[cport] |= (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        default:
            break;
    }
#elif defined(CCG5)
    switch(apple_term_id)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->chgdet_1_ctrl |= (1u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DP_POS);
            break;
        default:
            break;
    }
#endif /* CCGx */
}

void chgb_apply_apple_src_dm(uint8_t cport, chgb_src_term_t apple_term_id)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    switch(apple_term_id)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->bch_det_1_ctrl[cport] |= (1u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->bch_det_1_ctrl[cport] |= (2u << PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        default:
            break;
    }
#elif defined(CCG5)
    switch(apple_term_id)
    {
        case CHGB_SRC_TERM_APPLE_1A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_1A:
            pd->chgdet_1_ctrl |= (1u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        case CHGB_SRC_TERM_APPLE_2_4A:
            pd->chgdet_1_ctrl |= (2u << PDSS_CHGDET_1_CTRL_CHGDET_APPLE_MODE_DM_POS);
            break;
        default:
            break;
    }
#endif /* CCGx */
}

#if ((defined(CCG5C)) || (defined(CCG6)) || (defined(CCG3PA)) || (defined(CCG3PA2)))
void chgb_apply_vdm_src(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDM_SRC_EN;
}

void chgb_remove_vdm_src(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_VDM_SRC_EN);
}

void chgb_remove_apple_src_dp(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->bch_det_1_ctrl[cport] &= ~(PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DP_MASK);
}

void chgb_remove_apple_src_dm(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];
    pd->bch_det_1_ctrl[cport] &= ~(PDSS_BCH_DET_1_CTRL_CHGDET_APPLE_MODE_DM_MASK);
}

#if (!(CCG_SOURCE_ONLY)) && (!BC_SOURCE_ONLY)
void chgb_enable_apple_det(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->bch_det_1_ctrl[port] |= PDSS_BCH_DET_1_CTRL_CHGDET_APP_DET;
}

void chgb_disable_apple_det(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    pd->bch_det_1_ctrl[port] &= ~PDSS_BCH_DET_1_CTRL_CHGDET_APP_DET;
}
/* venkat 6Jul'22
   TaskID - V-1-T314 - Set voltage for QC2.0,3.0,4.0 implementation, 
   GRL custom QC sink term application block where this api is specificaly ment to set voltage for QC mode operation
   this block consists of all Qc2.0,Qc.3.0,QC4.0 implementations by changing D+and D- lines voltage results 
   in 5v,9v,12v,20v from source depending on switch case choosen*/

ccg_status_t grl_chgb_apply_sink_term(uint8_t cport,grl_chgb_snkset_term_t testermodeterms,uint8_t * lAppBuffer)
{
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    switch(testermodeterms)
    {
        case GRL_SET_VOLTAGE://01
            if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag == QC2MODE)
            {
              gBufLog(false,0xD1);
              grl_qc2_sink(cport,lAppBuffer);  
            }
            else if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag == QC3MODE)
            {
              gBufLog(false,0xD2);

                grl_chgb_QC3(lAppBuffer);
            }
             
        break;
            
    }
    return CCG_STAT_SUCCESS;
}
ccg_status_t chgb_apply_sink_term(uint8_t cport, chgb_snk_term_t charger_term)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2) || defined(CCG6))

#ifdef CCG6
    if(dpm_get_status(cport)->polarity)
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (2 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    else
    {
        pd->dpdm_ctrl = (pd->dpdm_ctrl & ~PDSS_DPDM_CTRL_DPDM_T_DPDM_MASK) | (8 << PDSS_DPDM_CTRL_DPDM_T_DPDM_POS);
    }
    CyDelayUs(10);
#endif /* CCG6 */

    switch(charger_term)
    {
        case CHGB_SINK_TERM_SPD:
            /* Pull up on D+ */
            pd->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDP_PU_EN;
            break;
        case CHGB_SINK_TERM_PCD:
            /* Connect VDP_SRC and IDM_SINK. */
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_VDP_SRC_EN |
                                    PDSS_BCH_DET_0_CTRL_IDM_SNK_EN);
            break;
        case CHGB_SINK_TERM_SCD:
            /* Connect VDM_SRC and IDP_SINK. */
            pd->bch_det_0_ctrl[cport] |= (PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
                                    PDSS_BCH_DET_0_CTRL_IDP_SNK_EN);
            break;

#if (!QC_AFC_CHARGING_DISABLED)
        case CHGB_SINK_TERM_AFC:
            /* 0.6 on D+, D- HiZ */
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            /* Remove other terms */
            PDSS->bch_det_0_ctrl[cport] &= ~ (PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
                                    PDSS_BCH_DET_0_CTRL_IDM_SNK_EN);
            break;
#endif /* (!QC_AFC_CHARGING_DISABLED) */

        case CHGB_SINK_TERM_APPLE:
            chgb_remove_term(cport);
            break;
        default:
            break;
    }
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}
#endif /* (!(CCG_SOURCE_ONLY)) */
#endif /* ((defined(CCG5C)) || (defined(CCG6)) || (defined(CCG3PA)) || (defined(CCG3PA2))) */

ccg_status_t chgb_remove_term(uint8_t cport)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2))
    pd->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_IDP_SNK_EN |
            PDSS_BCH_DET_0_CTRL_IDM_SNK_EN |
            PDSS_BCH_DET_0_CTRL_VDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
            PDSS_BCH_DET_0_CTRL_IDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_DCP_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN );
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;
#elif defined (CCG5)
    pd->chgdet_0_ctrl &= ~(PDSS_CHGDET_0_CTRL_IDP_SNK_EN |
            PDSS_CHGDET_0_CTRL_IDM_SNK_EN |
            PDSS_CHGDET_0_CTRL_VDP_SRC_EN |
            PDSS_CHGDET_0_CTRL_VDM_SRC_EN |
            PDSS_CHGDET_0_CTRL_IDP_SRC_EN |
            PDSS_CHGDET_0_CTRL_DCP_EN |
            PDSS_CHGDET_0_CTRL_RDM_PD_EN |
            PDSS_CHGDET_0_CTRL_RDM_PU_EN |
            PDSS_CHGDET_0_CTRL_RDP_PD_EN |
            PDSS_CHGDET_0_CTRL_RDP_PU_EN |
            PDSS_CHGDET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_CHGDET_0_CTRL_RDAT_LKG_DM_EN );
    pd->chgdet_1_ctrl = PDSS_CHGDET_1_CTRL_CHGDET_ISO_N;

#elif (defined (CCG5C) || defined (CCG6))
    pd->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_IDP_SNK_EN |
            PDSS_BCH_DET_0_CTRL_IDM_SNK_EN |
#ifdef CCG6
            PDSS_BCH_DET_0_CTRL_VDP_SRC_EN |
#endif /* CCG6 */
            PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
            PDSS_BCH_DET_0_CTRL_IDP_SRC_EN |
            PDSS_BCH_DET_0_CTRL_DCP_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PD_EN |
            PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DP_EN |
            PDSS_BCH_DET_0_CTRL_RDAT_LKG_DM_EN );
    pd->bch_det_1_ctrl[cport] = PDSS_BCH_DET_1_CTRL_CHGDET_ISO_N;
    pd->dpdm_ctrl           &= ~(PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM | PDSS_DPDM_CTRL_DCP_SHORT_DPDM_BOTTOM);
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}

bool chgb_set_comp(uint8_t cport, uint8_t comp_idx, chgb_comp_pinput_t p_input,
    chgb_comp_ninput_t n_input, chgb_vref_t vref, chgb_comp_edge_t edge)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2))
    uint32_t volatile *bch_ctrl_p;
    uint32_t volatile *cfg_bch_det_p;
    uint32_t regVal;
    uint32_t temp_bch_ctrl;
    uint32_t temp_intr_cfg;
    uint32_t temp_intr_mask;
    uint32_t intr_mask = (BCH_PORT_0_CMP1_INTR_MASK << comp_idx) << (cport << 1);
    uint32_t cfg_clr_mask;
    uint32_t cfg_pos;
    bool result = false;
    uint8_t intr_state;
    uint32_t filt_bypass_bit_mask;
    uint32_t cfg_set_mask;

    intr_state = CyEnterCriticalSection();

    cfg_bch_det_p = &pd->intr9_cfg_bch_det[cport];

    if (comp_idx == 0u)
    {
        bch_ctrl_p = &pd->bch_det_0_ctrl[cport];
        cfg_clr_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_RESET;
        filt_bypass_bit_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_BYPASS;
        cfg_set_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
            (2 << PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_POS);
        if (edge == CHGB_COMP_EDGE_FALLING)
        {
            cfg_set_mask |= PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_RESET;
        }
        cfg_pos = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_POS;

    }
    else
    {
        bch_ctrl_p = &pd->bch_det_1_ctrl[cport];
        cfg_clr_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN  |
                   PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_MASK |
                   PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_MASK |
                   PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_RESET;
        filt_bypass_bit_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_BYPASS;
        cfg_set_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN |
            (2 << PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_POS);
        if (edge == CHGB_COMP_EDGE_FALLING)
        {
            cfg_set_mask |= PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_RESET;
        }
        cfg_pos = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_POS;
    }

    temp_bch_ctrl = *bch_ctrl_p;
    temp_intr_cfg = *cfg_bch_det_p;
    temp_intr_mask =  pd->intr9_mask;

    pd->intr9_mask &= ~intr_mask;

    regVal = temp_bch_ctrl;
    regVal &= ~(PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_MASK |
                PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_MASK |
                PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_MASK |
                PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_SEL_MASK |
                PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_EN);
    regVal |= ((n_input << PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_POS) |
               (p_input << PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_POS) |
               (vref << PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_POS) |
               PDSS_BCH_DET_0_CTRL_EN_COMP1_CHGDET);

    if (vref == CHGB_VREF_0_425V)
    {
        /* Enable +100mV offset to VREF to get 0.425V from 0.325V/ */
        regVal |= ((5 << PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_SEL_POS) |
                PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_EN);
    }

    /* Enable comparator */
    *bch_ctrl_p = regVal;
    CyDelayUs(10);

    if (pd->intr9_status_0 & intr_mask)
    {
        result = true;
    }

    /* Enable Interrupt and check if condition already exists */
    if(edge == CHGB_COMP_NO_INTR)
    {
        *bch_ctrl_p = temp_bch_ctrl;
        *cfg_bch_det_p |= filt_bypass_bit_mask;
        *cfg_bch_det_p = temp_intr_cfg;
        pd->intr9_mask = temp_intr_mask;
        CyDelayUs(10);
    }
    else
    {
        /* Configure edge detection. */
        *cfg_bch_det_p &= ~(cfg_clr_mask | filt_bypass_bit_mask);
        *cfg_bch_det_p |= (edge << cfg_pos);
        *cfg_bch_det_p |= cfg_set_mask;

        /* Enable comparator interrupts. */
        pd->intr9_mask |= intr_mask;
    }

    /* Clear comparator interrupt. */
    pd->intr9 = intr_mask;

    CyExitCriticalSection(intr_state);
    return result;

#elif (defined(CCG6) || defined(CCG5C))

    uint32_t volatile *bch_ctrl_p;
    uint32_t volatile *cfg_bch_det_p;
    uint32_t regVal;
    uint32_t temp_bch_ctrl;
    uint32_t temp_intr_cfg;
    uint32_t temp_intr_mask;
    uint32_t intr_mask = (BCH_PORT_0_CMP1_INTR_MASK << comp_idx);
    uint32_t cfg_clr_mask;
    uint32_t cfg_pos;
    bool result = false;
    uint8_t intr_state;
    uint32_t filt_bypass_bit_mask;
    uint32_t cfg_set_mask;
    intr_state = CyEnterCriticalSection();

    cfg_bch_det_p = &pd->intr9_cfg_bch_det[cport];

    if (comp_idx == 0u)
    {
        bch_ctrl_p = &pd->bch_det_0_ctrl[cport];
        cfg_clr_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_RESET;
        filt_bypass_bit_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_BYPASS;
        cfg_set_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN |
            (2 << PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_SEL_POS);

        if (edge == CHGB_COMP_EDGE_FALLING)
        {
            cfg_set_mask |= PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_RESET;
        }
        cfg_pos = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_CFG_POS;
    }
    else
    {
        bch_ctrl_p = &pd->bch_det_1_ctrl[cport];
        cfg_clr_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN  |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_MASK |
            PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_RESET;
        filt_bypass_bit_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_BYPASS;
        cfg_set_mask = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN |
            (2 << PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_SEL_POS);

        if (edge == CHGB_COMP_EDGE_FALLING)
        {
            cfg_set_mask |= PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_RESET;
        }
        cfg_pos = PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_CFG_POS;
    }

    temp_bch_ctrl  = *bch_ctrl_p;
    temp_intr_cfg  = *cfg_bch_det_p;
    temp_intr_mask =  pd->intr9_mask;

    pd->intr9_mask &= ~intr_mask;

    regVal = temp_bch_ctrl;
    regVal &= ~(PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_MASK |
            PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_MASK |
            PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_MASK |
            PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_SEL_MASK |
            PDSS_BCH_DET_0_CTRL_CMP1_OFFSET_EN);
    regVal |= ((n_input << PDSS_BCH_DET_0_CTRL_CMP1_INN_SEL_POS) |
            (p_input << PDSS_BCH_DET_0_CTRL_CMP1_INP_SEL_POS) |
            (vref << PDSS_BCH_DET_0_CTRL_CMP1_VREF_SEL_POS) |
            PDSS_BCH_DET_0_CTRL_EN_COMP1_CHGDET);

    /* Enable comparator */
    *bch_ctrl_p = regVal;
    CyDelayUs(10);

    if (pd->intr9_status_0 & intr_mask)
    {
        result = true;
    }

    /* Enable Interrupt and check if condition already exists */
    if(edge == CHGB_COMP_NO_INTR)
    {
        *bch_ctrl_p = temp_bch_ctrl;
        *cfg_bch_det_p |= filt_bypass_bit_mask;
        *cfg_bch_det_p = temp_intr_cfg;
        pd->intr9_mask = temp_intr_mask;
        CyDelayUs(10);
    }
    else
    {
        /* Configure edge detection. */
        *cfg_bch_det_p &= ~(cfg_clr_mask | filt_bypass_bit_mask);
        *cfg_bch_det_p |= (edge << cfg_pos);
        *cfg_bch_det_p |= cfg_set_mask;

        /* Enable comparator interrupts. */
        pd->intr9_mask |= intr_mask;
    }

    /* Clear comparator interrupt. */
    pd->intr9 = intr_mask;

    CyExitCriticalSection(intr_state);
    return result;

#elif (defined(CCG5))

    uint32_t regVal;
    uint32_t temp_chgdet_ctrl_0;
    uint32_t temp_intr3_cfg_0;
    uint32_t temp_intr3_mask;
    bool out = false;
    bool result = false;
    uint8_t intr_state;

    intr_state = CyEnterCriticalSection();
    if(comp_idx == 0)
    {
        temp_chgdet_ctrl_0 = pd->chgdet_0_ctrl;
        temp_intr3_cfg_0 = pd->intr3_cfg_chgdet;
        temp_intr3_mask =  pd->intr3_mask;

        pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;

        regVal = pd->chgdet_0_ctrl;
        regVal &= ~(PDSS_CHGDET_0_CTRL_CMP_INN_SEL_MASK |
                PDSS_CHGDET_0_CTRL_CMP_INP_SEL_MASK |
                PDSS_CHGDET_0_CTRL_VREF_SEL_MASK);
        regVal |= ((n_input << PDSS_CHGDET_0_CTRL_CMP_INN_SEL_POS) |
                (p_input << PDSS_CHGDET_0_CTRL_CMP_INP_SEL_POS) |
                (vref << PDSS_CHGDET_0_CTRL_VREF_SEL_POS)|
                PDSS_CHGDET_0_CTRL_EN_COMP_CHGDET );


        /* Enable comparator */
        pd->chgdet_0_ctrl = regVal;

        CyDelayUs(10);

        if (pd->intr3_status_0 & PDSS_INTR3_STATUS_0_CHGDET_STATUS)
        {
            result = true;
        }

        /* Enable Interrupt and check if condition already exists */
        if(edge == CHGB_COMP_NO_INTR)
        {
            pd->chgdet_0_ctrl = temp_chgdet_ctrl_0;
            pd->intr3_cfg_chgdet = temp_intr3_cfg_0;
            pd->intr3_mask = temp_intr3_mask;
            CyDelayUs(10);
        }
        else
        {
            pd->intr3_cfg_chgdet &= ~(PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK | PDSS_INTR3_CFG_CHGDET_CHGDET_FILT_EN);
            pd->intr3_cfg_chgdet |= (edge << PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_POS);

            /* Enable comparator interrupts. */
            pd->intr3_mask |= PDSS_INTR3_CHGDET_CHANGED;
        }

        /* Clear comparator interrupt. */
        pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;

        if(pd->intr3_mask & PDSS_INTR3_CHGDET_CHANGED)
        {
            if (pd->intr3_status_0 & PDSS_INTR3_STATUS_0_CHGDET_STATUS)
            {
                out = true;
            }

            regVal = (pd->intr3_cfg_chgdet & PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_MASK)
                >> PDSS_INTR3_CFG_CHGDET_CHGDET_CFG_POS;

            if (((regVal == CHGB_COMP_EDGE_FALLING) && (out == false)) ||
                ((regVal == CHGB_COMP_EDGE_RISING) && (out == true)))
            {
                /* Raise an interrupt. */
                pd->intr3_set |= PDSS_INTR3_CHGDET_CHANGED;
            }
        }

    }
    else
    {
        /* TODO: Same settings for Comp1 for CCG3PA */
    }

    CyExitCriticalSection(intr_state);
    return result;

#endif /* CCGx */
}

ccg_status_t chgb_stop_comp(uint8_t cport, uint8_t comp_idx)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2))
    if(comp_idx == 0)
    {
        pd->bch_det_0_ctrl[cport] &= ~PDSS_BCH_DET_0_CTRL_EN_COMP1_CHGDET;
        pd->intr9_mask &= ~(BCH_PORT_0_CMP1_INTR_MASK << (cport << 1));
        pd->intr9 = BCH_PORT_0_CMP1_INTR_MASK << (cport << 1);
        pd->intr9_cfg_bch_det[cport] &= ~PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN;
    }
    else
    {
        pd->bch_det_1_ctrl[cport] &= ~PDSS_BCH_DET_1_CTRL_EN_COMP2_CHGDET;
        pd->intr9_mask &= ~(BCH_PORT_0_CMP2_INTR_MASK << (cport << 1));
        pd->intr9 = BCH_PORT_0_CMP2_INTR_MASK << (cport << 1);
        pd->intr9_cfg_bch_det[cport] &= ~PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN;
    }
#elif (defined(CCG6) || defined(CCG5C))
    if(comp_idx == 0)
    {
        pd->bch_det_0_ctrl[cport] &= ~PDSS_BCH_DET_0_CTRL_EN_COMP1_CHGDET;
        pd->intr9_mask &= ~(BCH_PORT_0_CMP1_INTR_MASK << (cport << 1));
        pd->intr9 = BCH_PORT_0_CMP1_INTR_MASK << (cport << 1);
        pd->intr9_cfg_bch_det[cport] &= ~PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP0_FILT_EN;
    }
    else
    {
        pd->bch_det_1_ctrl[cport] &= ~PDSS_BCH_DET_1_CTRL_EN_COMP2_CHGDET;
        pd->intr9_mask &= ~(BCH_PORT_0_CMP2_INTR_MASK << (cport << 1));
        pd->intr9 = BCH_PORT_0_CMP2_INTR_MASK << (cport << 1);
        pd->intr9_cfg_bch_det[cport] &= ~PDSS_INTR9_CFG_BCH_DET_BCH_DET_COMP1_FILT_EN;
    }
#elif (defined(CCG5))
    if(comp_idx == 0)
    {
        pd->chgdet_0_ctrl &= ~PDSS_CHGDET_0_CTRL_EN_COMP_CHGDET;
        pd->intr3_mask &= ~PDSS_INTR3_CHGDET_CHANGED;
        pd->intr3 = PDSS_INTR3_CHGDET_CHANGED;
    }
#endif /* CCGx */

    return CCG_STAT_SUCCESS;
}

bool chgb_get_comp_result(uint8_t cport, uint8_t comp_idx)
{
    PPDSS_REGS_T pd = gl_pdss[cport];

#if (defined(CCG3PA) || defined(CCG3PA2)) || defined(CCG6) || defined(CCG5C)
    if(comp_idx == 0)
    {
        if(pd->intr9_status_0 & (BCH_PORT_0_CMP1_INTR_MASK << (cport << 1)))
        {
            return true;
        }
    }
    else
    {
        if(pd->intr9_status_0 & (BCH_PORT_0_CMP2_INTR_MASK << (cport << 1)))
        {
            return true;
        }
    }
#elif defined(CCG5)
    if(comp_idx == 0)
    {
        if(pd->intr3_status_0 & PDSS_INTR3_STATUS_0_CHGDET_STATUS)
        {
            return true;
        }
    }
#endif /* CCGx */

    return false;
}

#if !(QC_AFC_CHARGING_DISABLED)
ccg_status_t chgb_qc_src_init(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_qc_src_stop(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}

int chgb_get_qc_pulse_count(uint8_t cport)
{
    return gl_pdss_status[cport].bc_qc_pulse_count;
}

void chgb_update_qc_pulse_count(uint8_t cport, int new_count)
{
    uint8_t intr_state = CyEnterCriticalSection();
    gl_pdss_status[cport].bc_qc_pulse_count = gl_pdss_status[cport].bc_qc_pulse_count - new_count;
    CyExitCriticalSection(intr_state);
}

ccg_status_t chgb_qc_src_cont_mode_start(uint8_t cport)
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    uint32_t regval;
    gl_pdss_status[cport].bc_qc_pulse_count = 0;

    /*
     * Enable pulse count and filter. DP filter should look for rising edge
     * and DM filter should look for falling edge.
     */
    PDSS->qc3_chrger_ctrl[cport] |= PDSS_QC3_CHRGER_CTRL_QC3_0_CHG_EN;

    /* Clear the DP/DM Count. */
    PDSS->qc3_chrger_ctrl[cport] |= PDSS_QC3_CHRGER_CTRL_READ_DPDM_COUNT;
    while(PDSS->qc3_chrger_ctrl[cport] & PDSS_QC3_CHRGER_CTRL_READ_DPDM_COUNT);

    regval = PDSS->afc_hs_filter_cfg[cport];

    regval &= ~(PDSS_AFC_HS_FILTER_CFG_DP_FILT_SEL_MASK |
        PDSS_AFC_HS_FILTER_CFG_DM_FILT_SEL_MASK | PDSS_AFC_HS_FILTER_CFG_DP_FILT_EN
        | PDSS_AFC_HS_FILTER_CFG_DM_FILT_EN |
        PDSS_AFC_HS_FILTER_CFG_DP_FILT_RESET);
    PDSS->afc_hs_filter_cfg[cport] = regval;

    regval |= ((QC3_DP_DM_PULSE_FILTER_CLOCK_SEL <<
        PDSS_AFC_HS_FILTER_CFG_DP_FILT_SEL_POS) | (QC3_DP_DM_PULSE_FILTER_CLOCK_SEL
        << PDSS_AFC_HS_FILTER_CFG_DM_FILT_SEL_POS) | PDSS_AFC_HS_FILTER_CFG_DM_FILT_RESET |
            (PDSS_AFC_HS_FILTER_CFG_DP_FILT_EN | PDSS_AFC_HS_FILTER_CFG_DM_FILT_EN));

    PDSS->afc_hs_filter_cfg[cport] = regval;

    /* Enable D+/D- pulse interrupts */
    PDSS->intr6 = QC3_PORT_0_DP_DM_PULSE_MASK << cport;
    PDSS->intr6_mask |= (QC3_PORT_0_DP_DM_PULSE_MASK << cport);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_qc_src_cont_mode_stop(uint8_t cport)
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    gl_pdss_status[cport].bc_qc_pulse_count = 0;
    PDSS->qc3_chrger_ctrl[cport] &= ~PDSS_QC3_CHRGER_CTRL_QC3_0_CHG_EN;
    /* Disable D+/D- pulse interrupts */
    PDSS->intr6_mask &= ~(QC3_PORT_0_DP_DM_PULSE_MASK << cport);
    PDSS->intr6 = QC3_PORT_0_DP_DM_PULSE_MASK << cport;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_afc_src_init(uint8_t cport)
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    uint32_t regval = PDSS->afc_1_ctrl[cport];

    regval &= ~(PDSS_AFC_1_CTRL_TX_RESET |
            PDSS_AFC_1_CTRL_NO_OF_BYTES_TX_MASK);
    regval |= PDSS_AFC_1_CTRL_UPDATE_TXCLK;

    PDSS->afc_1_ctrl[cport] = regval;


    regval = PDSS->afc_2_ctrl[cport];
    /* Update UI clock cylce count for 125kHz clock. */
    regval &= ~PDSS_AFC_2_CTRL_UI_COUNT_MASK;
    regval |= (QC3_DP_DM_PULSE_FILTER_CLOCK_SEL <<
        PDSS_AFC_2_CTRL_UI_COUNT_POS);
    PDSS->afc_2_ctrl[cport] = regval;

    /* Enable filter. */
    PDSS->afc_hs_filter_cfg[cport] &= ~(PDSS_AFC_HS_FILTER_CFG_DM_FILT_SEL_MASK);
    PDSS->afc_hs_filter_cfg[cport] |= (PDSS_AFC_HS_FILTER_CFG_DM_FILT_EN);
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    return CCG_STAT_SUCCESS;
}

#define CHGB_AFC_INTR4_PORT0_ALL_INTR_BIT_MASK      (0x11111001)
#define CHGB_AFC_INTR4_PORT1_ALL_INTR_BIT_MASK      (0x22222002)

ccg_status_t chgb_afc_src_start(uint8_t cport)
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    /* Set opcode for afc source operation */
    PDSS->afc_opcode_ctrl[cport] = AFC_SOURCE_OPCODE;

    /* Enable Interrupts */
    if (cport == 0)
    {
        PDSS->intr4 = CHGB_AFC_INTR4_PORT0_ALL_INTR_BIT_MASK;
    }
    else
    {
        PDSS->intr4 = CHGB_AFC_INTR4_PORT1_ALL_INTR_BIT_MASK;
    }

    PDSS->intr4_mask |= (((1 << PDSS_INTR4_AFC_SM_IDLE_POS) |
                (1 << PDSS_INTR4_UPDATE_PING_PONG_POS) |
                (1 << PDSS_INTR4_AFC_RX_RESET_POS)) << cport);

    pdss_stat->bc_afc_rx_idx = 0;

    /* Start operation */
    PDSS->afc_opcode_ctrl[cport] |= PDSS_AFC_OPCODE_CTRL_OP_CODE_START;

#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
    return CCG_STAT_SUCCESS;
}

ccg_status_t chgb_afc_src_stop(uint8_t cport)
{
#if (defined(CCG3PA) || defined(CCG3PA2))
    uint32_t intr_mask;
    if (cport == 0)
    {
        intr_mask = CHGB_AFC_INTR4_PORT0_ALL_INTR_BIT_MASK;
    }
    else
    {
        intr_mask = CHGB_AFC_INTR4_PORT1_ALL_INTR_BIT_MASK;
    }

    /* Disable Interrupts */
    PDSS->intr4_mask &= ~(intr_mask);
    PDSS->intr4 = intr_mask;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
    return CCG_STAT_SUCCESS;
}

uint8_t* chgb_afc_get_rx_data_ptr(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    return pdss_stat->bc_afc_rx_buf;
}

uint8_t chgb_afc_get_rx_data_count(uint8_t cport)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    return pdss_stat->bc_afc_rx_idx + 1;
}

void chgb_afc_set_tx_data(uint8_t cport, uint8_t* data_ptr, uint8_t count)
{
    pdss_status_t* pdss_stat = &gl_pdss_status[cport];
    pdss_stat->bc_afc_tx_size = count;
    pdss_stat->bc_afc_tx_idx = 0;
    PDSS->afc_1_ctrl[cport] &= ~PDSS_AFC_1_CTRL_NO_OF_BYTES_TX_MASK;
    PDSS->afc_1_ctrl[cport] |= count << PDSS_AFC_1_CTRL_NO_OF_BYTES_TX_POS;
    memcpy(pdss_stat->bc_afc_tx_buf, data_ptr, count);
    chgb_afc_load_tx_data(cport);
}

ccg_status_t chgb_afc_sink_init(uint8_t cport)
{
    /* TODO */
    return CCG_STAT_SUCCESS;
}
#endif /* (!QC_AFC_CHARGING_DISABLED) */
#endif /* BATTERY_CHARGING_ENABLE */

#if ((VBUS_OCP_ENABLE) | (VBUS_SCP_ENABLE) | (VBUS_OVP_ENABLE) | (VBUS_UVP_ENABLE) | (VBUS_RCP_ENABLE))

void pd_fet_automode_enable(uint8_t port, bool pctrl, filter_id_t filter_index)
{
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

#if (defined(CCG3PA) || defined(CCG3PA2))
    /* Provider FET. */
    if (pctrl)
    {
        regval = pd->pgdo_pu_1_cfg;
        /* Enable auto mode. */
        regval |= PDSS_PGDO_PU_1_CFG_AUTO_MODE;
        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
        regval |= PDSS_PGDO_PU_1_CFG_PGDO_PU_IN_LV_OFF_VALUE;
        pd->pgdo_pu_1_cfg = regval;
        /* Select source. */
        pd->pgdo_pu_2_cfg |= 1 << filter_index;
    }
    /* Consumer FET. */
    else
    {
        regval = pd->pgdo_1_cfg;
        /* Enable auto mode. */
        regval |= PDSS_PGDO_1_CFG_AUTO_MODE;
        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;
        pd->pgdo_1_cfg = regval;
        /* Select source. */
        pd->pgdo_2_cfg |= 1 << filter_index;
    }
#elif (defined (CCG5C) || defined (CCG6))
    /* Provider FET. */
    if (pctrl)
    {
        /* CCG5C/CCG6 use PGDO_PU_1_CFG register for Provider FET */
        regval = pd->pgdo_pu_1_cfg;

        /* Enable auto mode. */
        regval |= PDSS_PGDO_PU_1_CFG_AUTO_MODE;

        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
        pd->pgdo_pu_1_cfg = regval;

#ifdef CCG6
        /* Select source. */
        if (filter_index == FILTER_ID_CCG6_SCP)
            pd->pgdo_pu_3_cfg |= PDSS_PGDO_PU_3_CFG_PGDO_EN_SCP_LV;
        else if(filter_index == FILTER_ID_CCG6_RCP)
            pd->pgdo_pu_3_cfg |= PDSS_PGDO_PU_3_CFG_PGDO_EN_RCP_LV | PDSS_PGDO_PU_3_CFG_PGDO_EN_VBUS_OV_LV;
        else
#endif /* CCG6 */
            pd->pgdo_pu_2_cfg |= 1 << filter_index;
    }
    /* Consumer FET. */
    else
    {
        regval = pd->pgdo_1_cfg;
        /* Enable auto mode. */
        regval |= PDSS_PGDO_1_CFG_AUTO_MODE;
        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;
        pd->pgdo_1_cfg = regval;
        /* Select source. */
        pd->pgdo_2_cfg |= 1 << filter_index;
    }
#elif defined (CCG5)
    /* Provider FET. */
    if (pctrl)
    {
        regval = pd->pgdo_1_cfg[1];
        /* Enable auto mode. */
        regval |= PDSS_PGDO_1_CFG_AUTO_MODE;
        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;
        pd->pgdo_1_cfg[1] = regval;
        /* Select source. */
        pd->pgdo_2_cfg[1] |= 1 << filter_index;
    }
    /* Consumer FET. */
    else
    {
        regval = pd->pgdo_1_cfg[0];
        /* Enable auto mode. */
        regval |= PDSS_PGDO_1_CFG_AUTO_MODE;
        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;
        pd->pgdo_1_cfg[0] = regval;
        /* Select source. */
        pd->pgdo_2_cfg[0] |= 1 << filter_index;
    }
#endif /* CCGx */
}

void pd_fet_automode_disable(uint8_t port, bool pctrl, filter_id_t filter_index)
{
    PPDSS_REGS_T pd = gl_pdss[port];

#if (defined(CCG3PA) || defined(CCG3PA2))
    uint32_t regval_1_cfg;
    uint32_t regval_2_cfg;

    if (pctrl)
    {
        /* Remove source. */
        regval_1_cfg = pd->pgdo_pu_1_cfg;
        regval_2_cfg = pd->pgdo_pu_2_cfg;

        regval_2_cfg &= ~(1 << filter_index);

        /* Disable Auto mode if no other source is active. */
        if ((regval_2_cfg == 0) && ((regval_1_cfg & PGDO_PU_1_CFG_AUTO_SEL_MASK) == 0))
        {
           regval_1_cfg &= ~PDSS_PGDO_PU_1_CFG_AUTO_MODE;
        }
        pd->pgdo_pu_2_cfg = regval_2_cfg;
        pd->pgdo_pu_1_cfg = regval_1_cfg;
    }
    else
    {
        /* Remove source. */
        regval_1_cfg = pd->pgdo_1_cfg;
        regval_2_cfg = pd->pgdo_2_cfg;

        regval_2_cfg &= ~(1 << filter_index);

        if ((regval_2_cfg == 0) && ((regval_1_cfg & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            regval_1_cfg &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
        pd->pgdo_2_cfg = regval_2_cfg;
        pd->pgdo_1_cfg = regval_1_cfg;
    }
#elif (defined(CCG5C) || defined(CCG6))
    if (pctrl)
    {
        /* CCG5C/6 uses PGDO_PU_1_CFG register for Provider FET */

#ifdef CCG6
        /* Remove source. */
        if (filter_index == FILTER_ID_CCG6_SCP)
            pd->pgdo_pu_3_cfg &= ~PDSS_PGDO_PU_3_CFG_PGDO_EN_SCP_LV;
        else if(filter_index == FILTER_ID_CCG6_RCP)
            pd->pgdo_pu_3_cfg &= ~(PDSS_PGDO_PU_3_CFG_PGDO_EN_RCP_LV | PDSS_PGDO_PU_3_CFG_PGDO_EN_VBUS_OV_LV);
        else
#endif /* CCG6 */
            pd->pgdo_pu_2_cfg &= ~(1 << filter_index);

        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_pu_2_cfg == 0) && ((pd->pgdo_pu_1_cfg & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_pu_1_cfg &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }
    else
    {
        /* Remove source. */
        pd->pgdo_2_cfg &= ~(1 << filter_index);
        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_2_cfg == 0) && ((pd->pgdo_1_cfg & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_1_cfg &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }
#elif defined (CCG5)
    if (pctrl)
    {
        /* Remove source. */
        pd->pgdo_2_cfg[1] &= ~(1 << filter_index);
        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_2_cfg[1] == 0) && ((pd->pgdo_1_cfg[1] & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }
    else
    {
        /* Remove source. */
        pd->pgdo_2_cfg[0] &= ~(1 << filter_index);
        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_2_cfg[0] == 0) && ((pd->pgdo_1_cfg[0] & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_1_cfg[0] &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }

#endif /* CCGx */
}
#endif /* ((VBUS_OCP_ENABLE) | (VBUS_SCP_ENABLE) | (VBUS_OVP_ENABLE) | (VBUS_UVP_ENABLE) | (VBUS_RCP_ENABLE)) */

#if (defined(CCG3PA) || defined(CCG3PA2))

void pd_lscsa_calc_cfg(uint32_t cur_10mA, uint8_t *gain_sel, uint8_t *vref_sel)
{
    uint8_t gain;
    uint8_t av_sel = 0;
    uint32_t vref;
    bool err_sign;
    int32_t err_target;
    uint32_t vsense = (cur_10mA * gl_vbus_csa_rsense) / 100;

    /*
     * Vsense is now stored in 100uV units as the Rsense value is in 0.1mOhms.
     * The scale value of 10X is retained for better accuracy without float.
     */

    /* Adjust Vsense against min and max. */
    vsense = GET_MIN(vsense, LSCSA_VSENSE_MAX);
    vsense = GET_MAX(vsense, LSCSA_VSENSE_MIN);

    /* Identify the gain and gain_sel settings. */
    if (vsense <= LSCSA_GAIN_150_VSENSE_MAX)
    {
        gain = 150;
        av_sel = LSCSA_AV_SEL_150;
    }
    else if (vsense < LSCSA_GAIN_35_VSENSE_MIN)
    {
        gain = 125;
        av_sel = LSCSA_AV_SEL_125;
    }
    else /* if (vsense >= LSCSA_GAIN_35_VSENSE_MIN) */
    {
        gain = 35;
        av_sel = LSCSA_AV_SEL_35;
    }

    if (gain_sel != NULL)
    {
        *gain_sel = av_sel;
    }

    if (vref_sel != NULL)
    {
        /* Calculate ideal Vref. */
        vref = (vsense * gain) / 10;

        /* Now apply the error correction algorithm. */

        /*
         * The err_target variable is used for first calulating the error and then
         * calculating the error compensated vref value. The calculations done are
         * described at each step. float calculations are very code size sensitive
         * and so the calculations are done as int.
         */

        /*
         * For vsense < 15mV, the spec error is (20 - vsense); for vsense between 15mV
         * and 20mV, the error is 5%; for vsense between 20mV and 30mV, the error is
         * (7 - 0.1 * vsense)%; for vsense between 30mV and 50mV, the error is specd
         * to (5.5 - 0.05*vsense)%.
         *
         * The err_target is converted to 10X value to allow for division of 10 as int.
         * Since the error variation is very small post 30mV, it does not warrant
         * calculating for division with 20.
         */
        if (vsense < 150)
        {
            err_target = (200 - vsense);
        }
        else if (vsense < 200)
        {
            err_target = 50;
        }
        else if (vsense < 300)
        {
            err_target = (70 - (vsense / 10));
        }
        else
        {
            err_target = 55 - (vsense / 20);
        }

        /*
         * The formulae for Vref_new is as follows:
         * Vref_new = Vref * [1 - (spec error @vsense / spec error at 15mV%) * err @15mV]
         * Since actual error at 15mV is stored in the SFLASH at the specified locations
         * along with the sign. The error % is now calculated based on the available
         * data.
         */
        if ((av_sel & 0x10) == 0)
        {
            err_target *= (LSCSA_AMP1_ERR_VAL);
            err_sign = LSCSA_AMP1_ERR_SIGN;
        }
        else
        {
            err_target *= (LSCSA_AMP2_ERR_VAL);
            err_sign = LSCSA_AMP2_ERR_SIGN;
        }
        if (err_sign)
        {
            err_target *= -1;
        }

        /*
         * To make the calculations better, the 5 (spec error at 15mV) and the divider
         * for % (100) are taken upto the numerator and divided at a higher value.
         * Also, an additional 10X is done to accomodate for the err_target being
         * made 10X in previous calculation.
         */
        err_target = (((50000 - err_target) * vref) / 50000);

        /* Minimum Vref allowed is 130mV. So adjust for it to prevent underflow. */
        err_target = GET_MAX(err_target, 130);

        /*
         * err_target now holds the vref_new value in mV. This has to be now converted
         * to register field encoding and returned to the calling function.
         * Vref value can be extracted from the register: vref_sel = (Vref - 130) / 10.
         */
        *vref_sel = ((uint8_t)((err_target - 130) / 10));
    }
}

ccg_status_t pd_lscsa_cfg(lscsa_app_config_t lscsa_app, uint8_t gain_sel_value)
{
    /* Program LSCSA register with the gain value for the lscsa_app application. */
    uint32_t regval;
    uint8_t bit_pos;

    /* Ensure LSCSA app value is not out of bounds. */
    if (lscsa_app >= LSCSA_MAX_CONFIG_VALUE)
    {
        return CCG_STAT_BAD_PARAM;
    }

    /* Read the current value of register. */
    regval = PDSS->lscsa_0_ctrl;

    /*
     * lscsa_0_ctrl register contains gain sel values for all applications (lscsa_app).
     * Get the bit position of gain sel value.
     */
    bit_pos = (lscsa_app << 2) + lscsa_app;

    /*
     * gain_sel_value is actually :
     * Bit 0:3: Gain selection code
     * Bit 4: Output Mux select bit
     */
    regval &= ~(0x1F << bit_pos);
    regval |= ((gain_sel_value & 0x1F) << bit_pos);
    PDSS->lscsa_0_ctrl = regval;

    return CCG_STAT_SUCCESS;
}
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if VBUS_OCP_ENABLE

#if (defined(CCG3PA) || defined(CCG3PA2))
void pd_internal_vbus_ocp_en(uint8_t port, uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        uint8_t mode, uint8_t debounce_ms)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t regval;

    state = CyEnterCriticalSection ();

    if (mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_LSCSA_OCP);
    }

    /* Store OCP parameters. */
    gl_vbus_ocp_mode[port]      = mode;
    gl_ocp_sw_db_ms[port]       = debounce_ms;
    gl_vbus_ocp_pgdo_type[port] = pctrl;

    /* Configure LSCSA for OCP. */
    pd_lscsa_cfg (LSCSA_OCP_CONFIG, av_bw);
    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /* Configure Reference for comparator. */
    regval = pd->refgen_2_ctrl;
    regval &= ~(PDSS_REFGEN_2_CTRL_SEL5_MASK);
    regval |= (vref_sel << PDSS_REFGEN_2_CTRL_SEL5_POS);
    pd->refgen_2_ctrl = regval;

    /* Use Vref SEL 5 for OCP comparator. */
    pd->amux_ctrl |= (1 << VBUS_OCP_AMUX_CTRL_REF_SEL_BIT_POS);

    /* Enable Comparator. */
    pd->comp_ctrl[COMP_ID_LSCSA_OCP] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_OCP] &= ~PDSS_COMP_CTRL_COMP_PD;

    CyDelayUs(10);

    /*
     * By default OCP will work in software debounce mode. If end user want
     * us level debounce, then hardware filtering shall be enabled here.
     */

    /* Set edge detection. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
            PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK |
            PDSS_INTR5_FILTER_CFG_FILT_RESET |
            PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
            PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] |= (
            (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) |
            (16u << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS) |
            PDSS_INTR5_FILTER_CFG_FILT_EN);

    /* Clear interrupt. */
    pd->intr5 = (1 << FILTER_ID_LSCSA_OCP);

    /* Enable Interrupt. */
    pd->intr5_mask |= (1 << FILTER_ID_LSCSA_OCP);

    /* Check if Auto FET Control mode. */
    /* For PB application, P_CTRL and C_CTRL are flipped. Take care of this. */
    if (mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_enable (port, pctrl, FILTER_ID_LSCSA_OCP);
    }

    CyExitCriticalSection (state);
}

void pd_internal_vbus_ocp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection ();

    /* Disable and clear interrupt. */
    pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_OCP);
    pd->intr5 = (1 << FILTER_ID_LSCSA_OCP);

    /* Disable comparator. */
    pd->comp_ctrl[COMP_ID_LSCSA_OCP] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_OCP] |= PDSS_COMP_CTRL_COMP_PD;
    /* Clear reference. */
    pd->refgen_2_ctrl &= ~(PDSS_REFGEN_2_CTRL_SEL5_MASK);

    /* Disable filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_OCP] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_LSCSA_OCP);
    }

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_SW_DB)
    {
        /* Make sure any debounce timer is stopped. */
        timer_stop(port, PD_OCP_DEBOUNCE_TIMER);
    }

    CyExitCriticalSection (state);
}
#elif defined(CCG5)
void pd_internal_vbus_ocp_en(uint8_t port, uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        uint8_t mode, uint8_t debounce_ms)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t regval;

    /* Note: Assuming this function only gets called for P-CTRL and the mode is set to FW debounce. */

    state = CyEnterCriticalSection();

    gl_vbus_ocp_mode[port]      = mode;
    gl_ocp_sw_db_ms[port]       = debounce_ms;
    gl_vbus_ocp_pgdo_type[port] = pctrl;                /* OCP only gets called for P-CTRL. */

    regval = pd->csa_ctrl;

    /* Power up the CSA block. */
    regval &= ~PDSS_CSA_CTRL_PD_CSA;
    /* Default operational settings. */
    regval |= PDSS_CSA_CTRL_SEL_OUT_D | PDSS_CSA_CTRL_CSA_ISO_N;
    /* Clear the gain, bw and vref settings. */
    regval &= ~(PDSS_CSA_CTRL_AV1_MASK | PDSS_CSA_CTRL_CSA_VREF_SEL_MASK | PDSS_CSA_CTRL_BW_MASK);

    /* Set CSA gain. */
    regval |=  (av_bw >> 2) << PDSS_CSA_CTRL_AV1_POS;
    /* Set analog bandwidth. */
    regval |= ((av_bw & 0x03) << PDSS_CSA_CTRL_BW_POS);

    /* Set Vref trim select. */
    regval |= vref_sel << PDSS_CSA_CTRL_CSA_VREF_SEL_POS;

    /* Write out CSA_CTRL. */
    pd->csa_ctrl = regval;

    if (mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        /* OCP only gets called for P-CTRL. */
        regval = pd->pgdo_1_cfg[1];

        /* Enable auto mode. */
        regval |= PDSS_PGDO_1_CFG_AUTO_MODE;
        regval |= PDSS_PGDO_1_CFG_SEL_CSA_OC;

        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_1_CFG_PGDO_EN_LV_OFF_VALUE;
        pd->pgdo_1_cfg[1] = regval;
    }

    /* Bypassing filter is causing false OC triggers. Leaving filter enabled with a 5 us debounce period. */
    regval = pd->intr3_cfg_csa_oc_hs;
    regval &= ~(PDSS_INTR3_CFG_CSA_OC_HS_FILT_EN | PDSS_INTR3_CFG_CSA_OC_HS_FILT_CFG_MASK |
            PDSS_INTR3_CFG_CSA_OC_HS_FILT_SEL_MASK | PDSS_INTR3_CFG_CSA_OC_HS_FILT_RESET |
            PDSS_INTR3_CFG_CSA_OC_HS_FILT_BYPASS);

    /* Set edge detection. */
    regval |= (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR3_CFG_CSA_OC_HS_FILT_CFG_POS);
    regval |= (10 << PDSS_INTR3_CFG_CSA_OC_HS_FILT_SEL_POS) | PDSS_INTR3_CFG_CSA_OC_HS_FILT_EN |
        PDSS_INTR3_CFG_CSA_OC_HS_DPSLP_MODE;
    pd->intr3_cfg_csa_oc_hs = regval;

    /* Clear and enable the OC detect interrupt. */
    pd->intr3 = PDSS_INTR3_CSA_OC_CHANGED;
    pd->intr3_mask |= PDSS_INTR3_MASK_CSA_OC_CHANGED_MASK;

    CyExitCriticalSection(state);
}

void pd_internal_vbus_ocp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection();

    /* Default settings and power down. */
    pd->csa_ctrl = PDSS_CSA_CTRL_SEL_OUT_D | PDSS_CSA_CTRL_PD_CSA;

    /* Disable and clear interrupts. */
    pd->intr3_mask &= ~(PDSS_INTR3_CSA_OC_CHANGED);
    pd->intr3 = PDSS_INTR3_CSA_OC_CHANGED;

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        /* Remove source. */
        pd->pgdo_1_cfg[1] &= ~(PDSS_PGDO_1_CFG_SEL_CSA_OC);

        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_2_cfg[1] == 0) && ((pd->pgdo_1_cfg[1] & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_1_cfg[1] &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_SW_DB)
    {
        /* Make sure any debounce timer is stopped. */
        timer_stop(port, PD_OCP_DEBOUNCE_TIMER);
    }

    CyExitCriticalSection(state);
}
#elif (defined(CCG5C) || defined(CCG6))
void pd_internal_vbus_ocp_en(uint8_t port, uint8_t av_bw, uint8_t vref_sel, bool pctrl,
        uint8_t mode, uint8_t debounce_ms)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t regval;

    state = CyEnterCriticalSection();

    gl_vbus_ocp_mode[port]      = mode;
    gl_ocp_sw_db_ms[port]       = debounce_ms;
    gl_vbus_ocp_pgdo_type[port] = pctrl;                /* OCP only gets called for P-CTRL. */

    /* Set Vref trim select. */
#if (defined(CCG6))
    pd_set_refgen_voltage (vref_sel, CCG6_VREF_CSA_OCP);
#else
    pd->refgen_2_ctrl = (pd->refgen_2_ctrl & ~PDSS_REFGEN_2_CTRL_SEL5_MASK) | (vref_sel << PDSS_REFGEN_2_CTRL_SEL5_POS);
#endif /* (defined(CCG6)) */

    regval = pd->csa_scp_0_ctrl;

    /* Bring the CSA out of power-down, enable outputs and clear the gain setting. */
    regval &= ~(PDSS_CSA_SCP_0_CTRL_PD_CSA | PDSS_CSA_SCP_0_CTRL_AV1_MASK | PDSS_CSA_SCP_0_CTRL_BW_MASK);
    regval |= PDSS_CSA_SCP_0_CTRL_SEL_OUT_D | PDSS_CSA_SCP_0_CTRL_CSA_ISO_N;

    /* Set the gain and bandwidth based on requirement. */
    regval |= (((av_bw >> 2) & 0x07) << PDSS_CSA_SCP_0_CTRL_AV1_POS);
    regval |= ((av_bw & 0x03) << PDSS_CSA_SCP_0_CTRL_BW_POS);

    pd->csa_scp_0_ctrl = regval;

    if (mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        /* CCG6 uses PGDO_PU_1_CFG register for Provider FET */
        regval = pd->pgdo_pu_1_cfg;

        /* Enable auto mode. */
        regval |= PDSS_PGDO_PU_1_CFG_AUTO_MODE;
        regval |= PDSS_PGDO_PU_1_CFG_SEL_CSA_OC;

        /* Program the off value to turn off the PFET. */
        regval &= ~PDSS_PGDO_PU_1_CFG_PGDO_PU_EN_LV_OFF_VALUE;
        pd->pgdo_pu_1_cfg = regval;
    }

    /* Bypassing filter is causing false OC triggers. Leaving filter enabled with a 5 us debounce period. */
    regval = pd->intr13_cfg_csa_scp_hs;
    regval &= ~(PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_EN | PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_CFG_MASK |
            PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_SEL_MASK | PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_RESET |
            PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_BYPASS);

    /* Set edge detection. */
    regval |= (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_CFG_POS);
    regval |= (10 << PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_SEL_POS) | PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_FILT_EN |
        PDSS_INTR13_CFG_CSA_SCP_HS_CSA_OCP_DPSLP_MODE;
    pd->intr13_cfg_csa_scp_hs = regval;

    /* Clear and enable the OC detect interrupt. */
    pd->intr13 = PDSS_INTR13_CSA_OCP_CHANGED;
    pd->intr13_mask |= PDSS_INTR13_MASK_CSA_OCP_CHANGED_MASK;

    CyExitCriticalSection(state);
}

void pd_internal_vbus_ocp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection();

    /* Default settings and power down. */
    pd->csa_scp_0_ctrl = (pd->csa_scp_0_ctrl & ~PDSS_CSA_SCP_0_CTRL_CSA_ISO_N) | PDSS_CSA_SCP_0_CTRL_PD_CSA;

    /* Disable and clear interrupts. */
    pd->intr13_mask &= ~(PDSS_INTR13_CSA_OCP_CHANGED);
    pd->intr13 = PDSS_INTR13_CSA_OCP_CHANGED;

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        /* Remove source. */
        pd->pgdo_pu_1_cfg &= ~(PDSS_PGDO_PU_1_CFG_SEL_CSA_OC);

        /* Disable Auto mode if no other source is active. */
        if ((pd->pgdo_pu_2_cfg == 0) && ((pd->pgdo_pu_1_cfg & PGDO_1_CFG_AUTO_SEL_MASK) == 0))
        {
            pd->pgdo_pu_1_cfg &= ~PDSS_PGDO_1_CFG_AUTO_MODE;
        }
    }

    if (gl_vbus_ocp_mode[port] == VBUS_OCP_MODE_INT_SW_DB)
    {
        /* Make sure any debounce timer is stopped. */
        timer_stop(port, PD_OCP_DEBOUNCE_TIMER);
    }

    CyExitCriticalSection(state);
}
#endif /* CCGx */

#endif /* VBUS_OCP_ENABLE */

#if VBUS_SCP_ENABLE
#if (defined(CCG3PA) || defined(CCG3PA2))

/* SCP Comparator min reference voltage in mV. */
#define SCP_REF_VOLT_MIN            (130)

/* SCP Reference voltage step size in mV. */
#define SCP_REF_VOLT_STEP           (10)

/* SCP Gain. */
#define SCP_GAIN_VALUE              (10)

void pd_internal_vbus_scp_en(uint8_t port, uint32_t vsense, uint8_t filter_sel, bool pctrl,
    uint8_t mode)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t regval;
    uint32_t vref;

    state = CyEnterCriticalSection ();

    /* Store SCP mode. */
    gl_vbus_scp_mode = mode;
    /* Store PGDO type. */
    gl_vbus_scp_pgdo_type = pctrl;

    if (mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_LSCSA_SCP);
    }

    /* Gain for SCP is fixed to 10. So LSCSA programming is not required.
     * Just power up LSCSA block. */
    pd->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /*
     * We have vsense and gain = 10. So calculate vref based on that.
     * Min VREF is 130mV and each step is 10mv.
     */
    vref = vsense * SCP_GAIN_VALUE;
    if (vref < SCP_REF_VOLT_MIN)
    {
        vref = SCP_REF_VOLT_MIN;
    }
    vref = (vref - SCP_REF_VOLT_MIN)/SCP_REF_VOLT_STEP;

    /* Configure Reference for comparator. */
    regval = pd->refgen_2_ctrl;
    regval &= ~(PDSS_REFGEN_2_CTRL_SEL4_MASK);
    regval |= (vref << PDSS_REFGEN_2_CTRL_SEL4_POS);
    pd->refgen_2_ctrl = regval;

    /* Use Vref SEL 4 for SCP comparator. */
    pd->amux_ctrl |= (1 << VBUS_SCP_AMUX_CTRL_REF_SEL_BIT_POS);

    /* Enable Comparator. */
    pd->comp_ctrl[COMP_ID_LSCSA_SCP] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_SCP] &= ~PDSS_COMP_CTRL_COMP_PD;

    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SCP] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
            PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
            PDSS_INTR5_FILTER_CFG_FILT_RESET | PDSS_INTR5_FILTER_CFG_FILT_BYPASS);
    /* Set edge detection. */
    regval = pd->intr5_filter_cfg[FILTER_ID_LSCSA_SCP];
    regval |= (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS);

    if (filter_sel)
    {
        /* Subtracting 1 from filter clock cycle value as 0 translates to 1-2 clock cycles. */
        regval |= (((filter_sel - 1) & 0x1F) << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS);
        regval |= (PDSS_INTR5_FILTER_CFG_FILT_EN | PDSS_INTR5_FILTER_CFG_DPSLP_MODE);
    }
    else
    {
        regval |= PDSS_INTR5_FILTER_CFG_FILT_BYPASS;
    }

    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SCP] = regval;

    /* Clear interrupt. */
    pd->intr5 = (1 << FILTER_ID_LSCSA_SCP);

    /* Enable Interrupt. */
    pd->intr5_mask |= (1 << FILTER_ID_LSCSA_SCP);

    /* Auto FET turn off mode. */
    if (gl_vbus_scp_mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_enable (port, pctrl, FILTER_ID_LSCSA_SCP);
    }

    CyExitCriticalSection (state);
}

void pd_internal_vbus_scp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection ();

    /* Disable and clear interrupt. */
    pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_SCP);
    pd->intr5 = (1 << FILTER_ID_LSCSA_SCP);

    /* Disable comparator. */
    pd->comp_ctrl[COMP_ID_LSCSA_SCP] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_SCP] |= PDSS_COMP_CTRL_COMP_PD;

    /* Clear reference. */
    pd->refgen_2_ctrl &= ~(PDSS_REFGEN_2_CTRL_SEL4_MASK);

    /* Disable filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SCP] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Remove SCP as source. */
    if (gl_vbus_scp_mode == VBUS_OCP_MODE_INT_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_LSCSA_SCP);
    }

    CyExitCriticalSection (state);
}
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
#endif /* VBUS_SCP_ENABLE */

#if VBUS_OVP_ENABLE

/* OVP min reference voltage in mV. */
#define OVP_REF_VOLT_MIN            (200u)

/* Minimum voltage for VREF8 in mV. */
#define OVP_REF8_VOLT_MIN           (130u)

/* OVP reference voltage step size in mV. */
#define OVP_REF_VOLT_STEP           (10u)

/* Min OVP detection level for VBus. */
#define VBUS_OVP_DETECT_MIN         (6000u)

/* VBUS max voltage in mV. */
#define VBUS_VOLT_MAX               (24000u)

/* Max. VREF setting. */
#define VREF_MAX_SETTING            (199u)

void pd_internal_vbus_ovp_en(uint8_t port, uint16_t volt, int8_t per, PD_ADC_CB_T cb, bool pctrl,
        vbus_ovp_mode_t mode, uint8_t filter_sel)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint16_t threshold, vref;
    uint32_t regval;
    uint32_t comp_id      = COMP_ID_OV;
    filter_id_t filter_id = FILTER_ID_OV;
    uint32_t div_pos      = AMUX_OV_DIV_SEL_BIT_POS;

#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
    if (port != 0)
    {
        div_pos = CCG5_P1_OV_DIV_SEL_BIT_POS;
    }
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */

    /* Clear AUTO MODE OVP detect to avoid false auto off during reference change */
    if (gl_vbus_ovp_mode[port] == VBUS_OVP_MODE_UVOV_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, filter_id);
    }

    /* Store OVP parameters. */
    gl_vbus_ovp_mode[port]      = mode;
    gl_ovp_cb[port]             = cb;
    gl_vbus_ovp_pgdo_type[port] = pctrl;
    gl_vbus_ovp_filter_id[port] = filter_id;

    /* Calculate required VBUS for OVP. */
    threshold = volt + ((volt * per) / 100);

    /* Cap the maximum voltage to be measured. */
    if (threshold > VBUS_VOLT_MAX)
        threshold = VBUS_VOLT_MAX;
    /* Make sure threshold is above the minimum trip point to avoid false triggers. */
    if (threshold < VBUS_OVP_DETECT_MIN)
        threshold = VBUS_OVP_DETECT_MIN;

    regval = pd->amux_nhv_ctrl;
#if (defined(CCG3PA) || defined(CCG3PA2))
#if !CCG_FLIPPED_FET_CTRL
    /* Choose VBUS_C as source for AMUX. */
     regval &= ~(AMUX_OV_VBUS_SEL_MASK);
#else
    /* As TYPE C VBUS is connected to VBUS_IN node, choose VBUS_IN as source. */
    regval |= AMUX_OV_VBUS_SEL_MASK;
#endif /* CCG_FLIPPED_FET_CTRL */
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

    /*
     * Internal divider to be used depends on VBUS to be measured. For VBUS <
     * VBUS_DIV_20_PER_MAX_VOLT, use 20% divider(CCG3PA) and for greater,
     * use 8% divider. For CCG5, always use 8% divider.
     */

#if (defined(CCG3PA) || defined(CCG3PA2))
    if (threshold <= VBUS_DIV_20_PER_MAX_VOLT)
    {
        vref = threshold / VBUS_C_20_PER_DIV;

        /* Connect output to 20% */
        regval |= (1 << div_pos);
    }
    else
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
    {
#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
        if (port == 1)
        {
            /* Get 8% of threshold. */
            vref = (threshold * 10) / 125;
        }
        else
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */
        {
            /* Get 8% of threshold. */
            vref = (threshold * 10) / 125;
        }

        /* Connect output to 8% */
        regval &= ~(1 << div_pos);
    }

    pd->amux_nhv_ctrl = regval;

    /* Calculate the actual reference voltage. Cap the value to the max. supported. */
    vref = ((vref - OVP_REF_VOLT_MIN) / OVP_REF_VOLT_STEP);
    if (vref > VREF_MAX_SETTING)
        vref = VREF_MAX_SETTING;

    /* Program reference voltage for OV comparator. */
#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
    if (port == 1)
    {
        vref += ((OVP_REF_VOLT_MIN - OVP_REF8_VOLT_MIN) / OVP_REF_VOLT_STEP);
        if (vref > VREF_MAX_SETTING)
            vref = VREF_MAX_SETTING;

        /* OV comparator for Port 1 uses VREF[8]. */
        regval = gl_pdss[0]->refgen_3_ctrl;
        regval = (regval & ~PDSS_REFGEN_3_CTRL_SEL8_MASK) | (vref << PDSS_REFGEN_3_CTRL_SEL8_POS);
        gl_pdss[0]->refgen_3_ctrl = regval;
    }
    else
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */
    {
#if (defined(CCG6))
        pd_set_refgen_voltage (vref, CCG6_VREF_VBUS_OV);
#else
        /* OV comparator for Port 0 uses VREF[3]. */
        regval = pd->refgen_1_ctrl;
        regval &= ~(PDSS_REFGEN_1_CTRL_SEL3_MASK);
        regval |= (vref << PDSS_REFGEN_1_CTRL_SEL3_POS);
        pd->refgen_1_ctrl = regval;
#endif /* (defined(CCG6)) */
    }

    /* Turn on comparator. */
    pd->comp_ctrl[comp_id] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[comp_id] &= ~PDSS_COMP_CTRL_COMP_PD;

    CyDelayUs(10);

    /* Filter configuration. */
    pd->intr5_filter_cfg[filter_id] &= ~PDSS_INTR5_FILTER_CFG_FILT_EN;
    regval = pd->intr5_filter_cfg[filter_id] & ~(PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK
            | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK | PDSS_INTR5_FILTER_CFG_DPSLP_MODE |
            PDSS_INTR5_FILTER_CFG_FILT_BYPASS | PDSS_INTR5_FILTER_CFG_FILT_RESET);

    /* Set filter clock cycles if filter is required. */
    if (filter_sel)
    {
        /* Subtracting 1 from filter clock cycle value as 0 translates to 1-2 clock cycles. */
        regval |= (((filter_sel - 1) & 0x1F) << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS);

        /* Set edge detection. */
        regval |= (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS);
        regval |= PDSS_INTR5_FILTER_CFG_FILT_EN | PDSS_INTR5_FILTER_CFG_DPSLP_MODE;
    }
    else
    {
        regval |= PDSS_INTR5_FILTER_CFG_FILT_BYPASS;
    }

    pd->intr5_filter_cfg[filter_id] = regval;

    /* Clear interrupt. */
    pd->intr5 = (1 << filter_id);

    /* Enable Interrupt. */
    pd->intr5_mask |= (1 << filter_id);

    /* Handle Auto mode. */
    /* For PB application, P_CTRL and C_CTRL are flipped. Take care of this. */
    if (gl_vbus_ovp_mode[port] == VBUS_OVP_MODE_UVOV_AUTOCTRL)
    {
        pd_fet_automode_enable (port, pctrl, filter_id);
    }
}

void pd_internal_vbus_ovp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint16_t vref;
    uint32_t comp_id      = COMP_ID_OV;
    filter_id_t filter_id = FILTER_ID_OV;
    uint32_t div_pos      = AMUX_OV_DIV_SEL_BIT_POS;

#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
    if (port == 1)
    {
        div_pos   = CCG5_P1_OV_DIV_SEL_BIT_POS;
    }
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */

    state = CyEnterCriticalSection();

    /* Clear AUTO MODE OVP detect. */
    if (gl_vbus_ovp_mode[port] == VBUS_OVP_MODE_UVOV_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, filter_id);
    }

    /* Disable comparator. */
    pd->comp_ctrl[comp_id] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[comp_id] |= PDSS_COMP_CTRL_COMP_PD;

     /* Disable filter. */
    pd->intr5_filter_cfg[filter_id] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS | PDSS_INTR5_FILTER_CFG_DPSLP_MODE);

    /* Disable and clear OV interrupts. */
    pd->intr5_mask &= ~(1 << filter_id);
    pd->intr5 = 1 << filter_id;

    /* Connect OV comparator input to 20% / 10% of VBus. */
    pd->amux_nhv_ctrl |= (1 << div_pos);

    /* Restore reference voltage for OV comparator to 6V equivalent. */
#if (defined(CCG3PA) || defined(CCG3PA2))
    vref = VBUS_OVP_DETECT_MIN / VBUS_C_20_PER_DIV;
#else /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */
    vref = VBUS_OVP_DETECT_MIN / VBUS_C_10_PER_DIV;
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */
    vref = (vref - OVP_REF_VOLT_MIN) / OVP_REF_VOLT_STEP;

#if ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE))
    if (port == 1)
    {
        /* OV comparator on port 1 uses VREF[8]. */
        gl_pdss[0]->refgen_3_ctrl = (gl_pdss[0]->refgen_3_ctrl & ~PDSS_REFGEN_3_CTRL_SEL8_MASK) |
            (vref << PDSS_REFGEN_3_CTRL_SEL8_POS);
    }
    else
#endif /* ((defined(CCG5)) && (CCG_PD_DUALPORT_ENABLE)) */
    {
        /* OV comparator on port 0 uses VREF[3]. */
        pd->refgen_1_ctrl = (pd->refgen_1_ctrl & ~PDSS_REFGEN_1_CTRL_SEL3_MASK) | (vref << PDSS_REFGEN_1_CTRL_SEL3_POS);
    }

    /* Clear callback. */
    gl_ovp_cb[port] = NULL;

    CyExitCriticalSection(state);
}

#endif /* VBUS_OVP_ENABLE */

#if VBUS_UVP_ENABLE

/* UVP min reference voltage in mV. */
#define UVP_REF_VOLT_MIN            (200)

/* UVP min reference voltage on Port #1 of CCG5. */
#define UVP_REF_VOLT_MIN_P1         (130)

/* UVP Reference voltage step size in mV. */
#define UVP_REF_VOLT_STEP           (10)

/*
 * Minimum supported voltage for UVP. Any voltage lower may cause system to
 * not work as expected; the block references can get affected. This is now
 * limited to 3.15V.
 */
#define UVP_MIN_VOLT                (3100)

void pd_internal_vbus_uvp_en(uint8_t port, uint16_t volt, int8_t per, PD_ADC_CB_T cb, bool pctrl,
        vbus_uvp_mode_t mode, uint8_t filter_sel)
{
    uint16_t threshold, vref;
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    if (gl_vbus_uvp_mode == VBUS_UVP_MODE_INT_COMP_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_UV);
    }

    /* Store UVP mode. */
    gl_vbus_uvp_mode = mode;
    /* Set up UVP callback. */
    gl_uvp_cb = cb;
    /* Store PGDO type. */
    gl_vbus_uvp_pgdo_type = pctrl;

    /* Calculate required VBUS for UVP. */
    threshold = ((volt * per) / 100);

    /* Ensure that we are within the limits. */
    if (threshold < UVP_MIN_VOLT)
    {
        threshold = UVP_MIN_VOLT;
    }

    regval = pd->amux_nhv_ctrl;

#if (defined(CCG3PA) || defined(CCG3PA2))

#if !CCG_FLIPPED_FET_CTRL
    /* Choose VBUS_C as source for AMUX. */
    regval &= ~(AMUX_UV_VBUS_SEL_MASK);
#else
    /* Choose VBUS_IN as source for AMUX. */
    regval |= AMUX_UV_VBUS_SEL_MASK;
#endif /* CCG_FLIPPED_FET_CTRL */

    /*
     * Internal divider to be used depends on VBUS to be measured. For VBUS <
     * VBUS_DIV_20_PER_MAX_VOLT, use 20% divider and for greater, use 10% divider.
     */
    if (threshold <= VBUS_DIV_20_PER_MAX_VOLT)
    {
        vref = threshold / VBUS_C_20_PER_DIV;
        /* Connect output to 20% */
        regval |= (1 << AMUX_UV_DIV_SEL_BIT_POS);
    }
    else
    {
        vref = threshold / VBUS_C_10_PER_DIV;
        /* Connect output to 10% */
        regval &= ~(1 << AMUX_UV_DIV_SEL_BIT_POS);
    }
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

#if (defined(CCG5) || defined(CCG5C) || defined(CCG6))
    /* Use 10% of VBus as input to UVP comparator. */
    regval &= 0xFFFFFFFD;

    vref = threshold / VBUS_C_10_PER_DIV;
#endif /* (defined(CCG5) || defined(CCG5C) || defined(CCG6)) */

    pd->amux_nhv_ctrl = regval;

#if CCG_PD_DUALPORT_ENABLE
    if (port != 0)
    {
        /* UV comparator on Port #1 uses VREF_OUT[7]. */
        vref = ((vref - UVP_REF_VOLT_MIN_P1) / UVP_REF_VOLT_STEP);
        regval = pd->refgen_2_ctrl;
        regval &= ~(PDSS_REFGEN_2_CTRL_SEL7_MASK);
        regval |= (vref << PDSS_REFGEN_2_CTRL_SEL7_POS);
        pd->refgen_2_ctrl = regval;
    }
    else
#endif /* CCG_PD_DUALPORT_ENABLE */
    {
        /* Program reference voltage for UV comparator. */
        vref = ((vref - UVP_REF_VOLT_MIN) / UVP_REF_VOLT_STEP);
        regval = pd->refgen_1_ctrl;
        regval &= ~(PDSS_REFGEN_1_CTRL_SEL2_MASK);
        regval |= (vref << PDSS_REFGEN_1_CTRL_SEL2_POS);
        pd->refgen_1_ctrl = regval;
    }

    /* Turn on comparator. */
    pd->comp_ctrl[COMP_ID_UV] |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_UV] &= ~PDSS_COMP_CTRL_COMP_PD;

    CyDelayUs(10);

    /* Filter configuration. */
    pd->intr5_filter_cfg[FILTER_ID_UV] &= ~PDSS_INTR5_FILTER_CFG_FILT_EN;
    /* Reset filter to 1. */
    regval = pd->intr5_filter_cfg[FILTER_ID_UV] & ~(PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK
            | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK | PDSS_INTR5_FILTER_CFG_DPSLP_MODE |
            PDSS_INTR5_FILTER_CFG_FILT_BYPASS);
    regval |= PDSS_INTR5_FILTER_CFG_FILT_RESET;

    /* Set filter clock cycles if filter is required. */
    if (filter_sel)
    {
        /* Subtracting 1 from filter clock cycle value as 0 translates to 1-2 clock cycles. */
        regval |= (((filter_sel - 1) & 0x1F) << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS);
        /* Set edge detection. */
        regval |= (FILTER_CFG_POS_DIS_NEG_EN << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS);
        regval |= PDSS_INTR5_FILTER_CFG_FILT_EN | PDSS_INTR5_FILTER_CFG_DPSLP_MODE;
    }
    else
    {
        regval |= PDSS_INTR5_FILTER_CFG_FILT_BYPASS;
    }

    pd->intr5_filter_cfg[FILTER_ID_UV] =  regval;

    /* Clear interrupt. */
    pd->intr5 = (1 << FILTER_ID_UV);

    /* Enable Interrupt. */
    pd->intr5_mask |= (1 << FILTER_ID_UV);

    /* Handle Auto mode. */
    if (gl_vbus_uvp_mode == VBUS_UVP_MODE_INT_COMP_AUTOCTRL)
    {
        pd_fet_automode_enable (port, pctrl, FILTER_ID_UV);
    }
}

void pd_internal_vbus_uvp_dis(uint8_t port, bool pctrl)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection ();

    /* Clear AUTO MODE OVP detect. */
    if (gl_vbus_uvp_mode == VBUS_UVP_MODE_INT_COMP_AUTOCTRL)
    {
        pd_fet_automode_disable (port, pctrl, FILTER_ID_UV);
    }

    /* Disable comparator. */
    pd->comp_ctrl[COMP_ID_UV] &= ~PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_UV] |= PDSS_COMP_CTRL_COMP_PD;

    /* Disable filter. */
    pd->intr5_filter_cfg[FILTER_ID_UV] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Disable and clear UV interrupts. */
    pd->intr5_mask &= ~(1 << FILTER_ID_UV);
    pd->intr5 = 1 << FILTER_ID_UV;

    /* Clear callback. */
    gl_uvp_cb = NULL;

    CyExitCriticalSection (state);
}

#endif /* VBUS_UVP_ENABLE */

void pd_cf_enable(uint8_t port, uint32_t cur, vbus_cf_cbk_t cbk)
{
#if VBUS_CF_EN
    uint32_t regval;
    uint8_t state, gain_sel, vref_sel;
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t *pdss_stat = &gl_pdss_status[port];

    state = CyEnterCriticalSection();

    /* Retaining the callback for event based entry / exit implementation. */
    pdss_stat->cf_cbk = cbk;
    pdss_stat->cf_cur = cur;

    /* Calculate the gain and reference settings based on the current. */
    pd_lscsa_calc_cfg(cur, &gain_sel, &vref_sel);

    /* Configure the EA comparator for the current limit mode operation. */
    regval = pd->refgen_3_ctrl;
    regval &= ~(PDSS_REFGEN_3_CTRL_SEL10_MASK);
    regval |= (vref_sel << PDSS_REFGEN_3_CTRL_SEL10_POS);
    pd_lscsa_cfg(LSCSA_EA_CONFIG, gain_sel);
    pd->refgen_3_ctrl = regval;

    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /* Enable CC mode hardware control. */
    pd->ea_ctrl |= PDSS_EA_CTRL_EN_CC;

    CyExitCriticalSection(state);
#else /* !VBUS_CF_EN */
    (void)port;
    (void)cur;
    (void)cbk;
#endif /* VBUS_CF_EN */
}

void pd_cf_disable(uint8_t port)
{
#if VBUS_CF_EN
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection();

    /* Disable CC mode hardware control. */
    pd->ea_ctrl &= ~PDSS_EA_CTRL_EN_CC;
    /* Select LSCSA gain to initialized value */
    PDSS->lscsa_0_ctrl |= (LSCSA_AV_SEL_35 << PDSS_LSCSA_0_CTRL_AV_EA_POS);

    CyExitCriticalSection(state);
#else /* !VBUS_CF_EN */
    (void)port;
#endif /* VBUS_CF_EN */
}

bool pd_cf_get_status(uint8_t port)
{
#if VBUS_CF_EN
    uint32_t cur;
    pdss_status_t *pdss_stat = &gl_pdss_status[port];

    if (PDSS->ea_ctrl & PDSS_EA_CTRL_EN_CC)
    {
        cur = pdss_stat->cf_cur;

        if (pdss_stat->cf_cur > LSCSA_CF_CUR_HIGH_LIMIT)
        {
            cur -= LSCSA_CF_CUR_HIGH_THRES;
        }
        else
        {
            cur -= LSCSA_CF_CUR_THRES;
        }
        
        return pd_sample_pfc_comp(port, cur);
    }
    else
#endif /* !VBUS_CF_EN */
    {
        (void)port;
        return false;
    }
}

#if VCONN_OCP_ENABLE
void pd_hal_vconn_ocp_enable(uint8_t port, uint8_t debounce, PD_ADC_CB_T cb)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;
    uint32_t regval, ccint;

    state = CyEnterCriticalSection();

    /* Store the parameters. */
    gl_vconn_ocp_debounce[port] = debounce;
    gl_vconn_ocp_cb[port]       = cb;

    /* Configure the filter on the appropriate CC line. */
    pd->intr1_cfg_cc12_ocp_hs = 0;
    regval = 0;

    if (dpm_get_status(port)->polarity == CC_CHANNEL_1)
    {
        /* Enable filter with positive edge detection with 4 HF clock cycle period. */
        regval = (
                (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_FILT_CFG_POS) |
                (4 << PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_FILT_SEL_POS) |
                PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_DPSLP_MODE |
                PDSS_INTR1_CFG_CC12_OCP_HS_CC2_OCP_FILT_EN
                );
        gl_vconn_channel[port] = CC_CHANNEL_2;
        ccint = PDSS_INTR1_CC2_OCP_CHANGED;

        /* Enable OC detection on CC2 pin. */
        pd->vconn20_ctrl |= PDSS_VCONN20_CTRL_EN_OCP_CC2;
    }
    else
    {
        regval = (
                (FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_FILT_CFG_POS) |
                (4 << PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_FILT_SEL_POS) |
                PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_DPSLP_MODE |
                PDSS_INTR1_CFG_CC12_OCP_HS_CC1_OCP_FILT_EN
                );
        gl_vconn_channel[port] = CC_CHANNEL_1;
        ccint = PDSS_INTR1_CC1_OCP_CHANGED;

        /* Enable OC detection on CC1 pin. */
        pd->vconn20_ctrl |= PDSS_VCONN20_CTRL_EN_OCP_CC1;
    }

    /* Set the VConn OCP configuration. */
    pd->intr1_cfg_cc12_ocp_hs = regval;

    /* Make sure VConn OCP debounce timer is not running. */
    timer_stop (port, PD_VCONN_OCP_DEBOUNCE_TIMER);

    /* Clear and enable the interrupt associated with OC detection on CC lines. */
    pd->intr1 = (PDSS_INTR1_CC1_OCP_CHANGED | PDSS_INTR1_CC2_OCP_CHANGED);

    /* Enable the interrupt. */
    pd->intr1_mask |= ccint;

    CyExitCriticalSection (state);
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Avoid compiler warnings about unused arguments. */
    (void)port;
    (void)debounce;
    (void)cb;
}

void pd_hal_vconn_ocp_disable(uint8_t port)
{
#if (defined (CCG5) || defined(CCG6) || defined(CCG5C))
    PPDSS_REGS_T pd = gl_pdss[port];

    /* Disable OC detection on CC pins. */
    pd->vconn20_ctrl &= ~(PDSS_VCONN20_CTRL_EN_OCP_CC1 | PDSS_VCONN20_CTRL_EN_OCP_CC2);

    /* Disable the filter block associated with CC12 OC detection. */
    pd->intr1_cfg_cc12_ocp_hs = 0;

    /* Disable and clear the CC12 OC changed interrupt. */
    pd->intr1_mask &= ~(PDSS_INTR1_CC1_OCP_CHANGED | PDSS_INTR1_CC2_OCP_CHANGED);
    pd->intr1       = (PDSS_INTR1_CC1_OCP_CHANGED | PDSS_INTR1_CC2_OCP_CHANGED);

    /* Stop the CC12 OC debounce timer if it is running. */
    timer_stop (port, PD_VCONN_OCP_DEBOUNCE_TIMER);

    /* Clear the stored configuration. */
    gl_vconn_ocp_debounce[port] = 0;
    gl_vconn_ocp_cb[port]       = NULL;
    gl_vconn_channel[port]      = CC_CHANNEL_1;
#endif /* defined (CCG5) || defined(CCG6) || defined(CCG5C) */

    /* Avoid compiler warnings about unused arguments. */
    (void)port;
}
#endif /* VCONN_OCP_ENABLE */

#if (defined(CCG3PA) || defined(CCG3PA2))
bool pd_set_sr_comp(uint8_t port, uint32_t cur,
    filter_edge_detect_cfg_t edge, pd_cmp_cbk_t cbk)
{
    uint32_t regval;
    uint8_t state, gain_sel, vref_sel;
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    state = CyEnterCriticalSection();

    pdss_stat->sr_cmp_cbk = cbk;

    /* Configure the comparator to always use refgen sel8 */
    pd->refgen_sel6_sel8_ctrl |= PDSS_REFGEN_SEL6_SEL8_CTRL_SR_SEL8;

    /* Calculate the gain and reference settings based on the current. */
    pd_lscsa_calc_cfg(cur, &gain_sel, &vref_sel);
    regval = pd->refgen_3_ctrl;
    regval &= ~(PDSS_REFGEN_3_CTRL_SEL8_MASK);
    regval |= (vref_sel << PDSS_REFGEN_3_CTRL_SEL8_POS);
    /* Configure Reference for comparator. */
    pd_lscsa_cfg(LSCSA_SR_OFF_CONFIG, gain_sel);
    pd->refgen_3_ctrl = regval;
    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /* Enable Comparator. */
    regval = pd->comp_tr_ctrl & ~PDSS_COMP_CTRL_COMP_PD;
    regval |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_tr_ctrl = regval;

    /* Set up the filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SR] &= ~(
        PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_RESET |
        PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Set edge detection. */
    regval = ((edge << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) |
            (31u << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS) |
            PDSS_INTR5_FILTER_CFG_FILT_EN);
    if (edge == FILTER_CFG_POS_DIS_NEG_EN)
    {
        regval |= PDSS_INTR5_FILTER_CFG_FILT_RESET;
    }
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SR] |= regval;
    CyDelayUs(10);

    /* Enable the interrupt only if the callback is registered. */
    if (cbk)
    {
        /* Clear interrupt. */
        pd->intr5 = (1 << FILTER_ID_LSCSA_SR);
        /* Enable interrupt. */
        pd->intr5_mask |= (1 << FILTER_ID_LSCSA_SR);
    }
    else
    {
        /* Disable interrupt. */
        pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_SR);
        /* Clear interrupt. */
        pd->intr5 = (1 << FILTER_ID_LSCSA_SR);
    }

    CyExitCriticalSection (state);

    return ((pd->intr5_status_0 >> FILTER_ID_LSCSA_SR) & 0x01);
}

void pd_stop_sr_comp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection();

    /* Disable comparator and reference. */
    pd->comp_tr_ctrl = PDSS_COMP_CTRL_COMP_PD;
    pd->refgen_3_ctrl &= ~(PDSS_REFGEN_3_CTRL_SEL8_POS);

    /* Disable filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_SR] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Disable interrupt. */
    pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_SR);
    pd->intr5 = 1 << FILTER_ID_LSCSA_SR;

    CyExitCriticalSection (state);
}

bool pd_set_pfc_comp(uint8_t port, uint32_t cur,
    filter_edge_detect_cfg_t edge, pd_cmp_cbk_t cbk)
{
    uint32_t regval;
    uint8_t state, gain_sel, vref_sel;
    PPDSS_REGS_T pd = gl_pdss[port];
    pdss_status_t* pdss_stat = &gl_pdss_status[port];

    state = CyEnterCriticalSection();

    pdss_stat->pfc_cmp_cbk = cbk;

    /* Configure the comparator to always use refgen sel6 */
    pd->refgen_sel6_sel8_ctrl |= PDSS_REFGEN_SEL6_SEL8_CTRL_PFC_SEL6;

    /* Calculate the gain and reference settings based on the current. */
    pd_lscsa_calc_cfg(cur, &gain_sel, &vref_sel);
    regval = pd->refgen_2_ctrl;
    regval &= ~(PDSS_REFGEN_2_CTRL_SEL6_MASK);
    regval |= (vref_sel << PDSS_REFGEN_2_CTRL_SEL6_POS);
    /* Configure Reference for comparator. */
    pd_lscsa_cfg(LSCSA_PFC_OFF_CONFIG, gain_sel);
    pd->refgen_2_ctrl = regval;
    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /* Enable Comparator. */
    regval = pd->comp_ctrl[COMP_ID_LSCSA_PFC] & ~PDSS_COMP_CTRL_COMP_PD;
    regval |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_PFC] = regval;

    /* Setup the filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] &= ~(
        PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_RESET |
        PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Set edge detection. */
    regval = ((edge << PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) |
            (31u << PDSS_INTR5_FILTER_CFG_FILT_SEL_POS) |
            PDSS_INTR5_FILTER_CFG_FILT_EN);
    if (edge == FILTER_CFG_POS_DIS_NEG_EN)
    {
        regval |= PDSS_INTR5_FILTER_CFG_FILT_RESET;
    }
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] |= regval;
    CyDelayUs(10);

    /* Enable the interrupt only if the callback is registered. */
    if (cbk)
    {
        /* Clear interrupt. */
        pd->intr5 = (1 << FILTER_ID_LSCSA_PFC);
        /* Enable interrupt. */
        pd->intr5_mask |= (1 << FILTER_ID_LSCSA_PFC);
    }
    else
    {
        /* Disable interrupt. */
        pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_PFC);
        /* Clear interrupt. */
        pd->intr5 = (1 << FILTER_ID_LSCSA_PFC);
    }

    CyExitCriticalSection(state);

    return ((pd->intr5_status_0 >> FILTER_ID_LSCSA_PFC) & 0x01);
}

void pd_stop_pfc_comp(uint8_t port)
{
    PPDSS_REGS_T pd = gl_pdss[port];
    uint8_t state;

    state = CyEnterCriticalSection();

    /* Disable comparator and reference. */
    pd->comp_ctrl[COMP_ID_LSCSA_PFC] = PDSS_COMP_CTRL_COMP_PD;
    pd->refgen_2_ctrl &= ~(PDSS_REFGEN_2_CTRL_SEL6_POS);

    /* Disable filter. */
    pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] &= ~(PDSS_INTR5_FILTER_CFG_FILT_EN |
        PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK | PDSS_INTR5_FILTER_CFG_FILT_SEL_MASK |
        PDSS_INTR5_FILTER_CFG_FILT_BYPASS);

    /* Disable interrupt. */
    pd->intr5_mask &= ~(1 << FILTER_ID_LSCSA_PFC);
    pd->intr5 = 1 << FILTER_ID_LSCSA_PFC;

    CyExitCriticalSection (state);
}

bool pd_sample_pfc_comp(uint8_t port, uint32_t cur)
{
    uint8_t state, gain_sel, vref_sel;
    bool retval = false;
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];
    uint32_t reg_lscsa_0_ctrl, reg_refgen_2_ctrl, reg_comp_ctrl;

    state = CyEnterCriticalSection();

    /* Configure the comparator to always use refgen sel6 */
    pd->refgen_sel6_sel8_ctrl |= PDSS_REFGEN_SEL6_SEL8_CTRL_PFC_SEL6;

    /* Store the register content before modifying. */
    reg_lscsa_0_ctrl = pd->lscsa_0_ctrl;
    reg_refgen_2_ctrl = pd->refgen_2_ctrl;
    reg_comp_ctrl = pd->comp_ctrl[COMP_ID_LSCSA_PFC];

    /* Calculate the gain and reference settings based on the current. */
    pd_lscsa_calc_cfg(cur, &gain_sel, &vref_sel);
    regval = pd->refgen_2_ctrl;
    regval &= ~(PDSS_REFGEN_2_CTRL_SEL6_MASK);
    regval |= (vref_sel << PDSS_REFGEN_2_CTRL_SEL6_POS);
    /* Configure Reference for comparator. */
    pd_lscsa_cfg(LSCSA_PFC_OFF_CONFIG, gain_sel);
    pd->refgen_2_ctrl = regval;
    PDSS->lscsa_1_ctrl &= ~PDSS_LSCSA_1_CTRL_LSCSA_PD;

    /* Enable Comparator. */
    regval = pd->comp_ctrl[COMP_ID_LSCSA_PFC] & ~PDSS_COMP_CTRL_COMP_PD;
    regval |= PDSS_COMP_CTRL_COMP_ISO_N;
    pd->comp_ctrl[COMP_ID_LSCSA_PFC] = regval;

    /* Now sample the comparator after 50us. Need to wait one LF clock cycle. */
    CyDelayUs(50);
    if ((pd->intr5_status_0 >> FILTER_ID_LSCSA_PFC) & 0x01)
    {
        retval = true;
    }

    /* Restore the settings. */
    pd->lscsa_0_ctrl = reg_lscsa_0_ctrl;
    pd->refgen_2_ctrl = reg_refgen_2_ctrl;
    pd->comp_ctrl[COMP_ID_LSCSA_PFC] = reg_comp_ctrl;
    CyDelayUs(10);

    /* Reset the filter, if any interrupt was configured. */
    if ((pd->intr5_mask & (1 << FILTER_ID_LSCSA_PFC)) &&
            (pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] & PDSS_INTR5_FILTER_CFG_FILT_EN))
    {
        pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] &= ~(
                PDSS_INTR5_FILTER_CFG_FILT_EN |
                PDSS_INTR5_FILTER_CFG_FILT_RESET);

        /* Set edge detection. */
        regval = pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] |
            PDSS_INTR5_FILTER_CFG_FILT_EN;
        if (((regval & PDSS_INTR5_FILTER_CFG_FILT_CFG_MASK) >>
                PDSS_INTR5_FILTER_CFG_FILT_CFG_POS) == FILTER_CFG_POS_DIS_NEG_EN)
        {
            regval |= PDSS_INTR5_FILTER_CFG_FILT_RESET;
        }
        pd->intr5_filter_cfg[FILTER_ID_LSCSA_PFC] |= regval;
    }

    CyExitCriticalSection(state);

    return retval;
}

bool pd_cmp_get_status(uint8_t port, filter_id_t id, bool is_filtered)
{
    bool state;
    uint8_t intr_state;
    uint32_t regval;
    PPDSS_REGS_T pd = gl_pdss[port];

    intr_state = CyEnterCriticalSection();

    if (id < FILTER_ID_MAX)
    {
        regval = pd->intr5_status_0;
        if (is_filtered)
        {
            regval >>= PDSS_INTR5_STATUS_0_FILT_12_POS;
        }
        if (pd->intr5_status_0 & (1 << id))
        {
            state = true;
        }
    }

    CyExitCriticalSection (intr_state);

    return state;
}
#endif /* (defined(CCG3PA) || defined(CCG3PA2)) */

/* End of file */
