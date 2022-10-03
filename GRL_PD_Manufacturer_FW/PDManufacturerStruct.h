/*
 * PDManufacturerStruct.h
 *
 *  Created on: Jun 24, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERSTRUCT_H_
#define PDMANUFACTURERSTRUCT_H_

#include <stddef.h>
#include <stdint.h>

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3utils.h"
#include "cyu3i2c.h"
#include "cyu3spi.h"
#include "cyu3gpio.h"
#include "spi_regs.h"
#include "gpio_regs.h"
#include "cyfxusbenumdscr.h"

#include "PDManufacturerDef.h"
#include "PDManufacturerEnums.h"
#include "PDManufacturerPeripheral.h"
#include "PDManufacturerFwControl.h"
#include "PDManufacturerTimer.h"

	uint8_t extern gTCSerialNumber[];

	uint8_t extern gRS485RxBuf[BUF_SIZE_280BYTE];
	uint8_t extern gRS485TxBuf[BUF_SIZE_280BYTE];
	uint8_t extern gI2CRxBuf[BUF_SIZE_280BYTE];
	uint8_t extern gI2CTxBuf[BUF_SIZE_280BYTE];
	uint8_t extern gMiscBuf[BUF_SIZE_280BYTE];
	uint8_t extern gPPSi2cTxBuf[BUF_SIZE_16BYTE];
	uint8_t extern gPPSi2cRxBuf[BUF_SIZE_16BYTE];
	uint8_t extern gPPSFWBuf[BUF_SIZE_280BYTE];
	uint8_t extern gBattStatusBuf[32];

#ifdef DBGLOG
	uint8_t extern gDbgBuf1Index;
#endif

typedef struct
{
	struct gSystemInfo
	{
		uint8_t gRaselectionSwitch;
		uint8_t gEloadVconnSelection;
		uint8_t gDpDmSwitchSlection;
		uint8_t gSystemID;
		uint8_t *gFWVersion;
		uint8_t gEventType;
		uint8_t gFWWriteIndex;
		uint8_t gVbusSenseSwitch;
		uint32_t dataBufIndex;
		uint32_t PrevReadIndex;
		uint32_t PrevWriteIndex;
		uint16_t EloadCtrlFramByteCount;
		uint16_t EloadCtrlFramByteAddress;
		CyBool_t DefaultEloadingFlag;	/**Flag decides whether to draw Eload by default after getting PDC/BC1.2 interrupt or not,configurable from API */
		uint8_t EloadStatus;	/**Variable tells status of the Eload(Added specifically for Aptive requirement), If previously Eload is set or Not**/
		uint16_t AutoEloadPercntDraw;
		CyBool_t CapmismatchEloadHandle;
		uint16_t EloadVbusVoltage;
		uint16_t EloadVbusCurrent;
		uint16_t EloadVconnVoltage;
		uint16_t EloadVconnCurrent;
		uint16_t gFunctionCardSerial_Number;
		uint8_t gFunctionCardRevision;
		uint8_t gFunctionCardFramRevision;
		uint8_t gFunctionCardRevsion_index;
		uint8_t gUnicolorLED_State;
		uint8_t gBicolorLED_State;
		CyBool_t CapMismatchStatusInfoFlag;/**pranay,16Dec'20, Flag added to indicate the SW that Cap mismatch has occurred, instead of handling the actual flag, handling this flag**/
		uint16_t PPSVbusVoltage;
		uint16_t PPSVbusCurrent;
		uint8_t gPPSFWBufIndex;
		uint16_t PPSVConnVoltage;
		uint16_t PPSVConnCurrent;
		uint16_t PollingIterReachCount;
		CyBool_t EnableGetBatteryCapsFetching;
	}gSystemInfo_t;

	struct gTimerParameters
	{
		uint8_t gTimerNo;
		uint32_t gTimerName;
		uint32_t gTimerValue;
	}gTimerParameters_t[BUF_SIZE_5BYTE];

	 struct FwTimerInfo
	{
		uint32_t Timer_gManuCount;
		uint16_t gTimer2Info;
	}FwTimerInfoStruct;

//	gTimerParameters gTimerParameters_t[BUF_SIZE_2BYTE];

	struct gSystemConfig
	{
		uint16_t gRS485ByteCount;

	}gSystemConfig_t;

	struct gSystemControl
	{
		uint16_t gRS485RxIntrCount;
		uint8_t gRS485ReInitDevCount;

	}gSystemControl_t;

	struct gUSBControl
	{
		uint8_t gUSBSpeed;
		CyBool_t gUSBStatus;
		CyBool_t gUSBCmdWriteFlag;
		CyBool_t gUSBConnectTimer;
		uint16_t gByteSize;
	}gSUSBControl_t;

}gFwHandle;

/**
 * @typedef pd_devtype_t
 * @brief Enum of the attached device type.
 */
typedef enum
{
    DEV_SNK = 1,                        /**< Power sink device is attached. */
    DEV_SRC,                            /**< Power source device is attached. */
    DEV_DBG_ACC,                        /**< Debug accessory is attached. */
    DEV_AUD_ACC,                        /**< Audio accessory is attached. */
    DEV_PWRD_ACC,                       /**< Powered accessory is attached. */
    DEV_VPD,                            /**< Vconn powered device is attached. */
    DEV_UNSUPORTED_ACC                  /**< Unsupported device type is attached. */
} pd_devtype_t;


/**
 * @typedef port_role_t
 * @brief Enum of the PD port roles.
 */
typedef enum
{
    PRT_ROLE_SINK = 0,                  /**< Power sink */
    PRT_ROLE_SOURCE,                    /**< Power source */
    PRT_DUAL                            /**< Dual Role Power device: can be source or sink. */
} port_role_t;

typedef enum
{
	SPECREV_2_0 = 1,
	SPECREV_3_0 = 2,
}DUT_Spec_rev;
typedef struct
{
	struct gPDCStatus
	{
		CyBool_t CapMismatch;
		uint8_t IsPDCdone;
		uint8_t PDCStatus;
		uint8_t gDUTSpecRev;
		uint8_t PDC_PDOSupplyType;
		uint8_t PDC_PDOIndex;
		uint16_t PDC_CurPwr;
		uint16_t PDC_MaxVolt;
		uint16_t PDC_MinVolt;
		uint16_t VbusValue;
		port_role_t grole_at_Connect;
		port_role_t gcur_port_role;
		pd_devtype_t gAttached_dev_type;
		uint16_t gSnkReqCurrent;
		uint16_t gSnkReqCurrentOCPLimit;
		CyBool_t isAttachDetach;
		CyBool_t isNonPDDUTConnc;//means NON PD Device connected

	}gPDCStatus_t;

	struct gBC12Status
	{
		uint8_t BC_FSMState; /**< Current state of the BC state machine. */
		uint32_t BC_evt;  /**<  representing event notifications of the CCG3PA BC state machine. */
		CyBool_t connected; /**< Whether BC connection is detected. */
		CyBool_t attach;  /**< Whether BC attach has been detected. */
		uint8_t BC_CurMode; /**< Active charging scheme. */
		uint16_t BC_MaxCurrent;

	}gBCStatus_t;
}gPDCStatus;

typedef struct
{
		uint8_t gActiveCC;
		uint8_t gPDSSDataFetchRetryCount;
		CyBool_t gIsPortVerificationEnabled;/**Variabled tells is port verification is in progress*/
		CyBool_t gReadPortVerfVconnDataFrom; /**if port verification is in progress, read and give data from asked CC lines irrespective of Active CC**/
		uint16_t gMinutesCount;
		uint16_t gElapsedTicks;
		uint8_t gPollingIterCnt;
		CyBool_t isOCPTriggered;
		uint8_t OCPTriggerCount;

}gPD_Struct;

typedef struct
{
	uint16_t gPhyErrCnt;
	uint16_t gLinkErrCnt;
	uint16_t gTotalPhyErrCnt;
	uint16_t gTotalLinkErrCnt;
	uint16_t gErrTestCnt;
	uint16_t gUSB2ErrCnt;
	uint16_t gTotalUSB2ErrCnt;
	uint32_t gGetRegvalue;
}gUsbErrStatus;

//typedef struct
//{
//	port_role_t grole_at_Connect;
//	port_role_t gcur_port_role;
//	pd_devtype_t gAttached_dev_type;
//
//}gSrcStruct;

typedef struct
{
	gFwHandle gFwHandle_t;
	gPD_Struct gPD_Struct;
	gPDCStatus gPDCStatus;
	gUsbErrStatus gUsbErrStatus_t;
//	gSrcStruct gSrcStruct_t;

}gFunctStruct;

gFunctStruct* GetStructPtr();

/* FX3 USB 3.0 Function Physical Layer and Link Layer Register Interface */
#define USB3LNK_BASE_ADDR (0xe0033000)

typedef struct
{
uvint32_t lnk_conf; /* 0xe0033000 */
uvint32_t lnk_intr; /* 0xe0033004 */
uvint32_t lnk_intr_mask; /* 0xe0033008 */
uvint32_t lnk_error_conf; /* 0xe003300c */
uvint32_t lnk_error_status; /* 0xe0033010 */
uvint32_t lnk_error_count; /* 0xe0033014 */
uvint32_t lnk_error_count_threshold; /* 0xe0033018 */
uvint32_t lnk_phy_conf; /* 0xe003301c */
uvint32_t reserved0[3];
uvint32_t lnk_phy_mpll_status; /* 0xe003302c */
uvint32_t reserved1[3];
uvint32_t lnk_phy_tx_trim; /* 0xe003303c */
uvint32_t lnk_phy_error_conf; /* 0xe0033040 */
uvint32_t lnk_phy_error_status; /* 0xe0033044 */
uvint32_t reserved2[2];
uvint32_t lnk_device_power_control; /* 0xe0033050 */
uvint32_t lnk_ltssm_state; /* 0xe0033054 */
uvint32_t reserved3[3];
uvint32_t lnk_lfps_observe; /* 0xe0033064 */
uvint32_t reserved4[52];
uvint32_t lnk_compliance_pattern_0; /* 0xe0033138 */
uvint32_t lnk_compliance_pattern_1; /* 0xe003313c */
uvint32_t lnk_compliance_pattern_2; /* 0xe0033140 */
uvint32_t lnk_compliance_pattern_3; /* 0xe0033144 */
uvint32_t lnk_compliance_pattern_4; /* 0xe0033148 */
uvint32_t lnk_compliance_pattern_5; /* 0xe003314c */
uvint32_t lnk_compliance_pattern_6; /* 0xe0033150 */
uvint32_t lnk_compliance_pattern_7; /* 0xe0033154 */
uvint32_t lnk_compliance_pattern_8; /* 0xe0033158 */
} USB3LNK_REGS_T, *PUSB3LNK_REGS_T;

#define USB3LNK ((PUSB3LNK_REGS_T) USB3LNK_BASE_ADDR)

/* FX3 UIB Top Level Register Interface */
#define UIB_BASE_ADDR (0xe0030000)

typedef struct
{
uvint32_t intr; /* 0xe0030000 */
uvint32_t intr_mask; /* 0xe0030004 */
uvint32_t rsrvd0[1024];
uvint32_t phy_clk_and_test; /* 0xe0031008 */
uvint32_t reserved[2];
uvint32_t phy_chirp; /* 0xe0031014 */
uvint32_t rsrvd1[250];
uvint32_t dev_cs; /* 0xe0031400 */
uvint32_t dev_framecnt; /* 0xe0031404 */
uvint32_t dev_pwr_cs; /* 0xe0031408 */
uvint32_t dev_setupdat0; /* 0xe003140c */
uvint32_t dev_setupdat1; /* 0xe0031410 */
uvint32_t dev_toggle; /* 0xe0031414 */
uvint32_t dev_epi_cs[16]; /* 0xe0031418 */
uvint32_t dev_epi_xfer_cnt[16]; /* 0xe0031458 */
uvint32_t dev_epo_cs[16]; /* 0xe0031498 */
uvint32_t dev_epo_xfer_cnt[16]; /* 0xe00314d8 */
uvint32_t dev_ctl_intr_mask; /* 0xe0031518 */
uvint32_t dev_ctl_intr; /* 0xe003151c */
uvint32_t dev_ep_intr_mask; /* 0xe0031520 */
uvint32_t dev_ep_intr; /* 0xe0031524 */
uvint32_t rsrvd2[182];
uvint32_t chgdet_ctrl; /* 0xe0031800 */
uvint32_t chgdet_intr; /* 0xe0031804 */
uvint32_t chgdet_intr_mask; /* 0xe0031808 */
uvint32_t otg_ctrl; /* 0xe003180c */
uvint32_t otg_intr; /* 0xe0031810 */
uvint32_t otg_intr_mask; /* 0xe0031814 */
uvint32_t otg_timer; /* 0xe0031818 */
uvint32_t rsrvd3[249];
uvint32_t eepm_cs; /* 0xe0031c00 */
uvint32_t iepm_cs; /* 0xe0031c04 */
uvint32_t iepm_mult; /* 0xe0031c08 */
uvint32_t rsrvd4[13];
uvint32_t eepm_endpoint[16]; /* 0xe0031c40 */
uvint32_t iepm_endpoint[16]; /* 0xe0031c80 */
uvint32_t iepm_fifo; /* 0xe0031cc0 */
uvint32_t rsrvd5[207];
uvint32_t host_cs; /* 0xe0032000 */
uvint32_t host_ep_intr; /* 0xe0032004 */
uvint32_t host_ep_intr_mask; /* 0xe0032008 */
uvint32_t host_toggle; /* 0xe003200c */
uvint32_t host_shdl_cs; /* 0xe0032010 */
uvint32_t host_shdl_sleep; /* 0xe0032014 */
uvint32_t host_resp_base; /* 0xe0032018 */
uvint32_t host_resp_cs; /* 0xe003201c */
uvint32_t host_active_ep; /* 0xe0032020 */
uvint32_t ohci_revision; /* 0xe0032024 */
uvint32_t ohci_control; /* 0xe0032028 */
uvint32_t ohci_command_status; /* 0xe003202c */
uvint32_t ohci_interrupt_status; /* 0xe0032030 */
uvint32_t ohci_interrupt_enable; /* 0xe0032034 */
uvint32_t ohci_interrupt_disable; /* 0xe0032038 */
uvint32_t ohci_fm_interval; /* 0xe003203c */
uvint32_t ohci_fm_remaining; /* 0xe0032040 */
uvint32_t ohci_fm_number; /* 0xe0032044 */
uvint32_t ohci_periodic_start; /* 0xe0032048 */
uvint32_t ohci_ls_threshold; /* 0xe003204c */
uvint32_t reserved1;
uvint32_t ohci_rh_port_status; /* 0xe0032054 */
uvint32_t ohci_eof; /* 0xe0032058 */
uvint32_t ehci_hccparams; /* 0xe003205c */
uvint32_t ehci_usbcmd; /* 0xe0032060 */
uvint32_t ehci_usbsts; /* 0xe0032064 */
uvint32_t ehci_usbintr; /* 0xe0032068 */
uvint32_t ehci_frindex; /* 0xe003206c */
uvint32_t ehci_configflag; /* 0xe0032070 */
uvint32_t ehci_portsc; /* 0xe0032074 */
uvint32_t ehci_eof; /* 0xe0032078 */
uvint32_t shdl_chng_type; /* 0xe003207c */
uvint32_t shdl_state_machine; /* 0xe0032080 */
uvint32_t shdl_internal_status; /* 0xe0032084 */
uvint32_t rsrvd6[222];
struct
{
uvint32_t shdl_ohci0; /* 0xe0032400 */
uvint32_t shdl_ohci1; /* 0xe0032404 */
uvint32_t shdl_ohci2; /* 0xe0032408 */
} ohci_shdl[64];
uvint32_t rsrvd7[64];
struct
{
uvint32_t shdl_ehci0; /* 0xe0032800 */
uvint32_t shdl_ehci1; /* 0xe0032804 */
uvint32_t shdl_ehci2; /* 0xe0032808 */
} ehci_shdl[64];
uvint32_t rsrvd8[5376];
uvint32_t id; /* 0xe0037f00 */
uvint32_t power; /* 0xe0037f04 */
uvint32_t rsrvd9[62];
struct
{
uvint32_t dscr; /* 0xe0038000 */
uvint32_t size; /* 0xe0038004 */
uvint32_t count; /* 0xe0038008 */
uvint32_t status; /* 0xe003800c */
uvint32_t intr; /* 0xe0038010 */
uvint32_t intr_mask; /* 0xe0038014 */
uvint32_t rsrvd10[2];
uvint32_t dscr_buffer; /* 0xe0038020 */
uvint32_t dscr_sync; /* 0xe0038024 */
uvint32_t dscr_chain; /* 0xe0038028 */
uvint32_t dscr_size; /* 0xe003802c */
uvint32_t rsrvd11[19];
uvint32_t event; /* 0xe003807c */
} sck[16];
uvint32_t rsrvd12[7616];
uvint32_t sck_intr0; /* 0xe003ff00 */
uvint32_t sck_intr1; /* 0xe003ff04 */
uvint32_t sck_intr2; /* 0xe003ff08 */
uvint32_t sck_intr3; /* 0xe003ff0c */
uvint32_t sck_intr4; /* 0xe003ff10 */
uvint32_t sck_intr5; /* 0xe003ff14 */
uvint32_t sck_intr6; /* 0xe003ff18 */
uvint32_t sck_intr7; /* 0xe003ff1c */
uvint32_t rsrvd13[56];
} UIB_REGS_T, *PUIB_REGS_T;


#define UIB ((PUIB_REGS_T) UIB_BASE_ADDR)

//typedef uint8_t (*gPTestExeManufacturer)( uint8_t );
//Declare of Array of function pointers
//gPTestExeManufacturer gPTestExeManuArray[8];

#endif /* PDMANUFACTURERSTRUCT_H_ */
