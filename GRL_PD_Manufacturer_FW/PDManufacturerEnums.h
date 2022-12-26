/*
 * PDManufacturerEnums.h
 *
 *  Created on: Jun 14, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERENUMS_H_
#define PDMANUFACTURERENUMS_H_

#define I2C_SLAVE_ADDRESS               (0xA0u)
#define I2C_CMD_WRITE                   (0)
#define I2C_CMD_READ                    (1)
#define I2C_FW_CHECK_ADDR				(0xFFFC)		/**< FW Update Mode check is done at EEPROM Address 0x3FFFFFFFC */
#define GET_BYTE0(addr)                 ((uint8_t)((addr) & 0xFF))
#define GET_BYTE1(addr)                 ((uint8_t)(((addr) >> 8) & 0xFF))
#define I2C_START_BYTE2                 (0x0004)
static uint8_t gI2cSlaveAddrFwCheck 	= 3;

#define CCG3PA_HDR_BYTECNT		3//3Bytes
//#define GET_BATT_SOCTEMP_DETAILS_PAYLOAD_CNT		4//Bytes
#define POLLING_ITER_BATTCAP_INIT_COUNT		0
#define POLLING_ITER_REACH_CNT		50

#define INTR_SET						0
#define INTR_CLR						1
#define HEADER_BYTECOUNT				2
#define DATA_READ_RETRYCOUNT			1
#define GETBATTCAPS_PL_LENGTH			13
#define CCG3PA_RD_CHKSUM				0xDC
#define CCG3PA_WR_CHKSUM				0xCD
#define GET_DUTCAPS_CDWORD				0xD1
#define GET_PDC_DETS_CDWORD				0xD4
#define GET_UVDM_DATA_CDWORD				0xD6
#define GET_CCGX_PORTROLE_CDWORD			0xD7
#define GET_SRCCAPS_EXTND_CDWORD			0xD8
#define GET_GET_SOP1_RX_PKTDATA_CDWORD		0xD9
#define GET_DPDM_DATA_CDWORD				0xDA
#define GET_EVENTLOG_BUF_DATA_CDWORD		0xA2
#define GET_PDSS_VBUS_DATA					0xD3
#define GET_BATT_S0CTEMPDETAILS_CDWORD		0xA3
#define GET_STATUS_MSG_INFO_CDWORD			0xA4
#define GET_BATTERY_STATUS_INFO_CDWORD		0xA5
#define OCP_LIMIT_PERCENTAGE				130
#define OCP_LIMIT_3A_GREATER_P_I			105
#define RS485MAXDATA_TF_LENGTH				128
volatile uint8_t gErrRetryCount;

#define DBGLOG

//static uint32_t gBuf[32];

typedef enum FWCtrlTimer2TypeEnum
{
	Timer_RS485DevInit = 0,
}FWCtrlTimer2Type;

typedef enum FWCtrlTimer1TypeEnum
{
	Timer_CCG3PA_Pgm_I2C_Read = 1,
	Timer_TempLimitExceedHandle = 2,
	Timer_QC_MODE_DISABLE = 3,
	Timer_QC_MODE_ENABLE  = 4,
	Timer_Debug_LED_Toggle = 0xAA,

}FWCtrlTimer1Type;

typedef enum FWCtrlTimer0TypeEnum
{
	Timer_tPdGetCapability = 0,//1000mS
	PDSS_InterruptHandling = 1,//500mS
	USB_StatusRetrieve 		= 2,//100mS
	Eload_ActiveCCFetch		= 3,//10mS
	PDSS_ActiveCCDetect     = 4,//5mS
	Eload_ActiveCCDetectAPI	= 5,//5mS
	Read_FRAMData			= 6,//5mS
	CCG3PA_BootUp_Poll		= 7,//5mS
	PDSSInterrupt_Validation = 8,
	BC12_InterruptHandling  = 9,//1000mSec
	Eload_Sync_Handling		= 10,
//	Timer_Connect_State      = 11,
	Timer_tGetDUTCapabilities = 12,//30mS
	Timer_tGetPDCInfo 		= 13,//30mS
	Timer_tGetCCGxPortRole = 14,//30mS
	Timer_tGetuVDM_Status = 15,//30mS
	Timer_tGetSrcCapsEntnd = 16,//30mS
	Timer_tGetEventlogBufData = 17,//30mS
	Timer_tMiscHandler = 18,//30mS
	Timer_tGetSOp1RxPktData = 19, //30mS
	Timer_tGetDpDmData	= 20,//30mS
	Timer_ReadppsI2CData = 21,//30mS
	Timer_tGetPDSSVbusData = 22,//30mS
	Timer_tGetBatteryS0CTempDetails = 23,//30mS
	Timer_tGetStatusMsgInfo	 = 24,//Getstatus
	Timer_tGetBatterystatusInfo = 25,//Get Battery status
	Timer_PDSS_InitGetBatteryCaps = 26,//For Getting batteryCaps afterPDC
}FWCtrlTimer0Type;

typedef enum FWCtrlTimerTypeEnum
{
	Timer_tSampleCheck	= 0,//60000mSec (2^16 timer)

}FWCtrlTimerTypeEnum;

typedef enum FWCtrTimer4TypeEnum
{
	Timer4_Connect_State      = 0,

}FWCtrTimer4TypeEnum;

typedef enum
{
	VUP_AS_SINK = 0x03,
	VUP_AS_SRC	= 0x04,
	VUP_AS_DRP	= 0x06,
}VUPRole;
typedef enum
{
	Cmd_Set 	 = 0x01,
	Cmd_Program  = 0x02,
	Cmd_Get		 = 0x07,
	Cmd_Polling	 = 0x0F,
}cmd_t;

typedef enum
{
	getDUTCapabilities = 0x01,
	getSnkCapabilities,
	getBC12Status,
	getSrcCapsextd,
	getActiveCCStatus = 0xF0,
	getPDCInfo = 0xF1,
	getPdssVBUSdata = 0xF2,
	getVDMInfo = 0xF6,
	getCCGxPortRole = 0xF7,
	getSOP1RxPktData = 0xF8,
	getDpDmData = 0xF9,
	getEventlogBufData = 0xAA,
	gTCEventLogFetch = 0xAB,
	getBatteryS0CTempDetails = 0xB1, //Custom configuration includes battery status and get status responses
	getStatusMsgInfo	 = 0xB2,//Getstatus
	getBatterystatusInfo = 0xB3,//Get Battery status
}Cmd_PDSSRead_t;
typedef enum
{
	Stop = 0,
	Start = 1,
}TestCaseSignal_t;

typedef enum
{
	InTestcaseStartPhase =0xA5,
	InDUTCapsRecvPhase = 0x01,
	InRequestConfigPhase = 0x02,
	InEloadConfigPhase = 0x03,
	InCapabilitiesValidationPhase = 0x04,
	InTerminationPhase = 0x05,
	IdlePhase = 0x06,
	TestCaseEndAbruptly = 0x07,
	TestCaseNotRunning = 0xFE,
	InTestcaseStopPhase =0xF5,
}TestCaseStatus_t;


typedef enum
{
	TIMER0 = 0,
	TIMER1,
	TIMER2,
	TIMER3,
	TIMER4,
}timers_t;

typedef enum
{
	EnumerationDone = 1,
	DataTx_inProgress = 2,
	NotConnected 	= 3,
	Reserved	=4,
}DataTxStatus;

typedef enum
{
    PDO_FIXED_SUPPLY = 0,               /**< Fixed (voltage) supply power data object. */
    PDO_BATTERY,                        /**< Battery based power data object. */
    PDO_VARIABLE_SUPPLY,                /**< Variable (voltage) supply power data object. */
    PDO_AUGMENTED                       /**< Augmented power data object. */
} pdo_t;

typedef enum
{
	Eload_Testcase_NOLoad = 1,
	Eload_Testcase_FULLLoad = 2,
	Eload_Gen = 3,
	Eload_BC12 =4,
	Eload_BootUpSync = 5,
	Eload_BootUp_Init = 0xB0,
	Eload_TurnOFF = 0xF0,
	Eload_StandBy = 0xF1,

} ELoadCase_t;

typedef enum
{
	External_Connector =0,
	TypeC_Connector	=1,
} VbusSense;

typedef enum
{
	TEST_IDLE = 0,
	PD_NEGOTIATION_WAIT,
	TEST_EXECUTION,
	TEST_END,
}test_phase_t;

typedef enum
{
	STEP_IDLE = 0,
	STEP_PD_REQUEST,
	STEP_GET_PDC_STATUS,
	STEP_APPLY_E_LOAD,
	STEP_FETCH_DATA,
	STEP_UPDATE_RESULT,
	STEP_END,
}test_step_t;

/**
 * @typedef bc_charge_mode_t
 * @brief List of legacy battery charging schemes supported over a Type-C or Type-A port.
 */
typedef enum{
    BC_CHARGE_NONE = 0,         /**< No active battery charging modes. */
    BC_CHARGE_DCP,              /**< Dedicated Charging Port as defined by BC 1.2 spec. */
    BC_CHARGE_QC2,              /**< QC2.0 charger. */
    BC_CHARGE_QC3,              /**< QC3.0 charger. */
    BC_CHARGE_AFC,              /**< Adaptive Fast Charging mode. */
    BC_CHARGE_APPLE,            /**< Apple power brick. */
    BC_CHARGE_CDP               /**< Charging Downstream Port as defined by BC 1.2 spec. */
}bc_charge_mode_t;

typedef enum{
	CLR_ALL_ERRCOUNT = 0,       /**Clear all USB error count*/
	CLR_PRESENT_ERRCOUNT,		/**Clear present USB error count*/
	CLR_ALLUSB2_EERCOUNT,        /**Clear all the USB2 error count*/
	CLR_PRESENTUSB2_ERRCOUNT,   /**Clear present USB2 error count*/
}usb_error_status;




#endif /* PDMANUFACTURERENUMS_H_ */
