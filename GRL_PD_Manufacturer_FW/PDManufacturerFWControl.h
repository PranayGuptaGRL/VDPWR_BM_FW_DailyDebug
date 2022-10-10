/*
 * PDManufacturerFWControl.h
 *
 *  Created on: Jun 25, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERFWCONTROL_H_
#define PDMANUFACTURERFWCONTROL_H_

#define TI_PPS_SLAVEADDR			0xE0 //0x70
#define TI_SSBL_PPS_SLAVEADDR		0x1C//0x0E
#define CCG3PA_SLAVEADDR 			0x10 //0x08
#define ELOAD_SLAVEADDR  			0x12 //0x09
#define FRAM_SLAVEADDR 				0xAE //0x67
//#define DEBUG
#define CLK_PROG_SLAVE_ADDR			0x70
//#define CLK_PROG_SLAVE_ADDR_RD		(CLK_PROG_SLAVE_ADDR | 0x01)

#define I2C_DEV_ID_MEGACHIP			0x28 /**< The actual I2C Slave address of the Megachips device is 0x14 */
#define I2C_DEV_ID_MEGACHIP_RD		(0x28 | 0x01) /**< The actual I2C Slave address of the Megachips device is 0x14 */
#define DEFAULT						0xFF

#define BOOT_MODE	0xA555AA5A
#define PGM_MODE	0xFAAA55AF
uint8_t greadConfigBuf[24];

extern uint8_t gDbgBuf1[BUF_SIZE_1K_BYTE];
extern uint8_t gDbgBuf2[BUF_SIZE_128BYTE];


typedef enum
{
	FX3_Programming_mode_selection=0x01,

	CCG3PA_Programming_mode_selection = 0x03,
	CCG3PA_FW_Update = 0x04,
	DUTsFWUpdate = 0x06,
	TI_PPS_PROGRAMMING = 0x07,

}FW_update;

typedef enum
{
	TI_PPS_RESET			= 0x00,
	TI_PPS_MODESELECT 		= 0x01,
	TI_PSS_FW_ERASE = 0x0A,
	TI_FW_STREAM	= 0x03,
	TI_FW_FLASH_WRITE	= 0x04,
	TI_FW_SSBL_RESET	= 0x05,

}TI_PPS_FWUpdateAPIs;

typedef enum
{
	Boot_mode=0x01,
	second_stage_bootloader=0x04,
	CCG3PA_System_Reset = 0x0E,
	CCG3PA_I2C_Bootloader = 0x0F,

}prog_mode_selection;


typedef enum gTestFlowEnum
{
	Test_ReadDUTCaps = 1,
	Test_config_set = 2,
	Test_execution = 3,

}gTestFlow;

typedef enum gTestStepEnum
{
	Step_ConfigRequest = 1,
	Step_PDCValidation = 2,
	Step_ReadADC_Data_NOLoad = 3,
	Step_ReadADC_Data_FullLoad = 4,
	Step_ConfigEload_NOLoad = 5,
	Step_ConfigEload_FullLoad = 6,
	Step_ValidateADC_Data = 7,
	Step_Eload_TurnOFF	= 8,
	Step_DefaultPDO_Request = 9,
	Step_TestCaseTermination = 0xFF,
}gTestStep;

typedef enum gTestFlowEnum_Timer0
{
	Timer_tPdTxnHandle =0,


}gTestFlow_Timer0;
typedef enum{
	USB_3_0 = 0,
	USB_2_0,
}usb_speed;

typedef enum{
	LNK_PHY_TX_REG = 0,
	PHY_CONFIG_REG,
}usb_swing_de_emp;

CyBool_t RS485DataRecvIntrHandle(uint8_t *);
void CcgI2cdataTransfer(uint8_t *);
CyBool_t ValidateCommand(uint8_t *);
void RS485RXDataHandler(uint8_t *);
void DataHandler(uint8_t );
void InitBufStructInstance();
void InitFwConfigStructInstance();
void InitFwControlStructInstance();
void InitFwHandleInstance();
void InitTestConfigStructInstance();
void InitTestControlStructInstance();
void InitTestHandleInstance();
void InitializeFWApplication();
void ManufacturerTimer0Handler(uint8_t);
void ManufacturerTimer1Handler();
void StartManuTimer(uint8_t ,uint8_t,uint8_t );
void Config_Eload(uint8_t ,uint8_t *);
CyBool_t CheckUsbErrStatus(uint8_t );
void HandleEload(uint8_t ,uint8_t );
void DecodeADCData(uint8_t *);
void RS485TXDataHandler(uint8_t *, uint8_t  );
void FRAM_Write(uint8_t *);
void ADC_Data_Handle(uint16_t ,uint8_t *, CyBool_t );
void DecodeCapabilities();
void GetUsbStatusInfo(uint8_t *);
CyBool_t FwModeSelection(uint32_t );
void UsbLoopackCommands(uint8_t *);
void Fill_Datalog_buffer_2Bytes( uint16_t , uint8_t * );
void Fill_Datalog_buffer_1Byte(uint8_t , uint8_t * );
void BC12_InterruptHandle(uint8_t *);
void BC12_ValidationBufFill();
CyU3PReturnStatus_t BoardIDDetection();
void EloadFRAM_DataConfig();
void EloadFRAM_DataWrite();
uint8_t PDSSCaps_DataAppend();
void Validate_CCG3PA_Program(uint8_t *);
void TC_PDC_DataDecode(uint8_t );
void ToggleDebugLED(uint8_t);
void ToggleUSBTxnRxnLED();
void PDSS_InterruptHandler(uint8_t *);
void PDSSInterrupt_Validation_Handler();
void RS485WriteDataDriverDisable();
void GetErrorStatus();
void InitFWConfigControl();
void LinkSpeedCommIndicationHandle(uint8_t );
void MainLinkCommIndicationHandle(uint8_t );
void PDC_Validation();
void PDSS_StatusLEDHandle(uint8_t );
void VBUSDetection_Handler(CyBool_t );
void VbusPresenceLEDinidcator(uint8_t );
void PDNegLedIndication(uint8_t);
void DataErrorLEDIndicator(uint8_t );
void DataLockIndicator(uint8_t );
void DpDmSwitchHandle(uint8_t );
CyU3PReturnStatus_t SetUsbPowerState(uint8_t *);
void USBFailSafeCondition();
uint32_t PercentageLoadSetting(uint8_t ,uint32_t );
void ConvertDeToAsciiByte(uint8_t*, uint16_t, uint8_t);
void RackindicationLEDhandle(uint8_t *);
void StatusInformationHandle(uint8_t *);
void Funcard_Info(uint8_t *);
void CCline_SwithSelection(uint8_t );
void CableType_Selection(uint8_t );
void ReadLoopbackStatus(uint8_t );
void PD_Attach_Detach(uint8_t );
void FetchFuncard_Serialnum_Boardrev(uint8_t ,uint8_t *);
void Check_FcardBoardrev();
void Read_Serialnumber_Boardrev(uint8_t *);
void CapsMisMatchHandler();
void PD_Enable();
void GetPDstate(uint8_t *);
void DetachHandler();
void PDSSInterruptGPIOHandler(CyBool_t);
void ReadSrcCapBufFill(void);
uint8_t GetVconnStatusValues();
uint8_t PDC_LED_Status(uint8_t *,uint8_t  );
void GetPDC_CapabilityBuffFill(uint8_t *);
void ResetUSBErrorCount(uint8_t );
void RegGpioSetValue(uint8_t ,CyBool_t );
void PDSSDataReadHandler(uint8_t *);
void InterruptConfig(CyBool_t );
void GetPhy_LinkerrorCount(uint8_t *);
void FetchuVDMDataBufFill();
void CCGxPortRoleFetchBufFill();
uint32_t ReadReg(uint32_t );
void ConfigReg(uint32_t  , uint32_t );
void FetchEventLogBufFill();
CyU3PReturnStatus_t getElapsedTimerTicks();
void TimeStampTimerReset();
void TimeStampTimerStop();
void TimeStampTimerStart();
void RS485ISN_DriveEnableHandler();
uint16_t gFetchTimerVal(uint16_t );
void AttachStateIntrHandler(uint8_t * aBuffer);
void ReqStateIntrHandler(uint8_t * aBuffer);
void PpsVbusZeroIOHandler(CyBool_t  );
void FetchPDSSVbusBufFill();
void DetachStateVbusHandler();
uint8_t GetPPSVbusValues();
void PPSHWReset();
void SrcCapsUpdateHandler(uint8_t *aBuffer);
void FetchBattStatusDetailsBuff();
uint8_t StatusInfoBattCapsDataAppend(uint8_t *aTXBuffer, uint8_t *aRXBuffer);
void FetchStatusMsgInfoBufFill();
void FetchBattStatusInfoBufFill();

#endif /* PDMANUFACTURERFWCONTROL_H_ */
