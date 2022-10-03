
/*
 * PDManufacturerFWControl.c
 *
 *  Created on: Jun 25, 2019
 *      Author: Prasanna
 */

#include "PDManufacturerStruct.h"

uint8_t glFirmwareID[24] __attribute__ ((aligned(32))) = "5.3.5";

gFunctStruct * gFunctStruct_t;

/**
 * @brief This Function is used to calculate No. of mS after which RS48 has t be resetted after receiving First Byte of any write data.
 * Time is calculated based on the total payload length(2nd Byte[1:0] in received data) and for each 48Bytes 30mS should be a minimal delay
 * @param aPayLoadLength : total payload length received in write command . 2nd byte in read command indicates this value
 * @return return the timer value in mS
 * @author Pranay
 * @date 03Dec'20
 */
uint16_t gFetchTimerVal(uint16_t aPayLoadLength)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	/**If payload length is >48 BYtes than increment count by 1 and reduce payload lenght by 48 and again call same function*/
	if(aPayLoadLength > BUF_SIZE_48BYTE)
	{
		gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485ReInitDevCount += 1;
		return (gFetchTimerVal(aPayLoadLength - BUF_SIZE_48BYTE));/**Calling the same function again with reduced payload length**/
	}
	/***if payload lenght is less than 48Bytes (which will be in most of the cases) than return actual timer delay by multiplying with 30
	 * so that timer will start based in our requirement - which is to reset rs485 after every write, so it is expected that for every 48Bytes of data it takes 30mS to write.
	 *
	 * */
	else if(aPayLoadLength <= BUF_SIZE_48BYTE)
	{
		return (gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485ReInitDevCount * 30);
	}

}
/**
 * @brief The Interrupts from the rS485 module, MAX3140 will be generated for each individual bytes recveid from the host
 * Once the no of bytes become 255 bytes, then whole data is being received from the Host, the data need to be handled
 * This data handling will be done in Data Handler Thread, DataRxEvent is being set and infored the task to handle the rest
 * @author Prasanna
 * @date 02Sept'19
 */
CyBool_t RS485DataRecvIntrHandle(uint8_t *lReadBuf)
{
	CyBool_t lRetStatus = CyFalse;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	if(gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount <= (gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount - 1))
	{
		if(0x80 == (lReadBuf[0] & 0x80))
		{
			if(gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount == 0)
			{
				memset(gRS485RxBuf, 0, BUF_SIZE_256BYTE);
				/**Pranay,23March'21, To avoid Rs485 hung, Starting a 30mS timer and in expiry resetting rs485, if 1st byte is received will stop this timer and start timer based on API byte count*/
				gFunctStruct_t->gFwHandle_t.FwTimerInfoStruct.gTimer2Info = Timer_RS485DevInit;
				MsgTimerStart( 30 , TIMER2);
			}
			gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount] = lReadBuf[1];

			if(gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount == 1)
				gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = (gRS485RxBuf[1] + 2);

			/**Pranay,03Dec'20, Controlcard will come to know the payload length to be read in 2nd byte[1:0] in every read operation, so after receiving 2nd byte based on total payload length
			 * starting a timer with respective timer value and than resetting the rs485, to ensure communication doesnt hung after every read.**/
			if(gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount == 1)
			{
				gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485ReInitDevCount = 1;
				gFunctStruct_t->gFwHandle_t.FwTimerInfoStruct.gTimer2Info = Timer_RS485DevInit;
				MsgTimerStart( gFetchTimerVal(gRS485RxBuf[1]) , TIMER2);
			}
			gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount++;
		}
		else
		{
//			DEBUG_LOG(DBG1, 0xE1, 0);
			if(gErrRetryCount++ < 2)
				SPIDataReadErrorHandler();
			else
			{
				RS485DeviceInit();  // RS485 initialization
//				DEBUG_LOG(DBG1, 0xE2, 0);
			}
			return CyFalse;
		}
	}
	if(gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount == gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount)	//Entire 255 bytes of data received means then set the event for data handling
	{
		MsgTimerStop(TIMER2);
		gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;
		gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;

		lRetStatus = ValidateCommand(gRS485RxBuf);
	}
	return lRetStatus;
}

/**
 * @brief This Function Handles/Retrieves the status and speed of USB loopBack and Sending back the statu &speed to Application
 * whenever we receive the API from Application.
 *
 * @author Pranay
 * @date 02Sept'19
 */
void USBLoopbackHandler()
{
		gFunctStruct_t = (gFunctStruct *)GetStructPtr();
		gI2CTxBuf[0] = 0xD5;
		gI2CTxBuf[1] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed;
		gI2CTxBuf[2] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus;
		MsgTimerStart(USB_StatusRetrieve, TIMER0);
}
/**
 * @brief Function for handling the LED's and switches based on VBUS detected ON/OFF
 * @param aVar will  be 0: VBus is turned OFF, 1:If VBus is turned ON
 * @author Pranay
 * @date 10Spet'19
 */
void VBUSDetection_Handler(CyBool_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	switch(aVar)
	{
	case 0:	/**Detected that Vbus is OFF i.e., Got Detach interrupt */
		PDSS_StatusLEDHandle(0);
		VbusPresenceLEDinidcator(0);
		PDNegLedIndication(0);
		MainLinkCommIndicationHandle(NotConnected);
		LinkSpeedCommIndicationHandle(CY_U3P_NOT_CONNECTED);//1:HighSpped,2.FullSpeed; 3:SuperSpeed,0:NC
		DpDmSwitchHandle(2);
		DataErrorLEDIndicator(0);// turn off the data error indicator LED.

		//DataLockIndicator(1); //off
		HandleEload(Eload_TurnOFF,0);/*Pranay, Turning off eload incase if detach**/
		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus = CyFalse; // USB disconnected.
		ResetUSBErrorCount(CLR_PRESENT_ERRCOUNT); // clear present USB error count.
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapmismatchEloadHandle = CyFalse;
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch = CyFalse;
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag = CyFalse;

		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role = PRT_ROLE_SINK;
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gAttached_dev_type = 0x00;
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = DETACH;

	break;

	case 1:	/**Detected that Vbus is ON i.e., Got Attach interrupt*/
		VbusPresenceLEDinidcator(1);
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = ATTACH;

	break;
	}
}


/**
 * @brief This function Turns ON/OFF the respective LED based on the Link Status
 * @param alinkStatus is the parameter we decide the status of the Main link status
 * @author Pranay
 * @date 02Sept'19
 */
void MainLinkCommIndicationHandle(uint8_t alinkStatus)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(alinkStatus)
	{
	case EnumerationDone://For DataEnumeration Done
		CyU3PGpioSetValue (GPIO2_LED1_BI_A, 1);
		CyU3PGpioSetValue (GPIO50_LED1_BI_C, 0);

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x02);

	break;
	case DataTx_inProgress://For DataTx in progress
		CyU3PGpioSetValue (GPIO2_LED1_BI_A, 0);
		CyU3PGpioSetValue (GPIO50_LED1_BI_C, 1);

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x01);

	break;

	case NotConnected://Not Connected
		CyU3PGpioSetValue (GPIO2_LED1_BI_A, 0);
		CyU3PGpioSetValue (GPIO50_LED1_BI_C, 0);

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xFC);

	break;
	case Reserved:
		CyU3PGpioSetValue (GPIO2_LED1_BI_A, 1);
		CyU3PGpioSetValue (GPIO50_LED1_BI_C, 1);

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xFC);

		break;
	}

}

/**
 * @brief Led's Control based on the LinkSpeed
 * @param alinkSpeed, if alinkSpeed is default is it handled based on the main link speed, and remaining cases handled/forcing manually based on the alinkSpeed
 * @author Pranay
 * @date 02Sept'19
 */
void LinkSpeedCommIndicationHandle(uint8_t alinkSpeed)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(alinkSpeed)
	{
	case CY_U3P_FULL_SPEED://Turning OFf in FULL_Speed
	case CY_U3P_NOT_CONNECTED:
		CyU3PGpioSetValue (GPIO4_LED2_BI_A, 0);
		CyU3PGpioSetValue (GPIO5_LED2_BI_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xF3);
		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed = CY_U3P_NOT_CONNECTED;
		break;
	case CY_U3P_HIGH_SPEED:
		CyU3PGpioSetValue (GPIO4_LED2_BI_A, 1);
		CyU3PGpioSetValue (GPIO5_LED2_BI_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xF3);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x08);
		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed = CY_U3P_HIGH_SPEED;

	break;
	case CY_U3P_SUPER_SPEED://Superspeed
		CyU3PGpioSetValue (GPIO4_LED2_BI_A, 0);
		CyU3PGpioSetValue (GPIO5_LED2_BI_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xF3);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x04);
		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed = CY_U3P_SUPER_SPEED;

	break;

	default://Handling based on Connection Type
		if((gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus == CyTrue) &&
			((gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed == CY_U3P_HIGH_SPEED) ||
			(gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed == CY_U3P_FULL_SPEED)))
		{
			CyU3PGpioSetValue (GPIO4_LED2_BI_A, 1);
			CyU3PGpioSetValue (GPIO5_LED2_BI_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x08);
		}
		else if(gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus == CyTrue &&
				gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed == CY_U3P_SUPER_SPEED)
		{
			CyU3PGpioSetValue (GPIO4_LED2_BI_A, 0);
			CyU3PGpioSetValue (GPIO5_LED2_BI_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x04);
		}
		else //if Application is Not Active
		{
			CyU3PGpioSetValue (GPIO4_LED2_BI_A, 0);
			CyU3PGpioSetValue (GPIO5_LED2_BI_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xF3);
		}
	break;
	}
}
/**
* Pranay,30May'21, For R3 TCs For GND isolation repeaters are being used which are pulling up rs485 bus if multiple cards are connected,
* So handling the repeaters based on the GPIO state.
* This GPIO needs to be OFF when RS485 Driver is ON and vice versa.
* avar is the control signal to turn off/on GPIO
*/
//void RS485ISN_DriveEnableHandler(CyBool_t aVar)
//{
//	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
//
//	CyU3PGpioSetValue (GPIO35_ISN_RS485_DE, aVar);
//}

/**
 * @brief LED indication for whether the Current operation of CCG3pa is related to BC1.2/PD and aStatus->PDCStatus
 * @param aVar handles LED's based on this parameter
 * @author Pranay
 * @date 31July'19
 */
void PDSS_StatusLEDHandle(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();


		switch(aVar)
		{
		case 0://OFF both
			CyU3PGpioSetValue (GPIO6_LED3_BI_A, 0);
			CyU3PGpioSetValue (GPIO7_LED3_BI_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xCF);

			break;
		case 1://PD
			CyU3PGpioSetValue (GPIO6_LED3_BI_A, 1);
			CyU3PGpioSetValue (GPIO7_LED3_BI_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x20);

			break;
		case 2://BC1.2
			CyU3PGpioSetValue (GPIO6_LED3_BI_A, 0);
			CyU3PGpioSetValue (GPIO7_LED3_BI_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x10);

			break;
		case 3://ON both
			CyU3PGpioSetValue (GPIO6_LED3_BI_A, 1);
			CyU3PGpioSetValue (GPIO7_LED3_BI_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xCF);

			break;
		}
}
/**
 * @brief Data Error LED indicator, If any Error occurs while running the Mainlink testcase will Turn On LED based on aStatus
 * @param aStatus Handling LED's based on Error using this parameter
 * @author Pranay
 * @date 31July'19
 */
void DataErrorLEDIndicator(uint8_t aStatus)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aStatus)
	{
	case 0x00://OFF both LED's
		CyU3PGpioSetValue (GPIO8_LED4_BI_A, 0);
		CyU3PGpioSetValue (GPIO9_LED4_BI_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);

		break;
	case 1://Error
		CyU3PGpioSetValue (GPIO8_LED4_BI_A, 1);
		CyU3PGpioSetValue (GPIO9_LED4_BI_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x80);

		break;
	case 2://Normal
		CyU3PGpioSetValue (GPIO8_LED4_BI_A, 0);
		CyU3PGpioSetValue (GPIO9_LED4_BI_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x40);

		break;
	case 0x03:
		CyU3PGpioSetValue (GPIO8_LED4_BI_A, 1);
		CyU3PGpioSetValue (GPIO9_LED4_BI_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);

		break;
	}
}

/**
 * @brief Handling Dp/Dm Switch based on received API command and based on the operation related to PD/BC/LoopBack
 * @param aVar decides the switch handling if 1: BC1.2/PD, 2: USB loopback
 * @author Pranay
 * @date 31July'19
 */
void DpDmSwitchHandle(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
//	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gDpDmSwitchSlection = aVar;
	switch(aVar)
	{
	case 1: /**Command related to BC1.2/PD ie, For connecting DpDm to CCG3PA*/
		CyU3PGpioSetValue (GPIO14_SEL0_DP_SW, 1);
	break;
	case 2: /**Command for USB Loopback ie.,For Connecting DpDm to FX3 */
		CyU3PGpioSetValue (GPIO14_SEL0_DP_SW, 0);
	break;
	}
}

/**
 * @brief If Loop back data transaction started than turn ON else OFF
 * @param aVar 0-(ON)/1-(OFF)
 * @author Pranay
 */
void DataLockIndicator(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aVar)
	{
	case 0://on
		CyU3PGpioSetValue (GPIO12_LED3_S_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State &= 0xBF;

		break;
	case 1://Off
		CyU3PGpioSetValue (GPIO12_LED3_S_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x40;

		break;
	}
}

/**
 * @brief LED control based on PD negotiation success/Not
 * @param aVar Parameter for switching LED
 * @author Pranay
 * @date 31July'19
 */
void PDNegLedIndication(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aVar)
	{
	case 0://fail
		CyU3PGpioSetValue (GPIO13_LED4_S_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x80;

	break;
	case 1://pass
		CyU3PGpioSetValue (GPIO13_LED4_S_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State &= 0x7F;

	break;
	default://fail
		CyU3PGpioSetValue (GPIO13_LED4_S_C, 1);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x80;

	break;
	}
}
/**
 * @brief Turning On LED if Vbus present else OFF
 * @param aVar
 * @author Pranay
 * @date 31July'19
 */
void VbusPresenceLEDinidcator(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aVar)
		{
		case 0://Low//Fail
			CyU3PGpioSetValue (GPIO11_LED2_S_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x20; // Active high then Vbus off

		break;
		case 1://HIgh//Pass
			CyU3PGpioSetValue (GPIO11_LED2_S_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State &= 0xDF;  // Active low then Vbus on

		break;
		default://fail
			CyU3PGpioSetValue (GPIO11_LED2_S_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x20;  // Active high then Vbus off

		break;
		}
}
/**
 * @brief For Handling the Vbus selection switch whether to select from TypeC VBus or External Vbus Connector
 * @param aVar
 * @author Pranay
 * @date 31July'19
 */
void VbusSenseSlectionSwitch(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch = aVar;

	switch(aVar)
		{
		case External_Connector://External Vbus
			CyU3PGpioSetValue (GPIO45_VBUS_SENSE_VOLT_EN, 0);
		break;
		case TypeC_Connector://TypeC Vbus
			CyU3PGpioSetValue (GPIO45_VBUS_SENSE_VOLT_EN, 1);
		break;
		default://External Vbus
			CyU3PGpioSetValue (GPIO45_VBUS_SENSE_VOLT_EN, 0);
		break;
		}
}

/**
 * @brief Function for Handling the VBUS Short-Circuit Testing
 * @param aVar 0::OPen/OFF/Not-Shorted, 1 :: Closed/Shorted/ON
 * @author Pranay
 * @date 31Jul'19
 */
 void VbusShortCktGPIOHandler(uint8_t aVar)
 {
	 gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	 	switch(aVar)
	 	{
	 	case 0://Open/OFF/Not-Shorted
			CyU3PGpioSetValue (GPIO24_VBUS_SHORT, 0);
	 		break;
	 	case 1://Closed/Shorted/ON
			CyU3PGpioSetValue (GPIO24_VBUS_SHORT, 1);
	 		break;
	 	}
 }
/**
 * @brief Controlling based on Based On Active CC
 * @note Tx and Rx lines orientation will be varying based on Active CC so handling the switch respectively
 * @param aActiveCC (0) CC1 TxRx / (1)CC2 TxRx
 * @author Pranay
 */
 void TxRxSwitchHandler(uint8_t aActiveCC)
 {
	 gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	 switch(aActiveCC)
	 {
	 case 0://CC1
		 CyU3PGpioSetValue (GPIO15_DP_AUX_4, 0);
		 break;
	 case 1://CC2
		 CyU3PGpioSetValue (GPIO15_DP_AUX_4, 1);
		 break;
	 }
 }
/**
 * @brief Function to decide whether to draw Eload after PDC/BC1.2 device detection or not
 * @param aBuffer Control API FX3 receives from Application
 */
 void DefaultEloadingSelection(uint8_t *aBuffer)
 {
	 gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	 gFunctStruct_t->gFwHandle_t.gSystemInfo_t.DefaultEloadingFlag  = (aBuffer[4] & 0x01); /**Should either be On/OFF, By default OFF, only turning On if we get API from Application */
	 gFunctStruct_t->gFwHandle_t.gSystemInfo_t.AutoEloadPercntDraw = aBuffer[5]; /**Percentage of Max current to be drawn by default from Eload after every PDC*/
 }
/**
 * @brief Function called from control card if temp exceeds limit set , if temp exceeeds liit set tester card has to :
 * 	1. Turn Off Eload
 * 	2. Detach ccg3pa
 * 	3. Turn On 1sec timer in which All LEDs will be blinking until temp restores to normal condition
 *
 * 	if temp comes back to normal :
 * 	1. Stop timer and 2. Attach All ports
 * 	@date 17 Oct'20
 *  @author Pranay
 *  @param aBuffer
 */
void TempLimitExceedHandler(uint8_t *aBuffer)
{
	 gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	 switch(aBuffer[4])
	 {
	 case 0:/**Temp restored to normal range*/
		 MsgTimerStop(TIMER1);

		 PDNegLedIndication(0x00);
		 PDSS_StatusLEDHandle(0x00);
		 DataErrorLEDIndicator(0x00);

//		 DataLockIndicator(0x01);
//		 VbusPresenceLEDinidcator(0x00);
//		 MainLinkCommIndicationHandle(NotConnected);

		 PD_Attach_Detach(0x01);/**Attach*/

		 break;
	 case 1:/**Temp exceeded set range**/
		 HandleEload(Eload_TurnOFF,0);
		 MsgTimerStart(Timer_TempLimitExceedHandle, TIMER1);
		 PD_Attach_Detach(0x02);/**Detach*/
		 break;
	 }
}

/**
 * @brief Function for Handling Ra Assertions
 * @param aVar Parameter for Ra Selection
 * @note 0::Ra on both CC's,1::Ra on CC1,2:Ra on CC2,3:: Ra based on Communicating CC
 * @author Pranay
 * @date 30Aug'19
 */
void RaSelection(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	switch(aVar)
	{
	case 0x00://Ra Removal on both CC's
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue(GPIO41_CC1_RA, 0);
		CyU3PGpioSetValue (GPIO42_CC2_RA, 0);
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)  //check for board revision if it is rev1.0 disable the CC switch
		CCline_SwithSelection(4);  //both the CC are enabled.

	break;
	case 0x01://Ra on CC1
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue (GPIO41_CC1_RA, 1);
		CyU3PGpioSetValue (GPIO42_CC2_RA, 0);
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)  //check for board revision if it is rev1.0 disable the CC switch
		CCline_SwithSelection(2); //CC2 switch enable

	break;
	case 0x02://Ra on CC2
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue (GPIO41_CC1_RA, 0);
		CyU3PGpioSetValue (GPIO42_CC2_RA, 1);
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)   //check for board revision if it is rev1.0 disable the CC switch
		CCline_SwithSelection(1);  // CC1 switch enable

	break;
	case 0x03://Ra on Comm.CC
#if 0
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
		ActiveCClineEloadBufFill();
		Fx3I2CTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);//Writing
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
		MsgTimerStart(PDSS_ActiveCCDetect, TIMER0);
#endif
		if((gFunctStruct_t->gPD_Struct.gActiveCC == 0x01)&&(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == 0x01))//CC2 is active, So enable CC1 Switch
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = 1;
			CyU3PGpioSetValue (GPIO41_CC1_RA, 1);
			CyU3PGpioSetValue (GPIO42_CC2_RA, 0);
			if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)  //check for board revision if it is rev1.0 disable the CC switch
			CCline_SwithSelection(2); //CC2 switch enable
		}
		else
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = 2;
			CyU3PGpioSetValue (GPIO41_CC1_RA, 0);
			CyU3PGpioSetValue (GPIO42_CC2_RA, 1);
			if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)   //check for board revision if it is rev1.0 disable the CC switch
			CCline_SwithSelection(1);  // CC1 switch enable
		}
	break;
	case 0x04: // RA on both CC's
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue (GPIO41_CC1_RA, 1);
		CyU3PGpioSetValue (GPIO42_CC2_RA, 1);
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)   //check for board revision if it is rev1.0 disable the CC switch
		CCline_SwithSelection(0);   //Both the CC are disable
		break;

	default://default Ra on CC2
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;

		CyU3PGpioSetValue (GPIO41_CC1_RA, 0);
		CyU3PGpioSetValue (GPIO42_CC2_RA, 1);
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision != 0x01)   //check for board revision if it is rev1.0 disable the CC switch
		CCline_SwithSelection(1);  // CC1 switch enable
	break;
	}
}

/**
 * @brief Eload Vconn Switch Selection based on Non-Active CC if aVar is 0x03 else handling as per API
 * @param aVar Parameter for Eload Vconn Switch selection
 * @author Pranay
 * @date 30Aug'19
 */
void EloadVconnSelection(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEloadVconnSelection = aVar;
	switch(aVar)
	{
	case 0x00://disable both
		CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 0);
		CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 0);
		break;
	case 0x01://Ra on CC1 so enable cc1 Switch,CC2 is Active
		CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 0);
		CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 1);
		break;
	case 0x02://Ra on CC2 so enable CC2 Switch,CC1 is Active
		CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 1);
		CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 0);
		break;
	case 0x03://Based on Active CC
#if 0
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
		ActiveCClineEloadBufFill();
		Fx3I2CTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);//Writing
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
		MsgTimerStart(Eload_ActiveCCDetectAPI, TIMER0);
#endif
		if((gFunctStruct_t->gPD_Struct.gActiveCC == 0x01)&&(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == 0x01))//CC2 is active, So enable CC1 Switch
		{
			CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 0);
			CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 1);
		}
		else
		{
			CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 1);
			CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 0);
		}
		break;
	default:
		break;
	}
}

/**
 * @brief Buffer filling for Retrieving the FW version from CCG3PA
 * @author Pranay
 */
void CCG3PA_FWVersionBufFill()
{
	memset(greadConfigBuf, 0, 24);
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x01;
	greadConfigBuf[4]=0x01;
}

/**
 * @brief Functional Card Slot Detection B_ID3 (MSB) : B_ID1 (LSB)
 * @brief Will be called after the boot-up so that and tracked in "gSystemID" and will be used for validating the API received for respective card
 * @return CyU3PReturnStatus_t
 * 	CY_U3P_SUCCESS if able to read the GPIO's
 *
 */
CyU3PReturnStatus_t BoardIDDetection()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	CyU3PReturnStatus_t retVal=0;
	CyBool_t lVal_26 = 0,lVal_27 = 0,lVal_28 = 0,lVal_29 = 0;
	uint8_t lSlotID = 0;

	retVal = CyU3PGpioSimpleGetValue(GPIO26_B_ID0,&lVal_26);
	if(retVal != CY_U3P_SUCCESS)
	{
	return retVal;
	}
	retVal = CyU3PGpioSimpleGetValue(GPIO27_B_ID1,&lVal_27);
	if(retVal != CY_U3P_SUCCESS)
	{
	return retVal;
	}
	retVal = CyU3PGpioSimpleGetValue(GPIO28_B_ID2,&lVal_28);
	if(retVal != CY_U3P_SUCCESS)
	{
	return retVal;
	}
	retVal = CyU3PGpioSimpleGetValue(GPIO29_B_ID3,&lVal_29);
	if(retVal != CY_U3P_SUCCESS)
	{
	return retVal;
	}
	lSlotID = ((lVal_29<<3) | (lVal_28<<2) | (lVal_27<<1) | (lVal_26));

    gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID = lSlotID;
    return CY_U3P_SUCCESS;

}

void FetchSrcCapsExtndBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);
	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0x04;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);

}
/**
 * @brief Buffer filling used for Fetching uVDM response VDOs receied from DUT
 * @author Pranay
 * @date 15July'2020
 */
void FetchuVDMDataBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xF6;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);

}

void FetchEventLogBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xAA;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}
void FetchSop1RxPktBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xF8;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

void FetchDpDmBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xF9;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

void FetchPDSSVbusBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xF2;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}
void FetchStatusMsgInfoBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xB2;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}
void FetchBattStatusInfoBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xB3;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

/**
 * @brief Buffer used for fetching curretn port Data role, power role, PDC status from ccg3pa
 *  @author Pranay
 *  @date 15ul'2020
 */
void CCGxPortRoleFetchBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0xF7;

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);

}
/**
 * @brief Filling lreadConfigBuf Buffer for Reading capabilities from CCG3PA
 * @author Pranay
 * @date 30Jun'19
 */

void ReadSrcCapBufFill(void)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 24);
	greadConfigBuf[0]=0x00;
	greadConfigBuf[1]=0x17;
	greadConfigBuf[2]=0x02;
	greadConfigBuf[3]=0x05;
	greadConfigBuf[4]=0x01;
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

/**
 * @brief If  for API is for reading the Data from CCG3PA we need to pass no.of bytes along with header by appending the received API which is handled here
 * @return Apilength in no.of Bytes to be transfered to the control card
 * @author Pranay
 */
uint8_t PDSSCaps_DataAppend(uint8_t *aTXBuffer, uint8_t *aRXBuffer)
{

	uint8_t lTotalReceivedPayloadLength = 0, lTotal_I2C_TransmittedBytes = 0, l_I2C_PayloadLengthTansmitted = 0, lTotal_RS485_Payloadlength = 0,
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	lTotalReceivedPayloadLength = aRXBuffer[2];
	lTotal_I2C_TransmittedBytes = (aTXBuffer[1] + HEADER_BYTECOUNT);//Copying the API payload length +2bytes Header received from RS485 in to local Variable lRxApiLength
	l_I2C_PayloadLengthTansmitted = aTXBuffer[1];
	lTotal_RS485_Payloadlength = l_I2C_PayloadLengthTansmitted + lTotalReceivedPayloadLength;
	aTXBuffer[1] = lTotal_RS485_Payloadlength;

	CyU3PMemCopy (&aTXBuffer[lTotal_I2C_TransmittedBytes], &aRXBuffer[0],BUF_SIZE_256BYTE);

	return (lTotal_RS485_Payloadlength + HEADER_BYTECOUNT);//Header + total payload

}
uint8_t StatusInfoBattCapsDataAppend(uint8_t *aTXBuffer, uint8_t *aRXBuffer)
{
	uint8_t lTxBufByteCnt=0, lRxBufPayloadByteCnt =0 ;
	lTxBufByteCnt = aTXBuffer[1];

	lRxBufPayloadByteCnt = aRXBuffer[2] - CCG3PA_HDR_BYTECNT;//CCG3pA Header Byte count
	gBattStatusBuf[0] = lRxBufPayloadByteCnt;
	CyU3PMemCopy (&aTXBuffer[ (lTxBufByteCnt + HEADER_BYTECOUNT)], &aRXBuffer[CCG3PA_HDR_BYTECNT], lRxBufPayloadByteCnt);//Copying Actual payload data in to polling buffer
	CyU3PMemCopy(&gBattStatusBuf[1], &aRXBuffer[CCG3PA_HDR_BYTECNT], lRxBufPayloadByteCnt);//Copying received data into global buffer for further using

//	DEBUG_LOG(DBG2, 0xA1, (gBattStatusBuf[0] | (gBattStatusBuf[1] << 8) | (gBattStatusBuf[2] << 16) | (gBattStatusBuf[3] << 24)));

	aTXBuffer[1] = lTxBufByteCnt + lRxBufPayloadByteCnt;
	return (aTXBuffer[1] + HEADER_BYTECOUNT);//Header + total payload
}
/**
 * @brief This function will validate the return data received from the boot loader and verify whether program row is success or not
 * @param aBuffer is received from the Boot loader via I2C
 */
void Validate_CCG3PA_Program(uint8_t *aBuffer)
{
	if((aBuffer[0] == 0x01) && (aBuffer[1] != 0x00))
	{
		/*CCG3PA Program Row failed, Turn ON Data Error LED to indicate Program failed */
		DataErrorLEDIndicator(1);
	}
}
/**
 * @brief Function to send data to CCG3PA Boot loader to update the CCG3PA FW using I2C Boot loader
 * @param aBuffer is the data buffer received from RS485
 * @author Prasanna
 */
void CCG3PA_Program_Send(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lApiLength = gRS485RxBuf[1], lswitch = 0;

//	gRS485RxBuf[lApiLength+2] = CCG3PA_CHKSUM;/**Code word for CCg3pa to indicate whether All bytes has been received or not,CCG3pa(used incase of Request) will validate for this keyword and will perform respective action if not will inititate VconnSwap*/
//	gI2CTxBuf[0] = 0x00;
	CyU3PMemCopy (&gI2CTxBuf[0], &gRS485RxBuf[3],BUF_SIZE_255BYTE);
	lApiLength = lApiLength - 1;
	lswitch = gI2CTxBuf[1];
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,lApiLength,gI2CTxBuf,0);

#ifdef CCG3PA_FW_UPDATE_CHECK
	switch(lswitch)
	{
	case 0x37://Send Data to CCG3PA Boot loader For Sanity check read 7 Bytes if required

		break;
	case 0x39://Program Row

		MsgTimerStart(Timer_CCG3PA_Pgm_I2C_Read, TIMER1);

		break;

	case 0x38://Enter Boot loader Mode For Sanity check read 15 Bytes if required

		break;

	default:

		break;
	}
#endif
}

/**
 * @brief sending data fetched from RS485 to CCG3PA using FX3 I2C lines and handling based on interrupt received from CCg3pa/Control card
 * @param aBuffer contains the data/API received from control card
 * @author Pranay
 */

void CcgI2cdataTransfer(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lApiLength = 0;
	switch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType)
	{
		case GPIO0_INT_TO_FX3://got a interrupt from CCg3PA so should now send APi so that CCg3PA will copy data into I2cBuf than go and read.

				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
				PDSSInterrupt_Validation_Handler();
				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
				MsgTimerStart(PDSSInterrupt_Validation, TIMER0);

			break;
		case GPIO21_RS485_IRQ:
			lApiLength = aBuffer[1];
			PDSSInterruptGPIOHandler(INTR_CLR);
			aBuffer[lApiLength+2] = CCG3PA_WR_CHKSUM;/**Code word for CCg3pa to indicate whether All bytes has been received or not,CCG3pa(used incase of Request) will validate for this keyword and will perform respective action if not will inititate VconnSwap*/
			gI2CTxBuf[0] = 0x00;/**0th byte will be neglected by ccg3pa when data is received from i2c,so should prepend )th byte with 0 and append actual data*/
			CyU3PMemCopy (&gI2CTxBuf[1], &aBuffer[0],lApiLength+4);
			lApiLength = ((aBuffer[1] + HEADER_BYTECOUNT) + 4);

			if((aBuffer[0] & 0x07) == 0x07)/**If READ*/
			{
				gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
				CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,gI2CTxBuf,WRITE);
				MsgTimerStart(Timer_tPdGetCapability, TIMER0);
			}
			else/**If write*/
			{
				CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,lApiLength,gI2CTxBuf,WRITE);
			}
			PDSSInterruptGPIOHandler(INTR_SET);
			break;
		default:

			break;
	}
}
/***
 * Function used to toggle the GPIO that generates interrupt to ccg3pa
 * Interrupt is active low i.e. whenever this gpio goes LOW from HIGH ans interrupt will be generated in ccg3pa.
 * Once interrupt is generated in ccg3pa I2C read happens, So it is recommended to write required data and than generate interrupt.
 * @param aVar True or 0x01 :: Make GPIO high before writing anydata
 * 				False or 0x00 :: MAke Low to generate interrupt after writing data
 */
void PDSSInterruptGPIOHandler(CyBool_t aVar)
{
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, aVar);//Making High/LOW based on parameter passed (DQ1) GpIO1 for CCG3PA to handle the interrupt
}

/**
 * Function that fills the buffer to be written to ccg3pa with checksum and SOP and sends it to ccg3pa
 * Pranay,15March'2020.
 */
void PDSSDataReadHandler(uint8_t *aBuffer)
{

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	uint8_t lApiLength = aBuffer[1];

	aBuffer[lApiLength+HEADER_BYTECOUNT] = CCG3PA_WR_CHKSUM;/**Code word for CCg3pa to indicate whether All bytes has been received or not,CCG3pa(used incase of Request) will validate for this keyword and will perform respective action if not will inititate VconnSwap*/

	PDSSInterruptGPIOHandler(INTR_CLR);

	gI2CTxBuf[0] = 0x00;

	CyU3PMemCopy (&gI2CTxBuf[1], &aBuffer[0],(lApiLength+ HEADER_BYTECOUNT + 4) );

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,(lApiLength+HEADER_BYTECOUNT),gI2CTxBuf,WRITE);

	PDSSInterruptGPIOHandler(INTR_SET);

}
/**
 * Pranay,15MArch'2020.
 * Fucntion is used to fill active cc into an buffer and send it to application
 * @param *aBuffer API received from Application
 */
void ActiveCCStatusFetch(uint8_t *aBuffer)
{
	uint8_t lIndex,lDataLength,lTxIndex = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gRS485TxBuf, 0, BUF_SIZE_255BYTE);
	gRS485TxBuf[lTxIndex++] = gFunctStruct_t->gPD_Struct.gActiveCC;

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength + HEADER_BYTECOUNT;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gRS485TxBuf[0],BUF_SIZE_256BYTE);
	CyU3PBusyWait (100);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+HEADER_BYTECOUNT));
}

/**
 * @brief When BC1.2 DUT is detected and interrupt occurred, Fx3 will fetch and store DUT's BC FSM_State and BC Current mode and
 * @brief that variables will be streamed back to the Application.
 * @note Ensure that before sending this API BC1.2 DUT is connected and D+/D- line are connected to CCG3PA
 * @note If we un-comment the commented section BC1.2 details will be fetched in live from CCg3pA and will be streamed, so based on requirement shall handle this
 * @author Pranay
 * @date 20Sept'19
 */
void BC12DataReadHandler()
{

	uint8_t lIndex,lDataLength,lTxIndex = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gRS485TxBuf, 0, BUF_SIZE_255BYTE);

/*//commented section shall be used for fetching BC1.2 DUT status lively by reading from CCG3PA

	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
	BC12_ValidationBufFill();
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
	CyU3PBusyWait (10000);//Delay for ensuring enough time for BC1.2 Data has written in to I2C buff.
   	memset(gFunctStruct_t->gDataBuf_t.i2cRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,128,gFunctStruct_t->gDataBuf_t.i2cRxBuf,READ);//i2C data read from CCG3PA
	BC12_InterruptHandle(gFunctStruct_t->gDataBuf_t.i2cRxBuf);

*/
   	gRS485TxBuf[lTxIndex++] =  gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_FSMState;
   	gRS485TxBuf[lTxIndex++] =  gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_CurMode;

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gRS485TxBuf[0],BUF_SIZE_256BYTE);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+HEADER_BYTECOUNT));
}

/**
 * If interrupt is from CCG3pa and is related to BC1.2 drawing the Eload based on the DUT type
 * @param aBuffer buffer values consists of DUT type and Cur mode
 * @author Pranay
 */
void BC12_InterruptHandle(uint8_t *aBuffer)
{
#if 0/** gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_FSMState -- ||*/
    BC_FSM_OFF = 0,                                     /**< BC state machine inactive. */
	BC_FSM_SINK_START = 9,                                  /**< BC sink state machine start state. */
    BC_FSM_SINK_APPLE_CHARGER_DETECT = 10,                   /**< Sink looking for an Apple charger. */
    BC_FSM_SINK_APPLE_BRICK_ID_DETECT = 11,                  /**< Sink identified Apple charger, identifying brick ID. */
    BC_FSM_SINK_PRIMARY_CHARGER_DETECT = 12,                 /**< BC 1.2 primary charger detect state. */
    BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED = 13,           /**< BC 1.2 src detection failed, connected as Type-C sink. */
    BC_FSM_SINK_SECONDARY_CHARGER_DETECT = 14,               /**< BC 1.2 secondary charger detect state. */
    BC_FSM_SINK_DCP_CONNECTED = 15,                          /**< Sink connected to a BC 1.2 DCP. */
    BC_FSM_SINK_SDP_CONNECTED = 16,                          /**< Sink connected to a Standard Downstream Port (SDP). */
    BC_FSM_SINK_CDP_CONNECTED = 17,                          /**< Sink connected to a BC 1.2 CDP. */
#endif

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_FSMState  =	aBuffer[3];
	gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_CurMode	=	aBuffer[7];

	//PDSS_StatusLEDHandle(2);
	//DpDmSwitchHandle(1);

	if(gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_CurMode == BC_CHARGE_DCP)
	{
		//Eload of 1.5A
		gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_MaxCurrent = 0x5DC;
	}
	else if(gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_CurMode == BC_CHARGE_CDP)
	{
		//Eload of 500mA
		gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_MaxCurrent = 0x1F4;
	}
	else
	{
		//else 100mA
		gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_MaxCurrent = 0x64;
	}

	if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.DefaultEloadingFlag)
		HandleEload(Eload_BC12,0);

}
/**
 * @brief This function will calculate the % of max current to be drawn based on the % value given by user
 * @param aMaxCurrentReq :: it is the value of max current in latest PDC fetched by Fcard
 * @param aPercntEloadSetting :: % eload value given by user
 * @return return calculated value
 */
uint16_t percentageCurrentDrawingCalc(uint16_t aMaxCurrentReq, uint16_t aPercntEloadSetting)
{
	return (( aMaxCurrentReq / 100 ) * aPercntEloadSetting );
}
/**
 * @brief If interrupt is from CCG3pa and is related to PD Subsystem, drawing the Eload based and handling switches respectively
 * Decode the PDC details in aBuffer and Configure Eload accordingly
 * @param aBuffer buffer values consists of DUT type and Cur mode
 * @author Pranay
 */
void PDSS_InterruptHandler(uint8_t *aBuffer)
{
/*
 * Decode the PDC details in aBuffer and Configure Eload accordingly
 * */
	CyBool_t lIsVbusPresent = 0;
	uint8_t Bufindex=0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	++Bufindex;++Bufindex,++Bufindex;//Neglecting the keywords(Index0 & Index1)and Bytelength(Index2)
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus = (aBuffer[Bufindex] & 0x01 );
//	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev = ((aBuffer[Bufindex] >> 1) & 0x03);
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role = ((aBuffer[Bufindex] & 0x03) >> 1);
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gAttached_dev_type = ((aBuffer[Bufindex++] & 0xF0) >> 4);
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOIndex =  (aBuffer[Bufindex] & 0x0F);/**PDO index in lower nibble*/
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOSupplyType = ((aBuffer[Bufindex++] & 0xF0) >> 4);/**Supply type in upper nibble*/
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr = (aBuffer[Bufindex++] | (aBuffer[Bufindex++]<<8));//10mA p.u
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_MaxVolt = (aBuffer[Bufindex++] | (aBuffer[Bufindex++]<<8));//1mv p.u
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_MinVolt = (aBuffer[Bufindex++] | (aBuffer[Bufindex++]<<8));//1mV p.u
	gFunctStruct_t->gPD_Struct.gActiveCC = (aBuffer[13] & 0x01);//0:CC1,1:CC2
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev = ((aBuffer[13] >> 1) & 0x03);

	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev != SPECREV_3_0)
	{
		memset(gBattStatusBuf,0x00,32);//For 2.0 Spec rev DUTs there is no GeBattStatus msgs so resetting buffer incase of 2.0 DUTs, making it not to send old data
	}
	else
	{
		gFunctStruct_t->gPD_Struct.gPollingIterCnt = POLLING_ITER_BATTCAP_INIT_COUNT;//Pranay,29Sept'22, Inorder to reset and initiate GetBatteryCaps everytime after PDC

	}

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = ATTACH;

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr *= 10;//Converting to 1mA P.U

	if((gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch == CyFalse) &&
				(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.DefaultEloadingFlag) && (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus)
		)
	{
		/**Incase if Auto Eload is set and the values are 0 or 100, draw max current and incase of APDO draw 150mA less*/
		if((gFunctStruct_t->gFwHandle_t.gSystemInfo_t.AutoEloadPercntDraw != 0 ) && (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.AutoEloadPercntDraw != 100))
		{
			gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr = percentageCurrentDrawingCalc(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr,
																								gFunctStruct_t->gFwHandle_t.gSystemInfo_t.AutoEloadPercntDraw);
		}
		else /**if % to be drawn is 0 or 100*/
		{
			if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOSupplyType == PDO_AUGMENTED)
				gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr -= 150;
		}

		HandleEload(Eload_Gen,gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOIndex);

	}
	/**If capability mismatch interrupt comes adjust eload to the new capabilities, if last req.PDO is APDO draw 150mA less than requested current*/
	else if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch &&
				gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus &&
				gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapmismatchEloadHandle )
	{

			if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOSupplyType == PDO_AUGMENTED)
				gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr -= 150;

			HandleEload(Eload_Gen,gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOIndex);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapmismatchEloadHandle = CyFalse;
			gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch = CyFalse;

	}
	CyU3PGpioGetValue (GPIO44_VBUS_VBUS_DETECT, &lIsVbusPresent);

	if((gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == 0x01) && (lIsVbusPresent))//Based on Active CC handling the USB 3.0(Tx.Rx) lines.
	{
		if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.IsPDCdone == CyFalse)
		{
			gFunctStruct_t->gPDCStatus.gPDCStatus_t.IsPDCdone = CyTrue;
//			CyU3PConnectState(CyFalse, CyTrue);//disconnect USB
			if(gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBConnectTimer == CyTrue)// If
				TxRxSwitchHandler(gFunctStruct_t->gPD_Struct.gActiveCC);
			PDNegLedIndication(1);
			PDSS_StatusLEDHandle(1);
	 		CyU3PConnectState(CyTrue, CyTrue);					//After getting active CC state Connecting data lines
		}
	}
	else if((gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == 0x00) && (lIsVbusPresent))
	{
 		CyU3PConnectState(CyTrue, CyTrue);					//If there is no PDC make sure that the USB is connected.
	}
}

/**
 *
 * @brief After getting the interrupt from CCg3pa or During the testcase execution if needed to fetch PDC details this function is used.
 * @author Pranay
 */
void PDSSInterrupt_Validation_Handler()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 8);

		greadConfigBuf[0] = 0x00;
		greadConfigBuf[1] = 0x17;
		greadConfigBuf[2] = 0x02;
		greadConfigBuf[3]= 0x05;
		greadConfigBuf[4]= 0xF4;//For Interrupt validation

	//	Fx3I2CTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);//Writing
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

void PpsVbusZeroIOHandler(CyBool_t aCtrl )
{
//	CyU3PGpioSetValue (GPIO33_DETACH_INTR_TO_PPS, aCtrl);

//
//	gPPSi2cTxBuf[0] = 0x01;
//	gPPSi2cTxBuf[1] = 0x0A;
//	gPPSi2cTxBuf[2] = 0x05;
//	gPPSi2cTxBuf[3] = 0x00;
//	gPPSi2cTxBuf[4] = 0x00;
//	memset(&gPPSi2cTxBuf[5], 0x00, 11);
//	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
}

void AttachStateIntrHandler(uint8_t * aBuffer)
{
	uint16_t lVbusVal = 0;
	gPPSi2cTxBuf[0] = 0x01;
	gPPSi2cTxBuf[1] = 0x0A;
	gPPSi2cTxBuf[2] = 0x05;

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.grole_at_Connect = ((aBuffer[6] & 0b1100) >> 2) ;
//	gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = ATTACH;
	/**Decode voltage and add offset if needed incase of No Vltage Feedback*/
	lVbusVal = (aBuffer[7] | (aBuffer[8] << 8));
//	lVbusVal += 1200;
	gPPSi2cTxBuf[3] = lVbusVal;
	gPPSi2cTxBuf[4] = (lVbusVal >> 8);
	/**Decode requested max current and push it to PPS for Current limit DAC Configuration*/
	gPPSi2cTxBuf[5] = aBuffer[9];
	gPPSi2cTxBuf[6] = aBuffer[10];

	memset(&gPPSi2cTxBuf[7], 0x00, 9);
	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
//	DEBUG_LOG(DBG2, 0xC1, (aBuffer[3] | aBuffer[4] << 8 | aBuffer[5] << 16 | aBuffer[6] << 24));
//	DEBUG_LOG(DBG2, 0xC2, (aBuffer[7] | aBuffer[8] << 8 | aBuffer[9] << 16 | aBuffer[10] << 24));
}
void ReqStateIntrHandler(uint8_t * aBuffer)
{
	uint16_t lVbusVal = 0;
	gPPSi2cTxBuf[0] = 0x01;
	gPPSi2cTxBuf[1] = 0x0A;
	gPPSi2cTxBuf[2] = 0x05;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev = ((aBuffer[5] >> 1) & 0x03);
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role = (aBuffer[6] & 0b0011);
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gAttached_dev_type = ((aBuffer[6] & 0xF0) >> 4);

	/**Decode voltage and add offset if needed incase of No Vltage Feedback*/
	lVbusVal = (aBuffer[7] | (aBuffer[8] << 8));
//	lVbusVal += 1200;
	gPPSi2cTxBuf[3] = lVbusVal;
	gPPSi2cTxBuf[4] = (lVbusVal >> 8);
	/**Decode requested max current and push it to PPS for Current limit DAC Configuration*/
	gPPSi2cTxBuf[5] = aBuffer[9];
	gPPSi2cTxBuf[6] = aBuffer[10];
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent = (aBuffer[9] | (aBuffer[10] << 8) );

	/**
	 * 	Pranay,25Aug'22, Modifying as per spec,
	 * 	iPpsCLNew -- Current Limit accuracy Section -- 7.1.4.4
	 *  if 1A <= Operating Current <= 3A  ------   +-150mA
	 *  for Operating currents > 3A 		 ------ 	+- 5%
	 *  So for Operating currents < 1000mA no need of OCP,
	 */

	if( (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent > 1000) && (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent <= 3000))
	{
//		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit = (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent + 150);
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit = ((gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent * OCP_LIMIT_PERCENTAGE) / 100);
	}
	else if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent <= 1000)
	{
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit = 1000;
	}
	else if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent > 3000)
	{
//		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit = ((gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent * OCP_LIMIT_3A_GREATER_P_I) / 100);
		gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit = ((gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrent * OCP_LIMIT_PERCENTAGE) / 100);

	}
	memset(&gPPSi2cTxBuf[7], 0x00, 9);

	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
//	DEBUG_LOG(DBG2, 0xC3, (aBuffer[3] | aBuffer[4] << 8 | aBuffer[5] << 16 | aBuffer[6] << 24));
//	DEBUG_LOG(DBG2, 0xC4, (aBuffer[7] | aBuffer[8] << 8 | aBuffer[9] << 16 | aBuffer[10] << 24));
}
void DetachStateVbusHandler()
{
	gPPSi2cTxBuf[0] = 0x01;
	gPPSi2cTxBuf[1] = 0x0A;
	gPPSi2cTxBuf[2] = 0x05;
	gPPSi2cTxBuf[3] = 0x00;
	gPPSi2cTxBuf[4] = 0x00;

//	gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = DETACH;

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role = PRT_ROLE_SINK;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gAttached_dev_type = 0x00;

	memset(&gPPSi2cTxBuf[6], 0x00, 10);
	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
//	DEBUG_LOG(DBG2, 0xCE, 0);
}
void CapsMisMatchHandler()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	/**Pranay,15Sept'2020, If Eload was running earlier than we should turn off running eload and after PS_RDY need to turn on Eload, so "CapmismatchEloadHandle" variable will be used for that**/
	if((gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus != Eload_TurnOFF ) &&
			(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus != Eload_BootUpSync))
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapmismatchEloadHandle = CyTrue;

	HandleEload(Eload_TurnOFF,0);

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch = CyTrue;

	/**Pranay,16Dec'20, making the cap mismatch status info flag to true only when cap mismacth occures and making it false only after SW reading it*/
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag = CyTrue;
}
/**
 * @brief After getting the interrupt from CCg3pa if needed to fetch DUT details this function is used.
 * @author Pranay
 */
void BC12_ValidationBufFill()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 8);

	greadConfigBuf[0] = 0x00;
	greadConfigBuf[1] = 0x17;
	greadConfigBuf[2] = 0x03;
	greadConfigBuf[3]= 0x02;
	greadConfigBuf[4]= 0x03;
	greadConfigBuf[5]= 0x05;

//	Fx3I2CTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);//Writing
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

/**
 * @brief Configuring the buffer for receiving PDC details from CCg3PA
 * @author Pranay
 * @date 22Jult'19
 */
void PDC_Validation()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 8);

	greadConfigBuf[0] = 0x00;
	greadConfigBuf[1] = 0x17;
	greadConfigBuf[2] = 0x02;
	greadConfigBuf[3] = 0x05;
	greadConfigBuf[4] = 0xF1;
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
}

/**
 * @brief FX3 Fw version Fetching
 * @author Pranay
 */
void FX3_FWVersion_Fetch()
{
	CyU3PMemCopy (gI2CRxBuf,glFirmwareID,6);
	gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex++] = 0xA2; //Keyword
	CyU3PMemCopy (&gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex],gI2CRxBuf,6);
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex += 6;
	gRS485RxBuf[1] += 7;
}

/**
 * @brief CCG3PA FE Version fetching
 * @author Pranay
 */
void CCG3PA_FWVersion_Fetch()
{
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
	CCG3PA_FWVersionBufFill();
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,11,gI2CRxBuf,READ);
	CyU3PMemCopy (&gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex],&gI2CRxBuf[1],9);
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex += 9;
	gRS485RxBuf[1] += 9;
}
/**
 * @brief Eload FW version fetch, Last 2 bytes from the data read will be the FW version
 * @author Pranay
 */
void ELoad_FWVersion_Fetch()
{
	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,gI2CRxBuf,READ);
	gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex++] = 0xA5;
	gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex++] = (gI2CRxBuf[15] + 0x30); //adding 30 to convert to ASCII
	gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex++] = '.';
	gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex] = (gI2CRxBuf[16] + 0x30);  //adding 30 to convert to ASCII
	gRS485RxBuf[1] += 4;
}
 /**
  * @brief Handling FW version fetching and streaing back to Application based on the API received
  * @param aBuffer bytes received from API will be in this buffer
  */
void FWVersion_Fetch(uint8_t *aBuffer)
{
	uint8_t lDataLength = 0;
	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex = 0;
	lDataLength = gRS485RxBuf[1];
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex = lDataLength+2;
		switch(aBuffer[3])
		{
		case(0x02)://Fx3
			FX3_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex);
		break;
		case(0x03)://CCG3PA
				//Keyword (A3) will be fetched from CCG3PA
			CCG3PA_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex );
		break;
		case(0x04)://CCG4
				//Should Add Keyword (A4)
				/*TBD*/
		break;
		case(0x05)://E-Load
			ELoad_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex);
		break;
		case (0x06)://FW update status
				/**After updating the FW, Control card will request for Keyword,So if FW is updated will Function card responds with 0xA1 else if in Boot stage responds with 0xA0*/
				gRS485RxBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex++] = 0xA1; /** Keyword */
				gRS485RxBuf[1] += 1;
				FX3_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex); /** FW version will also be streamed back to Application */
		break;

		case(0x0F)://All Device FW
			FX3_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex);
			CCG3PA_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex );
			ELoad_FWVersion_Fetch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFWWriteIndex );
			break;
		}
	RS485TXDataHandler(gRS485RxBuf, ((gRS485RxBuf[1]+2)));
}
/**
 * @brief This Function is used for the Data Fetching from Control in Endpoint based on the 3rd Index of aBuffer
 * @param aBuffer buffer received from Control in Endpoint
 * @date 11Oct'19
 */
void ControlEP_DataReadHandler(uint8_t *aBuffer)
{
	uint8_t lPayloadCount = gRS485RxBuf[1];
	uint8_t lByteCount = 0;
	switch(aBuffer[3])
	{
	case 0x01://System ID fetch
		CyU3PMemSet(gI2CRxBuf, 0x00, BUF_SIZE_256BYTE);

		gI2CRxBuf[lByteCount++] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID;//Fetching only present System/Card ID

		gRS485RxBuf[1] = lByteCount + lPayloadCount;
		CyU3PMemCopy (&gRS485RxBuf[lPayloadCount+2], &gI2CRxBuf[0],lByteCount);

		CyU3PMemSet(&gRS485TxBuf[0], 0x00, (BUF_SIZE_256BYTE));
		CyU3PMemCopy (&gRS485TxBuf[0], &gRS485RxBuf[0],BUF_SIZE_256BYTE);

//		RS485TXDataHandler(gFunctStruct_t->gDataBuf_t.rs485RxBuf, (lPayloadCount + lByteCount + 2));
		break;
	default:
		break;
	}
}
/**
 * @brief This function used to get the i2c data from the Megachips, which is connected to the Functional card via I2C
 * @param aBuffer - It contains API command and the register address, No of bytes to be read.
 * @note In actual i2c read operation, we need to inform the slave that, from which register we need to get the data.
 * So first write operation is performed. Write the register Address and then send Read i2c with No. of bytes to be read.
 * In the function API 0x01, will handle internally writing the register address and read operation.
 * API 0x02, performs only read i2c instruction. (Provision is provided)
 * @warning It is preferred to use the API 0x01, Since it will internally handles the write reg address and read data.
 * @author Prasanna
 * @date 21/08/2019
 */
void GetMegachipsDevData(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	uint32_t lReadRegAddr = 0;
	uint16_t lByteCount = 0, lDataLen = 0, lIndex = 0;

	switch(aBuffer[3])
	{
	case 1:/**< Megachips I2C Register Write reg address and Read Data Operation, It will fetch data from the respective register*/
		lReadRegAddr = aBuffer[5] | aBuffer[4] << 8; /**< All our input from the App will be LSB first, If the actula Register address in Megachip App is "0504" means on i2c line we need to give as 0x04 0x05 It is handled here*/
		lByteCount = aBuffer[6];
		CyFxUsbI2cTransfer(lReadRegAddr, I2C_DEV_ID_MEGACHIP, lByteCount, gI2CRxBuf, READ);

		break;

	case 2:/**< Megachips I2C Register Data Read only Operation, It will fetch data from the respective register*/

		lReadRegAddr = aBuffer[5] | aBuffer[4] << 8;
		lByteCount = aBuffer[6];
		CyFxUsbI2cTransfer(lReadRegAddr, I2C_DEV_ID_MEGACHIP_RD, lByteCount, gI2CRxBuf, READ);

		break;

	default:

		break;
	}

	lDataLen = gRS485RxBuf[1];
	lIndex = lDataLen + 2;
	gRS485RxBuf[1] = lDataLen + lByteCount;
	CyU3PMemSet(&gRS485RxBuf[lIndex], 0x00, (BUF_SIZE_256BYTE - lIndex));
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gI2CRxBuf[0],lByteCount);
	CyU3PMemSet(&gRS485TxBuf[0], 0x00, (BUF_SIZE_256BYTE));
	CyU3PMemCopy (&gRS485TxBuf[0], &gRS485RxBuf[0],BUF_SIZE_256BYTE);

//	RS485TXDataHandler(gFunctStruct_t->gDataBuf_t.rs485RxBuf, (lIndex + lByteCount));	/**< At Present This API is provided only for COntrol EP Read< In Next version need to include for RS485 communication also */
}

/**
 * @brief This function used to write or set any inputs to Megachip i2c slave
 * @param aBuffer - It contains API command and the register address, No of bytes to be written and the write data
 * @note In this API 0x01, Two bytes register address is provided by the Application LSB - MSB, Byte count will be no of bytes to be written. and 4 bytes of register data
 * @author Prasanna
 * @date 21/08/2019
 */
void SetMegachipsDevData(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint32_t lReadRegAddr = 0;
	uint16_t lByteCount = 0;

	switch(aBuffer[4])
	{
	case 1:/**< Megachips I2C Register Write */

		lReadRegAddr = aBuffer[6] | aBuffer[5] << 8;/**< Megachips I2C Register Write reg address and Read Data Operation, It will fetch data from the respective register*/
		lByteCount = aBuffer[7] + 2;/**< Byte count to be written to the Megachip, Since register address also have two bytes that is added internally to the actual byte count of the data*/
		gI2CTxBuf[0] = lReadRegAddr >> 8;/**< LSB register address in megachip */
		gI2CTxBuf[1] = lReadRegAddr;/**< MSB register address in megachip */
		CyU3PMemCopy (&gI2CTxBuf[2], &aBuffer[8],aBuffer[7]);/**< Data to be written to the reg address LSB -- MSB*/
		CyFxUsbI2cTransfer(lReadRegAddr, I2C_DEV_ID_MEGACHIP, lByteCount, gI2CTxBuf, WRITE);

		break;

	}
}
/**
 * @brief This Fucntion resets the TImestamp timer to zero and starts again from zero
 * @author Pranay
 * @date 16March'21
 *
 */
void TimeStampTimerReset()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	MsgTimerStop(TIMER3);
	gFunctStruct_t->gPD_Struct.gElapsedTicks = 0;
	gFunctStruct_t->gPD_Struct.gMinutesCount = 0;

	MsgTimerStart(Timer_tSampleCheck, TIMER3);

}
/**
 * @brief This Function stops the Time stamp timer from running
 * @author Pranay
 * @date 16March'21
 */
void TimeStampTimerStop()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	MsgTimerStop(TIMER3);

}

/**
 * @brief This Function starts the Time stamp timer from running
 * @author Pranay
 * @date 16March'21
 */
void TimeStampTimerStart()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	gFunctStruct_t->gPD_Struct.gElapsedTicks = 0;
	gFunctStruct_t->gPD_Struct.gMinutesCount = 0;

	MsgTimerStart(Timer_tSampleCheck, TIMER3);

}

/**
 * @brief Writing the Received FRAM data in to the FRAM SLAVE
 * @note lIndex & lDatalength will be varied as per the API
 * @param aBuffer FRAM data Buffer/API received from Application
 * @author Pranay
 */
void FRAM_Write(uint8_t *aBuffer)
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	lDataLength = aBuffer[4];
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

		lDataLength = aBuffer[4];
		lIndex = (aBuffer[5] | (aBuffer[6]<<8));
		CyU3PMemCopy (&gI2CTxBuf[0], &aBuffer[7],(BUF_SIZE_256BYTE-8));
		CyFxUsbI2cTransfer(lIndex,FRAM_SLAVEADDR,lDataLength,gI2CTxBuf,WRITE);
}

/**
 * @brief Reading the FRAM Data and storing it in 'i2cRxBuf" as per Received API
 * @note lIndex & lDatalength will be varied as per the API
 * @param aBuffer FRAM data Buffer/API received from Application
 * @author Pranay
 */
void FRAM_Read(uint8_t *aBuffer)
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

		lIndex = (aBuffer[4] | (aBuffer[5]<<8));
		lDataLength = aBuffer[3];
		memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(lIndex,FRAM_SLAVEADDR,lDataLength,gI2CRxBuf,READ);
}

uint8_t GetPPSVbusValues()
{
	uint8_t lTxIndex = 0;
	uint8_t lPPSReadData[10] = {0};

	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,lPPSReadData,READ);//Only reading 17 bytes as said in API sheet
	CyU3PBusyWait(10000);//10mS delay
	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,lPPSReadData,READ);//Reading twice because of I2C issue in TI

	memset(gRS485TxBuf, 0, BUF_SIZE_280BYTE);

//	if(lPPSReadData[9] == 0xFE)
	{
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage = (lPPSReadData[0] | (lPPSReadData[1] << 8));

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent = (lPPSReadData[2] | (lPPSReadData[3] << 8));

		CyU3PMemCopy (&gRS485TxBuf[0], &lPPSReadData[0],10);
		lTxIndex = 10;

	}
	return lTxIndex;

}

/**
 * @brief Reading the Eload Data and storing those data in respective structure variables.
 */

uint8_t GetVbusStatusValues()
{
	uint8_t lTxIndex = 0;

	memset(gRS485TxBuf, 0, BUF_SIZE_255BYTE);

	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role != PRT_ROLE_SOURCE)
	{
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,greadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

		/** Vbus voltages streaming based on Slected Connector type*/
		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch == External_Connector)
		{
			gRS485TxBuf[lTxIndex++] = greadConfigBuf[3];
			gRS485TxBuf[lTxIndex++] = greadConfigBuf[4];

			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusVoltage = (greadConfigBuf[3] | (greadConfigBuf[4]<<8));
		}
		else if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch == TypeC_Connector)
		{
			gRS485TxBuf[lTxIndex++] = greadConfigBuf[5];
			gRS485TxBuf[lTxIndex++] = greadConfigBuf[6];

			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusVoltage = (greadConfigBuf[5] | (greadConfigBuf[6]<<8));
		}
		/** VbusCurrents */
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[7];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[8];

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusCurrent = (greadConfigBuf[7] | (greadConfigBuf[8]<<8));
	}
	else //If Source
	{
		CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,greadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

		/** Vbus Voltages*/
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[0];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[1];

		/** VbusCurrents */
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[2];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[3];

	}
		return lTxIndex;
}
/**
 * Pranay,25March'21, This function is used for fetching timestamp details explicitly(other than polling)
 * This function fills current timestamp and sends it back to control card
 */
void getCurrentTimestamp()
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *) GetStructPtr();

	lTxIndex = 4;

	getElapsedTimerTicks();

	/**Filling milli Seconds*/
	gRS485RxBuf[++lIndex] = gFunctStruct_t->gPD_Struct.gElapsedTicks & 0x00FF;
	gRS485RxBuf[++lIndex] = (gFunctStruct_t->gPD_Struct.gElapsedTicks & 0xFF00) >> 8;

	/**Filling minutes*/
	gRS485RxBuf[++lIndex] = (gFunctStruct_t->gPD_Struct.gMinutesCount & 0x00FF);
	gRS485RxBuf[++lIndex] = (gFunctStruct_t->gPD_Struct.gMinutesCount & 0xFF00) >> 8;



	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength + 2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;

	RS485TXDataHandler(gRS485RxBuf, (lDataLength + 2));

}
/**
 * This function fills Elapsed Timer count from the timer inception in mS
 * @return false if the timer is not active
 */
CyU3PReturnStatus_t getElapsedTimerTicks()
{
	CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;

	unsigned int  lisActive = 0;
	uint32_t lRemainingTicks = 0;

	retStatus = CyU3PTimeElapsed(&gMsgTimer3, &pTimername, &lisActive, &lRemainingTicks, 0, NULL );
//	retStatus = tx_timer_info_get(&gMsgTimer3, &pTimername, &lisActive, &lRemainingTicks, 0, NULL );

	gFunctStruct_t->gPD_Struct.gElapsedTicks = (MAX_RTOS_TIMER_TICKS_IN_MIN - lRemainingTicks); /** subtracting Max ticks from remaining ticks gives us Elapsed ticks*/

	return retStatus;


}

void GetPPSData(uint8_t *aBuffer)
{
	uint8_t lIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	lTxIndex = GetPPSVbusValues();

	lDataLength = aBuffer[1];
	lIndex = lDataLength+2;
	aBuffer[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&aBuffer[lIndex], &gRS485TxBuf[0],BUF_SIZE_20BYTE);
	RS485TXDataHandler(aBuffer, (lDataLength+2));
}
/**
 * @brief Reading the Eload Data and streaming it back to application
 */
void GetVbusData()
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(greadConfigBuf, 0, 24);
	lTxIndex = GetVbusStatusValues();

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gRS485TxBuf[0],lTxIndex);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}
/**
 * Function used to read Vconn values values from elaod if Port Verfication is in progress
 * read data from any CC based on the parameter passed irrespective of active CC.
 */
void GetPortVerfVconnStatusValues(CyBool_t CCPinToRead)
{
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,greadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

	memset(gRS485TxBuf, 0, BUF_SIZE_255BYTE);

	if(CCPinToRead == 0)/**if CC pin to read is CC1*/
	{
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[9];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[10];
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage = (greadConfigBuf[9] | (greadConfigBuf[10]<<8));
	}
	else/**if CC pin to read is CC2*/
	{
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[11];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[12];
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage = (greadConfigBuf[11] | (greadConfigBuf[12]<<8));
	}
	/** Vconn Currents (CC lines Current) */
	gRS485TxBuf[lTxIndex++] = greadConfigBuf[13];
	gRS485TxBuf[lTxIndex++] = greadConfigBuf[14];
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent = (greadConfigBuf[13] | (greadConfigBuf[14]<<8));

}
uint8_t GetVconnStatusValues()
{
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,greadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

	memset(gRS485TxBuf, 0, BUF_SIZE_255BYTE);

	/** Vconn Voltage streaming based on Active CC */
	if(gFunctStruct_t->gPD_Struct.gActiveCC == 0x01) /** CC2 is Active,So only CC1 Eload Streaming */
	{
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[9];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[10];
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage = (greadConfigBuf[9] | (greadConfigBuf[10]<<8));
	}
	else
	{
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[11];
		gRS485TxBuf[lTxIndex++] = greadConfigBuf[12];
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage = (greadConfigBuf[11] | (greadConfigBuf[12]<<8));
	}
	/** Vconn Currents (CC lines Current) */
	gRS485TxBuf[lTxIndex++] = greadConfigBuf[13];
	gRS485TxBuf[lTxIndex++] = greadConfigBuf[14];
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent = (greadConfigBuf[13] | (greadConfigBuf[14]<<8));

	return lTxIndex;
}

/**
 * @brief Reading the Eload Data and streaming it back to application.
 */
void GetVconnData()
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lTxIndex = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(greadConfigBuf, 0, 24);
	lTxIndex = GetVconnStatusValues();

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gRS485TxBuf[0],BUF_SIZE_256BYTE);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * @brief Reading the Eload Data and streaming it back to application
 * @note If requested for Vconn, fetching data will be based on Active CC, and if requested for Vbus fetching data will be based on VBus switch selection
 * Fetching and streaming the respective ADC values Vbus/Vconn/both voltages and currents both as per the API sheet Eload read format
 * @author Pranay
 */
void GetADC_Data()
{
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lTxIndex = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lReadConfigBuf[24] = {0};

	memset(gRS485TxBuf, 0, 255);

	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role != PRT_ROLE_SOURCE)
	{
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,lReadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch == External_Connector)
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[3];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[4];
		}
		else if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch == TypeC_Connector)
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[5];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[6];
		}
		//VbusCurrent
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[7];
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[8];

		if(gFunctStruct_t->gPD_Struct.gActiveCC == 0x01)//CC2 is Active,So only CC1 Eload Streaming
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[9];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[10];
		}
		else
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[11];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[12];
		}
		//CC lines Current
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[13];
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[14];
	}
	else // if source
	{
		CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,lReadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

		/** Vbus Voltages*/
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[0];
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[1];

		/** VbusCurrents */
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[2];
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[3];

		memset(lReadConfigBuf, 0, 24);
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,17,lReadConfigBuf,READ);//Only reading 17 bytes as said in API sheet

		if(gFunctStruct_t->gPD_Struct.gActiveCC == 0x01)//CC2 is Active,So only CC1 Eload Streaming
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[9];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[10];
		}
		else
		{
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[11];
			gRS485TxBuf[lTxIndex++] = lReadConfigBuf[12];
		}
		//CC lines Current
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[13];
		gRS485TxBuf[lTxIndex++] = lReadConfigBuf[14];
	}

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &gRS485TxBuf[0],BUF_SIZE_256BYTE);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 *@brief Configuring the Eload as per the API received
 *@brief but overwriting Byte 8.5 based on the Active CC
 *@note if by sending 0x01 0x02,0x02,0xF2, he enables Vconn selection based on Active CC(FW control)
 *@note than only in that case handle switch here based on active CC and byte 8.5 else send API directly to Eload
 *@author Pranay
 */
void SetADC_Data(uint8_t *aBuffer)
{
/*
* For Eload Writing, As per the API received,
* but overwriting Byte 8.5 based on the Active CC,
*
* if by sending 0x01 0x02,0x02,0xF2, he enables Vconn selection based on Active CC(FW control)
* than only in that case handle switch here based on active CC and byte 8.5 else send API directly to Eload
*/
	uint8_t lBuffer[32] = {0};
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEloadVconnSelection == 0x03)//Pranay,07Aug'19,need to Selection Vconn switch based on Active CC only when asked to select based on Active CC(was selecting by default on Active CC earlier)
		{
		if((gFunctStruct_t->gPD_Struct.gActiveCC == 0x01)&&(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == 0x01))//CC2 is active, So enable CC1 Switch
			EloadVconnSelection(0x01);
		else//CC1 is active, So enable CC2 Switch //if PDC is failed considering CC1 is active and Applying Eload on CC2
			EloadVconnSelection(0x02);

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEloadVconnSelection = 0x03;//assigning here back to 0x03
		aBuffer[8] |= (gFunctStruct_t->gPD_Struct.gActiveCC << 5);//overwriting bit5 based on active CC
		}
		CyU3PMemCopy (lBuffer, &aBuffer[4],BUF_SIZE_26BYTE);
		Config_Eload(Eload_Gen,lBuffer);
}

/**
 * @brief It will check for the Phy and Link Layer error counts, and will update the Present Error count and
 * as well Total No of Errors detected during Connect/Disconnect
 * @warning PhyErrCnt, LnkErrCnt will be the present connect or loopdata count, where as Total Phy & Link Error count
 * will be the total sum of error counts in total no of Connect events It can be reset When Application will read the Count or using Seperate API can be reset
 * Iteration count also will be reset after reading is done.
 * @return Error status
 */
CyBool_t CheckUsbErrStatus(uint8_t aVal)
{
	uint16_t lPhy_error_cnt = 0 , lLnk_error_cnt = 0, lUsb2errorCnt = 0;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	CyU3PUsbGetErrorCounts(&lPhy_error_cnt, &lLnk_error_cnt);
	GetUSB2ErrorCnt(&lUsb2errorCnt);

	gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt = lPhy_error_cnt;
	gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt = lLnk_error_cnt;
	gFunctStruct_t->gUsbErrStatus_t.gTotalPhyErrCnt += lPhy_error_cnt;
	gFunctStruct_t->gUsbErrStatus_t.gTotalLinkErrCnt += lLnk_error_cnt;
	gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = lUsb2errorCnt;
	gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt += lUsb2errorCnt;
	if(aVal)  //Check if the function is called from API or Connect event & read loopback status (True - increment the iteration count)
		gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt++;

	if((lPhy_error_cnt) || (lLnk_error_cnt) || (lUsb2errorCnt))/*If Error Count is Non-Zero*/
	{
		status = CyTrue;
	}
	else
	{
		status = CyFalse;
	}
	return status;
}
/**
 *	Function for Eload Packet configuration
 * @param aCodeIndex will be the requirement/usecase specific configuration of eload
 * @param aPDOIndex used only incase of Testcase execution for configuring eload with respective Currenst based on PDO index
 */
void HandleEload(uint8_t aCodeIndex,uint8_t aPDOIndex)
{
	uint8_t lBuffer[32] = {0};
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	switch(aCodeIndex)
	{
	case Eload_StandBy:
	case Eload_Gen:

		if(aCodeIndex == Eload_StandBy)
		{
			lBuffer[0] = 0x64;
			lBuffer[1] = 0x00;
		}
		else
		{
			lBuffer[0]	= gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr;		//[2]
			lBuffer[1]	= (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_CurPwr >> 8);	//[3]
		}
		/*4&5 :: Vconn Load value based on Mode*/
		lBuffer[2] = 0x00;//[4]
		lBuffer[3] = 0x00;//[5]
		if(gFunctStruct_t->gPD_Struct.gActiveCC)
		{
			//CC2 is ACtive
			lBuffer[4] = 0x20; //Setting b5 to 1	[6]
		}
		else//CC1 is Active
		{
			lBuffer[4] = 0x00; //Setting b5 to 0	[6]
		}
		lBuffer[4] |= 0x11;//Switch  b0: 1/0 :: Vbus Eload on/off ;		[6]\
									b1: 0/1 :: TypeC Vbus/EXt_Vbus;\
									b4: 1/0 VConn Eload ON/OFF ; \
									b5 0/1 :: CC1/CC2 is Vconn

		lBuffer[5] = 0x00;//Mode :: 0x00 Vbus[b1:0] & Vconn[b5:4] CC mode	[7]
	break;
	case Eload_BootUp_Init:
		lBuffer[0] = 0xB0;//Keyword
		lBuffer[1] = 0x01;//MsgID
		lBuffer[2] = 0x00;
		lBuffer[3] = 0x00;
		lBuffer[4] = 0x00;
		lBuffer[5] = 0x00;
		lBuffer[6] = 0x00;//Switch
		lBuffer[7] = 0x00;//Mode
		break;
	case Eload_BC12:
		lBuffer[0] = 0xAB;//Keyword
		lBuffer[1] = 0x01;//MsgID
		lBuffer[2] = gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_MaxCurrent;
		lBuffer[3] = (gFunctStruct_t->gPDCStatus.gBCStatus_t.BC_MaxCurrent >> 8);
		/*4&5 :: Vconn Load value based on Mode*/
		lBuffer[4] = 0x00;
		lBuffer[5] = 0x00;
		lBuffer[6] = 0x00;//Turning Off Vconn Eload incase of BC1.2 as No presence CC lines in BC1.2
		lBuffer[7] = 0x00;//Mode :: 0x00 Vbus[b1:0] & Vconn[b5:4] CC mode
		break;
	case Eload_TurnOFF:
		lBuffer[0] = 0xAB;//Keyword
		lBuffer[1] = 0x01;//MsgID
		lBuffer[2] = 0x00;
		lBuffer[3] = 0x00;
		/*4&5 :: Vconn Load value based on Mode*/
		lBuffer[4] = 0x00;
		lBuffer[5] = 0x00;
		lBuffer[6] = 0x00;//Turning Off Vconn Eload incase of BC1.2 as No presence CC lines in BC1.2
		lBuffer[7] = 0x00;//Mode :: 0x00 Vbus[b1:0] & Vconn[b5:4] CC mode
		break;
	}

	Config_Eload(aCodeIndex,lBuffer);
}

/**
 * @brief Convert 16bit Decimal value to ASCII
 * @param lSerial Serial Number of the Test Card
 */
void ConvertDeToAscii(uint8_t* lBuf, uint16_t lSerial)
{
	uint8_t iCnt = 5, lRem = 0, i = 4;

	for(iCnt = 0; iCnt < 5; iCnt++)
	{
		lRem = (lSerial % 10);
		lSerial = lSerial / 10;
		lBuf[i] = (0x30 + lRem);
		i--;
	}
}
/**
 * @brief Convert 16bit Decimal value to ASCII
 * @param lSerial Serial Number of the Test Card
 */
void ConvertDeToAsciiByte(uint8_t* lBuf, uint16_t lvalue, uint8_t lLen)
{
	uint8_t iCnt = 0, lRem = 0, i = (lLen - 1);

	for(iCnt = 0; iCnt < lLen; iCnt++)
	{
		lRem = (lvalue % 10);
		lvalue = lvalue / 10;
		lBuf[i] = (0x30 + lRem);
		i--;
	}
}

/**
 * @brief Get the Device Information details from FRAM, This Data contains Manufacturing year and the Serial Number.
 * This will be used to set the Seriala Number Descriptor
 * Test card serial number : FRAM Addr 0x08, 2 Bytes LSB, MSB
 * Test card Manufacturing year : FRAM Addr 0x46, 2 Bytes LSB, MSB
 * @note
 * @author Prasanna
 * @date 29Feb'2020
 */
void GetTestCardInfo()
{
	uint8_t lSerialNumAddr = 0x08;
	uint8_t lManufactureYearAddr = 0x46;
	uint16_t lTC_Serial_No = 0, lTC_Manufacture_Year = 0, lSys_Id = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(lSerialNumAddr,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);
	lTC_Serial_No = ((gI2CRxBuf[1] << 8) | gI2CRxBuf[0]);

	ConvertDeToAscii(&gTCSerialNumber[5], lTC_Serial_No);

	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(lManufactureYearAddr,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);
	lTC_Manufacture_Year = ((gI2CRxBuf[1] << 8) | gI2CRxBuf[0]);

	ConvertDeToAscii(&gTCSerialNumber[0], lTC_Manufacture_Year);

	lSys_Id = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID;

	ConvertDeToAsciiByte(&gTCSerialNumber[10], lSys_Id, 2);

	SetProductSerNumDesc(gTCSerialNumber);
}

/**
 * @brief This Function handles the reading of total BYte-count of FRAM data, which will be in [2]and [3] indexes
 * @note For the E-load bring up and Booting synchronization with FX3, so
 * @note From FX3 during its bootup time reading FRAM data and pushing it to E-load, 8bytes at a time in 20mS time interval by appending Code word(0xB0) and MSG ID(0x01)
 * @author Pranay
 * @date 09Sept'19
 */
void EloadFRAM_DataConfig()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress = 0;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteCount = 0;

	CyFxUsbI2cTransfer(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteCount = ((gI2CRxBuf[2]) | (gI2CRxBuf[3]<<8));
	EloadFRAM_DataWrite();
}
void TxPPSCalibDataCalcAPI()
{
	memset(gPPSi2cTxBuf, 0, BUF_SIZE_16BYTE);

	gPPSi2cTxBuf[0] = 0x0A;
	gPPSi2cTxBuf[1] = 0x02;//MSG ID

	CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,16,gPPSi2cTxBuf,WRITE);
}
/**
 * @brief This Function handles the reading of 8bytes data from FRAM by incrementing the "EloadCtrlFramByteAddress" by 8 bytes after every read operation
 * @note For the E-load bring up and Booting synchronization with FX3, so
 * @note From FX3 during its bootup time reading FRAM data and pushing it to E-load, 8bytes at a time in 20mS time interval by appending Code word(0xB0) and MSG ID(0x01)
 * @author Pranay
 * @date 09Sept'19
 */
void EloadFRAM_DataWrite()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gI2CRxBuf, 0, BUF_SIZE_12BYTE);

	CyFxUsbI2cTransfer(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress,FRAM_SLAVEADDR,8,gI2CRxBuf,READ);   	/** Reading from FRAM SLAVE ADRRESS 8bytes of data in every iteration*/

	if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress <= gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteCount)
	{
		Config_Eload(Eload_BootUpSync, gI2CRxBuf);
		MsgTimerStart(Eload_Sync_Handling, TIMER0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress += 8;

		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevsion_index == 1)
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision = gI2CRxBuf[2]; //get the function card board revision for CC switch enable/disable.

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevsion_index++;
	}
	else
	{
//		TxPPSCalibDataCalcAPI();/**After writing total Calib data to PPS, Send an API to make TI PPS start calculating Slope and gradients*/
		RaSelection(0); /** By default their will be no Ra Assertion & and which will work for normal type-c cable*/
		PD_Attach_Detach(1); //attach command after power on of function card.
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadCtrlFramByteAddress = 0;
	}
}

/**
 * Function for writing data to Eload through I2C
 * @param aCodeIndex case specific configuration and Eload writing
 * @param aBuffer with Configuration of Eload
 * @author Pranay
 *
 */
void Config_Eload(uint8_t aCodeIndex,uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gI2CTxBuf, 0, BUF_SIZE_256BYTE);

	/**Pranay,07Sept'20,Tracking whether Eload is set or not **/
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus = aCodeIndex;

	switch(aCodeIndex)
	{
	case Eload_StandBy:
	case Eload_Gen:
		gI2CTxBuf[0] = 0xAB;
		gI2CTxBuf[1] = 0x01;//MSG ID needs to changed dynamically
		CyU3PMemCopy (&gI2CTxBuf[2], &aBuffer[0],BUF_SIZE_32BYTE);
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,10,gI2CTxBuf,WRITE);

		break;
	case Eload_BC12:
	case Eload_TurnOFF:
	case Eload_BootUp_Init:
		CyU3PMemCopy (&gI2CTxBuf[0], &aBuffer[0],BUF_SIZE_32BYTE);
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,10,gI2CTxBuf,WRITE);

		break;
	case Eload_BootUpSync: /**Handling for Eload boot up synchronization */
		memset(gI2CTxBuf, 0, BUF_SIZE_12BYTE);
//		memset(gPPSi2cTxBuf, 0, BUF_SIZE_16BYTE);

		gI2CTxBuf[0] = 0xB0; 	/** Code word */
		gI2CTxBuf[1] = 0x01;		/** Msg ID(This can be made dynamic) */
		CyU3PMemCopy( &gI2CTxBuf[2],&aBuffer[0],8);
		CyFxUsbI2cTransfer(0x01,ELOAD_SLAVEADDR,10,gI2CTxBuf,WRITE);
		/**Pushing Calibration data to PPS aswell in 8 BYtes chnk but filling rest of the 8bytes with zero as to generate FIFO interrupt*/
		gPPSi2cTxBuf[0] = 0x0A;
		gPPSi2cTxBuf[1] = 0x01;
		CyU3PMemCopy( &gPPSi2cTxBuf[2],&aBuffer[0],8);
//		CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,16,gPPSi2cTxBuf,WRITE);

		break;
	}

}
/**
 * @brief Validation is API received is related to respective Functional card/not
 * @param aBuffer is the received API from Application
 * @return return true if API is related to functional card /false if not
 */
CyBool_t ValidateCommand(uint8_t *aBuffer)
{
	CyBool_t lRetStatus = CyFalse;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	if(FUNCTION_CARD_ID_MASK == (aBuffer[0] & FUNCTION_CARD_ID_MASK))
		lRetStatus = CyTrue;
	else if(((aBuffer[0] & (FUNCTION_CARD_ID_MASK))>>4) == ((gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID)))
		lRetStatus = CyTrue;

	return lRetStatus;

}

CyBool_t FwModeSelection(uint32_t lModeSel)
{
	CyU3PErrorCode_t  status;
	CyU3PI2cPreamble_t preamble;
//	uint8_t* rd_buf = (uint8_t*)gBuf;
	uint8_t lW_buf[8] = {0};
	preamble.buffer[0] = I2C_SLAVE_ADDRESS | (gI2cSlaveAddrFwCheck << 1) | I2C_CMD_WRITE;
	preamble.buffer[1] = GET_BYTE1 (I2C_FW_CHECK_ADDR);
	preamble.buffer[2] = GET_BYTE0 (I2C_FW_CHECK_ADDR);
//	preamble.buffer[3] = I2C_SLAVE_ADDRESS | (gI2cSlaveAddrFwCheck << 1) | I2C_CMD_READ;
	preamble.length    = 3;
	preamble.ctrlMask  = 0x0000;

	lW_buf[0] = lModeSel;
	lW_buf[1] = lModeSel >> 8;
	lW_buf[2] = lModeSel >> 16;
	lW_buf[3] = lModeSel >> 24;

	status = CyU3PI2cTransmitBytes(&preamble, lW_buf, 4, 0);
	if(status != CY_U3P_SUCCESS)
	{
		return CyFalse;
	}
	return CyTrue;

//	FwUpdateModeCheck();
}

void PPSFWBufCpy(uint8_t *aBuffer)
{
	uint8_t lBufLength = aBuffer[4];
	memcpy(&gPPSFWBuf[gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gPPSFWBufIndex], &aBuffer[5], lBufLength);
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gPPSFWBufIndex += lBufLength;

}
/**
* @author basweshwar <6-aug 2021>
* function will used to update firmware for Eload/PPS
* @param SSBL_SLAVE_ADDR  i2c slave address for ssbl
* @param buffer data buffer
*/

void PPSFWTxHandler(uint8_t *aBuffer)
{

	uint8_t frame=1;
	uint8_t j,buf[256];
	uint16_t bytecount=0;
	uint8_t temp[16],temp1[16];

	memset(temp,0,16);
	memset(temp1,0,16);

	temp[0]=0x08;
	temp1[0]=0x09;

//	no_of_bytes=aBuffer[5]|(aBuffer[6]<<8);
	memcpy(buf,aBuffer,256);

	while(bytecount<256)
	{
		if(frame==19)
		{
			for(j=2;j<6;j++)
				temp[j]=buf[bytecount++];
			frame=1;
			CyFxUsbI2cTransfer(0x01,TI_SSBL_PPS_SLAVEADDR,BUF_SIZE_16BYTE, temp,WRITE);
			CyU3PBusyWait(60000);
			CyU3PBusyWait(60000);
			CyFxUsbI2cTransfer(0x01,TI_SSBL_PPS_SLAVEADDR,BUF_SIZE_16BYTE, temp1,WRITE);
			CyU3PBusyWait(60000);
			CyU3PBusyWait(60000);
		}
		else
		{
			for(j=2;j<16;j++)
				temp[j]=buf[bytecount++];
			CyFxUsbI2cTransfer(0x01,TI_SSBL_PPS_SLAVEADDR,BUF_SIZE_16BYTE, temp,WRITE);
			CyU3PBusyWait(60000);
			CyU3PBusyWait(60000);
			frame++;
		}
	}
}
void PPSHWReset()
{
//	CyU3PGpioSetValue (GPIO35_PPS_RESET, 1);
//	CyU3PBusyWait (2000);//2mS
//	CyU3PGpioSetValue (GPIO35_PPS_RESET, 0);
//	CyU3PBusyWait (2000);//2mS
//	CyU3PGpioSetValue (GPIO35_PPS_RESET, 1);

}
void FWUpdateHandling(uint8_t *aBuffer)
{
	switch(aBuffer[2])
	{
	case FX3_Programming_mode_selection:
		switch(aBuffer[3])
		{
			case Boot_mode:
			FwModeSelection(BOOT_MODE);
			//	  			CyU3PBusyWait(1000);
			//	  			CyU3PDeviceReset(CyTrue);
			break;

			case second_stage_bootloader:
			FwModeSelection(PGM_MODE);
			//	  			CyU3PBusyWait(1000);
			//	  			CyU3PDeviceReset(CyTrue);

			break;
		}
		break;

	case CCG3PA_Programming_mode_selection:

	   switch(aBuffer[3])
	   {
		   case CCG3PA_System_Reset:
			   CcgI2cdataTransfer(aBuffer);
		   break;
		  case CCG3PA_I2C_Bootloader://Send command to CCg3PA to Reset the device to Boot loader.
			  CcgI2cdataTransfer(aBuffer);
			  CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Keep FX3 --> CCG3PA interrupt to High So that CCG3PA will remain in Boot loader
			  break;
	   }
	   break;

	case CCG3PA_FW_Update:
		DataLockIndicator(0);
		CCG3PA_Program_Send(aBuffer);
		DataLockIndicator(1);
		break;

	case DUTsFWUpdate:/**For Updating DUTs FW using CC line*/

		CcgI2cdataTransfer(aBuffer);

		break;
	case TI_PPS_PROGRAMMING:

		switch(aBuffer[3])
		{
		case TI_PPS_RESET:
			PPSHWReset();
		break;
		case TI_PPS_MODESELECT:

			switch(aBuffer[4])
			{
			case TI_PGM_MODE:

				PDSS_StatusLEDHandle(2);//Indication

				gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gPPSFWBufIndex = 0;
				memset(gPPSFWBuf,0x00,BUF_SIZE_280BYTE);

				CyU3PGpioSetValue (GPIO33_PPS_PGM_MODE_SELECTION, TI_PGM_MODE);
//				CyU3PBusyWait (2000);//2mS
//				PPSHWReset();

				break;

			case TI_BOOT_MODE:

				CyU3PGpioSetValue (GPIO33_PPS_PGM_MODE_SELECTION, TI_BOOT_MODE);
				CyU3PBusyWait (2000);//2mS
	//			PPSHWReset();
				DataErrorLEDIndicator(0);
				PDSS_StatusLEDHandle(0);//Indication
				break;
			}
		break;
		case TI_PSS_FW_ERASE:
			PDSS_StatusLEDHandle(1);//Indication
			//Perform I2C Erase operation here by starting a timer and sending I2c Commands
			memcpy(&gPPSi2cTxBuf[0], &aBuffer[4],BUF_SIZE_16BYTE);
			CyFxUsbI2cTransfer(0x01,TI_SSBL_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);

		break;
		case TI_FW_STREAM:
			DataErrorLEDIndicator(2);
			//From next byte based on no. of FW payload count store in a global buffer
			PPSFWBufCpy(aBuffer);

		break;
		case TI_FW_FLASH_WRITE:
			DataErrorLEDIndicator(1);
			//From Global buffer start streaming FW to TI PPS
			PPSFWTxHandler(gPPSFWBuf);

			memset(gPPSFWBuf,0x00,BUF_SIZE_280BYTE);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gPPSFWBufIndex = 0;
			DataErrorLEDIndicator(3);

		break;
		case TI_FW_SSBL_RESET:

			CyU3PGpioSetValue (GPIO33_PPS_PGM_MODE_SELECTION, TI_BOOT_MODE);
			CyU3PBusyWait (2000);//2mS
//			PPSHWReset();
			DataErrorLEDIndicator(0);
			PDSS_StatusLEDHandle(0);//Indication
//			memcpy(&gPPSi2cTxBuf[0], &aBuffer[4],BUF_SIZE_16BYTE);
//			CyFxUsbI2cTransfer(0x01,TI_SSBL_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
		break;

		}

	break;
	}
}

/**
 * @brief Function for handling RS485 data Transfer to the Controlcard/Application
 * @note Rs485 has a limitation of writing at maX. 255Bytes each time
 * @param aBuffer buffer that needs to be streamed back to he Application
 * @param byteCount no. of bytes that should be written
 */
void RS485TXDataHandler(uint8_t *aBuffer, uint8_t  byteCount)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    uint8_t checkbit = (aBuffer[0] << 4);

	if(checkbit == 0x70)
	{
		byteCount = PDC_LED_Status(aBuffer,byteCount);
	}
	RS485SpiTransfer (1,byteCount,aBuffer,NULL,WRITE);
}

static void ledToggle(uint8_t aLED)
{
    static uint32_t counter = 0;

    counter++;
	CyU3PGpioSetValue (aLED, counter);
}

void InitGetBattStatusSetMsg()
{
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0] = 0x21;
	greadConfigBuf[1] = 0x0A;
	greadConfigBuf[2] = 0x02;
	greadConfigBuf[3] = 0x03;
	greadConfigBuf[4] = 0xF2;//Custom config
	greadConfigBuf[5] = 0x0D;//GetBatterystatus Extnded msg
	greadConfigBuf[6] = 0x01;
	greadConfigBuf[7] = 0x00;
	greadConfigBuf[8] = 0x00;
	greadConfigBuf[9] = 0x00;

	CcgI2cdataTransfer(greadConfigBuf);

}
void InitOCPHardreset()
{
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0] = 0x21;
	greadConfigBuf[1] = 0x0A;
	greadConfigBuf[2] = 0x02;
	greadConfigBuf[3] = 0x03;
	greadConfigBuf[4] = 0xF2;//Custom config
	greadConfigBuf[5] = 0x0A;//Hard reset for OCP.Pranay,26May'22, To get the backward compatibility with existing APIs
	greadConfigBuf[6] = 0x01;
	greadConfigBuf[7] = 0x00;
	greadConfigBuf[8] = 0x00;
	greadConfigBuf[9] = 0x00;

	CcgI2cdataTransfer(greadConfigBuf);
}
void FetchBattStatusDetailsBuff()
{
	memset(greadConfigBuf, 0, 24);

	greadConfigBuf[0] = 0x00;
	greadConfigBuf[1] = 0x27;
	greadConfigBuf[2] = 0x02;
	greadConfigBuf[3] = 0x05;
	greadConfigBuf[4] = 0xB1;
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,WRITE);

}

void Toggle_TxinProgressLEDindicator()
{
    static uint32_t counter = 0;

    counter++;

	DataLockIndicator(counter & 1);//On//PRanay,16Spet'22, Turning on Data lock LED when any transcations are happening

}


/**
 * @brief Handler for handling the API received and perform tasks as per the API
 * @note Before handling the API we'll validate whether the API is for particular function card/not
 * @param aBuffer consists of the Data received from the Application
 */
void RS485RXDataHandler(uint8_t *aBuffer)
{
	CyBool_t lIsCmdValid = CyFalse;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	/**
	 * Validate whether the command is a valid command for the respective Function Card,
	 * If valid, then only handle the command, else discard the command
	 * Don't validate iff cmd is from USB Control endpoint
	 */

	if(!gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBCmdWriteFlag)
		lIsCmdValid = ValidateCommand(aBuffer);
	else
		lIsCmdValid =CyTrue;


	if(lIsCmdValid)
	{
		/**Pranay,05Jan'20, (V-1-T250) Copying received rs485 Rx buf data into a local miscellaneous buffer to avoid data mismatch**/
		CyU3PMemCopy (&gMiscBuf[0], &gRS485RxBuf[0],BUF_SIZE_280BYTE);
		switch(aBuffer[0] & 0x0F)
		{
		case Cmd_Set://0x01

			Toggle_TxinProgressLEDindicator();//PRanay,16Spet'22, Turning on Data lock LED when any transcations are happening

			switch(aBuffer[3])
			{
			case (0x01):
				UsbLoopackCommands(aBuffer);
			break;
			case (0x02)://Eload writing
				SetADC_Data(aBuffer);
			break;
			case (0x03)://PD Subsystem Handling
			{
				switch(aBuffer[4])
				{
				case 0x01:
					DetachHandler();
					CcgI2cdataTransfer(gMiscBuf);
					break;
				case 0x20://Tester Src caps update
					SrcCapsUpdateHandler(gMiscBuf);
					break;
				case VUP_AS_SINK://0x03
					CcgI2cdataTransfer(gMiscBuf);
		        	DetachStateVbusHandler();

					break;
				case VUP_AS_SRC://0x04
					CcgI2cdataTransfer(gMiscBuf);
		        	DetachStateVbusHandler();

					break;
				case VUP_AS_DRP://0x06
					CcgI2cdataTransfer(gMiscBuf);
		        	DetachStateVbusHandler();

					break;
				default:
					if(gMiscBuf[5] == 0x02)/**If Request message, ensuring that before sending request turning off eload*/
					{
						if((gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus != Eload_TurnOFF ) &&
									(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus != Eload_BootUpSync))
						{
							HandleEload(Eload_TurnOFF,0);
						}
					}
					CcgI2cdataTransfer(gMiscBuf);
					break;
				}
			}
			break;
			case (0x04)://BC1.2 Handling
				CcgI2cdataTransfer(gMiscBuf);
			break;
			case (0x05)://Testcase execution
				
			break;
			case (0x06): // SPI init & deinit
				switch(aBuffer[4])
				{
					case 0x01: // SPI initialization
			    		CyFxSpiInit (1);

						break;
					case 0x02: //SPI de-initialization
			    		CyU3PSpiDeInit();

						break;
					default:

						break;
				}
				break;
			case 0x07:  // Device reset & Interrupt configuration
				switch(aBuffer[4])
				{
					case 0x01: // Device reset
				    	CyU3PDeviceReset(CyFalse); // soft reset
						break;
					case 0x02: // Intr config & Device reset
						InterruptConfig(0); //Interrupt disable
				    	CyU3PDeviceReset(CyFalse); // soft reset
						break;
					case 0x03: // Intr conig, RA on Both CC & Device reset
						InterruptConfig(0); //Interrupt disable
						RaSelection(4);  //RA on both CC
				    	CyU3PDeviceReset(CyFalse); // soft reset
						break;
					case 0x04: // Intr conig, USB disconnect & Device reset
						InterruptConfig(0); //Interrupt disable
						CyU3PConnectState (CyFalse, CyTrue);    // USB disconnect
						CyU3PDeviceReset(CyFalse); // soft reset
						break;
					default:

						break;
				}
				break;
			case 0x08:  // configure physical configuration register.
					WritePhyConfigReg(aBuffer);
			break;
			case (0x0A): /** To select whether to draw eload by default after PDC or not*/
				DefaultEloadingSelection(aBuffer);
			break;
			case (0x0B):
				RackindicationLEDhandle(aBuffer);
			break;
			case (0x0C):
				GRLUSB_SwingDeemphasisConfig(aBuffer);
			break;
			case (0x0F)://FRAM DATA WRITE
				FRAM_Write(aBuffer);
			break;
			case (0x20):
				SetMegachipsDevData(aBuffer);
			break;
			/**Time stamp related API Handlers**/
			case (0xAB):
				TimeStampTimerReset();
			break;
			case (0xAC):
				TimeStampTimerStop();
			break;
			case (0xAD):
				TimeStampTimerStart();
			break;
			/****/
			case (0xF1)://Ra Selection
				RaSelection(aBuffer[4]);
			break;
			case (0xF2)://eload Ra Switch
				EloadVconnSelection(aBuffer[4]);
			break;
			case (0xF4): // CC line switch selection.
				CCline_SwithSelection(aBuffer[4]);
			break;
			case (0xF5): // Cable type selection.
				CableType_Selection(aBuffer[4]);
			break;
			case (0xF3):
				switch(aBuffer[4])
				{
				case 0x01:
					MainLinkCommIndicationHandle(aBuffer[5]);//1:Enumeration Done, 2:DataTx.In progress,3:NotConnected
					break;
				case 0x02:
					LinkSpeedCommIndicationHandle(aBuffer[5]);//1:HighSpped/FullSpeed; 2:SuperSpeed
					break;
				case 0x03:
					PDSS_StatusLEDHandle(aBuffer[5]);//0:Off,1:PD,2:BC1.2,3:Both ON
					break;
				case 0x04:
					DataErrorLEDIndicator(aBuffer[5]);//0:OFF,1:Error,2:Normal,3:Both ON
					break;
				case 0x05:
					DataLockIndicator(aBuffer[5]);//0:LoopBack ON, 1:Loopback Off
					break;
				case 0x06:
					PDNegLedIndication(aBuffer[5]);//0:PD Neg Fail, 1: PD Neg Pass
					break;
				case 0x07:
					VbusSenseSlectionSwitch(aBuffer[5]);//0:ExternalVbus, 1:TypeC Vbus
					break;
				case 0x08:
					VbusPresenceLEDinidcator(aBuffer[5]);//1:High(Pass),0:Low(Fail)
					break;
				case 0x09:
					VbusShortCktGPIOHandler(aBuffer[5]);//0:No-short(Off),1:Shorted(On)
					break;
				case 0x0A://0:CC1,1:CC2
					TxRxSwitchHandler(aBuffer[5]);
					break;
				case 0x0B:/**D+/D- Switch whether to be connected to Fx3/CCG3PA */
					DpDmSwitchHandle(aBuffer[5]);
					break;
				case 0x0C:
					break;
				case 0x0D:
					break;
				case 0xAA:/**Used for port verification**/
					gFunctStruct_t->gPD_Struct.gIsPortVerificationEnabled = (aBuffer[5] & 0x01);
					gFunctStruct_t->gPD_Struct.gReadPortVerfVconnDataFrom = ((aBuffer[5] & 0x02) >> 1 );/**if 0 :: CC1 read, 1 :: cc2 read*/
					break;
				case 0xAB: //Considering this variable Fx3 will initiate GetBattCaps in respective configured frequency.
					gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EnableGetBatteryCapsFetching = aBuffer[5];//Default value is true
					gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PollingIterReachCount = aBuffer[6];
					break;
				case 0xFA:
					CyU3PGpioSetValue (aBuffer[5], (aBuffer[6]&0x01));
					break;
				default:
					break;
				}
			break;
			case (0xF6):
					TempLimitExceedHandler(aBuffer);
			break;

			case 0xB1:
				CyU3PMemCopy(&gPPSi2cTxBuf[0], &aBuffer[5],BUF_SIZE_16BYTE);
				CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,BUF_SIZE_16BYTE, gPPSi2cTxBuf,WRITE);
			break;

			default://Sending any default Commands directly to CCg3PA
				CcgI2cdataTransfer(gMiscBuf);
			break;
			}
		break;
		case Cmd_Program://0x02

			FWUpdateHandling(aBuffer);

		break;
		case Cmd_Polling://0x0F
		case Cmd_Get://0x07

			if(aBuffer[2] != 0x0C)
				Toggle_TxinProgressLEDindicator();//PRanay,16Spet'22, Turning on Data lock LED when any transcations are happening

			switch(aBuffer[2])
			{
			case (0x01)://FW version fetch
				 FWVersion_Fetch(aBuffer);
			break;
			case (0x03)://Test case Data Fetching

			break;
			case (0x04)://Loopback status and Speed
				USBLoopbackHandler();
			break;
			case (0x05):/**Get Info from ccg3PA*/
				{
					switch(aBuffer[3])/**API that differs what to read from ccg3pa, so handling in separate handlers for every command*/
					{
					case getDUTCapabilities://0x01
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag)
							gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag = 0x00;	/**Pranay,16Dec'20, making cap mismatch flag to false only after SW reading it*/
						PDSSDataReadHandler(aBuffer);
						MsgTimerStart(Timer_tGetDUTCapabilities, TIMER0);
						break;
					case getBC12Status://0x03 /** Handling BC1.2 DUT details Read operation*/
						BC12DataReadHandler(aBuffer);
						break;
					case getSrcCapsextd://0x04
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetSrcCapsEntnd, TIMER0);
						break;
					case getActiveCCStatus://0xF0
						ActiveCCStatusFetch(aBuffer);
						break;
					case getPDCInfo://0xF1
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetPDCInfo, TIMER0);

						break;
					case getVDMInfo:/**0xF6*/
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetuVDM_Status, TIMER0);

						break;
					case getCCGxPortRole:/**0xF7*/
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetCCGxPortRole, TIMER0);

						break;
					case getSOP1RxPktData:/**0xF8*/
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetSOp1RxPktData, TIMER0);

						break;
					case getDpDmData:/**0xF9**/
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetDpDmData, TIMER0);

						break;
					case getEventlogBufData:/**0xAA*/
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetEventlogBufData, TIMER0);
						break;
					/**Need to discuss whether need to implement timer handlers for these conditions or not*/
					case getSnkCapabilities://0x02
					case getPdssVBUSdata://0xF2
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetPDSSVbusData, TIMER0);

						break;
					case getBatteryS0CTempDetails://0xB1
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetBatteryS0CTempDetails, TIMER0);

						break;
					case getStatusMsgInfo:
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetStatusMsgInfo, TIMER0);

						break;
					case getBatterystatusInfo:
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tGetBatterystatusInfo, TIMER0);

						break;
					default:
						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
						PDSSDataReadHandler(aBuffer);/**Function that fills the buffer with checksum and sends it to ccg3pa*/
						MsgTimerStart(Timer_tMiscHandler, TIMER0);

						break;
					}
				}
			break;
			case (0x06)://Get Voltage
				GetVbusData();
//				GetADC_Voltages(aBuffer[3]);
			break;
			case (0x07)://Get Currents
				GetVconnData();
//				GetADC_Currents(aBuffer[3]);
			break;
			case (0x08)://ADC Data Fetch
				GetADC_Data();
			break;
			case (0xA1):
				GetPPSData(aBuffer);
				break;
			case (0x10):
				GetMegachipsDevData(aBuffer);
			break;
			case (0x40): // Read Physical & Link error
				GetPhy_LinkerrorCount(aBuffer);
			break;
			case (0x81)://Reading from FRAM
				FRAM_Read(aBuffer);
				MsgTimerStart(Read_FRAMData, TIMER0);
			break;
			case (0x90)://Data sending from Control Endpoint
				ControlEP_DataReadHandler(aBuffer);
			break;
			case (0x0C): //Reading the Status information of function card .

				//Pranay,16Septt'22, Provsion to configure if GettBattCaps are required to be initiated or not
				if( (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EnableGetBatteryCapsFetching) &&
							(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev == SPECREV_3_0)
					)
				{
					if( (gFunctStruct_t->gPD_Struct.gPollingIterCnt == POLLING_ITER_BATTCAP_INIT_COUNT)
						&& (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus)
							&& (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev == SPECREV_3_0)
						)
					{
						InitGetBattStatusSetMsg();//Preparing API for Initiating Get_battery_status Extended message in CCG3PA
					}


					StatusInformationHandle(gMiscBuf);

					if( (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit < gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent)
							 && (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role == PRT_ROLE_SOURCE ) )

					{
						gFunctStruct_t->gPD_Struct.OCPTriggerCount++;
						if(gFunctStruct_t->gPD_Struct.OCPTriggerCount > 1)
						{
							gFunctStruct_t->gPD_Struct.isOCPTriggered = CyTrue;
							InitOCPHardreset();
							gFunctStruct_t->gPD_Struct.OCPTriggerCount = 0;
							gFunctStruct_t->gPD_Struct.isOCPTriggered = CyFalse;
							break;
						}
					}

					if( (gFunctStruct_t->gPD_Struct.gPollingIterCnt == POLLING_ITER_BATTCAP_INIT_COUNT)
							&& (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus)
								&& (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gDUTSpecRev == SPECREV_3_0)
						)
					{
	//					gFunctStruct_t->gPD_Struct.gPollingIterCnt = 0;
						gFunctStruct_t->gPD_Struct.gPollingIterCnt++;

						memset(gBattStatusBuf,0x00,32);//Pranay, 06Jun'22,

						gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;/**Resetting the Flag to 0 that decides no. of times needs to be retried if complete data in not received*/
	//					PDSSInterruptGPIOHandler(INTR_CLR);

						FetchBattStatusDetailsBuff();

	//					PDSSInterruptGPIOHandler(INTR_SET);

	//					PDSSDataReadHandler(greadConfigBuf);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

						MsgTimerStart(Timer_tGetBatteryS0CTempDetails, TIMER0);
					}
					else
					{
						gFunctStruct_t->gPD_Struct.gPollingIterCnt++;
						if(gFunctStruct_t->gPD_Struct.gPollingIterCnt >= gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PollingIterReachCount)//Pranay,16Sept'22,Making this polling iteration configurable
							gFunctStruct_t->gPD_Struct.gPollingIterCnt = POLLING_ITER_BATTCAP_INIT_COUNT;
						CyU3PMemCopy(&gRS485RxBuf[ (gRS485RxBuf[1] + HEADER_BYTECOUNT) ], &gBattStatusBuf[1], gBattStatusBuf[0]);//Copying received data into global buffer for further using
						gRS485RxBuf[1] += gBattStatusBuf[0];
						RS485TXDataHandler(gRS485RxBuf, (gRS485RxBuf[1]+HEADER_BYTECOUNT));
					}
				}
				else
				{
					StatusInformationHandle(gMiscBuf);

					if( (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gSnkReqCurrentOCPLimit < gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent)
							 && (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role == PRT_ROLE_SOURCE ) )

					{
						gFunctStruct_t->gPD_Struct.OCPTriggerCount++;
						if(gFunctStruct_t->gPD_Struct.OCPTriggerCount > 1)
						{
							gFunctStruct_t->gPD_Struct.isOCPTriggered = CyTrue;
							InitOCPHardreset();
							gFunctStruct_t->gPD_Struct.OCPTriggerCount = 0;
							gFunctStruct_t->gPD_Struct.isOCPTriggered = CyFalse;
							break;
						}
					}

					RS485TXDataHandler(gRS485RxBuf, (gRS485RxBuf[1]+HEADER_BYTECOUNT));

				}
			break;

			case(0x0D):   //Get the PD state of the particular test card.
				GetPDstate(aBuffer);
			break;
			case(0x0E):   // Reading the function card serial number and board revision.
				Funcard_Info(aBuffer);
			break;

			case(0x0F):
				GRLGetRegData(aBuffer[3]);
			break;

			case 0xAB:/**API to read the timestamp*/

				getCurrentTimestamp();

			break;

			default:
				CcgI2cdataTransfer(gMiscBuf);
			break;
			}
		break;
		default:
		break;
		}
	}

}

CyU3PReturnStatus_t SetUsbPowerState(uint8_t *aBuffer)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    apiRetStatus = CyU3PUsbLPMEnable();
	switch(aBuffer[5])
	{
	case 0:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U0);
		break;

	case 1:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U1);
		break;

	case 2:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U2);
		break;
	case 3:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U3);
		break;

	case 4:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_COMP);
		break;
	case 5:
		apiRetStatus = CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_Unknown);
		break;

	default:

		break;
	}
	return apiRetStatus;
}
/***
 *Pranay,07Feb2020, when connected with USB3.0 TypeC-C cable to testercard,
 * it is being enumerated as 2 devices(one 3.0 and other 2.0) in USB Device tree when we connect and disconnect continuosly
 * multiple times. To avoid that disconnecting and connecting again couple of times when we get VBus low interrupt. 
 * After Disconnect and connect turning of LEDs and Switches back to default which was in DataRx handler earlier.
 */
void USBFailSafeCondition()
{
	MsgTimerStop(TIMER0);   // Stop the timer 2 if PD interrupt is received to avoid the multiple USB connect state

//	CyU3PUsbLPMEnable();
//	CyU3PConnectState (CyFalse, CyTrue);    // disconnect
//	CyFxBulkLpApplnStop ();
	CyU3PConnectState (CyTrue, CyFalse);    // connect with USB 2.0 (FS/HS device)
	CyU3PConnectState (CyFalse, CyTrue);   // disconnect
	CyU3PConnectState (CyTrue, CyTrue);    // connect with USB 3.0 (SS device)
	CyU3PConnectState (CyFalse, CyTrue);    // disconnect

	VBUSDetection_Handler(0);/**Handling the Vbus removed interrupt */
}

void UsbLoopackCommands(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aBuffer[4])
	{
	case 0:
		CyFxBulkLpApplnStop ();
		break;

	case 1:
		CyFxBulkLpApplnStart();
		break;
	case 2:  //USB soft disconnect
//		  DpDmSwitchHandle(1); // Connecting D+/D- lines to CCG3PA
		CyU3PConnectState (CyFalse, CyTrue);    // disconnect
		CyU3PConnectState (CyFalse, CyFalse);    // disconnect
        DataLockIndicator(1);    // off
        MainLinkCommIndicationHandle(NotConnected);
        LinkSpeedCommIndicationHandle(CY_U3P_NOT_CONNECTED);
        ResetUSBErrorCount(CLR_PRESENT_ERRCOUNT);
	break;
	case 3:
        CyU3PConnectState (CyTrue, CyFalse);    // connect with USB 2.0 (FS/HS device)
		break;
	case 4: // USB soft connect
//		DpDmSwitchHandle(2); // Connecting D+/D- lines to FX3
		CyU3PConnectState (CyTrue, CyTrue);    // connect with USB 3.0 (SS device)
		break;
	case 0x05:  // USB fallback feature.(/ Switch to a USB 2.0 Connection. /)
        CyU3PUsbAckSetup ();
        CyU3PThreadSleep (100);
        CyFxBulkLpApplnStop ();
        CyU3PConnectState (CyFalse, CyTrue);    // disconnect
        CyU3PThreadSleep (100);
        CyU3PConnectState (CyTrue, CyFalse);    // connect with USB 2.0 (FS/HS device)
		break;
	case 0x06:  // USB fallback feature.(/ Switch to a USB 3.0 connection. /)
		CyU3PUsbAckSetup ();
		CyU3PThreadSleep (100);
		CyFxBulkLpApplnStop ();
		CyU3PConnectState (CyFalse, CyTrue);   // disconnect
		CyU3PThreadSleep (10);
		CyU3PConnectState (CyTrue, CyTrue);    // connect with USB 3.0 (SS device)
		break;

	case 0x07:   // Reset the USb error count
		CheckUsbErrStatus(0);
		ResetUSBErrorCount(CLR_ALL_ERRCOUNT);

		break;
	case 0x0E:
		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBConnectTimer = aBuffer[5];
		break;
	case (0x0F):
			SetUsbPowerState(aBuffer);
		break;
	default:

		break;
	}
}
/**
 * based on the inerrupt type and API received handling it repectively
 * @param lEventType is interrupt type Active Low/High interrupt
 */
void DataHandler(uint8_t lEventType)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(lEventType)
	{
	case GPIO0_INT_TO_FX3://Data handler event is received from the PDSS & USB CtrlEOP
		CcgI2cdataTransfer(gRS485RxBuf);
		break;

	case GPIO21_RS485_IRQ://Data handler event is received from the Control Card
		RS485RXDataHandler(gRS485RxBuf);
		break;
	case GPIO44_VBUS_VBUS_DETECT:
	//	CCG3PA_SnkConfig();
		break;
	default:
		break;
	}
}
/**
 * @brief When the USB loop back Data is received from the host.USB Data Transaction LED Need to be toggled
 *
 */

void ToggleUSBTxnRxnLED()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	CyU3PGpioSetValue (GPIO2_LED1_BI_A, 0);
	uvint32_t * gpioClockPtr = CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(GPIO50_LED1_BI_C);
	* gpioClockPtr = (*gpioClockPtr & ~CY_U3P_LPP_GPIO_INTR) ^ OUT_VALUE;
//	* gpioClockPtr = (*gpioClockPtr & ~CY_U3P_LPP_GPIO_INTR) | USB_TXN_LED_SET;
//	CyU3PBusyWait(10);
//	* gpioClockPtr = ((*gpioClockPtr & ~CY_U3P_LPP_GPIO_INTR) & (~USB_TXN_LED_SET));
//	CyU3PGpioSetValue (GPIO50_USB_DATA_TXN_LED, 1);
//	CyU3PBusyWait(10);
//	CyU3PGpioSetValue (GPIO50_USB_DATA_TXN_LED, 0);
}

void ToggleDebugLED(uint8_t LEDVal)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uvint32_t * gpioClockPtr = CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(LEDVal);
	* gpioClockPtr = (*gpioClockPtr & ~CY_U3P_LPP_GPIO_INTR) ^ OUT_VALUE;
}

#if 0
void GetErrorStatus()
{
	uint16_t phy_error_cnt=0 ,lnk_error_cnt=0;
	CyU3PUsbGetErrorCounts(&phy_error_cnt,&lnk_error_cnt);
	if(phy_error_cnt == 0 || lnk_error_cnt == 0)
	{
		/*register access*/

		uvint32_t * gpioClockPtr1 = CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(GPIO8_LED4_BI_A);
		uvint32_t * gpioClockPtr2 = CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(GPIO9_LED4_BI_C);
//	    * gpioClockPtr1 = (*gpioClockPtr1 & ~CY_U3P_LPP_GPIO_INTR) ^ OUT_VALUE;
//	    * gpioClockPtr2 = (*gpioClockPtr2 & ~CY_U3P_LPP_GPIO_INTR) ^ OUT_VALUE;
	    * gpioClockPtr1 = (*gpioClockPtr1 & ~CY_U3P_LPP_GPIO_INTR) | USB_TXN_LED_SET;
	    * gpioClockPtr2 = ((*gpioClockPtr2 & ~CY_U3P_LPP_GPIO_INTR) & (~USB_TXN_LED_SET));
//	     CyU3PBusyWait(10);/**Pranay,24Sept'19,Commented Delay, As this delay might affect the Data Transaction packets */
	    * gpioClockPtr1 = ((*gpioClockPtr1 & ~CY_U3P_LPP_GPIO_INTR) & (~USB_TXN_LED_SET));
	    * gpioClockPtr2 = ((*gpioClockPtr2 & ~CY_U3P_LPP_GPIO_INTR) & (~USB_TXN_LED_SET));

	    /*function*/

//		CyU3PGpioSetValue (GPIO8_USB_DATA_ERROR_LED, 1);
//		CyU3PGpioSetValue (GPIO9_USB_DATA_NORMAL_LED, 0);
//		CyU3PBusyWait(10);
//		CyU3PGpioSetValue (GPIO8_USB_DATA_ERROR_LED, 0);
//		CyU3PGpioSetValue (GPIO9_USB_DATA_NORMAL_LED, 0);

	}
}
#endif

/**
 * @brief initating the Pointers to the respective buffers to Zero after booting up
 */
void InitBufStructInstance()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

//	gFunctStruct_t->gDataBuf_t.rs485RxBuf = (&grs485RxBuf);
	memset(gRS485RxBuf, 0, BUF_SIZE_256BYTE);

//	gRS485TxBuf = (&grs485TxBuf);
	memset(gRS485TxBuf, 0, BUF_SIZE_256BYTE);

//	gI2CRxBuf = (&gi2cRxBuf);
	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);

//	gI2CTxBuf = (&gi2cTxBuf);
	memset(gI2CTxBuf, 0, BUF_SIZE_256BYTE);

	memset(gBattStatusBuf,0x00,32);

	DebuglogInit();
}

void InitFwConfigStructInstance()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;
}
/**
 * @brief Initiating Default values to the respective flags based on requirement/Function
 */
void InitFwControlStructInstance()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;
	gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.DefaultEloadingFlag  = CyFalse;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardSerial_Number = 0;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision = 0;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadStatus = Eload_TurnOFF;
	/**pranay,16Dec'20, Flag added to indicate the SW that Cap mismatch has occurred, instead of handling the actual flag, handling this flag**/
	/**Making this initially False**/
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag = CyFalse;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapmismatchEloadHandle = CyFalse;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.CapMismatch = CyFalse;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.AutoEloadPercntDraw = 0;
	gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBConnectTimer = CyTrue;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus = CyFalse;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.IsPDCdone = CyFalse;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State = 0xF0;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State  = 0x00;

	gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt = 0;
	gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt = 0;
	gFunctStruct_t->gUsbErrStatus_t.gTotalPhyErrCnt = 0;
	gFunctStruct_t->gUsbErrStatus_t.gTotalLinkErrCnt = 0;
	gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt = 0;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gVbusSenseSwitch = TypeC_Connector;

	gFunctStruct_t->gPD_Struct.gIsPortVerificationEnabled = CyFalse;
	gFunctStruct_t->gPD_Struct.gReadPortVerfVconnDataFrom = 0;
	/**Pranay, 16March'21,Flags used for fetching the current timestamp**/
	gFunctStruct_t->gPD_Struct.gMinutesCount = 0;
	gFunctStruct_t->gPD_Struct.gElapsedTicks = 0;

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role = PRT_ROLE_SINK;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.grole_at_Connect = PRT_ROLE_SINK;
	gFunctStruct_t->gPD_Struct.gPollingIterCnt = 0;

	gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach = DETACH;
	gFunctStruct_t->gPDCStatus.gPDCStatus_t.isNonPDDUTConnc = CyFalse;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnVoltage = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnCurrent = 0x00;

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusVoltage = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusCurrent = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage = 0x00;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent = 0x00;

	//As of 16Sept'22, Polling freq from SW is every 200mS so initiating GetBattCpas for every 10Secs, this variable is configurable from SW.
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PollingIterReachCount = 50;
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EnableGetBatteryCapsFetching = CyTrue;
}
/**
 * Default FW configuration are to be done in this function
 */
void InitFWConfigControl()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	//Check_FcardBoardrev();  /**Check for board revision of function card ,if revision 1.0(disable CC lines switch) */

//	RaSelection(2); /** By default Ra Assertion on CC2*/

	PD_Attach_Detach(2); /**detach command after power on of function card.*/

	GetTestCardInfo();

	EloadFRAM_DataConfig();  /**Handling the Eload Synchronization, i.e., reading data from FRAM and writing it to Eload in 8Bytes step for every 50mS */

	VbusSenseSlectionSwitch(TypeC_Connector); /** Selecting TypeC connector by default*/

	DpDmSwitchHandle(2); /** Selecting Dp/D- lines Switch for FX3*/

	InitFwControlStructInstance(); /** Flags/Variables that needs to be initialized to default values should be handled here*/

	TimeStampTimerStart(); /**Pranay,16March'21, Starting the timestamp timer after bootup*/
}

void InitFwHandleInstance()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

//	InitFwConfigStructInstance();

	InitFwControlStructInstance();

}

/**
 * @brief Associating the pointers to the respective buffers,
 * @note Using pointers here instead of using buffers directly from structures because of memory constraints
 */
void InitTestControlStructInstance()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

//	gTestDataBuf = TestDataBuf;
//	gI2CRxBuf = I2CRxBuf;
//	gI2CTxBuf = I2CTxBuf;
//	gFunctStruct_t->gDataBuf_t.rs485RxBuf = RS485RxBuf;
//	gRS485TxBuf= RS485TxBuf;

}


void InitializeFWApplication()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	InitTestControlStructInstance();

	InitBufStructInstance();

	InitFwHandleInstance();

}

/**
 * @breif Function to turn on the LED if their is data error.
 * @author Harsha
 * @date 18sep'19
 */

void ReadLoopbackStatus(uint8_t aVar)
{
	CyBool_t lErrStatus = 0,lUS2ErrStatus = 0;
//	uint16_t phy_error_cnt=0 ,lnk_error_cnt=0;
	if(aVar == 1) // turn off the Data error LED.
	{
#ifndef REG_GPIO
		DataErrorLEDIndicator(0); // Off
#else
		RegGpioSetValue(GPIO8_LED4_BI_A, 0);
		RegGpioSetValue(GPIO9_LED4_BI_C, 0);
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
#endif
	}
	else
	{
		lErrStatus = CheckUsbErrStatus(1);

		if(lErrStatus)/* Usb Error is present Set Error LED (RED)*/
		{
#ifndef REG_GPIO
			DataErrorLEDIndicator(1); // red
#else
			RegGpioSetValue(GPIO8_LED4_BI_A, 1);
			RegGpioSetValue(GPIO9_LED4_BI_C, 0);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x80);
#endif
		}
		else/*No USB error Status Set the LED to Normal Operation (GREEN)*/
		{
#ifndef REG_GPIO
			DataErrorLEDIndicator(2); // green
#else
			RegGpioSetValue(GPIO8_LED4_BI_A, 0);
			RegGpioSetValue(GPIO9_LED4_BI_C, 1);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x40);
#endif
		}
	}
}

/**
 * @breif Function to control the rack reference LED for user notification which he wish to communicate.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @author Harsha
 * @date 18sep'19
 */

void RackindicationLEDhandle(uint8_t *aBuffer)
{
switch(aBuffer[4])
			{
			case 0:  // OFF
				LinkSpeedCommIndicationHandle(CY_U3P_NOT_CONNECTED);
				MsgTimerStop(TIMER1);
				break;
			case 1: //Toggle the HS/SS LED.
				LinkSpeedCommIndicationHandle(CY_U3P_HIGH_SPEED);
				MsgTimerStart(Timer_Debug_LED_Toggle, TIMER1);
				break;
			}
}


/**
 * @brief Function to get the status information.
 *  @param aBuffer character pointer which holds the base address of the particular buffer.
 * @author Harsha
 * @date 26sep'19
 */

void StatusInformationHandle(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	uint8_t lBuffer[16] = {0},lTxIndex = 0;
	uint16_t lIndex = 0;
	uint8_t lDataLength = 0;
	CyBool_t GpioStatus,GpioStatus1;

	lBuffer[lTxIndex] =   gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus; //0th bit[0]
//	lBuffer[lTxIndex] |=  (gFunctStruct_t->gPDCStatus.gPDCStatus_t.gisPDCapChanged << 1); /**Streaming this status to CC inorder to avoid reading whole 128Bytes PDC info data all the time even if Capabilities not changed**/
	//lBuffer[lTxIndex] |= (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOIndex << 1) ; //[3:1]index

	CyU3PGpioGetValue(GPIO10_LED1_S_C,&GpioStatus);   // power indication LED
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= (GpioStatus << 4);
	lBuffer[lTxIndex] |= (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State);   //Coping the PD contract status to the respective buffer.
	lBuffer[++lTxIndex] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State;   //Coping the PD contract status to the respective buffer.


	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role == PRT_ROLE_SOURCE)
	{
		GetPPSVbusValues();
		//VBus voltage
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage);       //LSB
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage >> 8);  //MSB

		//Vbus Current
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent);       //LSB
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent >> 8);  //MSB

		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnVoltage = 0x00;
		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnCurrent = 0x00;
		//Vconn Values
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnVoltage);
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnVoltage >> 8);

		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnCurrent);
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVConnCurrent >> 8);

	}
	else /*if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role == PRT_ROLE_SINK)*/
	{
		GetVbusStatusValues();    // to get the VBUS voltage and current we are calling this function.

		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusVoltage);       //LSB
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusVoltage >> 8);  //MSB

		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusCurrent);       //LSB
		lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVbusCurrent >> 8);  //MSB
		/**Pranay,If Port verification is enabled means give Vconn data based on the variable selected using API-CC1/CC2, else give based on active CC*/
		if(!gFunctStruct_t->gPD_Struct.gIsPortVerificationEnabled)
		{

			if( (gFunctStruct_t->gPDCStatus.gPDCStatus_t.isAttachDetach == ATTACH) &&
					(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus == CyTrue) )//Pranay,06May'22, Push the CC lines i.e,Vconn data only if in Attach State,
			{

				GetVconnStatusValues();   // to get the Vconn voltage and current we are calling this function.

				lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage);        //LSB
				lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage >> 8);   //MSB

				lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent);        //LSB
				lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent >> 8);   //MSB

				gFunctStruct_t->gPDCStatus.gPDCStatus_t.isNonPDDUTConnc = CyFalse;
			}
			else//Pranay,06May'22, Push the CC lines i.e,Vconn data only if in Attach State,
			{
				gFunctStruct_t->gPDCStatus.gPDCStatus_t.isNonPDDUTConnc = CyTrue;

				lBuffer[++lTxIndex] = 0x00; //LSB
				lBuffer[++lTxIndex] = 0x00;	//MSB

				lBuffer[++lTxIndex] = 0x00; //LSB
				lBuffer[++lTxIndex] = 0x00; //MSB

			}

		}
		else
		{
			GetPortVerfVconnStatusValues(gFunctStruct_t->gPD_Struct.gReadPortVerfVconnDataFrom);

			lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage);        //LSB
			lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnVoltage >> 8);   //MSB

			lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent);        //LSB
			lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.EloadVconnCurrent >> 8);   //MSB
		}
	}

	/**Function to read and populate the current timestamp details in respective variables**/
	getElapsedTimerTicks();

	/**Filling milli Seconds*/
	lBuffer[++lTxIndex] = (gFunctStruct_t->gPD_Struct.gElapsedTicks & 0x00FF);
	lBuffer[++lTxIndex] = (gFunctStruct_t->gPD_Struct.gElapsedTicks & 0xFF00) >> 8;

	/**Filling minutes*/
	lBuffer[++lTxIndex] = (gFunctStruct_t->gPD_Struct.gMinutesCount & 0x00FF);
	lBuffer[++lTxIndex] = (gFunctStruct_t->gPD_Struct.gMinutesCount & 0xFF00) >> 8;

	/**Pranay,16Dec'20, Added 0th bit for informing SW about the Capmismatch status, and once after reading if the value flag is true revert it to false*///14th byte
	lBuffer[++lTxIndex] = (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.CapMismatchStatusInfoFlag & 0x01);/**0th bit for capmismatch status, remaining[7:1] Reserved*/

	lTxIndex++;

	switch(gRS485RxBuf[0] & 0x0F)
	{
		case (0x07):  //run time API command

		gRS485RxBuf[0] = (gRS485RxBuf[0] & 0xF7);
			break;

		case (0x0F):  //Polling API command

		gRS485RxBuf[0] = (gRS485RxBuf[0] & 0xFF);
			break;

		default:

			break;
	}
	//Filling the whole data except Battery status details here, battery status details will be filled in timer expiry handler
	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex], &lBuffer[0],lTxIndex);
//	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}


/**
 * @brief Function to get the serial number and Board revision of Function card.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @author Harsha
 * @date 22Nov'19
 */

void Funcard_Info(uint8_t *aBuffer)
{
	uint16_t lIndex = 0,lTxIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lBuffer[8] = {0};

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
	CyFxUsbI2cTransfer(lIndex,FRAM_SLAVEADDR,20,gI2CRxBuf,READ);  // reading the Fram data

	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardSerial_Number = (gI2CRxBuf[8] | gI2CRxBuf[9] << 8); //Function card serial number.
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardRevision = gI2CRxBuf[10];   // Function card board revision.
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gFunctionCardFramRevision = gI2CRxBuf[7];

	memcpy(&lBuffer[lTxIndex],&gI2CRxBuf[8],BYTE_2);    //copy function card serial number.
	lBuffer[lTxIndex + 2] = gI2CRxBuf[10];              //copy function card board revision.
	lBuffer[lTxIndex + 3] = gI2CRxBuf[7];               //copy function card FRAM revision.

	lTxIndex = 4;

	switch(gRS485RxBuf[0] & 0x0F)
	{
		case (0x07):  //run time API command

		gRS485RxBuf[0] = (gRS485RxBuf[0] & 0xF7);
			break;

		case (0x0F):  //Polling API command

		gRS485RxBuf[0] = (gRS485RxBuf[0] & 0xFF);
			break;

		default:

			break;
	}

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex],&lBuffer[0],BYTE_8);
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * @brief Function to set the CC line switch.
 * @param aVar character variable.
 * @author Harsha
 * @date 28Nov'19
 */

void CCline_SwithSelection(uint8_t aVar)
{
	switch(aVar)
	{
	case 0x00: //Both CC lines are disabled
		//gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue(GPIO3_CC1_SW1, 0);
		CyU3PGpioSetValue (GPIO25_CC2_SW2, 0);
		break;
	case 0x01://CC1 switch enable
		//gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue (GPIO3_CC1_SW1, 1);
		CyU3PGpioSetValue (GPIO25_CC2_SW2, 0);
		break;
	case 0x02://CC2 switch enable
		//gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gRaselectionSwitch = aVar;
		CyU3PGpioSetValue (GPIO3_CC1_SW1, 0);
		CyU3PGpioSetValue (GPIO25_CC2_SW2, 1);
		break;
	case 0x04: //both CC1/CC2 switch are enabled
		CyU3PGpioSetValue (GPIO3_CC1_SW1, 1);
		CyU3PGpioSetValue (GPIO25_CC2_SW2, 1);
		break;
	}

}

/**
 * @brief Function to select the type of cable we are using (normal/special).
 * @param aVar character variable.
 * @author Harsha
 * @date 28Nov'19
 */

void CableType_Selection(uint8_t aVar)
{
	switch(aVar)
	{
	case 0x01: // special cable (assert RA on any CC lines and we are asserting the RA on CC2)
		RaSelection(2);
		break;

	case 0x02: // normal type-c cable
		RaSelection(0);
		break;

	default:

		break;
	}
}

/**
 * @brief Function to detach and attach the function card after function card power on.
 * @param aVar character variable.
 * @author Harsha
 * @date 5Dec'19
 */

void PD_Attach_Detach(uint8_t aVar)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	memset(greadConfigBuf, 0, 8);
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
	switch(aVar)
	{
		case 0x01: //attach
			greadConfigBuf[0] = 0x00;
			greadConfigBuf[1] = 0x11;
			greadConfigBuf[2] = 0x03;
			greadConfigBuf[3]= 0x02;
			greadConfigBuf[4]= 0x03;
			greadConfigBuf[5]= 0x02;
		break;
		case 0x02: //detach
			greadConfigBuf[0] = 0x00;
			greadConfigBuf[1] = 0x11;
			greadConfigBuf[2] = 0x03;
			greadConfigBuf[3]= 0x02;
			greadConfigBuf[4]= 0x03;
			greadConfigBuf[5]= 0x01;

		break;
	}
	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,8,greadConfigBuf,0);
//	CyU3PBusyWait(500);
	CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
}

/**
 * @brief Function to enable the GPIO interrupt from CCG3PA & VBUS interrupt.
 * @author Harsha
 * @date 23Dec'19
 */

void PD_Enable()
{
	GpioConfigure (GPIO0_INT_TO_FX3,CyFalse, CyFalse,CyFalse,CyTrue,CyTrue,CyTrue);// 2nd Par Def. low state	i/p

	GpioConfigure (GPIO44_VBUS_VBUS_DETECT,CyTrue, CyFalse,CyFalse,CyTrue,CyTrue,CyTrue);

//	PD_Attach_Detach(1);
}

/**
 * @brief Function to get the PD state of the particular test card.
 * @author Harsha
 * @date 03Mar'20
 */

void GetPDstate(uint8_t *aBuffer)
{
	uint16_t lIndex = 0,lTxIndex = 0;
	uint8_t lDataLength = 0;
	uint8_t lBuffer[8] = {0};
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	lBuffer[lTxIndex++] = gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus;   //Coping the PD contract status to the respective buffer.

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex],&lBuffer[0],BYTE_8); // copy the data to the RXBuffer
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * @brief Function to disconnect the USB when detach is received.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @author Harsha
 * @date 12Mar'20
 */

void DetachHandler()
{
	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus) //USB disconnect is done only for PD devices.
	{
	    CyU3PConnectState (CyFalse, CyTrue);    // disconnect
	    CyU3PConnectState (CyFalse, CyFalse);    // disconnect
	}
	if(gFunctStruct_t->gPDCStatus.gPDCStatus_t.gcur_port_role == PRT_ROLE_SOURCE)
	{
		DetachStateVbusHandler(NULL);
	}
}

void SrcCapsUpdateHandler(uint8_t *aBuffer)
{
	uint8_t lApiLength = 0;

	lApiLength = aBuffer[1];
	PDSSInterruptGPIOHandler(INTR_CLR);
	aBuffer[lApiLength+2] = CCG3PA_WR_CHKSUM;/**Code word for CCg3pa to indicate whether All bytes has been received or not,CCG3pa(used incase of Request) will validate for this keyword and will perform respective action if not will inititate VconnSwap*/
	gI2CTxBuf[0] = 0x00;/**0th byte will be neglected by ccg3pa when data is received from i2c,so should prepend 0th byte with 0x00 and append actual data*/
	CyU3PMemCopy (&gI2CTxBuf[1], &aBuffer[0],BUF_SIZE_255BYTE);
	lApiLength = (gMiscBuf[1] + HEADER_BYTECOUNT);

	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,128,gI2CTxBuf,WRITE);

	PDSSInterruptGPIOHandler(INTR_SET);
}

/**
 * @brief Function to get the PDC & LED status of test card.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @param byteCount which holds the payload lenght.
 * @author Harsha
 * @date 9Apr'20
 */

uint8_t PDC_LED_Status(uint8_t *aBuffer,uint8_t byteCount)
{
	CyBool_t GpioStatus;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	CyU3PGpioGetValue(GPIO10_LED1_S_C,&GpioStatus);   // power indication LED
	gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= (GpioStatus << 4);

	aBuffer[byteCount] =   gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus; //0th bit[0]

//	aBuffer[byteCount] |= (gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDC_PDOIndex << 1) ; //[3:1]index
	aBuffer[byteCount] |= (gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State);   //Coping the PD contract status to the respective buffer.
	aBuffer[byteCount+ 1] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State;   //Coping the PD contract status to the respective buffer.
	byteCount += 2;
	aBuffer[1]+= 2;
	return byteCount;
}

/**
 * @brief Function to get the USB Physical/Link error & iteration count.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @author Harsha.
 * @date 25Apr'20
 */
void GetPhy_LinkerrorCount(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lBuffer[16] = {0};
	uint16_t lIndex = 0,lTxIndex = 0;
	uint8_t lDataLength = 0;

	CheckUsbErrStatus(0);   //Read the USB error status without iteration count incrementing

	lBuffer[lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt;        //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt >> 8); //MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt;       //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt >> 8);//MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt;        //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt >> 8); //MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gTotalPhyErrCnt;       //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gTotalPhyErrCnt >> 8);//MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gTotalLinkErrCnt;       //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gTotalLinkErrCnt >> 8);//MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt;

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt;       //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt >> 8);//MSB

	lTxIndex++;

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex],&lBuffer[0],BYTE_16); // copy the data to the RXBuffer
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * @brief Function to reset the USB Physical/Link error & iteration count.
 * @author Harsha.
 * @date 25Apr'20
 */
void ResetUSBErrorCount(uint8_t lReset)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(lReset)
	{
		case CLR_ALL_ERRCOUNT: //Reset all error count
			gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gTotalPhyErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gTotalLinkErrCnt =0x00;
			gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt = 0x00;
		break;
		case CLR_PRESENT_ERRCOUNT: //Reset present error count
			gFunctStruct_t->gUsbErrStatus_t.gPhyErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gLinkErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = 0x00;
			break;
		case CLR_ALLUSB2_EERCOUNT: //Reset USB2 error count.
			gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = 0x00;
			gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt = 0x00;
			break;
		case CLR_PRESENTUSB2_ERRCOUNT: //Reset present error count of USB2 register.
			gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = 0x00;
			break;

		default:

			break;
	}
}


/**
 * Pranay,30April'2020, Function that toggles the bit0 in respective GPIO value passed
 * @param lGpioId required GPIO in which bit0 needs to handled
 * @param lValue decides whether GPIO should be high/low
 */
void RegGpioSetValue(uint8_t lGpioId, CyBool_t lValue)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uvint32_t * gpioAddrPtr = CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(lGpioId);
	switch(lValue)
	{
	case 0:
		*gpioAddrPtr = (*gpioAddrPtr  & REG_GPIO_CLR_BIT_1);
		break;

	case 1:
		*gpioAddrPtr = (*gpioAddrPtr  | REG_GPIO_SET_BIT_1);
		break;
	}
}

/**
 * @brief Function to interrupt configuration.
 * @param isEnable boolean variable to enable or disable the interrupt.
 * @date 18May'20
 */
void InterruptConfig(CyBool_t isEnable)
{
	GpioConfigure (GPIO0_INT_TO_FX3,CyFalse, CyFalse,CyFalse,CyTrue,isEnable,CyTrue);

	GpioConfigure (GPIO44_VBUS_VBUS_DETECT,CyTrue, CyFalse,CyFalse,CyTrue,isEnable,CyTrue);

	GpioConfigure (GPIO21_RS485_IRQ,CyTrue, CyFalse,CyFalse,CyTrue,isEnable,CyTrue);
}

/**
 * @brief Function to configure PHY Error Counter Configuration Register.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @ author Harsha
 * @date 27May'20
 */
void WritePhyConfigReg(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	uint32_t ConfigVal = 0;
	uint32_t* LNK_PHY_ERR_STATUS = (uint32_t*) (0xE0033040);

	ConfigVal = (aBuffer[5]  << 8) | aBuffer[4];

	*LNK_PHY_ERR_STATUS = ConfigVal;// Byte 4 of API consist of the value to returned to (7:0) of PHY Error Counter Configuration Register
}

/**
 * @brief Function to get the present & total USB2 error count (Device Controller Master Control and Status Register).
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @ author Harsha
 * @date 28May'20
 */
void ReadUSB2ErrorCnt(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lBuffer[16] = {0}, lUS2ErrStatus = 0;
	uint16_t lIndex = 0,lTxIndex = 0;
	uint8_t lDataLength = 0;

	lUS2ErrStatus = CheckUsb2_ErrStatus();

	lBuffer[lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt;        //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt >> 8); //MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt;       //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt >> 8);//MSB

	lBuffer[++lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt;        //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gErrTestCnt >> 8); //MSB

	lTxIndex++;

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex],&lBuffer[0],BYTE_8); // copy the data to the RXBuffer
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * @brief Function to get the USB2 error count form the register
 * and clearing the error count value after reading.
 * @ author Harsha
 * @date 29May'20
 */
void GetUSB2ErrorCnt(uint8_t *Usb2_error_count)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	*Usb2_error_count = ((UIB->dev_cs >> 8) & 0x000000FF); //to get only error count value from register
	UIB->dev_cs &= (0xFFFF00FF); //Clearing error count [15:8] of USB2 register.
}

/**
 * @brief Function to get the present & total USB2 error count.
 * @ author Harsha
 * @date 29May'20
 */
CyBool_t CheckUsb2_ErrStatus()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t Usb2errorCnt = 0;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

	GetUSB2ErrorCnt(&Usb2errorCnt);

	gFunctStruct_t->gUsbErrStatus_t.gUSB2ErrCnt = Usb2errorCnt;
	gFunctStruct_t->gUsbErrStatus_t.gTotalUSB2ErrCnt += Usb2errorCnt;

	if(Usb2errorCnt)/*If Error Count is Non-Zero*/
	{
		status = CyTrue;
	}
	else
	{
		status = CyFalse;
	}
	return status;
}

/**
 * @brief Function to set the swing & de-emphasis configuration value to the USB 3.0 PHY Transmitter Config Register & USB2.0 PHY_CONF register.
 * @ aRegAddr which holds the address of the register
 * @ aRegValue which hold the value to be return into register
 * @ aParConfig is the configuration type (ie de-emphisis 3.5db/6db and swing low/full)
 * @date 05Jun'20
 */

void GRLSwingDeEmpConfigWrapper(uint8_t UsbSpeed,uint8_t aSwingType,
		uint32_t aSwingRegData,uint8_t aDeEmpType,uint32_t aDeEmpRegData,
		uint8_t aUSB2PreEmpType,uint8_t aUSB2PreEmpData)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint32_t lReadRegVal = 0;
	uint32_t lPar = 0;

	if(UsbSpeed == USB_3_0)
	{
		lReadRegVal = ReadReg(LNK_PHY_TX_TRIM_REG);//Reading Register data

		switch(aDeEmpType)
		{
			case CONFIG_3P5DB_DE_EMP:
				lPar = lReadRegVal & CLR_3P5DB_DEEMP_MASK; // clear the respective bits
				lPar |= (aDeEmpRegData << 0);
				break;
			case CONFIG_6DB_DE_EMP:
				lPar = lReadRegVal & CLR_6DB_DEEMP_MASK; // clear the respective bits
				lPar |= (aDeEmpRegData << 7);
				break;
		}
		switch(aSwingType)
		{
			case CONFIG_FULLSWING:
				lPar = lReadRegVal & CLR_TX_SWING_FULL_MASK; // clear the respective bits
				lPar |= (aSwingRegData << 14);
				break;
			case CONFIG_LOWSWING:
				lPar = lReadRegVal & CLR_TX_SWING_LOW_MASK; // clear the respective bits
				lPar |= (aSwingRegData << 21);
				break;
		}
		ConfigReg(LNK_PHY_TX_TRIM_REG,lPar);//Configuring register
	}
	else if(UsbSpeed == USB_2_0)
	{
		lReadRegVal = ReadReg(PHY_CONF_REG);//Reading Register data

		switch(aUSB2PreEmpType)
		{
			case USB2_0_PRE_EMP_ENABLE:
				lPar = lReadRegVal & CLR_USB2_0_PRE_EMP_MASK; // clear the respective bits
				lPar |= (aUSB2PreEmpData << 21);
			break;

			case USB2_0_PRE_EMP_DISABLE:
				lPar = lReadRegVal & CLR_USB2_0_PRE_EMP_MASK; // clear the respective bits
				lPar |= (aUSB2PreEmpData << 21);
			break;

			default:

			break;
		}
		ConfigReg(PHY_CONF_REG,lPar);//Configuring register
	}
}


/**
 * @brief Function to handle the Swing and de-emphasis configuration sent from application.
 * @param aBuffer character pointer which holds the base address of the particular buffer.
 * @date 05Jun'20
 */

void GRLUSB_SwingDeemphasisConfig(uint8_t *aBuffer)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(aBuffer[4])
	{
	case USB_3_0:
		CyU3PConnectState(CyFalse, CyFalse);//USB disconnect
		CyFx3BusyWait(10);
		CyU3PConnectState(CyTrue, CyTrue);//USB connect
		GRLSwingDeEmpConfigWrapper(USB_3_0,aBuffer[5],aBuffer[6],aBuffer[7],aBuffer[8],
				aBuffer[9],aBuffer[10]);
		break;

	case USB_2_0:
		CyU3PConnectState(CyFalse, CyFalse);//USB disconnect
		CyFx3BusyWait(10);
		CyU3PConnectState(CyTrue, CyFalse);//USB connect
		GRLSwingDeEmpConfigWrapper(USB_2_0,aBuffer[5],aBuffer[6],aBuffer[7],aBuffer[8],
				aBuffer[9],aBuffer[10]);
		break;

	default:

		break;
	}
}

/**
 * @brief Function to get the FX3 register value and sent back the value to application.
 * @ aReg Value which holds the register type
 * @ author Harsha
 * @date 04Jun'20
 */
void GRLGetRegData(uint8_t aReg)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lBuffer[16] = {0};
	uint16_t lIndex = 0,lTxIndex = 0;
	uint8_t lDataLength = 0;
	gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue = 0;

	switch(aReg)
	{
	case LNK_PHY_TX_REG:        /** USB 3.0 PHY Transmitter Config Register*/

		gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue = ReadReg(LNK_PHY_TX_TRIM_REG);

		break;
	case PHY_CONFIG_REG:       /**USB PHY Programmability and Serial Interface Register*/

		gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue = ReadReg(PHY_CONF_REG);

		break;
	default:

		break;
	}

	lBuffer[lTxIndex] = gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue;        //LSB
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue >> 8);
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue >> 16);
	lBuffer[++lTxIndex] = (gFunctStruct_t->gUsbErrStatus_t.gGetRegvalue >> 24); //MSB

	lTxIndex++;

	lDataLength = gRS485RxBuf[1];
	lIndex = lDataLength+2;
	gRS485RxBuf[1] = lDataLength + lTxIndex;
	lDataLength += lTxIndex;
	CyU3PMemCopy (&gRS485RxBuf[lIndex],&lBuffer[0],BYTE_8); // copy the data to the RXBuffer
	RS485TXDataHandler(gRS485RxBuf, (lDataLength+2));
}

/**
 * Pranay,28May'20,Function that reads and returns data read from required register
 * @param aRegAddr Address of register from which data needs to be read
 * @return 32bit data present in register
 */
uint32_t ReadReg(uint32_t aRegAddr)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint32_t *pPHYRegVal = (uint32_t*) (aRegAddr);
	return *pPHYRegVal;
}

/**
 * Pranay,28May'20 Function to Write required value in to particualr register.
 * @param aRegAddr Register Address into which value has to be written
 * @param aRegValue Value that needs to be written into particular register
 */
void ConfigReg(uint32_t aRegAddr , uint32_t aRegValue)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint32_t *pPHYRegVal = (uint32_t*) (aRegAddr);
	*pPHYRegVal = aRegValue;
}
