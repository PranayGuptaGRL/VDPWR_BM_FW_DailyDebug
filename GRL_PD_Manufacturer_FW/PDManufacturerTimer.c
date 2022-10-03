/*
 * PDManufacturerTimer.c
 *
 *  Created on: Jun 26, 2019
 *      Author: Prasanna
 */


#include "PDManufacturerDef.h"
#include "PDManufacturerStruct.h"
#include "PDManufacturerFWControl.h"

gFunctStruct * gFunctStruct_t;

CyU3PReturnStatus_t MsgTimerStart(uint16_t lTimerIndex, uint8_t lTimerNo)
{
	uint32_t msgTimerValue = 0;
	CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(lTimerNo)
	{
	case TIMER0:

//		CyU3PMutexGet (&msgTimerMutex, CYU3P_WAIT_FOREVER);
		msgTimerValue = GetMsgTimer0Count(lTimerIndex);
		SetCurrentRunningTimer(lTimerIndex,msgTimerValue, TIMER0);	//Set the current timer, to send event upon timer expires
		MsgTimerStop(TIMER0);					//Stop current running timer
		retStatus = CyU3PTimerModify (&gMsgTimer0, msgTimerValue, 0);	//Change current timer to new Time Period
		retStatus = CyU3PTimerStart (&gMsgTimer0);
//		CyU3PMutexPut(&msgTimerMutex);
		break;

	case TIMER1:

		msgTimerValue = GetMsgTimer1Count(lTimerIndex);
		SetCurrentRunningTimer(lTimerIndex,msgTimerValue, TIMER1);	//Set the current timer, to send event upon timer expires
		MsgTimerStop(TIMER1);					//Stop current running timer
		retStatus = CyU3PTimerModify (&gMsgTimer1, msgTimerValue, 0);	//Change current timer to new Time Period
		retStatus = CyU3PTimerStart (&gMsgTimer1);

		break;

	case TIMER2:

		msgTimerValue = lTimerIndex;
		SetCurrentRunningTimer(lTimerIndex,msgTimerValue, TIMER2);	//Set the current timer, to send event upon timer expires
		MsgTimerStop(TIMER2);					//Stop current running timer
		retStatus = CyU3PTimerModify (&gMsgTimer2, msgTimerValue, 0);	//Change current timer to new Time Period
		retStatus = CyU3PTimerStart (&gMsgTimer2);

		break;

	case TIMER3:

		msgTimerValue = GetMsgTimer3Count(lTimerIndex);
		SetCurrentRunningTimer(lTimerIndex,msgTimerValue, TIMER3);	//Set the current timer, to send event upon timer expires
		MsgTimerStop(TIMER3);					//Stop current running timer
		retStatus = CyU3PTimerModify (&gMsgTimer3, msgTimerValue, 0);	//Change current timer to new Time Period
		retStatus = CyU3PTimerStart (&gMsgTimer3);

		break;

	default:

		break;
	}
	return retStatus;
}


const uint16_t Timer0Count[]=
{
		30,//1000		//	Timer_tPdGetCapability,			//0
		5,//500			// PDSS_InterruptHandling			//1
		5,			    // USB_StatusRetrieve				//2
		10,				//Eload_ActiveCCFetch				//3
		5,				//PDSS_ActiveCCDetect				//4
		5,				//Eload_ActiveCCDetectAPI			//5
		20,				//Read_FRAMData						//6
		5,				//CCG3PA_BootUp_Poll				//7
		5,//500		 	//PDSSInterrupt_Validation			//8
		2000,//1000			//BC12_InterruptHandling		//9
		50,				//Eload_Sync_Handling				//10
		800,            //USB connect state handling       //11
		30,				//Timer_tGetDUTCapabilities			//12
		30,				//Timer_tGetPDCInfo 				//13
		30,				//Timer_tGetCCGxPortRole 			//14
		30,				//Timer_tGetuVDM_Status				//15
		30,				//Timer_tGetSrcCapsEntnd			//16
		30,				//Timer_tGetEventlogBufData			//17
		30,				//Timer_tMiscHandler				//18
		30,				//Timer_tGetSOp1RxPktData			//19
		30,				//Timer_tGetDpDmData				//20
		20, 			//Timer_ReadppsI2CData				//21
		30,				//Timer_tGetPDSSVbusData			//22
		30,				//Timer_tGetBatteryS0CTempDetails	//23
		30,				//Timer_tGetStatusMsgInfo	 		//24
		30,				//Timer_tGetBatterystatusInfo 		//25
};

const uint16_t Timer3Count[]=
{
		60000,			//Timer_tSampleCheck				//0

};

uint32_t GetMsgTimer0Count(uint16_t lTimerIndex)
{
	return (Timer0Count[lTimerIndex]);

}

uint32_t GetMsgTimer1Count(uint16_t lTimerIndex)
{
	uint32_t lRetTimerval = 0;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(lTimerIndex)
	{
	case Timer_Debug_LED_Toggle:
		lRetTimerval = 150;
		break;
	case Timer_CCG3PA_Pgm_I2C_Read:
		lRetTimerval = 4;
		break;
	case Timer_TempLimitExceedHandle:
		lRetTimerval = 1000;
		break;
	default:
		lRetTimerval = 200;//Timer1Count[lTimerIndex];
		break;
	}
	return (lRetTimerval);

}

uint32_t GetMsgTimer3Count(uint16_t lTimerIndex)
{
	return (Timer3Count[lTimerIndex]);
}
/**
 *
 */
void SetCurrentRunningTimer(uint16_t lTimerIndex, uint32_t lTimePeriod, uint8_t lTimerNo)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	gFunctStruct_t->gFwHandle_t.gTimerParameters_t[lTimerNo].gTimerNo = lTimerNo;
	gFunctStruct_t->gFwHandle_t.gTimerParameters_t[lTimerNo].gTimerName = lTimerIndex;
	gFunctStruct_t->gFwHandle_t.gTimerParameters_t[lTimerNo].gTimerValue = lTimePeriod;
}
/**
 *
 */
uint16_t GetCurrentRunningTimer(uint8_t lTimerNo)
{
	uint16_t lTimerIndex = 0;
	uint32_t lTimerValue = 0;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	lTimerIndex = gFunctStruct_t->gFwHandle_t.gTimerParameters_t[lTimerNo].gTimerName;
	lTimerValue = gFunctStruct_t->gFwHandle_t.gTimerParameters_t[lTimerNo].gTimerValue;

	return lTimerIndex;
}
//
CyU3PReturnStatus_t MsgTimerStop(uint8_t lTimerNo)
{
	CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(lTimerNo)
	{
		case TIMER0:
//			CyU3PMutexGet (&msgTimerMutex, CYU3P_WAIT_FOREVER);
				CyU3PTimerStop (&gMsgTimer0);
//			CyU3PMutexPut (&msgTimerMutex);
		break;
		case TIMER1:
//			CyU3PMutexGet (&msgTimerMutex1, CYU3P_WAIT_FOREVER);
				CyU3PTimerStop (&gMsgTimer1);
//			CyU3PMutexPut (&msgTimerMutex1);
		break;
		case TIMER2:
//			CyU3PMutexGet (&msgTimerMutex1, CYU3P_WAIT_FOREVER);
				CyU3PTimerStop (&gMsgTimer2);
//			CyU3PMutexPut (&msgTimerMutex1);
		break;
		case TIMER3:
			CyU3PTimerStop (&gMsgTimer3);
			break;

	}
	return retStatus;
}
//
CyU3PReturnStatus_t MsgTimerModify(uint32_t lTimerValue,uint8_t timerNo)
{
	CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;

	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	switch(timerNo)
	{
		case TIMER0:
//			CyU3PMutexGet (&msgTimerMutex, CYU3P_WAIT_FOREVER);
			CyU3PTimerStop (&gMsgTimer0);	//Stop current running timer
			retStatus = CyU3PTimerModify (&gMsgTimer0, lTimerValue, 0);	//Change the timer to new
//			CyU3PMutexPut (&msgTimerMutex);
		break;
		case TIMER1:
//			CyU3PMutexGet (&msgTimerMutex1, CYU3P_WAIT_FOREVER);
			CyU3PTimerStop (&gMsgTimer1);	//Stop current running timer
			retStatus = CyU3PTimerModify (&gMsgTimer1, lTimerValue, 0);	//Change the timer to new
//			CyU3PMutexPut (&msgTimerMutex1);
		break;
		case TIMER2:
//			CyU3PMutexGet (&msgTimerMutex1, CYU3P_WAIT_FOREVER);
			CyU3PTimerStop (&gMsgTimer2);	//Stop current running timer
			retStatus = CyU3PTimerModify (&gMsgTimer2, lTimerValue, 0);	//Change the timer to new
//			CyU3PMutexPut (&msgTimerMutex1);
		break;
		case TIMER3:
			CyU3PTimerStop (&gMsgTimer3);
			retStatus = CyU3PTimerModify (&gMsgTimer3, lTimerValue, 0);
			break;
	}
	return retStatus;
}

CyU3PReturnStatus_t MsgTimer0ExpiredHandle (uint8_t regMsgTimerValue)
{
	CyBool_t lIsVbusPresent = 0;
	uint16_t lTimerIndex = 0;

    CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    uint8_t lI2CReadByteCount = 0, byteCount = 0;
    lTimerIndex = GetCurrentRunningTimer(TIMER0);
	CyU3PGpioGetValue (GPIO44_VBUS_VBUS_DETECT, &lIsVbusPresent);

    switch(lTimerIndex)
    {
    case Timer_tPdGetCapability:/**Handling of Sending PD capabilities fetched from CCG3PA to Rs485*/
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
    	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Data reading from ccg3pa through i2c**/
    	lI2CReadByteCount = gI2CRxBuf[2];/**Tracking total bytes read from ccg3pa */

    	if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM))/**If checksum doesn't met, it means data is not read completely, So retry*/
    	{
    		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
    		byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
    		RS485TXDataHandler(gRS485RxBuf, byteCount);
    	}
    	else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**Retrying based on count*/
    	{
    		++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
    		PDSSDataReadHandler(gRS485RxBuf);
			MsgTimerStart(Timer_tPdGetCapability, TIMER0);/**starting the same timer again*/
    	}
    	else
    	{
    		/**TBD ,what to do if even after retrying data not read completely*/
    	}
    	break;
    case PDSS_InterruptHandling:
    	if(lIsVbusPresent)
    	{
        	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
    		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,32,gI2CRxBuf,READ);//i2C data read from CCG3PA
    		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
    		PDSS_InterruptHandler(gI2CRxBuf);
    	}

    	break;
    case USB_StatusRetrieve:
    	byteCount = 7/*(gFunctStruct_t->gDataBuf_t.rs485RxBuf[1]+4)*/;

		CyU3PMemCopy (&gRS485RxBuf[4], &gI2CTxBuf[0],BUF_SIZE_256BYTE);
		gRS485RxBuf[1] = (byteCount-2);
    	RS485TXDataHandler(gRS485RxBuf, byteCount);
    	break;

    case PDSS_ActiveCCDetect://Ra Assertion based on Communicating CC
    	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,128,gI2CRxBuf,READ);//i2C data read from CCG3PA
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
    	gFunctStruct_t->gPD_Struct.gActiveCC = gI2CRxBuf[3];
    	if(gFunctStruct_t->gPD_Struct.gActiveCC == 0x00)//CC1 is active, So enable CC2 Switch
    		CyU3PGpioSetValue (GPIO42_CC2_RA, 1);
    	else//CC2 is active, So enable CC1 Switch
    		CyU3PGpioSetValue (GPIO41_CC1_RA, 1);
    	break;
    case Eload_ActiveCCDetectAPI:
		memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,128,gI2CRxBuf,READ);//i2C data read from CCG3PA
		CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
		gFunctStruct_t->gPD_Struct.gActiveCC = gI2CRxBuf[3];
		if(gFunctStruct_t->gPD_Struct.gActiveCC == 0x00)//CC1 is active, So enable CC2 Switch
			CyU3PGpioSetValue (GPIO18_SEL_CC2_ELOAD_0, 1);
		else//CC2 is active, So enable CC1 Switch
			CyU3PGpioSetValue (GPIO19_SEL_CC1_ELOAD_0, 1);
    	break;
    case Read_FRAMData:

		CyU3PMemCopy(&gRS485RxBuf[(gRS485RxBuf[1]+2)],&gI2CRxBuf[0],BUF_SIZE_256BYTE);

		gRS485RxBuf[1] += gRS485RxBuf[3];

		RS485TXDataHandler(gRS485RxBuf, (gRS485RxBuf[1]+2));

    	break;

    case PDSSInterrupt_Validation:
    	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,16,gI2CRxBuf,READ);//i2C data read from CCG3PA
		if(gI2CRxBuf[4] == 0x05)//PD interrupt
		{
			if(lIsVbusPresent)//
			{
				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
				PDC_Validation();
				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
				MsgTimerStart(PDSS_InterruptHandling, TIMER0);
			}
		}
		else if(gI2CRxBuf[4] == 0x06)//BC1.2 interrupt
		{
			if(lIsVbusPresent)
			{
				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
				BC12_ValidationBufFill();
				CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
				MsgTimerStart(BC12_InterruptHandling, TIMER0);
			}
		}
		else if(gI2CRxBuf[4] == 0x07)//Fallback/Cap mismatch detected
		{
			CapsMisMatchHandler();
		}
		else if(gI2CRxBuf[4] == 0x08)// Attach state interrupt
		{
			AttachStateIntrHandler(gI2CRxBuf);
		}
		else if(gI2CRxBuf[4] == 0x09)//REQ State interrupt
		{
			ReqStateIntrHandler(gI2CRxBuf);
		}
		else if(gI2CRxBuf[4] == 0x0A)
		{
        	DetachStateVbusHandler();
		}
		else/*If buffer filling is incomplete or some discrepencies happened in that case also handle pd interrup*/
		{
			CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 1);//Making High (DQ1) GpIO1 for CCG3PA to handle the interrupt
			PDSSInterrupt_Validation_Handler();
			CyU3PGpioSetValue (GPIO1_INT_TO_CCG3PA, 0);//Making low (DQ1) GpIO1 for CCG3PA to handle the interrupt
			MsgTimerStart(PDSSInterrupt_Validation, TIMER0);
		}
		break;
    case BC12_InterruptHandling:
    	memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,128,gI2CRxBuf,READ);//i2C data read from CCG3PA
		BC12_InterruptHandle(gI2CRxBuf);

		break;
    case Eload_Sync_Handling:
    	EloadFRAM_DataWrite();
    	break;

    case Timer_Connect_State:              // USB connect state handling
    	CyU3PGpioGetValue (GPIO44_VBUS_VBUS_DETECT, &lIsVbusPresent);
    	if(lIsVbusPresent )
    	{
     		CyU3PConnectState(CyFalse, CyTrue);					//If the PD fails also make sure the USB connection is established.
     		CyFx3BusyWait(10);
     		CyU3PConnectState(CyTrue, CyTrue);//Connecting data lines
    	}
     	break;
    case Timer_tGetPDCInfo:/**Pranay15march'2020,Handling Data that which needs to be read from ccg3pa in separate handlers */
    		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
        	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
        	lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/

        	if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_PDC_DETS_CDWORD))/**Validating for total checksum received and Get_PDC_Details keyword(0xD4)*/
        	{
        		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
        		byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);/**If data is read completely, now need to append it whole data to API received previously and bytecount needs to be handled accordingly*/
        		RS485TXDataHandler(gRS485RxBuf, byteCount);/**After appending data transferring only particular no of bytes to Controlcard via rs485 */

        	}
        	else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
        	{
        		++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
        		PDSSInterruptGPIOHandler(INTR_CLR);
        		PDC_Validation();	/**Filling the Get_PDC_Details buffer and sending to ccg3pa*/
        		PDSSInterruptGPIOHandler(INTR_SET);
        		MsgTimerStart(Timer_tGetPDCInfo, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
        	}
        	else
        	{
        		//TBD, what to do if data is not completely read even after retrying ???
        	}
    	break;
    case Timer_tGetDUTCapabilities:/**Pranay15march'2020,Handling Data that which needs to be read from ccg3pa in separate handlers */
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
    	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
    	lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
    	if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_DUTCAPS_CDWORD))/**Validating for total checksum received and GetDUTCaps keyword(0xD1)*/
    	{
    		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);/**If data is read completely, now need to append it whole data to API received previously and bytecount needs to be handled accordingly*/
			RS485TXDataHandler(gRS485RxBuf, byteCount);/**After appending data transferring only particular no of bytes to Controlcard via rs485 */

    	}
    	else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
    	{
    		++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
    		PDSSInterruptGPIOHandler(INTR_CLR);
    		ReadSrcCapBufFill(); /**Filling the Get_DUTcapabilities buffer and sending to ccg3pa*/
    		PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetDUTCapabilities, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
    	}
    	else
    	{
    		//TBD, what to do if data is not completely read even after retrying ???
    	}
    	break;
    case Timer_tGetCCGxPortRole:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_CCGX_PORTROLE_CDWORD))/**Validating for total checksum received and GetDUTCaps keyword(0xD1)*/
		{
    		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
    		byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
    		RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			CCGxPortRoleFetchBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetCCGxPortRole, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/

		}
		else
		{
    		//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetuVDM_Status:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_UVDM_DATA_CDWORD))/**Validating for total checksum received and GetDUTCaps keyword(0xD1)*/
		{
    		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
    		byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
    		RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchuVDMDataBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetuVDM_Status, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/

		}
		else
		{
    		//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetSrcCapsEntnd:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_SRCCAPS_EXTND_CDWORD))/**Validating for total checksum received and GetDUTCaps keyword(0xD1)*/
		{
	   		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchuVDMDataBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetuVDM_Status, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/

		}
		else
		{
    		//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetEventlogBufData:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if(/*(gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && */(gI2CRxBuf[1] == GET_EVENTLOG_BUF_DATA_CDWORD))/**Validating for total checksum received and GetDUTCaps keyword(0xD1)*/
		{
	   		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchEventLogBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetEventlogBufData, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/

		}
		else
		{
    		//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetSOp1RxPktData:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_GET_SOP1_RX_PKTDATA_CDWORD))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
	   		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchSop1RxPktBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetSOp1RxPktData, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
		}
		else
		{
    		//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetDpDmData:
		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_DPDM_DATA_CDWORD))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
			gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchDpDmBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetDpDmData, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
		}
		else
		{
			//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tMiscHandler:
    	memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
    		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
    		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
    		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) )
    		{
    	   		gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
    			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
    			RS485TXDataHandler(gRS485RxBuf, byteCount);
    		}
    		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
    		{
    			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
				PDSSDataReadHandler(gRS485RxBuf);/**Function that fills the buffer with checksum and sends it to ccg3pa*/

    			MsgTimerStart(Timer_tMiscHandler, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/

    		}
    		else
    		{
        		//TBD, what to do if data is not completely read even after retrying ???
    		}
    	break;

    case Timer_ReadppsI2CData:
    	memset(gPPSi2cRxBuf,0x00,BUF_SIZE_16BYTE);

		CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,gPPSi2cRxBuf,READ);/**Reading I2cData from ccg3pa*/
		byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gPPSi2cRxBuf);
		RS485TXDataHandler(gRS485RxBuf, byteCount);
    	break;

    case Timer_tGetPDSSVbusData:
		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_PDSS_VBUS_DATA))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
			gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchPDSSVbusBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetPDSSVbusData, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
		}
		else
		{
			//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetBatteryS0CTempDetails://This has required and decoded data of Get status and get batt status extended messages

		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
    	CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
    	lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
//
//    	DebuglogInit();
//    	DEBUG_LOG(DBG1, 0xB0, gI2CRxBuf[0]);
//    	DEBUG_LOG(DBG1, 0xB1, gI2CRxBuf[1] );
//    	DEBUG_LOG(DBG1, 0xB2, gI2CRxBuf[2] );
//    	DEBUG_LOG(DBG1, 0xB3, gI2CRxBuf[3] );
//    	DEBUG_LOG(DBG1, 0xB4, gI2CRxBuf[4] );
//    	DEBUG_LOG(DBG1, 0xB5, gI2CRxBuf[5] );
//    	DEBUG_LOG(DBG1, 0xB6, gI2CRxBuf[6] );
//    	DEBUG_LOG(DBG1, 0xB7, gI2CRxBuf[7] );
//    	DEBUG_LOG(DBG1, 0xB8, gI2CRxBuf[8] );
//    	DEBUG_LOG(DBG1, 0xAB,  lI2CReadByteCount);

//    	DEBUG_LOG(DBG2, 0xA2, gI2CRxBuf[lI2CReadByteCount+2] );

    	if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_BATT_S0CTEMPDETAILS_CDWORD))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
//        	DEBUG_LOG(DBG1, 0xB9, 0xBE );
			gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = StatusInfoBattCapsDataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
    	else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{

//        	DEBUG_LOG(DBG1, 0xBA, 0xBE );

    		++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);

			FetchBattStatusDetailsBuff();

			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetBatteryS0CTempDetails, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
//			DataLockIndicator(0);

		}
    	else
    	{
//        	DEBUG_LOG(DBG1, 0xBB, 0xBE );

    	}
    	break;
    case Timer_tGetStatusMsgInfo:
		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_STATUS_MSG_INFO_CDWORD))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
			gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchStatusMsgInfoBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetStatusMsgInfo, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
		}
		else
		{
			//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    case Timer_tGetBatterystatusInfo:
		memset(gI2CRxBuf,0x00,BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,64,gI2CRxBuf,READ);/**Reading I2cData from ccg3pa*/
		lI2CReadByteCount = gI2CRxBuf[2];/**Tracking Total byte count from ccg3pa*/
		if((gI2CRxBuf[lI2CReadByteCount+2] == CCG3PA_RD_CHKSUM) && (gI2CRxBuf[1] == GET_BATTERY_STATUS_INFO_CDWORD))/**Validating for total checksum received and GET_GET_SOP1_RX_PKTDATA_CDWORD keyword(0xD1)*/
		{
			gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount = 0;
			byteCount = PDSSCaps_DataAppend(gRS485RxBuf,gI2CRxBuf);
			RS485TXDataHandler(gRS485RxBuf, byteCount);
		}
		else if(gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount < DATA_READ_RETRYCOUNT)/**If data read is not complete i.e, if checksum isnt received so retrying based on count*/
		{
			++gFunctStruct_t->gPD_Struct.gPDSSDataFetchRetryCount;
			PDSSInterruptGPIOHandler(INTR_CLR);
			FetchBattStatusInfoBufFill();
			PDSSInterruptGPIOHandler(INTR_SET);
			MsgTimerStart(Timer_tGetBatterystatusInfo, TIMER0);/**Starting the timer again for validating checksum and pushing to rs485 **/
		}
		else
		{
			//TBD, what to do if data is not completely read even after retrying ???
		}
    	break;
    default:
    	break;
    }

    return retStatus;
}

CyU3PReturnStatus_t MsgTimer1ExpiredHandle (uint8_t regMsgTimerValue)
{
	uint16_t lTimerIndex = 0;
    CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    lTimerIndex = GetCurrentRunningTimer(TIMER1);
    switch(lTimerIndex)
    {
    case Timer_Debug_LED_Toggle:

    	ToggleDebugLED(GPIO4_LED2_BI_A);
    	ToggleDebugLED(GPIO5_LED2_BI_C);
    	MsgTimerStart(Timer_Debug_LED_Toggle, TIMER1);
    	break;
    case Timer_CCG3PA_Pgm_I2C_Read:
    	memset(gI2CTxBuf, 0, BUF_SIZE_256BYTE);
		CyFxUsbI2cTransfer(0x01,CCG3PA_SLAVEADDR,7,gI2CTxBuf,1);
		Validate_CCG3PA_Program(gI2CTxBuf);

    	break;
    case Timer_TempLimitExceedHandle:

    	updateLEDToggleStatus(GPIO13_LED4_S_C);

    	updateLEDToggleStatus(GPIO6_LED3_BI_A);

    	updateLEDToggleStatus(GPIO8_LED4_BI_A);

		MsgTimerStart(Timer_TempLimitExceedHandle, TIMER1);

    	break;
    default:
    break;
    }

    return retStatus;
}

void updateLEDToggleStatus(uint8_t aLEDVal)
{
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	ToggleDebugLED(aLEDVal);

	CyBool_t GPIOStatus = 0;

	CyU3PGpioSimpleGetValue(aLEDVal,&GPIOStatus);

	switch(aLEDVal)
	{
	case GPIO13_LED4_S_C://Uni-color
		if(GPIOStatus)//glowing
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State |= 0x80;

		}
		else //Not-glowing
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gUnicolorLED_State &= 0x7F;

		}
		break;

	case GPIO6_LED3_BI_A://Bi-color
		if(GPIOStatus)
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x20);

		}
		else
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0xCF);

		}
		break;

	case GPIO8_LED4_BI_A://Bi-Color
		if(GPIOStatus)
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State |= (0x80);
		}
		else
		{
			gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gBicolorLED_State &= (0x3F);
		}
		break;
	}


}
CyU3PReturnStatus_t MsgTimer2ExpiredHandle (uint8_t regMsgTimerValue)
{
	uint16_t lTimerIndex = 0;
    CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    lTimerIndex = GetCurrentRunningTimer(TIMER2);

    switch(gFunctStruct_t->gFwHandle_t.FwTimerInfoStruct.gTimer2Info)
    {
    case Timer_RS485DevInit:

		gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;
    	gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;
    	RS485DeviceInit();  // RS485 initialization

    	break;

    default:

    	break;

    }
    return retStatus;
}
CyU3PReturnStatus_t MsgTimer3ExpiredHandle (uint8_t regMsgTimerValue)
{
	uint16_t lTimerIndex = 0;
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    CyU3PReturnStatus_t retStatus = CY_U3P_SUCCESS;

    lTimerIndex = GetCurrentRunningTimer(TIMER3);
    switch(lTimerIndex)
    {
		case Timer_tSampleCheck :

			++gFunctStruct_t->gPD_Struct.gMinutesCount;
			MsgTimerStart(Timer_tSampleCheck, TIMER3);

			/**if the minutes variable reached max count than reset it to zero*/
			if(gFunctStruct_t->gPD_Struct.gMinutesCount == 65535)
				gFunctStruct_t->gPD_Struct.gMinutesCount = 0;
		break;
		default:

			break;

    }
    return retStatus;


}

