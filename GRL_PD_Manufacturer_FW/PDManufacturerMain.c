/*
 ## Cypress USB 3.0 Platform source file (cyfxbulklpauto.c)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2018,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file illustrates the bulkloop application example using the DMA AUTO mode */

/*
   This example illustrates a loopback mechanism between two USB bulk endpoints. The example comprises of
   vendor class USB enumeration descriptors with 2 bulk endpoints. A bulk OUT endpoint acts as the producer
   of data from the host. A bulk IN endpint acts as the consumer of data to the host.

   The loopback is achieved with the help of a DMA AUTO channel. DMA AUTO channel is created between the
   producer USB bulk endpoint and the consumer USB bulk endpoint. Data is transferred from the host into
   the producer endpoint which is then directly transferred to the consumer endpoint by the DMA engine.
   CPU is not involved in the data transfer.

   The DMA buffer size is defined based on the USB speed. 64 for full speed, 512 for high speed and 1024
   for super speed. CY_FX_BULKLP_DMA_BUF_COUNT in the header file defines the number of DMA buffers.
 */
/*
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
*/
#include "PDManufacturerStruct.h"
#include "PDManufacturerMain.h"

//uint8_t grs485RxBuf[BUF_SIZE_256BYTE] = {0};
//uint8_t grs485TxBuf[BUF_SIZE_256BYTE] = {0};
//uint8_t gi2cRxBuf[BUF_SIZE_256BYTE] = {0};
//uint8_t gi2cTxBuf[BUF_SIZE_256BYTE] = {0};
//uint8_t gtestDataBuf[BUF_SIZE_1K_BYTE] = {0};

CyU3PThread     DataRxThread;           /*I2C thread*/
CyU3PEvent      DataRxEvent;        /*I2C event for data transfer to CCG3PA*/
CyU3PThread     TimerThread;           /*I2C thread*/
CyU3PEvent      gTimerEvent;           /* Event group used to signal the application thread. */

CyU3PThread     BulkLpAppThread;	 /* Bulk loop application thread structure */
CyU3PEvent      glAppEvent;              /* Event group used to signal the application thread. */
CyU3PEvent      gErrHandleEvent;         /* RS485 Recv Interrupt is received. */
CyU3PDmaChannel glChHandleBulkLp;        /* DMA Channel handle */
#ifdef IDLE_THREAD

CyU3PThread		IdleThread;		/*Idle thread*/
CyU3PEvent		IdleEvent;			/* Event group used to signal the application thread. */
#endif

#ifdef ERR_QUEUE
CyU3PQueue 		gEvtQueue;
#endif

CyBool_t glIsApplnActive = CyFalse;      /* Whether the application is active or not. */
CyBool_t glResetDevice = CyFalse;        /* Request to reset the device has been received. */
CyBool_t glForceLinkU2      = CyFalse;   /* Whether the device should try to initiate U2 mode. */
CyBool_t glLPMSupported      = CyTrue;   /* LPM is supported or not. */
uint8_t glUSBTxnCnt = 0;
CyBool_t gIsSuperSpeed      = CyFalse;   /* LPM is supported or not. */

CyU3PUsbEventType_t glAppLastEvent = CY_U3P_USB_EVENT_DISCONNECT;       /* Last USB event notification received. */

#define CYFX_APP_SUSPEND_TASK   (1 << 0) /* Event to request the application thread to place FX3 in suspend mode. */
#define CYFX_APP_U2ENTRY_TASK   (1 << 1) /* Event to request the application thread to initiate U2 entry. */


#define CY_FX_TIMER_GPIO_HIGH_EVENT    (1 << 0)   /* GPIO high event */
#define CY_FX_TIMER_GPIO_LOW_EVENT     (1 << 1)   /* GPIO low event */

uint16_t glI2cPageSize = 0x03;   /* I2C Page size to be used for transfers. */


//Variable declaration
CyU3PEvent glFxGpioAppEvent;    /* GPIO input event group. */

uint8_t gTCSerialNumber[16]  __attribute__ ((aligned(32)));

uint8_t gRS485RxBuf[BUF_SIZE_280BYTE];
uint8_t gRS485TxBuf[BUF_SIZE_280BYTE];
uint8_t gI2CRxBuf[BUF_SIZE_280BYTE];
uint8_t gI2CTxBuf[BUF_SIZE_280BYTE];
uint8_t gMiscBuf[BUF_SIZE_280BYTE];
uint8_t gPPSi2cTxBuf[BUF_SIZE_16BYTE];
uint8_t gPPSi2cRxBuf[BUF_SIZE_16BYTE];
uint8_t gPPSFWBuf[BUF_SIZE_280BYTE];
uint8_t gBattStatusBuf[32];

#ifdef DBGLOG
uint8_t gDbgBuf1[BUF_SIZE_1K_BYTE];
uint8_t gDbgBuf2[BUF_SIZE_128BYTE];
uint8_t gDbgBuf1Index = 0;
uint8_t gDbgBuf2Index = 0;
#endif
CyBool_t isWdTimerSet = CyFalse;

gFunctStruct gStructPtr;
gFunctStruct *gFunctStruct_t;

/**
 *
 * @param lDbg Based on this parameter the The DebugBuffer is being used will be decided
 * @param lStage The Execution Sequence will be logged
 * @param lValue If any value need to be logged then this can be used
 */
void DEBUG_LOG(uint8_t lDbg, uint8_t lStage, uint32_t lValue)
{
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
#ifdef DBGLOG

	switch(lDbg)
	{
	case DBG1:
		if((gDbgBuf1Index + 2) >= BUF_SIZE_1K_BYTE)
			gDbgBuf1Index = 0;

		gDbgBuf1[gDbgBuf1Index++] = lStage;
		gDbgBuf1[gDbgBuf1Index++] = lValue;

		break;

	case DBG2:
		if((gDbgBuf2Index + 2) >= BUF_SIZE_64BYTE)
			gDbgBuf2Index = 0;

		gDbgBuf2[gDbgBuf2Index++] = lStage;
		if( !lValue )
			break;
		gDbgBuf2[gDbgBuf2Index++] = lValue;
		gDbgBuf2[gDbgBuf2Index++] = lValue >> 8;
		gDbgBuf2[gDbgBuf2Index++] = lValue >> 16;
		gDbgBuf2[gDbgBuf2Index++] = lValue >> 24;

		break;
	default :

		break;
	}
#endif
}

/**
 *
 */
void DebuglogInit()
{
#ifdef DBGLOG

	gDbgBuf1Index = 0;
	gDbgBuf2Index = 0;
	memset(gDbgBuf1, 0, BUF_SIZE_1K_BYTE);
	memset(gDbgBuf2, 0, BUF_SIZE_128BYTE);
#endif
}

/* Application Error Handler */
void
CyFxAppErrorHandler (
        CyU3PReturnStatus_t apiRetStatus    /* API return status */
        )
{
    /* Application failed with the error code apiRetStatus */

    /* Add custom debug or recovery actions here */

    /* Loop Indefinitely */
    for (;;)
    {
        /* Thread sleep : 100 ms */
        CyU3PThreadSleep (100);
    }
}

/* This function initializes the debug module. The debug prints
 * are routed to the UART and can be seen using a UART console
 * running at 115200 baud rate. */
void
CyFxBulkLpApplnDebugInit (void)
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Initialize the UART for printing debug messages */
    apiRetStatus = CyU3PUartInit();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set UART configuration */
    CyU3PMemSet ((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyFalse;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig (&uartConfig, NULL);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Set the UART transfer to a really large value. */
    apiRetStatus = CyU3PUartTxSetBlockXfer (0xFFFFFFFF);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Initialize the debug module. */
    apiRetStatus = CyU3PDebugInit (CY_U3P_LPP_SOCKET_UART_CONS, 8);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Turn off the preamble to the debug messages. */
    CyU3PDebugPreamble (CyFalse);
}

/* Callback funtion for the DMA event notification. */
void
CyFxBulkLpDmaCallback (
        CyU3PDmaChannel   *chHandle, /* Handle to the DMA channel. */
        CyU3PDmaCbType_t  type,      /* Callback type.             */
        CyU3PDmaCBInput_t *input)    /* Callback status.           */
{
//	 CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    if (type == CY_U3P_DMA_CB_PROD_EVENT)
    {
        /* This is a produce event notification to the CPU. This notification is
         * received upon reception of every buffer. The DMA transfer will not wait
         * for the commit from CPU. Increment the counter. */
    	if((glLPMSupported) && (gIsSuperSpeed))
    	{
    		glLPMSupported = CyFalse;
    		CyU3PUsbLPMDisable();
    	}
    	ToggleUSBTxnRxnLED();
//    	GetErrorStatus();
    	ReadLoopbackStatus(0);         //check for the status of loop back and LED is handled.
   	    glUSBTxnCnt++;
   	    if(glUSBTxnCnt == 2)
   	    {
   	   	    MainLinkCommIndicationHandle(EnumerationDone);
   	   	    glUSBTxnCnt = 0;
   	    }
//        glDMARxCount++;
    }
}

/* This function starts the bulk loop application. This is called
 * when a SET_CONF event is received from the USB host. The endpoints
 * are configured and the DMA pipe is setup in this function. */
void
CyFxBulkLpApplnStart (
        void)
{
    gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    uint8_t lUsbLinkPwrState = 0;
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
    gIsSuperSpeed = CyFalse;
//    gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed = usbSpeed;
    /* First identify the usb speed. Once that is identified,
     * create a DMA channel and start the transfer on this. */

    /* Based on the Bus Speed configure the endpoint packet size */
    switch (usbSpeed)
    {
        case CY_U3P_FULL_SPEED:
            size = 64;
            break;

        case CY_U3P_HIGH_SPEED:
            size = 512;
            break;

        case  CY_U3P_SUPER_SPEED:
            size = 1024;
            gIsSuperSpeed = CyTrue;
            break;

        default:
            CyU3PDebugPrint (4, "Error! Invalid USB speed.\n");
            CyFxAppErrorHandler (CY_U3P_ERROR_FAILURE);
            break;
    }
//	LinkSpeedCommIndicationHandle(usbSpeed);//1:HighSpped,2.FullSpeed; 3:SuperSpeed

    gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gByteSize = size;
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen =  (usbSpeed == CY_U3P_SUPER_SPEED) ? 15 : 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    /* Producer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Create a DMA Auto Channel between two sockets of the U port.
     * DMA size is set based on the USB speed. */
    dmaCfg.size = size;
    dmaCfg.count = (usbSpeed == CY_U3P_SUPER_SPEED) ? CY_FX_BULKLP_DMA_BUF_COUNT : (2 * CY_FX_BULKLP_DMA_BUF_COUNT);
    dmaCfg.prodSckId = CY_FX_EP_PRODUCER_SOCKET;
    dmaCfg.consSckId = CY_FX_EP_CONSUMER_SOCKET;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
#ifdef LOOPBACK_EVENT
    /* Enabling the callback for produce event. */
    dmaCfg.notification = CY_U3P_DMA_CB_PROD_EVENT;
    dmaCfg.cb = CyFxBulkLpDmaCallback;
#else
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
#endif
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

#ifdef LOOPBACK_EVENT
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkLp,
            CY_U3P_DMA_TYPE_AUTO_SIGNAL, &dmaCfg);
#else
    apiRetStatus = CyU3PDmaChannelCreate (&glChHandleBulkLp,
            CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
#endif
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Flush the Endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Set DMA Channel transfer size */
    apiRetStatus = CyU3PDmaChannelSetXfer (&glChHandleBulkLp, CY_FX_BULKLP_DMA_TX_SIZE);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Update the status flag. */
    glIsApplnActive = CyTrue;
    gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus = glIsApplnActive;
    MainLinkCommIndicationHandle(glIsApplnActive);
}

/* This function stops the bulk loop application. This shall be called whenever
 * a RESET or DISCONNECT event is received from the USB host. The endpoints are
 * disabled and the DMA pipe is destroyed by this function. */
void
CyFxBulkLpApplnStop (
        void)
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    glIsApplnActive = CyFalse;
    gIsSuperSpeed = CyFalse;
    gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus = glIsApplnActive;

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(CY_FX_EP_PRODUCER);
    CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

    /* Destroy the channel */
    CyU3PDmaChannelDestroy (&glChHandleBulkLp);

    /* Disable endpoints. */
    CyU3PMemSet ((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Producer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_PRODUCER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
    }

    /* Consumer endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONSUMER, &epCfg);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
    }
//  MainLinkCommIndicationHandle( NotConnected);
//	LinkSpeedCommIndicationHandle(CY_U3P_NOT_CONNECTED);//1:HighSpped,2.FullSpeed; 3:SuperSpeed,0:NC
	ReadLoopbackStatus(1);
}


/* This is the callback function to handle the USB events. */
void
CyFxBulkLpApplnUSBEventCB (
    CyU3PUsbEventType_t evtype, /* Event type */
    uint16_t            evdata  /* Event data */
    )
{
	CyU3PReturnStatus_t apiRetStatus = 0;
    glAppLastEvent = evtype;

    switch (evtype)
    {
        case CY_U3P_USB_EVENT_SETCONF:
            /* Stop the application before re-starting. */
//        	CyU3PUsbLPMDisable();
            if (glIsApplnActive)
            {
                CyFxBulkLpApplnStop ();
            }
            /* Start the loop back function. */
            apiRetStatus = CyU3PUsbLPMEnable;//CyU3PUsbLPMEnable();//CyU3PUsbLPMDisable();
            glLPMSupported = CyTrue;
            glUSBTxnCnt = 0;

        	apiRetStatus = CheckUsbErrStatus(0);
            if(apiRetStatus)
            	DataErrorLEDIndicator(1);   //Red Indication will be shown if there is a Physical or link error.

            CyU3PBusyWait (10000);   /**harsha, 18sep'19, 10 millisec delay is been given b/w re-enable the USB driver state machine
                                      and get speed of usb port */
            CyFxBulkLpApplnStart ();
            break;
#if 0// 28feb'20 USB 2.0 loopback ( V-1-I12)
        case CY_U3P_USB_EVENT_RESUME:
             /* Stop the application before re-starting. */
            if (glIsApplnActive)
            {
                CyFxBulkLpApplnStop ();
            }
            /* Start the loop back function. */
//            apiRetStatus = CyU3PUsbLPMEnable();//CyU3PUsbLPMDisable();
//            CyU3PBusyWait(10000);
            CyFxBulkLpApplnStart ();
            break;
#endif
        case CY_U3P_USB_EVENT_RESET:
        case CY_U3P_USB_EVENT_DISCONNECT:
            /* Stop the loop back function. */
        //CyU3PUsbLPMEnable();
        //glLPMSupported = CyTrue;
            if (glIsApplnActive)
            {
                CyFxBulkLpApplnStop ();
            }
            glForceLinkU2 = CyFalse;
            break;
#if 0
        case CY_U3P_USB_EVENT_SUSPEND:
			MainLinkCommIndicationHandle( NotConnected);
			LinkSpeedCommIndicationHandle(CY_U3P_NOT_CONNECTED);//1:HighSpped,2.FullSpeed; 3:SuperSpeed,0:NC
			gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus = CyFalse;

            CyU3PEventSet (&glAppEvent, CYFX_APP_SUSPEND_TASK, CYU3P_EVENT_OR);
            break;
#endif
        case CY_U3P_USB_EVENT_CONNECT:

        	DataLockIndicator(0); //On to represent USB connect event is received.
        	apiRetStatus = CheckUsbErrStatus(1);
        	if(apiRetStatus)
        		DataErrorLEDIndicator(1);   //Red Indication will be shown if there is a Physical or link error.
        	if(evdata == 1) //USB 3.0
        		LinkSpeedCommIndicationHandle(CY_U3P_SUPER_SPEED);
        	else            //USB 2.0
        		LinkSpeedCommIndicationHandle(CY_U3P_HIGH_SPEED);
        	break;

        default:
            break;
    }
}

/* Callback to handle the USB setup requests. */
CyBool_t
CyFxBulkLpApplnUSBSetupCB (
        uint32_t setupdat0, /* SETUP Data 0 */
        uint32_t setupdat1  /* SETUP Data 1 */
    )
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */

    uint8_t  bRequest, bReqType,lTxIndex,lBuffer[16] = {0};
    uint32_t lReadRegVal = 0;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;
    CyU3PReturnStatus_t lRetStatus;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyBool_t GPIOstate = 0;
    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

    /* The only supported vendor command is to automatically reset the FX3 device. */
    if ((bType == CY_U3P_USB_VENDOR_RQT) && (bRequest == 0xE0) && (wLength == 0))
    {
        glResetDevice = CyTrue;
        CyU3PUsbAckSetup ();
        isHandled = CyTrue;
    }

    if (bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if ((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                    || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if (glIsApplnActive)
            {
                CyU3PUsbAckSetup ();

                /* As we have only one interface, the USB link can be pushed into the U2 state as
                   soon as this interface is suspended.
                 */
                if (bRequest == CY_U3P_USB_SC_SET_FEATURE)
                {
                    glForceLinkU2 = CyTrue;
                    CyU3PEventSet (&glAppEvent, CYFX_APP_U2ENTRY_TASK, CYU3P_EVENT_OR);
                }
                else
                {
                    glForceLinkU2 = CyFalse;
                }
            }
            else
                CyU3PUsbStall (0, CyTrue, CyFalse);

            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a loopback operation, there is no higher
         * level protocol. So flush the EP memory and reset the DMA channel associated
         * with it. If there are more than one EP associated with the channel reset both
         * the EPs. The endpoint stall and toggle / sequence number is also expected to be
         * reset. Return CyFalse to make the library clear the stall and reset the endpoint
         * toggle. Or invoke the CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue.
         * Here we are clearing the stall. */
        if ((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if ((wIndex == CY_FX_EP_PRODUCER) || (wIndex == CY_FX_EP_CONSUMER))
            {
                if (glIsApplnActive)
                {
                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyTrue);
                    CyU3PUsbSetEpNak (CY_FX_EP_CONSUMER, CyTrue);
                    CyU3PBusyWait (125);

                    CyU3PDmaChannelReset (&glChHandleBulkLp);
                    CyU3PUsbFlushEp (CY_FX_EP_PRODUCER);
                    CyU3PUsbFlushEp (CY_FX_EP_CONSUMER);
                    CyU3PUsbResetEp (CY_FX_EP_PRODUCER);
                    CyU3PUsbResetEp (CY_FX_EP_CONSUMER);
                    CyU3PDmaChannelSetXfer (&glChHandleBulkLp, CY_FX_BULKLP_DMA_TX_SIZE);
                    CyU3PUsbStall (wIndex, CyFalse, CyTrue);

                    CyU3PUsbSetEpNak (CY_FX_EP_PRODUCER, CyFalse);
                    CyU3PUsbSetEpNak (CY_FX_EP_CONSUMER, CyFalse);

                    CyU3PUsbAckSetup ();
                    isHandled = CyTrue;
                }
            }
        }
    }
    if (bType == CY_U3P_USB_VENDOR_RQT)
    {
    	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
    	isHandled = CyTrue;

    	switch (bRequest)
    	{
    	case 1:
    		lRetStatus = CyTrue;
    		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBCmdWriteFlag = CyTrue;
    		//memset(gFunctStruct_t->gDataBuf_t.rs485RxBuf, 0x00,BUF_SIZE_256BYTE);
    		lRetStatus = CyU3PUsbGetEP0Data(wLength, gRS485RxBuf, NULL);

    		gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType = GPIO21_RS485_IRQ;
    		DataHandler(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType);
//    		if((gFunctStruct_t->gDataBuf_t.rs485RxBuf[0] & Cmd_Get) == Cmd_Get)
//    			lRetStatus = CyU3PUsbSendEP0Data(gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gByteSize, gFunctStruct_t->gDataBuf_t.rs485RxBuf);
    		gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBCmdWriteFlag = CyFalse;
    		break;
    	case 2:
    		switch(wValue)
    		{
    		case 0x01:  // test card ID
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
        		lBuffer[0] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID;  // copy card ID to TX buffer.
        		lRetStatus = CyU3PUsbSendEP0Data(BYTE_8,lBuffer);  // send data to host control in endpoint.
    //    		memset(gFunctStruct_t->gDataBuf_t.rs485TxBuf, 0, BUF_SIZE_256BYTE); // after sending data to host clear the buffer.
    			break;

    		case 0x02: // Get test card number (From Fram)
        		memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
        		CyFxUsbI2cTransfer(0x00,FRAM_SLAVEADDR,20,gI2CRxBuf,READ);  // reading the Fram data
        		lTxIndex = 0;
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
        		memcpy(&lBuffer[lTxIndex],&gI2CRxBuf[8],BYTE_2);    //copy function card serial number.
        		lBuffer[lTxIndex + 2] = gI2CRxBuf[10];              //copy function card board revision.
        		lBuffer[lTxIndex + 3] = gI2CRxBuf[7];                //copy function card FRAM revision.
        		lRetStatus = CyU3PUsbSendEP0Data(BYTE_7,lBuffer);            // send data to host control in endpoint.
    			break;

    		case 0x03:  // loopback status & speed.
        		lTxIndex = 0;
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
        		lBuffer[lTxIndex] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed;    // USB speed (0-->Not connected/1-->FS/2-->HS/3-->SS)
        		lBuffer[lTxIndex + 1] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus;  // loopback status. (0--> not active/1-->active )
        		lRetStatus = CyU3PUsbSendEP0Data(BYTE_5,lBuffer);
    			break;

    		case 0x04:  // USB fallback feature.(/ Switch to a USB 2.0 Connection. /)
    	        CyU3PUsbAckSetup ();
    	        CyU3PThreadSleep (100);
    	        CyFxBulkLpApplnStop ();
    	        CyU3PConnectState (CyFalse, CyTrue);    // disconnect
    	        CyU3PThreadSleep (100);
    	        CyU3PConnectState (CyTrue, CyFalse);    // connect with USB 2.0 (FS/HS device)
    	        CyU3PThreadSleep (10);
    			break;

    		case 0x05:  // USB fallback feature.(/ Switch to a USB 3.0 connection. /)
    	        CyU3PUsbAckSetup ();
    	        CyU3PThreadSleep (100);
    	        CyFxBulkLpApplnStop ();
    	        CyU3PConnectState (CyFalse, CyTrue);   // disconnect
    	        CyU3PThreadSleep (10);
    	        CyU3PConnectState (CyTrue, CyTrue);    // connect with USB 3.0 (SS device)
    	        CyU3PThreadSleep (10);
    			break;
    		case 0x06:
        		memset(gRS485RxBuf, 0x00,BUF_SIZE_280BYTE);
        		lRetStatus = CyU3PUsbGetEP0Data(wLength, gRS485RxBuf, NULL);

				CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,16,gRS485RxBuf,WRITE);
        		lRetStatus = CyTrue;

    			break;
    		case 0x07:
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

				CyFxUsbI2cTransfer(0x01,TI_PPS_SLAVEADDR,10,gRS485RxBuf,READ);
				CyU3PMemCopy(lBuffer,gRS485RxBuf,10);
        		lRetStatus = CyU3PUsbSendEP0Data(BYTE_5,lBuffer);

        		lRetStatus = CyTrue;

    			break;
    		case 0x08:
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

    			GetPPSVbusValues();
				CyU3PMemCopy(lBuffer,gRS485TxBuf,10);
				lRetStatus = CyU3PUsbSendEP0Data(10,lBuffer);

				lRetStatus = CyTrue;
    			break;
    		case 0x09:
    			PpsVbusZeroIOHandler(wIndex);
    			break;
    		case 0x0A:
    			memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    			lBuffer[0] = 0xAA;
    			lBuffer[1] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage;
    			lBuffer[2] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusVoltage >> 8;

    			lBuffer[3] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent;
				lBuffer[4] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.PPSVbusCurrent >> 8;
    			lBuffer[5] = 0xBB;
				lRetStatus = CyU3PUsbSendEP0Data(10,lBuffer);

				lRetStatus = CyTrue;
    			break;
    		default:

    			break;
    		}
    		break;

    	case 3:

    		lRetStatus = CyU3PUsbGetEP0Data(BUF_SIZE_256BYTE, gRS485TxBuf,NULL);
    		CyFxUsbI2cTransfer (1,CCG3PA_SLAVEADDR,8,gRS485TxBuf,WRITE);

    		break;
    	case 4:  //Read SPI configuration register
    		switch(wValue)
    		{
    		case 0x01: // Configuration register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_config);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_config >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_config >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_config >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    		    break;
    		case 0x02: // status register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_status);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_status >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_status >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_status >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x03: // interrupt register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x04: // interrupt mask register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr_mask);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr_mask >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr_mask >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_intr_mask >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x05: //Egress Data Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_egress_data);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_egress_data >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_egress_data >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_egress_data >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x06: //Ingress Data Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_ingress_data);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_ingress_data >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_ingress_data >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_ingress_data >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x07: // Socket Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_socket);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_socket >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_socket >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_socket >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x08: // Receive Byte Count Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_rx_byte_count);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_rx_byte_count >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_rx_byte_count >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_rx_byte_count >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x09: //Transmit Byte Count Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_tx_byte_count);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_tx_byte_count >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_tx_byte_count >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_tx_byte_count >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x0A: //Block Identification and Version Number Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_id);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_id >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_id >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_id >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		case 0x0B: //Power, Clock, and Reset Control Register
    		    lTxIndex = 0;
    		    memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_power);           //LSB
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_power >> 8);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_power >> 16);
    		    lBuffer[lTxIndex++] = (SPI->lpp_spi_power >> 24);     //MSB
    		    lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);
    			break;
    		default:

    			break;
    		}
    		break;

    	case 5:  // get Test card ID

//    		lRetStatus = CyU3PUsbGetEP0Data(BUF_SIZE_256BYTE, gFunctStruct_t->gDataBuf_t.rs485TxBuf,NULL);
    		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

    		lBuffer[0] = gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gSystemID;  // copy card ID to TX buffer.
    		lRetStatus = CyU3PUsbSendEP0Data(BYTE_8,lBuffer);  // send data to host control in endpoint.
//    		memset(gRS485TxBuf, 0, BUF_SIZE_256BYTE); // after sending data to host clear the buffer.

    		break;

    	case 6: // Get test card number (From Fram)

    		//lRetStatus = CyU3PUsbGetEP0Data(BUF_SIZE_256BYTE, gRS485TxBuf,NULL);

    		memset(gI2CRxBuf, 0, BUF_SIZE_256BYTE);
    		CyFxUsbI2cTransfer(0x00,FRAM_SLAVEADDR,20,gI2CRxBuf,READ);  // reading the Fram data

    		lTxIndex = 0;
    		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		memcpy(&lBuffer[lTxIndex],&gI2CRxBuf[8],BYTE_2);    //copy function card serial number.
    		lBuffer[lTxIndex + 2] = gI2CRxBuf[10];              //copy function card board revision.
    		lBuffer[lTxIndex + 3] = gI2CRxBuf[7];                //copy function card FRAM revision.
    		lRetStatus = CyU3PUsbSendEP0Data(BYTE_7,lBuffer);            // send data to host control in endpoint.

    		break;

    	case 7:   // Device reset & Interrupt configuration
			switch(wValue)
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

    	case 8:  // loopback status & speed.

    		lTxIndex = 0;
    		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.
    		lBuffer[1] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBSpeed;
    		lBuffer[2] = gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBStatus;
    		lRetStatus = CyU3PUsbSendEP0Data(BYTE_5,lBuffer);

    		break;
    	case 9:      // LPM enable

    		MainLinkCommIndicationHandle(1); // Turn on the DataEnumeration LED
    		DataErrorLEDIndicator(0);       // Turn off the Data error LED
            CyU3PUsbLPMEnable();            //
            glLPMSupported = CyTrue;

    		break;

    	case 0x0A:  // SPI init

    		CyFxSpiInit (1);

    		break;

    	case 0x0B:  // Read Intrcount & Bytecount
    		lTxIndex = 0;
    		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

    		lBuffer[lTxIndex++] = (gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount);      // LSB
    		lBuffer[lTxIndex++] = (gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount >> 8); // MSB
    		lBuffer[lTxIndex++] = (gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount);       //LSB
    		lBuffer[lTxIndex++] = (gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount >> 8);  //MSB
    		lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);

    		break;

    	case 0x0C:   // SPI de-init

    		CyU3PSpiDeInit();

    		break;

        case 0x0D:  // RS485 TX disable
        	RS485WriteDataDriverDisable();
        	break;

        case 0x0E:  // RS485 initialization
        	RS485DeviceInit();
        	break;

        case 0x0F:  // Interrupt count disable
    		switch(wValue)
    		{
    			case 0x01:  // SPI initialization
    				CyU3PSpiInit();
    				break;
    			case 0x02:
//    				GpioConfigure (GPIO34_FAN_TC,CyFalse, CyTrue,CyTrue,CyTrue,CyFalse,CyTrue);
//    				0xE0000C08
       	    		lTxIndex = 0;
        	    	memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

        	    	lBuffer[lTxIndex++] = 0xAB;
        	    	lReadRegVal = ReadReg(SPI_INTR);//Reading Register data
        	    	lBuffer[lTxIndex++] = lReadRegVal;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>8;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>16;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>24;

        	    	lBuffer[lTxIndex++] = 0xAC;
        	    	lReadRegVal = ReadReg(SPI_INTR_MASK);//Reading Register data
        	    	lBuffer[lTxIndex++] = lReadRegVal;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>8;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>16;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>24;



        	    	lBuffer[lTxIndex++] = 0xAD;
        	    	lReadRegVal = ReadReg(SPI_STATUS);//Reading Register data
        	    	lBuffer[lTxIndex++] = lReadRegVal;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>8;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>16;
        	    	lBuffer[lTxIndex++] = lReadRegVal>>24;

        	    	CyU3PUsbSendEP0Data(lTxIndex,lBuffer);

    				break;

    			case 0x08: // Clear Interrupt count
    		    	gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;

    				break;

    			case 0x0A: //Error buffer read
#ifdef DBGLOG

    	    		CyU3PUsbSendEP0Data(BUF_SIZE_64BYTE,&gDbgBuf1[wIndex]);
#endif
    				break;
    			case 0x0B: //Reset error buffer
    				DebuglogInit();

    				break;
    			case 0xB1:
    	    		CyU3PUsbSendEP0Data(BUF_SIZE_128BYTE,&gDbgBuf2[0]);

    				break;

    			case 0x0C:
    	    		lTxIndex = 0;
					memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

    	    		lBuffer[lTxIndex++] = 0xAB;
    	    		lBuffer[lTxIndex++] = SPIDataRxRegRead();
    	    		lBuffer[lTxIndex++] = 0xAC;

					CyU3PUsbSendEP0Data(lTxIndex,lBuffer);

    				break;

    			case 0xF1:  //Physical & Link error count read
    	    		lTxIndex = 0;
    	    		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

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

    	    		lRetStatus = CyU3PUsbSendEP0Data(lTxIndex,lBuffer);

    				break;
    			default:

    				break;
    		}
        	break;
        	case 0xF1:/**Write*/
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

        		lRetStatus = CyU3PUsbGetEP0Data(wLength, lBuffer, NULL);

    			CyFxUsbI2cTransfer(0x01,CLK_PROG_SLAVE_ADDR, 4, lBuffer,WRITE);

        		break;
        	case 0xF2:/**read*/
        		memset(lBuffer, 0, BYTE_16);   // clear the local buffer.

        		lRetStatus = CyU3PUsbGetEP0Data(wLength, lBuffer, NULL);

    			CyFxUsbI2cTransfer(lBuffer[0],CLK_PROG_SLAVE_ADDR,1,lBuffer,WRITE);

    			CyFxUsbI2cTransfer(lBuffer[0],CLK_PROG_SLAVE_ADDR,1,lBuffer,READ);

        		lRetStatus = CyU3PUsbSendEP0Data(BYTE_8,lBuffer);  // send data to host control in endpoint.

        		break;
            default:
                /* This is unknown request. */
                isHandled = CyFalse;
                break;
        }

        /* If there was any error, return not handled so that the library will
         * stall the request. Alternatively EP0 can be stalled here and return
         * CyTrue. */
        if (status != CY_U3P_SUCCESS)
        {
            isHandled = CyFalse;
        }
//        if(gIsSuperSpeed == CyFalse)
//        {
//        	CyFxIntrSrcSinkApplnStart();//Re start the streamer
//        	CyU3PDmaChannelResume( &glChHandleIntrSrc, CyFalse, CyTrue);
//        }
    }

    return isHandled;
}



/* Callback function to handle LPM requests from the USB 3.0 host. This function is invoked by the API
   whenever a state change from U0 -> U1 or U0 -> U2 happens. If we return CyTrue from this function, the
   FX3 device is retained in the low power state. If we return CyFalse, the FX3 device immediately tries
   to trigger an exit back to U0.

   This application does not have any state in which we should not allow U1/U2 transitions; and therefore
   the function always return CyTrue.
 */
CyBool_t
CyFxBulkLpApplnLPMRqtCB (
        CyU3PUsbLinkPowerMode link_mode)
{
    return CyTrue;
}

/* This function initializes the USB Module, sets the enumeration descriptors.
 * This function does not start the bulk streaming and this is done only when
 * SET_CONF event is received. */
void
CyFxBulkLpApplnInit (void)
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PUsbStart failed to Start, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxBulkLpApplnUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxBulkLpApplnUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxBulkLpApplnLPMRqtCB);    

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set device qualifier descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set configuration descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Other Speed Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Set Configuration Descriptor failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

    /* String descriptor 3 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 3, (uint8_t *)CyFxUSBProductSerNumDscr);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB set string descriptor failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
#ifdef DEBUG
    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
#else
        apiRetStatus = CyU3PConnectState(CyFalse, CyTrue);
#endif
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "USB Connect failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
}

void MsgTimerHandling0 (uint32_t arg)
{
	CyU3PEventSet(&gTimerEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
			CYU3P_EVENT_OR);
}

void MsgTimerHandling1 (uint32_t arg)
{
	CyU3PEventSet(&gTimerEvent, CY_FX_GPIOAPP_GPIO_HIGH_EVENT,
			CYU3P_EVENT_OR);
}

void MsgTimerHandling2 (uint32_t arg)
{
	CyU3PEventSet(&gTimerEvent, CY_FX_GPIOAPP_GPIO_HIGH_EVENT_1,
			CYU3P_EVENT_OR);
}
void MsgTimerHandling3 (uint32_t arg)
{
	CyU3PEventSet(&gTimerEvent, CY_FX_GPIOAPP_GPIO_HIGH_EVENT_2,
			CYU3P_EVENT_OR);
}

/**
 * Function to invoke the thread if there is RS485 error
 * @author Harsha
 * @date 16May'20
 */
void RS485ErrHandler()
{
#ifdef ERR_QUEUE

	CyU3PReturnStatus_t Status = CY_U3P_SUCCESS;
	uint32_t TxMsgQueue[10];

//	DEBUG_LOG(DBG1, 0xE2, 0);
	Status = CyU3PQueueSend(&gEvtQueue, TxMsgQueue, CYU3P_NO_WAIT);
	if(Status != CY_U3P_SUCCESS)
	{
		//CyU3PGpioSetValue (GPIO12_LED3_S_C, 1); // on DT-LK LED
		/* Queue Send Failed*/
//		DEBUG_LOG(DBG1, 0xE3, Status);
	}
#endif
}

void SPIDataReadErrorHandler()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t readBuffer[8] = {0};
//	DEBUG_LOG(DBG1, 0xEA, 0);

	CyBool_t lRs485EvtSet = CyFalse;

	SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line Low
	readBuffer[0] = RS485_READ_DATA;
	RecvSpiWords (&readBuffer[0], 2);
	SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line High

	if(readBuffer[0] != 0)
		lRs485EvtSet = RS485DataRecvIntrHandle(readBuffer);
	if(lRs485EvtSet)
		CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
						CYU3P_EVENT_OR);
}

/* GPIO interrupt callback handler. This is received from
 * the interrupt context. So DebugPrint API is not available
 * from here. Set an event in the event group so that the
 * GPIO thread can print the event information. */
void CyFxGpioIntrCb (
        uint8_t gpioId /* Indicates the pin that triggered the interrupt */
        )
{
//	CyBool_t RetVal = CyTrue;
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	CyBool_t gpioValue = CyFalse, lSetEvent = CyFalse;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    uint8_t readBuffer[8] = {0};

    /* Get the status of the pin */
    gpioValue = (CyBool_t)((GPIO->lpp_gpio_simple[gpioId] & CY_U3P_LPP_GPIO_IN_VALUE) >> 1);
    gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType = gpioId;

	if (!gpioValue)
	{
		switch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType)
		{
		case GPIO0_INT_TO_FX3:		/*Interrupt received from the PD Subsystem Need to get the data over I2C from PDSS*/

			lSetEvent = CyTrue;
//			CyU3PEventSet(&glFxGpioAppEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
//							CYU3P_EVENT_OR);

			break;

		case GPIO21_RS485_IRQ:		/*Interrupt received from the RS485 block, get the data over SPI*/

			SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line Low
			readBuffer[0] = RS485_READ_DATA;
			apiRetStatus = RecvSpiWords (&readBuffer[0], 2);
			SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line High
			gErrRetryCount = 0;
			if(readBuffer[0] != 0)
				lSetEvent = RS485DataRecvIntrHandle(readBuffer);

			break;
		case GPIO44_VBUS_VBUS_DETECT:
//			VBUSDetection_Handler(0);/**Handling the Vbus removed interrupt */
//			CyU3PEventSet (&glAppEvent, CYFX_APP_VBUS_RMVD_TASK, CYU3P_EVENT_OR);
			CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,CYU3P_EVENT_OR);
//			lSetEvent = CyTrue;
			break;
		default:

			break;
		}

		if(lSetEvent)
		{
			CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
						CYU3P_EVENT_OR);
		}
	}
	else
	{
		switch(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType)
		{
		case GPIO0_INT_TO_FX3:		/*Interrupt received from the PD Subsystem Need to get the data over I2C from PDSS*/

			lSetEvent = CyTrue;
			CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
						CYU3P_EVENT_OR);
			break;
		case GPIO21_RS485_IRQ:		/*Interrupt received from the RS485 block, get the data over SPI*/

			SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line Low
			readBuffer[0] = RS485_READ_DATA;
			apiRetStatus = RecvSpiWords (&readBuffer[0], 2);
			SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line High

			if(readBuffer[0]!= 0)
				lSetEvent = RS485DataRecvIntrHandle(readBuffer);

			break;
		case GPIO44_VBUS_VBUS_DETECT:
//				apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
//				VBUSDetection_Handler(1); /**Handling the Vbus attached interrupt here*/
//				CyU3PEventSet (&glAppEvent, CYFX_APP_VBUS_CONNECT_TASK, CYU3P_EVENT_OR);
				CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_HIGH_EVENT,CYU3P_EVENT_OR);
			break;
		default:
			break;
		}
		if(lSetEvent)
		{
			CyU3PEventSet(&DataRxEvent, CY_FX_GPIOAPP_GPIO_LOW_EVENT,
						CYU3P_EVENT_OR);
		}
	}
}

/* Entry function for the BulkLpAppThread. */
CyU3PReturnStatus_t
BulkLpAppThread_Entry (
        uint32_t input)
{
	CyBool_t lSetEvent = CyFalse;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t eventFlag;
    uint8_t readBuffer[8] = {0};
	uint32_t RxMsgQueue[10];

    /* Initialize the debug module */
    CyFxBulkLpApplnDebugInit();

    /* Initialize the GPIO module*/
    status = CyFxGpioInit();
    if (status != CY_U3P_SUCCESS)
    {
    	 CyU3PDebugPrint (2, "CyFxGpioInit Failed\r\n");
        return status;
    }
    /* Initialize the SPI interface for flash of page size 256 bytes. */
    status = CyFxSpiInit (1);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /*Initialize the I2C Module*/
    status = CyFxI2cInit (16);
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Initialize the bulk loop application */
    CyU3PBusyWait (10000);

//	CyU3PBusyWait (50000);
//	CyU3PBusyWait (50000);
//
//	CyU3PBusyWait (50000);
//	CyU3PBusyWait (50000);
//
//	CyU3PBusyWait (50000);
//	CyU3PBusyWait (50000);
//
//	CyU3PBusyWait (50000);
//	CyU3PBusyWait (50000);
//
//	CyU3PBusyWait (50000);
//	CyU3PBusyWait (50000);

#if 0
	InitFWConfigControl();	/** Initiating FW Default values to GPIO/Switches */

	CyU3PBusyWait (10000);

    RS485DeviceInit(); /** Initialize the RS485 System for Write/ Read Operation*/

    CyU3PThreadSleep (100);
#endif
    status =  BoardIDDetection(); /** Board Detection logic, BAsed on the Board ID will be validating the API's received.*/
	if (status != CY_U3P_SUCCESS)
	{
		return status;
	}

	InitFWConfigControl();	/** Initiating FW Default values to GPIO/Switches */

	CyU3PBusyWait (10000);

    RS485DeviceInit(); /** Initialize the RS485 System for Write/ Read Operation*/

    CyU3PThreadSleep (100);

	CyFxBulkLpApplnInit();

	for(int blink = 0; blink < 4; blink++)
	{
		ToggleDebugLED(GPIO13_LED4_S_C);
		CyU3PThreadSleep (150);
	}

	PD_Enable(); // Interrupt enable function call.

	CyU3PDebugPrint (4, "\nBlk loop\n");

#ifdef ERR_QUEUE
    for (;;)
    {
    	status = CyU3PQueueReceive(&gEvtQueue, RxMsgQueue, CYU3P_WAIT_FOREVER);
        if(status == CY_U3P_SUCCESS)
        {
//        	DEBUG_LOG(DBG1, 0xE5, status);
			CyU3PSpiDeInit();  // SPI de-initialization
			CyU3PBusyWait(100);
			CyFxSpiInit (1);   //SPI initialization
			CyU3PBusyWait(100);
			RS485DeviceInit();  // RS485 initialization
//			DEBUG_LOG(DBG1, 0xE6, status);

        }
        else
        {
//        	DEBUG_LOG(DBG1, 0xE4, status);
        	/* Mutex Get Failed*/
        }
    }

//handle_error:
    CyU3PDebugPrint (4, "%x: Application failed to initialize. Error code: %d.\n", status);
    while (1);
#endif
#ifdef MAIN_TASK
    for (;;)
    {
#ifdef USB_LPBACK_EVENT_HANDLE
        /*
           This thread sleeps waiting for an event indicating that the device should be placed into
           suspend mode. Even if no event is received, the thread wakes up once in 5 seconds to check
           whether we have a vendor request to reset the FX3 device, and to print a keep-alive message
           to the UART console.
         */
        stat = CyU3PEventGet (&glAppEvent, evMask, CYU3P_EVENT_OR_CLEAR, &evStat, 5000);
        if (stat == CY_U3P_SUCCESS)
        {
            if (evStat & CYFX_APP_SUSPEND_TASK)
            {
                /*
                   Confirm that the last USB event received is a suspend notification.
                   This is required because a spurious suspend event could be raised when FX3 is getting connected
                   to a USB 2.0 host.
                 */
                if (glAppLastEvent == CY_U3P_USB_EVENT_SUSPEND)
                {
                    CyU3PDebugPrint (2, "Entering suspend mode\r\n");
                    stat = CyU3PSysEnterSuspendMode (CY_U3P_SYS_USB_BUS_ACTVTY_WAKEUP_SRC, 0, &wakeupReason);
                    CyU3PDebugPrint (2, "EnterSuspendMode returned %d, wakeup reason=%d\r\n", stat, wakeupReason);
                }
                else
                {
                    CyU3PDebugPrint (2, "Skipped EnterSuspend call: Last event=%d\r\n", glAppLastEvent);
                }
            }
        }

        /* The host has suspended the only USB function. Try to get the link to U2 state. */
        if (glForceLinkU2)
        {
            stat = CyU3PUsbGetLinkPowerState (&curState);
            while ((glForceLinkU2) && (stat == CY_U3P_SUCCESS) && (curState == CyU3PUsbLPM_U0))
            {
                /* Repeatedly try to go into U2 state.*/
                CyU3PUsbSetLinkPowerState (CyU3PUsbLPM_U2);
                CyU3PThreadSleep (5);
                stat = CyU3PUsbGetLinkPowerState (&curState);
            }
        }

        /* Received a request to disconnect from the USB host and reset the FX3 device. */
        if (glResetDevice)
        {
            glResetDevice = CyFalse;
            CyU3PThreadSleep (1000);
            CyU3PConnectState (CyFalse, CyTrue);
            CyU3PThreadSleep (1000);
            CyU3PDeviceReset (CyFalse);
            for (;;);
        }

        CyU3PDebugPrint (2, "Loopback thread: %d\r\n", iter++);
        if (glIsApplnActive)
        {
            /* Keep checking for memory leaks and memory corruption. */
            buf_p = CyU3PMemGetActiveList ();
            while (buf_p != NULL)
            {
                CyU3PDebugPrint (2, "MemAlloc Buffer: %x\r\n", (uint32_t)buf_p);
                buf_p = buf_p->prev_blk;
            }

            buf_p = CyU3PBufGetActiveList ();
            while (buf_p != NULL)
            {
                CyU3PDebugPrint (2, "BufAlloc Buffer: %x\r\n", (uint32_t)buf_p);
                buf_p = buf_p->prev_blk;
            }

            if (CyU3PMemCorruptionCheck () != CY_U3P_SUCCESS)
                CyU3PDebugPrint (2, "Corruption found in CyU3PMemAlloc buffers\r\n");
            if (CyU3PBufCorruptionCheck () != CY_U3P_SUCCESS)
                CyU3PDebugPrint (2, "Corruption found in CyU3PDmaBufferAlloc buffers\r\n");

            CyU3PDebugPrint (2, "Corruption check done\r\n");
        }
#endif //USB_LPBACK_EVENT_HANDLE

        /* Wait for a GPIO event */
        status = CyU3PEventGet (&glFxGpioAppEvent,
                (CY_FX_GPIOAPP_GPIO_HIGH_EVENT | CY_FX_GPIOAPP_GPIO_LOW_EVENT),
                CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
        if (status == CY_U3P_SUCCESS)
        {
        	if(eventFlag & CY_FX_GPIOAPP_GPIO_LOW_EVENT)
        	{

        	}
        }

    }
#endif
//handle_error:
//    CyU3PDebugPrint (4, "%x: Application failed to initialize. Error code: %d.\n", status);
//    while (1);
}

 /*Entry function for I2C transfer of data */
CyU3PReturnStatus_t
DataRxHandlerThread_Entry (
        uint32_t input)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t eventFlag;
    CyU3PThreadSleep (100);
    for (;;)
    {
        status = CyU3PEventGet (&DataRxEvent,
                    (CY_FX_GPIOAPP_GPIO_HIGH_EVENT | CY_FX_GPIOAPP_GPIO_LOW_EVENT | CY_FX_GPIOAPP_GPIO_HIGH_EVENT_1 | CY_FX_GPIOAPP_GPIO_HIGH_EVENT_2),
                    CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
            if (status == CY_U3P_SUCCESS)
            {
            	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
            	if(eventFlag & CY_FX_GPIOAPP_GPIO_LOW_EVENT)
            	{
            		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType == GPIO44_VBUS_VBUS_DETECT)
            		{
            			gFunctStruct_t->gPDCStatus.gPDCStatus_t.PDCStatus = CyFalse;
            			gFunctStruct_t->gPDCStatus.gPDCStatus_t.IsPDCdone = CyFalse;
                        if (glIsApplnActive)
                        {
                            CyFxBulkLpApplnStop ();
                        }
                        if(!glLPMSupported)
                        {
                			CyU3PUsbLPMEnable();
                			glLPMSupported = CyTrue;
                        }

                        CyU3PConnectState(CyFalse, CyTrue);
            			glUSBTxnCnt = 0;
            			USBFailSafeCondition();
            		}
            		else
            		{
            			DataHandler(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType);
            		}
            	}
            	else if(eventFlag & CY_FX_GPIOAPP_GPIO_HIGH_EVENT)
            	{
            		if(gFunctStruct_t->gFwHandle_t.gSystemInfo_t.gEventType == GPIO44_VBUS_VBUS_DETECT)
            			{
							VBUSDetection_Handler(1); /**Handling the Vbus attached interrupt here*/

							if(gFunctStruct_t->gFwHandle_t.gSUSBControl_t.gUSBConnectTimer)
								MsgTimerStart(Timer_Connect_State,TIMER0);
							else
								CyU3PConnectState(CyTrue, CyTrue);//Connecting data lines
            			}
            	}
            }
    }

handle_error:
    CyU3PDebugPrint (4, "%x: Application failed to initialize. Error code: %d.\n", status);
    while (1);
}
#ifdef IDLE_THREAD

CyU3PReturnStatus_t
IdleThreadEntry(
		uint32_t input)
{

	   CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	   uint32_t eventFlag;
	    CyU3PThreadSleep (100);
	    for (;;)
	    {
	    	if(DataRxThread.tx_thread_state != TX_READY)
	    		   tx_thread_resume(&DataRxThread);
	    }

}
#endif

/*Thread Entry for Timr Handling */
CyU3PReturnStatus_t
TimerHandlerThread_Entry (
       uint32_t input)
{
   CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
   uint32_t eventFlag;

   CyU3PThreadSleep (100);

   for (;;)
   {
       status = CyU3PEventGet (&gTimerEvent,
               (CY_FX_GPIOAPP_GPIO_HIGH_EVENT | CY_FX_GPIOAPP_GPIO_LOW_EVENT | CY_FX_GPIOAPP_GPIO_HIGH_EVENT_1 | CY_FX_GPIOAPP_GPIO_HIGH_EVENT_2),
               CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);
       if (status == CY_U3P_SUCCESS)
       {
			if(eventFlag & CY_FX_GPIOAPP_GPIO_LOW_EVENT)
			{
				MsgTimer0ExpiredHandle (NULL);
			}
			else if(eventFlag & CY_FX_GPIOAPP_GPIO_HIGH_EVENT)
			{
				MsgTimer1ExpiredHandle (NULL);
			}
			else if(eventFlag & CY_FX_GPIOAPP_GPIO_HIGH_EVENT_1)
			{
				MsgTimer2ExpiredHandle (NULL);
			}
			else if(eventFlag & CY_FX_GPIOAPP_GPIO_HIGH_EVENT_2)
			{
				MsgTimer3ExpiredHandle (NULL);
			}
       }
   }

handle_error:
   CyU3PDebugPrint (4, "%x: Application failed to initialize. Error code: %d.\n", status);
   while (1);
}


/* Application define function which creates the threads. */
void
CyFxApplicationDefine (
        void)
{
    void *ptr = NULL;
    void *ptr_i2c=NULL;
    void *ptr_timer=NULL;
    void *ptr_Queue = NULL;
#ifdef IDLE_THREAD
    void *ptr_idle = NULL;
#endif

    uint32_t ret = CY_U3P_SUCCESS;

    /* Allocate the memory for the threads */
    ptr = CyU3PMemAlloc (CY_FX_BULKLP_THREAD_STACK);

    /* Create the thread for the application */
    ret = CyU3PThreadCreate (&BulkLpAppThread,           /* Bulk loop App Thread structure */
                          "21:Bulk_loop_AUTO",                     /* Thread ID and Thread name */
                          BulkLpAppThread_Entry,                   /* Bulk loop App Thread Entry function */
                          0,                                       /* No input parameter to thread */
                          ptr,                                     /* Pointer to the allocated thread stack */
                          CY_FX_BULKLP_THREAD_STACK,               /* Bulk loop App Thread stack size */
                          CY_FX_BULKLP_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CY_FX_BULKLP_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the Thread immediately */
                          );

    /* Check the return code */
    if (ret != 0)
    {
        /* Thread Creation failed with the error code ret */

        /* Add custom recovery or debug actions here */

        /* Application cannot continue */
        /* Loop indefinitely */
        while (1);
    }

    ret = CyU3PEventCreate (&glAppEvent);
    if (ret != 0)
    {
        /* Creation of event group failed. Loop indefinitely because the application
           cannot function normally.
         */
        while (1);
    }

#ifdef IDLE_THREAD
    ret = CyU3PEventCreate(&gErrHandleEvent);
    if (ret != 0)
    {
        /* Event group creation failed with the error code retThrdCreate */

        /* Application cannot continue */
        /* Loop indefinitely */
        while(1);
    }
#endif
    /* Allocate the memory for the threads */  /*I2C thread*/
    ptr_i2c = CyU3PMemAlloc (CY_FX_DATA_RX_THREAD_STACK);

    /* Create the thread for the application */
    ret = CyU3PThreadCreate (&DataRxThread,           /* Bulk loop App Thread structure */
                          "22:DATA_RX_THREAD",                     /* Thread ID and Thread name */
                          DataRxHandlerThread_Entry,                   /* Bulk loop App Thread Entry function */
                          0,                                       /* No input parameter to thread */
                          ptr_i2c,                                     /* Pointer to the allocated thread stack */
                          CY_FX_DATA_RX_THREAD_STACK,               /* Bulk loop App Thread stack size */
                          CY_FX_DATA_RX_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CY_FX_DATA_RX_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the Thread immediately */
                          );
    if (ret != 0)
    {
    	/**< Failed to Create Thread */
        while (1);
    }

    /*I2C event for data transfer*/
    ret = CyU3PEventCreate (&DataRxEvent);
    if (ret != 0)
    {
        /* Creation of event group failed. Loop indefinitely because the application
           cannot function normally.
         */
        while (1);
    }
#ifdef TIMER_THREAD
    /* Allocate the memory for the threads */  /*I2C thread*/
    ptr_timer = CyU3PMemAlloc (CY_FX_TIMER_THREAD_STACK);

    /* Create the thread for the application */
    ret = CyU3PThreadCreate (&TimerThread,           /* Bulk loop App Thread structure */
                          "23:TIMER_THREAD",                     /* Thread ID and Thread name */
                          TimerHandlerThread_Entry,                   /* Bulk loop App Thread Entry function */
                          0,                                       /* No input parameter to thread */
                          ptr_timer,                                     /* Pointer to the allocated thread stack */
                          CY_FX_TIMER_THREAD_STACK,               /* Bulk loop App Thread stack size */
                          CY_FX_TIMER_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CY_FX_TIMER_THREAD_PRIORITY,            /* Bulk loop App Thread priority */
                          CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
                          CYU3P_AUTO_START                         /* Start the Thread immediately */
                          );
    if (ret != 0)
    {
    	/**< Failed to Create Thread */
        while (1);
    }

    ret = CyU3PEventCreate (&gTimerEvent);
    if (ret != 0)
    {
        /* Creation of event group failed. Loop indefinitely because the application
           cannot function normally.
         */
        while (1);
    }
#endif
    // 	Timer event
#ifdef TIMER_THREAD
    ret = CyU3PTimerCreate (&gMsgTimer0, MsgTimerHandling0, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#else
    ret = CyU3PTimerCreate (&gMsgTimer0, MsgTimer0ExpiredHandle, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#endif
    if (ret != 0)
    {
    	while(1);
    }
#ifdef TIMER_THREAD
    ret = CyU3PTimerCreate (&gMsgTimer1, MsgTimerHandling1, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#else
    ret = CyU3PTimerCreate (&gMsgTimer1, MsgTimer1ExpiredHandle, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#endif
    if (ret != 0)
    {
    	while(1);
    }
#ifdef TIMER_THREAD
    ret = CyU3PTimerCreate (&gMsgTimer2, MsgTimerHandling2, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#else
    ret = CyU3PTimerCreate (&gMsgTimer1, MsgTimer1ExpiredHandle, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#endif
    if (ret != 0)
    {
    	while(1);
    }

#ifdef TIMER_THREAD
    ret = CyU3PTimerCreate (&gMsgTimer3, MsgTimerHandling3, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#else
    ret = CyU3PTimerCreate (&gMsgTimer1, MsgTimer1ExpiredHandle, 0, DEFAULT_TIMER_VALUE, 0, CYU3P_NO_ACTIVATE);
#endif
    if (ret != 0)
    {
    	while(1);
    }
#ifdef ERR_QUEUE
    ptr_Queue = CyU3PMemAlloc (CY_FX_DATA_QUEUE_SIZE);

    ret = CyU3PQueueCreate (&gEvtQueue, CY_FX_DATA_QUEUE_SIZE/4, ptr_Queue, CY_FX_DATA_QUEUE_SIZE);
    if (ret != 0)
    {
    	while(1);
    }
#endif

#ifdef IDLE_THREAD
    ptr_idle = CyU3PMemAlloc (CY_FX_IDLE_THREAD_STACK);
    ret = CyU3PThreadCreate ( &IdleThread,           /* Bulk loop App Thread structure */
    						 "24:IDLE_THREAD",      /* Thread ID and Thread name */
    						 IdleThreadEntry, 		/* Bulk loop App Thread Entry function */
    						 0,					 	/* No input parameter to thread */
    						 ptr_idle,				/* Pointer to the allocated thread stack */
    						 CY_FX_IDLE_THREAD_STACK,		/* Bulk loop App Thread stack size */
    						 CY_FX_IDLE_THREAD_PRIORITY,	/* Bulk loop App Thread priority */
    						 CY_FX_IDLE_THREAD_PRIORITY,	/* Bulk loop App Thread priority */
    						 CYU3P_NO_TIME_SLICE,		/* No time slice for the application thread */
    						 CYU3P_AUTO_START			/* Start the Thread immediately */
    						 );

    if (ret != 0)
     {
     	/**< Failed to Create Thread */
         while (1);
     }

     /*I2C event for data transfer*/
     ret = CyU3PEventCreate (&IdleEvent);
     if (ret != 0)
     {
         /* Creation of event group failed. Loop indefinitely because the application
            cannot function normally.
          */
         while (1);
     }
#endif

}

static void
FxAppMemCorruptCb (
        void *mem_p)
{
    CyU3PDebugPrint (2, "Memory corruption detected at buffer %x\r\n", (uint32_t)mem_p);
}

gFunctStruct* GetStructPtr()
{
    return (&gStructPtr);
}
/*
 * Main function
 */
int main (void)
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Enable memory corruption checks. */
    CyU3PMemEnableChecks (CyTrue, FxAppMemCorruptCb);
    CyU3PBufEnableChecks (CyTrue, FxAppMemCorruptCb);

    /* Initialize the device */
    status = CyU3PDeviceInit (NULL);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /* Initialize the caches. Enable both Instruction and Data Caches. */
    status = CyU3PDeviceCacheControl (CyTrue, CyTrue, CyTrue);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    /*
     * GRL FW Application Initialization,
     * Buffers, Structures for configuration and control data will be initialized here
     */
    InitializeFWApplication();
    /* Configure the IO matrix for the device. On the FX3 DVK board, the COM port 
     * is connected to the IO(53:56). This means that either DQ32 mode should be
     * selected or lppMode should be set to UART_ONLY. Here we are choosing
     * UART_ONLY configuration. */

    CyU3PMemSet ((uint8_t *)&io_cfg, 0, sizeof(io_cfg));
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyTrue;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    /* No GPIOs are enabled. */
    io_cfg.gpioSimpleEn[0]  = 0x00200000;
    io_cfg.gpioSimpleEn[1]  = 0;
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    status = CyU3PDeviceConfigureIOMatrix (&io_cfg);
    if (status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }
     /* This is a non returnable call for initializing the RTOS kernel */
    CyU3PKernelEntry ();

    /* Dummy return to make the compiler happy */
    return 0;

handle_fatal_error:

    /* Cannot recover from this error. */
    while (1);
}

/* [ ] */

