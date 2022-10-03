/*
 * PDManufacturerPeripheral.c
 *
 *  Created on: 05-Apr-2019
 *      Author: Lenovo
 */


#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3gpio.h"
#include "cyu3spi.h"
#include "cyu3uart.h"
#include "cyu3utils.h"
#include "gpio_regs.h"
#include <stddef.h>
#include <spi_regs.h>
#include "PDManufacturerDef.h"
#include "PDManufacturerStruct.h"

gFunctStruct * gFunctStruct_t;

void RS485WriteConfig()
{
	uint8_t lWriteBuf[8] = {0};

	lWriteBuf[0] = RS485_WRITE_CONFIG >> 8;
	lWriteBuf[1] = RS485_WRITE_CONFIG & 0xFF;

	CyFxSpiTransfer (1, 2, lWriteBuf, NULL, WRITE);
}

void RS485ReadConfig()
{
	uint8_t lWriteBuf[8] = {0};

	lWriteBuf[0] = RS485_READ_CONFIG >> 8;
	lWriteBuf[1] = RS485_READ_CONFIG & 0xFF;

	CyFxSpiTransfer (1, 2, lWriteBuf, NULL, WRITE);
}

uint8_t SPIDataRxRegRead()
{
	uint8_t readBuffer[8] = {0};

	SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line Low
	readBuffer[0] = RS485_READ_DATA;
	RecvSpiWords (&readBuffer[0], 2);
	SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line High

	return (readBuffer[0] & READ_RS485_DATA_REG_SUCCESS_VAL);
}

CyU3PReturnStatus_t
RS485SpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        CyBool_t  isRead)
{
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	uint8_t i = 0;
	uint8_t lWriteBuf[8] = {0};
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();
	uint8_t lRetryCount =0;
	if(isRead)
	{
		lWriteBuf[0] = RS485_READ_DATA;
		for(i = 0; i < byteCount; i++)
		{
			//lWriteBuf[1] = buffer[i];
			CyFxSpiTransfer (byteAddress, 2, lWriteBuf, readBuffer, isRead);
		}
	}
	else
	{
		MsgTimerStart(Timer_RS485DevInit,TIMER2);  // start the RS485 init timer
//		RS485ISN_DriveEnableHandler(CyTrue);
		lWriteBuf[0] = RS485_WRITE_DATA >> 8;
		for(i = 0; i < byteCount; ++i)
		{
			lWriteBuf[1] = buffer[i];
			do
			{
				status = CyFxSpiTransfer (byteAddress, 2, lWriteBuf, NULL, isRead);
			}while((status != CY_U3P_SUCCESS) && (lRetryCount++ < 2));

			status = CY_U3P_SUCCESS;lRetryCount = 0;

			CyFx3BusyWait(200);
		}
		if(byteCount == i)
		{
			MsgTimerStop(TIMER2);       // stop timer 2
#if 0
			/***Pranay,19May'21, After write operation of n bytes, reading the rs485 data reg. and verifying whether
			 * transmit buffer (b14)is full (return 0 in b14) or empty (return 1 in b14). if full retry for 2 times and reset rs485 if not empty even after retry.
			 * */
			lRetryCount =0;lReadRegData =0;
			do
			{
				lReadRegData = SPIDataRxRegRead();

			}while(lReadRegData != READ_RS485_DATA_REG_SUCCESS_VAL && lRetryCount++ < 2 );

			if(lReadRegData != READ_RS485_DATA_REG_SUCCESS_VAL && lRetryCount++ >= 2 )
			{
				gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;
				gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;
		    	RS485DeviceInit();
			}
			else
#endif
				RS485WriteDataDriverDisable();
		}
	}

	return status;
}

void RS485WriteDataDriverDisable()
{
	uint8_t lWriteBuf[8] = {0};

//	RS485ISN_DriveEnableHandler(CyFalse);

	lWriteBuf[0] = RS485_WRITE_DATA_DE_DISABLE >> 8;
	lWriteBuf[1] = RS485_WRITE_DATA_DE_DISABLE & 0xFF;

	CyFxSpiTransfer (1, 2, lWriteBuf, NULL, WRITE);
}

#if 0
CyU3PReturnStatus_t
Fx3I2CTransfer(
        uint16_t  byteAddress,
        uint8_t   devAddr,
        uint16_t  byteCount,
        uint8_t  *Buffer,
        CyBool_t  isRead)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	if(isRead)//Reads Data from CCG3PA I2C into the i2cRx.Buf
		CyFxUsbI2cTransfer(byteAddress,devAddr,128,Buffer,1);
	else
	{
		CyFxUsbI2cTransfer(byteAddress,devAddr,byteCount,Buffer,0);
	}
	return status;
}
#endif

void RS485DeviceInit()
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

	gFunctStruct_t->gFwHandle_t.gSystemConfig_t.gRS485ByteCount = RS485_DATA_MAX_COUNT;
	gFunctStruct_t->gFwHandle_t.gSystemControl_t.gRS485RxIntrCount = 0;

	RS485WriteConfig();

	RS485ReadConfig();

	RS485WriteDataDriverDisable();
}

#if 0
/*
 * Aserts / deasserts the SSN line.
 */
CyU3PReturnStatus_t
MasterSpiSetSsnLine (CyBool_t isHigh)
{
    if (isHigh)
    {
        SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;
    }
    else
    {
        SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;
    }

    return CY_U3P_SUCCESS;
}
#endif

/*
 * Resets the FIFO.
 * Leaves SPI block disabled at the end.
 */
CyU3PReturnStatus_t
GpioSpiResetFifo (
                   CyBool_t isTx,
                   CyBool_t isRx
                   )
{
	uint32_t intrMaskRstFifo = 0, ctrlMaskRstFifo = 0, tempRstFifo = 0;
    /* No lock is acquired or error checked as this is
    * an internal function. Locks need to be acquired
    * prior to this call. */

    /* Temporarily disable interrupts. */
    intrMaskRstFifo = SPI->lpp_spi_intr_mask;
    SPI->lpp_spi_intr_mask = 0;

    if (isTx)
    {
        ctrlMaskRstFifo = CY_U3P_LPP_SPI_TX_CLEAR;
    }
    if (isRx)
    {
        ctrlMaskRstFifo |= CY_U3P_LPP_SPI_RX_CLEAR;
    }

    /* Disable the SPI block and reset. */
    tempRstFifo = ~(CY_U3P_LPP_SPI_RX_ENABLE | CY_U3P_LPP_SPI_TX_ENABLE |
        CY_U3P_LPP_SPI_DMA_MODE | CY_U3P_LPP_SPI_ENABLE);
    SPI->lpp_spi_config &= tempRstFifo;
    while ((SPI->lpp_spi_config & CY_U3P_LPP_SPI_ENABLE) != 0);

    /* Clear the FIFOs and wait until they have been cleared. */
    SPI->lpp_spi_config |= ctrlMaskRstFifo;
    if (isTx)
    {
        while ((SPI->lpp_spi_status & CY_U3P_LPP_SPI_TX_DONE) == 0);
    }
    if (isRx)
    {
        while ((SPI->lpp_spi_status & CY_U3P_LPP_SPI_RX_DATA) != 0);
    }
    SPI->lpp_spi_config &= ~ctrlMaskRstFifo;

    /* Clear all interrupts and re-enable them. */
    SPI->lpp_spi_intr |= CY_U3P_LPP_SPI_TX_DONE;
    SPI->lpp_spi_intr_mask = intrMaskRstFifo;

    return CY_U3P_SUCCESS;
}

/*
 * Transmits data word by word over
 * the SPI interface.
 */
CyU3PReturnStatus_t
SendSpiWords (
                       uint8_t *data,
                       uint32_t byteCount)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    uint32_t glSpiWriteTimeout = CY_U3P_SPI_TIMEOUT1; /* Timeout for SPI write transfers. */
    uint8_t  wordLenTxWords = 1;
    uint32_t intrMaskTxWords = 0, tempTxWords = 0, timeoutTxWords = 0, i = 0;

    /* Get the wordLen from the config register and convert it to byte length. */
    tempTxWords = SPI->lpp_spi_config;
    wordLenTxWords = ((tempTxWords & CY_U3P_LPP_SPI_WL_MASK) >> CY_U3P_LPP_SPI_WL_POS);
    if ((wordLenTxWords & 0x07) != 0)
    {
        wordLenTxWords = (wordLenTxWords >> 3) + 1;
    }
    else
    {
    wordLenTxWords = (wordLenTxWords >> 3);
    }
    if ((byteCount % wordLenTxWords) != 0)
    {
        status = CY_U3P_ERROR_BAD_ARGUMENT;
    }

    /* Check if the DMA mode is running. */
    if ((timeoutTxWords & (CY_U3P_LPP_SPI_DMA_MODE | CY_U3P_LPP_SPI_ENABLE))
        == (CY_U3P_LPP_SPI_DMA_MODE | CY_U3P_LPP_SPI_ENABLE))
    {
        status = CY_U3P_ERROR_ALREADY_STARTED;
    }

    GpioSpiResetFifo (CyTrue, CyFalse);

    /* Disable interrupts. */
    intrMaskTxWords = SPI->lpp_spi_intr_mask;
    SPI->lpp_spi_intr_mask = 0;

    /* Enable the TX. */
    SPI->lpp_spi_config |= CY_U3P_LPP_SPI_TX_ENABLE;

    /* Re-enable SPI block. */
    SPI->lpp_spi_config |= CY_U3P_LPP_SPI_ENABLE;
    timeoutTxWords = glSpiWriteTimeout;

    for (i = 0; i < byteCount; i += wordLenTxWords)
    {
        /* Wait for the tx_space bit in status register */
        while (!(SPI->lpp_spi_status & CY_U3P_LPP_SPI_TX_SPACE))
        {
            if (timeoutTxWords-- == 0)
            {
                status = CY_U3P_ERROR_TIMEOUT;
                break;
            }
        }

        if (status != CY_U3P_SUCCESS)
        {
            break;
        }

        /* Copy data to be written into local variable.
        * The padding required is to nearest byte.
        * Do fallthrough switch instead of a loop. */
        tempTxWords = 0;
        switch (wordLenTxWords)
        {
        case 4:
            tempTxWords = (data[i + 3] << 24);
        break;
        case 3:
            tempTxWords |= (data[i + 2] << 16);
            break;
        case 2:
            tempTxWords |= (data[i + 1] << 8);
            break;
        case 1:
            tempTxWords |= data[i];
            break;
        default:
            break;
        }

        SPI->lpp_spi_egress_data = tempTxWords;

        /* wait for the TX_DONE */
        while (!(SPI->lpp_spi_intr & CY_U3P_LPP_SPI_TX_DONE))
        {
            if (timeoutTxWords-- == 0)
            {
                status = CY_U3P_ERROR_TIMEOUT;
                break;
            }
        }

        if (status != CY_U3P_SUCCESS)
        {
            break;
        }

        /* Clear the TX_DONE interrupt. */
        SPI->lpp_spi_intr = CY_U3P_LPP_SPI_TX_DONE;
    }

    /* Disable the TX. */
    SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_TX_ENABLE;

    /* Clear all interrupts and restore interrupt mask. */
    SPI->lpp_spi_intr |= CY_U3P_LPP_SPI_TX_DONE;
    SPI->lpp_spi_intr_mask = intrMaskTxWords;

    /* Wait until the SPI block is no longer busy and disable. */
    while ((SPI->lpp_spi_status & CY_U3P_LPP_SPI_BUSY) != 0);
    SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_ENABLE;

    return status;
}

/*
 * Receive data word by word over the SPI interface.
 */
CyU3PReturnStatus_t
RecvSpiWords (
                      uint8_t *data,
                      uint32_t byteCount)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint32_t glSpiReadTimeout = CY_U3P_SPI_TIMEOUT1;  /* Timeout for SPI read transfers. */
    uint8_t  wordLenRxWords;
    uint32_t intrMaskRxWords, tempRxWords, timeoutRxWords, i;

    /* Get the wordLen from the config register and convert it to byte length. */
    tempRxWords = SPI->lpp_spi_config;
    wordLenRxWords = ((tempRxWords & CY_U3P_LPP_SPI_WL_MASK) >> CY_U3P_LPP_SPI_WL_POS);
        wordLenRxWords = (wordLenRxWords >> 3);
    GpioSpiResetFifo (CyTrue, CyTrue);

    /* Disable interrupts. */
    intrMaskRxWords = SPI->lpp_spi_intr_mask;
    SPI->lpp_spi_intr_mask = 0;

    /* Enable TX and RX. */
    SPI->lpp_spi_config |= (CY_U3P_LPP_SPI_TX_ENABLE | CY_U3P_LPP_SPI_RX_ENABLE);

    /* Re-enable SPI block. */
    SPI->lpp_spi_config |= CY_U3P_LPP_SPI_ENABLE;
    timeoutRxWords = glSpiReadTimeout;

    for (i = 0; i < byteCount; i += wordLenRxWords)
    {
        /* Transmit zero. */
        SPI->lpp_spi_egress_data = 0;

        /* wait for the TX_DONE and RX_DATA. */
        tempRxWords = CY_U3P_LPP_SPI_RX_DATA | CY_U3P_LPP_SPI_TX_DONE;
        while ((SPI->lpp_spi_intr & tempRxWords) != tempRxWords)
        {
            if (timeoutRxWords-- == 0)
            {
                status = CY_U3P_ERROR_TIMEOUT;
                break;
            }
        }
        /* Clear interrupt bits. */
        SPI->lpp_spi_intr = tempRxWords;

        /* Copy the data from the fifo. The padding
        * required is to nearest byte. Do fallthrough
        * switch instead of a loop. */
        tempRxWords = SPI->lpp_spi_ingress_data;
        switch (wordLenRxWords)
        {
        case 4:
            data[i + 3] = (uint8_t)((tempRxWords >> 24) & 0xFF);
            break;
        case 3:
            data[i + 2] = (uint8_t)((tempRxWords >> 16) & 0xFF);
            break;
        case 2:
            data[i + 1] = (uint8_t)((tempRxWords >> 8) & 0xFF);
            break;
        case 1:
            data[i] = (uint8_t)(tempRxWords & 0xFF);
            break;
        default:
            break;
        }
    }

    /* Disable the TX and RX. */
    SPI->lpp_spi_config &= ~(CY_U3P_LPP_SPI_TX_ENABLE | CY_U3P_LPP_SPI_RX_ENABLE);

    /* Clear all interrupts and restore interrupt mask. */
    SPI->lpp_spi_intr |= CY_U3P_LPP_SPI_TX_DONE;
    SPI->lpp_spi_intr_mask = intrMaskRxWords;

    /* Wait until the SPI block is no longer busy and disable. */
    while ((SPI->lpp_spi_status & CY_U3P_LPP_SPI_BUSY) != 0);
    SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_ENABLE;

    return status;
}


/*void RS485DeviceValidation()
{
	uint8_t lWriteBuf[8] = {0};
	uint8_t lReadBuf[8] = {0};

	lReadBuf[0] = RS485_READ_CONFIG >> 8;
	lReadBuf[1] = RS485_READ_CONFIG & 0xFF;

	do
	{
		//CyFxSpiTransfer (1, 2, lWriteBuf, lReadBuf, READ);
        SPI->lpp_spi_config &= ~CY_U3P_LPP_SPI_SSN_BIT;
		RecvSpiWords (&lReadBuf[0], 2);
        SPI->lpp_spi_config |= CY_U3P_LPP_SPI_SSN_BIT;	//Set SS Line High

	}while(lReadBuf[0] != (0xCC));

}*/

/*
 * GPIO Configuration
 * ioDirection : inputEnable :True->Input,False->Output
 */
void GpioConfigure (uint8_t gpioNo,CyBool_t lOutValue,CyBool_t lDriveLowEn,CyBool_t lDriveHighEn,CyBool_t ioDirection,
					CyBool_t lIntrEnable, CyBool_t lResPullUpDown)
{
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    /* Setting Run time GPIO
     *
     *
     *
     */
    apiRetStatus = CyU3PDeviceGpioOverride (gpioNo, CyTrue);
    if (apiRetStatus != 0)
    {
        /* Error Handling */
        CyU3PDebugPrint (4, "%d:CyU3PDeviceGpioOverride failed, error code = %d\n",
        		gpioNo,apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }
    /**pranay,24May'21,If the pin is INPUT configured than dont assert any pull-up/down*/
    if(ioDirection == CyTrue)
    {
		apiRetStatus = CyU3PGpioSetIoMode (gpioNo, CY_U3P_GPIO_IO_MODE_NONE);
    }
    else
    {
		if(lResPullUpDown)
			apiRetStatus = CyU3PGpioSetIoMode (gpioNo, CY_U3P_GPIO_IO_MODE_WPU);
		else
			apiRetStatus = CyU3PGpioSetIoMode (gpioNo, CY_U3P_GPIO_IO_MODE_WPD);
	}

    /* Configure GPIO gpioNo as output */
    gpioConfig.outValue = lOutValue;
    gpioConfig.driveLowEn = lDriveLowEn;
    gpioConfig.driveHighEn = lDriveHighEn;
    gpioConfig.inputEn = ioDirection;
    if(!lIntrEnable)
    	gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    else
    	{
    	gpioConfig.intrMode = CY_U3P_GPIO_INTR_NEG_EDGE;
    	if(gpioNo == GPIO44_VBUS_VBUS_DETECT)
        	gpioConfig.intrMode = CY_U3P_GPIO_INTR_BOTH_EDGE;
    	}
    apiRetStatus = CyU3PGpioSetSimpleConfig(gpioNo, &gpioConfig);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        /* Error handling */
        CyU3PDebugPrint (4, "%d_1:CyU3PGpioSetSimpleConfig failed, error code = %d\n",
        		gpioNo,apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus);
    }

}

CyU3PReturnStatus_t
CyFxGpioInit (void)
{
	CyU3PGpioClock_t gpioClock;
//	CyU3PGpioSimpleConfig_t gpioConfig;
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

	/* Init the GPIO module */
	gpioClock.fastClkDiv = 2;
	gpioClock.slowClkDiv = 0;
	gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
	gpioClock.clkSrc = CY_U3P_SYS_CLK;
	gpioClock.halfDiv = 0;

	apiRetStatus = CyU3PGpioInit(&gpioClock, CyFxGpioIntrCb);
	if (apiRetStatus != 0)
	{
		/* Error Handling */
		CyU3PDebugPrint (4, "CyU3PGpioInit failed, error code = %d\n", apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}
	/*
	 * GPIO Configuration is done here
	 */
//	GpioConfigure (GPIO0_INT_TO_FX3,CyFalse, CyFalse,CyFalse,CyTrue,CyTrue,CyTrue);// 2nd Par Def. low state	i/p
	GpioConfigure (GPIO0_INT_TO_FX3,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);// 2nd Par Def. low state	i/p (Disable)
	GpioConfigure (GPIO1_INT_TO_CCG3PA,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);// 2ndPar. Def. low state o/p
	GpioConfigure (GPIO2_LED1_BI_A,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO3_CC1_SW1,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO50_LED1_BI_C,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);	/*NOTE : This Should be configured as Complex GPIO*/
	GpioConfigure (GPIO4_LED2_BI_A,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO5_LED2_BI_C,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO6_LED3_BI_A,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO7_LED3_BI_C,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO8_LED4_BI_A,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO9_LED4_BI_C,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO10_LED1_S_C,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO11_LED2_S_C,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO12_LED3_S_C,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO13_LED4_S_C,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO14_SEL0_DP_SW,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO15_DP_AUX_4,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);   // default the CC1 TX-RX switch is selected.
	GpioConfigure (GPIO17_F_IO_ARD_FX3,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	//GpioConfigure (GPIO17_F_IO_ARD_FX3,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO18_SEL_CC2_ELOAD_0,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO19_SEL_CC1_ELOAD_0,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO20_MASTER_RESET_FX3,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO21_RS485_IRQ,CyTrue, CyFalse,CyFalse,CyTrue,CyTrue,CyTrue);
	GpioConfigure (GPIO22_ERASE_CMD_FX3,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO23_I2C_INT_CCG4,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO24_VBUS_SHORT,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO25_CC2_SW2,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
//	Tri-Stated
	GpioConfigure (GPIO26_B_ID0,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO27_B_ID1,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO28_B_ID2,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO29_B_ID3,CyFalse, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO30_PMODE0,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	/**PPS GPIOs : DQ16, DQ17, DQ18*/
	GpioConfigure (GPIO33_PPS_PGM_MODE_SELECTION,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);//Default mode should be in Boot mode =1

	GpioConfigure (GPIO34_PPS_GPIO2,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	//Pranay,15March'22.IO expander Input
	GpioConfigure (GPIO35_PPS_RESET,CyTrue, CyTrue,CyTrue,CyTrue,CyTrue,CyTrue);

	/***/
	GpioConfigure (GPIO36_VCONN_SHORT,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO37_DEBUG_LED_1,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO38_DEBUG_LED_2,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO39_LOOPBACK_LOCK_STATE,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO40_LOOPBACK_VBUS_LOCK,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO43_VBUS_DATA_ERROR,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO48_UART_TX,CyTrue, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO49_UART_RX,CyTrue, CyTrue,CyTrue,CyTrue,CyFalse,CyTrue);
	//GpioConfigure (GPIO41_CCG4_EN1,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO41_CC1_RA,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	GpioConfigure (GPIO42_CC2_RA,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	//GpioConfigure (GPIO42_CCG4_EN2,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);
	//GpioConfigure (GPIO44_VBUS_VBUS_DETECT,CyFalse, CyTrue,CyTrue,CyTrue,CyFalse,CyTrue);
	GpioConfigure (GPIO45_VBUS_SENSE_VOLT_EN,CyFalse, CyTrue,CyTrue,CyFalse,CyFalse,CyTrue);

//	GpioConfigure (GPIO44_VBUS_VBUS_DETECT,CyTrue, CyFalse,CyFalse,CyTrue,CyTrue,CyTrue);
	GpioConfigure (GPIO44_VBUS_VBUS_DETECT,CyTrue, CyFalse,CyFalse,CyTrue,CyFalse,CyTrue);


#if 0
	apiRetStatus = CyU3PGpioSetValue (GPIO0_PDSS_INTR, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO1_PDSS_INTR_SET, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO2_USB_ENUMERATION_DONE_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO50_USB_DATA_TXN_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO4_USB_HIGH_SPEED_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO5_USB_SUPER_SPEED_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO6_PD_MODE_ENABLE_LED, 1);
	apiRetStatus = CyU3PGpioSetValue (GPIO7_BC_1_2_MODE_ENABLE_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO8_USB_DATA_ERROR_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO9_USB_DATA_NORMAL_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO10_POWER_INDICATION_LED, 1);
	apiRetStatus = CyU3PGpioSetValue (GPIO11_VBUS_DETECT_LED, 1);
	apiRetStatus = CyU3PGpioSetValue (GPIO12_USB_DATA_LOCK_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO13_TYPE_C_FLIP_LED, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO14_USB_DP_DM_SEL_SW, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO15_DP_AUX_SEL_SW, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO18_CC2_ELOAD_SEL_SW, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO19_CC1_ELOAD_SEL_SW, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO20_ELOAD_FW_UPDATE_RESET, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO21_RS485_IRQ, 1);
	apiRetStatus = CyU3PGpioSetValue (GPIO22_ELOAD_FW_UPDATE_ERASE, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO23_SWD_CLK, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO24_SWD_DATA, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO25_FW_DEBUG_OUT, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO26_FID_DETECTION_IO_1, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO27_FID_DETECTION_IO_2, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO28_FID_DETECTION_IO_3, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO29_FID_DETECTION_IO_4, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO33_FREE_IO_1, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO34_FREE_IO_2, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO35_FREE_IO_3, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO36_FREE_IO_4, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO37_DEBUG_LED_1, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO38_DEBUG_LED_2, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO39_USBDATA_LOOPBACK_LOCK, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO40_USB_VBUS_LOCK, 0);
	apiRetStatus = CyU3PGpioSetValue (GPIO43_USB_DATA_ERROR, 0);
#endif
	return apiRetStatus;
}

/* SPI read / write for programmer application. */
CyU3PReturnStatus_t
CyFxSpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        CyBool_t  isRead)
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }
        if (isRead) /*Read*/
        {
			CyU3PSpiSetSsnLine (CyFalse);
				status = CyU3PSpiReceiveWords (&readBuffer[0], byteCount);
				if (status != CY_U3P_SUCCESS)
				{
					CyU3PSpiSetSsnLine (CyTrue);
					return status;
				}
			CyU3PSpiSetSsnLine (CyTrue);

        }

        else /*Write*/
        {
            CyU3PSpiSetSsnLine (CyFalse);
            status = CyU3PSpiTransmitWords (buffer, byteCount);
//            status = SendSpiWords(buffer, byteCount);
            if (status != CY_U3P_SUCCESS)
            {
                CyU3PSpiSetSsnLine (CyTrue);
                return status;
            }

            CyU3PSpiSetSsnLine (CyTrue);
        }
    return CY_U3P_SUCCESS;
}


/* SPI initialization for application. */
CyU3PReturnStatus_t
CyFxSpiInit (uint16_t pageLen)
{
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
#if 0
    /* Start the SPI module and configure the master. */
    status = CyFxGpioInit();
    if (status != CY_U3P_SUCCESS)
    {
    	 CyU3PDebugPrint (2, "CyFxGpioInit Failed\r\n");
        return status;
    }
#endif
    /* Start the SPI module and configure the master. */
    status = CyU3PSpiInit();
    if (status != CY_U3P_SUCCESS)
    {
//    	DEBUG_LOG(DBG1, 0xE7, status);
    	CyU3PDebugPrint (2, "CyU3PSpiInit Failed\r\n");
        return status;
    }

    /* Start the SPI master block. Run the SPI clock at 4MHz
     * and configure the word length to 8 bits. Also configure
     * the slave select using FW. */
    CyU3PMemSet ((uint8_t *)&spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    //spiConfig.cpol       = CyTrue;
    spiConfig.cpol       = CyFalse;
    spiConfig.ssnPol     = CyFalse;
    //spiConfig.cpha       = CyTrue;
    spiConfig.cpha       = CyFalse;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 100000;//16000000;
    spiConfig.wordLen    = 8;

    status = CyU3PSpiSetConfig (&spiConfig, NULL);
    if (status != CY_U3P_SUCCESS)
    {
//    	DEBUG_LOG(DBG1, 0xE8, status);
    	CyU3PDebugPrint (2, "CyU3PSpiSetConfig Failed\r\n");
        return status;
    }
    return status;
}

/* I2C read / write for programmer application. */
CyU3PReturnStatus_t
CyFxUsbI2cTransfer (
        uint16_t  byteAddress,
        uint8_t   devAddr,
        uint16_t  byteCount,
        uint8_t  *buffer,
        CyBool_t  isRead)
{
	gFunctStruct_t = (gFunctStruct *)GetStructPtr();

    CyU3PI2cPreamble_t preamble;
    uint16_t glI2cPageSize = 128;
    uint16_t pageCount = (byteCount / glI2cPageSize);
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    uint16_t resCount = glI2cPageSize;
    //int8_t debugData[250] = "";

    if (byteCount == 0)
    {
        return CY_U3P_SUCCESS;
    }

    if ((byteCount % glI2cPageSize) != 0)
    {
        pageCount ++;
        resCount = byteCount % glI2cPageSize;
    }

    uint8_t inpDevAddress = devAddr;
    while (pageCount != 0)
    {

        inpDevAddress = devAddr;

        if( byteAddress > 0xFFFF )
        {
        	inpDevAddress += 2;
        }

        if (isRead)
        {
        	switch(devAddr)
        	{
        	case FRAM_SLAVEADDR:
        	case I2C_DEV_ID_MEGACHIP:
                preamble.buffer[0] = inpDevAddress;
                preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
                preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
                preamble.buffer[3] = (devAddr | 0x01);
                preamble.length    = 4;
//                preamble.ctrlMask  = 0x0002;
                preamble.ctrlMask  = 0x0004;
        	break;
        	case I2C_DEV_ID_MEGACHIP_RD:
                preamble.length    = 1;
                preamble.buffer[0] = inpDevAddress | 1;
                preamble.ctrlMask  = 0x0002;
        		break;
        	default:
                /* Update the preamble information. */
                 preamble.length    = 1;
                 preamble.buffer[0] = inpDevAddress | 1;
              //   preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
                 preamble.buffer[1] = (uint8_t)(byteAddress & 0xFF);
                 preamble.buffer[2] = (devAddr | 0x01);
                 preamble.ctrlMask  = 0x0002;
                 break;
        	}
            status = CyU3PI2cReceiveBytes (&preamble, buffer, byteCount, 0);
//          status = CyU3PI2cReceiveBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
        }
        else /* Write */
        {
        	switch(devAddr)
        	{
        	case FRAM_SLAVEADDR:
        		preamble.length    = 3;
				preamble.buffer[0] = inpDevAddress | 0;
				preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
				preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
				preamble.ctrlMask  = 0x0000;
				break;
        	case I2C_DEV_ID_MEGACHIP:
        		preamble.length    = 1;//preamble.length    = 3;
				preamble.buffer[0] = inpDevAddress | 0;
				preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
				preamble.buffer[2] = (uint8_t)(byteAddress & 0xFF);
				preamble.ctrlMask  = 0x0000;
        	break;
        	default:
                /* Update the preamble information. */
                 preamble.length    = 1;
                 preamble.buffer[0] = inpDevAddress | 0;
     //            preamble.buffer[1] = (uint8_t)(byteAddress >> 8);
                 preamble.buffer[1] = (uint8_t)(byteAddress & 0xFF);
     //            preamble.buffer[2] = (devAddr | 0x01);
                 preamble.ctrlMask  = 0x0000;
        		break;
        	}

#if 0//Working
            status = CyU3PI2cTransmitBytes (&preamble, buffer, (pageCount == 1) ? resCount : glI2cPageSize, 0);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
#endif
//#if 0
            status = CyU3PI2cTransmitBytes (&preamble, buffer, byteCount, 0);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
//#endif
            /* Wait for the write to complete. */
#if 0
            preamble.length = 1;
            status = CyU3PI2cWaitForAck(&preamble, 200);
            if (status != CY_U3P_SUCCESS)
            {
                return status;
            }
#endif
        }

        /* An additional delay seems to be required after receiving an ACK. */
        CyU3PThreadSleep (1);

        /* Update the parameters */
        byteAddress  += glI2cPageSize;
        buffer += glI2cPageSize;
        pageCount --;
    }
#ifdef DEBUG_PRINT
    CyU3PDebugPrint (2, "I2C access1 - dev: 0x%x, address: 0x%x, size: 0x%x, pages: 0x%x, resCount: 0x%x. Data: 0x%x\r\n",
            devAddr, byteAddress, byteCount, pageCount,resCount,buffer[0]);
#endif
    return CY_U3P_SUCCESS;
}


CyU3PReturnStatus_t
CyFxI2cInit (uint16_t pageLen)
{
    CyU3PI2cConfig_t i2cConfig;
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;

    /* Initialize and configure the I2C master module. */
    status = CyU3PI2cInit ();
    if (status != CY_U3P_SUCCESS)
    {
        return status;
    }

    /* Start the I2C master block. The bit rate is set at 100KHz.
     * The data transfer is done via DMA. */
    CyU3PMemSet ((uint8_t *)&i2cConfig, 0, sizeof(i2cConfig));
    i2cConfig.bitRate    = CY_FX_USBI2C_I2C_BITRATE_100KHZ;/*CY_FX_USBI2C_I2C_BITRATE_100KHZ;*//*CY_FX_USBI2C_I2C_BITRATE_400KHZ*/
    i2cConfig.busTimeout = 0xFFFFFFFF;
    i2cConfig.dmaTimeout = 0xFFFF;
    i2cConfig.isDma      = CyFalse;

    status = CyU3PI2cSetConfig (&i2cConfig, NULL);
    if (status == CY_U3P_SUCCESS)
    {
//        glI2cPageSize = pageLen;
        return status;
    }

    return status;
}
