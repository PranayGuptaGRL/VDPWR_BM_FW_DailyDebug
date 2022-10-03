/*
 * PDManufacturerPeripheral.h
 *
 *  Created on: Jun 13, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERGPIO_H_
#define PDMANUFACTURERGPIO_H_


/*######################GPIO's are Define Here#######################*/
#define GPIO0_INT_TO_FX3					0

#define GPIO1_INT_TO_CCG3PA					1

#define GPIO2_LED1_BI_A						2		//GPIO2_USB_ENUMERATION_DONE_LED

#define GPIO3_CC1_SW1                       3

#define GPIO50_LED1_BI_C			  		50

#define GPIO4_LED2_BI_A						4

#define GPIO5_LED2_BI_C						5

#define GPIO6_LED3_BI_A						6

#define GPIO7_LED3_BI_C						7

#define GPIO8_LED4_BI_A						8

#define GPIO9_LED4_BI_C						9

#define GPIO10_LED1_S_C						10

#define GPIO11_LED2_S_C						11

#define GPIO12_LED3_S_C						12

#define GPIO13_LED4_S_C						13

#define GPIO14_SEL0_DP_SW					14		/*Switch for  Selecting DP and DM signals to FX3 or CCg3PA*/

#define GPIO15_DP_AUX_4						15		/*Switch for  Selecting Rx/Tx signals orientation based on Active CC*/

#define GPIO17_F_IO_ARD_FX3					17		/* Status IO for for Eload Bootedup/not */

#define GPIO18_SEL_CC2_ELOAD_0				18		/*CC2 e-load selection*/

#define GPIO19_SEL_CC1_ELOAD_0				19		/*CC1 e-load selection*/

#define GPIO20_MASTER_RESET_FX3				20		/*Master Reset control to e-load FW update via FX3's UART*/

//#define GPIO21_RS485_RECV_INTR				21		/*RS485 Data Receive Interrupt*/

#define GPIO21_RS485_IRQ					21

#define GPIO22_ERASE_CMD_FX3				22		/*Erase control to e-load FW update via FX3's UART*/

#define GPIO23_I2C_INT_CCG4					23		/*FX3's SWD Clock pin for CCG3PA FW update using SWD Programming*/

#define GPIO24_VBUS_SHORT					24

//#define GPIO25_FW_DEBUG_OUT_NC				25		/*FX3 FW Debug Out Pin*/

#define GPIO25_CC2_SW2						25

#define GPIO26_B_ID0						26		/*Functional Card ID Detection Input I/O's, Based On slot FID is detected*/

#define GPIO27_B_ID1						27		/*Functional Card ID Detection Input I/O's, Based On slot FID is detected*/

#define GPIO28_B_ID2						28		/*Functional Card ID Detection Input I/O's, Based On slot FID is detected*/

#define GPIO29_B_ID3						29		/*Functional Card ID Detection Input I/O's, Based On slot FID is detected*/

#define GPIO30_PMODE0						30

/**IOs from Fx3 to TI's PPS Module*/
#define GPIO33_PPS_PGM_MODE_SELECTION		33		/** I/O's to PPS TI Controller*/

#define GPIO34_PPS_GPIO2					34		/** I/O's to PPS TI Controller*/

#define GPIO35_PPS_RESET					35		/** I/O's to PPS TI Controller*/
/***/
//#define GPIO36_FREE_IO_4					36		/*Free I/O's to backplane from FX3*/

#define GPIO36_VCONN_SHORT                  36      /*Vconn short circuit test io*/

#define GPIO37_DEBUG_LED_1					37

#define GPIO38_DEBUG_LED_2					38

#define GPIO39_LOOPBACK_LOCK_STATE			39		/*USB Data Loop Back is enabled for DAQ System*/

#define GPIO40_LOOPBACK_VBUS_LOCK			40		/*USB Vbus is present for DAQ System*/

#define GPIO43_VBUS_DATA_ERROR 				43

#define GPIO48_UART_TX 						48

#define GPIO49_UART_RX 						49
/*1*/
//#define GPIO41_CCG4_EN1 					41

#define GPIO41_CC1_RA 						41

//#define GPIO42_CCG4_EN2 					42

#define GPIO42_CC2_RA                       42

#define GPIO44_VBUS_VBUS_DETECT				 44

#define GPIO45_VBUS_SENSE_VOLT_EN 			 45

/*NW*/
//#define GPIO53_SCLK_C 53
//#define GPIO54_RS485_CS_C 54
/**/

/*NW*/
//#define GPIO55_MISO_C 55
//#define GPIO56_MOSI_C 56
/**/

/**/
//#define GPIO58_M_SCL_FX3 58
//#define GPIO59_M_SDA_FX3 59
/**/

#define CY_FX_GPIOAPP_GPIO_HIGH_EVENT    (1 << 0)   /* GPIO high event */
#define CY_FX_GPIOAPP_GPIO_LOW_EVENT     (1 << 1)   /* GPIO low event */
#define CY_FX_GPIOAPP_GPIO_HIGH_EVENT_1  (1 << 2)   /* GPIO high event 1*/
#define CY_FX_GPIOAPP_GPIO_HIGH_EVENT_2  (1 << 3)   /* GPIO high event 2*/


/*****************************GPIO Peripheral Specfic for Register Access***********/
#define CY_U3P_LPP_GPIO_SIMPLE_ADDRESS(n) (uvint32_t*)(0xe0001100 + ((n)*(0x0004)))
//#define CY_U3P_LPP_GPIO_INTR (1<<27) /* Do not touch the interrupt bit */
#define OUT_VALUE 0x01 /* The 0th bit is the value that is output on the pins */
#define USB_TXN_LED_SET 0x00000001 /* The 0th bit is the value that is output on the pins */
#define REG_GPIO_OUT_VALUE 0x00000001 /* The 0th bit is the value that is output on the pins */
#define USB_TXN_LED_CLEAR 0x00 /* The 0th bit is the value that is output on the pins */

/*######################GPIO Definition Ends Here#######################*/

/*######################RS485 Data are Define Here#######################*/
#define RS485_WRITE_CONFIG 				0xC400
#define RS485_READ_CONFIG				0x4000
#define RS485_WRITE_DATA				0x8000
#define RS485_READ_DATA					0x0000
#define RS485_WRITE_DATA_DE_DISABLE	    0x8600
#define RS485_UART_SHUTDOWN             0xD400

#define RS485_DATA_AVAIL_MASK			0x80

#define RS485_DATA_MAX_COUNT			0xFF
/*######################RS485 Definition Ends Here#######################*/

/*######################SPI is Define Here#######################*/
#define CY_U3P_SPI_TIMEOUT1              (0xFFFFF)               /* Default timeout for SPI data transfers. */
#define SPI_INVALID_ADDR   91
/*######################SPI Definition Ends Here#######################*/

#define CY_FX_USBI2C_I2C_BITRATE_100KHZ        (100000)
#define CY_FX_USBI2C_I2C_BITRATE_400KHZ        (400000)


/*#############Functions Declarations################*/

void RS485WriteConfig();
void RS485ReadConfig();
CyU3PReturnStatus_t RS485SpiTransfer (uint16_t ,uint16_t ,uint8_t  *, uint8_t  *, CyBool_t );
void RS485DeviceInit();
CyU3PReturnStatus_t MasterSpiSetSsnLine (CyBool_t);
CyU3PReturnStatus_t GpioSpiResetFifo (CyBool_t, CyBool_t);
CyU3PReturnStatus_t SendSpiWords (uint8_t *, uint32_t);
CyU3PReturnStatus_t RecvSpiWords (uint8_t *, uint32_t);
void GpioConfigure (uint8_t ,CyBool_t,CyBool_t,CyBool_t,CyBool_t,CyBool_t , CyBool_t);
CyU3PReturnStatus_t CyFxGpioInit (void);
CyU3PReturnStatus_t CyFxSpiTransfer (uint16_t ,uint16_t ,uint8_t  *,uint8_t  *,CyBool_t );
CyU3PReturnStatus_t CyFxSpiInit (uint16_t);
CyU3PReturnStatus_t CyFxUsbI2cTransfer (uint16_t,uint8_t,uint16_t,uint8_t  *,CyBool_t);
CyU3PReturnStatus_t CyFxI2cInit (uint16_t);
CyU3PReturnStatus_t CyFxSpiTransfer(uint16_t, uint16_t, uint8_t  *, uint8_t  *, CyBool_t);
CyU3PReturnStatus_t Fx3I2CTransfer(uint16_t  ,uint8_t  ,uint16_t  ,uint8_t  *,CyBool_t  );
uint8_t SPIDataRxRegRead();
#endif /* PDMANUFACTURERGPIO_H_ */
