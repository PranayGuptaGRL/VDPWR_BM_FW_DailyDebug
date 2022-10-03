/*
 * peripheral.h
 *
 *  Created on: 09-Mar-2022
 *      Author: prana
 */

#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_



#define CPU2_CTRL_VBUS_GPIO12          12

#define TC_BOARD_DET_GPIO14            14

#define SPIA_MOSI_GPIO16               16
#define SPIA_MISO_GPIO17               17
#define SPIOA_SCLK_GPIO18              18
#define SPIOA_CS_GPIO19                19

#define SPIC_LCD_MOSI_GPIO20            20
#define SPIC_LCD_MISO_GPIO21            21
#define SPIC_LCD_SCLK_GPIO22            22
#define SPIC_LCD_CS_GPIO23              23

#define SPIA_MOSI_GPIO32               32
#define SPIA_MISO_GPIO33               33
#define SPIOA_SCLK_GPIO34              34
#define SPIOA_CS_GPIO35                35

#define RS485_IRQ_GPIO25                25
//
// SCI for USB-to-UART adapter on FTDI chip
//
#define DEVICE_GPIO85_PIN_SCIRXDA     85U             // GPIO number for SCI RX
#define DEVICE_GPIO84_PIN_SCITXDA     84U             // GPIO number for SCI TX
#define DEVICE_GPIO85_CFG_SCIRXDA     GPIO_85_SCIA_RX // "pinConfig" for SCI RX
#define DEVICE_GPIO84_CFG_SCITXDA     GPIO_84_SCIA_TX // "pinConfig" for SCI TX

#define TC100_EN_PS_GPIO66              66
#define TC_PMOD_CONTROL_GPIO65          65

#define DBG_GREEN_LED_GPIO36            36
#define DBG_BLUE_LED_GPIO37            37
#define OCP_RED_LED_GPIO38              38


#define GPIO0_PIN_SDAA              0
#define GPIO1_PIN_SCLA              1

#define GPIO34_PIN_SDAB             34
#define GPIO35_PIN_SCLB             35

/**Fan 1 GPIO and PWM*/
#define FAN1_GPIO3                      3
#define FAN1_PWM_GPIO2                  2
/**Fan2 GPIO and PWM*/
#define FAN2_GPIO4                      4
#define FAN2_PWM_GPIO5                  5

#define DISPLAY_COMMAND   2
#define DISPLAY_DATA      1
#define START_DATA        0xFA
#define START_COMMAND     0xF8

#define WRITE 0
#define READ 1
#define RS485_WRITE_CONFIG              0xC400
#define RS485_READ_CONFIG               0x4000
#define RS485_WRITE_DATA                0x8000
#define RS485_READ_DATA                 0x0000
#define RS485_WRITE_DATA_DE_DISABLE     0x8600
#define FW_PL_LENGTH       1052
// Defines for RS485 Chip Select toggle.
//
#define RS485_CS_LOW                        GPIO_writePin(SPIOA_CS_GPIO19, 0)
#define RS485_CS_HIGH                       GPIO_writePin(SPIOA_CS_GPIO19, 1)
#define TURN_ON_TC                          GPIO_writePin(TC100_EN_PS_GPIO66, 1)
#define TURN_OFF_TC                         GPIO_writePin(TC100_EN_PS_GPIO66, 0)
#define HEADER_BYTE_CNT                     2U
#define LCD_SPI_CS_HIGH                     GPIO_writePin(SPIC_LCD_CS_GPIO23, 1)
#define LCD_SPI_CS_LOW                      GPIO_writePin(SPIC_LCD_CS_GPIO23, 0)

//#define POLLING    1

void initSPI();
void SPI0ReConfig();

void CM_EthernetIOMuxHandler();
void ControllerGpioInit_CPU1();
void ControllerGpioInit_CPU2();

#define MAX_DATA_TX_SIZE        128
#define MAX_FW_DATA_TX_SIZE        286

bool grlSpiTransmitWords(uint8_t  *buffer, uint16_t  byteCount);
bool grlSpiReceiveWords(uint8_t  *buffer, uint16_t  byteCount);
bool
grlSpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        bool  isRead);
bool
RS485SpiTransfer (
        uint16_t  byteAddress,
        uint16_t  byteCount,
        uint8_t  *buffer,uint8_t  *readBuffer,
        bool  isRead);
void RS485WriteDataDriverDisable();
void RS485ReadConfig();
void RS485WriteConfig();
void RS485WriteConfig();
void RS485WriteConfig_Sample(uint16_t aPar);
void RS485DeviceInit();
void grlRs485Rx_CmTx_DataHandler();
void grllcddisplay(uint8_t line,char *display_data);
void lcd_init();
void display_data_command(uint8_t type,uint8_t data);

extern bool RS485_interruptReceived;
extern bool CM_IPC_DataReceived;


#endif /* PERIPHERAL_H_ */
