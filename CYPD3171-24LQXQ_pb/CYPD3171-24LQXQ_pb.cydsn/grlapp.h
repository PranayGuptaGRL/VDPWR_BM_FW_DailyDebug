/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#include <pd.h>
#include <timer.h>

#define BUF_SIZE_32BYTE 32
#define BUF_SIZE_64BYTE 64
#define BUF_SIZE_128BYTE 128
#define BUF_SIZE_256BYTE 256
#define BUF_SIZE_280BYTE 280
#define BUF_SIZE_496BYTE 496
#define BUF_SIZE_512BYTE 512

#define SRCSNK_CAPS_SUPPLYTYPE_BYTEINDEX   30
#define SRCSNK_CAPS_SUPPLYTYPE_OFFSET      0xC0000000


void gTypec_Cable_Attach();
void g_PDSS_Status(uint8_t *);

void g_PDSS_Config(uint8_t *);
//void grl_Config(uint8_t *);
void grl_Control(uint8_t *);
bool gDataManager(uint8_t *);
void gI2cHandle_task();
void i2cDevice_Init();
void grlapp_Initialization();
void grlSystem_init();
void InitGetBattStatusExtndMsg(uint8_t * aBuffer);
bool gBC12_DataManager(uint8_t * );
uint8_t g_Decode_Rx_Ack();
void CableConnectionSimulation(uint8_t isAttach);


/* [] END OF FILE */
