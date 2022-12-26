/*
 * PDPPSManuDefs.h
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#ifndef PDPPSMANUDEFS_H_
#define PDPPSMANUDEFS_H_

#include "PDPPSManuIncludes.h"
#include "PDPPSManuCalib.h"


#define REQ_DAC_VOLTAGE_CALC(x)      ((32808 - x ) / (float) 10000)

//#define REQ_DAC_COUNT(y)             (y / 0.7326)
#define REQ_DAC_COUNT(y)             (y / 0.7324)//12Dec'22, Pranay, As per rajesh inputs, For APDO Alert message issue debug

#define STATIC_SLOPE_CALC   1
#define HEADER_BYTECOUNT    2

/**
 * Pranay,14Dec'22,
 * This Value defines the OCP limit breach, As feedback logic is running at 200us resolution,
 * we need to verify if OCP limit has been reached continuously for 10mS, so 200uS*50x = 10mS
 */
#define OCP_LIMIT_BREACH   50 // 200uS * 50times = 10mS, Need to tweak this value w.r.t to VFB resolution time
#define OCP_TOLERANCE  50
#define OCP_MIN_LIMIT     100
#define OCP_MAX_LIMIT     200
#define OCP_MIN_REQUEST    1000

#define EX_ADC_RESOLUTION    12
#define I2C_SLAVE_ADDR      0x70
#define FIFO_DEPTH          0xA

#define STACK_SIZE          512U
#define ADC_DATA_READ_STACK_SIZE    512U
#define DAC_DATA_READ_STACK_SIZE    512U
#define VBUS_5V         5000
#define VBUS_STEP_INC_DEC_SIZE      1000
#define RESULTS_BUFFER_SIZE     4 //buffer for storing conversion results
#define ADC_VBUS_SCALING_FACTOR      10
#define ADC_VBUS_I_SCALING_FACTOR    2.5
#define DAC_LOWER_V_LIMIT       1000//1200
#define DAC_HIGHER_V_LIMIT      3000
#define BOARD_IR_DROP_VSAFE5V             0.165 //Calculated and Hardcoded TC board IR Drop from TypeC end to PPS End is 0.165mohm/165ohm
#define BOARD_IR_DROP_BELOW_VSAFE5V       0.220 //Calculated and Hardcoded TC board IR Drop from TypeC end to PPS End is 0.220mohm
#define LED_BLINK_ITER_COUNT        5000
#define VSAFE_5V            4750
#define VBUS_I_OCP_LIMIT    130//130% of received current is the limit for OCP to trigger

#define GPIO12_CTRL_VBUS        12
#define GPIO64_OCP_TRIGGER     64                  // gpio connected between fx3 and PPS always high
#define OCP_TRIGGER(X)    GPIO_writePin(GPIO64_OCP_TRIGGER, X);
#define HANDLE_VBUS_CTRL_SWITCH(X)      GPIO_writePin(GPIO12_CTRL_VBUS, X);
//#define RTOSTIMER
#define VBUS_TOLR_VALUE     50 //10mv
#define VFB_STEP_DELAY     100//100uS
#define VFB_STEP_SIZE_DAC_COUNT       1
#define CC_PPS_SNO_DATA_READ_BYTECOUNT      6
#define FRAM_REV_DATA_READ_BYTECOUNT      1
#define SYSTEM_SNO_BYTECOUNT                2

#define CC_SNO_INDEX_FRAM       0x0A
#define FRAM_REV_INDEX          0x07

#define VBUS_INCDEC_STEPSIZE        1
#define VBUS_INCDEC_STEP_TIME       200
#define VFB_STEP_TIME               200 //Feedback logic is running at 200uS frequency
#define ADC_CONVERSION_TIME         100//As per the discussions with rajesh, but as per datasheet this value is in nS
#define VBUS_CURRENT_OCP_TOL        150

#define mS     1000
#define VBUS_IRDROP_OFFSET      250//250mv Vbus path IR DROP offset needs to be added for all the voltages requests(added this as per rajeshs comments)


#define BUF_SIZE_20BYTE     20
#define BUF_SIZE_24BYTE     24

#define BUF_SIZE_2BYTE      2
#define BUF_SIZE_32BYTE     32
#define BUF_SIZE_12BYTE     12

#define IPC_CMD_READ_MEM   0x1001

#define CALIBRATION_DATA_SIZE   400
#define VBUS_DEC_PERCT      105
#define VBUS_INC_PERCT      95

//********** Global Variables Definition / Initialization**************
uint8_t extern I2C_RX_Buffer[16];   // I2C Receive data buffer
uint8_t extern I2C_TX_Buffer[10];   //I2C transmitter data buffer.
uint8_t extern I2CGLOBALBUF[16];
uint8_t extern PPSCalibRxData[CALIBRATION_DATA_SIZE];

extern uint8_t  gTimer0Var;
extern uint8_t  gTimer1Var;

extern bool gBootTime;
extern uint16_t gCalibRxBufIndex;

uint16_t extern gRxVbusVoltage;//Tracking input received voltage in a global variable
uint16_t extern gCurrentVbusSetting; //Tracking currently iterating Vbus voltage count
float extern gADCAvgedVal;
//uint16_t extern gPresentVbusVoltage;
//uint16_t extern gPresentVbusCurrent;
uint16_t extern gLastDacValueWritten;
bool volatile extern gI2CCmdStatus;
uint8_t extern gPPSOperatingMode;
#define CUSTOM_OCP_LIMIT_CALC(X,Y)  ( (X / 100) * Y )

#define TIMEOUT 0xFFFFFFFF


#endif /* PDPPSMANUDEFS_H_ */
