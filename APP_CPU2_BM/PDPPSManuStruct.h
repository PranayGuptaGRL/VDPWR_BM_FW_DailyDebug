/*
 * PDPPSManuStruct.h
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#ifndef PDPPSMANUSTRUCT_H_
#define PDPPSMANUSTRUCT_H_

#include "PDPPSManuIncludes.h"

#define bool_t  bool

extern uint8_t CPU2DataRxbuf[286];
extern uint8_t CMDataRxbuf[286];
//extern uint8_t FRAMReadData[286];
extern uint8_t FRAM_Misc_buf[16];

//extern uint8_t I2CGLOBALBUF[16];
//extern uint8_t I2C_RX_Buffer[16];
//extern uint8_t I2C_TX_Buffer[16];
#define I2C_CMD_RX_EVT_ID_0      Event_Id_00



#define BUF_SIZE_20BYTE     20
#define BUF_SIZE_24BYTE     24

#define BUF_SIZE_2BYTE      2
#define BUF_SIZE_32BYTE     32
#define BUF_SIZE_12BYTE     12

typedef enum
{
    TI_CM   = 0,
    TI_CPU1_CONTROLCARD = 1,
    TI_CPU2_PPS = 2
}Ti_Cores;

typedef enum
{
    RX_API_IS_SET = 0x01,
    RX_API_IS_PGM = 0x02,
    Rx_API_IS_GET = 0x07,

}rxApiType;

typedef enum
{
    BANK0_SYSTEM_DETAILS = 0x00,
    BANK1_MFD_DETAILS    = 0x01,
    BANK3_PPS_DATA  = 0x03,

}CalibDataBanks;

typedef struct gCommonBankDetails
{
    int gChannelType;
    bool gIsChannelEnabled;
    uint8_t gReferenceChannelNo;
    uint16_t gChannelData[BUF_SIZE_24BYTE][BUF_SIZE_2BYTE];// Stored value at 00,10,20,30..... 12BitValue at 01,11,21......
    double gGradientData[BUF_SIZE_20BYTE];
    double gOffsetData[BUF_SIZE_20BYTE];

}gBankDetails;

typedef struct gADCChannel
{
    gBankDetails gADCChannel[BUF_SIZE_2BYTE];  // Channel starts From 0,1,2,3...
    gBankDetails gDACChannel[1];

}gADCChannelDetails;

struct gFramData
{
    unsigned int gReadOffset;
    unsigned int gTotalLength;
    gADCChannelDetails CalibDataBank;  //Contains the Channel details of all Banks[Total 8 Banks]// index=0
    bool_t gCalbDataDecodeStatus;
//  gADCChannelDetails gCalibData;  //Contains gradient and offset of all ADC Channels
};

struct gVbusDefs
{
    volatile uint16_t gPresentRxVbusCount;
    uint16_t gPrevRxVbusCount;
    uint16_t gPresentReqVbusReachVolt;
    uint16_t gPresentReqVbusReachDACCount;
    uint16_t gReqVbusStartingDACCount;
    uint16_t LiveVbusDACCount;
    volatile uint16_t gADCLiveVbusVoltage;
    volatile uint16_t gTypeCendVbusVoltage;
    volatile uint16_t gADCLiveVbusCurrent;
    uint16_t gRxVbusCurrentValue;
    uint16_t gRxVbus_i_OCPLimitVal;
    uint16_t gReqVbusFinalDACCount;
};

struct gMiscDefs
{
    bool gIsVfbEnabled;
//    bool gIsCalibEnabled;
    bool gIsRxVbusSet;
    uint16_t gFramCurrentByteAddress;
    uint16_t gFramByteCount;
    uint16_t gSystemID;
    uint8_t gMfdMonth;
    uint16_t gMfdYear;

};
typedef struct gPPSConfig
{
    struct gFramData gFramData;
    struct gVbusDefs gVbusHandler;
    struct gMiscDefs gMiscStruct;
}gPPSStruct;

enum CalibrationDataType
{
    ADC_TYPE_NONE =0,
    ADC_CURRENT_1mA_STEP=1,
    ADC_VOLTAGE_1mV_STEP=2,
    DAC_VOLTAGE_1mV_STEP=3,
};
enum CalibChannelNumber
{
    PPS_ADC_VBUS_VOLTAGE = 0x00,
    PPS_ADC_VBUS_CURRENT = 0x01,
    PPS_DAC_VBUS_VOLTAGE = 0x02,
};
enum ADCBanks
{
    PPS_ADC_BANK_1 = 0x01,
};

enum DACBanks
{
    PPS_DAC_BANK_1 = 0x01,
};
typedef enum
{
    TIMER0 = 0,
    TIMER1,
    TIMER2,

}MsgTimers_t;
typedef enum
{
    SET_SNK_MODE     = 0x04,
    SET_VBUS         = 0x0A,
}I2CRxAPIs_t;
typedef enum
{
    SET_CONTROLLER_RESET     = 0x01,

}PGMModeAPIS_t;
typedef enum
{
    I2C_CMD_SET = 0x01,
    I2C_CMD_PGM = 0x02,
    I2C_CMD_GET = 0x07,
    I2C_CMD_CALIB = 0x0A,

}I2CCmds;


typedef enum
{
    TURN_OFF = 0,
    TURN_ON = 1,

}GPIOCtrlCmds_t;

typedef enum
{
    TIMER1_BOOTUP_VBUSHANDLER = 0,
    TIMER1_VBUS_INC = 1,
    TIMER1_VBUS_DEC,
    TIMER1_VBUS_ZERO,
    TIMER1_VBUS_DAC_STEP_INC,
    TIMER1_VBUS_DAC_STEP_DEC,
    TIMER1_VBUS_VOL_STEP_INC,
    TIMER1_VBUS_VOL_STEP_DEC,
    TIMER1_VBUS_FB_TRIGGER,


}Timer1Var_t;

typedef enum
{
    TIMER0_VBUS_FB_TRIGGER = 1,
    ADC_DATA_READ=2,
    I2C_FRAM_BYTECOUNT_DECODE = 3,
    I2C_FRAM_CALIB_DATA_HANDLER = 4,
    CALIB_DATA_POP = 5,
    CM_IPC_SYNC_TRIGGER=6,

}Timer0Var_t;

typedef enum
{
    I2C_CMD_HANDLED = 0,
    I2C_CMD_RECEIVED  = 1

}I2CCmdStatus;

typedef enum
{
    PPS_IN_NORMAL_MODE  = 0x00,
    PPS_IN_CALIB_MODE_v = 0xCA,
    PPS_IN_CALIB_MODE_i = 0xCB,
}PPS_OP_Mode;

typedef enum
{
    VBUS_DEC = 0,
    VBUS_INC = 1,
}VbusIncDec;

typedef enum
{
    PPS_CALIB_DATA_FETCH     = 0x01,
    PPS_CALIB_DATA_DECODE     = 0x02,
}PPS_Data_Cmd_Mode;

extern gPPSStruct *pStructPtr;

#endif /* PDPPSMANUSTRUCT_H_ */
