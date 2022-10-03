/*
 * PDPPSManuStruct.h
 *
 *  Created on: 18-Nov-2021
 *      Author: GRL
 */

#ifndef PD_CONTROLLERSTRUCT_H_
#define PD_CONTROLLERSTRUCT_H_

#define BUF_SIZE_20BYTE     20
#define BUF_SIZE_24BYTE     24
#define BUF_SIZE_36BYTE     36

#define BUF_SIZE_2BYTE      2
#define BUF_SIZE_32BYTE     32
#define BUF_SIZE_12BYTE     12

#define bool_t  bool
extern uint8_t  gTimer0Var;
extern uint8_t  gTimer1Var;
extern uint8_t  gTimer2Var;


#define RS485_IDLE      1
#define RS485_BUSY      0

#define RUNTIME_API_RECEIVED       1
#define RUNTIME_API_HANDLED         0

#define TC_CONNECTED        1
#define TC_DISCONNECTED     0
#define mS                  1000
#define RESULTS_BUFFER_SIZE     4

extern uint16_t adcHeatSinkTemp2SensorDataBuffer[RESULTS_BUFFER_SIZE];
extern EPWM_SignalParams pwmSignal;


typedef enum
{
    TIMER0 = 0,
    TIMER1,
    TIMER2,
}MsgTimers_t;

typedef struct
{


    struct gTestControl
    {
        uint8_t gRS485BusStatus;  //to check the status of RS485 bus is free or busy.
        uint8_t gRuntimeCmd_Flag;
        uint8_t gIsPollingEnabled;
        bool TestCard_PowerState;
        bool Test_Booting_Status;
        bool Polling_State;

    }gTestControl_t;

}gTestHandle;

typedef struct
{
    gTestHandle gTestHandle_t;

}gControlStruct;

typedef enum
{
    Timer1_RunTime_CmdHandler = 0,
    Timer1_Power_Sequencing = 1,
    Timer1_TestCard_Power_State = 2,
//  Start_Polling_Timer,
    Timer1_RS485DevInit = 3,

}Timer1Var_t;

typedef enum
{
    Timer0_Init_Polling_CmdHandler = 0,
    Timer0_Start_Polling_Handler,
    Timer0_Polling_Expiry_Handler,
    Timer0_TempDataRead,
    Timer0_Read_Expiry_Timer ,


}Timer0Var_t;

typedef enum
{

    Timer2_TempDataRead = 0,


}Timer2Var_t;
gControlStruct* GetStructPtr();
extern gControlStruct *gControlStruct_t;
//gControlStruct *gControlStruct_t;

#endif /* PD_CONTROLLERSTRUCT_H_ */
