/*
 *  ======== main.c ========
 */


#include "PDPPSManufacturerMain.h"
#include "PDPPSManuTimer.h"
#include "PDPPSManuCalib.h"
#include <PDPPSFlash.h>
#include "FRAM.h"
#pragma DATA_SECTION(CMDataRxbuf, "MSGRAM_CPU_TO_CM")
#pragma DATA_SECTION(CPU2DataRxbuf, "MSGRAM_CPU2_TO_CPU1")
uint8_t CPU2DataRxbuf[286];
uint8_t CMDataRxbuf[286];
//uint8_t FRAMReadData[286];
uint8_t FRAM_Misc_buf[16];

uint8_t glFirmwareID[6] = { '4', '2', '\1' };


//gPPSStruct *pStructPtr_t;
//gPPSStruct *pStructPtr;

//********** Global Variables Definition / Initialization**************
uint8_t I2C_RX_Buffer[16];   // I2C Receive data buffer
uint8_t I2C_TX_Buffer[10];   //I2C transmitter data buffer.
uint8_t I2CGLOBALBUF[16];
uint8_t PPSCalibRxData[CALIBRATION_DATA_SIZE];

uint16_t gCalibRxBufIndex;
uint8_t gTimer0Var;
uint8_t gTimer1Var;

bool gBootTime;
uint16_t gRxVbusVoltage;//Tracking input received voltage in a global variable
uint16_t gCurrentVbusSetting; //Tracking currently iterating Vbus voltage count
float gADCAvgedVal;
//uint16_t gPresentVbusVoltage;
//uint16_t gPresentVbusCurrent;
uint16_t gLastDacValueWritten;
bool volatile gI2CCmdStatus;
uint8_t gPPSOperatingMode;
uint8_t gi2c_rx_intr;

uint16_t adcDVbusDataBuffer[RESULTS_BUFFER_SIZE];
uint16_t adcDVbusCurrentDataBuffer[RESULTS_BUFFER_SIZE];
uint16_t adcOnboardTempSensorDataBuffer[RESULTS_BUFFER_SIZE];
uint16_t adcHeatSinkTemp1SensorDataBuffer[RESULTS_BUFFER_SIZE];

void SetVbustoZero()
{
    uint8_t lBuffer[] = {0x01,0x00,0x00,0x00,0x00};
//    DecodeRxDACVoltage(lBuffer);/**When turned into Sink from Src bring Vbus voltage to 5v and turn off ctrl switch*/
    RecvVbusSetCmdHandler(lBuffer);
}

void FWInitializations()
{
    memset(PPSCalibRxData, 0x00, CALIBRATION_DATA_SIZE);
    gCalibRxBufIndex = 0;

    pStructPtr->gMiscStruct.gIsVfbEnabled = false;
    pStructPtr->gMiscStruct.gIsRxVbusSet = false;

    gI2CCmdStatus = I2C_CMD_RECEIVED;

}
uint16_t DecodeBootTimeDACCount(uint16_t aReqVoltage)
{
//    uint16_t aReqVoltage = 5000;
    float lReqDACVoltage = 0.0,lReqDACCount = 0.0;
//    float mVDACVal = lDacVal/1000;
    lReqDACVoltage = REQ_DAC_VOLTAGE_CALC(aReqVoltage); //For required Vout voltage calculating required DAC input voltage
    lReqDACVoltage *= 1000; //V to mV
    lReqDACCount = round( REQ_DAC_COUNT( lReqDACVoltage) );  //Rounding off to the nearest integer variable

    return lReqDACCount;
}

void initADCSOC_A(void)
{

    //
    // - NOTE: A longer sampling window will be required if the ADC driving
    //   source is less than ideal (an ideal source would be a high bandwidth
    //   op-amp with a small series resistance). See TI application report
    //   SPRACT6 for guidance on ADC driver design.
    // - NOTE: SOCs need not use the same S+H window duration, but SOCs
    //   occurring in parallel (in this example, SOC0 on both ADCs occur in
    //   parallel, as do the SOC1s on both ADCs, etc.) should usually
    //   use the same value to ensure simultaneous samples and synchronous
    //   operation.

    //
    // Select the channels to convert and the configure the ePWM trigger
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);

    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER3, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN3, 15);

    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER4, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN4, 15);

    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_SW_ONLY,
                     ADC_CH_ADCIN5, 15);

    //
    // Selec SOC2 on ADCA as the interrupt source.  SOC2 on ADCC will end at
    // the same time, so either SOC2 would be an acceptable interrupt triggger.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);
}

void initADCSOC_B(void)
{

    //
    // - NOTE: A longer sampling window will be required if the ADC driving
    //   source is less than ideal (an ideal source would be a high bandwidth
    //   op-amp with a small series resistance). See TI application report
    //   SPRACT6 for guidance on ADC driver design.
    // - NOTE: SOCs need not use the same S+H window duration, but SOCs
    //   occurring in parallel (in this example, SOC0 on both ADCs occur in
    //   parallel, as do the SOC1s on both ADCs, etc.) should usually
    //   use the same value to ensure simultaneous samples and synchronous
    //   operation.

    //
    // Select the channels to convert and the configure the ePWM trigger
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 15);


    //
    // Selec SOC2 on ADCA as the interrupt source.  SOC2 on ADCC will end at
    // the same time, so either SOC2 would be an acceptable interrupt triggger.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);
}
//
// configureADC - Write ADC configurations and power up the ADC for the
// selected ADC
//
void configureADC(uint32_t adcBase)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(adcBase, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
    ADC_setMode(adcBase, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(adcBase, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(adcBase);

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DEVICE_DELAY_US(1000);
}

void Task_ADC_DataRead()
{
    uint16_t lADCVbusCalcVal = 0, lADCVbusCurrentCalcVal = 0;
//    uint8_t lOCPLimitReachCount = 0;
    float lAvgedVbusValue = 0.0, lAvgedCurrentValue = 0.0 ;
    float lLiveADCVal = 0;
    uint8_t i = 0;

//    for(;;)
    {
        ADC_forceMultipleSOC(ADCA_BASE, (ADC_FORCE_SOC2 | ADC_FORCE_SOC3 | ADC_FORCE_SOC4 | ADC_FORCE_SOC5));
        //
        // Wait for ADCD to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2) == false)
        {
        }
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER2);

//        adcDResult0 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0);
//        adcDResult1 = ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER1);
        memset(adcDVbusDataBuffer, 0x00, RESULTS_BUFFER_SIZE);
        memset(adcDVbusCurrentDataBuffer,0x00, RESULTS_BUFFER_SIZE);
        memset(adcOnboardTempSensorDataBuffer, 0x00, RESULTS_BUFFER_SIZE);
        memset(adcHeatSinkTemp1SensorDataBuffer,0x00, RESULTS_BUFFER_SIZE);

        lADCVbusCalcVal = lADCVbusCurrentCalcVal = 0;
        for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
        {
            adcDVbusDataBuffer[i] =  ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2 );
            adcDVbusCurrentDataBuffer[i] =  ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3 );
            adcOnboardTempSensorDataBuffer[i] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4 );
            adcHeatSinkTemp1SensorDataBuffer[i] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5 );
            DEVICE_DELAY_US(5);//tLAT for pre-scalar of 4 is 34mS as per TRM
        }
        for(i = 0; i < RESULTS_BUFFER_SIZE; i++)
        {
            lADCVbusCalcVal += adcDVbusDataBuffer[i];
            lADCVbusCurrentCalcVal += adcDVbusCurrentDataBuffer[i];
        }
        //Vbus Voltage Data calculation
        lAvgedVbusValue = lADCVbusCalcVal / ((float) RESULTS_BUFFER_SIZE);

        //Static calculations that needs to be done incase calibration is going on, else already populated calibration data
        if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
        {
            lLiveADCVal = (lAvgedVbusValue * 0.7326 * ADC_VBUS_SCALING_FACTOR);//Static calculations
        }
        else
        {
            lLiveADCVal = GetLiveVbusVoltageValue( PPS_ADC_VBUS_VOLTAGE, lAvgedVbusValue);//Applying calibrated data
        }
        pStructPtr->gVbusHandler.gADCLiveVbusVoltage =  round(lLiveADCVal);

        //Vbus Current data calculation
        lAvgedCurrentValue = lADCVbusCurrentCalcVal / ((float) RESULTS_BUFFER_SIZE);

        //Static calculations that needs to be done incase calibration is going on, else already populated calibration data
        if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
        {
            lLiveADCVal = (lAvgedCurrentValue * 0.7326 * ADC_VBUS_I_SCALING_FACTOR);//Static calculations
        }
        else
        {
            lLiveADCVal = GetLiveVbusCurrentValue( PPS_ADC_VBUS_CURRENT, lAvgedCurrentValue);//Applying calibrated data
        }

        pStructPtr->gVbusHandler.gADCLiveVbusCurrent = round(lLiveADCVal);

        //As TypeC end voltage and ADC voltage at PPS controller is different because of board IR drop, So to compensate Baord IR drop applying this formula
        pStructPtr->gVbusHandler.gTypeCendVbusVoltage =  (pStructPtr->gVbusHandler.gADCLiveVbusVoltage - (pStructPtr->gVbusHandler.gADCLiveVbusCurrent * BOARD_IR_DROP) ); //Vc = Vp - I * Rb (V at Typec  = V at ADC - I being drawn * Board R)

        //For calibration need to push the data read from ADC directly so no need to push calculated Vbus values
        if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
        {
            I2C_TX_Buffer[0] = ( (uint8_t)lAvgedVbusValue & 0xFF);/**Vbus Voltage Avg'ed ADC Count*/
            I2C_TX_Buffer[1] = ((uint8_t)lAvgedVbusValue >> 8);

            I2C_TX_Buffer[2] = ((uint8_t)lAvgedCurrentValue & 0xFF);/**Vbus Current Avg'ed ADC Count*/
            I2C_TX_Buffer[3] = ((uint8_t)lAvgedCurrentValue >> 8);
//            I2C_TX_Buffer[0] = 0xFA;
//            I2C_TX_Buffer[1] = 0xFB;
//            I2C_TX_Buffer[2] = 0xFC;
//            I2C_TX_Buffer[3] = 0xFD;
        }
        else
        {
            I2C_TX_Buffer[0] = (pStructPtr->gVbusHandler.gTypeCendVbusVoltage & 0xFF); //Pushing calculated Type C end Vbus voltage and ADC currents read
            I2C_TX_Buffer[1] = (pStructPtr->gVbusHandler.gTypeCendVbusVoltage >> 8);

            I2C_TX_Buffer[2] = (pStructPtr->gVbusHandler.gADCLiveVbusCurrent & 0xFF);
            I2C_TX_Buffer[3] = pStructPtr->gVbusHandler.gADCLiveVbusCurrent >> 8;
        }

    }
}
void SetDACVoltage(uint16_t aDACVal)
{
    float lReqDACVoltage = 0;
//    float mVDACVal = lDacVal/1000;
    lReqDACVoltage = REQ_DAC_VOLTAGE_CALC(aDACVal); //For required Vout voltage calculating required DAC input voltage
    lReqDACVoltage *= 1000; //V to mV

    if(lReqDACVoltage < DAC_LOWER_V_LIMIT || lReqDACVoltage > DAC_HIGHER_V_LIMIT)// In any instance if the DAC voltage is greater than max limit(3v) or less than min limit(1.2v) than dont set dac at all, just return
        return ;

    float lReqDACCount = REQ_DAC_COUNT(lReqDACVoltage); //From calculated DAC voltage determining DAC Count
    aDACVal = round(lReqDACCount);  //Rounding off to the nearest integer variable

    //applying slope and offset
//    aDACVal = SetLiveValue(PPS_DAC_VBUS_VOLTAGE,aDACVal);
    gLastDacValueWritten = aDACVal;
    if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
    {
     I2C_TX_Buffer[4] = (aDACVal & 0xFF);/**Vbus Voltage Avg'ed ADC Count*/
     I2C_TX_Buffer[5] = (aDACVal >> 8);
    }
    DAC_setShadowValue(DACB_BASE, aDACVal); //Setting voltage to DAC

    DEVICE_DELAY_US(2);//2us delay as per datasheet for setting DAC output voltage
    if( gBootTime == false)
    {
        HANDLE_VBUS_CTRL_SWITCH(TURN_ON);
    }
    else
    {
        HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
    }
}

uint8_t DACSetShadowValue(uint16_t aDACVal)
{
    DAC_setShadowValue(DACB_BASE, aDACVal); //Setting voltage to DAC
    return 0;
}

uint8_t DecodeLiveVbusDACCount(uint16_t aReqVoltage)
{
    float lReqDACVoltage = 0.0,lReqDACCount = 0.0;
//    float mVDACVal = lDacVal/1000;
    lReqDACVoltage = REQ_DAC_VOLTAGE_CALC(aReqVoltage); //For required Vout voltage calculating required DAC input voltage
    lReqDACVoltage *= 1000; //V to mV

    lReqDACCount = round( REQ_DAC_COUNT( lReqDACVoltage) );  //Rounding off to the nearest integer variable

    //applying slope and offset
    return ( SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lReqDACCount) );
}

uint16_t DecodeDACCount(uint16_t aReqVoltage)
{
    float lReqDACVoltage = 0.0,lReqDACCount = 0.0;
//    float mVDACVal = lDacVal/1000;
    lReqDACVoltage = REQ_DAC_VOLTAGE_CALC(aReqVoltage); //For required Vout voltage calculating required DAC input voltage
    lReqDACVoltage *= 1000; //V to mV

    if(lReqDACVoltage < DAC_LOWER_V_LIMIT || lReqDACVoltage > DAC_HIGHER_V_LIMIT)// In any instance if the DAC voltage is greater than max limit(3v) or less than min limit(1.2v) than dont set dac at all, just return
        return 0;

    lReqDACCount = round( REQ_DAC_COUNT( lReqDACVoltage) );  //Rounding off to the nearest integer variable

    //applying slope and offset
    return ( SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lReqDACCount) );

}

uint16_t CalcVbusReachRange(uint16_t aRxVbusVoltage, uint8_t aIncDec)
{
    uint16_t lCalcVal = 0;

    if( aIncDec == VBUS_DEC)
    {
        lCalcVal =  (aRxVbusVoltage / 100) ;
        lCalcVal *= VBUS_DEC_PERCT;
        return lCalcVal;//105%
    }

    else
    {
        lCalcVal =  ( (aRxVbusVoltage / 100 ) * VBUS_INC_PERCT);//95%
        return lCalcVal;
    }

}


void VbusStepIncDec(uint8_t aPar)
{

    pStructPtr->gVbusHandler.gReqVbusStartingDACCount = pStructPtr->gVbusHandler.LiveVbusDACCount;

    if(aPar == VBUS_DEC)
        MsgTimerStart(5, TIMER1_VBUS_VOL_STEP_DEC, TIMER1);

    else if(aPar == VBUS_INC)
        MsgTimerStart(5, TIMER1_VBUS_VOL_STEP_INC, TIMER1);

    else//if received request voltage is same as set voltage
    {
        DACSetShadowValue(pStructPtr->gVbusHandler.gReqVbusFinalDACCount);

        gLastDacValueWritten = pStructPtr->gVbusHandler.gReqVbusStartingDACCount;

         if(pStructPtr->gMiscStruct.gIsVfbEnabled)//Enabling Feedback logic
         {
             gI2CCmdStatus = I2C_CMD_HANDLED;
             //MsgTimerStart(VBUS_INCDEC_STEP_TIME, TIMER1_VBUS_FB_TRIGGER, TIMER1);
             pStructPtr->gMiscStruct.gIsRxVbusSet = true;
         }

    }
}

uint16_t Vbus_i_OCPLimitCalc(uint16_t aRxCurrent)
{

    return ( (aRxCurrent * VBUS_I_OCP_LIMIT) / 100);
}

void RecvVbusSetCmdHandler(uint8_t * aBuffer)
{

    //Vbus Voltage Received
     uint16_t lRxVbusVolt = ( (aBuffer[3] | (aBuffer[4] << 8)) );

    //Vbus Current received
    pStructPtr->gVbusHandler.gRxVbusCurrentValue = (aBuffer[5] | (aBuffer[6] << 8));

    pStructPtr->gVbusHandler.gRxVbus_i_OCPLimitVal = Vbus_i_OCPLimitCalc(pStructPtr->gVbusHandler.gRxVbusCurrentValue);

    uint16_t lRxVbusCalcPercentage = 0;
    gRxVbusVoltage = lRxVbusVolt;
    pStructPtr->gVbusHandler.gPresentRxVbusCount = lRxVbusVolt;

//    gRxVbusVoltage = RxVbusVolt; //Tracking input received voltage in a global variable


    pStructPtr->gVbusHandler.gReqVbusFinalDACCount =  SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lRxVbusVolt);
    pStructPtr->gVbusHandler.LiveVbusDACCount = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, pStructPtr->gVbusHandler.gADCLiveVbusVoltage);

//    gRxVbusVoltage = RxVbusVolt; //Tracking input received voltage in a global variable

    gCurrentVbusSetting = pStructPtr->gVbusHandler.gADCLiveVbusVoltage;

    if(lRxVbusVolt <= 3300)
    {
        if(pStructPtr->gVbusHandler.gADCLiveVbusVoltage > VBUS_5V)//If current Vbus Value is greater than 5v then set it to 5v and turn off switch
        {
            MsgTimerStart(2, TIMER1_VBUS_ZERO, TIMER1);
        }
        else /* if(gCurrentVbusSetting <= VBUS_5V) */
        {
            HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
            if( gBootTime )
                gBootTime = false;
        }
    }
    else if( pStructPtr->gVbusHandler.LiveVbusDACCount > pStructPtr->gVbusHandler.gReqVbusFinalDACCount ) // Increase Vbus as present Vbus DAC count > requested Vbus DAC Count, DAC count is indirectly proportion to Vbus Voltage
    {
        lRxVbusCalcPercentage = CalcVbusReachRange(lRxVbusVolt,VBUS_INC);
        pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lRxVbusCalcPercentage);
        VbusStepIncDec(VBUS_INC);

    }
    else if( pStructPtr->gVbusHandler.LiveVbusDACCount < pStructPtr->gVbusHandler.gReqVbusFinalDACCount )// Decrease Vbus as present Vbus DAC count < requested Vbus DAC Count
    {
        lRxVbusCalcPercentage = CalcVbusReachRange(lRxVbusVolt,VBUS_DEC);
        pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lRxVbusCalcPercentage);
        VbusStepIncDec(VBUS_DEC);

    }
    else if(pStructPtr->gVbusHandler.LiveVbusDACCount == pStructPtr->gVbusHandler.gReqVbusFinalDACCount)
    {
        lRxVbusCalcPercentage = CalcVbusReachRange(lRxVbusVolt,VBUS_DEC);
        pStructPtr->gVbusHandler.gPresentReqVbusReachDACCount = SetLiveValue(PPS_DAC_VBUS_VOLTAGE, lRxVbusCalcPercentage);
        VbusStepIncDec(2);
    }

}

void DecodeRxDACVoltage(uint8_t * aBuffer)
{
    uint16_t RxVbusVolt = (aBuffer[3] | (aBuffer[4] << 8));

    gRxVbusVoltage = RxVbusVolt; //Tracking input received voltage in a global variable
    gCurrentVbusSetting = pStructPtr->gVbusHandler.gADCLiveVbusVoltage;

    if(RxVbusVolt == 0)//If Voltage needs to be set to zero
    {
        if(pStructPtr->gVbusHandler.gADCLiveVbusVoltage > VBUS_5V)//If current Vbus Value is greater than 5v then set it to 5v and turn off switch
        {
            MsgTimerStart(2, TIMER1_VBUS_ZERO, TIMER1);
        }
        else /* if(gCurrentVbusSetting <= VBUS_5V) */
        {
          HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
          if( gBootTime )
             gBootTime = false;
        }
    }
    else if(RxVbusVolt <= pStructPtr->gVbusHandler.gADCLiveVbusVoltage)//If Vbus needs to be lowered from current voltage level
    {
      MsgTimerStart(2, TIMER1_VBUS_DEC, TIMER1);
    }
    else if(RxVbusVolt > pStructPtr->gVbusHandler.gADCLiveVbusVoltage)
    {
        MsgTimerStart(2, TIMER1_VBUS_INC, TIMER1);
    }
    else if(gRxVbusVoltage == VBUS_5V)
    {
        SetDACVoltage(RxVbusVolt);
    }

}

void I2CSetCmdHandler(uint8_t * aBuffer)
{

    gPPSOperatingMode = aBuffer[2];

    if (gPPSOperatingMode == PPS_IN_CALIB_MODE_v || gPPSOperatingMode == PPS_IN_CALIB_MODE_i)
        pStructPtr->gMiscStruct.gIsVfbEnabled = false;
    else
        pStructPtr->gMiscStruct.gIsVfbEnabled = true;

    switch(aBuffer[1])
     {

     case SET_VBUS:
         pStructPtr->gMiscStruct.gIsRxVbusSet = false;

//         MsgTimerStop(TIMER0);//Stopping Feedback logic timer in case

         if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
              DecodeRxDACVoltage(aBuffer);
         else
             RecvVbusSetCmdHandler(aBuffer);

         break;

     case SET_SNK_MODE:
         pStructPtr->gMiscStruct.gIsRxVbusSet = false;
         SetVbustoZero();
//         DecodeRxDACVoltage(0);/**When turned into Sink from Src bring Vbus voltage to 5v and turn off ctrl switch*/
         break;

     //Some miscellaneous APIs for internal/Debug purposes
     case 0xF1:
         DAC_setShadowValue(DACB_BASE, (aBuffer[1] | (aBuffer[2] << 8))); //Setting voltage to DAC
         break;

     case 0xF2:
         SetDACVoltage( (aBuffer[1] | (aBuffer[2] << 8)) );
         break;
     case 0xF3:
         HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
         break;
     case 0xF4:
         HANDLE_VBUS_CTRL_SWITCH(TURN_ON);
         break;
     case 0xF5:
         gBootTime = true;
         break;
     case 0xF6:
         gBootTime = false;
         break;
     }
}
void I2CRXHandler(uint8_t * aBuffer)
{

    switch(aBuffer[0])
    {
    case I2C_CMD_SET:
        I2CSetCmdHandler(aBuffer);
        break;
    case I2C_CMD_PGM:
        switch(aBuffer[1])
        {
            case SET_CONTROLLER_RESET:

//                if( GPIOmode_check() == PGM_MODE)
//                    SysCtl_resetDevice();
            break;
        }
        break;
    case I2C_CMD_GET:
//        I2CDataReadCmdHandler();
        break;
    case I2C_CMD_CALIB:
        switch(aBuffer[1])
        {
        case PPS_CALIB_DATA_FETCH://Receives and stores the total Calibration data in local buf
            PPSCalibDataRxHandler(aBuffer);
            gI2CCmdStatus = I2C_CMD_HANDLED;

            break;

        case PPS_CALIB_DATA_DECODE://After writing total Calib data to PPS, Send an API to make TI PPS start calculating Slope and gradients
//            MsgTimerStart((5000), TIMER1_BOOTUP_VBUSHANDLER, TIMER1);
            GetDataFromCalibBuf();
            gI2CCmdStatus = I2C_CMD_HANDLED;

            break;
        }
        break;
    }

}
bool VfbHandler()
{
    uint16_t lDacValue = gLastDacValueWritten;
    uint16_t lRxVbusVoltage = gRxVbusVoltage;

    if((abs(pStructPtr->gVbusHandler.gTypeCendVbusVoltage - lRxVbusVoltage) < VBUS_TOLR_VALUE ) || ( gI2CCmdStatus != I2C_CMD_HANDLED))
    {
        // if I2C command not received but vbus tolerance is well within limits of 100mv tolerance than consider vbus is set so start feedback logic in ADC task
//        if( gI2CCmdStatus == I2C_CMD_HANDLED )
//            pStructPtr->gMiscStruct.gIsRxVbusSet = true;
        return true;
    }

    else if (pStructPtr->gVbusHandler.gTypeCendVbusVoltage < lRxVbusVoltage)
    {
        lDacValue -= VFB_STEP_SIZE_DAC_COUNT;// Incrementing / decrementing interms of 10mV
//        lDacValue -= VBUS_INCDEC_STEPSIZE;
        gLastDacValueWritten = lDacValue;
        DAC_setShadowValue(DACB_BASE, lDacValue); //Setting voltage to DAC
        return false;

    }
    else if(pStructPtr->gVbusHandler.gTypeCendVbusVoltage > lRxVbusVoltage)
    {
        lDacValue += VFB_STEP_SIZE_DAC_COUNT;// Incrementing / decrementing interms of 10mV
//        lDacValue += VBUS_INCDEC_STEPSIZE;
        gLastDacValueWritten = lDacValue;
        DAC_setShadowValue(DACB_BASE, lDacValue); //Setting voltage to DAC
        return false;
    }
//    else if(lRxVbusVoltage == 0 || lRxVbusVoltage < 3300)
//    {
//        HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);
//        return;
//    }
    return 0 ;
}
void DataRxTask()
{
    memcpy(&I2CGLOBALBUF[0], &I2C_RX_Buffer[0], 16);
    I2CRXHandler(I2CGLOBALBUF);
    gi2c_rx_intr=0;
}

void configureDAC(uint32_t dacBase)
{
    //
    // Set VDAC as the DAC reference voltage.
    // Edit here to use ADC VREF as the reference voltage.
    //
    DAC_setReferenceVoltage(dacBase, DAC_REF_ADC_VREFHI);

    //
    // Enable the DAC output
    //
    DAC_enableOutput(dacBase);

    //
    // Set the DAC shadow output to 0-0; 0xFFF for Max
    //

    if(dacBase == DACB_BASE)
    {
        DAC_setShadowValue(dacBase, DecodeBootTimeDACCount(5000) );
    }
    else if(dacBase == DACA_BASE)
    {
        //configuring Current foldback DAC to MAX limit in bootup itself, ie., 6.25A
        // As per LT3741/LT3741-1 Data sheet Io = Vctrl1/(30*Rs) where Rs = 8mOhm (as per rajesh input)
        // mAx DAC Out is 3 and respc. PPS_CTRL1_VOLTAGE is 1.5v and 1 bit DAC Change is 0.0007326v
        // for 3v, 3v/0.0007326 = 4095Count so DAC Count shall be 4095 which is 0xFFF
//        DAC_setShadowValue(dacBase, 0x7FF);
        DAC_setShadowValue(dacBase, 0xFFF);
    }

    //
    // Delay for buffered DAC to power up
    //
    DEVICE_DELAY_US(10);//it was 10us
}


void Ti_CPU2BootModeSelection(uint8_t * aRxBuffer)
{
    uint8_t lBootMode = aRxBuffer[4];

    if(lBootMode == 0x02) //Program mode received -> Enter in to SSBL
    {
        grlCPU2PGMmodeSetHandler(aRxBuffer);
//        SysCtl_resetDevice();
    }
    else
    {

    }

}
void is_TiFWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lCpuSelection = aRxBuffer[3];

    switch(lCpuSelection)
    {
    case TI_CM://0x00
//        SysCtl_controlCMReset(SYSCTL_CORE_ACTIVE);
//        MsgTimerStart(TIMER_1, 2000);
        break;
    case TI_CPU1_CONTROLCARD://0x01

//        Ti_CPU1BootModeSelection(aRxBuffer);

        break;

    case TI_CPU2_PPS://0x02
        Ti_CPU2BootModeSelection(aRxBuffer);
        break;
    }

}

void isAPI_FWUpdate(uint8_t * aRxBuffer)
{
    uint8_t lFwController = aRxBuffer[2];

    switch(lFwController)
    {
        case 0x06 ://Ti
                is_TiFWUpdate(aRxBuffer);
            break;
        default: // CCG3PA / ELOAD /TC-FX3
            //PUSH to SPI
            break;

    }

}


void SetDataCmd(uint8_t * aRxBuffer)
{

    switch(aRxBuffer[3])
    {

    case 0x0F://FRAM write

        FRAM_I2C_write(aRxBuffer[4], (aRxBuffer[5] | aRxBuffer[6] << 8), &aRxBuffer[7]);

        break;
    }

}

void GetDataCmdHandler(uint8_t * aRxBuffer)
{
    switch(aRxBuffer[2])
    {
    case 0x80://Get FRAM DATA

        FRAM_I2C_read(aRxBuffer[3], (aRxBuffer[4] | aRxBuffer[5] << 8), CPU2DataRxbuf);

        memcpy(&CMDataRxbuf[(CMDataRxbuf[1]+2)], &CPU2DataRxbuf[0], CMDataRxbuf[3]);

        CMDataRxbuf[1] = CMDataRxbuf[1] + CMDataRxbuf[3] + 2;

        IPC_sendCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                   IPC_CMD_READ_MEM, (uint32_t)CMDataRxbuf, CMDataRxbuf[1]);

        IPC_waitForAck(IPC_CPU2_L_CM_R, IPC_FLAG3);

        break;
    }
}
void grlIPCDataRxHandler(uint8_t * aRxBuffer)
{

    uint8_t lRxCmdType = (aRxBuffer[0] & 0x0F);


    switch(lRxCmdType)
    {
        case RX_API_IS_SET:

            SetDataCmd(aRxBuffer);

            break;

        case RX_API_IS_PGM:

            isAPI_FWUpdate(aRxBuffer);

            break;

        case Rx_API_IS_GET:

             GetDataCmdHandler(aRxBuffer);
            break;
    }

}
//CPU2 to CM
__interrupt void IPC_ISR1()
{
    uint32_t lVar[286];

    uint32_t command, addr, data;

    uint16_t aByteCount = 0;
    //
    // Read the command
    //
    IPC_readCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CPU2_L_CM_R, IPC_FLAG3);

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

    memset(CMDataRxbuf, 0x00, 128);

    int i =0,index = 0;

    for (i = 0; i < (data); i++)
    {
        lVar[0+i] = (*((uint32_t *)addr + i) );
//        buf_rx1[0 + i ] = (*((uint32_t *)addr + i) );
    }
    for (i = 0; i < (data/4)+1; i++)
    {
        CMDataRxbuf[index++] = (lVar[i] & 0xFF);
        CMDataRxbuf[index++] = (lVar[i] & 0xFF00) >> 8;
        CMDataRxbuf[index++] = (lVar[i] & 0xFF0000) >> 16;
        CMDataRxbuf[index++] = (lVar[i] & 0xFF000000) >> 24;
    }


//    aByteCount  = CMDataRxbuf[1] + 2;
//    IPC_sendCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
//                    IPC_CMD_READ_MEM, (uint32_t)CMDataRxbuf, aByteCount);
//
//    IPC_waitForAck(IPC_CPU2_L_CM_R, IPC_FLAG3);

//    CmDataRxHandler(CMDataRxbuf);
    grlIPCDataRxHandler(CMDataRxbuf);

}
//
// IPC ISR for Flag 0.
// C28x core sends data without message queue using Flag 0
//
__interrupt void IPC_ISR0()
{
    uint32_t lVar[286];
//    int i;
    uint32_t command, addr, data;
//    uint16_t aVal = 0;
//    bool status = false;
    uint16_t lTxBuf[4] = {0};

    //
    // Read the command
    //
    IPC_readCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CPU2_L_CPU1_R, IPC_FLAG2);


    int i =0,index = 0;

    for (i = 0; i < (data); i++)
    {
        lVar[0+i] = (*((uint32_t *)addr + i) );
//        buf_rx1[0 + i ] = (*((uint32_t *)addr + i) );
    }
    for (i = 0; i < ((data/4) + 1 * 2); i++)
    {

        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        CPU2DataRxbuf[index++] = (lTxBuf[0] | (lTxBuf[1] << 8)) ;
        CPU2DataRxbuf[index++] = (lTxBuf[2] | (lTxBuf[3] << 8)) ;
    }


    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP11);

//    aVal = (CPU2DataRxbuf[3] | (CPU2DataRxbuf[4] << 8));
//
//    aVal = DecodeBootTimeDACCount(aVal);
//
//    DAC_setShadowValue(DACB_BASE, aVal);
//
//    HANDLE_VBUS_CTRL_SWITCH(1);

//
//    IPC_sendCommand(IPC_CPU2_L_CPU1_R, IPC_FLAG2, IPC_ADDR_CORRECTION_ENABLE,
//                    IPC_CMD_READ_MEM, (uint32_t)CPU2DataRxbuf, CPU2DataRxbuf[1]+2);
//
//    IPC_waitForAck(IPC_CPU2_L_CPU1_R, IPC_FLAG2);

}

void ipc_init()
{


    //
    // Enable IPC interrupts
    //
    IPC_registerInterrupt(IPC_CPU2_L_CPU1_R, IPC_INT2, IPC_ISR0);
    //
    // Clear any IPC flags if set already
    //
    IPC_clearFlagLtoR(IPC_CPU2_L_CPU1_R, IPC_FLAG_ALL);


    //
    // Enable IPC interrupts
    //
    IPC_registerInterrupt(IPC_CPU2_L_CM_R, IPC_INT3, IPC_ISR1);
    //
    // Clear any IPC flags if set already
    //
    IPC_clearFlagLtoR(IPC_CPU2_L_CM_R, IPC_FLAG_ALL);

}

__interrupt void i2cAFIFOISR(void)
{
    uint8_t i = 0;

     if ((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_RXFF) != 0)
     {
         for (i = 0; i < 16; i++)
         {
             I2C_RX_Buffer[i] = I2C_getData(I2CA_BASE);
         }
         I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_RXFF);
         gI2CCmdStatus = I2C_CMD_RECEIVED;
         gi2c_rx_intr=I2C_CMD_RECEIVED;

//         GiveSemaphoreDataRx();
     }
     else if ((I2C_getInterruptStatus(I2CA_BASE) & I2C_INT_TXFF) != 0)
     {
         memcpy(&I2C_TX_Buffer[7], glFirmwareID, 2);//FW version
         I2C_TX_Buffer[9] = 0xFE;//Code word

        if((gPPSOperatingMode == PPS_IN_CALIB_MODE_v) || (gPPSOperatingMode == PPS_IN_CALIB_MODE_i))
        {
            I2C_TX_Buffer[4] = (gLastDacValueWritten & 0xFF);/**Vbus Voltage Avg'ed ADC Count*/
            I2C_TX_Buffer[5] = (gLastDacValueWritten >> 8);
        }
         for (i = 0; i < 10; i++)
         {
             I2C_putData(I2CA_BASE, I2C_TX_Buffer[i]);
         }

         I2C_sendStartCondition(I2CA_BASE);
         I2C_clearInterruptStatus(I2CA_BASE, I2C_INT_TXFF);
         memset(I2C_TX_Buffer, 0, 10);
         HWREGH(I2CA_BASE + I2C_O_FFRX) |= I2C_FFRX_RXFFRST;

     }
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP8);
}

//I2CA FIFO init
void Init_I2CA_FIFO(void)
{
    //
    // Must put I2C into reset before configuring it
    //
    I2C_disableModule(I2CA_BASE);

    // I2C slave configuration
    //
    I2C_setConfig(I2CA_BASE, I2C_SLAVE_RECEIVE_MODE | I2C_SLAVE_SEND_MODE );

    I2C_setDataCount(I2CA_BASE, 16);
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);

    //
    // Configure for external loopback
    //
    I2C_setOwnSlaveAddress(I2CA_BASE, I2C_SLAVE_ADDR);
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    I2C_enableFIFO(I2CA_BASE);
    I2C_clearInterruptStatus(I2CA_BASE, (I2C_INT_RXFF | I2C_INT_TXFF) );

    //
    // Receive FIFO interrupt levels are set to generate an interrupt
    // when the 16 byte RX fifo contains 16 bytes of data.
    //
    //
    I2C_setFIFOInterruptLevel(I2CA_BASE, I2C_FIFO_TX6, I2C_FIFO_RX16);
    I2C_enableInterrupt(I2CA_BASE, (I2C_INT_RXFF | I2C_INT_TXFF) );

    //
    // Configuration complete. Enable the module.
    //
    //
    I2C_enableModule(I2CA_BASE);

}

void ADC_init()
{
    configureADC(ADCA_BASE);
    initADCSOC_A();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;
    FWInitializations();

}

void i2cB_init()
{
    Interrupt_register(INT_I2CB, &I2C_FRAM_ISR);
    Init_I2C_Master();
    Interrupt_enable(INT_I2CB);

}

void i2cA_init()
{
    //Initialize I2CA
    //
    Interrupt_enable(INT_I2CA_FIFO);
    Init_I2CA_FIFO();
    Interrupt_register(INT_I2CA_FIFO, &i2cAFIFOISR);

}

void TxSystemDetailstoCM()
{
    memset(CMDataRxbuf, 0x00, 12);

    CMDataRxbuf[0] = 0xC1;
    CMDataRxbuf[1] = 0xC2;
    CMDataRxbuf[2] = 0x0A;//No of Bytes payload length

    CMDataRxbuf[3] = (pStructPtr->gMiscStruct.gSystemID & 0xFF);
    CMDataRxbuf[4] = (pStructPtr->gMiscStruct.gSystemID >> 8);

    CMDataRxbuf[5] = (pStructPtr->gMiscStruct.gMfdMonth & 0xFF);

    CMDataRxbuf[6] = (pStructPtr->gMiscStruct.gMfdYear & 0xFF);
    CMDataRxbuf[7] = (pStructPtr->gMiscStruct.gMfdYear >> 8);

    CMDataRxbuf[8] = 0xFD;//Checksum Keyword

    IPC_sendCommand(IPC_CPU2_L_CM_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                    IPC_CMD_READ_MEM, (uint32_t)CMDataRxbuf, CMDataRxbuf[2]);

    IPC_waitForAck(IPC_CPU2_L_CM_R, IPC_FLAG3);

}

/*
 *  ======== main ========
 */
void main()
{ 
//    Device_init();
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    configureDAC(DACB_BASE);//Vbus v
    configureDAC(DACA_BASE);//Vbus i

    ipc_init();
    i2cA_init();
    i2cB_init();
    ADC_init();

    INIT_CPU_TIMERS();//Initialize only CPU0 and CPU1 because CPU2 is being reserved for RTOS Idle task operation, Ensure no CPU2 timer will be created

    setupMsgTimer0();

    setupMsgTimer1();

    HANDLE_VBUS_CTRL_SWITCH(TURN_OFF);

    //
    // Enables CPU interrupts
    //
    EINT;
    ERTM;

    MsgTimerStart(1000, I2C_FRAM_BYTECOUNT_DECODE, TIMER0);

    while(1)
    {
//        GPIO_writePin(38, 0);
//        DEVICE_DELAY_US(100000);//it was 10us
//        GPIO_writePin(38, 1);
//        DEVICE_DELAY_US(100000);//it was 10us

        if(gi2c_rx_intr == I2C_CMD_RECEIVED)
        {
            DataRxTask();
    //        gi2c_rx_intr=0;
        }

    }
}
