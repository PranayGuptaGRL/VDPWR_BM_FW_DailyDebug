/*
 * PDPPSManuCalib.c
 *
 *  Created on: 15-Nov-2021
 *      Author: GRL
 */

#include "PDPPSManuDefs.h"
#include "PDPPSManufacturerMain.h"
//#include "PDPPSManuCalib.h"
#include "PDPPSManuStruct.h"

//gPPSStruct *_pStructPtr;

//#define DECODE_FRAM_VALIDATION_FIELDS(X,Y)       ( (X[Y+1] << 8) |  X[Y] )

void GradientnOffsetCalculation(gBankDetails *lChannel)
{
    int j=0, lChannelNo=0;
    float Max12BitValue=0, Min12BitValue=0, MaxValue=0, MinValue=0;

    for(lChannelNo = 0; lChannelNo < 2; lChannelNo++)   // To Loop through Different Channels, Channel count is continuous till 23..
    {
        if(lChannel->gIsChannelEnabled == true)
        {
//          lChannel[lChannelNo].gOffsetData = lChannel[lChannelNo].gChannelData[0][1]; // First ADC Count is the Offset of that particular channel{Harsha's Input}
            for(j=9; j> 0; j--)
            {
                    Max12BitValue = lChannel->gChannelData[j][1];// y =count
                    Min12BitValue = lChannel->gChannelData[j-1][1];
                    MaxValue = lChannel->gChannelData[j][0];//x = voltage
                    MinValue = lChannel->gChannelData[j-1][0];

                    //gradient or slope = y2-y1/x2-x1
                    lChannel->gGradientData[j-1] = ((Max12BitValue -Min12BitValue)/(MaxValue -MinValue));  //double Gradient = ((Max12BitValue - Min12BitValue) / (MaxValue - MinValue));
                    //offset or c = y-mx
                    lChannel->gOffsetData[j-1] = (Min12BitValue - ((lChannel->gGradientData[j-1]) * MinValue)) ;

            }
        }
    }
}

uint32_t SetLiveValue(uint8_t aAdcNumber, float lLiveValue)
{
    uint8_t j=0;
    double lGradient =0, lOffset = 0;
            uint32_t aValue=0;

    if(lLiveValue >= pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[9][0])
     {
         lGradient = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gGradientData[8];
         lOffset = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gOffsetData[8];
     }
     else if(lLiveValue <= pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[0][0])
     {
         lGradient = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gGradientData[0];
         lOffset = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gOffsetData[0];
     }
     else
     {
        for(j=9; j > 0; j--)
        {
            if( (lLiveValue >= pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[j-1][0]) &&
                  (lLiveValue <= pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[j][0]) )
            {
                lGradient = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gGradientData[j-1];
                lOffset = pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gOffsetData[j-1];
                break;
            }
        }
     }
    aValue = (lLiveValue * lGradient) + lOffset;

    return aValue;//Final DAC Count to be set
}
uint32_t GetLiveVbusCurrentValue(uint8_t aAdcNumber, float aAvgValue)
{
    uint8_t j=0;
    double lGradient =0, lLiveValue=0, lOffset = 0;

    for(j=9; j> 0; j--)
    {
        if(aAvgValue > pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[9][1])
          {
              lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gGradientData[8];
              lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gOffsetData[8];
          }
         else if(aAvgValue < pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[0][1])
         {
             lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gGradientData[0];
             lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gOffsetData[0];
         }
         else if((aAvgValue <= pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[j][1]) &&
                      (aAvgValue >= pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[j-1][1]))
         {
              lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gGradientData[j-1];
              lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gOffsetData[j-1];
               break;
         }
    }

    lLiveValue = ((aAvgValue - lOffset)/lGradient);
    return lLiveValue;
}
uint32_t GetLiveVbusVoltageValue(uint8_t aAdcNumber, float aAvgValue)
{
    uint8_t j=0;
    double lGradient =0, lLiveValue=0, lOffset = 0;


    for(j=9; j> 0; j--)
    {

        if(aAvgValue > pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[9][1])
        {
            lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gGradientData[8];
            lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gOffsetData[8];
        }
        else if(aAvgValue < pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[0][1])
        {
            lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gGradientData[0];
            lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gOffsetData[0];
        }
        else if((aAvgValue <= pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[j][1]) &&
                (aAvgValue >= pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[j-1][1]))
        {
            lGradient = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gGradientData[j-1];
            lOffset = pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gOffsetData[j-1];
             break;
        }
    }

    lLiveValue = ((aAvgValue - lOffset)/lGradient);//(x = y-c / m)
    return lLiveValue;

}
void GradientnOffset()
{
    /**Vbus Voltage ADC*/
    GradientnOffsetCalculation(&pStructPtr->gFramData.CalibDataBank.gADCChannel[0]);
    /**Vbus Currrent ADC*/
    GradientnOffsetCalculation(&pStructPtr->gFramData.CalibDataBank.gADCChannel[1]);
    /**DAC Vout*/
    GradientnOffsetCalculation(&pStructPtr->gFramData.CalibDataBank.gDACChannel[0]);
}

void PPSCalibDataRxHandler(uint8_t * aBuffer)
{
    memcpy(&PPSCalibRxData[gCalibRxBufIndex], &aBuffer[0], 8);
    gCalibRxBufIndex += 8;
}

uint16_t Get2ByteValue()
{
    return ( (PPSCalibRxData[++pStructPtr->gFramData.gReadOffset] << 8) | PPSCalibRxData[++pStructPtr->gFramData.gReadOffset] );
}

uint8_t Get1ByteValue()
{
    return ( PPSCalibRxData[++pStructPtr->gFramData.gReadOffset] );
}

bool_t CheckforDelimiter()
{

    if(PPSCalibRxData[pStructPtr->gFramData.gReadOffset + 1] == (int)(';'))
    {
        pStructPtr->gFramData.gReadOffset++;
        return true;
    }
    else
        return false;

}

void DecodeSystemDetails()
{
    int Fram_Rev = Get1ByteValue();
    pStructPtr->gMiscStruct.gSystemID = Get2ByteValue();
}

void DecodeMfdDetails()
{
    pStructPtr->gMiscStruct.gMfdMonth = Get1ByteValue();
    pStructPtr->gMiscStruct.gMfdYear = Get2ByteValue();
}
bool_t DecodeADCCalibrationData()
{
    bool_t retVal = true;
    int i=0, count=0, j=0, k=0;
    uint16_t resVal =0;
    int adcBank = Get1ByteValue();
    int adcNumber = Get1ByteValue();
    int adcUnit = Get1ByteValue();

    if(adcBank == PPS_ADC_BANK_1 && adcNumber == PPS_ADC_VBUS_VOLTAGE)
    {
        pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelType = ADC_VOLTAGE_1mV_STEP;
        pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gIsChannelEnabled = true;

        for (i = pStructPtr->gFramData.gReadOffset; i < pStructPtr->gFramData.gTotalLength; i++)
        {
            resVal = 0, k=0;
            resVal = Get2ByteValue();
            count = Get2ByteValue();
            pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[j][k++] = resVal;
            pStructPtr->gFramData.CalibDataBank.gADCChannel[0].gChannelData[j++][k] = count;

            if ( CheckforDelimiter() )
                break;
        }
    }
    else if(adcBank == PPS_ADC_BANK_1 && adcNumber == PPS_ADC_VBUS_CURRENT)
    {
        pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelType = ADC_CURRENT_1mA_STEP;
        pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gIsChannelEnabled = true;

        for (i = pStructPtr->gFramData.gReadOffset; i < pStructPtr->gFramData.gTotalLength; i++)
         {
             resVal = 0, k=0;
             resVal = Get2ByteValue();
             count = Get2ByteValue();

             pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[j][k++] = resVal;
             pStructPtr->gFramData.CalibDataBank.gADCChannel[1].gChannelData[j++][k] = count;

             if ( CheckforDelimiter() )
                 break;
         }
    }
    else if(adcBank == PPS_DAC_BANK_1 && adcNumber == PPS_DAC_VBUS_VOLTAGE)
    {
        pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelType = DAC_VOLTAGE_1mV_STEP;
        pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gIsChannelEnabled = true;

        for (i = pStructPtr->gFramData.gReadOffset; i < pStructPtr->gFramData.gTotalLength; i++)
         {
             resVal = 0, k=0;
             resVal = Get2ByteValue();
             count = Get2ByteValue();

             pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[j][k++] = resVal;
             pStructPtr->gFramData.CalibDataBank.gDACChannel[0].gChannelData[j++][k] = count;

             if ( CheckforDelimiter() )
                 break;
         }
    }
    else
    {
        return false;
    }

    return retVal;

}

bool_t DecodeBlock(unsigned int start, unsigned int *blockEnd)
{

    bool_t retVal = true, blockStartFound = false, calDataEnds =false, calDataFound=false;

    int blkOffset =0, blockStart = 0, blockID = 0, readOffset = 0;

TRACK_BLOCKID:

    if(start > pStructPtr->gFramData.gTotalLength)
    {
        retVal =false;
        return retVal;
    }
    blkOffset = pStructPtr->gFramData.gTotalLength;
    do
    {
        if ( (PPSCalibRxData[start + 1] == 0x0A) && (PPSCalibRxData[start + 2] == 0x0A) )
        {
            blockStartFound = true;
            pStructPtr->gFramData.gReadOffset = start;
        }
        else
        {
            blockStartFound = false;
            start++;
        }
    } while (blockStartFound == false && start < pStructPtr->gFramData.gTotalLength -1);

    blockStart = Get2ByteValue();
    if (blockStart != 0x0A0A)
    {
        return false;
    }

    readOffset = blkOffset;
    blockID = Get1ByteValue();
    retVal = false;

    switch(blockID)
    {

    case BANK3_PPS_DATA:

        calDataEnds = false;
        calDataFound = false;

        do
        {
            calDataFound = DecodeADCCalibrationData();
            //Adding +30 because of reserved bits
            calDataEnds = (PPSCalibRxData[pStructPtr->gFramData.gReadOffset + 30+1] == 0x0A) && (PPSCalibRxData[pStructPtr->gFramData.gReadOffset +30+ 2] == 0x0A);
        } while (calDataFound == true && calDataEnds == false);

        blockEnd = &pStructPtr->gFramData.gReadOffset;
        retVal = true;
        pStructPtr->gFramData.gCalbDataDecodeStatus = true;

    break;

    case BANK0_SYSTEM_DETAILS:

        DecodeSystemDetails();

        start = pStructPtr->gFramData.gReadOffset ;
        blockStartFound = false;
        goto TRACK_BLOCKID;

        break;

    case BANK1_MFD_DETAILS:

        DecodeMfdDetails();

        start = pStructPtr->gFramData.gReadOffset ;
        blockStartFound = false;
        goto TRACK_BLOCKID;

        break;

    default:
        start = pStructPtr->gFramData.gReadOffset ;
        blockStartFound = false;
        goto TRACK_BLOCKID;

    break;
    }
    return retVal;
}

uint8_t GetDataFromCalibBuf()
{
    uint32_t lBlockStart=0;
    unsigned int blockEnd =0;
    bool_t retVal=0;
//    pStructPtr = (gQiStruct *)GetStructPtr(PortA);

    pStructPtr->gFramData.gReadOffset = -1;

    //Check the validity of FRAM
//    lBlockStart = DECODE_FRAM_VALIDATION_FIELDS(PPSCalibRxData, pStructPtr->gFramData.gReadOffset);
    lBlockStart = Get2ByteValue();
    if(lBlockStart != 0x0A0A)
        return 0;

    pStructPtr->gFramData.gTotalLength = Get2ByteValue();
    pStructPtr->gFramData.gCalbDataDecodeStatus = false;
    do
    {
        retVal = DecodeBlock(pStructPtr->gFramData.gReadOffset,&blockEnd);

    }
    while (retVal == true);

    if(pStructPtr->gFramData.gCalbDataDecodeStatus)
        GradientnOffset();

    return retVal;
}
