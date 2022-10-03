/*
 * PDPPSManuCalib.h
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#ifndef PDPPSMANUCALIB_H_
#define PDPPSMANUCALIB_H_

void PPSCalibDataRxHandler(uint8_t * aBuffer);
uint8_t GetDataFromCalibBuf();
uint16_t Get2ByteValue();
uint8_t Get1ByteValue();
uint32_t GetLiveValue(uint8_t aAdcNumber,float aAvgValue);
uint32_t SetLiveValue(uint8_t aAdcNumber, float lLiveValue);
uint32_t GetLiveVbusCurrentValue(uint8_t aAdcNumber, float aAvgValue);
uint32_t GetLiveVbusVoltageValue(uint8_t aAdcNumber, float aAvgValue);





#endif /* PDPPSMANUCALIB_H_ */
