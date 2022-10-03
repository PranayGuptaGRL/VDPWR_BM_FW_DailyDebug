/*
 * PDPPSManufacturerMain.h
 *
 *  Created on: 14-Mar-2022
 *      Author: prana
 */

#ifndef PDPPSMANUFACTURERMAIN_H_
#define PDPPSMANUFACTURERMAIN_H_

#include "PDPPSManuStruct.h"
#include "PDPPSManuDefs.h"
#include "PDPPSManuTimer.h"

void SetDACVoltage(uint16_t );
void DecodeRxDACVoltage(uint8_t * aBuffer);
void SetVbustoZero();

void TriggerVbusVfb();
bool VfbHandler();
void PPSCalibDataRxHandler(uint8_t * aBuffer);
uint8_t DACSetShadowValue(uint16_t aDACVal);
uint16_t DecodeBootTimeDACCount(uint16_t aReqVoltage);
void RecvVbusSetCmdHandler(uint8_t * aBuffer);
void Task_ADC_DataRead();
void TxSystemDetailstoCM();

#endif /* PDPPSMANUFACTURERMAIN_H_ */
