/*
 * PDManufacturerMain.h
 *
 *  Created on: Jun 12, 2019
 *      Author: Prasanna
 */

#ifndef PDMANUFACTURERMAIN_H_
#define PDMANUFACTURERMAIN_H_

void CyFxAppErrorHandler (CyU3PReturnStatus_t);
void CyFxBulkLpApplnDebugInit (void);
void CyFxBulkLpApplnStart (void);
void CyFxBulkLpApplnStop (void);
CyBool_t CyFxBulkLpApplnUSBSetupCB (uint32_t, uint32_t);
void CyFxBulkLpApplnUSBEventCB (CyU3PUsbEventType_t, uint16_t);
CyBool_t CyFxBulkLpApplnLPMRqtCB (CyU3PUsbLinkPowerMode);
void CyFxBulkLpApplnInit (void);
CyU3PReturnStatus_t BulkLpAppThread_Entry (uint32_t);
CyU3PReturnStatus_t DataRxHandlerThread_Entry (uint32_t);
void CyFxApplicationDefine (void);
static void FxAppMemCorruptCb (void *);
void CyU3PBusyWait_mS(uint32_t );
void SPIDataReadErrorHandler();
void DebuglogInit();


#endif /* PDMANUFACTURERMAIN_H_ */
