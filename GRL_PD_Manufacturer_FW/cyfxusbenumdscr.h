/*
 ## Cypress USB 3.0 Platform header file (cyfxusbenumdscr.h)
 ## ===========================
 ##
 ##  Copyright Cypress Semiconductor Corporation, 2010-2018,
 ##  All Rights Reserved
 ##  UNPUBLISHED, LICENSED SOFTWARE.
 ##
 ##  CONFIDENTIAL AND PROPRIETARY INFORMATION
 ##  WHICH IS THE PROPERTY OF CYPRESS.
 ##
 ##  Use of this file is governed
 ##  by the license agreement included in the file
 ##
 ##     <install>/license/license.txt
 ##
 ##  where <install> is the Cypress software
 ##  installation root directory path.
 ##
 ## ===========================
*/

/* This file contains the externants used by the bulk loop application example */

#ifndef _INCLUDED_CYFXUSBENUMDSCR_H_
#define _INCLUDED_CYFXUSBENUMDSCR_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define CY_FX_PDPWR_DMA_BUF_COUNT      (64)                       /* PDPWR loop channel buffer count */
#define CY_FX_PDPWR_DMA_TX_SIZE        (0)                       /* PDPWR transfer size is set to infinite */
#define CY_FX_PDPWR_THREAD_STACK       (0x400)                  /* PD PWR application thread stack size */
#define CY_FX_PDPWR_THREAD_PRIORITY    (8)                       /* PD PWR application thread priority */

#define CY_FX_I2C_DMA_BUF_COUNT      		(8)                       /* I2C channel buffer count */
#define CY_FX_I2C_DMA_TX_SIZE        		(0)                       /* DMA transfer size is set to infinite */
#define CY_FX_DATA_RX_THREAD_STACK       	(0x2000)                  /*  RS485 application thread stack size */
#define CY_FX_DATA_RX_THREAD_PRIORITY    	(9)                       /* RS485 application thread priority */

#define CY_FX_TIMER_THREAD_STACK       	(0x1000)                  /*  TIMER application thread stack size */
#define CY_FX_TIMER_THREAD_PRIORITY    	(10)                       /* TIMER application thread priority */

#define CY_FX_DATA_QUEUE_SIZE		(64)

#define CY_FX_PDSS_COMM_THREAD_STACK       (0x800)				/*  PD Communication application thread stack size */
#define CY_FX_PDSS_COMM_THREAD_PRIORITY    (7)					/* PD Communication application thread priority */
#ifdef IDLE_THREAD

#define CY_FX_IDLE_THREAD_PRIORITY    	(20)                       /* IDLE thread priority */
#define CY_FX_IDLE_THREAD_STACK       	(0x200)                  /*  application thread stack size */

#endif
/* Endpoint and socket definitions for the bulkloop application */

/* To change the producer and consumer EP enter the appropriate EP numbers for the #defines.
 * In the case of IN endpoints enter EP number along with the direction bit.
 * For eg. EP 6 IN endpoint is 0x86
 *     and EP 6 OUT endpoint is 0x06.
 * To change sockets mention the appropriate socket number in the #defines. */

/* Note: For USB 2.0 the endpoints and corresponding sockets are one-to-one mapped
         i.e. EP 1 is mapped to UIB socket 1 and EP 2 to socket 2 so on */

#define CY_FX_EP_PRODUCER               0x01    /* EP 1 OUT */
#define CY_FX_EP_CONSUMER               0x81    /* EP 1 IN */

#define CY_FX_EP_PRODUCER_SOCKET        CY_U3P_UIB_SOCKET_PROD_1    /* Socket 1 is producer */
#define CY_FX_EP_CONSUMER_SOCKET        CY_U3P_UIB_SOCKET_CONS_1    /* Socket 1 is consumer */

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];
extern uint8_t CyFxUSBProductSerNumDscr[];

void SetProductSerNumDesc(uint8_t*);

#include "cyu3externcend.h"

#endif /* _INCLUDED_CYFXUSBENUMDSCR_H_ */

/*[]*/
