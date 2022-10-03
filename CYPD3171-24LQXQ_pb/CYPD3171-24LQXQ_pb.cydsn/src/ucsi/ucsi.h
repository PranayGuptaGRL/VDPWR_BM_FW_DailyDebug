/**
 * @file ucsi.h
 *
 * @brief @{USB Type-C Connector System Software Interface (UCSI) header file.@}
 */

/*
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#ifndef _UCSI_H_
#define _UCSI_H_

#include <stdbool.h>
#include <stdint.h>
#include <config.h>
#include "status.h"
#include "i2c.h"
#include "timer.h"
#include "ucsi_internal.h"

#include "pd.h"
#include "dpm.h"


#define UCSI_NOTIFICATION_EN_ALL        (0xDBE7)
/**< Value to enable all (required & optional) UCSI Notification. */

#define UCSI_NOTIFICATION_EN_REQ        (0xDA05)
/**< Value to enable only Required UCSI Notification. */

#define UCSI_BUFFER_SIZE                (HPI_BUFFER_SIZE)
/**< Size of data receive buffer to be used for the UCSI interface. This size includes two bytes of
     register address and 256 bytes corresponding to the maximum write size. */

#if CCG_UCSI_ENABLE_DIFF_ADDR
#define UCSI_MIN_WRITE_SIZE             (3u)
/**< Minimum UCSI write size: Two address bytes + 1 data byte. */
#endif /*CCG_UCSI_ENABLE_DIFF_ADDR*/

#define MAX_DPM_CMD_RETRY_COUNT         (6u)    /**< Maximum retry count for DPM commands. */

#define CABLE_CURR_3A	                (60u)   /**< Type-C cable current capacity of 3A represented in 50 mA units. */
#define CABLE_CURR_5A	                (100u)  /**< Type-C cable current capacity of 5A represented in 50 mA units. */

#define CABLE_SPEED_480MBPS             (0x7820)        /**< USB 2.0 Type-C cable data speed capability (480 Mbps) */
#define CABLE_SPEED_5GBPS               (0x0017)        /**< USB 3.0 Type-C cable data speed capability (5 Gbps). */
#define CABLE_SPEED_10GBPS              (0x002B)        /**< USB 3.1 Type-C cable data speed capability (10 Gbps). */

#define CABLE_VDO_DIRECTION             (0x0780)        /**< Mask for directionality flags in USB-PD Cable VDO. */

#define ALT_MODES_RECIPIENT_CONNECTOR   (0u)            /**< Connector value for Alternate mode recipient. */
#define ALT_MODES_RECIPIENT_SOP         (1u)            /**< SOP value for Alternate mode recipient. */
#define ALT_MODES_RECIPIENT_SOP_PRIME   (2u)            /**< SOP' value for Alternate mode recipient. */
#define ALT_MODES_RECIPIENT_SOP_DPRIME  (3u)            /**< SOP'' value for Alternate mode recipient. */

#define VDM_DISCOVER_ID                 (1u)            /**< VDM command value for Discover Identity. */
#define VDM_DISCOVER_SVID               (2u)            /**< VDM command value for Discover SVID */
#define VDM_DISCOVER_MODES              (3u)            /**< VDM command value for Discover Modes */

#define SRC_CUR_LEVEL_DEF               (0x00)          /**< Current source value: Type-C default (900 mA). */
#define SRC_CUR_LEVEL_1_5A              (0x01)          /**< Current source value: 1.5 A */
#define SRC_CUR_LEVEL_3A                (0x02)          /**< Current source value: 3 A */

#define UCSI_SET_RP_PPM_DEFAULT             (0)         /**< UCSI command to set PPM power level to default value. */
#define UCSI_SET_RP_3A                      (1)         /**< UCSI command to set PPM power level to 3 A. */
#define UCSI_SET_RP_1_5A                    (2)         /**< UCSI command to set PPM power level to 1.5 A. */
#define UCSI_SET_RP_DEF                     (3)         /**< UCSI command to set PPM power level to Type-C default. */

#define POM_NO_CONSUMER                 (0x00)          /**< Power Operation Mode (POM) of connector: reserved. */
#define POM_CUR_LEVEL_DEF               (0x01)          /**< Power Operation Mode (POM) of connector: Type-C default. */
#define POM_BC                          (0x02)          /**< Power Operation Mode (POM) of connector: BC 1.2 */
#define POM_PD                          (0x03)          /**< Power Operation Mode (POM) of connector: USB-PD. */

#define POM_CUR_LEVEL_1_5A              (0x04)          /**< Power Operation Mode (POM) of connector: Type-C 1.5 A. */
#define POM_CUR_LEVEL_3A                (0x05)          /**< Power Operation Mode (POM) of connector: Type-C 3 A. */

#define UCSI_READ_PENDING_EVENT         (7)
/**< UCSI Read Pending status bit in INTERRUPT register.
 * Note: In the AMD TED platform, Bit 3 of the INTERRUPT register is used to indicate
 * that a UCSI read is pending. This is changed to bit 7 in the final spec that was released
 * to other customers.
 * Change as necessary.
 */

#define UCSI_READ_PENDING_MASK          (1 << UCSI_READ_PENDING_EVENT)
/**< Mask to be applied for UCSI Read Pending Event. */

/*****************************************************************************
 **************************** Function prototypes ****************************
 *****************************************************************************/

#if CCG_UCSI_ENABLE_DIFF_ADDR

/**
 *  @brief I2C command callback for UCSI Implemenation.
 *  @param i2c_cb_cmd_t Type of I2C operation
 *  @param i2c_scb_state_t I2C block states
 *  @param count write size
 *  @return true if I2C operation is successful, false otherwise.
 */
bool ucsi_i2c_cmd_callback(i2c_cb_cmd_t cmd, i2c_scb_state_t i2c_state, uint16_t count);

#endif/*CCG_UCSI_ENABLE_DIFF_ADDR*/

/**
 * @brief Initialize the UCSI interface.
 * @return None
 */
void ucsi_init(void);

/* Run the UCSI state machine */

/**
 * @brief UCSI task handler.
 *
 * This function handles the commands from the EC through the UCSI registers. UCSI writes from the EC
 * are handled in interrupt context, all UCSI commands are sent in sequence by OPM. There is no event
 * queue maintained for UCSI interface. Instead, CCGx maintains a bit map of pending events for each port.
 * CCGx forwards one connector change at a time to OPM. OPM can retrieve the connector status and receive
 * a bit-map of events.
 * The ucsi_task is expected to be called periodically from the main task loop of the firmware
 * application.
 *
 * @return None
 */
void ucsi_task(void);

/**
 *  @brief Check any change in port connection.
 *  @param port port index
 *
 *  @return true if the port connect is changed, false otherwise.
 */
bool is_port_connect_changed(uint8_t port);

/**
 * @brief Handler for PD events reported from the stack.
 * Internal function used to receive PD events from the stack and to update the UCSI registers.
 *
 * @param port PD port corresponding to the event.
 * @param evt Event that is being notified.
 * @param data Data associated with the event. This is an opaque pointer that needs to be de-referenced
 * based on event type.
 *
 * @return None
 */
void ucsi_pd_event_handler(uint8_t port, app_evt_t evt, const void *data);

/**
 *  @brief Function to update UCSI notifications.
 *  @param port port index
 *  @param notification UCSI notification to be sent.
 *  @return None
 */
void ucsi_notify(uint8_t port, uint16_t notification);

/**
 *  @brief Function to configure VDM for getting Alt Mode SVIDs & Modes
 *  @param cmd_sop Enum of the SOP (Start Of Frame) types.
 *  @param svid Standard ID / Vendor ID
 *  @param cmd DPM command
 *  @return None
 */
void ucsi_configure_send_vdm(sop_t cmd_sop, uint32_t svid, uint32_t cmd);

/**
 *  @brief Check if the UCSI module can be put to sleep.
 *  @return true if the UCSI module can be put to sleep, false otherwise.
 */
bool ucsi_sleep_allowed(void);

#endif /* _UCSI_H_ */

/* [] END OF FILE */
