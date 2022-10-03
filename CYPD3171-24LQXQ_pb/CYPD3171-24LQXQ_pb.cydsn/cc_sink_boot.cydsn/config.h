/**
 * @file config.h
 *
 * @brief @{Header file that enables/disables various CCG firmware features.
 *
 * This file also provides mapping to the implementation for hardware dependent
 * functions like FET control, voltage selection etc.
 *
 * This current implementation matches the thunderbolt active cable reference
 * design from Cypress. This can be updated by users to match their hardware 
 * implementation.@}
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

#ifndef __CONFIG_H__
#define __CONFIG_H__
    
#include <stack_params.h>

#define CCG_BOOT                        (1u)

/* Enable PD. */
#define CCG_PD_ENABLE                   (1u)

/* Enable/Disable BIST Support. */
#define ENABLE_BIST_SUPPORT             (0u)

/* Enable Non-Blocking Flash Update. */
#define FLASH_ENABLE_NB_MODE            (0u)

/* Disable Boot Wait Window. */
#define BOOT_WAIT_WINDOW_DISABLE        (1u)

/* Enabling flashing of the device via PD interface. */
#define FLASHING_MODE_PD_ENABLE         (1u)
    
/* Enable image selection based on APP Priority Field. */
#define APP_PRIORITY_FEATURE_ENABLE         (0u)

/* Enable Secure Boot. */
#define SECURE_FW_UPDATE                    (0u)
 
/* Source bootloader application*/    
#define SOURCE_BOOT                     (0u)

/* Enable Internal UVP Comparator to be used as VBUS divider. */
#define VBUS_MON_INTERNAL               (1u)

/* Single firmware Image*/    
#define CCG_DUALAPP_DISABLE             (1u)
    
/* Disable Pseudo-metadata feature */
#define CCG_PSEUDO_METADATA_DISABLE     (1u)
    
/* Enabling flashing of the device via PD interface. */
#define FLASHING_MODE_PD_ENABLE         (1u)

/*******************************************************************************
 * VBus Control Type
 ******************************************************************************/

/* VBUS Control using PWM. */
#define VBUS_CTRL_PWM                               (1)

/* VBUS Control using Direct Feedback. */
#define VBUS_CTRL_DIR_FB                            (2)

/* VBUS Control using Opto Feedback. */
#define VBUS_CTRL_OPTO_FB                           (3)

/* This is the type used by the application. */
#define VBUS_CTRL_TYPE                              (VBUS_CTRL_DIR_FB)
#endif /*__CONFIG_H__ */

/* [] END OF FILE */
