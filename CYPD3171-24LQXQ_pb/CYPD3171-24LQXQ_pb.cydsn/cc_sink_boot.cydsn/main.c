/*******************************************************************************
 * File Name: main.c
 *
 * @brief @{Main source file for CCG firmware implementation.@}
 *
 *******************************************************************************
 * Â© (2014-2018), Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation (Cypress) and is
 * protected by and subject to worldwide patent protection (United States and
 * foreign), United States copyright laws and international treaty provisions.
 * Cypress hereby grants to licensee a personal, non-exclusive,
 * non-transferable license to copy, use, modify, create derivative works of,
 * and compile the Cypress source code and derivative works for the sole
 * purpose of creating custom software in support of licensee product, such
 * licensee product to be used only in conjunction with Cypress's integrated
 * circuit as specified in the applicable agreement. Any reproduction,
 * modification, translation, compilation, or representation of this Software
 * except as specified above is prohibited without the express written
 * permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * Cypress reserves the right to make changes to the Software without notice.
 * Cypress does not assume any liability arising out of the application or use
 * of Software or any product or circuit described in the Software.
 * Cypress does not authorize its products for use as critical components in
 * any products where a malfunction or failure may reasonably be expected to
 * result in significant injury or death ("High Risk Product"). By including
 * Cypress's product in a High Risk Product, the manufacturer of such system or
 * application assumes all risk of such use and in doing so indemnifies Cypress
 * against all liability.
 *
 * Use of this Software may be limited by and subject to the applicable Cypress
 * software license agreement.
 ******************************************************************************/
#include "project.h"
#include "config.h"
#include "system.h"
#include "gpio.h"
#include "flash.h"
#include "flash_config.h"
#include "ccgx_version.h"
#include "ccgx_regs.h"
#include "cc_boot.h"
#include "boot.h"
#include "app_version.h"
#include "stack_params.h"
#include "utils.h"
#include "pdss_hal.h"
#include "hal_ccgx.h"

/* Place following information in last 32 bytes of the first 256 bytes section. */
__attribute__ ((section(".base_version"), used))
const uint32_t base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
const uint32_t app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
const uint32_t ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".boot_type"), used))
const uint32_t boot_loader_type = ((APP_PRIORITY_FEATURE_ENABLE << SYS_BOOT_TYPE_APP_PRIORITY_POS)
    | ((~FLASHING_MODE_PD_ENABLE & 0x00000001) << SYS_BOOT_TYPE_FW_UPDATE_INTERFACE_POS)
    | SECURE_FW_UPDATE);
__attribute__ ((section(".fw_reserved"), used))
const uint32_t reserved_buf[4] = {0};

#if (FLASHING_MODE_PD_ENABLE == 1)
ccg_status_t uvdm_handle_device_reset(uint32_t reset_sig)
{
    /* Disable all interrupts. */
    CyGlobalIntDisable;

#if !CCG_FIRMWARE_APP_ONLY
    /*
     * NOTE: Using Bootloader component provided variable
     * to store the signature. This makes sure that this value
     * is never overwritten by compiler and it ratains the value
     * through resets and jumps. We use lower two bytes of this
     * variable to store the siganture. */
    /* NOTE: Signature will be zero for RESET command. */
    cyBtldrRunType |= reset_sig;
#endif /* CCG_FIRMWARE_APP_ONLY */

    /*
     * Workaround for Vregulator issue. Ensure bootloader removes termianions, disables
     * regulator and then goes through Reset.
     */
    PDSS->vreg_vsys_ctrl &= ~(PDSS_VREG_VSYS_CTRL_VREG_EN);
    PDSS->cc_ctrl_0 &= ~(PDSS_CC_CTRL_0_RD_CC1_EN | PDSS_CC_CTRL_0_RD_CC2_EN);
    CyDelay (30);
    /* Call device reset routine. */
    CySoftwareReset ();

    return CCG_STAT_NO_RESPONSE;
}
#endif /*(FLASHING_MODE_PD_ENABLE == 1) */


int main(void)
{
    /* If boot_start returns FALSE, this means CCGx has to remain
     * in boot mode as either no valid FW image exists or FW has made
     * a request to be in boot mode. If boot_start returns TRUE, this
     * means valid FW image exists. In that case jump to FW. */
    if (true == boot_start ())
    {
      /* Jump to scheduled FW. */
       boot_jump_to_fw ();
    }
    else
    {
#if (FLASHING_MODE_PD_ENABLE == 1)
    /* Set the flash access boundaries so that the boot-loader itself cannot be overwritten. */
    flash_set_access_limits (CCG_BOOT_LOADER_LAST_ROW + 1, CCG_LAST_FLASH_ROW_NUM,
            CCG_LAST_FLASH_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);
#endif /* FLASHING_MODE_PD_ENABLE */

        pdss_phy_init();

        /* Enable system interrupts. */
        CyGlobalIntEnable;
    }

    while (1)
    {
        typec_state_machine();
        pd_state_machine();
    }
}
