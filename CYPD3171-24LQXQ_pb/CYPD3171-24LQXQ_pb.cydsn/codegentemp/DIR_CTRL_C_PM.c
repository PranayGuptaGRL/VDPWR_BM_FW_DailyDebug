/*******************************************************************************
* File Name: DIR_CTRL_C.c  
* Version 2.20
*
* Description:
*  This file contains APIs to set up the Pins component for low power modes.
*
* Note:
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "DIR_CTRL_C.h"

static DIR_CTRL_C_BACKUP_STRUCT  DIR_CTRL_C_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: DIR_CTRL_C_Sleep
****************************************************************************//**
*
* \brief Stores the pin configuration and prepares the pin for entering chip 
*  deep-sleep/hibernate modes. This function applies only to SIO and USBIO pins.
*  It should not be called for GPIO or GPIO_OVT pins.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None 
*  
* \sideeffect
*  For SIO pins, this function configures the pin input threshold to CMOS and
*  drive level to Vddio. This is needed for SIO pins when in device 
*  deep-sleep/hibernate modes.
*
* \funcusage
*  \snippet DIR_CTRL_C_SUT.c usage_DIR_CTRL_C_Sleep_Wakeup
*******************************************************************************/
void DIR_CTRL_C_Sleep(void)
{
    #if defined(DIR_CTRL_C__PC)
        DIR_CTRL_C_backup.pcState = DIR_CTRL_C_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            DIR_CTRL_C_backup.usbState = DIR_CTRL_C_CR1_REG;
            DIR_CTRL_C_USB_POWER_REG |= DIR_CTRL_C_USBIO_ENTER_SLEEP;
            DIR_CTRL_C_CR1_REG &= DIR_CTRL_C_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(DIR_CTRL_C__SIO)
        DIR_CTRL_C_backup.sioState = DIR_CTRL_C_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        DIR_CTRL_C_SIO_REG &= (uint32)(~DIR_CTRL_C_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: DIR_CTRL_C_Wakeup
****************************************************************************//**
*
* \brief Restores the pin configuration that was saved during Pin_Sleep(). This 
* function applies only to SIO and USBIO pins. It should not be called for
* GPIO or GPIO_OVT pins.
*
* For USBIO pins, the wakeup is only triggered for falling edge interrupts.
*
* <b>Note</b> This function is available in PSoC 4 only.
*
* \return 
*  None
*  
* \funcusage
*  Refer to DIR_CTRL_C_Sleep() for an example usage.
*******************************************************************************/
void DIR_CTRL_C_Wakeup(void)
{
    #if defined(DIR_CTRL_C__PC)
        DIR_CTRL_C_PC = DIR_CTRL_C_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            DIR_CTRL_C_USB_POWER_REG &= DIR_CTRL_C_USBIO_EXIT_SLEEP_PH1;
            DIR_CTRL_C_CR1_REG = DIR_CTRL_C_backup.usbState;
            DIR_CTRL_C_USB_POWER_REG &= DIR_CTRL_C_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(DIR_CTRL_C__SIO)
        DIR_CTRL_C_SIO_REG = DIR_CTRL_C_backup.sioState;
    #endif
}


/* [] END OF FILE */
