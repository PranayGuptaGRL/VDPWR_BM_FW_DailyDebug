/*******************************************************************************
* File Name: MasterInput.c  
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
#include "MasterInput.h"

static MasterInput_BACKUP_STRUCT  MasterInput_backup = {0u, 0u, 0u};


/*******************************************************************************
* Function Name: MasterInput_Sleep
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
*  \snippet MasterInput_SUT.c usage_MasterInput_Sleep_Wakeup
*******************************************************************************/
void MasterInput_Sleep(void)
{
    #if defined(MasterInput__PC)
        MasterInput_backup.pcState = MasterInput_PC;
    #else
        #if (CY_PSOC4_4200L)
            /* Save the regulator state and put the PHY into suspend mode */
            MasterInput_backup.usbState = MasterInput_CR1_REG;
            MasterInput_USB_POWER_REG |= MasterInput_USBIO_ENTER_SLEEP;
            MasterInput_CR1_REG &= MasterInput_USBIO_CR1_OFF;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(MasterInput__SIO)
        MasterInput_backup.sioState = MasterInput_SIO_REG;
        /* SIO requires unregulated output buffer and single ended input buffer */
        MasterInput_SIO_REG &= (uint32)(~MasterInput_SIO_LPM_MASK);
    #endif  
}


/*******************************************************************************
* Function Name: MasterInput_Wakeup
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
*  Refer to MasterInput_Sleep() for an example usage.
*******************************************************************************/
void MasterInput_Wakeup(void)
{
    #if defined(MasterInput__PC)
        MasterInput_PC = MasterInput_backup.pcState;
    #else
        #if (CY_PSOC4_4200L)
            /* Restore the regulator state and come out of suspend mode */
            MasterInput_USB_POWER_REG &= MasterInput_USBIO_EXIT_SLEEP_PH1;
            MasterInput_CR1_REG = MasterInput_backup.usbState;
            MasterInput_USB_POWER_REG &= MasterInput_USBIO_EXIT_SLEEP_PH2;
        #endif
    #endif
    #if defined(CYIPBLOCK_m0s8ioss_VERSION) && defined(MasterInput__SIO)
        MasterInput_SIO_REG = MasterInput_backup.sioState;
    #endif
}


/* [] END OF FILE */
