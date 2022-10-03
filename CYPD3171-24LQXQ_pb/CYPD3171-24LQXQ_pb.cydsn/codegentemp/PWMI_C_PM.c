/*******************************************************************************
* File Name: PWMI_C_PM.c
* Version 2.10
*
* Description:
*  This file contains the setup, control, and status commands to support
*  the component operations in the low power mode.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2013-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "PWMI_C.h"

static PWMI_C_BACKUP_STRUCT PWMI_C_backup;


/*******************************************************************************
* Function Name: PWMI_C_SaveConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to save here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_SaveConfig(void)
{

}


/*******************************************************************************
* Function Name: PWMI_C_Sleep
********************************************************************************
*
* Summary:
*  Stops the component operation and saves the user configuration.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_Sleep(void)
{
    if(0u != (PWMI_C_BLOCK_CONTROL_REG & PWMI_C_MASK))
    {
        PWMI_C_backup.enableState = 1u;
    }
    else
    {
        PWMI_C_backup.enableState = 0u;
    }

    PWMI_C_Stop();
    PWMI_C_SaveConfig();
}


/*******************************************************************************
* Function Name: PWMI_C_RestoreConfig
********************************************************************************
*
* Summary:
*  All configuration registers are retention. Nothing to restore here.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_RestoreConfig(void)
{

}


/*******************************************************************************
* Function Name: PWMI_C_Wakeup
********************************************************************************
*
* Summary:
*  Restores the user configuration and restores the enable state.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void PWMI_C_Wakeup(void)
{
    PWMI_C_RestoreConfig();

    if(0u != PWMI_C_backup.enableState)
    {
        PWMI_C_Enable();
    }
}


/* [] END OF FILE */
