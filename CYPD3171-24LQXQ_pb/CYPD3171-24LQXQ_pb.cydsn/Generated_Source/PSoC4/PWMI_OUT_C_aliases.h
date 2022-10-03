/*******************************************************************************
* File Name: PWMI_OUT_C.h  
* Version 2.20
*
* Description:
*  This file contains the Alias definitions for Per-Pin APIs in cypins.h. 
*  Information on using these APIs can be found in the System Reference Guide.
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_PWMI_OUT_C_ALIASES_H) /* Pins PWMI_OUT_C_ALIASES_H */
#define CY_PINS_PWMI_OUT_C_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define PWMI_OUT_C_0			(PWMI_OUT_C__0__PC)
#define PWMI_OUT_C_0_PS		(PWMI_OUT_C__0__PS)
#define PWMI_OUT_C_0_PC		(PWMI_OUT_C__0__PC)
#define PWMI_OUT_C_0_DR		(PWMI_OUT_C__0__DR)
#define PWMI_OUT_C_0_SHIFT	(PWMI_OUT_C__0__SHIFT)
#define PWMI_OUT_C_0_INTR	((uint16)((uint16)0x0003u << (PWMI_OUT_C__0__SHIFT*2u)))

#define PWMI_OUT_C_INTR_ALL	 ((uint16)(PWMI_OUT_C_0_INTR))


#endif /* End Pins PWMI_OUT_C_ALIASES_H */


/* [] END OF FILE */
