/*******************************************************************************
* File Name: MasterInput.h  
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

#if !defined(CY_PINS_MasterInput_ALIASES_H) /* Pins MasterInput_ALIASES_H */
#define CY_PINS_MasterInput_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define MasterInput_0			(MasterInput__0__PC)
#define MasterInput_0_PS		(MasterInput__0__PS)
#define MasterInput_0_PC		(MasterInput__0__PC)
#define MasterInput_0_DR		(MasterInput__0__DR)
#define MasterInput_0_SHIFT	(MasterInput__0__SHIFT)
#define MasterInput_0_INTR	((uint16)((uint16)0x0003u << (MasterInput__0__SHIFT*2u)))

#define MasterInput_INTR_ALL	 ((uint16)(MasterInput_0_INTR))


#endif /* End Pins MasterInput_ALIASES_H */


/* [] END OF FILE */
