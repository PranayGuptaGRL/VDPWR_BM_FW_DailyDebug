/*******************************************************************************
* File Name: MasterInputInt.h
* Version 1.70
*
*  Description:
*   Provides the function definitions for the Interrupt Controller.
*
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(CY_ISR_MasterInputInt_H)
#define CY_ISR_MasterInputInt_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void MasterInputInt_Start(void);
void MasterInputInt_StartEx(cyisraddress address);
void MasterInputInt_Stop(void);

CY_ISR_PROTO(MasterInputInt_Interrupt);

void MasterInputInt_SetVector(cyisraddress address);
cyisraddress MasterInputInt_GetVector(void);

void MasterInputInt_SetPriority(uint8 priority);
uint8 MasterInputInt_GetPriority(void);

void MasterInputInt_Enable(void);
uint8 MasterInputInt_GetState(void);
void MasterInputInt_Disable(void);

void MasterInputInt_SetPending(void);
void MasterInputInt_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the MasterInputInt ISR. */
#define MasterInputInt_INTC_VECTOR            ((reg32 *) MasterInputInt__INTC_VECT)

/* Address of the MasterInputInt ISR priority. */
#define MasterInputInt_INTC_PRIOR             ((reg32 *) MasterInputInt__INTC_PRIOR_REG)

/* Priority of the MasterInputInt interrupt. */
#define MasterInputInt_INTC_PRIOR_NUMBER      MasterInputInt__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable MasterInputInt interrupt. */
#define MasterInputInt_INTC_SET_EN            ((reg32 *) MasterInputInt__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the MasterInputInt interrupt. */
#define MasterInputInt_INTC_CLR_EN            ((reg32 *) MasterInputInt__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the MasterInputInt interrupt state to pending. */
#define MasterInputInt_INTC_SET_PD            ((reg32 *) MasterInputInt__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the MasterInputInt interrupt. */
#define MasterInputInt_INTC_CLR_PD            ((reg32 *) MasterInputInt__INTC_CLR_PD_REG)



#endif /* CY_ISR_MasterInputInt_H */


/* [] END OF FILE */
