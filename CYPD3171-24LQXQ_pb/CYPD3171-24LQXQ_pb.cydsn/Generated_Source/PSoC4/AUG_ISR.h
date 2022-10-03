/*******************************************************************************
* File Name: AUG_ISR.h
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
#if !defined(CY_ISR_AUG_ISR_H)
#define CY_ISR_AUG_ISR_H


#include <cytypes.h>
#include <cyfitter.h>

/* Interrupt Controller API. */
void AUG_ISR_Start(void);
void AUG_ISR_StartEx(cyisraddress address);
void AUG_ISR_Stop(void);

CY_ISR_PROTO(AUG_ISR_Interrupt);

void AUG_ISR_SetVector(cyisraddress address);
cyisraddress AUG_ISR_GetVector(void);

void AUG_ISR_SetPriority(uint8 priority);
uint8 AUG_ISR_GetPriority(void);

void AUG_ISR_Enable(void);
uint8 AUG_ISR_GetState(void);
void AUG_ISR_Disable(void);

void AUG_ISR_SetPending(void);
void AUG_ISR_ClearPending(void);


/* Interrupt Controller Constants */

/* Address of the INTC.VECT[x] register that contains the Address of the AUG_ISR ISR. */
#define AUG_ISR_INTC_VECTOR            ((reg32 *) AUG_ISR__INTC_VECT)

/* Address of the AUG_ISR ISR priority. */
#define AUG_ISR_INTC_PRIOR             ((reg32 *) AUG_ISR__INTC_PRIOR_REG)

/* Priority of the AUG_ISR interrupt. */
#define AUG_ISR_INTC_PRIOR_NUMBER      AUG_ISR__INTC_PRIOR_NUM

/* Address of the INTC.SET_EN[x] byte to bit enable AUG_ISR interrupt. */
#define AUG_ISR_INTC_SET_EN            ((reg32 *) AUG_ISR__INTC_SET_EN_REG)

/* Address of the INTC.CLR_EN[x] register to bit clear the AUG_ISR interrupt. */
#define AUG_ISR_INTC_CLR_EN            ((reg32 *) AUG_ISR__INTC_CLR_EN_REG)

/* Address of the INTC.SET_PD[x] register to set the AUG_ISR interrupt state to pending. */
#define AUG_ISR_INTC_SET_PD            ((reg32 *) AUG_ISR__INTC_SET_PD_REG)

/* Address of the INTC.CLR_PD[x] register to clear the AUG_ISR interrupt. */
#define AUG_ISR_INTC_CLR_PD            ((reg32 *) AUG_ISR__INTC_CLR_PD_REG)



#endif /* CY_ISR_AUG_ISR_H */


/* [] END OF FILE */
