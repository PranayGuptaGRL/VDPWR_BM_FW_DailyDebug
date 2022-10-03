/*******************************************************************************
* File Name: DIR_CTRL_C.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_DIR_CTRL_C_H) /* Pins DIR_CTRL_C_H */
#define CY_PINS_DIR_CTRL_C_H

#include "cytypes.h"
#include "cyfitter.h"
#include "DIR_CTRL_C_aliases.h"


/***************************************
*     Data Struct Definitions
***************************************/

/**
* \addtogroup group_structures
* @{
*/
    
/* Structure for sleep mode support */
typedef struct
{
    uint32 pcState; /**< State of the port control register */
    uint32 sioState; /**< State of the SIO configuration */
    uint32 usbState; /**< State of the USBIO regulator */
} DIR_CTRL_C_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   DIR_CTRL_C_Read(void);
void    DIR_CTRL_C_Write(uint8 value);
uint8   DIR_CTRL_C_ReadDataReg(void);
#if defined(DIR_CTRL_C__PC) || (CY_PSOC4_4200L) 
    void    DIR_CTRL_C_SetDriveMode(uint8 mode);
#endif
void    DIR_CTRL_C_SetInterruptMode(uint16 position, uint16 mode);
uint8   DIR_CTRL_C_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void DIR_CTRL_C_Sleep(void); 
void DIR_CTRL_C_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(DIR_CTRL_C__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define DIR_CTRL_C_DRIVE_MODE_BITS        (3)
    #define DIR_CTRL_C_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - DIR_CTRL_C_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the DIR_CTRL_C_SetDriveMode() function.
         *  @{
         */
        #define DIR_CTRL_C_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define DIR_CTRL_C_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define DIR_CTRL_C_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define DIR_CTRL_C_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define DIR_CTRL_C_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define DIR_CTRL_C_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define DIR_CTRL_C_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define DIR_CTRL_C_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define DIR_CTRL_C_MASK               DIR_CTRL_C__MASK
#define DIR_CTRL_C_SHIFT              DIR_CTRL_C__SHIFT
#define DIR_CTRL_C_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in DIR_CTRL_C_SetInterruptMode() function.
     *  @{
     */
        #define DIR_CTRL_C_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define DIR_CTRL_C_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define DIR_CTRL_C_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define DIR_CTRL_C_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(DIR_CTRL_C__SIO)
    #define DIR_CTRL_C_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(DIR_CTRL_C__PC) && (CY_PSOC4_4200L)
    #define DIR_CTRL_C_USBIO_ENABLE               ((uint32)0x80000000u)
    #define DIR_CTRL_C_USBIO_DISABLE              ((uint32)(~DIR_CTRL_C_USBIO_ENABLE))
    #define DIR_CTRL_C_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define DIR_CTRL_C_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define DIR_CTRL_C_USBIO_ENTER_SLEEP          ((uint32)((1u << DIR_CTRL_C_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << DIR_CTRL_C_USBIO_SUSPEND_DEL_SHIFT)))
    #define DIR_CTRL_C_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << DIR_CTRL_C_USBIO_SUSPEND_SHIFT)))
    #define DIR_CTRL_C_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << DIR_CTRL_C_USBIO_SUSPEND_DEL_SHIFT)))
    #define DIR_CTRL_C_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(DIR_CTRL_C__PC)
    /* Port Configuration */
    #define DIR_CTRL_C_PC                 (* (reg32 *) DIR_CTRL_C__PC)
#endif
/* Pin State */
#define DIR_CTRL_C_PS                     (* (reg32 *) DIR_CTRL_C__PS)
/* Data Register */
#define DIR_CTRL_C_DR                     (* (reg32 *) DIR_CTRL_C__DR)
/* Input Buffer Disable Override */
#define DIR_CTRL_C_INP_DIS                (* (reg32 *) DIR_CTRL_C__PC2)

/* Interrupt configuration Registers */
#define DIR_CTRL_C_INTCFG                 (* (reg32 *) DIR_CTRL_C__INTCFG)
#define DIR_CTRL_C_INTSTAT                (* (reg32 *) DIR_CTRL_C__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define DIR_CTRL_C_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(DIR_CTRL_C__SIO)
    #define DIR_CTRL_C_SIO_REG            (* (reg32 *) DIR_CTRL_C__SIO)
#endif /* (DIR_CTRL_C__SIO_CFG) */

/* USBIO registers */
#if !defined(DIR_CTRL_C__PC) && (CY_PSOC4_4200L)
    #define DIR_CTRL_C_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define DIR_CTRL_C_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define DIR_CTRL_C_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define DIR_CTRL_C_DRIVE_MODE_SHIFT       (0x00u)
#define DIR_CTRL_C_DRIVE_MODE_MASK        (0x07u << DIR_CTRL_C_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins DIR_CTRL_C_H */


/* [] END OF FILE */
