/*******************************************************************************
* File Name: ant3.h  
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

#if !defined(CY_PINS_ant3_H) /* Pins ant3_H */
#define CY_PINS_ant3_H

#include "cytypes.h"
#include "cyfitter.h"
#include "ant3_aliases.h"


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
} ant3_BACKUP_STRUCT;

/** @} structures */


/***************************************
*        Function Prototypes             
***************************************/
/**
* \addtogroup group_general
* @{
*/
uint8   ant3_Read(void);
void    ant3_Write(uint8 value);
uint8   ant3_ReadDataReg(void);
#if defined(ant3__PC) || (CY_PSOC4_4200L) 
    void    ant3_SetDriveMode(uint8 mode);
#endif
void    ant3_SetInterruptMode(uint16 position, uint16 mode);
uint8   ant3_ClearInterrupt(void);
/** @} general */

/**
* \addtogroup group_power
* @{
*/
void ant3_Sleep(void); 
void ant3_Wakeup(void);
/** @} power */


/***************************************
*           API Constants        
***************************************/
#if defined(ant3__PC) || (CY_PSOC4_4200L) 
    /* Drive Modes */
    #define ant3_DRIVE_MODE_BITS        (3)
    #define ant3_DRIVE_MODE_IND_MASK    (0xFFFFFFFFu >> (32 - ant3_DRIVE_MODE_BITS))

    /**
    * \addtogroup group_constants
    * @{
    */
        /** \addtogroup driveMode Drive mode constants
         * \brief Constants to be passed as "mode" parameter in the ant3_SetDriveMode() function.
         *  @{
         */
        #define ant3_DM_ALG_HIZ         (0x00u) /**< \brief High Impedance Analog   */
        #define ant3_DM_DIG_HIZ         (0x01u) /**< \brief High Impedance Digital  */
        #define ant3_DM_RES_UP          (0x02u) /**< \brief Resistive Pull Up       */
        #define ant3_DM_RES_DWN         (0x03u) /**< \brief Resistive Pull Down     */
        #define ant3_DM_OD_LO           (0x04u) /**< \brief Open Drain, Drives Low  */
        #define ant3_DM_OD_HI           (0x05u) /**< \brief Open Drain, Drives High */
        #define ant3_DM_STRONG          (0x06u) /**< \brief Strong Drive            */
        #define ant3_DM_RES_UPDWN       (0x07u) /**< \brief Resistive Pull Up/Down  */
        /** @} driveMode */
    /** @} group_constants */
#endif

/* Digital Port Constants */
#define ant3_MASK               ant3__MASK
#define ant3_SHIFT              ant3__SHIFT
#define ant3_WIDTH              1u

/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in ant3_SetInterruptMode() function.
     *  @{
     */
        #define ant3_INTR_NONE      ((uint16)(0x0000u)) /**< \brief Disabled             */
        #define ant3_INTR_RISING    ((uint16)(0x5555u)) /**< \brief Rising edge trigger  */
        #define ant3_INTR_FALLING   ((uint16)(0xaaaau)) /**< \brief Falling edge trigger */
        #define ant3_INTR_BOTH      ((uint16)(0xffffu)) /**< \brief Both edge trigger    */
    /** @} intrMode */
/** @} group_constants */

/* SIO LPM definition */
#if defined(ant3__SIO)
    #define ant3_SIO_LPM_MASK       (0x03u)
#endif

/* USBIO definitions */
#if !defined(ant3__PC) && (CY_PSOC4_4200L)
    #define ant3_USBIO_ENABLE               ((uint32)0x80000000u)
    #define ant3_USBIO_DISABLE              ((uint32)(~ant3_USBIO_ENABLE))
    #define ant3_USBIO_SUSPEND_SHIFT        CYFLD_USBDEVv2_USB_SUSPEND__OFFSET
    #define ant3_USBIO_SUSPEND_DEL_SHIFT    CYFLD_USBDEVv2_USB_SUSPEND_DEL__OFFSET
    #define ant3_USBIO_ENTER_SLEEP          ((uint32)((1u << ant3_USBIO_SUSPEND_SHIFT) \
                                                        | (1u << ant3_USBIO_SUSPEND_DEL_SHIFT)))
    #define ant3_USBIO_EXIT_SLEEP_PH1       ((uint32)~((uint32)(1u << ant3_USBIO_SUSPEND_SHIFT)))
    #define ant3_USBIO_EXIT_SLEEP_PH2       ((uint32)~((uint32)(1u << ant3_USBIO_SUSPEND_DEL_SHIFT)))
    #define ant3_USBIO_CR1_OFF              ((uint32)0xfffffffeu)
#endif


/***************************************
*             Registers        
***************************************/
/* Main Port Registers */
#if defined(ant3__PC)
    /* Port Configuration */
    #define ant3_PC                 (* (reg32 *) ant3__PC)
#endif
/* Pin State */
#define ant3_PS                     (* (reg32 *) ant3__PS)
/* Data Register */
#define ant3_DR                     (* (reg32 *) ant3__DR)
/* Input Buffer Disable Override */
#define ant3_INP_DIS                (* (reg32 *) ant3__PC2)

/* Interrupt configuration Registers */
#define ant3_INTCFG                 (* (reg32 *) ant3__INTCFG)
#define ant3_INTSTAT                (* (reg32 *) ant3__INTSTAT)

/* "Interrupt cause" register for Combined Port Interrupt (AllPortInt) in GSRef component */
#if defined (CYREG_GPIO_INTR_CAUSE)
    #define ant3_INTR_CAUSE         (* (reg32 *) CYREG_GPIO_INTR_CAUSE)
#endif

/* SIO register */
#if defined(ant3__SIO)
    #define ant3_SIO_REG            (* (reg32 *) ant3__SIO)
#endif /* (ant3__SIO_CFG) */

/* USBIO registers */
#if !defined(ant3__PC) && (CY_PSOC4_4200L)
    #define ant3_USB_POWER_REG       (* (reg32 *) CYREG_USBDEVv2_USB_POWER_CTRL)
    #define ant3_CR1_REG             (* (reg32 *) CYREG_USBDEVv2_CR1)
    #define ant3_USBIO_CTRL_REG      (* (reg32 *) CYREG_USBDEVv2_USB_USBIO_CTRL)
#endif    
    
    
/***************************************
* The following code is DEPRECATED and 
* must not be used in new designs.
***************************************/
/**
* \addtogroup group_deprecated
* @{
*/
#define ant3_DRIVE_MODE_SHIFT       (0x00u)
#define ant3_DRIVE_MODE_MASK        (0x07u << ant3_DRIVE_MODE_SHIFT)
/** @} deprecated */

#endif /* End Pins ant3_H */


/* [] END OF FILE */
