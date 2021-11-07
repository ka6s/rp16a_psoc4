/*******************************************************************************
* File Name: ant3.h  
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

#if !defined(CY_PINS_ant3_ALIASES_H) /* Pins ant3_ALIASES_H */
#define CY_PINS_ant3_ALIASES_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"


/***************************************
*              Constants        
***************************************/
#define ant3_0			(ant3__0__PC)
#define ant3_0_PS		(ant3__0__PS)
#define ant3_0_PC		(ant3__0__PC)
#define ant3_0_DR		(ant3__0__DR)
#define ant3_0_SHIFT	(ant3__0__SHIFT)
#define ant3_0_INTR	((uint16)((uint16)0x0003u << (ant3__0__SHIFT*2u)))

#define ant3_INTR_ALL	 ((uint16)(ant3_0_INTR))


#endif /* End Pins ant3_ALIASES_H */


/* [] END OF FILE */
