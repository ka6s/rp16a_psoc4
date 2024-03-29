/***************************************************************************//**
* \file SCB_1.c
* \version 4.0
*
* \brief
*  This file provides the source code to the API for the SCB Component.
*
* Note:
*
*******************************************************************************
* \copyright
* Copyright 2013-2017, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "SCB_1_PVT.h"

#if (SCB_1_SCB_MODE_I2C_INC)
    #include "SCB_1_I2C_PVT.h"
#endif /* (SCB_1_SCB_MODE_I2C_INC) */

#if (SCB_1_SCB_MODE_EZI2C_INC)
    #include "SCB_1_EZI2C_PVT.h"
#endif /* (SCB_1_SCB_MODE_EZI2C_INC) */

#if (SCB_1_SCB_MODE_SPI_INC || SCB_1_SCB_MODE_UART_INC)
    #include "SCB_1_SPI_UART_PVT.h"
#endif /* (SCB_1_SCB_MODE_SPI_INC || SCB_1_SCB_MODE_UART_INC) */


/***************************************
*    Run Time Configuration Vars
***************************************/

/* Stores internal component configuration for Unconfigured mode */
#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Common configuration variables */
    uint8 SCB_1_scbMode = SCB_1_SCB_MODE_UNCONFIG;
    uint8 SCB_1_scbEnableWake;
    uint8 SCB_1_scbEnableIntr;

    /* I2C configuration variables */
    uint8 SCB_1_mode;
    uint8 SCB_1_acceptAddr;

    /* SPI/UART configuration variables */
    volatile uint8 * SCB_1_rxBuffer;
    uint8  SCB_1_rxDataBits;
    uint32 SCB_1_rxBufferSize;

    volatile uint8 * SCB_1_txBuffer;
    uint8  SCB_1_txDataBits;
    uint32 SCB_1_txBufferSize;

    /* EZI2C configuration variables */
    uint8 SCB_1_numberOfAddr;
    uint8 SCB_1_subAddrSize;
#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */


/***************************************
*     Common SCB Vars
***************************************/
/**
* \addtogroup group_general
* \{
*/

/** SCB_1_initVar indicates whether the SCB_1 
*  component has been initialized. The variable is initialized to 0 
*  and set to 1 the first time SCB_Start() is called. This allows 
*  the component to restart without reinitialization after the first 
*  call to the SCB_1_Start() routine.
*
*  If re-initialization of the component is required, then the 
*  SCB_1_Init() function can be called before the 
*  SCB_1_Start() or SCB_1_Enable() function.
*/
uint8 SCB_1_initVar = 0u;


#if (! (SCB_1_SCB_MODE_I2C_CONST_CFG || \
        SCB_1_SCB_MODE_EZI2C_CONST_CFG))
    /** This global variable stores TX interrupt sources after 
    * SCB_1_Stop() is called. Only these TX interrupt sources 
    * will be restored on a subsequent SCB_1_Enable() call.
    */
    uint16 SCB_1_IntrTxMask = 0u;
#endif /* (! (SCB_1_SCB_MODE_I2C_CONST_CFG || \
              SCB_1_SCB_MODE_EZI2C_CONST_CFG)) */
/** \} globals */

#if (SCB_1_SCB_IRQ_INTERNAL)
#if !defined (CY_REMOVE_SCB_1_CUSTOM_INTR_HANDLER)
    void (*SCB_1_customIntrHandler)(void) = NULL;
#endif /* !defined (CY_REMOVE_SCB_1_CUSTOM_INTR_HANDLER) */
#endif /* (SCB_1_SCB_IRQ_INTERNAL) */


/***************************************
*    Private Function Prototypes
***************************************/

static void SCB_1_ScbEnableIntr(void);
static void SCB_1_ScbModeStop(void);
static void SCB_1_ScbModePostEnable(void);


/*******************************************************************************
* Function Name: SCB_1_Init
****************************************************************************//**
*
*  Initializes the SCB_1 component to operate in one of the selected
*  configurations: I2C, SPI, UART or EZI2C.
*  When the configuration is set to "Unconfigured SCB", this function does
*  not do any initialization. Use mode-specific initialization APIs instead:
*  SCB_1_I2CInit, SCB_1_SpiInit, 
*  SCB_1_UartInit or SCB_1_EzI2CInit.
*
*******************************************************************************/
void SCB_1_Init(void)
{
#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
    if (SCB_1_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SCB_1_initVar = 0u;
    }
    else
    {
        /* Initialization was done before this function call */
    }

#elif (SCB_1_SCB_MODE_I2C_CONST_CFG)
    SCB_1_I2CInit();

#elif (SCB_1_SCB_MODE_SPI_CONST_CFG)
    SCB_1_SpiInit();

#elif (SCB_1_SCB_MODE_UART_CONST_CFG)
    SCB_1_UartInit();

#elif (SCB_1_SCB_MODE_EZI2C_CONST_CFG)
    SCB_1_EzI2CInit();

#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SCB_1_Enable
****************************************************************************//**
*
*  Enables SCB_1 component operation: activates the hardware and 
*  internal interrupt. It also restores TX interrupt sources disabled after the 
*  SCB_1_Stop() function was called (note that level-triggered TX 
*  interrupt sources remain disabled to not cause code lock-up).
*  For I2C and EZI2C modes the interrupt is internal and mandatory for 
*  operation. For SPI and UART modes the interrupt can be configured as none, 
*  internal or external.
*  The SCB_1 configuration should be not changed when the component
*  is enabled. Any configuration changes should be made after disabling the 
*  component.
*  When configuration is set to “Unconfigured SCB_1”, the component 
*  must first be initialized to operate in one of the following configurations: 
*  I2C, SPI, UART or EZ I2C, using the mode-specific initialization API. 
*  Otherwise this function does not enable the component.
*
*******************************************************************************/
void SCB_1_Enable(void)
{
#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
    /* Enable SCB block, only if it is already configured */
    if (!SCB_1_SCB_MODE_UNCONFIG_RUNTM_CFG)
    {
        SCB_1_CTRL_REG |= SCB_1_CTRL_ENABLED;

        SCB_1_ScbEnableIntr();

        /* Call PostEnable function specific to current operation mode */
        SCB_1_ScbModePostEnable();
    }
#else
    SCB_1_CTRL_REG |= SCB_1_CTRL_ENABLED;

    SCB_1_ScbEnableIntr();

    /* Call PostEnable function specific to current operation mode */
    SCB_1_ScbModePostEnable();
#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SCB_1_Start
****************************************************************************//**
*
*  Invokes SCB_1_Init() and SCB_1_Enable().
*  After this function call, the component is enabled and ready for operation.
*  When configuration is set to "Unconfigured SCB", the component must first be
*  initialized to operate in one of the following configurations: I2C, SPI, UART
*  or EZI2C. Otherwise this function does not enable the component.
*
* \globalvars
*  SCB_1_initVar - used to check initial configuration, modified
*  on first function call.
*
*******************************************************************************/
void SCB_1_Start(void)
{
    if (0u == SCB_1_initVar)
    {
        SCB_1_Init();
        SCB_1_initVar = 1u; /* Component was initialized */
    }

    SCB_1_Enable();
}


/*******************************************************************************
* Function Name: SCB_1_Stop
****************************************************************************//**
*
*  Disables the SCB_1 component: disable the hardware and internal 
*  interrupt. It also disables all TX interrupt sources so as not to cause an 
*  unexpected interrupt trigger because after the component is enabled, the 
*  TX FIFO is empty.
*  Refer to the function SCB_1_Enable() for the interrupt 
*  configuration details.
*  This function disables the SCB component without checking to see if 
*  communication is in progress. Before calling this function it may be 
*  necessary to check the status of communication to make sure communication 
*  is complete. If this is not done then communication could be stopped mid 
*  byte and corrupted data could result.
*
*******************************************************************************/
void SCB_1_Stop(void)
{
#if (SCB_1_SCB_IRQ_INTERNAL)
    SCB_1_DisableInt();
#endif /* (SCB_1_SCB_IRQ_INTERNAL) */

    /* Call Stop function specific to current operation mode */
    SCB_1_ScbModeStop();

    /* Disable SCB IP */
    SCB_1_CTRL_REG &= (uint32) ~SCB_1_CTRL_ENABLED;

    /* Disable all TX interrupt sources so as not to cause an unexpected
    * interrupt trigger after the component will be enabled because the 
    * TX FIFO is empty.
    * For SCB IP v0, it is critical as it does not mask-out interrupt
    * sources when it is disabled. This can cause a code lock-up in the
    * interrupt handler because TX FIFO cannot be loaded after the block
    * is disabled.
    */
    SCB_1_SetTxInterruptMode(SCB_1_NO_INTR_SOURCES);

#if (SCB_1_SCB_IRQ_INTERNAL)
    SCB_1_ClearPendingInt();
#endif /* (SCB_1_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: SCB_1_SetRxFifoLevel
****************************************************************************//**
*
*  Sets level in the RX FIFO to generate a RX level interrupt.
*  When the RX FIFO has more entries than the RX FIFO level an RX level
*  interrupt request is generated.
*
*  \param level: Level in the RX FIFO to generate RX level interrupt.
*   The range of valid level values is between 0 and RX FIFO depth - 1.
*
*******************************************************************************/
void SCB_1_SetRxFifoLevel(uint32 level)
{
    uint32 rxFifoCtrl;

    rxFifoCtrl = SCB_1_RX_FIFO_CTRL_REG;

    rxFifoCtrl &= ((uint32) ~SCB_1_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    rxFifoCtrl |= ((uint32) (SCB_1_RX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    SCB_1_RX_FIFO_CTRL_REG = rxFifoCtrl;
}


/*******************************************************************************
* Function Name: SCB_1_SetTxFifoLevel
****************************************************************************//**
*
*  Sets level in the TX FIFO to generate a TX level interrupt.
*  When the TX FIFO has less entries than the TX FIFO level an TX level
*  interrupt request is generated.
*
*  \param level: Level in the TX FIFO to generate TX level interrupt.
*   The range of valid level values is between 0 and TX FIFO depth - 1.
*
*******************************************************************************/
void SCB_1_SetTxFifoLevel(uint32 level)
{
    uint32 txFifoCtrl;

    txFifoCtrl = SCB_1_TX_FIFO_CTRL_REG;

    txFifoCtrl &= ((uint32) ~SCB_1_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK); /* Clear level mask bits */
    txFifoCtrl |= ((uint32) (SCB_1_TX_FIFO_CTRL_TRIGGER_LEVEL_MASK & level));

    SCB_1_TX_FIFO_CTRL_REG = txFifoCtrl;
}


#if (SCB_1_SCB_IRQ_INTERNAL)
    /*******************************************************************************
    * Function Name: SCB_1_SetCustomInterruptHandler
    ****************************************************************************//**
    *
    *  Registers a function to be called by the internal interrupt handler.
    *  First the function that is registered is called, then the internal interrupt
    *  handler performs any operation such as software buffer management functions
    *  before the interrupt returns.  It is the user's responsibility not to break
    *  the software buffer operations. Only one custom handler is supported, which
    *  is the function provided by the most recent call.
    *  At the initialization time no custom handler is registered.
    *
    *  \param func: Pointer to the function to register.
    *        The value NULL indicates to remove the current custom interrupt
    *        handler.
    *
    *******************************************************************************/
    void SCB_1_SetCustomInterruptHandler(void (*func)(void))
    {
    #if !defined (CY_REMOVE_SCB_1_CUSTOM_INTR_HANDLER)
        SCB_1_customIntrHandler = func; /* Register interrupt handler */
    #else
        if (NULL != func)
        {
            /* Suppress compiler warning */
        }
    #endif /* !defined (CY_REMOVE_SCB_1_CUSTOM_INTR_HANDLER) */
    }
#endif /* (SCB_1_SCB_IRQ_INTERNAL) */


/*******************************************************************************
* Function Name: SCB_1_ScbModeEnableIntr
****************************************************************************//**
*
*  Enables an interrupt for a specific mode.
*
*******************************************************************************/
static void SCB_1_ScbEnableIntr(void)
{
#if (SCB_1_SCB_IRQ_INTERNAL)
    /* Enable interrupt in NVIC */
    #if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
        if (0u != SCB_1_scbEnableIntr)
        {
            SCB_1_EnableInt();
        }

    #else
        SCB_1_EnableInt();

    #endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */
#endif /* (SCB_1_SCB_IRQ_INTERNAL) */
}


/*******************************************************************************
* Function Name: SCB_1_ScbModePostEnable
****************************************************************************//**
*
*  Calls the PostEnable function for a specific operation mode.
*
*******************************************************************************/
static void SCB_1_ScbModePostEnable(void)
{
#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
#if (!SCB_1_CY_SCBIP_V1)
    if (SCB_1_SCB_MODE_SPI_RUNTM_CFG)
    {
        SCB_1_SpiPostEnable();
    }
    else if (SCB_1_SCB_MODE_UART_RUNTM_CFG)
    {
        SCB_1_UartPostEnable();
    }
    else
    {
        /* Unknown mode: do nothing */
    }
#endif /* (!SCB_1_CY_SCBIP_V1) */

#elif (SCB_1_SCB_MODE_SPI_CONST_CFG)
    SCB_1_SpiPostEnable();

#elif (SCB_1_SCB_MODE_UART_CONST_CFG)
    SCB_1_UartPostEnable();

#else
    /* Unknown mode: do nothing */
#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */
}


/*******************************************************************************
* Function Name: SCB_1_ScbModeStop
****************************************************************************//**
*
*  Calls the Stop function for a specific operation mode.
*
*******************************************************************************/
static void SCB_1_ScbModeStop(void)
{
#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
    if (SCB_1_SCB_MODE_I2C_RUNTM_CFG)
    {
        SCB_1_I2CStop();
    }
    else if (SCB_1_SCB_MODE_EZI2C_RUNTM_CFG)
    {
        SCB_1_EzI2CStop();
    }
#if (!SCB_1_CY_SCBIP_V1)
    else if (SCB_1_SCB_MODE_SPI_RUNTM_CFG)
    {
        SCB_1_SpiStop();
    }
    else if (SCB_1_SCB_MODE_UART_RUNTM_CFG)
    {
        SCB_1_UartStop();
    }
#endif /* (!SCB_1_CY_SCBIP_V1) */
    else
    {
        /* Unknown mode: do nothing */
    }
#elif (SCB_1_SCB_MODE_I2C_CONST_CFG)
    SCB_1_I2CStop();

#elif (SCB_1_SCB_MODE_EZI2C_CONST_CFG)
    SCB_1_EzI2CStop();

#elif (SCB_1_SCB_MODE_SPI_CONST_CFG)
    SCB_1_SpiStop();

#elif (SCB_1_SCB_MODE_UART_CONST_CFG)
    SCB_1_UartStop();

#else
    /* Unknown mode: do nothing */
#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */
}


#if (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG)
    /*******************************************************************************
    * Function Name: SCB_1_SetPins
    ****************************************************************************//**
    *
    *  Sets the pins settings accordingly to the selected operation mode.
    *  Only available in the Unconfigured operation mode. The mode specific
    *  initialization function calls it.
    *  Pins configuration is set by PSoC Creator when a specific mode of operation
    *  is selected in design time.
    *
    *  \param mode:      Mode of SCB operation.
    *  \param subMode:   Sub-mode of SCB operation. It is only required for SPI and UART
    *             modes.
    *  \param uartEnableMask: enables TX or RX direction and RTS and CTS signals.
    *
    *******************************************************************************/
    void SCB_1_SetPins(uint32 mode, uint32 subMode, uint32 uartEnableMask)
    {
        uint32 pinsDm[SCB_1_SCB_PINS_NUMBER];
        uint32 i;
        
    #if (!SCB_1_CY_SCBIP_V1)
        uint32 pinsInBuf = 0u;
    #endif /* (!SCB_1_CY_SCBIP_V1) */
        
        uint32 hsiomSel[SCB_1_SCB_PINS_NUMBER] = 
        {
            SCB_1_RX_SCL_MOSI_HSIOM_SEL_GPIO,
            SCB_1_TX_SDA_MISO_HSIOM_SEL_GPIO,
            0u,
            0u,
            0u,
            0u,
            0u,
        };

    #if (SCB_1_CY_SCBIP_V1)
        /* Supress compiler warning. */
        if ((0u == subMode) || (0u == uartEnableMask))
        {
        }
    #endif /* (SCB_1_CY_SCBIP_V1) */

        /* Set default HSIOM to GPIO and Drive Mode to Analog Hi-Z */
        for (i = 0u; i < SCB_1_SCB_PINS_NUMBER; i++)
        {
            pinsDm[i] = SCB_1_PIN_DM_ALG_HIZ;
        }

        if ((SCB_1_SCB_MODE_I2C   == mode) ||
            (SCB_1_SCB_MODE_EZI2C == mode))
        {
        #if (SCB_1_RX_SCL_MOSI_PIN)
            hsiomSel[SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_RX_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_OD_LO;
        #elif (SCB_1_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX] = SCB_1_RX_WAKE_SCL_MOSI_HSIOM_SEL_I2C;
            pinsDm  [SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_OD_LO;
        #else
        #endif /* (SCB_1_RX_SCL_MOSI_PIN) */
        
        #if (SCB_1_TX_SDA_MISO_PIN)
            hsiomSel[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_TX_SDA_MISO_HSIOM_SEL_I2C;
            pinsDm  [SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_PIN_DM_OD_LO;
        #endif /* (SCB_1_TX_SDA_MISO_PIN) */
        }
    #if (!SCB_1_CY_SCBIP_V1)
        else if (SCB_1_SCB_MODE_SPI == mode)
        {
        #if (SCB_1_RX_SCL_MOSI_PIN)
            hsiomSel[SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_RX_SCL_MOSI_HSIOM_SEL_SPI;
        #elif (SCB_1_RX_WAKE_SCL_MOSI_PIN)
            hsiomSel[SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX] = SCB_1_RX_WAKE_SCL_MOSI_HSIOM_SEL_SPI;
        #else
        #endif /* (SCB_1_RX_SCL_MOSI_PIN) */
        
        #if (SCB_1_TX_SDA_MISO_PIN)
            hsiomSel[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_TX_SDA_MISO_HSIOM_SEL_SPI;
        #endif /* (SCB_1_TX_SDA_MISO_PIN) */
        
        #if (SCB_1_CTS_SCLK_PIN)
            hsiomSel[SCB_1_CTS_SCLK_PIN_INDEX] = SCB_1_CTS_SCLK_HSIOM_SEL_SPI;
        #endif /* (SCB_1_CTS_SCLK_PIN) */

            if (SCB_1_SPI_SLAVE == subMode)
            {
                /* Slave */
                pinsDm[SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
                pinsDm[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsDm[SCB_1_CTS_SCLK_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;

            #if (SCB_1_RTS_SS0_PIN)
                /* Only SS0 is valid choice for Slave */
                hsiomSel[SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm  [SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
            #endif /* (SCB_1_RTS_SS0_PIN) */

            #if (SCB_1_TX_SDA_MISO_PIN)
                /* Disable input buffer */
                 pinsInBuf |= SCB_1_TX_SDA_MISO_PIN_MASK;
            #endif /* (SCB_1_TX_SDA_MISO_PIN) */
            }
            else 
            {
                /* (Master) */
                pinsDm[SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsDm[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
                pinsDm[SCB_1_CTS_SCLK_PIN_INDEX] = SCB_1_PIN_DM_STRONG;

            #if (SCB_1_RTS_SS0_PIN)
                hsiomSel [SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_RTS_SS0_HSIOM_SEL_SPI;
                pinsDm   [SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsInBuf |= SCB_1_RTS_SS0_PIN_MASK;
            #endif /* (SCB_1_RTS_SS0_PIN) */

            #if (SCB_1_SS1_PIN)
                hsiomSel [SCB_1_SS1_PIN_INDEX] = SCB_1_SS1_HSIOM_SEL_SPI;
                pinsDm   [SCB_1_SS1_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsInBuf |= SCB_1_SS1_PIN_MASK;
            #endif /* (SCB_1_SS1_PIN) */

            #if (SCB_1_SS2_PIN)
                hsiomSel [SCB_1_SS2_PIN_INDEX] = SCB_1_SS2_HSIOM_SEL_SPI;
                pinsDm   [SCB_1_SS2_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsInBuf |= SCB_1_SS2_PIN_MASK;
            #endif /* (SCB_1_SS2_PIN) */

            #if (SCB_1_SS3_PIN)
                hsiomSel [SCB_1_SS3_PIN_INDEX] = SCB_1_SS3_HSIOM_SEL_SPI;
                pinsDm   [SCB_1_SS3_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                pinsInBuf |= SCB_1_SS3_PIN_MASK;
            #endif /* (SCB_1_SS3_PIN) */

                /* Disable input buffers */
            #if (SCB_1_RX_SCL_MOSI_PIN)
                pinsInBuf |= SCB_1_RX_SCL_MOSI_PIN_MASK;
            #elif (SCB_1_RX_WAKE_SCL_MOSI_PIN)
                pinsInBuf |= SCB_1_RX_WAKE_SCL_MOSI_PIN_MASK;
            #else
            #endif /* (SCB_1_RX_SCL_MOSI_PIN) */

            #if (SCB_1_CTS_SCLK_PIN)
                pinsInBuf |= SCB_1_CTS_SCLK_PIN_MASK;
            #endif /* (SCB_1_CTS_SCLK_PIN) */
            }
        }
        else /* UART */
        {
            if (SCB_1_UART_MODE_SMARTCARD == subMode)
            {
                /* SmartCard */
            #if (SCB_1_TX_SDA_MISO_PIN)
                hsiomSel[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_TX_SDA_MISO_HSIOM_SEL_UART;
                pinsDm  [SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_PIN_DM_OD_LO;
            #endif /* (SCB_1_TX_SDA_MISO_PIN) */
            }
            else /* Standard or IrDA */
            {
                if (0u != (SCB_1_UART_RX_PIN_ENABLE & uartEnableMask))
                {
                #if (SCB_1_RX_SCL_MOSI_PIN)
                    hsiomSel[SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_RX_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [SCB_1_RX_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
                #elif (SCB_1_RX_WAKE_SCL_MOSI_PIN)
                    hsiomSel[SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX] = SCB_1_RX_WAKE_SCL_MOSI_HSIOM_SEL_UART;
                    pinsDm  [SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
                #else
                #endif /* (SCB_1_RX_SCL_MOSI_PIN) */
                }

                if (0u != (SCB_1_UART_TX_PIN_ENABLE & uartEnableMask))
                {
                #if (SCB_1_TX_SDA_MISO_PIN)
                    hsiomSel[SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_TX_SDA_MISO_HSIOM_SEL_UART;
                    pinsDm  [SCB_1_TX_SDA_MISO_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                    
                    /* Disable input buffer */
                    pinsInBuf |= SCB_1_TX_SDA_MISO_PIN_MASK;
                #endif /* (SCB_1_TX_SDA_MISO_PIN) */
                }

            #if !(SCB_1_CY_SCBIP_V0 || SCB_1_CY_SCBIP_V1)
                if (SCB_1_UART_MODE_STD == subMode)
                {
                    if (0u != (SCB_1_UART_CTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* CTS input is multiplexed with SCLK */
                    #if (SCB_1_CTS_SCLK_PIN)
                        hsiomSel[SCB_1_CTS_SCLK_PIN_INDEX] = SCB_1_CTS_SCLK_HSIOM_SEL_UART;
                        pinsDm  [SCB_1_CTS_SCLK_PIN_INDEX] = SCB_1_PIN_DM_DIG_HIZ;
                    #endif /* (SCB_1_CTS_SCLK_PIN) */
                    }

                    if (0u != (SCB_1_UART_RTS_PIN_ENABLE & uartEnableMask))
                    {
                        /* RTS output is multiplexed with SS0 */
                    #if (SCB_1_RTS_SS0_PIN)
                        hsiomSel[SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_RTS_SS0_HSIOM_SEL_UART;
                        pinsDm  [SCB_1_RTS_SS0_PIN_INDEX] = SCB_1_PIN_DM_STRONG;
                        
                        /* Disable input buffer */
                        pinsInBuf |= SCB_1_RTS_SS0_PIN_MASK;
                    #endif /* (SCB_1_RTS_SS0_PIN) */
                    }
                }
            #endif /* !(SCB_1_CY_SCBIP_V0 || SCB_1_CY_SCBIP_V1) */
            }
        }
    #endif /* (!SCB_1_CY_SCBIP_V1) */

    /* Configure pins: set HSIOM, DM and InputBufEnable */
    /* Note: the DR register settings do not effect the pin output if HSIOM is other than GPIO */

    #if (SCB_1_RX_SCL_MOSI_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_RX_SCL_MOSI_HSIOM_REG,
                                       SCB_1_RX_SCL_MOSI_HSIOM_MASK,
                                       SCB_1_RX_SCL_MOSI_HSIOM_POS,
                                        hsiomSel[SCB_1_RX_SCL_MOSI_PIN_INDEX]);

        SCB_1_uart_rx_i2c_scl_spi_mosi_SetDriveMode((uint8) pinsDm[SCB_1_RX_SCL_MOSI_PIN_INDEX]);

        #if (!SCB_1_CY_SCBIP_V1)
            SCB_1_SET_INP_DIS(SCB_1_uart_rx_i2c_scl_spi_mosi_INP_DIS,
                                         SCB_1_uart_rx_i2c_scl_spi_mosi_MASK,
                                         (0u != (pinsInBuf & SCB_1_RX_SCL_MOSI_PIN_MASK)));
        #endif /* (!SCB_1_CY_SCBIP_V1) */
    
    #elif (SCB_1_RX_WAKE_SCL_MOSI_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_RX_WAKE_SCL_MOSI_HSIOM_REG,
                                       SCB_1_RX_WAKE_SCL_MOSI_HSIOM_MASK,
                                       SCB_1_RX_WAKE_SCL_MOSI_HSIOM_POS,
                                       hsiomSel[SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        SCB_1_uart_rx_wake_i2c_scl_spi_mosi_SetDriveMode((uint8)
                                                               pinsDm[SCB_1_RX_WAKE_SCL_MOSI_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_uart_rx_wake_i2c_scl_spi_mosi_INP_DIS,
                                     SCB_1_uart_rx_wake_i2c_scl_spi_mosi_MASK,
                                     (0u != (pinsInBuf & SCB_1_RX_WAKE_SCL_MOSI_PIN_MASK)));

         /* Set interrupt on falling edge */
        SCB_1_SET_INCFG_TYPE(SCB_1_RX_WAKE_SCL_MOSI_INTCFG_REG,
                                        SCB_1_RX_WAKE_SCL_MOSI_INTCFG_TYPE_MASK,
                                        SCB_1_RX_WAKE_SCL_MOSI_INTCFG_TYPE_POS,
                                        SCB_1_INTCFG_TYPE_FALLING_EDGE);
    #else
    #endif /* (SCB_1_RX_WAKE_SCL_MOSI_PIN) */

    #if (SCB_1_TX_SDA_MISO_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_TX_SDA_MISO_HSIOM_REG,
                                       SCB_1_TX_SDA_MISO_HSIOM_MASK,
                                       SCB_1_TX_SDA_MISO_HSIOM_POS,
                                        hsiomSel[SCB_1_TX_SDA_MISO_PIN_INDEX]);

        SCB_1_uart_tx_i2c_sda_spi_miso_SetDriveMode((uint8) pinsDm[SCB_1_TX_SDA_MISO_PIN_INDEX]);

    #if (!SCB_1_CY_SCBIP_V1)
        SCB_1_SET_INP_DIS(SCB_1_uart_tx_i2c_sda_spi_miso_INP_DIS,
                                     SCB_1_uart_tx_i2c_sda_spi_miso_MASK,
                                    (0u != (pinsInBuf & SCB_1_TX_SDA_MISO_PIN_MASK)));
    #endif /* (!SCB_1_CY_SCBIP_V1) */
    #endif /* (SCB_1_RX_SCL_MOSI_PIN) */

    #if (SCB_1_CTS_SCLK_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_CTS_SCLK_HSIOM_REG,
                                       SCB_1_CTS_SCLK_HSIOM_MASK,
                                       SCB_1_CTS_SCLK_HSIOM_POS,
                                       hsiomSel[SCB_1_CTS_SCLK_PIN_INDEX]);

        SCB_1_uart_cts_spi_sclk_SetDriveMode((uint8) pinsDm[SCB_1_CTS_SCLK_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_uart_cts_spi_sclk_INP_DIS,
                                     SCB_1_uart_cts_spi_sclk_MASK,
                                     (0u != (pinsInBuf & SCB_1_CTS_SCLK_PIN_MASK)));
    #endif /* (SCB_1_CTS_SCLK_PIN) */

    #if (SCB_1_RTS_SS0_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_RTS_SS0_HSIOM_REG,
                                       SCB_1_RTS_SS0_HSIOM_MASK,
                                       SCB_1_RTS_SS0_HSIOM_POS,
                                       hsiomSel[SCB_1_RTS_SS0_PIN_INDEX]);

        SCB_1_uart_rts_spi_ss0_SetDriveMode((uint8) pinsDm[SCB_1_RTS_SS0_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_uart_rts_spi_ss0_INP_DIS,
                                     SCB_1_uart_rts_spi_ss0_MASK,
                                     (0u != (pinsInBuf & SCB_1_RTS_SS0_PIN_MASK)));
    #endif /* (SCB_1_RTS_SS0_PIN) */

    #if (SCB_1_SS1_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_SS1_HSIOM_REG,
                                       SCB_1_SS1_HSIOM_MASK,
                                       SCB_1_SS1_HSIOM_POS,
                                       hsiomSel[SCB_1_SS1_PIN_INDEX]);

        SCB_1_spi_ss1_SetDriveMode((uint8) pinsDm[SCB_1_SS1_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_spi_ss1_INP_DIS,
                                     SCB_1_spi_ss1_MASK,
                                     (0u != (pinsInBuf & SCB_1_SS1_PIN_MASK)));
    #endif /* (SCB_1_SS1_PIN) */

    #if (SCB_1_SS2_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_SS2_HSIOM_REG,
                                       SCB_1_SS2_HSIOM_MASK,
                                       SCB_1_SS2_HSIOM_POS,
                                       hsiomSel[SCB_1_SS2_PIN_INDEX]);

        SCB_1_spi_ss2_SetDriveMode((uint8) pinsDm[SCB_1_SS2_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_spi_ss2_INP_DIS,
                                     SCB_1_spi_ss2_MASK,
                                     (0u != (pinsInBuf & SCB_1_SS2_PIN_MASK)));
    #endif /* (SCB_1_SS2_PIN) */

    #if (SCB_1_SS3_PIN)
        SCB_1_SET_HSIOM_SEL(SCB_1_SS3_HSIOM_REG,
                                       SCB_1_SS3_HSIOM_MASK,
                                       SCB_1_SS3_HSIOM_POS,
                                       hsiomSel[SCB_1_SS3_PIN_INDEX]);

        SCB_1_spi_ss3_SetDriveMode((uint8) pinsDm[SCB_1_SS3_PIN_INDEX]);

        SCB_1_SET_INP_DIS(SCB_1_spi_ss3_INP_DIS,
                                     SCB_1_spi_ss3_MASK,
                                     (0u != (pinsInBuf & SCB_1_SS3_PIN_MASK)));
    #endif /* (SCB_1_SS3_PIN) */
    }

#endif /* (SCB_1_SCB_MODE_UNCONFIG_CONST_CFG) */


#if (SCB_1_CY_SCBIP_V0 || SCB_1_CY_SCBIP_V1)
    /*******************************************************************************
    * Function Name: SCB_1_I2CSlaveNackGeneration
    ****************************************************************************//**
    *
    *  Sets command to generate NACK to the address or data.
    *
    *******************************************************************************/
    void SCB_1_I2CSlaveNackGeneration(void)
    {
        /* Check for EC_AM toggle condition: EC_AM and clock stretching for address are enabled */
        if ((0u != (SCB_1_CTRL_REG & SCB_1_CTRL_EC_AM_MODE)) &&
            (0u == (SCB_1_I2C_CTRL_REG & SCB_1_I2C_CTRL_S_NOT_READY_ADDR_NACK)))
        {
            /* Toggle EC_AM before NACK generation */
            SCB_1_CTRL_REG &= ~SCB_1_CTRL_EC_AM_MODE;
            SCB_1_CTRL_REG |=  SCB_1_CTRL_EC_AM_MODE;
        }

        SCB_1_I2C_SLAVE_CMD_REG = SCB_1_I2C_SLAVE_CMD_S_NACK;
    }
#endif /* (SCB_1_CY_SCBIP_V0 || SCB_1_CY_SCBIP_V1) */


/* [] END OF FILE */
