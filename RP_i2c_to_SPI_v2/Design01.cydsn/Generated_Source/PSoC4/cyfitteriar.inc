;
; File Name: cyfitteriar.inc
; 
; PSoC Creator  4.4
;
; Description:
; 
;
;-------------------------------------------------------------------------------
; Copyright (c) 2007-2020 Cypress Semiconductor.  All rights reserved.
; You may use this file only in accordance with the license, terms, conditions, 
; disclaimers, and limitations in the end user license agreement accompanying 
; the software package with which this file was provided.
;-------------------------------------------------------------------------------

#ifndef INCLUDED_CYFITTERIAR_INC
#define INCLUDED_CYFITTERIAR_INC
    INCLUDE cydeviceiar_trm.inc

/* I2C */
I2C_SCB__CTRL EQU CYREG_SCB0_CTRL
I2C_SCB__EZ_DATA0 EQU CYREG_SCB0_EZ_DATA0
I2C_SCB__EZ_DATA1 EQU CYREG_SCB0_EZ_DATA1
I2C_SCB__EZ_DATA10 EQU CYREG_SCB0_EZ_DATA10
I2C_SCB__EZ_DATA11 EQU CYREG_SCB0_EZ_DATA11
I2C_SCB__EZ_DATA12 EQU CYREG_SCB0_EZ_DATA12
I2C_SCB__EZ_DATA13 EQU CYREG_SCB0_EZ_DATA13
I2C_SCB__EZ_DATA14 EQU CYREG_SCB0_EZ_DATA14
I2C_SCB__EZ_DATA15 EQU CYREG_SCB0_EZ_DATA15
I2C_SCB__EZ_DATA16 EQU CYREG_SCB0_EZ_DATA16
I2C_SCB__EZ_DATA17 EQU CYREG_SCB0_EZ_DATA17
I2C_SCB__EZ_DATA18 EQU CYREG_SCB0_EZ_DATA18
I2C_SCB__EZ_DATA19 EQU CYREG_SCB0_EZ_DATA19
I2C_SCB__EZ_DATA2 EQU CYREG_SCB0_EZ_DATA2
I2C_SCB__EZ_DATA20 EQU CYREG_SCB0_EZ_DATA20
I2C_SCB__EZ_DATA21 EQU CYREG_SCB0_EZ_DATA21
I2C_SCB__EZ_DATA22 EQU CYREG_SCB0_EZ_DATA22
I2C_SCB__EZ_DATA23 EQU CYREG_SCB0_EZ_DATA23
I2C_SCB__EZ_DATA24 EQU CYREG_SCB0_EZ_DATA24
I2C_SCB__EZ_DATA25 EQU CYREG_SCB0_EZ_DATA25
I2C_SCB__EZ_DATA26 EQU CYREG_SCB0_EZ_DATA26
I2C_SCB__EZ_DATA27 EQU CYREG_SCB0_EZ_DATA27
I2C_SCB__EZ_DATA28 EQU CYREG_SCB0_EZ_DATA28
I2C_SCB__EZ_DATA29 EQU CYREG_SCB0_EZ_DATA29
I2C_SCB__EZ_DATA3 EQU CYREG_SCB0_EZ_DATA3
I2C_SCB__EZ_DATA30 EQU CYREG_SCB0_EZ_DATA30
I2C_SCB__EZ_DATA31 EQU CYREG_SCB0_EZ_DATA31
I2C_SCB__EZ_DATA4 EQU CYREG_SCB0_EZ_DATA4
I2C_SCB__EZ_DATA5 EQU CYREG_SCB0_EZ_DATA5
I2C_SCB__EZ_DATA6 EQU CYREG_SCB0_EZ_DATA6
I2C_SCB__EZ_DATA7 EQU CYREG_SCB0_EZ_DATA7
I2C_SCB__EZ_DATA8 EQU CYREG_SCB0_EZ_DATA8
I2C_SCB__EZ_DATA9 EQU CYREG_SCB0_EZ_DATA9
I2C_SCB__I2C_CFG EQU CYREG_SCB0_I2C_CFG
I2C_SCB__I2C_CTRL EQU CYREG_SCB0_I2C_CTRL
I2C_SCB__I2C_M_CMD EQU CYREG_SCB0_I2C_M_CMD
I2C_SCB__I2C_S_CMD EQU CYREG_SCB0_I2C_S_CMD
I2C_SCB__I2C_STATUS EQU CYREG_SCB0_I2C_STATUS
I2C_SCB__INTR_CAUSE EQU CYREG_SCB0_INTR_CAUSE
I2C_SCB__INTR_I2C_EC EQU CYREG_SCB0_INTR_I2C_EC
I2C_SCB__INTR_I2C_EC_MASK EQU CYREG_SCB0_INTR_I2C_EC_MASK
I2C_SCB__INTR_I2C_EC_MASKED EQU CYREG_SCB0_INTR_I2C_EC_MASKED
I2C_SCB__INTR_M EQU CYREG_SCB0_INTR_M
I2C_SCB__INTR_M_MASK EQU CYREG_SCB0_INTR_M_MASK
I2C_SCB__INTR_M_MASKED EQU CYREG_SCB0_INTR_M_MASKED
I2C_SCB__INTR_M_SET EQU CYREG_SCB0_INTR_M_SET
I2C_SCB__INTR_RX EQU CYREG_SCB0_INTR_RX
I2C_SCB__INTR_RX_MASK EQU CYREG_SCB0_INTR_RX_MASK
I2C_SCB__INTR_RX_MASKED EQU CYREG_SCB0_INTR_RX_MASKED
I2C_SCB__INTR_RX_SET EQU CYREG_SCB0_INTR_RX_SET
I2C_SCB__INTR_S EQU CYREG_SCB0_INTR_S
I2C_SCB__INTR_S_MASK EQU CYREG_SCB0_INTR_S_MASK
I2C_SCB__INTR_S_MASKED EQU CYREG_SCB0_INTR_S_MASKED
I2C_SCB__INTR_S_SET EQU CYREG_SCB0_INTR_S_SET
I2C_SCB__INTR_SPI_EC EQU CYREG_SCB0_INTR_SPI_EC
I2C_SCB__INTR_SPI_EC_MASK EQU CYREG_SCB0_INTR_SPI_EC_MASK
I2C_SCB__INTR_SPI_EC_MASKED EQU CYREG_SCB0_INTR_SPI_EC_MASKED
I2C_SCB__INTR_TX EQU CYREG_SCB0_INTR_TX
I2C_SCB__INTR_TX_MASK EQU CYREG_SCB0_INTR_TX_MASK
I2C_SCB__INTR_TX_MASKED EQU CYREG_SCB0_INTR_TX_MASKED
I2C_SCB__INTR_TX_SET EQU CYREG_SCB0_INTR_TX_SET
I2C_SCB__RX_CTRL EQU CYREG_SCB0_RX_CTRL
I2C_SCB__RX_FIFO_CTRL EQU CYREG_SCB0_RX_FIFO_CTRL
I2C_SCB__RX_FIFO_RD EQU CYREG_SCB0_RX_FIFO_RD
I2C_SCB__RX_FIFO_RD_SILENT EQU CYREG_SCB0_RX_FIFO_RD_SILENT
I2C_SCB__RX_FIFO_STATUS EQU CYREG_SCB0_RX_FIFO_STATUS
I2C_SCB__RX_MATCH EQU CYREG_SCB0_RX_MATCH
I2C_SCB__SPI_CTRL EQU CYREG_SCB0_SPI_CTRL
I2C_SCB__SPI_STATUS EQU CYREG_SCB0_SPI_STATUS
I2C_SCB__SS0_POSISTION EQU 0
I2C_SCB__SS1_POSISTION EQU 1
I2C_SCB__SS2_POSISTION EQU 2
I2C_SCB__SS3_POSISTION EQU 3
I2C_SCB__STATUS EQU CYREG_SCB0_STATUS
I2C_SCB__TX_CTRL EQU CYREG_SCB0_TX_CTRL
I2C_SCB__TX_FIFO_CTRL EQU CYREG_SCB0_TX_FIFO_CTRL
I2C_SCB__TX_FIFO_STATUS EQU CYREG_SCB0_TX_FIFO_STATUS
I2C_SCB__TX_FIFO_WR EQU CYREG_SCB0_TX_FIFO_WR
I2C_SCB__UART_CTRL EQU CYREG_SCB0_UART_CTRL
I2C_SCB__UART_FLOW_CTRL EQU CYREG_SCB0_UART_FLOW_CTRL
I2C_SCB__UART_RX_CTRL EQU CYREG_SCB0_UART_RX_CTRL
I2C_SCB__UART_RX_STATUS EQU CYREG_SCB0_UART_RX_STATUS
I2C_SCB__UART_TX_CTRL EQU CYREG_SCB0_UART_TX_CTRL
I2C_SCB_IRQ__INTC_CLR_EN_REG EQU CYREG_CM0P_ICER
I2C_SCB_IRQ__INTC_CLR_PD_REG EQU CYREG_CM0P_ICPR
I2C_SCB_IRQ__INTC_MASK EQU 0x80
I2C_SCB_IRQ__INTC_NUMBER EQU 7
I2C_SCB_IRQ__INTC_PRIOR_MASK EQU 0xC0000000
I2C_SCB_IRQ__INTC_PRIOR_NUM EQU 3
I2C_SCB_IRQ__INTC_PRIOR_REG EQU CYREG_CM0P_IPR1
I2C_SCB_IRQ__INTC_SET_EN_REG EQU CYREG_CM0P_ISER
I2C_SCB_IRQ__INTC_SET_PD_REG EQU CYREG_CM0P_ISPR
I2C_SCBCLK__CTRL_REGISTER EQU CYREG_PERI_PCLK_CTL0
I2C_SCBCLK__DIV_ID EQU 0x00000041
I2C_SCBCLK__DIV_REGISTER EQU CYREG_PERI_DIV_16_CTL1
I2C_SCBCLK__PA_DIV_ID EQU 0x000000FF
I2C_scl__0__DR EQU CYREG_GPIO_PRT1_DR
I2C_scl__0__DR_CLR EQU CYREG_GPIO_PRT1_DR_CLR
I2C_scl__0__DR_INV EQU CYREG_GPIO_PRT1_DR_INV
I2C_scl__0__DR_SET EQU CYREG_GPIO_PRT1_DR_SET
I2C_scl__0__HSIOM EQU CYREG_HSIOM_PORT_SEL1
I2C_scl__0__HSIOM_GPIO EQU 0
I2C_scl__0__HSIOM_I2C EQU 14
I2C_scl__0__HSIOM_I2C_SCL EQU 14
I2C_scl__0__HSIOM_MASK EQU 0x0000000F
I2C_scl__0__HSIOM_SHIFT EQU 0
I2C_scl__0__HSIOM_SPI EQU 15
I2C_scl__0__HSIOM_SPI_MOSI EQU 15
I2C_scl__0__HSIOM_UART EQU 9
I2C_scl__0__HSIOM_UART_RX EQU 9
I2C_scl__0__INTCFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_scl__0__INTR EQU CYREG_GPIO_PRT1_INTR
I2C_scl__0__INTR_CFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_scl__0__INTSTAT EQU CYREG_GPIO_PRT1_INTR
I2C_scl__0__MASK EQU 0x01
I2C_scl__0__PC EQU CYREG_GPIO_PRT1_PC
I2C_scl__0__PC2 EQU CYREG_GPIO_PRT1_PC2
I2C_scl__0__PORT EQU 1
I2C_scl__0__PS EQU CYREG_GPIO_PRT1_PS
I2C_scl__0__SHIFT EQU 0
I2C_scl__DR EQU CYREG_GPIO_PRT1_DR
I2C_scl__DR_CLR EQU CYREG_GPIO_PRT1_DR_CLR
I2C_scl__DR_INV EQU CYREG_GPIO_PRT1_DR_INV
I2C_scl__DR_SET EQU CYREG_GPIO_PRT1_DR_SET
I2C_scl__INTCFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_scl__INTR EQU CYREG_GPIO_PRT1_INTR
I2C_scl__INTR_CFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_scl__INTSTAT EQU CYREG_GPIO_PRT1_INTR
I2C_scl__MASK EQU 0x01
I2C_scl__PC EQU CYREG_GPIO_PRT1_PC
I2C_scl__PC2 EQU CYREG_GPIO_PRT1_PC2
I2C_scl__PORT EQU 1
I2C_scl__PS EQU CYREG_GPIO_PRT1_PS
I2C_scl__SHIFT EQU 0
I2C_sda__0__DR EQU CYREG_GPIO_PRT1_DR
I2C_sda__0__DR_CLR EQU CYREG_GPIO_PRT1_DR_CLR
I2C_sda__0__DR_INV EQU CYREG_GPIO_PRT1_DR_INV
I2C_sda__0__DR_SET EQU CYREG_GPIO_PRT1_DR_SET
I2C_sda__0__HSIOM EQU CYREG_HSIOM_PORT_SEL1
I2C_sda__0__HSIOM_GPIO EQU 0
I2C_sda__0__HSIOM_I2C EQU 14
I2C_sda__0__HSIOM_I2C_SDA EQU 14
I2C_sda__0__HSIOM_MASK EQU 0x000000F0
I2C_sda__0__HSIOM_SHIFT EQU 4
I2C_sda__0__HSIOM_SPI EQU 15
I2C_sda__0__HSIOM_SPI_MISO EQU 15
I2C_sda__0__HSIOM_UART EQU 9
I2C_sda__0__HSIOM_UART_TX EQU 9
I2C_sda__0__INTCFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_sda__0__INTR EQU CYREG_GPIO_PRT1_INTR
I2C_sda__0__INTR_CFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_sda__0__INTSTAT EQU CYREG_GPIO_PRT1_INTR
I2C_sda__0__MASK EQU 0x02
I2C_sda__0__PC EQU CYREG_GPIO_PRT1_PC
I2C_sda__0__PC2 EQU CYREG_GPIO_PRT1_PC2
I2C_sda__0__PORT EQU 1
I2C_sda__0__PS EQU CYREG_GPIO_PRT1_PS
I2C_sda__0__SHIFT EQU 1
I2C_sda__DR EQU CYREG_GPIO_PRT1_DR
I2C_sda__DR_CLR EQU CYREG_GPIO_PRT1_DR_CLR
I2C_sda__DR_INV EQU CYREG_GPIO_PRT1_DR_INV
I2C_sda__DR_SET EQU CYREG_GPIO_PRT1_DR_SET
I2C_sda__INTCFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_sda__INTR EQU CYREG_GPIO_PRT1_INTR
I2C_sda__INTR_CFG EQU CYREG_GPIO_PRT1_INTR_CFG
I2C_sda__INTSTAT EQU CYREG_GPIO_PRT1_INTR
I2C_sda__MASK EQU 0x02
I2C_sda__PC EQU CYREG_GPIO_PRT1_PC
I2C_sda__PC2 EQU CYREG_GPIO_PRT1_PC2
I2C_sda__PORT EQU 1
I2C_sda__PS EQU CYREG_GPIO_PRT1_PS
I2C_sda__SHIFT EQU 1

/* SPI */
SPI_mosi_m__0__DR EQU CYREG_GPIO_PRT0_DR
SPI_mosi_m__0__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_mosi_m__0__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_mosi_m__0__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_mosi_m__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
SPI_mosi_m__0__HSIOM_GPIO EQU 0
SPI_mosi_m__0__HSIOM_I2C EQU 14
SPI_mosi_m__0__HSIOM_I2C_SCL EQU 14
SPI_mosi_m__0__HSIOM_MASK EQU 0x000F0000
SPI_mosi_m__0__HSIOM_SHIFT EQU 16
SPI_mosi_m__0__HSIOM_SPI EQU 15
SPI_mosi_m__0__HSIOM_SPI_MOSI EQU 15
SPI_mosi_m__0__HSIOM_UART EQU 9
SPI_mosi_m__0__HSIOM_UART_RX EQU 9
SPI_mosi_m__0__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_mosi_m__0__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_mosi_m__0__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_mosi_m__0__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_mosi_m__0__MASK EQU 0x10
SPI_mosi_m__0__PC EQU CYREG_GPIO_PRT0_PC
SPI_mosi_m__0__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_mosi_m__0__PORT EQU 0
SPI_mosi_m__0__PS EQU CYREG_GPIO_PRT0_PS
SPI_mosi_m__0__SHIFT EQU 4
SPI_mosi_m__DR EQU CYREG_GPIO_PRT0_DR
SPI_mosi_m__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_mosi_m__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_mosi_m__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_mosi_m__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_mosi_m__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_mosi_m__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_mosi_m__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_mosi_m__MASK EQU 0x10
SPI_mosi_m__PC EQU CYREG_GPIO_PRT0_PC
SPI_mosi_m__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_mosi_m__PORT EQU 0
SPI_mosi_m__PS EQU CYREG_GPIO_PRT0_PS
SPI_mosi_m__SHIFT EQU 4
SPI_SCB__CTRL EQU CYREG_SCB1_CTRL
SPI_SCB__EZ_DATA0 EQU CYREG_SCB1_EZ_DATA0
SPI_SCB__EZ_DATA1 EQU CYREG_SCB1_EZ_DATA1
SPI_SCB__EZ_DATA10 EQU CYREG_SCB1_EZ_DATA10
SPI_SCB__EZ_DATA11 EQU CYREG_SCB1_EZ_DATA11
SPI_SCB__EZ_DATA12 EQU CYREG_SCB1_EZ_DATA12
SPI_SCB__EZ_DATA13 EQU CYREG_SCB1_EZ_DATA13
SPI_SCB__EZ_DATA14 EQU CYREG_SCB1_EZ_DATA14
SPI_SCB__EZ_DATA15 EQU CYREG_SCB1_EZ_DATA15
SPI_SCB__EZ_DATA16 EQU CYREG_SCB1_EZ_DATA16
SPI_SCB__EZ_DATA17 EQU CYREG_SCB1_EZ_DATA17
SPI_SCB__EZ_DATA18 EQU CYREG_SCB1_EZ_DATA18
SPI_SCB__EZ_DATA19 EQU CYREG_SCB1_EZ_DATA19
SPI_SCB__EZ_DATA2 EQU CYREG_SCB1_EZ_DATA2
SPI_SCB__EZ_DATA20 EQU CYREG_SCB1_EZ_DATA20
SPI_SCB__EZ_DATA21 EQU CYREG_SCB1_EZ_DATA21
SPI_SCB__EZ_DATA22 EQU CYREG_SCB1_EZ_DATA22
SPI_SCB__EZ_DATA23 EQU CYREG_SCB1_EZ_DATA23
SPI_SCB__EZ_DATA24 EQU CYREG_SCB1_EZ_DATA24
SPI_SCB__EZ_DATA25 EQU CYREG_SCB1_EZ_DATA25
SPI_SCB__EZ_DATA26 EQU CYREG_SCB1_EZ_DATA26
SPI_SCB__EZ_DATA27 EQU CYREG_SCB1_EZ_DATA27
SPI_SCB__EZ_DATA28 EQU CYREG_SCB1_EZ_DATA28
SPI_SCB__EZ_DATA29 EQU CYREG_SCB1_EZ_DATA29
SPI_SCB__EZ_DATA3 EQU CYREG_SCB1_EZ_DATA3
SPI_SCB__EZ_DATA30 EQU CYREG_SCB1_EZ_DATA30
SPI_SCB__EZ_DATA31 EQU CYREG_SCB1_EZ_DATA31
SPI_SCB__EZ_DATA4 EQU CYREG_SCB1_EZ_DATA4
SPI_SCB__EZ_DATA5 EQU CYREG_SCB1_EZ_DATA5
SPI_SCB__EZ_DATA6 EQU CYREG_SCB1_EZ_DATA6
SPI_SCB__EZ_DATA7 EQU CYREG_SCB1_EZ_DATA7
SPI_SCB__EZ_DATA8 EQU CYREG_SCB1_EZ_DATA8
SPI_SCB__EZ_DATA9 EQU CYREG_SCB1_EZ_DATA9
SPI_SCB__I2C_CFG EQU CYREG_SCB1_I2C_CFG
SPI_SCB__I2C_CTRL EQU CYREG_SCB1_I2C_CTRL
SPI_SCB__I2C_M_CMD EQU CYREG_SCB1_I2C_M_CMD
SPI_SCB__I2C_S_CMD EQU CYREG_SCB1_I2C_S_CMD
SPI_SCB__I2C_STATUS EQU CYREG_SCB1_I2C_STATUS
SPI_SCB__INTR_CAUSE EQU CYREG_SCB1_INTR_CAUSE
SPI_SCB__INTR_I2C_EC EQU CYREG_SCB1_INTR_I2C_EC
SPI_SCB__INTR_I2C_EC_MASK EQU CYREG_SCB1_INTR_I2C_EC_MASK
SPI_SCB__INTR_I2C_EC_MASKED EQU CYREG_SCB1_INTR_I2C_EC_MASKED
SPI_SCB__INTR_M EQU CYREG_SCB1_INTR_M
SPI_SCB__INTR_M_MASK EQU CYREG_SCB1_INTR_M_MASK
SPI_SCB__INTR_M_MASKED EQU CYREG_SCB1_INTR_M_MASKED
SPI_SCB__INTR_M_SET EQU CYREG_SCB1_INTR_M_SET
SPI_SCB__INTR_RX EQU CYREG_SCB1_INTR_RX
SPI_SCB__INTR_RX_MASK EQU CYREG_SCB1_INTR_RX_MASK
SPI_SCB__INTR_RX_MASKED EQU CYREG_SCB1_INTR_RX_MASKED
SPI_SCB__INTR_RX_SET EQU CYREG_SCB1_INTR_RX_SET
SPI_SCB__INTR_S EQU CYREG_SCB1_INTR_S
SPI_SCB__INTR_S_MASK EQU CYREG_SCB1_INTR_S_MASK
SPI_SCB__INTR_S_MASKED EQU CYREG_SCB1_INTR_S_MASKED
SPI_SCB__INTR_S_SET EQU CYREG_SCB1_INTR_S_SET
SPI_SCB__INTR_SPI_EC EQU CYREG_SCB1_INTR_SPI_EC
SPI_SCB__INTR_SPI_EC_MASK EQU CYREG_SCB1_INTR_SPI_EC_MASK
SPI_SCB__INTR_SPI_EC_MASKED EQU CYREG_SCB1_INTR_SPI_EC_MASKED
SPI_SCB__INTR_TX EQU CYREG_SCB1_INTR_TX
SPI_SCB__INTR_TX_MASK EQU CYREG_SCB1_INTR_TX_MASK
SPI_SCB__INTR_TX_MASKED EQU CYREG_SCB1_INTR_TX_MASKED
SPI_SCB__INTR_TX_SET EQU CYREG_SCB1_INTR_TX_SET
SPI_SCB__RX_CTRL EQU CYREG_SCB1_RX_CTRL
SPI_SCB__RX_FIFO_CTRL EQU CYREG_SCB1_RX_FIFO_CTRL
SPI_SCB__RX_FIFO_RD EQU CYREG_SCB1_RX_FIFO_RD
SPI_SCB__RX_FIFO_RD_SILENT EQU CYREG_SCB1_RX_FIFO_RD_SILENT
SPI_SCB__RX_FIFO_STATUS EQU CYREG_SCB1_RX_FIFO_STATUS
SPI_SCB__RX_MATCH EQU CYREG_SCB1_RX_MATCH
SPI_SCB__SPI_CTRL EQU CYREG_SCB1_SPI_CTRL
SPI_SCB__SPI_STATUS EQU CYREG_SCB1_SPI_STATUS
SPI_SCB__SS0_POSISTION EQU 1
SPI_SCB__SS1_POSISTION EQU 0
SPI_SCB__SS2_POSISTION EQU 2
SPI_SCB__SS3_POSISTION EQU 3
SPI_SCB__STATUS EQU CYREG_SCB1_STATUS
SPI_SCB__TX_CTRL EQU CYREG_SCB1_TX_CTRL
SPI_SCB__TX_FIFO_CTRL EQU CYREG_SCB1_TX_FIFO_CTRL
SPI_SCB__TX_FIFO_STATUS EQU CYREG_SCB1_TX_FIFO_STATUS
SPI_SCB__TX_FIFO_WR EQU CYREG_SCB1_TX_FIFO_WR
SPI_SCB__UART_CTRL EQU CYREG_SCB1_UART_CTRL
SPI_SCB__UART_FLOW_CTRL EQU CYREG_SCB1_UART_FLOW_CTRL
SPI_SCB__UART_RX_CTRL EQU CYREG_SCB1_UART_RX_CTRL
SPI_SCB__UART_RX_STATUS EQU CYREG_SCB1_UART_RX_STATUS
SPI_SCB__UART_TX_CTRL EQU CYREG_SCB1_UART_TX_CTRL
SPI_SCB_IRQ__INTC_CLR_EN_REG EQU CYREG_CM0P_ICER
SPI_SCB_IRQ__INTC_CLR_PD_REG EQU CYREG_CM0P_ICPR
SPI_SCB_IRQ__INTC_MASK EQU 0x100
SPI_SCB_IRQ__INTC_NUMBER EQU 8
SPI_SCB_IRQ__INTC_PRIOR_MASK EQU 0xC0
SPI_SCB_IRQ__INTC_PRIOR_NUM EQU 3
SPI_SCB_IRQ__INTC_PRIOR_REG EQU CYREG_CM0P_IPR2
SPI_SCB_IRQ__INTC_SET_EN_REG EQU CYREG_CM0P_ISER
SPI_SCB_IRQ__INTC_SET_PD_REG EQU CYREG_CM0P_ISPR
SPI_SCBCLK__CTRL_REGISTER EQU CYREG_PERI_PCLK_CTL1
SPI_SCBCLK__DIV_ID EQU 0x00000040
SPI_SCBCLK__DIV_REGISTER EQU CYREG_PERI_DIV_16_CTL0
SPI_SCBCLK__PA_DIV_ID EQU 0x000000FF
SPI_sclk_m__0__DR EQU CYREG_GPIO_PRT0_DR
SPI_sclk_m__0__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_sclk_m__0__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_sclk_m__0__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_sclk_m__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
SPI_sclk_m__0__HSIOM_GPIO EQU 0
SPI_sclk_m__0__HSIOM_MASK EQU 0x0F000000
SPI_sclk_m__0__HSIOM_SHIFT EQU 24
SPI_sclk_m__0__HSIOM_SPI EQU 15
SPI_sclk_m__0__HSIOM_SPI_CLK EQU 15
SPI_sclk_m__0__HSIOM_UART EQU 9
SPI_sclk_m__0__HSIOM_UART_CTS EQU 9
SPI_sclk_m__0__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_sclk_m__0__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_sclk_m__0__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_sclk_m__0__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_sclk_m__0__MASK EQU 0x40
SPI_sclk_m__0__PC EQU CYREG_GPIO_PRT0_PC
SPI_sclk_m__0__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_sclk_m__0__PORT EQU 0
SPI_sclk_m__0__PS EQU CYREG_GPIO_PRT0_PS
SPI_sclk_m__0__SHIFT EQU 6
SPI_sclk_m__DR EQU CYREG_GPIO_PRT0_DR
SPI_sclk_m__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_sclk_m__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_sclk_m__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_sclk_m__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_sclk_m__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_sclk_m__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_sclk_m__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_sclk_m__MASK EQU 0x40
SPI_sclk_m__PC EQU CYREG_GPIO_PRT0_PC
SPI_sclk_m__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_sclk_m__PORT EQU 0
SPI_sclk_m__PS EQU CYREG_GPIO_PRT0_PS
SPI_sclk_m__SHIFT EQU 6
SPI_ss0_m__0__DR EQU CYREG_GPIO_PRT2_DR
SPI_ss0_m__0__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
SPI_ss0_m__0__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
SPI_ss0_m__0__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
SPI_ss0_m__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
SPI_ss0_m__0__HSIOM_GPIO EQU 0
SPI_ss0_m__0__HSIOM_MASK EQU 0x000F0000
SPI_ss0_m__0__HSIOM_SHIFT EQU 16
SPI_ss0_m__0__HSIOM_SPI EQU 15
SPI_ss0_m__0__HSIOM_SPI_SELECT1 EQU 15
SPI_ss0_m__0__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss0_m__0__INTR EQU CYREG_GPIO_PRT2_INTR
SPI_ss0_m__0__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss0_m__0__INTSTAT EQU CYREG_GPIO_PRT2_INTR
SPI_ss0_m__0__MASK EQU 0x10
SPI_ss0_m__0__PC EQU CYREG_GPIO_PRT2_PC
SPI_ss0_m__0__PC2 EQU CYREG_GPIO_PRT2_PC2
SPI_ss0_m__0__PORT EQU 2
SPI_ss0_m__0__PS EQU CYREG_GPIO_PRT2_PS
SPI_ss0_m__0__SHIFT EQU 4
SPI_ss0_m__DR EQU CYREG_GPIO_PRT2_DR
SPI_ss0_m__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
SPI_ss0_m__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
SPI_ss0_m__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
SPI_ss0_m__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss0_m__INTR EQU CYREG_GPIO_PRT2_INTR
SPI_ss0_m__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss0_m__INTSTAT EQU CYREG_GPIO_PRT2_INTR
SPI_ss0_m__MASK EQU 0x10
SPI_ss0_m__PC EQU CYREG_GPIO_PRT2_PC
SPI_ss0_m__PC2 EQU CYREG_GPIO_PRT2_PC2
SPI_ss0_m__PORT EQU 2
SPI_ss0_m__PS EQU CYREG_GPIO_PRT2_PS
SPI_ss0_m__SHIFT EQU 4
SPI_ss1_m__0__DR EQU CYREG_GPIO_PRT0_DR
SPI_ss1_m__0__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_ss1_m__0__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_ss1_m__0__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_ss1_m__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
SPI_ss1_m__0__HSIOM_GPIO EQU 0
SPI_ss1_m__0__HSIOM_MASK EQU 0xF0000000
SPI_ss1_m__0__HSIOM_SHIFT EQU 28
SPI_ss1_m__0__HSIOM_SPI EQU 15
SPI_ss1_m__0__HSIOM_SPI_SELECT0 EQU 15
SPI_ss1_m__0__HSIOM_UART EQU 9
SPI_ss1_m__0__HSIOM_UART_RTS EQU 9
SPI_ss1_m__0__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_ss1_m__0__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_ss1_m__0__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_ss1_m__0__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_ss1_m__0__MASK EQU 0x80
SPI_ss1_m__0__PC EQU CYREG_GPIO_PRT0_PC
SPI_ss1_m__0__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_ss1_m__0__PORT EQU 0
SPI_ss1_m__0__PS EQU CYREG_GPIO_PRT0_PS
SPI_ss1_m__0__SHIFT EQU 7
SPI_ss1_m__DR EQU CYREG_GPIO_PRT0_DR
SPI_ss1_m__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
SPI_ss1_m__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
SPI_ss1_m__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
SPI_ss1_m__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_ss1_m__INTR EQU CYREG_GPIO_PRT0_INTR
SPI_ss1_m__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
SPI_ss1_m__INTSTAT EQU CYREG_GPIO_PRT0_INTR
SPI_ss1_m__MASK EQU 0x80
SPI_ss1_m__PC EQU CYREG_GPIO_PRT0_PC
SPI_ss1_m__PC2 EQU CYREG_GPIO_PRT0_PC2
SPI_ss1_m__PORT EQU 0
SPI_ss1_m__PS EQU CYREG_GPIO_PRT0_PS
SPI_ss1_m__SHIFT EQU 7
SPI_ss2_m__0__DR EQU CYREG_GPIO_PRT2_DR
SPI_ss2_m__0__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
SPI_ss2_m__0__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
SPI_ss2_m__0__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
SPI_ss2_m__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
SPI_ss2_m__0__HSIOM_GPIO EQU 0
SPI_ss2_m__0__HSIOM_MASK EQU 0x00F00000
SPI_ss2_m__0__HSIOM_SHIFT EQU 20
SPI_ss2_m__0__HSIOM_SPI EQU 15
SPI_ss2_m__0__HSIOM_SPI_SELECT2 EQU 15
SPI_ss2_m__0__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss2_m__0__INTR EQU CYREG_GPIO_PRT2_INTR
SPI_ss2_m__0__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss2_m__0__INTSTAT EQU CYREG_GPIO_PRT2_INTR
SPI_ss2_m__0__MASK EQU 0x20
SPI_ss2_m__0__PC EQU CYREG_GPIO_PRT2_PC
SPI_ss2_m__0__PC2 EQU CYREG_GPIO_PRT2_PC2
SPI_ss2_m__0__PORT EQU 2
SPI_ss2_m__0__PS EQU CYREG_GPIO_PRT2_PS
SPI_ss2_m__0__SHIFT EQU 5
SPI_ss2_m__DR EQU CYREG_GPIO_PRT2_DR
SPI_ss2_m__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
SPI_ss2_m__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
SPI_ss2_m__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
SPI_ss2_m__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss2_m__INTR EQU CYREG_GPIO_PRT2_INTR
SPI_ss2_m__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
SPI_ss2_m__INTSTAT EQU CYREG_GPIO_PRT2_INTR
SPI_ss2_m__MASK EQU 0x20
SPI_ss2_m__PC EQU CYREG_GPIO_PRT2_PC
SPI_ss2_m__PC2 EQU CYREG_GPIO_PRT2_PC2
SPI_ss2_m__PORT EQU 2
SPI_ss2_m__PS EQU CYREG_GPIO_PRT2_PS
SPI_ss2_m__SHIFT EQU 5

/* UART */
UART_SCB__CTRL EQU CYREG_SCB2_CTRL
UART_SCB__EZ_DATA0 EQU CYREG_SCB2_EZ_DATA0
UART_SCB__EZ_DATA1 EQU CYREG_SCB2_EZ_DATA1
UART_SCB__EZ_DATA10 EQU CYREG_SCB2_EZ_DATA10
UART_SCB__EZ_DATA11 EQU CYREG_SCB2_EZ_DATA11
UART_SCB__EZ_DATA12 EQU CYREG_SCB2_EZ_DATA12
UART_SCB__EZ_DATA13 EQU CYREG_SCB2_EZ_DATA13
UART_SCB__EZ_DATA14 EQU CYREG_SCB2_EZ_DATA14
UART_SCB__EZ_DATA15 EQU CYREG_SCB2_EZ_DATA15
UART_SCB__EZ_DATA16 EQU CYREG_SCB2_EZ_DATA16
UART_SCB__EZ_DATA17 EQU CYREG_SCB2_EZ_DATA17
UART_SCB__EZ_DATA18 EQU CYREG_SCB2_EZ_DATA18
UART_SCB__EZ_DATA19 EQU CYREG_SCB2_EZ_DATA19
UART_SCB__EZ_DATA2 EQU CYREG_SCB2_EZ_DATA2
UART_SCB__EZ_DATA20 EQU CYREG_SCB2_EZ_DATA20
UART_SCB__EZ_DATA21 EQU CYREG_SCB2_EZ_DATA21
UART_SCB__EZ_DATA22 EQU CYREG_SCB2_EZ_DATA22
UART_SCB__EZ_DATA23 EQU CYREG_SCB2_EZ_DATA23
UART_SCB__EZ_DATA24 EQU CYREG_SCB2_EZ_DATA24
UART_SCB__EZ_DATA25 EQU CYREG_SCB2_EZ_DATA25
UART_SCB__EZ_DATA26 EQU CYREG_SCB2_EZ_DATA26
UART_SCB__EZ_DATA27 EQU CYREG_SCB2_EZ_DATA27
UART_SCB__EZ_DATA28 EQU CYREG_SCB2_EZ_DATA28
UART_SCB__EZ_DATA29 EQU CYREG_SCB2_EZ_DATA29
UART_SCB__EZ_DATA3 EQU CYREG_SCB2_EZ_DATA3
UART_SCB__EZ_DATA30 EQU CYREG_SCB2_EZ_DATA30
UART_SCB__EZ_DATA31 EQU CYREG_SCB2_EZ_DATA31
UART_SCB__EZ_DATA4 EQU CYREG_SCB2_EZ_DATA4
UART_SCB__EZ_DATA5 EQU CYREG_SCB2_EZ_DATA5
UART_SCB__EZ_DATA6 EQU CYREG_SCB2_EZ_DATA6
UART_SCB__EZ_DATA7 EQU CYREG_SCB2_EZ_DATA7
UART_SCB__EZ_DATA8 EQU CYREG_SCB2_EZ_DATA8
UART_SCB__EZ_DATA9 EQU CYREG_SCB2_EZ_DATA9
UART_SCB__I2C_CFG EQU CYREG_SCB2_I2C_CFG
UART_SCB__I2C_CTRL EQU CYREG_SCB2_I2C_CTRL
UART_SCB__I2C_M_CMD EQU CYREG_SCB2_I2C_M_CMD
UART_SCB__I2C_S_CMD EQU CYREG_SCB2_I2C_S_CMD
UART_SCB__I2C_STATUS EQU CYREG_SCB2_I2C_STATUS
UART_SCB__INTR_CAUSE EQU CYREG_SCB2_INTR_CAUSE
UART_SCB__INTR_I2C_EC EQU CYREG_SCB2_INTR_I2C_EC
UART_SCB__INTR_I2C_EC_MASK EQU CYREG_SCB2_INTR_I2C_EC_MASK
UART_SCB__INTR_I2C_EC_MASKED EQU CYREG_SCB2_INTR_I2C_EC_MASKED
UART_SCB__INTR_M EQU CYREG_SCB2_INTR_M
UART_SCB__INTR_M_MASK EQU CYREG_SCB2_INTR_M_MASK
UART_SCB__INTR_M_MASKED EQU CYREG_SCB2_INTR_M_MASKED
UART_SCB__INTR_M_SET EQU CYREG_SCB2_INTR_M_SET
UART_SCB__INTR_RX EQU CYREG_SCB2_INTR_RX
UART_SCB__INTR_RX_MASK EQU CYREG_SCB2_INTR_RX_MASK
UART_SCB__INTR_RX_MASKED EQU CYREG_SCB2_INTR_RX_MASKED
UART_SCB__INTR_RX_SET EQU CYREG_SCB2_INTR_RX_SET
UART_SCB__INTR_S EQU CYREG_SCB2_INTR_S
UART_SCB__INTR_S_MASK EQU CYREG_SCB2_INTR_S_MASK
UART_SCB__INTR_S_MASKED EQU CYREG_SCB2_INTR_S_MASKED
UART_SCB__INTR_S_SET EQU CYREG_SCB2_INTR_S_SET
UART_SCB__INTR_SPI_EC EQU CYREG_SCB2_INTR_SPI_EC
UART_SCB__INTR_SPI_EC_MASK EQU CYREG_SCB2_INTR_SPI_EC_MASK
UART_SCB__INTR_SPI_EC_MASKED EQU CYREG_SCB2_INTR_SPI_EC_MASKED
UART_SCB__INTR_TX EQU CYREG_SCB2_INTR_TX
UART_SCB__INTR_TX_MASK EQU CYREG_SCB2_INTR_TX_MASK
UART_SCB__INTR_TX_MASKED EQU CYREG_SCB2_INTR_TX_MASKED
UART_SCB__INTR_TX_SET EQU CYREG_SCB2_INTR_TX_SET
UART_SCB__RX_CTRL EQU CYREG_SCB2_RX_CTRL
UART_SCB__RX_FIFO_CTRL EQU CYREG_SCB2_RX_FIFO_CTRL
UART_SCB__RX_FIFO_RD EQU CYREG_SCB2_RX_FIFO_RD
UART_SCB__RX_FIFO_RD_SILENT EQU CYREG_SCB2_RX_FIFO_RD_SILENT
UART_SCB__RX_FIFO_STATUS EQU CYREG_SCB2_RX_FIFO_STATUS
UART_SCB__RX_MATCH EQU CYREG_SCB2_RX_MATCH
UART_SCB__SPI_CTRL EQU CYREG_SCB2_SPI_CTRL
UART_SCB__SPI_STATUS EQU CYREG_SCB2_SPI_STATUS
UART_SCB__SS0_POSISTION EQU 0
UART_SCB__SS1_POSISTION EQU 1
UART_SCB__SS2_POSISTION EQU 2
UART_SCB__SS3_POSISTION EQU 3
UART_SCB__STATUS EQU CYREG_SCB2_STATUS
UART_SCB__TX_CTRL EQU CYREG_SCB2_TX_CTRL
UART_SCB__TX_FIFO_CTRL EQU CYREG_SCB2_TX_FIFO_CTRL
UART_SCB__TX_FIFO_STATUS EQU CYREG_SCB2_TX_FIFO_STATUS
UART_SCB__TX_FIFO_WR EQU CYREG_SCB2_TX_FIFO_WR
UART_SCB__UART_CTRL EQU CYREG_SCB2_UART_CTRL
UART_SCB__UART_FLOW_CTRL EQU CYREG_SCB2_UART_FLOW_CTRL
UART_SCB__UART_RX_CTRL EQU CYREG_SCB2_UART_RX_CTRL
UART_SCB__UART_RX_STATUS EQU CYREG_SCB2_UART_RX_STATUS
UART_SCB__UART_TX_CTRL EQU CYREG_SCB2_UART_TX_CTRL
UART_SCBCLK__CTRL_REGISTER EQU CYREG_PERI_PCLK_CTL2
UART_SCBCLK__DIV_ID EQU 0x00000042
UART_SCBCLK__DIV_REGISTER EQU CYREG_PERI_DIV_16_CTL2
UART_SCBCLK__PA_DIV_ID EQU 0x000000FF
UART_tx__0__DR EQU CYREG_GPIO_PRT0_DR
UART_tx__0__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
UART_tx__0__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
UART_tx__0__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
UART_tx__0__HSIOM EQU CYREG_HSIOM_PORT_SEL0
UART_tx__0__HSIOM_GPIO EQU 0
UART_tx__0__HSIOM_MASK EQU 0x00F00000
UART_tx__0__HSIOM_SHIFT EQU 20
UART_tx__0__HSIOM_UART EQU 11
UART_tx__0__HSIOM_UART_TX EQU 11
UART_tx__0__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
UART_tx__0__INTR EQU CYREG_GPIO_PRT0_INTR
UART_tx__0__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
UART_tx__0__INTSTAT EQU CYREG_GPIO_PRT0_INTR
UART_tx__0__MASK EQU 0x20
UART_tx__0__PC EQU CYREG_GPIO_PRT0_PC
UART_tx__0__PC2 EQU CYREG_GPIO_PRT0_PC2
UART_tx__0__PORT EQU 0
UART_tx__0__PS EQU CYREG_GPIO_PRT0_PS
UART_tx__0__SHIFT EQU 5
UART_tx__DR EQU CYREG_GPIO_PRT0_DR
UART_tx__DR_CLR EQU CYREG_GPIO_PRT0_DR_CLR
UART_tx__DR_INV EQU CYREG_GPIO_PRT0_DR_INV
UART_tx__DR_SET EQU CYREG_GPIO_PRT0_DR_SET
UART_tx__INTCFG EQU CYREG_GPIO_PRT0_INTR_CFG
UART_tx__INTR EQU CYREG_GPIO_PRT0_INTR
UART_tx__INTR_CFG EQU CYREG_GPIO_PRT0_INTR_CFG
UART_tx__INTSTAT EQU CYREG_GPIO_PRT0_INTR
UART_tx__MASK EQU 0x20
UART_tx__PC EQU CYREG_GPIO_PRT0_PC
UART_tx__PC2 EQU CYREG_GPIO_PRT0_PC2
UART_tx__PORT EQU 0
UART_tx__PS EQU CYREG_GPIO_PRT0_PS
UART_tx__SHIFT EQU 5

/* ant1 */
ant1__0__DR EQU CYREG_GPIO_PRT2_DR
ant1__0__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant1__0__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant1__0__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant1__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
ant1__0__HSIOM_MASK EQU 0x0000000F
ant1__0__HSIOM_SHIFT EQU 0
ant1__0__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant1__0__INTR EQU CYREG_GPIO_PRT2_INTR
ant1__0__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant1__0__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant1__0__MASK EQU 0x01
ant1__0__PC EQU CYREG_GPIO_PRT2_PC
ant1__0__PC2 EQU CYREG_GPIO_PRT2_PC2
ant1__0__PORT EQU 2
ant1__0__PS EQU CYREG_GPIO_PRT2_PS
ant1__0__SHIFT EQU 0
ant1__DR EQU CYREG_GPIO_PRT2_DR
ant1__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant1__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant1__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant1__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant1__INTR EQU CYREG_GPIO_PRT2_INTR
ant1__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant1__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant1__MASK EQU 0x01
ant1__PC EQU CYREG_GPIO_PRT2_PC
ant1__PC2 EQU CYREG_GPIO_PRT2_PC2
ant1__PORT EQU 2
ant1__PS EQU CYREG_GPIO_PRT2_PS
ant1__SHIFT EQU 0

/* ant2 */
ant2__0__DR EQU CYREG_GPIO_PRT2_DR
ant2__0__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant2__0__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant2__0__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant2__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
ant2__0__HSIOM_MASK EQU 0x000000F0
ant2__0__HSIOM_SHIFT EQU 4
ant2__0__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant2__0__INTR EQU CYREG_GPIO_PRT2_INTR
ant2__0__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant2__0__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant2__0__MASK EQU 0x02
ant2__0__PC EQU CYREG_GPIO_PRT2_PC
ant2__0__PC2 EQU CYREG_GPIO_PRT2_PC2
ant2__0__PORT EQU 2
ant2__0__PS EQU CYREG_GPIO_PRT2_PS
ant2__0__SHIFT EQU 1
ant2__DR EQU CYREG_GPIO_PRT2_DR
ant2__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant2__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant2__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant2__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant2__INTR EQU CYREG_GPIO_PRT2_INTR
ant2__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant2__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant2__MASK EQU 0x02
ant2__PC EQU CYREG_GPIO_PRT2_PC
ant2__PC2 EQU CYREG_GPIO_PRT2_PC2
ant2__PORT EQU 2
ant2__PS EQU CYREG_GPIO_PRT2_PS
ant2__SHIFT EQU 1

/* ant3 */
ant3__0__DR EQU CYREG_GPIO_PRT2_DR
ant3__0__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant3__0__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant3__0__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant3__0__HSIOM EQU CYREG_HSIOM_PORT_SEL2
ant3__0__HSIOM_MASK EQU 0x00000F00
ant3__0__HSIOM_SHIFT EQU 8
ant3__0__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant3__0__INTR EQU CYREG_GPIO_PRT2_INTR
ant3__0__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant3__0__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant3__0__MASK EQU 0x04
ant3__0__PC EQU CYREG_GPIO_PRT2_PC
ant3__0__PC2 EQU CYREG_GPIO_PRT2_PC2
ant3__0__PORT EQU 2
ant3__0__PS EQU CYREG_GPIO_PRT2_PS
ant3__0__SHIFT EQU 2
ant3__DR EQU CYREG_GPIO_PRT2_DR
ant3__DR_CLR EQU CYREG_GPIO_PRT2_DR_CLR
ant3__DR_INV EQU CYREG_GPIO_PRT2_DR_INV
ant3__DR_SET EQU CYREG_GPIO_PRT2_DR_SET
ant3__INTCFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant3__INTR EQU CYREG_GPIO_PRT2_INTR
ant3__INTR_CFG EQU CYREG_GPIO_PRT2_INTR_CFG
ant3__INTSTAT EQU CYREG_GPIO_PRT2_INTR
ant3__MASK EQU 0x04
ant3__PC EQU CYREG_GPIO_PRT2_PC
ant3__PC2 EQU CYREG_GPIO_PRT2_PC2
ant3__PORT EQU 2
ant3__PS EQU CYREG_GPIO_PRT2_PS
ant3__SHIFT EQU 2

/* Miscellaneous */
CYDEV_BCLK__HFCLK__HZ EQU 24000000
CYDEV_BCLK__HFCLK__KHZ EQU 24000
CYDEV_BCLK__HFCLK__MHZ EQU 24
CYDEV_BCLK__SYSCLK__HZ EQU 24000000
CYDEV_BCLK__SYSCLK__KHZ EQU 24000
CYDEV_BCLK__SYSCLK__MHZ EQU 24
CYDEV_CHIP_DIE_LEOPARD EQU 1
CYDEV_CHIP_DIE_PSOC4A EQU 26
CYDEV_CHIP_DIE_PSOC5LP EQU 2
CYDEV_CHIP_DIE_PSOC5TM EQU 3
CYDEV_CHIP_DIE_TMA4 EQU 4
CYDEV_CHIP_DIE_UNKNOWN EQU 0
CYDEV_CHIP_FAMILY_FM0P EQU 5
CYDEV_CHIP_FAMILY_FM3 EQU 6
CYDEV_CHIP_FAMILY_FM4 EQU 7
CYDEV_CHIP_FAMILY_PSOC3 EQU 1
CYDEV_CHIP_FAMILY_PSOC4 EQU 2
CYDEV_CHIP_FAMILY_PSOC5 EQU 3
CYDEV_CHIP_FAMILY_PSOC6 EQU 4
CYDEV_CHIP_FAMILY_UNKNOWN EQU 0
CYDEV_CHIP_FAMILY_USED EQU CYDEV_CHIP_FAMILY_PSOC4
CYDEV_CHIP_JTAG_ID EQU 0x254811B5
CYDEV_CHIP_MEMBER_3A EQU 1
CYDEV_CHIP_MEMBER_4A EQU 26
CYDEV_CHIP_MEMBER_4AA EQU 25
CYDEV_CHIP_MEMBER_4AB EQU 30
CYDEV_CHIP_MEMBER_4AC EQU 14
CYDEV_CHIP_MEMBER_4AD EQU 15
CYDEV_CHIP_MEMBER_4AE EQU 16
CYDEV_CHIP_MEMBER_4D EQU 20
CYDEV_CHIP_MEMBER_4E EQU 6
CYDEV_CHIP_MEMBER_4F EQU 27
CYDEV_CHIP_MEMBER_4G EQU 4
CYDEV_CHIP_MEMBER_4H EQU 24
CYDEV_CHIP_MEMBER_4I EQU 32
CYDEV_CHIP_MEMBER_4J EQU 21
CYDEV_CHIP_MEMBER_4K EQU 22
CYDEV_CHIP_MEMBER_4L EQU 31
CYDEV_CHIP_MEMBER_4M EQU 29
CYDEV_CHIP_MEMBER_4N EQU 11
CYDEV_CHIP_MEMBER_4O EQU 8
CYDEV_CHIP_MEMBER_4P EQU 28
CYDEV_CHIP_MEMBER_4Q EQU 17
CYDEV_CHIP_MEMBER_4R EQU 9
CYDEV_CHIP_MEMBER_4S EQU 12
CYDEV_CHIP_MEMBER_4T EQU 10
CYDEV_CHIP_MEMBER_4U EQU 5
CYDEV_CHIP_MEMBER_4V EQU 23
CYDEV_CHIP_MEMBER_4W EQU 13
CYDEV_CHIP_MEMBER_4X EQU 7
CYDEV_CHIP_MEMBER_4Y EQU 18
CYDEV_CHIP_MEMBER_4Z EQU 19
CYDEV_CHIP_MEMBER_5A EQU 3
CYDEV_CHIP_MEMBER_5B EQU 2
CYDEV_CHIP_MEMBER_6A EQU 33
CYDEV_CHIP_MEMBER_FM3 EQU 37
CYDEV_CHIP_MEMBER_FM4 EQU 38
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE1 EQU 34
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE2 EQU 35
CYDEV_CHIP_MEMBER_PDL_FM0P_TYPE3 EQU 36
CYDEV_CHIP_MEMBER_UNKNOWN EQU 0
CYDEV_CHIP_MEMBER_USED EQU CYDEV_CHIP_MEMBER_4V
CYDEV_CHIP_DIE_EXPECT EQU CYDEV_CHIP_MEMBER_USED
CYDEV_CHIP_DIE_ACTUAL EQU CYDEV_CHIP_DIE_EXPECT
CYDEV_CHIP_REV_LEOPARD_ES1 EQU 0
CYDEV_CHIP_REV_LEOPARD_ES2 EQU 1
CYDEV_CHIP_REV_LEOPARD_ES3 EQU 3
CYDEV_CHIP_REV_LEOPARD_PRODUCTION EQU 3
CYDEV_CHIP_REV_PSOC4A_ES0 EQU 17
CYDEV_CHIP_REV_PSOC4A_PRODUCTION EQU 17
CYDEV_CHIP_REV_PSOC5LP_ES0 EQU 0
CYDEV_CHIP_REV_PSOC5LP_PRODUCTION EQU 0
CYDEV_CHIP_REV_PSOC5TM_ES0 EQU 0
CYDEV_CHIP_REV_PSOC5TM_ES1 EQU 1
CYDEV_CHIP_REV_PSOC5TM_PRODUCTION EQU 1
CYDEV_CHIP_REV_TMA4_ES EQU 17
CYDEV_CHIP_REV_TMA4_ES2 EQU 33
CYDEV_CHIP_REV_TMA4_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_3A_ES1 EQU 0
CYDEV_CHIP_REVISION_3A_ES2 EQU 1
CYDEV_CHIP_REVISION_3A_ES3 EQU 3
CYDEV_CHIP_REVISION_3A_PRODUCTION EQU 3
CYDEV_CHIP_REVISION_4A_ES0 EQU 17
CYDEV_CHIP_REVISION_4A_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_4AA_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4AB_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4AC_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4AD_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4AE_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4D_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4E_CCG2_NO_USBPD EQU 0
CYDEV_CHIP_REVISION_4E_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION_256DMA EQU 0
CYDEV_CHIP_REVISION_4F_PRODUCTION_256K EQU 0
CYDEV_CHIP_REVISION_4G_ES EQU 17
CYDEV_CHIP_REVISION_4G_ES2 EQU 33
CYDEV_CHIP_REVISION_4G_PRODUCTION EQU 17
CYDEV_CHIP_REVISION_4H_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4I_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4J_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4K_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4L_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4M_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4N_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4O_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4P_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4Q_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4R_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4S_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4T_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4U_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4V_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4W_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4X_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4Y_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_4Z_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_5A_ES0 EQU 0
CYDEV_CHIP_REVISION_5A_ES1 EQU 1
CYDEV_CHIP_REVISION_5A_PRODUCTION EQU 1
CYDEV_CHIP_REVISION_5B_ES0 EQU 0
CYDEV_CHIP_REVISION_5B_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_6A_ES EQU 17
CYDEV_CHIP_REVISION_6A_NO_UDB EQU 33
CYDEV_CHIP_REVISION_6A_PRODUCTION EQU 33
CYDEV_CHIP_REVISION_FM3_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_FM4_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE1_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE2_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_PDL_FM0P_TYPE3_PRODUCTION EQU 0
CYDEV_CHIP_REVISION_USED EQU CYDEV_CHIP_REVISION_4V_PRODUCTION
CYDEV_CHIP_REV_EXPECT EQU CYDEV_CHIP_REVISION_USED
CYDEV_CONFIG_READ_ACCELERATOR EQU 1
CYDEV_CONFIG_UNUSED_IO_AllowButWarn EQU 0
CYDEV_CONFIG_UNUSED_IO_AllowWithInfo EQU 1
CYDEV_CONFIG_UNUSED_IO_Disallowed EQU 2
CYDEV_CONFIG_UNUSED_IO EQU CYDEV_CONFIG_UNUSED_IO_Disallowed
CYDEV_CONFIGURATION_COMPRESSED EQU 1
CYDEV_CONFIGURATION_MODE_COMPRESSED EQU 0
CYDEV_CONFIGURATION_MODE EQU CYDEV_CONFIGURATION_MODE_COMPRESSED
CYDEV_CONFIGURATION_MODE_DMA EQU 2
CYDEV_CONFIGURATION_MODE_UNCOMPRESSED EQU 1
CYDEV_DEBUG_PROTECT_KILL EQU 4
CYDEV_DEBUG_PROTECT_OPEN EQU 1
CYDEV_DEBUG_PROTECT EQU CYDEV_DEBUG_PROTECT_OPEN
CYDEV_DEBUG_PROTECT_PROTECTED EQU 2
CYDEV_DEBUGGING_DPS_Disable EQU 3
CYDEV_DEBUGGING_DPS_SWD EQU 2
CYDEV_DEBUGGING_DPS EQU CYDEV_DEBUGGING_DPS_SWD
CYDEV_DEBUGGING_ENABLE EQU 1
CYDEV_DFT_SELECT_CLK0 EQU 8
CYDEV_DFT_SELECT_CLK1 EQU 9
CYDEV_DMA_CHANNELS_AVAILABLE EQU 8
CYDEV_HEAP_SIZE EQU 0x80
CYDEV_IMO_TRIMMED_BY_USB EQU 0
CYDEV_IMO_TRIMMED_BY_WCO EQU 0
CYDEV_INTR_NUMBER_DMA EQU 14
CYDEV_IS_EXPORTING_CODE EQU 0
CYDEV_IS_IMPORTING_CODE EQU 0
CYDEV_PROJ_TYPE EQU 0
CYDEV_PROJ_TYPE_BOOTLOADER EQU 1
CYDEV_PROJ_TYPE_LAUNCHER EQU 5
CYDEV_PROJ_TYPE_LOADABLE EQU 2
CYDEV_PROJ_TYPE_LOADABLEANDBOOTLOADER EQU 4
CYDEV_PROJ_TYPE_MULTIAPPBOOTLOADER EQU 3
CYDEV_PROJ_TYPE_STANDARD EQU 0
CYDEV_STACK_SIZE EQU 0x0800
CYDEV_USE_BUNDLED_CMSIS EQU 1
CYDEV_VARIABLE_VDDA EQU 1
CYDEV_VDDA_MV EQU 3300
CYDEV_VDDD_MV EQU 3300
CYDEV_WDT_GENERATE_ISR EQU 1
CYIPBLOCK_m0s8cpussv3_VERSION EQU 1
CYIPBLOCK_m0s8crypto_VERSION EQU 2
CYIPBLOCK_m0s8csdv2_VERSION EQU 2
CYIPBLOCK_m0s8exco_VERSION EQU 1
CYIPBLOCK_m0s8ioss_VERSION EQU 1
CYIPBLOCK_m0s8lcd_VERSION EQU 2
CYIPBLOCK_m0s8lpcomp_VERSION EQU 2
CYIPBLOCK_m0s8pass4a_VERSION EQU 1
CYIPBLOCK_m0s8peri_VERSION EQU 1
CYIPBLOCK_m0s8scb_VERSION EQU 2
CYIPBLOCK_m0s8tcpwm_VERSION EQU 2
CYIPBLOCK_m0s8wco_VERSION EQU 1
CYIPBLOCK_s8srsslt_VERSION EQU 1
DMA_CHANNELS_USED__MASK EQU 0
CYDEV_BOOTLOADER_ENABLE EQU 0

#endif /* INCLUDED_CYFITTERIAR_INC */
