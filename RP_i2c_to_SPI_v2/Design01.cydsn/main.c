/* ========================================
 *
 * Copyright KA6S, 2021
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 *
 *
 * ========================================
*/
#include "project.h"

/* Valid command packet size of five bytes */
#define PACKET_SIZE          (0x05u)

/* Master write and read buffer of size five and three bytes */
#define SL_RD_BUFFER_SIZE    (0x03u)
#define SL_WR_BUFFER_SIZE    (PACKET_SIZE)

/* Start and end of packet markers */
#define PACKET_SOP           (0x01u)
#define PACKET_EOP           (0x17u)

/* Command valid status */
#define STS_CMD_DONE         (0x00u)
#define STS_CMD_FAIL         (0xFFu)

/* Packet positions */
#define PACKET_SOP_POS       (0x00u)
#define PACKET_STS_POS       (0x01u)
#define PACKET_RED_POS       (0x01u)
#define PACKET_GREEN_POS     (0x02u)
#define PACKET_BLUE_POS      (0x03u)
#define PACKET_EOP_POS       (0x04u)


/* Controls the frequency at which the light changes */
#define CMD_TO_CMD_DELAY    (500u)
#define CMD_STALL           (20u)

// SPI
#define BUFFER_SIZE (64UL)
/* Data buffers */



/*******************************************************************************
* Function prototypes
*******************************************************************************/
static void pollI2CwriteBuffer(void);

/* I2C read and write buffers */
uint8_t i2cReadBuffer [SL_RD_BUFFER_SIZE] = {PACKET_SOP, STS_CMD_FAIL, PACKET_EOP};
uint8_t i2cWriteBuffer[SL_WR_BUFFER_SIZE] ;

// SPI read/write buffers
/*
uint8_t bufferTX(BUFFER_SIZE);  
uint8_t bufferRX(BUFFER_SIZE);
*/


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    SPI_Start();
    
    // Turn off all of the Antenna controls - leaving 1 enabled at reset
    ant1_Write(1);
    ant2_Write(0);
    ant3_Write(0);
    

   
    /* Initialize and enable I2C component in slave mode. */
    I2C_I2CSlaveInitReadBuf (i2cReadBuffer,  BUFFER_SIZE);
    I2C_I2CSlaveInitWriteBuf(i2cWriteBuffer, BUFFER_SIZE);
    I2C_Start();

    for(;;)
    {
       /* continuously poll to check whether master has 
          written  data to write buffer. */
       pollI2CwriteBuffer();
       /* Poll every 1ms */
        CyDelay(1);   // Redefine this for 1ms 
    }


}
/*******************************************************************************
* Function Name: pollI2CwriteBuffer
****************************************************************************//**
*
* Checks the status of slave to know if any data is written by master into write
* buffer. If data is written, check for error. If no error while writing 
* then check whether required number of bytes is written. If required number of bytes are 
* written, if start and end packets are correct set the TCPWM compare value to the respective 
* RGB LED TCPWM's to control the color.
*
* \param None
*
* \return
*  None
*
*******************************************************************************/
static void pollI2CwriteBuffer(void)
{
    int work;
    if( 0UL != (I2C_I2CSlaveStatus() & I2C_I2C_SSTAT_WR_CMPLT))
    {
 
     if (PACKET_SIZE == I2C_I2CSlaveGetWriteBufSize() ) 
        {
           
         /* Check start and end of packet markers. */
         if ((i2cWriteBuffer[PACKET_SOP_POS] == PACKET_SOP) &&
             (i2cWriteBuffer[PACKET_EOP_POS] == PACKET_EOP))
            {                  
              // Send SPI message                   
              if(i2cWriteBuffer[PACKET_SOP_POS] == 0x1) 
                {
                  // ALEX RX control
                  // Need SS select = 2
                  SPI_SpiSetActiveSlaveSelect(SPI_SPI_SLAVE_SELECT2);
                  SPI_SpiUartPutArray(i2cWriteBuffer[PACKET_SOP_POS+1], 2);
            
                }
            if(i2cWriteBuffer[PACKET_SOP_POS] == 0x2) 
                {
                  // ALEX TX control
                  // Need SS select = 1
                  work = i2cWriteBuffer[PACKET_SOP_POS+1];
                  work = (work >> 4);
                  if(work & 0x2) {
                      ant1_Write(0);
                      ant2_Write(1);
                      ant3_Write(0);
                  }
                  if(work & 0x4) {
                      ant1_Write(0);
                      ant2_Write(0);
                      ant3_Write(1);
                  }
                 
                  SPI_SpiSetActiveSlaveSelect(SPI_SPI_SLAVE_SELECT1);
                  SPI_SpiUartPutArray(i2cWriteBuffer[PACKET_SOP_POS+1], 2);
                }
            }
        }
       
        
        /* Configure write buffer for the next write. */
        I2C_I2CSlaveClearWriteBuf();
        (void) I2C_I2CSlaveClearWriteStatus();
       
    
        /* Check read complete event. */
        if (0u != (I2C_I2CSlaveStatus() & I2C_I2C_SSTAT_RD_CMPLT))
        {
            /* Clear the slave read buffer and status */
            I2C_I2CSlaveClearReadBuf();
            (void) I2C_I2CSlaveClearReadStatus();
        }
    }

}


/* [] END OF FILE */
