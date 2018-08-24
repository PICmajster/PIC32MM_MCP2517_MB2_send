/*******************************************************************************
 Simple SPI Transfer function

  File Name:
    drv_spi.c

  Summary:
    Initializes SPI 1. Transfers data over SPI.
    Uses SPI FIFO to speed up transfer.
    
 *******************************************************************************/


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "drv_spi.h"
#include "mcc_generated_files/spi3.h"


int8_t DRV_SPI_ChipSelectAssert(bool assert)
{
    int8_t error = 0;
            
            if (assert) CS_CLR;
            else CS_SET;
       
    return error;
}


uint8_t DRV_SPI_TransferData(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
    int8_t error = 0;
   
    // Assert CS
    error = DRV_SPI_ChipSelectAssert(true);
    if (error != 0) return error;

    SPI3_Exchange8bitBuffer(SpiTxData, spiTransferSize, SpiRxData);
        // Make sure data gets transmitted even if buffer wasn't empty when we started out with
   
    // De-assert CS
    error = DRV_SPI_ChipSelectAssert(false);

    return error;
}

