/*******************************************************************************
 Simple SPI Transfer function

  File Name:
    drv_spi.h

    
 *******************************************************************************/



#ifndef _DRV_SPI_H
#define	_DRV_SPI_H

#include <string.h>
#define CS_SET  PORTBbits.RB14 = 1
#define CS_CLR  PORTBbits.RB14 = 0




//! SPI Read/Write Transfer

uint8_t DRV_SPI_TransferData(uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t XferSize);

//! SPI Chip Select assert/de-assert

int8_t DRV_SPI_ChipSelectAssert(bool assert);

#endif	/* _DRV_SPI_H */

