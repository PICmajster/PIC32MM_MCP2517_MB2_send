/*
    File Name:        :  i2c1.h

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/

#ifndef I2C1_H
#define	I2C1_H

extern uint8_t error_flag ;
/*deklaracje funkcji*/
void i2c1_Initialize(void);               /*Init the I2C module*/
void i2c_start(void);                    /*generates an I2C Start condition*/
void i2c_restart(void);                  /*generates an I2C Restart condition (for reads)*/
void i2c_stop(void);                     /*generates an I2C Stop condition*/
void i2c_write(unsigned char i2c_data);  /*writes a byte to the I2C bus*/
unsigned char i2c_read(void);            /*reads a byte from the I2C bus*/
void i2c_ack(void);                      /*generates a Master Acknowledge*/
void i2c_nack(void);                     /*generates a Master No Acknowledge*/
void error_handling(void);
void i2c_write_buf( uint8_t adr_device, uint8_t adr, uint8_t len, uint8_t *buf);
void i2c_read_buf(uint8_t adr_device, uint8_t adr, uint8_t len, uint8_t *buf);

/*dedykowana obsluga EEPROM*/
void EEPROM_sequential_read_buf(uint8_t adr_device, uint16_t subAddr, uint16_t len, char *buf);
void EEPROM_write_buf(uint8_t adr_device, uint16_t subAddr,  uint16_t len, char *buf);
void EEPROM_write_buf_page_write(uint8_t adr_device, uint8_t numer_strony, uint8_t numer_komorki, uint16_t len, char *buf);

/*dedykowana obsluga EERAM*/
void EERAM_write_STATUS_REGISTER(uint8_t adr_device, uint16_t subAddr,  unsigned char i2c_data);
void EERAM_write_buf(uint8_t adr_device, uint16_t subAddr,  uint16_t len, char *buf);
void EERAM_write_byte(uint8_t adr_device, uint16_t subAddr,  unsigned char i2c_data);
unsigned char EERAM_read_byte(uint8_t adr_device, uint16_t subAddr);

/*zapis i odczyt struktur*/
void EERAM_write_structure(uint8_t adr_device, uint16_t subAddr,  uint16_t len, void *struktura);
void EERAM_sequential_read_structure(uint8_t adr_device, uint16_t subAddr, uint16_t len, void *struktura);

#endif	/* I2C1_H */

