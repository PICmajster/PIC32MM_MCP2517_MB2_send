/*
    File Name:        :  i2c.c

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/
#include "mcc.h"
#include "xc.h" /* wykrywa rodzaj procka i includuje odpowiedni plik naglowkowy "pic32mm0256gpm048.h"*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h> /*definicje uint8_t itp*/
#include "../delay.h"
#include "i2c1.h"

uint8_t error_flag ;
/*definicje funkcji*/

void i2c1_Initialize(void)
{
    /*pin RB5(SDA) i RC9(SCL) piny alternatywne dla I2C1*/
    
     //AFDEVOPTbits.ALTI2C = 0;
    
    // initialize the hardware
    // Baud Rate Generator Value: I2CBRG 116; FCPU 24 MHz, CLK_I2C 100kHz  
    I2C1BRG = 0x74;
    /*ACKEN disabled; STRICT disabled; STREN disabled; GCEN disabled; SMEN disabled; DISSLW enabled; SBCDE disabled; SIDL disabled; BOEN disabled; 
    ACKDT Sends ACK; SCIE disabled; PCIE disabled; SCLREL Holds; RSEN disabled; SDAHT disabled; A10M 7 Bit; PEN disabled; RCEN disabled; 
    SEN disabled; ON enabled;*/ 
    I2C1CON = 0x8000;
    // BCL disabled; P disabled; S disabled; I2COV disabled; IWCOL disabled; 
    I2C1STAT = 0x0;
    /*zerujemy rejestr nadawczy i odbiorczy*/
    I2C1RCV = 0;
    I2C1TRN = 0;
    /* I2C 1 Master */
    /*clear the master interrupt flag and bus collision*/
    IFS2bits.I2C1MIF = 0;
    IFS2bits.I2C1BCIF = 0;
    /*enable the master interrupt and bus collision*/
    IEC2bits.I2C1MIE = 1;
    IEC2bits.I2C1BCIE = 1;
    

}

void error_handling(void){
    /*Timer1 zlicza przez ok 21 ms przy FCY 24MHz*/
    TMR1 =0; /*clear Timer1*/
    IFS0bits.T1IF=0 ; /*zeruj flage od przepelnienia Timer1*/
    T1CONbits.TCKPS = 0b01 ; /*Prescaler 1:8 , daje nam 3 MHz*/
    T1CONbits.ON=1;    /*start Timer1*/
    /*czekaj na flage I2C1MIF po zakonczeniu poprawnie dowolnej operacji przez Mastera na I2c1
      lub na flage I2C1BCIF - Bus Collision
      lub na przepelnienie Timer1 czyli ok 21 ms*/ 
    while(!(IFS2bits.I2C1MIF | IFS2bits.I2C1BCIF | IFS0bits.T1IF ));  
        if(IFS0bits.T1IF){ /*jesli Timer1 przepelniony*/
        IFS0bits.T1IF=0 ; /*zeruj flage od przepelnienia Timer1*/
        IFS2bits.I2C1MIF=0; /*clear the I2C1 general flag*/
        IFS2bits.I2C1BCIF=0; /*clear the I2C1 flag for Bus Collision*/
        error_flag = 1; /*set the error flag*/
        PORTBbits.RB6 = 1 ; /*LED ON*/
        /*tutaj kod uzytkownika do obslugi bledu, np zapalenie diody LED etc*/
    }
    else {error_flag = 0; /*clear the error flag*/
        IFS2bits.I2C1MIF=0; /*clear the I2C3 general flag*/
        IFS2bits.I2C1BCIF=0; /*clear the I2C3 flag for Bus Collision*/
        PORTBbits.RB6 = 0; /*LED OFF*/
    }
    
    T1CONbits.ON=0; /*stop Timer1*/
    TMR1 =0; /*clear Timer1*/
    
}


void i2c_start(void){
   while(I2C1STATbits.TRSTAT); /*czekaj az linia bedzie wolna */
   I2C1CONbits.SEN = 1;        /*generuj sygnal start*/
   error_handling();  
    
}

void i2c_restart(void){
    while(I2C1STATbits.TRSTAT); /*czekaj az linia bedzie wolna */
    I2C1CONbits.RSEN=1; /*generuj restart*/
    error_handling();  
}

void i2c_stop(void){
    while(I2C1STATbits.TRSTAT); /*czekaj az linia bedzie wolna */
    I2C1CONbits.PEN=1; /*generuj stop*/
    error_handling();  
}

void i2c_write(unsigned char i2c_data){
    while(I2C1STATbits.TRSTAT); /*czekaj az linia bedzie wolna */
    I2C1TRN=i2c_data; /*load byte in the transmiter buffer*/
    error_handling();
}

unsigned char i2c_read(void){
    while(I2C1STATbits.TRSTAT); /*czekaj az linia bedzie wolna */
    I2C1CONbits.RCEN=1; /*enable Master receive*/
    error_handling();
    return(I2C1RCV); /*return data in buffer*/
}

void i2c_ack(void){
    I2C1CONbits.ACKDT=0; /*clear the related flag for ACK*/
    I2C1CONbits.ACKEN=1; /*start ACK sequence*/
    error_handling();
}

void i2c_nack(void){
    I2C1CONbits.ACKDT=1; /*set the related flag for NotACK*/
    I2C1CONbits.ACKEN=1; /*start ACK sequence*/
    error_handling();
    I2C1CONbits.ACKDT=0; /*clear the related flag for ACK*/
}

void i2c_write_buf( uint8_t adr_device, uint8_t adr, uint8_t len, uint8_t *buf ) {
	i2c_start();
	i2c_write(adr_device); /*wyslanie adresu urzadzenia z bitem R/W w stanie niskim*/
	i2c_write(adr);
	while (len--) i2c_write(*buf++);
	i2c_stop();
}

void i2c_read_buf(uint8_t adr_device, uint8_t adr, uint8_t len, uint8_t *buf) {
	i2c_start();
	i2c_write(adr_device);/*wys?anie adresu urzadzenia z bitem R/W w stanie niskim*/
	i2c_write(adr);
	i2c_start();
	i2c_write(adr_device + 1);/*zapisuje adres urzadzenia z bitem R/W ustawionym na 1 czyli o 1 zwiekszamy adres urzadzenia*/
	while (len--){
     if(len) {
     *buf++ = i2c_read();
     i2c_ack();
        }
     else {
     *buf++ = i2c_read();
     i2c_nack();
         }
     }
   	i2c_stop();
}
/*Funkcje dedykowane do obslugi pamieci EEPROM PIC 24LCxxx*/
/*odczyt sekwencyjny danych z pamieci EEPROM.*/
void EEPROM_sequential_read_buf(uint8_t adr_device, uint16_t subAddr, uint16_t len, char *buf) {

        i2c_start();
		i2c_write(adr_device); /*wyslanie adresu urzadzenia z bitem R/W w stanie niskim*/
        i2c_write((subAddr & 0xFF00) >> 8) ; /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF)  ;       /*wyslanie mlodszego bajtu adresu pamieci*/
        i2c_start();
		i2c_write(adr_device + 1);/*zapisuje adres urzadzenia z bitem R/W ustawionym na 1 czyli o 1 zwiekszamy adres urzadzenia*/
		i2c_ack();
        while(len--)
        {
        *buf++ = i2c_read();
        i2c_ack();
        }
        i2c_nack();
        i2c_stop();
}

/*zapis danych do pamieci EEPROM w pojedynczych bajtach*/
void EEPROM_write_buf(uint8_t adr_device, uint16_t subAddr,  uint16_t len, char *buf) {

	while (len--) {
		i2c_start();
		i2c_write(adr_device); /*wyslanie bajtu adresu z bitem R/W ustawionym na zero*/
		i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
		i2c_write(*buf++);
		i2c_stop();
		delayMs(5); /*oczekiwanie na zapis*/
		subAddr++;
	}
}

/*zapis danych do pamieci EEPROM w trybie PAGE WRITE czyli maksymalnie 128 bajty w obrembie jednej strony*/
void EEPROM_write_buf_page_write(uint8_t adr_device, uint8_t numer_strony, uint8_t numer_komorki, uint16_t len, char *buf) {
      uint16_t subAddr;
      /*subAddr sklada sie z :
        numer komorki to numer w obrebie jednej strony czyli od 0 do 127 
        numer strony to numer jednej z 512 stron od 0 do 511
        24LC512 ma 512 stron 128 bajtowych co lacznie daje 65536 bajtow*/
		i2c_start();
        i2c_write(adr_device); /*wyslanie bajtu adresu Slave z bitem R/W ustawionym na zero*/
        /*wyliczamy adres poczatku uwzgledniajac numer strony 0-511 i numer komorki na stronie 0-127*/
		subAddr = (numer_strony*128) + numer_komorki;
        
        i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
        
        while (len--) {
		/*inkrementujemy tylko tablice z danymi,adres w obrebie strony 0-127 bajtow jest inkrementowany automatycznie przez eeprom*/
            i2c_write(*buf++); 
            }
		i2c_stop();
		delayMs(5); /*oczekiwanie na zapis*/
		}

/*Funkcje dedykowane do obslugi pamieci EERAM PIC 47L04 lub 47L16*/
/*zapis danych do rejestru pamieci EERAM*/
void EERAM_write_STATUS_REGISTER(uint8_t adr_device, uint16_t subAddr,  unsigned char i2c_data) {

		i2c_start();
		i2c_write(adr_device); /*wyslanie bajtu adresu z bitem R/W ustawionym na zero*/
		i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
		i2c_write(i2c_data);
		i2c_stop();
	    delayMs(1); /*oczekiwanie na zapis do STATUS REGISTER*/
}

/*zapis danych do pamieci EERAM - tryb sekwencyjny czyli ciag bajtow*/
void EERAM_write_buf(uint8_t adr_device, uint16_t subAddr,  uint16_t len, char *buf) {
        i2c_start();
        i2c_write(adr_device); /*wyslanie bajtu adresu z bitem R/W ustawionym na zero*/
        i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
        i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
        while (len--) {
        /*inkrementujemy tylko dane,adres jest inkrementowany automatycznie przez EERAM*/
            i2c_write(*buf++); 
            }
        i2c_stop();
     }
/*zapis danych do pamieci EERAM - tryb pojedynczego bajtu*/
void EERAM_write_byte(uint8_t adr_device, uint16_t subAddr,  unsigned char i2c_data) {
	
		i2c_start();
		i2c_write(adr_device); /*wyslanie bajtu adresu z bitem R/W ustawionym na zero*/
		i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
		i2c_write(i2c_data);
		i2c_stop();
	}
/*odczyt jednego bajtu z pamieci EERAM.*/
unsigned char EERAM_read_byte(uint8_t adr_device, uint16_t subAddr) {
        unsigned char i2c_data;
        i2c_start();
		i2c_write(adr_device); /*wyslanie adresu urzadzenia z bitem R/W w stanie niskim*/
        i2c_write((subAddr & 0xFF00) >> 8) ; /*wyslanie starszego bajtu adresu pamieci*/
		i2c_write(subAddr & 0x00FF)  ;       /*wyslanie mlodszego bajtu adresu pamieci*/
        i2c_start();
		i2c_write(adr_device + 1);/*zapisuje adres urzadzenia z bitem R/W ustawionym na 1 czyli o 1 zwiekszamy adres urzadzenia*/
		i2c_ack();
        i2c_data = i2c_read();
        i2c_nack();
        i2c_stop();
        return(i2c_data);
}

/*funkcje do zapisu i odczytu struktur*/
/*zapis danych do pamieci EERAM - tryb sekwencyjny czyli ciag bajtow*/
void EERAM_write_structure(uint8_t adr_device, uint16_t subAddr,  uint16_t len, void *struktura) {
        /*tworzymy wskaznik ktory wskazuje na pierwszy element struktury zrzutowanej do typu uint8_t*/
        uint8_t *wsk = (uint8_t*)struktura;
        i2c_start();
        i2c_write(adr_device); /*wyslanie bajtu adresu z bitem R/W ustawionym na zero*/
        i2c_write((subAddr & 0xFF00) >> 8);  /*wyslanie starszego bajtu adresu pamieci*/
        i2c_write(subAddr & 0x00FF);		 /*wyslanie mlodszego bajtu adresu pamieci*/
        while (len--) {
        /*inkrementujemy tylko dane,adres jest inkrementowany automatycznie przez EERAM*/
            i2c_write(*wsk++); 
            }
        i2c_stop();
     }
    
/*odczyt sekwencyjny danych z pamieci EERAM.*/
void EERAM_sequential_read_structure(uint8_t adr_device, uint16_t subAddr, uint16_t len, void *struktura) {
        /*tworzymy wskaznik ktory wskazuje na pierwszy element struktury zrzutowanej do typu uint8_t*/
        uint8_t *wsk = (uint8_t*)struktura;
        i2c_start();
        i2c_write(adr_device); /*wyslanie adresu urzadzenia z bitem R/W w stanie niskim*/
        i2c_write((subAddr & 0xFF00) >> 8) ; /*wyslanie starszego bajtu adresu pamieci*/
        i2c_write(subAddr & 0x00FF)  ;       /*wyslanie mlodszego bajtu adresu pamieci*/
        i2c_start();
        i2c_write(adr_device + 1);/*zapisuje adres urzadzenia z bitem R/W ustawionym na 1 czyli o 1 zwiekszamy adres urzadzenia*/
        i2c_ack();
        while(len--) {
        /*inkrementujemy tylko dane,adres jest inkrementowany automatycznie przez EERAM*/
        *wsk++ = i2c_read();
        i2c_ack();
        }
        i2c_nack();
        i2c_stop();
    }