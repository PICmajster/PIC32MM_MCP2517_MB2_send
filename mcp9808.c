/*
    File Name:        :  mcp9808.c

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/
#include "mcc_generated_files/mcc.h"
#include "xc.h" /* wykrywa rodzaj procka i includuje odpowiedni plik naglowkowy "pic32mm0256gpm048.h"*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h> /*definicje uint8_t itp*/
#include "delay.h"
#include "dogm162.h" /*wyswietlacz*/
#include "mcp9808.h"

 uint8_t UpperByte ;
 uint8_t LowerByte ;
 float Temperature ;
 char Temperature_bufor[5] ;/*bufor do konwersji z float na char*/
 
void read_Temp(void) {
    
    /*komunikacja i odczyt temperatury MCP9808*/
    i2c_start();
    i2c_write(MCP9808_ADDRESS & 0xFE); /*wyslij bajt z adresem i bitem R/W ustawionym na 0*/
    i2c_write(MCP9808_REG_AMBIENT_TEMP); /*ustawiamy rejestr z ktorym chcemy gadac*/
    i2c_restart();
    i2c_write(MCP9808_ADDRESS | 0x01); /*wyslij bajt z adresem i bitem R/W ustawionym na 1*/
    UpperByte = i2c_read();
    i2c_ack();
    LowerByte = i2c_read();
    i2c_nack();
    i2c_stop(); /*THE END transmission with MCP9808*/
    /*Convert the temperature data*/
    /*First check flag bits*/
    if ((UpperByte & 0x80) == 0x80){/*tutaj kod do obslugi zdarzenia*/}   //TA >= TCRIT
    if ((UpperByte & 0x40) == 0x40){/*tutaj kod do obslugi zdarzenia*/}   //TA > TUPPER
    if ((UpperByte & 0x20) == 0x20){/*tutaj kod do obslugi zdarzenia*/}   //TA < TLOWER
    
    UpperByte = UpperByte & 0x1F;                //Clear flag bits
    if ((UpperByte & 0x10) == 0x10){             //TA < 0°C
    UpperByte = UpperByte & 0x0F;                //Clear SIGN
    Temperature = 256 - ((float)UpperByte * 16 + (float)LowerByte / 16); 
    } else                                       
    Temperature = ((float)UpperByte * 16 + (float)LowerByte / 16); //TA >= 0°C
    
    /*tu zerujmy bufor*/
    memset(Temperature_bufor, ' ', 5); /*reset*/
    
    /*Temperature = Ambient Temperature (°C)*/
    /*conversion of float to string and saving the result to a buffer*/
    sprintf(Temperature_bufor,"%2.1f",Temperature);
    /*wyswietlamy temperature na LCD*/
    lcd_Locate(2,8);
    lcd_String(Temperature_bufor);
    lcd_String("\x01""C");/*wyswietl znak stopnia plus literka C*/
  
          
}