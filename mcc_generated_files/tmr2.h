/**
  TMR2 Generated Driver API Header File 

  
        Compiler          :  XC32 v1.44
        MPLAB 	          :  MPLAB X v4.05
*/



#ifndef _TMR2_H
#define _TMR2_H

/**
  Section: Included Files
*/

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h> /*dyrektywy do uint8_t itp*/


extern volatile uint8_t TimerA_Programowy , TimerB_Programowy ;

void TMR2_Initialize (void);
void TMR2_Start( void );
void TMR2_Stop( void );


#endif //_TMR2_H