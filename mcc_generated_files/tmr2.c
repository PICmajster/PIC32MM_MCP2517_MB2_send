
/**
  TMR2 Generated Driver API Source File 

        Compiler          :  XC32 v2,05
        MPLAB 	          :  MPLAB X v4.05
*/


#include <xc.h>
#include "tmr2.h"

volatile uint8_t TimerA_Programowy , TimerB_Programowy ;

void TMR2_Initialize (void)
{
    uint32_t tcon_value = 0x00000000;
    //  TCKPS 1:16; T32 16 Bit; TCS PBCLK; SIDL disabled; TGATE disabled; ON enabled; 
    T2CON = 0x8040;   
    tcon_value = 0x8040;  // Temporary storage of value
    T2CONCLR = _T2CON_ON_MASK;  // disable Timer, before loading the period/counter value
    // Period = 0.025 s; Frequency = 24000000 Hz; PR2 37500; 
    PR2 = 0x927C ;

    T2CON = tcon_value;//restore the TCON value
    IFS0CLR= 1 << _IFS0_T2IF_POSITION;
    
}

void __attribute__ ((vector(_TIMER_2_VECTOR), interrupt(IPL3SOFT))) TMR2_ISR()
{
    uint8_t x;
    x = TimerA_Programowy ;
    if (x) TimerA_Programowy = --x ;
    x = TimerB_Programowy ;
    if (x) TimerB_Programowy = --x ;
    /*zerowanie flagi przerwania*/
    IFS0CLR= 1 << _IFS0_T2IF_POSITION;
}


void TMR2_Start( void )
{
    /*Enable the interrupt*/
    IEC0bits.T2IE = true;
    /* Start the Timer */
    T2CONSET = _T2CON_ON_MASK;
}

void TMR2_Stop( void )
{
    /* Stop the Timer */
    T2CONCLR = _T2CON_ON_MASK;
    /*Disable the interrupt*/
    IEC0bits.T2IE = false;
}