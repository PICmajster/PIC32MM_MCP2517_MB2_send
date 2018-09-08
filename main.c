/*
    File Name:        :  main.c

    Device            :  PIC32MM0256GPM048
    Compiler          :  XC32 2.05
    MPLAB             :  MPLAB X 4.15
    Created by        :  http://strefapic.blogspot.com
*/

#include <string.h>
#include "mcc_generated_files/mcc.h"
#include "delay.h"
#include "dogm162.h"
#include "drv_canfdspi_api.h"
#include "drv_canfdspi_defines.h"
#include "drv_canfdspi_register.h"
#include "mcp9808.h"

#define LED1_TOG PORTB ^= (1<<_PORTB_RB6_POSITION) /*changes the bit state to the opposite*/
#define cREGADDR_CiFIFOUA1   0x64
#define cREGADDR_CiFIFSTA1   0x60


void TransmitMessageLoad(char *msg);
        
CAN_TX_FIFO_EVENT         txFlags;
CAN_TX_MSGOBJ             txObj;
uint8_t                   txData[MAX_DATA_BYTES];
bool                      flush;

     int main(void)
{
    /*initialize the device*/
    SYSTEM_Initialize();
    DRV_CANFDSPI_MY_Configure();
    lcd_Initialize();
        
    WpiszSwojeZnaki(); /*definjujemy znak stopien celciusza czyli male koleczko*/
    lcd_Locate(1,1);
    lcd_String("EFMSGCNT"); /*free error message counter*/
     
   
    TMR2_Start();
       
    while (1)
    {
        if(!TimerA_Programowy) {
              TimerA_Programowy = 60 ; /*Timer2 sprzetowy x TimerA_Programowy = 25ms x 10 = 250ms*/       
              //read_Temp(); /*read from MCP9808 temperature and display it on LCD*/
         
   
    TransmitMessageLoad("TEST"); /*send temperature Temperature_bufor*/
           //       Read data back from registers
  
            
      
              LED1_TOG;
                                          
          }
      

    }
}


 void TransmitMessageLoad( char *msg )
 {
          
     flush = true;
     txObj.word[0] = 0;
     txObj.word[1] = 0;
     
     txObj.bF.id.SID = 0x300; /*standard or Base ID*/
     txObj.bF.id.EID = 0;
     
     txObj.bF.ctrl.FDF = 0; /*CAN 2.0 frame*/
     txObj.bF.ctrl.BRS = 0; /*switch bit rate for CAN FD*/
     txObj.bF.ctrl.IDE = 0; /*standard frame*/
     txObj.bF.ctrl.RTR = 0; /*not a remote frame request*/
     txObj.bF.ctrl.DLC = CAN_DLC_8; /*8 data bytes*/
     txObj.bF.ctrl.SEQ = 1; /*sequence: doesn't get transmitted but will be stored in TEF*/
     
//     memset(txData, 0, MAX_DATA_BYTES); /*reset*/
//     strcpy(txData, msg);
            
  //Read data back from registers
    uint8_t rxd[4];
    DRV_CANFDSPI_ReadByteArray(cREGADDR_CiBDIAG1 , rxd, 4);
    lcd_Locate(2,1);
    lcd_Integer(rxd[0]+rxd[1]);
//    lcd_String(":");
//    lcd_Integer(rxd[1]);
//    lcd_String(":");
//    lcd_Integer(rxd[2]);
//    lcd_String(":");
//    lcd_Integer(rxd[3]);
   
//      uint8_t rxd[72];
//    // Read data back from RAM
//    DRV_CANFDSPI_ReadByteArray((cRAMADDR_START), rxd, 72);
//    lcd_Locate(2,1);
//   
//    lcd_Integer(rxd[4]);
//    lcd_String(":");
//    lcd_Integer(rxd[5]);
//    lcd_String(":");
//    lcd_Integer(rxd[6]);
//    lcd_String(":");
//    lcd_Integer(rxd[7]);
       
    DRV_CANFDSPI_TransmitChannelLoad(CAN_FIFO_CH1, &txObj, msg, DRV_CANFDSPI_DlcToDataBytes(txObj.bF.ctrl.DLC), true); 
    
    
        
    
     
      
 }



//  /* RAM access test */
//  void MCP2517FD_TEST_RAM_ACCESS(void)
//        {
//            // Variables
//            uint8_t txd[MAX_DATA_BYTES];
//            uint8_t rxd[MAX_DATA_BYTES];
//            uint8_t i;
//            uint8_t length;
//
//            // Verify read/write with different access length
//            // Note: RAM can only be accessed in multiples of 4 bytes
//            for (length = 4; length <= MAX_DATA_BYTES; length += 4) {
//                for (i = 0; i < length; i++) {
//                    txd[i] = rand() & 0xff;
//                    rxd[i] = 0xff;
//                }
//
//                // Write data to RAM
//                DRV_CANFDSPI_WriteByteArray(cRAMADDR_START, txd, length);
//
//                // Read data back from RAM
//                DRV_CANFDSPI_ReadByteArray(cRAMADDR_START, rxd, length);
//
//                // Verify
//                bool good = false;
//                for (i = 0; i < length; i++) {
//                    good = txd[i] == rxd[i];
//
//                    if (!good) {
//                       PORTBbits.RB6 = 1; /*on LED if inconformity tx and rx data*/
//
//                        // Data mismatch
//                    }
//                }
//            }
//
//           
//        }

            /* Register access test */
//   void MCP2517_TEST_REGISTER_ACCESS(void)
//        {
//            // Variables
//            uint8_t txd[MAX_DATA_BYTES];
//            uint8_t rxd[MAX_DATA_BYTES];
//            uint8_t i;
//            uint8_t length;
//
//            // Verify read/write with different access length
//            // Note: registers can be accessed in multiples of bytes
//            for (length = 1; length <= MAX_DATA_BYTES; length++) {
//                for (i = 0; i < length; i++) {
//                    txd[i] = rand() & 0x7f; // Bit 31 of Filter objects is not implemented , txd[i] <= 127
//                    rxd[i] = 0xff;
//                }
//                // Write data to registers
//                DRV_CANFDSPI_WriteByteArray(cREGADDR_CiFLTOBJ, txd, length);
//
//                // Read data back from registers
//                DRV_CANFDSPI_ReadByteArray(cREGADDR_CiFLTOBJ, rxd, length);
//
//                // Verify
//                bool good = false;
//                for (i = 0; i < length; i++) {
//                    good = txd[i] == rxd[i];
//
//                    if (!good) {
//                       PORTBbits.RB6 = 1; /*on LED if inconformity tx and rx data*/
//
//                        // Data mismatch
//                    }
//                }
//            }
//
//            Nop();
//            Nop();
//         
//        }
  
//   void MCP2517_TEST_REGISTER_ACCESS(void)
//        {
//         /*the data are entered and read from the CiFLTCON0 register, address 0x1D0*/  
//         /*the register shall consist of 4 bytes = 32 bytes*/
//          uint8_t length , i;
//          uint8_t txd[MAX_DATA_BYTES];
//          uint8_t rxd[MAX_DATA_BYTES];
//           
//          /* We fill in the buffers with data*/
//          for (length = 0; length < 4; length++) {
//            txd[length] = foo.byte ; /*powielamy 4 razy bajt bo rejestr ma 32bity */
//            rxd[length] = 0xff; /*wypelniamy wartoscia*/
//             }
//          // Write data to registers
//           DRV_CANFDSPI_WriteByteArray(cREGADDR_CiFLTCON, txd, length);
//
//          // Read data back from registers
//           DRV_CANFDSPI_ReadByteArray(cREGADDR_CiFLTCON, rxd, length);
//
//               
//           PORTBbits.RB6 = 0; /*zgas LED*/
//           /*we verify the correctness of the data stored in the CAN controller and received*/
//           _Bool good = false ;
//           for(i=0 ; i < length; i++ ) {
//               good = txd[i] == rxd[i] ;
//               if(!good) PORTBbits.RB6 = 1; /*LED lighting if the data sent and received do not match*/
//                 
//             } 
//            }
  