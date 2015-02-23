/*
 * File name            : main.c
 * Compiler             : MPLAB XC8/ MPLAB C18 compiler
 * IDE                  : Microchip  MPLAB X IDE v1.90/ MPLAB IDE v8
 * Author               : Etiq Technologies
 * Processor            : PIC18
 * Description          : Example_12.2 Relay control through UART
 * Created on           : January 03, 2014, 10:17 AM
 *                      : Example code for Controlling a Power device (Resistive/ Inductive Load) through UART.
*/

/*
 * PORT connections
 * ----------------
 * 1. PORTC
 *      J2(Motor relay board)            -  J15(Base board)
 *
 * 2. USART
 *      J21 DB9 connenctor of PIC board  -  Computer serial port
 *
 * 3. Insert a jumper for 0th bit of J4 on Motor relay board.
 */

/*  OPERATION
    ---------
 * 1. Open PC Hyper Terminal. Reset the microcontroller.
 * 2. Select an option from the choice to ON/ OFF the device.
 * 3. Reading the user input from the USART, microcontroller sets or resets bit RB0 to switch ON or OFF
 *    the relay connected to it depending on user input.
 */

#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File 		*/
    #include <plib/usart.h>
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File 	*/
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File 		*/
    #include <usart.h>
#endif

#if defined(__XC) || defined(HI_TECH_C)
#include <stdint.h>        /* For uint8_t definition 		*/
#include <stdbool.h>       /* For true/false definition 	*/
#endif

#if defined(__XC) || defined(__18CXX)
#include "config.h"
#include "delay.h"
#include "uart.h"
#endif

#define _XTAL_FREQ 20000000
#define baud 9600                                                  /* UART baud rate                                       */
#define config2 ((_XTAL_FREQ/baud/64)-1)
#define OUTPUT PORTCbits.RC0                                       /* bit RC0 is defined as output, fed to Relay circuit   */
#define OUTPUT_DIR TRISCbits.TRISC0
unsigned char input = 0, str[45] = "Press 1 to switch ON\n\rPress 0 to switch OFF\n\r";
unsigned char config1 = 0x00, c;

/*______________________________________  Device Switching Function   ___________________________________________________*/

void Switch(void)
    {
    input = Read();                                                 /* Read the Input from serial port                   */
    switch(input)
        {
        case '1' : OUTPUT = 1;                                      /* Switch ON   RC0 = 1                               */
        break;
        default  : OUTPUT = 0;                                      /* Switch OFF  RC0 = 0                               */
        break;
        }
    }

/*_____________________________________ C18 compiler ISR __________________________________________________*/

#if defined(__18CXX)

void USART_Rx (void); 									/* prototype for 'goto' specified function	 			 */

#pragma interruptlow USART_Rx
void USART_Rx (void)
{
 if(PIR1bits.RCIF == 1)                                 /* Test for USART receive interrupt is set or not        */
   {
	Switch();                                           /* Invoking Switching function                           */
   }
PIR1bits.RCIF = 0;                                      /* clearing USART receive flag                           */
}

#pragma code low_vector_section=0x08  					/* control is transferred to address 0x08				 */
void low_vector (void)									
{
_asm
GOTO USART_Rx
_endasm
}
#pragma code 											/* allow the linker to locate the remaining code 		 */

void ei(void)
{
INTCONbits.GIE = 1;										/* Global interrupt enable								 */
}
#elif defined(__XC)
/*______________________________________  Interrupt Service Routine   ____________________________________________________*/

#pragma interrupt_level 0 @0x0018
void interrupt Rx_isr()                                          /* Interrupt service routine for USART receive interrupt */
{
   if(PIR1bits.RCIF == 1)                                        /* Test for USART receive interrupt is set or not        */
   {
   Switch();                                                     /* Invoking Switching function                           */
   }
   if(OUTPUT)                                                    /* Testing bit RC0 is set or not                         */
   Write_str("ON ");                                              /* Writing a string through UART module if RB0 set       */
   else
   Write_str("OFF");                                             /* Writing a string through UART module if RB0 not set   */
   PIR1bits.RCIF = 0;                                            /* clearing USART receive flag                           */
}

#endif

void main(void)
{
    config1 = (USART_TX_INT_OFF  & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_LOW);
    OUTPUT_DIR = 0;                                             /* Direction of bit RB0 as output                       */
    OUTPUT = 0;                                                 /* Resets bit RC0                                       */
TRISDbits.TRISD0 = 0;
LATDbits.LATD0 = 0;     
Config_Interrupt_Rx();
    UsartConfig(config1, config2);                              /* Invoking UART Configure function                     */
    Write_str(str);                                             /* Writing string to display the choice                 */
    ei();                                                       /* Global Interrupt Enable (INTCONbits.GIE = 1;)        */
while(1);
                                                  /* Infinite loop                                        */
}