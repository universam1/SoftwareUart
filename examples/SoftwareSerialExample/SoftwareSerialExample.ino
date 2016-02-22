/*
Software serial multiple serial test

Receives from the hardware serial, sends to software serial.
Receives from software serial, sends to hardware serial.

The circuit:
* RX is digital pin 2 (connect to TX of other device)
* TX is digital pin 3 (connect to RX of other device)

created back in the mists of time
modified 9 Apr 2012
by Tom Igoe
based on Mikal Hart's example
modified for SoftwareSerialWithHalfDuplex
by Nick Stedman 15 July 2012 
modified for SoftwareUart
by Mark Cooke 10 Feb 2016 https://github.com/micooke/SoftwareUart

This example code is in the public domain.

*/
// Slightly leaner than it older brother
// SoftwareUart                 : Nano compile size = 4868 Flash, 454 SRAM
// SoftwareSerialWithHalfDuplex : Nano compile size = 3880 Flash, 348 SRAM
#include <SoftwareUart.h>
SoftwareUart<> uart(8,9);

// You need to roll your own interrupt for SoftwareUart
// Example interrupt routine for SoftwareUart on pin 8,8
// Note : make sure you know what pin interrupt bank your pin is attached to.
//
// Arduino uno/nano (ATmega328p)
// PCICR  = [   -   |   -   |   -   |   -   |   -   |  PCIF2|  PCIF1|  PCIF0]
// PCMSK2 = [PCINT23|PCINT22|PCINT21|PCINT20|PCINT19|PCINT18|PCINT17|PCINT16]
//          [     D7|     D6|     D5|     D4|     D3|     D2|     D1|     D0]
// PCMSK1 = [   -   |PCINT14|PCINT13|PCINT12|PCINT11|PCINT10|PCINT09|PCINT08]
//          [   -   |    RST|     A5|     A4|     A3|     A2|     A1|     A0]
// PCMSK0 = [ PCINT7| PCINT6| PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [  XTAL2|  XTAL1|    D13|    D11|    D12|    D10|     D9|     D8]
// Digispark (ATtiny85)
// GIMSK  = [   -   |  INT0 |  PCIE |   -   |   -   |   -   |   -   |   -   ]
//GIMSK = 0x00;
// PCMSK0 = [   -   |   -   | PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [   -   |   -   |    RST|    PB4|    PB3|    PB2|    PB1|    PB0]
// Arduino Micro (ATmega32u4)
// PCICR  = [   -   |   -   |   -   |   -   |   -   |   -   |   -   |  PCIE0]
// PCMSK0 = [ PCINT7| PCINT6| PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [    D12|    D11|    D10|     D9|   MISO|   MOSI|    SCK|     SS]
ISR(PCINT0_vect)
{
	uart.handle_interrupt();
}

void setup()
{
   Serial.begin(9600);
   
   // Wait for the Serial connection - required for Leonardo/Micro only
   #ifdef __AVR_ATmega32U4__
   while (!Serial);
   #endif
   
   Serial.println("Goodnight moon!");
   
   uart.begin(9600);
   uart.println("Hello, World!");
}

void loop()
{
   // read from SoftwareUart on pin 8, write to hardware serial
   if (uart.available())
   Serial.write(uart.read());
   
   // read from hardware serial, write to SoftwareUart on pin 8
   if (Serial.available())
   uart.write(Serial.read());
}
