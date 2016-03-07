/*
Example for SoftwareUart
by Mark Cooke 5 Mar 2016 https://github.com/micooke/SoftwareUart

Compile size examples
+------------+---------+--------+------+
|  microchip | SU_MODE | Flash  | SRAM |
+------------+---------+--------+------+
| ATmega328p | 0:TX&RX | 3,670b | 316b |
|            | 1:TX    | 2,672b | 216b |
|            | 2:RX    | 3,274b | 310b |
+------------+---------+--------+------+
*/

#define SU_MODE 0
//#define SU_DUPLEX 0
//#define SU_TX_ONLY 1
//#define SU_RX_ONLY 2

#include <SoftwareUart.h>
#if (SU_MODE == SU_TX_ONLY)
SoftwareUart uart(8, 9);
#else
SoftwareUart<> uart(8,9);
#endif

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
#if (SU_MODE != SU_TX_ONLY)
ISR(PCINT0_vect)
{
	uart.handle_interrupt();
}
#endif

void setup()
{
   Serial.begin(9600);
   
   // Wait for the Serial connection - required for Leonardo/Micro only
   #ifdef __AVR_ATmega32U4__
   while (!Serial);
   #endif
   
   Serial.println(F("Goodnight moon!"));
   
   uart.begin(9600);
#if (SU_MODE != SU_TX_ONLY)
   uart.println(F("Hello, World!"));
#endif
}

void loop()
{
#if (SU_MODE != SU_TX_ONLY)
   // read from SoftwareUart on pin 8, write to hardware serial
   if (uart.available())
   Serial.write(uart.read());
#endif

#if (SU_MODE != SU_RX_ONLY)
   // read from hardware serial, write to SoftwareUart on pin 8
   if (Serial.available())
   uart.print(Serial.read());
#endif
}
