/*
  Software serial multiple serial test

 Receives from the two software serial ports,
 sends to the hardware serial port.

 In order to listen on a software port, you call port.listen().
 When using two software serial ports, you have to switch ports
 by listen()ing on each one in turn. Pick a logical time to switch
 ports, like the end of an expected transmission, or when the
 buffer is empty. This example switches ports when there is nothing
 more to read from a port

 The circuit:
 Two devices which communicate serially are needed.
 * First serial device's TX attached to digital pin 2, RX to pin 3
 * Second serial device's TX attached to digital pin 4, RX to pin 5

 created 18 Apr. 2011
 modified 9 Apr 2012
 by Tom Igoe
 based on Mikal Hart's twoPortRXExample
 adapted by Nick Stedman 15 July 2012
 modified for SoftwareUart
 by Mark Cooke 10 Feb 2016 https://github.com/micooke/SoftwareUart

 This example code is in the public domain.

 */
 // SoftwareUart                 : Nano compile size = 4054 Flash, 456 SRAM
#include <SoftwareUart.h>
#if defined(__AVR_ATtiny25__) | defined(__AVR_ATtiny45__) | defined(__AVR_ATtiny85__)
// PCMSK0 = [   -   |   -   | PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [   -   |   -   |    RST|    PB4|    PB3|    PB2|    PB1|    PB0]
SoftwareUart<> portOne(0, 1); // RX, TX
SoftwareUart<> portTwo(2, 3); // RX, TX
#elif defined(__AVR_ATmega16U4__) | defined(__AVR_ATmega32U4__)
// PCMSK0 = [ PCINT7| PCINT6| PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [    D12|    D11|    D10|     D9|   MISO|   MOSI|    SCK|     SS]
SoftwareUart<> portOne(9, 10); // RX, TX
SoftwareUart<> portTwo(11, 12); // RX, TX
#elif defined(__AVR_ATmega168__) | defined(__AVR_ATmega168P__) | defined(__AVR_ATmega328P__)
// PCMSK0 = [ PCINT7| PCINT6| PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
//          [  XTAL2|  XTAL1|    D13|    D11|    D12|    D10|     D9|     D8]
SoftwareUart<> portOne(9, 10); // RX, TX
SoftwareUart<> portTwo(11, 12); // RX, TX
#else
#error Board not supported - please inform the author https://github.com/micooke/SoftwareUart
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
#define MAX_LINE_LENGTH 3*14
uint32_t line_length = MAX_LINE_LENGTH;

ISR(PCINT0_vect)
{
	// Note : handle_interrupt has internal logic (below) to determine if the 
	// pin change interrupt was for itself
	// 
	//     Inv  notInv
	//low   Us   -
	//high  -    Us
	portOne.handle_interrupt();
	portTwo.handle_interrupt();
}

void setup()
{
	// Open serial communications and wait for port to open:
	Serial.begin(9600);

	// Wait for the Serial connection - required for Leonardo/Micro only
#ifdef __AVR_ATmega32U4__
	while (!Serial);
#endif

	// Start each software serial port
	portOne.begin(9600);
	portTwo.begin(9600);

	Serial.println("Data from port one = [x]\nData from port two = (x)");
}

void loop()
{
	// while there is data coming in, read it and send to the hardware serial port:
	uint16_t data_available = portOne.available() + portTwo.available();
	char c;
	while (data_available > 0)
	{
		if (portOne.available())
		{
			c = portOne.read();
			Serial.write('[');
			if ((c != '\n') & (c != '\r'))
			{
				Serial.write(c);
				line_length -= 3;
			}
			else if (c == '\r')
			{
				Serial.write("\\r");
				line_length -= 3;
			}
			else
			{
				Serial.write("\\n");
				line_length = 0;
			}
			Serial.write(']');
		}
		else
		{
			c = portTwo.read();
			Serial.write('(');
			if ((c != '\n') & (c != '\r'))
			{
				Serial.write(c);
				line_length -= 3;
			}
			else if (c == '\r')
			{
				Serial.write("\\r");
				line_length -= 3;
			}
			else
			{
				Serial.write("\\n");
				line_length = 0;
			}
			Serial.write(')');
		}

		if (line_length == 0)
		{
			line_length = MAX_LINE_LENGTH;
			Serial.println();
		}
		data_available = portOne.available() + portTwo.available();
	}
}