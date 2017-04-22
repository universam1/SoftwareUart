/*
SoftwareUart (adapted from SoftwareSerialWithHalfDuplex)

This library is based off SoftwareSerial from Arduino 1.6.5r5, with half-duplex changes
from @nickstedman's SoftwareSerialWithHalfDuplex library.

Modifications by @micooke to reduce codespace, and template it to allow changing the rx buffer.

[SoftwareSerialWithHalfDuplex] (https://github.com/nickstedman/SoftwareSerialWithHalfDuplex)
is a simple modification that should definitely be rolled into the SoftwareSerial core library.

This library is an attempt to add greater transparency and useability to SoftwareSerial, but
it requires the user to setup a few more things, namely an interrupt routine.

In short the differences to SoftwareSerial are/will be:
* templated - change the buffer size without changing the library!
* HalfDuplex - taken from [SoftwareSerialWithHalfDuplex] (https://github.com/nickstedman/SoftwareSerialWithHalfDuplex)
* flush() call to cli() to stop interrupts was removed (cli() inside write() is needed unfortunately)
* Removed the static functions which are there to enable time sharing of the pinchange interrupt
  (used to recv() serial data) - your interrupt routine can handle the time sharing! If you dont
  know what i mean, see how 'active_object' was used. In short the new implementation does not block
  a pinchange interrupt on bank 0 from blocking pinchange interrupts on every other bank. There
  will always be interrupt scheduling conflicts, but now the user can manage the conflicts!
* Removed/exposed the pin change interrupts so that developers can make use of them

I do not claim copyright on the code.
This is adapted from : NewSoftSerial, SoftwareSerial and SoftwareSerialWithHalfDuplex

----
SoftwareSerialWithHalfDuplex.h (formerly SoftwareSerial.h) -
Multi-instance software serial with half duplex library for Arduino/Wiring

By default the library works the same as the SoftwareSerial library,
but by adding a couple of additional arguments it can be configured for
half-duplex. In that case, the transmit pin is set by default to an input,
with the pull-up set. When transmitting, the pin temporarily switches to
an output until the byte is sent, then flips back to input. When a module
is receiving it should not be able to transmit, and vice-versa.
This library probably won't work as is if you need inverted-logic.

This is a first draft of the library and test programs. It appears to work,
but has only been tested on a limited basis. The library also works with
Robotis Bioloid AX-12 motors. Seems fairly reliable up to 57600 baud.
As with all serial neither error checking, nor addressing are implemented,
so it is likely that you will need to do this yourself. Also, you can make
use of other protocols such as i2c. I am looking for any feedback, advice
and help at this stage. Changes from SoftwareSerial have been noted with a
comment of "//NS" for your review. Only a few were required.
Contact me at n.stedman@steddyrobots.com, or on the arduino forum.
----
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) -
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SoftwareUart_h
#define SoftwareUart_h

#ifndef SU_MODE
#define SU_MODE 0
#endif

#define SU_DUPLEX 0
#define SU_TX_ONLY 1
#define SU_RX_ONLY 2

#include <inttypes.h>
#include <Arduino.h>

#ifdef __AVR__
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay_basic.h>
#elif defined(ESP8266)
#include <interrupts.h>
#include <pgmspace.h>
#elif defined(ARDUINO_ARCH_SAMD)
// nothing special needed
#elif defined(ARDUINO_SAM_DUE)
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif

/******************************************************************************
* Definitions
******************************************************************************/

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif
#if (SU_MODE != SU_TX_ONLY)
#include <Stream.h>
template <uint8_t _SU_RX_BUFFER = 64>
class SoftwareUart : public Stream
#else
class SoftwareUart
#endif
{
private:
	// per object data
#if (SU_MODE != SU_TX_ONLY)
	uint8_t _rxPin;
	uint8_t _rxBitMask;
	volatile uint8_t *_rxPort;

	volatile uint8_t *_pcint_maskreg;
	uint8_t _pcint_maskvalue;

	// Expressed as 4-cycle delays (must never be 0!)  
	uint16_t _rx_delay_centering;
	uint16_t _rx_delay_intrabit;
	uint16_t _rx_delay_stopbit;

	bool _buffer_overflow;

	char _receive_buffer[_SU_RX_BUFFER];
	volatile uint8_t _receive_buffer_tail;
	volatile uint8_t _receive_buffer_head;

	// private methods
	void recv();
	uint8_t rx_pin_read() { return *_rxPort & _rxBitMask; }
	void setRxIntMsk(bool enable);

#endif
#if (SU_MODE != SU_RX_ONLY)
	uint8_t _txPin;								//NS Added
	uint8_t _txBitMask;
	volatile uint8_t *_txPort;

	uint16_t _tx_delay;
#endif

	bool _inverse_logic;
	bool _full_duplex;							//NS Added

	// Return num - sub, or 1 if the result would be < 1
	//static uint16_t subtract_cap(uint16_t num, uint16_t sub);
	uint16_t subtract_cap(uint16_t num, uint16_t sub);

	// private static method for timing
	//static inline void tunedDelay(uint16_t delay);
#ifdef __AVR__
	inline void tunedDelay(uint16_t delay) { _delay_loop_2(delay); }
#else
	inline void tunedDelay(uint16_t delay)
	{
		uint32_t delay_us = (4000000 / F_CPU);
		delay_us *= delay;
		delayMicroseconds(delay_us);
	}
#endif

public:
	// public methods
	SoftwareUart(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);
	~SoftwareUart();
	bool inverse_logic() { return _inverse_logic; }
	void begin(long speed);

#if (SU_MODE != SU_TX_ONLY)
	bool listen() { _buffer_overflow = _receive_buffer_head = _receive_buffer_tail = 0; setRxIntMsk(true); return false; }
	bool stopListening() { setRxIntMsk(false); return true; }
	bool overflow() { bool ret = _buffer_overflow; if (ret) { _buffer_overflow = false; } return ret; }
	int16_t peek();
	virtual int16_t read();
	virtual int16_t available();
	virtual void flush() { _receive_buffer_head = _receive_buffer_tail = 0; }

	// public only for easy access by interrupt handlers
	inline void handle_interrupt() __attribute__((__always_inline__)) { recv(); };
#endif

#if (SU_MODE == SU_RX_ONLY)
	virtual size_t write(uint8_t byte) { return 0; };
#elif (SU_MODE != SU_RX_ONLY)
	virtual size_t write(uint8_t byte);
	//using Print::write;
#endif
};

//
// Private methods
//

#if (SU_MODE != SU_TX_ONLY)
//
// The receive routine called by the interrupt handler
//
template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::recv()
{
#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Preserve the registers that the compiler misses
	// (courtesy of Arduino forum user *etracer*)
	asm volatile(
		"push r18 \n\t"
		"push r19 \n\t"
		"push r20 \n\t"
		"push r21 \n\t"
		"push r22 \n\t"
		"push r23 \n\t"
		"push r26 \n\t"
		"push r27 \n\t"
		::);
#endif

	uint8_t d = 0;

	// If RX line is high, then we don't see any start bit
	// so interrupt is probably not for us
	if (_inverse_logic ? rx_pin_read() : !rx_pin_read())
	{
		// Disable further interrupts during reception, this prevents
		// triggering another interrupt directly after we return, which can
		// cause problems at higher baudrates.
		setRxIntMsk(false);

		// Wait approximately 1/2 of a bit width to "center" the sample
		tunedDelay(_rx_delay_centering);

		// Read each of the 8 bits
		for (uint8_t i = 8; i > 0; --i)
		{

			tunedDelay(_rx_delay_intrabit);
			d >>= 1;
			if (rx_pin_read())
				d |= 0x80;
		}

		if (_inverse_logic)
			d = ~d;

		// if buffer full, set the overflow flag and return
		uint8_t next = (_receive_buffer_tail + 1) % _SU_RX_BUFFER;
		if (next != _receive_buffer_head)
		{

			// save new data in buffer: tail points to where byte goes
			_receive_buffer[_receive_buffer_tail] = d; // save new byte
			_receive_buffer_tail = next;
		}
		else
		{
			_buffer_overflow = true;
		}

		// skip the stop bit
		tunedDelay(_rx_delay_stopbit);

		// Re-enable interrupts when we're sure to be inside the stop bit
		setRxIntMsk(true);
	}

#if GCC_VERSION < 40302
	// Work-around for avr-gcc 4.3.0 OSX version bug
	// Restore the registers that the compiler misses
	asm volatile(
		"pop r27 \n\t"
		"pop r26 \n\t"
		"pop r23 \n\t"
		"pop r22 \n\t"
		"pop r21 \n\t"
		"pop r20 \n\t"
		"pop r19 \n\t"
		"pop r18 \n\t"
		::);
#endif
}
#endif

#if (SU_MODE != SU_TX_ONLY)
template <uint8_t _SU_RX_BUFFER>
uint16_t SoftwareUart<_SU_RX_BUFFER>::subtract_cap(uint16_t num, uint16_t sub)
#else
uint16_t SoftwareUart::subtract_cap(uint16_t num, uint16_t sub)
#endif
{
	if (num > sub)
		return num - sub;
	else
		return 1;
}

//
// Public methods
//
//
// Constructor
//
#if (SU_MODE != SU_TX_ONLY)
template <uint8_t _SU_RX_BUFFER>
SoftwareUart<_SU_RX_BUFFER>::SoftwareUart(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */)
#else
SoftwareUart::SoftwareUart(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic /* = false */)
#endif
{
	_full_duplex = (receivePin != transmitPin);
	_inverse_logic = inverse_logic;

#if (SU_MODE != SU_RX_ONLY)
	_tx_delay = 0;
	_txPin = transmitPin;
	_txBitMask = digitalPinToBitMask(transmitPin);
	_txPort = portOutputRegister(digitalPinToPort(transmitPin));
	pinMode(_txPin, OUTPUT);

	// First write, then set output. If we do this the other way around,
	// the pin would be output low for a short while before switching to
	// output high. Now, it is input with pullup for a short while, which
	// is fine. With inverse logic, either order is fine.
	digitalWrite(_txPin, _inverse_logic ? LOW : HIGH);

	if (_full_duplex == false)
	{
		pinMode(_txPin, INPUT);
	}
#endif

#if (SU_MODE != SU_TX_ONLY)
	_buffer_overflow = false;

	_rx_delay_centering = 0;
	_rx_delay_intrabit = 0;
	_rx_delay_stopbit = 0;

	_rxPin = receivePin;
	_rxBitMask = digitalPinToBitMask(receivePin);
	_rxPort = portOutputRegister(digitalPinToPort(receivePin));

	pinMode(_rxPin, INPUT);
	if (!_inverse_logic) { digitalWrite(_rxPin, HIGH); } // pullup for normal logic
#endif
}

//
// Destructor
//
#if (SU_MODE != SU_TX_ONLY)
template <uint8_t _SU_RX_BUFFER>
SoftwareUart<_SU_RX_BUFFER>::~SoftwareUart()
#else
SoftwareUart::~SoftwareUart()
#endif
{
#if (SU_MODE != SU_TX_ONLY)
	stopListening();
#endif
}

#if (SU_MODE != SU_TX_ONLY)
template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::begin(long speed)
#else
void SoftwareUart::begin(long speed)
#endif
{
#if (SU_MODE != SU_TX_ONLY)
	_rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = 0;
#endif
	// Precalculate the various delays, in number of 4-cycle delays
	uint16_t bit_delay = (F_CPU / speed) / 4;

	// 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
	// 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
	// 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
	// These are all close enough to just use 15 cycles, since the inter-bit
	// timings are the most critical (deviations stack 8 times)
#if (SU_MODE != SU_RX_ONLY)
	_tx_delay = subtract_cap(bit_delay, 15 / 4);
#endif

#if (SU_MODE != SU_TX_ONLY)
	// Only setup rx when we have a valid PCINT for this pin
	if (digitalPinToPCICR(_rxPin)) {
#if GCC_VERSION > 40800
		// Timings counted from gcc 4.8.2 output. This works up to 115200 on
		// 16Mhz and 57600 on 8Mhz.
		//
		// When the start bit occurs, there are 3 or 4 cycles before the
		// interrupt flag is set, 4 cycles before the PC is set to the right
		// interrupt vector address and the old PC is pushed on the stack,
		// and then 75 cycles of instructions (including the RJMP in the
		// ISR vector table) until the first delay. After the delay, there
		// are 17 more cycles until the pin value is read (excluding the
		// delay in the loop).
		// We want to have a total delay of 1.5 bit time. Inside the loop,
		// we already wait for 1 bit time - 23 cycles, so here we wait for
		// 0.5 bit time - (71 + 18 - 22) cycles.
		_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

		// There are 23 cycles in each loop iteration (excluding the delay)
		_rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

		// There are 37 cycles from the last bit read to the start of
		// stopbit delay and 11 cycles from the delay until the interrupt
		// mask is enabled again (which _must_ happen during the stopbit).
		// This delay aims at 3/4 of a bit time, meaning the end of the
		// delay will be at 1/4th of the stopbit. This allows some extra
		// time for ISR cleanup, which makes 115200 baud at 16Mhz work more
		// reliably
		_rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
#else // Timings counted from gcc 4.3.2 output
		// Note that this code is a _lot_ slower, mostly due to bad register
		// allocation choices of gcc. This works up to 57600 on 16Mhz and
		// 38400 on 8Mhz.
		_rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
		_rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
		_rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
#endif


		// Enable the PCINT for the entire port here, but never disable it
		// (others might also need it, so we disable the interrupt by using
		// the per-pin PCMSK register).
		*digitalPinToPCICR(_rxPin) |= _BV(digitalPinToPCICRbit(_rxPin));
		// Precalculate the pcint mask register and value, so setRxIntMask
		// can be used inside the ISR without costing too much time.
		_pcint_maskreg = digitalPinToPCMSK(_rxPin);
		_pcint_maskvalue = _BV(digitalPinToPCMSKbit(_rxPin));

#if (SU_MODE != SU_RX_ONLY)
		tunedDelay(_tx_delay); // if we were low this establishes the end
#endif
	}
#endif
#if (SU_MODE == SU_TX_ONLY)
	tunedDelay(_tx_delay); // if we were low this establishes the end
#endif
#if (SU_MODE != SU_TX_ONLY)
	listen();
#endif
}

#if (SU_MODE != SU_TX_ONLY)
template <uint8_t _SU_RX_BUFFER>
void SoftwareUart<_SU_RX_BUFFER>::setRxIntMsk(bool enable)
{
	if (enable)
		*_pcint_maskreg |= _pcint_maskvalue;
	else
		*_pcint_maskreg &= ~_pcint_maskvalue;
}

// Read data from buffer
template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::read()
{
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
	uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
	_receive_buffer_head = (_receive_buffer_head + 1) % _SU_RX_BUFFER;
	return d;
}

template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::available()
{
	return (_receive_buffer_tail + _SU_RX_BUFFER - _receive_buffer_head) % _SU_RX_BUFFER;
}

template <uint8_t _SU_RX_BUFFER>
int16_t SoftwareUart<_SU_RX_BUFFER>::peek()
{
	// Empty buffer?
	if (_receive_buffer_head == _receive_buffer_tail)
		return -1;

	// Read from "head"
	return _receive_buffer[_receive_buffer_head];
}

#endif

#if (SU_MODE != SU_RX_ONLY)
#if (SU_MODE == SU_TX_ONLY)
size_t SoftwareUart::write(uint8_t b)
#else
template <uint8_t _SU_RX_BUFFER>
size_t SoftwareUart<_SU_RX_BUFFER>::write(uint8_t b)
#endif
{
	if (_tx_delay == 0)
	{
#if (SU_MODE != SU_TX_ONLY)
		setWriteError();
#endif
		return 0;
	}

	// By declaring these as local variables, the compiler will put them
	// in registers _before_ disabling interrupts and entering the
	// critical timing sections below, which makes it a lot easier to
	// verify the cycle timings
	volatile uint8_t *reg = _txPort;
	uint8_t reg_mask = _txBitMask;
	uint8_t inv_mask = ~_txBitMask;
	uint8_t oldSREG = SREG;
	bool inv = _inverse_logic;
	uint16_t delay = _tx_delay;

	if (inv)
		b = ~b;

	cli();  // turn off interrupts for a clean txmit

	// NS - Set Pin to Output
	if (_full_duplex == false)
	{
		pinMode(_txPin, OUTPUT);
	}

	// Write the start bit
	if (inv)
		*reg |= reg_mask;
	else
		*reg &= inv_mask;

	tunedDelay(delay);

	// Write each of the 8 bits
	for (uint8_t i = 8; i > 0; --i)
	{
		if (b & 1) // choose bit
			*reg |= reg_mask; // send 1
		else
			*reg &= inv_mask; // send 0

		tunedDelay(delay);
		b >>= 1;
	}

	// NS - Set Pin back to Input
	if (_full_duplex == false)
	{
		pinMode(_txPin, INPUT);
	}

	// restore pin to natural state
	if (inv)
		*reg &= inv_mask;
	else
		*reg |= reg_mask;

	SREG = oldSREG; // turn interrupts back on
	tunedDelay(_tx_delay);

	return 1;
}
#endif

#endif
