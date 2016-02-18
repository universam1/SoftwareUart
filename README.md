SoftwareUart (adapted from SoftwareSerialWithHalfDuplex)

This library is based off SoftwareSerial from Arduino 1.6.5r5, with half-duplex changes 
from @nickstedman's SoftwareSerialWithHalfDuplex library.

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