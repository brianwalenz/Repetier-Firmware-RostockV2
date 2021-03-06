/*
  This file is part of Repetier-Firmware.

  Repetier-Firmware is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Repetier-Firmware is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

  This firmware is a nearly complete rewrite of the sprinter firmware
  by kliment (https://github.com/kliment/Sprinter)
  which based on Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

//  This is idiotic.  sprintf() adds 2KB ROM usage.
//  And at most 256 bytes RAM usage for two buffers
//   - one for copying the PSTR to RAM
//   - one for the output string
//
//  But it doesn't support floating point, thanks, Arduino!
//  (Adds another 1.5KB to ROM)
//
//  http://forum.arduino.cc/index.php/topic,124809.0.html
//    cd /Applications/Arduino.app/Contents/Java/hardware/tools/avr/avr/lib/avr6/
//    cp libc.a libc.a.orig
//    ../../bin/ar -dv libc.a vfprintf_std.o
//    ../../bin/ar -xv libprintf_flt.a vfprintf_flt.o
//    ../../bin/ar -rv libc.a vfprintf_flt.o

namespace Com {
  void print(char c);
  void printF(FSTRINGPARAM(text));
  void print(const char *text);

  void printf(FSTRINGPARAM(fmtrom), ...);

  void printF(FSTRINGPARAM(text), const char *msg);

  //  void printF(FSTRINGPARAM(text), int value);

  void printF(FSTRINGPARAM(text), int8_t value);
  void printF(FSTRINGPARAM(text), uint8_t value);

  void printF(FSTRINGPARAM(text), int16_t value);
  void printF(FSTRINGPARAM(text), uint16_t value);

  void printF(FSTRINGPARAM(text), int32_t value);
  void printF(FSTRINGPARAM(text), uint32_t value);

  void printF(FSTRINGPARAM(text), float value, uint8_t digits=2);


  void print(uint8_t value);
  void print( int8_t value);

  void print(uint16_t value);
  void print( int16_t value);

  void print(uint32_t value);
  void print( int32_t value);

  void print(float number);

  void printNumber(uint32_t n);
  void printFloat(float number, uint8_t digits);

  void printArrayF(FSTRINGPARAM(text),float *arr,uint8_t n=4,uint8_t digits=2);
  void printArrayF(FSTRINGPARAM(text),long *arr,uint8_t n=4);
}

#endif // COMMUNICATION_H
