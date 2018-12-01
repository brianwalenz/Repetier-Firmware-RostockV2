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

#include "Repetier.h"
#include "HAL.h"
#include "Communication.h"


//  Communication speed.
//   - 250000 : Fastest with error rate of 0% with 16 or 32 MHz - update wiring_serial.c in your board files. See boards/readme.txt
//   - 115200 : Fast, but may produce communication errors on quite regular basis, Error rate -3,5%
//   - 76800  : Best setting for Arduino with 16 MHz, Error rate 0,2% page 198 AVR1284 Manual. Result: Faster communication then 115200
//   - 57600  : Should produce nearly no errors, on my gen 6 it's faster than 115200 because there are no errors slowing down the connection
//   - 38600
//  Overridden if EEPROM activated.
//
#define BAUDRATE 250000

long baudrate = BAUDRATE;



void
Com::printF(FSTRINGPARAM(ptr)) {
  while (pgm_read_byte(ptr) != 0)
    RFSERIAL.write(pgm_read_byte(ptr++));
}



void
Com::print(const char *text) {
  while (*text != 0)
    RFSERIAL.write(*text++);
}

void
Com::print(const char c) {
  RFSERIAL.write(c);
}





void
Com::printF(FSTRINGPARAM(text), const char *msg) {
  Com::printF(text);
  Com::print(msg);
}
#if 0
void
Com::printF(FSTRINGPARAM(text), int value) {
  Com::printF(text);
  Com::print((int32_t)value);
}
#endif
void
Com::printF(FSTRINGPARAM(text), int8_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), uint8_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), int16_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), uint16_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), int32_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), uint32_t value) {
  Com::printF(text);
  Com::print(value);
}

void
Com::printF(FSTRINGPARAM(text), float value, uint8_t digits) {
  Com::printF(text);
  Com::printFloat(value, digits);
}



void
Com::print(uint8_t value)   { Com::print((uint32_t)value); }

void
Com::print( int8_t value)   { Com::print(( int32_t)value); }


void
Com::print(uint16_t value)  { Com::print((uint32_t)value); }

void
Com::print( int16_t value)  { Com::print(( int32_t)value); }


void
Com::print(uint32_t value)  { Com::printNumber(value);     }

void
Com::print( int32_t value)  {
  if (value < 0) {
    Com::print('-');
    value = -value;
  }
  Com::printNumber((uint32_t)value);
}



void
Com::print(float number)    { Com::printFloat(number, 6); }







void
Com::printNumber(uint32_t n) {
  char buf[11]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[10];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    *--str = '0' + (m - 10 * n);
  } while(n);

  Com::print(str);
}



void
Com::printFloat(float number, uint8_t digits) {
  if (isnan(number)) {
    Com::printF(PSTR("NAN"));
    return;
  }
  if (isinf(number)) {
    Com::printF(PSTR("INF"));
    return;
  }
  // Handle negative numbers
  if (number < 0.0) {
    Com::print('-');
    number = -number;
  }
  // Round correctly so that print(1.999, 2) prints as "2.00"
  float rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  float remainder = number - (float)int_part;
  Com::printNumber(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Com::print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Com::print(toPrint);
    remainder -= toPrint;
  }
}



void
Com::printArrayF(FSTRINGPARAM(text), float *arr, uint8_t n, uint8_t digits) {
  Com::printF(text);
  for(uint8_t i = 0; i < n; i++)
    Com::printF(PSTR(" "), arr[i], digits);
  Com::printF(PSTR("\n"));
}



void
Com::printArrayF(FSTRINGPARAM(text), int32_t *arr, uint8_t n) {
  Com::printF(text);
  for(uint8_t i = 0; i < n; i++)
    Com::printF(PSTR(" "), arr[i]);
  Com::printF(PSTR("\n"));
}

