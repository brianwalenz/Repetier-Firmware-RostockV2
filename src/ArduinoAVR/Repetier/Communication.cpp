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






void Com::printF(FSTRINGPARAM(ptr)) {
  while (pgm_read_byte(ptr) != 0)
    RFSERIAL.write(pgm_read_byte(ptr++));
}



void Com::print(const char *text) {
  while (*text != 0)
    RFSERIAL.write(*text++);
}

void Com::print(const char c) {
  RFSERIAL.write(c);
}




void Com::printNumber(uint32_t n) {
  char buf[11]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[10];
  *str = '\0';
  do {
    unsigned long m = n;
    n /= 10;
    *--str = '0' + (m - 10 * n);
  } while(n);

  print(str);
}



void Com::printFloat(float number, uint8_t digits) {
  if (isnan(number)) {
    printF(PSTR("NAN"));
    return;
  }
  if (isinf(number)) {
    printF(PSTR("INF"));
    return;
  }
  // Handle negative numbers
  if (number < 0.0) {
    print('-');
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
  printNumber(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    print('.');

  // Extract digits from the remainder one at a time
  while (digits-- > 0) {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint;
  }
}



void Com::printArrayF(FSTRINGPARAM(text), float *arr, uint8_t n, uint8_t digits) {
  printF(text);
  for(uint8_t i = 0; i < n; i++)
    printF(PSTR(" "), arr[i], digits);
  printF(PSTR("\n"));
}



void Com::printArrayF(FSTRINGPARAM(text), int32_t *arr, uint8_t n) {
  printF(text);
  for(uint8_t i = 0; i < n; i++)
    printF(PSTR(" "), arr[i]);
  printF(PSTR("\n"));
}

