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

#ifndef MACHINE_TYPE
#define MACHINE_TYPE "Delta"
#endif


bool Com::writeToAll = true; // transmit start messages to all devices!

void Com::cap(FSTRINGPARAM(text)) {
  printF("Cap:");
  printFLN(text);
}
void Com::config(FSTRINGPARAM(text)) {
  printF("Config:");
  printFLN(text);
}
void Com::config(FSTRINGPARAM(text), int value) {
  printF("Config:");
  printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), const char *msg) {
  printF("Config:");
  printF(text);
  print(msg);
  println();
}
void Com::config(FSTRINGPARAM(text), int32_t value) {
  printF("Config:");
  printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), uint32_t value) {
  printF("Config:");
  printFLN(text, value);
}
void Com::config(FSTRINGPARAM(text), float value, uint8_t digits) {
  printF("Config:");
  printFLN(text, value, digits);
}
void Com::printWarningF(FSTRINGPARAM(text)) {
  printF("Warning:");
  printF(text);
}
void Com::printWarningFLN(FSTRINGPARAM(text)) {
  printF("Warning:");
  printFLN(text);
}
void Com::printInfoF(FSTRINGPARAM(text)) {
  printF("Info:");
  printF(text);
}
void Com::printInfoFLN(FSTRINGPARAM(text)) {
  printF("Info:");
  printFLN(text);
}

void Com::printErrorF(FSTRINGPARAM(text)) {
  printF("Error:");
  printF(text);
}
void Com::printErrorFLN(FSTRINGPARAM(text)) {
  printF("Error:");
  printFLN(text);
}
void Com::printFLN(FSTRINGPARAM(text)) {
  printF(text);
  println();
}
void Com::printFLN(FSTRINGPARAM(text), const char *msg) {
  printF(text);
  print(msg);
  println();
}

void Com::printF(FSTRINGPARAM(ptr)) {
  char c;
  while ((c = pgm_read_byte(ptr++)) != 0)
    GCodeSource::writeToAll(c);
}
void Com::printF(FSTRINGPARAM(text), const char *msg) {
  printF(text);
  print(msg);
}

void Com::printF(FSTRINGPARAM(text), int value) {
  printF(text);
  print((int32_t)value);
}
void Com::printF(FSTRINGPARAM(text), int32_t value) {
  printF(text);
  print(value);
}
void Com::printF(FSTRINGPARAM(text), uint32_t value) {
  printF(text);
  printNumber(value);
}
void Com::printFLN(FSTRINGPARAM(text), int value) {
  printF(text);
  print((int32_t)value);
  println();
}
void Com::printFLN(FSTRINGPARAM(text), int32_t value) {
  printF(text);
  print(value);
  println();
}
void Com::printFLN(FSTRINGPARAM(text), uint32_t value) {
  printF(text);
  printNumber(value);
  println();
}
void Com::printFLN(FSTRINGPARAM(text), float value, uint8_t digits) {
  printF(text);
  printFloat(value, digits);
  println();
}
void Com::printF(FSTRINGPARAM(text), float value, uint8_t digits) {
  printF(text);
  printFloat(value, digits);
}

void Com::print(const char *text) {
  while(*text) {
    GCodeSource::writeToAll(*text++);
  }
}
void Com::print(int32_t value) {
  if(value < 0) {
    GCodeSource::writeToAll('-');
    value = -value;
  }
  printNumber(value);
}
void Com::print(uint32_t value) {
  printNumber(value);
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
void Com::printArrayFLN(FSTRINGPARAM(text), float *arr, uint8_t n, uint8_t digits) {
  printF(text);
  for(uint8_t i = 0; i < n; i++)
    printF(PSTR(" "), arr[i], digits);
  println();
}
void Com::printArrayFLN(FSTRINGPARAM(text), int32_t *arr, uint8_t n) {
  printF(text);
  for(uint8_t i = 0; i < n; i++)
    printF(PSTR(" "), arr[i]);
  println();
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

