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

  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef HAL_H
#define HAL_H

/**
   This is the main Hardware Abstraction Layer (HAL).
   To make the firmware work with different processors and tool chains,
   all hardware related code should be packed into the hal files.
*/

#include <avr/pgmspace.h>
#include <avr/io.h>


#if 0
#define NUM_ANALOG_TEMP_SENSORS 2
#define ANALOG_INPUTS 2
extern uint16_t osAnalogInputValues  [ANALOG_INPUTS];
#endif


uint16_t      integerSqrt(uint32_t a);
int32_t       Div4U2U(uint32_t a,uint16_t b);
unsigned long U16SquaredToU32(unsigned int val);
unsigned int  ComputeV(long timer,long accel);
uint32_t      mulu16xu16to32(unsigned int a,unsigned int b);
unsigned int  mulu6xu16shift16(unsigned int a,unsigned int b);

int32_t       CPUDivU2(unsigned int divisor);


#define INLINE __attribute__((always_inline))

#include <avr/io.h>

#define PACK

#define FSTRINGVALUE(var,value) const char var[] PROGMEM = value;
#define FSTRINGVAR(var) static const char var[] PROGMEM;
#define FSTRINGPARAM(var) PGM_P var

#include <avr/eeprom.h>
#include <avr/wdt.h>

/** \brief Prescale factor, timer0 runs at.
    All known Arduino boards use 64. This value is needed for the extruder timing. */
#define TIMER0_PRESCALE 64


#define ANALOG_PRESCALER _BV(ADPS0)|_BV(ADPS1)|_BV(ADPS2)


#ifndef EXTERNALSERIAL
#undef HardwareSerial_h
#define  HardwareSerial_h // Don't use standard serial console
#endif

#include <inttypes.h>
#include "Stream.h"
#ifdef EXTERNALSERIAL
//#define SERIAL_RX_BUFFER_SIZE 128
#endif



#include "Arduino.h"
#undef min
#undef max

#include "fastio.h"



    //  Only call when in an interrupt handler.
inline void allowInterrupts(void)    { sei(); }
inline void forbidInterrupts(void)   { cli(); }

class InterruptProtectedBlock {
public:
  InterruptProtectedBlock(bool later = false) {
    sreg = SREG;

    if (later == false)
      cli();
  };

  ~InterruptProtectedBlock() {
    SREG = sreg;
  };

  void protect()   { cli();       };
  void unprotect() { SREG = sreg; };

private:
  uint8_t sreg;
};




//#define ANALOG_REF_AREF      0
//#define ANALOG_REF_AVCC      _BV(REFS0)
//#define ANALOG_REF_INT_1_1   _BV(REFS1)
//#define ANALOG_REF_INT_2_56  _BV(REFS0) | _BV(REFS1)
//#define ANALOG_REF           ANALOG_REF_AVCC


#define PWM_COOLER        0
#define PWM_BED           1
#define PWM_FAN1          2
#define PWM_FAN2          3
#define NUM_PWM           4


class HAL {
public:

  HAL() {
    counter500ms      = 0;
    counter100ms      = 0;

    execute500ms      = 0;
    execute100ms      = 0;
  };
  ~HAL() {
  };

  void setup(void);

  uint16_t getFreeRAM(void) {
    extern int __heap_start;
    extern int *__brkval; 

    uint16_t f = (uint16_t) &f - (__brkval == 0 ? (uint16_t) &__heap_start : (uint16_t) __brkval); 

    return(f);
  };

  void resetHardware();


  inline float maxExtruderTimerFrequency() {
    return (float)F_CPU/TIMER0_PRESCALE;
  }

  volatile uint16_t     counter500ms;
  volatile uint16_t     counter100ms;

  volatile uint8_t      execute500ms;
  volatile uint8_t      execute100ms;
};

extern HAL  hal;

#endif // HAL_H
