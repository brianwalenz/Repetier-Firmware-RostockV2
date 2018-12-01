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

*/

#ifndef _ENDSTOPS_H
#define _ENDSTOPS_H

#include "Repetier.h"

#define ENDSTOP_X_MIN_ID     1
#define ENDSTOP_X_MAX_ID     2
#define ENDSTOP_Y_MIN_ID     4
#define ENDSTOP_Y_MAX_ID     8
#define ENDSTOP_Z_MIN_ID    16
#define ENDSTOP_Z_MAX_ID    32

#define ENDSTOP_MAX_IDS    (ENDSTOP_X_MAX_ID | ENDSTOP_Y_MAX_ID | ENDSTOP_Z_MAX_ID)
#define ENDSTOP_MIN_IDS    (ENDSTOP_X_MIN_ID | ENDSTOP_Y_MIN_ID | ENDSTOP_Z_MIN_ID)
#define ENDSTOP_ANY_IDS    (ENDSTOP_MAX_IDS | ENDSTOP_MIN_IDS)

class Endstops {
public:
  Endstops() {
    _currState   = 0;
    _accumulated = 0;
    _lastRead    = 0;
  };

  ~Endstops() {
  };

  uint8_t _currState;     //  the current state saved in the class.
  uint8_t _accumulated;   //  accumulated on state
  uint8_t _lastRead;      //  the last bitmask read from hardware

public:
  void setup(void);
  void report(void);
  void update(void);

  void resetAccumulator(void)     { _accumulated = 0;            };
  void fillFromAccumulator(void)  { _currState   = _accumulated; };

  bool any      (void)        { return((_currState)                      != 0); };
  bool anyXYZMax(void)        { return((_currState & ENDSTOP_MAX_IDS)    != 0); };
  bool anyXYZMin(void)        { return((_currState & ENDSTOP_MIN_IDS)    != 0); };
  bool anyXYZ   (void)        { return((_currState & ENDSTOP_ANY_IDS)    != 0); };

  bool xMin     (void)        { return((_currState & ENDSTOP_X_MIN_ID)   != 0); };
  bool xMax     (void)        { return((_currState & ENDSTOP_X_MAX_ID)   != 0); };
  bool yMin     (void)        { return((_currState & ENDSTOP_Y_MIN_ID)   != 0); };
  bool yMax     (void)        { return((_currState & ENDSTOP_Y_MAX_ID)   != 0); };
  bool zMin     (void)        { return((_currState & ENDSTOP_Z_MIN_ID)   != 0); };
  bool zMax     (void)        { return((_currState & ENDSTOP_Z_MAX_ID)   != 0); };

  bool xMin     (uint8_t s)   { return((s & ENDSTOP_X_MIN_ID)   != 0); };
  bool xMax     (uint8_t s)   { return((s & ENDSTOP_X_MAX_ID)   != 0); };
  bool yMin     (uint8_t s)   { return((s & ENDSTOP_Y_MIN_ID)   != 0); };
  bool yMax     (uint8_t s)   { return((s & ENDSTOP_Y_MAX_ID)   != 0); };
  bool zMin     (uint8_t s)   { return((s & ENDSTOP_Z_MIN_ID)   != 0); };
  bool zMax     (uint8_t s)   { return((s & ENDSTOP_Z_MAX_ID)   != 0); };
};

extern Endstops endstops;

#endif
