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

#include "Repetier.h"
#include "motion.h"

flag8_t Endstops::lastState = 0;
flag8_t Endstops::lastRead = 0;
flag8_t Endstops::accumulator = 0;

void Endstops::update() {
  flag8_t newRead = 0;

#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
  if(READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING)
    newRead |= ENDSTOP_Y_MIN_ID;
#endif

#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
  if(READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING)
    newRead |= ENDSTOP_Y_MAX_ID;
#endif

#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
  if(READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING) {
    newRead |= ENDSTOP_X_MIN_ID;
  }
#endif

#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
  if(READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING)
    newRead |= ENDSTOP_X_MAX_ID;
#endif

#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
  if(READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING)
    newRead |= ENDSTOP_Z_MIN_ID;
#endif

#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
  if(READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING)
    newRead |= ENDSTOP_Z_MAX_ID;
#endif

#if FEATURE_Z_PROBE
#if Z_PROBE_PIN == Z_MIN_PIN && MIN_HARDWARE_ENDSTOP_Z
  if(newRead & ENDSTOP_Z_MIN_ID) // prevent different results causing confusion
    newRead |= ENDSTOP_Z_PROBE_ID;
  if(!Printer::isHoming())
    newRead &= ~ENDSTOP_Z_MIN_ID; // could cause wrong signals depending on probe position
#else
  if(Z_PROBE_ON_HIGH ? READ(Z_PROBE_PIN) : !READ(Z_PROBE_PIN))
    newRead |= ENDSTOP_Z_PROBE_ID;
#endif
#endif

  InterruptProtectedBlock noInts; // bad idea to run this from different interrupts at once!
  lastRead &= newRead;
  if(lastRead != lastState
     ) { // Report endstop hit changes
    lastState = lastRead;
    accumulator |= lastState;
    if (Printer::debugEndStop())  Endstops::report();
  } else {
    lastState = lastRead;
  }
  lastRead = newRead;
}




void Endstops::report() {
  Com::printF(PSTR("endstops hit: "));

#if (X_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_X
  Com::printF(PSTR("x_min:"));
  Com::printF(xMin() ? PSTR("H ") : PSTR("L "));
#endif

#if (X_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_X
  Com::printF(PSTR("x_max:"));
  Com::printF(xMax() ? PSTR("H ") : PSTR("L "));
#endif

#if (Y_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Y
  Com::printF(PSTR("y_min:"));
  Com::printF(yMin() ? PSTR("H ") : PSTR("L "));
#endif

#if (Y_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Y
  Com::printF(PSTR("y_max:"));
  Com::printF(yMax() ? PSTR("H ") : PSTR("L "));
#endif

#if (Z_MIN_PIN > -1) && MIN_HARDWARE_ENDSTOP_Z
  Com::printF(PSTR("z_min:"));
  Com::printF(zMin() ? PSTR("H ") : PSTR("L "));
#endif

#if (Z_MAX_PIN > -1) && MAX_HARDWARE_ENDSTOP_Z
  Com::printF(PSTR("z_max:"));
  Com::printF(zMax() ? PSTR("H ") : PSTR("L "));
#endif

#if FEATURE_Z_PROBE
  Com::printF(PSTR("Z-probe state:"));
  Com::printF(zProbe() ? PSTR("H ") : PSTR("L "));
#endif
  Com::printF(PSTR("\n"));
}




void Endstops::setup() {
  // Set end stops to input and enable pullup if required
#if MIN_HARDWARE_ENDSTOP_X
#if X_MIN_PIN > -1
  SET_INPUT(X_MIN_PIN);
#if ENDSTOP_PULLUP_X_MIN
  PULLUP(X_MIN_PIN, HIGH);
#endif
#else
#error You have defined hardware x min endstop without pin assignment. Set pin number for X_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_Y
#if Y_MIN_PIN > -1
  SET_INPUT(Y_MIN_PIN);
#if ENDSTOP_PULLUP_Y_MIN
  PULLUP(Y_MIN_PIN, HIGH);
#endif
#else
#error You have defined hardware y min endstop without pin assignment. Set pin number for Y_MIN_PIN
#endif
#endif

#if MIN_HARDWARE_ENDSTOP_Z
#if Z_MIN_PIN > -1
  SET_INPUT(Z_MIN_PIN);
#if ENDSTOP_PULLUP_Z_MIN
  PULLUP(Z_MIN_PIN, HIGH);
#endif
#else
#error You have defined hardware z min endstop without pin assignment. Set pin number for Z_MIN_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_X
#if X_MAX_PIN > -1
  SET_INPUT(X_MAX_PIN);
#if ENDSTOP_PULLUP_X_MAX
  PULLUP(X_MAX_PIN, HIGH);
#endif
#else
#error You have defined hardware x max endstop without pin assignment. Set pin number for X_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_Y
#if Y_MAX_PIN > -1
  SET_INPUT(Y_MAX_PIN);
#if ENDSTOP_PULLUP_Y_MAX
  PULLUP(Y_MAX_PIN, HIGH);
#endif
#else
#error You have defined hardware y max endstop without pin assignment. Set pin number for Y_MAX_PIN
#endif
#endif

#if MAX_HARDWARE_ENDSTOP_Z
#if Z_MAX_PIN>-1
  SET_INPUT(Z_MAX_PIN);
#if ENDSTOP_PULLUP_Z_MAX
  PULLUP(Z_MAX_PIN, HIGH);
#endif
#else
#error You have defined hardware z max endstop without pin assignment. Set pin number for Z_MAX_PIN
#endif
#endif
}
