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

#include "Endstops.h"
#include "Printer.h"

#define ENDSTOP_X_MIN_INVERTING true    //  true to invert the logic of endstops.
#define ENDSTOP_Y_MIN_INVERTING true
#define ENDSTOP_Z_MIN_INVERTING true
#define ENDSTOP_X_MAX_INVERTING false
#define ENDSTOP_Y_MAX_INVERTING false
#define ENDSTOP_Z_MAX_INVERTING false

Endstops  endstops;

void
Endstops::update(void) {
  uint8_t state = 0;

  //
  //  Disable/enable based on what endstops exist.
  //    Rostock has no min endstops.
  //

  //if (READ(Y_MIN_PIN) != ENDSTOP_Y_MIN_INVERTING)
  //  state |= ENDSTOP_Y_MIN_ID;

  //if (READ(X_MIN_PIN) != ENDSTOP_X_MIN_INVERTING)
  //  state |= ENDSTOP_X_MIN_ID;

  //if (READ(Z_MIN_PIN) != ENDSTOP_Z_MIN_INVERTING)
  //  state |= ENDSTOP_Z_MIN_ID;

  if (READ(Y_MAX_PIN) != ENDSTOP_Y_MAX_INVERTING)
    state |= ENDSTOP_Y_MAX_ID;

  if (READ(X_MAX_PIN) != ENDSTOP_X_MAX_INVERTING)
    state |= ENDSTOP_X_MAX_ID;

  if (READ(Z_MAX_PIN) != ENDSTOP_Z_MAX_INVERTING)
    state |= ENDSTOP_Z_MAX_ID;

  InterruptProtectedBlock noInts;

  //  The effect of the below is to require a switch to be on for two cycles
  //  before it's retained in currState.

  //  Debounce.  If any switches were on then and now, keep them on.  Otherwise, turn them off.
  _lastRead &= state;

  //  If the debounced state diffes from the current state, reset.
  if (_lastRead != _currState) {
    //if (Printer::debugEndStop()) {
    Com::printF(PSTR("endstops changed:\n"));
    Com::printF(PSTR("  x_min:"));   Com::printF(xMin(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(xMin(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    Com::printF(PSTR("  x_max:"));   Com::printF(xMax(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(xMax(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    Com::printF(PSTR("  y_min:"));   Com::printF(yMin(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(yMin(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    Com::printF(PSTR("  y_max:"));   Com::printF(yMax(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(yMax(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    Com::printF(PSTR("  z_min:"));   Com::printF(zMin(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(zMin(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    Com::printF(PSTR("  z_max:"));   Com::printF(zMax(_currState) ? PSTR(" On  -> ") : PSTR("Off -> "));   Com::printF(zMax(_lastRead) ? PSTR("On\n") : PSTR("Off\n"));
    //}

    _currState    = _lastRead;
    _accumulated |= _lastRead;
  }

  //  Regardless of state change, update the last read to whatever we just read.
  _lastRead = state;
}



void
Endstops::report(void) {
  Com::printF(PSTR("endstops hit:\n"));
  Com::printF(PSTR("  x_min:"));   Com::printF(xMin() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("  x_max:"));   Com::printF(xMax() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("  y_min:"));   Com::printF(yMin() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("  y_max:"));   Com::printF(yMax() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("  z_min:"));   Com::printF(zMin() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("  z_max:"));   Com::printF(zMax() ? PSTR("H\n") : PSTR("L\n"));
  Com::printF(PSTR("\n"));
}



//
//  If there is a pullup resistor on the endstop, make sure PULLUP() is enabled.
//  Comment out if any pin isn't defined.
//
void
Endstops::setup(void) {
  SET_INPUT(X_MIN_PIN);   PULLUP(X_MIN_PIN, HIGH);
  SET_INPUT(Y_MIN_PIN);   PULLUP(Y_MIN_PIN, HIGH);
  SET_INPUT(Z_MIN_PIN);   PULLUP(Z_MIN_PIN, HIGH);
  SET_INPUT(X_MAX_PIN);   PULLUP(X_MAX_PIN, HIGH);
  SET_INPUT(Y_MAX_PIN);   PULLUP(Y_MAX_PIN, HIGH);
  SET_INPUT(Z_MAX_PIN);   PULLUP(Z_MAX_PIN, HIGH);
}
