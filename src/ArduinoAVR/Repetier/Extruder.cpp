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
#include "gcode.h"
#include "Commands.h"
#include "Eeprom.h"
#include "motion.h"
#include "Printer.h"

#include "Extruder.h"

#include "rmath.h"


Extruder  extruder(0);
Extruder *activeExtruder = &extruder;

Extruder::Extruder(uint8_t idx) {

  if (idx == 0) {
    id                     = 0;

    xOffset                = 0;     //  Offsets in steps, not mm!
    yOffset                = 0;
    zOffset                = 0;

    invert                 = 1;

    stepsPerMM             = 92.4;  //  For EZStruder

    enablePin              = E0_ENABLE_PIN;
    enableOn               = 0;

    dirPin                 = E0_DIR_PIN;
    stepPin                = E0_STEP_PIN;

    maxFeedrate            = 100;    //  In mm/s
    maxAcceleration        = 6500;   //  In mm/s^2
    maxStartFeedrate       = 45;     //  In mm/s

    extrudePosition        = 0;

    watchPeriod            = 3;      //  Wait 3 seconds after reacing target temp, for M109

    waitRetractTemperature = 150;
    waitRetractUnits       = 0;

    advanceK               = 0;
    advanceL               = 0;
    advanceBacklash        = 0;
    diameter               = 0;
    flags                  = 0;

    selectCommands         = PSTR("M117 Extruder 1");
    deselectCommands       = PSTR("");
  }
}



//  These are G10 and G11
//
void
Extruder::retractDistance(float dist, bool extraLength) {
#if 0
  float   oldFeedrate = Printer::feedrate;
  int32_t distance    = static_cast<int32_t>(dist * stepsPerMM / Printer::extrusionFactor);
  int32_t oldEPos     = Printer::currentPositionSteps[E_AXIS];
  float   speed       = distance > 0 ? EEPROM_FLOAT(RETRACTION_SPEED) : EEPROM_FLOAT(RETRACTION_UNDO_SPEED);

  PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -distance, RMath::max(speed, 1.f), false, false);

  Printer::currentPositionSteps[E_AXIS] = oldEPos; // restore previous extruder position
  Printer::feedrate                     = oldFeedrate;
#endif  //  DISABLED
}



void
Extruder::retract(bool isRetract, bool isLong) {
#if 0
  float   oldFeedrate = Printer::feedrate;
  float   distance = (isLong ? EEPROM_FLOAT( RETRACTION_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_LENGTH));

  float   zLiftF = EEPROM_FLOAT(RETRACTION_Z_LIFT);
  int32_t zlift = static_cast<int32_t>(zLiftF * Printer::axisStepsPerMM[Z_AXIS]);

  if(isRetract && !isRetracted()) {
#ifdef EARLY_ZLIFT
    if(zlift > 0) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
      Printer::coordinateOffset[Z_AXIS] -= zLiftF;
    }
#endif

    retractDistance(distance);
    setRetracted(true);

#ifndef EARLY_ZLIFT
    if(zlift > 0) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
      Printer::coordinateOffset[Z_AXIS] -= zLiftF;
    }
#endif
  } else if(!isRetract && isRetracted()) {
    if(zlift > 0) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, -zlift, 0, Printer::maxFeedrate[Z_AXIS], false);
      Printer::coordinateOffset[Z_AXIS] += zLiftF;
    }
    retractDistance(-distance - (isLong ? EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH) : EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH)), false);
    setRetracted(false);
  }
  Printer::feedrate = oldFeedrate;
#endif  //  DISABLED
}

