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

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#include "Repetier.h"


//#define INCREMENT_MIN_MAX(a,steps,_min,_max) if ( (increment<0) && (_min>=0) && (a<_min-increment*steps) ) {a=_min;} else { a+=increment*steps; if(a<_min) a=_min; else if(a>_max) a=_max;};

// this version not have single byte variable rollover bug
#define INCREMENT_MIN_MAX(a,steps,_min,_max) a = constrain((a + increment * steps), _min, _max);



extern UIDisplay uid;


const long baudrates[] PROGMEM = {9600, 14400, 19200, 28800, 38400, 56000,
                                  57600, 76800, 111112, 115200, 128000, 230400,
                                  250000, 256000, 460800, 500000, 921600, 1000000, 1500000, 0 };



bool
UIDisplay::doEncoderChange(int16_t encoderChange, bool allowMoves) {

  if (Printer::isUIErrorMessage()) {
    Printer::setUIErrorMessage(false);
    // return true;
  }

  //  Find the time delta sine the last encoder change, and remember the time of this change.
  //
  //  I've never seen dtActual drop below 100 (maybe 99):
  //      doEncoderChange() is called by slowAction(),
  //      which is called in Commands::checkForPeriodicalActions(),
  //      which seems to have an interrupt driven action every 100ms.
  //
  //  Then, use that time to increase the acceleration if we're near the minimal time, decrease it
  //  if we're somewhat slow, and reset it if we've been idle for a second or more.

  millis_t thisEncoderTime = HAL::timeInMilliseconds();
  millis_t dtActual        = thisEncoderTime - lastEncoderTime;

  lastEncoderTime = thisEncoderTime;

  if (dtActual < 110)   encoderAccel *= 1.21;
  if (dtActual > 220)   encoderAccel /= 1.66;
  if (dtActual > 999)   encoderAccel  = 1.0;

  if (encoderAccel < 1.0) encoderAccel  = 1.0;




  //  No menu displayed, we're showing a status display page.
  //  No acceleration, only responds to positive/negative turns of the knob.
  //
  if ((menuLevel == 0) && (encoderChange > 0)) {
    menuPos[0]++;

    if(menuPos[0] >= UI_NUM_PAGES)
      menuPos[0] = 0;

    return(true);
  }

  if ((menuLevel == 0) && (encoderChange < 0)) {
    if (menuPos[0] == 0)
      menuPos[0] = UI_NUM_PAGES;

    menuPos[0]--;

    return(true);
  }

  //  Otherwise, we're in a menu.  Figure out what menu, etc.
  //
  UIMenu       *men        = menu[menuLevel];
  uint8_t       mtype      =                pgm_read_byte(&(men->menuType)) & 127;
  UIMenuEntry **entries    = (UIMenuEntry**)pgm_read_word(&(men->entries));
  uint8_t       numEntries =                pgm_read_byte(&(men->numEntries));
  UIMenuEntry  *ent        = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
  unsigned int  action     =                pgm_read_word(&(ent->entryAction));


  //  If there's a menu page showing, move forward/backward through the menu items.
  //  Made compllicated by the need to search forward/backward for the next visible item.
  //  Again, doesn't use acceleration, just positive/negative turns of the knob.
  //
  if ((mtype == UI_MENU_TYPE_SUBMENU) && (activeAction == 0)) {

    //  If a positive change, search forward for the next visible entry.
    if (encoderChange > 0) {
      while (menuPos[menuLevel] + 1 < numEntries) {
        UIMenuEntry *nent = (UIMenuEntry *)pgm_read_word(&(entries[++menuPos[menuLevel]]));

        if (nent->showEntry())
          break;
      }
    }

    //  Otherwise, a negative change, search backward for the previous visible entry.
    if (encoderChange < 0) {
      while (menuPos[menuLevel] > 0) {
        UIMenuEntry *nent = (UIMenuEntry *)pgm_read_word(&(entries[--menuPos[menuLevel]]));

        if (nent->showEntry())
          break;
      }
    }

    shift = -2; // reset shift position  ?????????

    adjustMenuPos();

    return(true);
  }

  //  If in an SD card file listing, same story as a menu page, except every file is visible (well,
  //  technically, silently filtered by the SD card menu itself).
  //
  if (mtype == UI_MENU_TYPE_FILE_SELECTOR) {
    if ((encoderChange > 0) && (menuPos[menuLevel] < nFilesOnCard))
      menuPos[menuLevel]++;

    if ((encoderChange < 0) && (menuPos[menuLevel] > 0))
      menuPos[menuLevel]--;

    if (menuTop[menuLevel] > menuPos[menuLevel])              //  If we back up past the previous top,
      menuTop[menuLevel] = menuPos[menuLevel];                //  reset the first shown item.

    if (menuTop[menuLevel] + UI_ROWS <= menuPos[menuLevel])   //  Likewise, if we go past the bottom,
      menuTop[menuLevel] = menuPos[menuLevel] + 1 - UI_ROWS;  //  reset the first shown item.

    shift = -2; // reset shift position

    return(true);
  }


  //  If in a modification menu reset the action to that specified by the menu.
  //
  if (mtype == UI_MENU_TYPE_MODIFICATION_MENU)
    action = pgm_read_word(&(men->menuAction));
  else
    action = activeAction;









  //  Move!
  //
  if ((action == UI_ACTION_XPOSITION) ||
      (action == UI_ACTION_YPOSITION) ||
      (action == UI_ACTION_ZPOSITION) ||
      (action == UI_ACTION_ZPOSITION_NOTEST)) {
    if (allowMoves == false)
      return false;

    uint8_t  axis = Z_AXIS;

    if (action == UI_ACTION_XPOSITION)   axis = X_AXIS;
    if (action == UI_ACTION_YPOSITION)   axis = Y_AXIS;

    //  Compte a distance to move.
    //
    //  Limit it to some maximum distance.  This only happens when dtActual is 100ms,
    //  and so that is hardcoded.  It used to be Printer::maxFeedrate[axis] * 0.100
    //
    //  From that distance, then compute the number of steps we need to move, resetting
    //  if we somehow end up wanting to move zero steps.
    //
    float distance    = encoderChange * encoderAccel / 100.0;   //  Distance in mm
    float distanceMax = 10.0;

    if ((encoderChange > 0) && (distance > distanceMax))
      distance = distanceMax;

    if ((encoderChange < 0) && (distance < -distanceMax))
      distance = -distanceMax;

    int32_t steps = distance * Printer::axisStepsPerMM[axis];

    if ((encoderChange > 0) && (steps == 0))   steps =  1;
    if ((encoderChange < 0) && (steps == 0))   steps = -1;

    //  Now just move.

    if (action == UI_ACTION_XPOSITION) {
      PrintLine::moveRelativeDistanceInStepsReal(steps, 0, 0, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (action == UI_ACTION_YPOSITION) {
      PrintLine::moveRelativeDistanceInStepsReal(0, steps, 0, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (action == UI_ACTION_ZPOSITION) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (action == UI_ACTION_ZPOSITION_NOTEST) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[axis], false, false);
      Printer::setNoDestinationCheck(true);
    }

    Commands::printCurrentPosition();

    return(true);
  }


  else if (action == UI_ACTION_BED_TARGET) {
    int temp = (int)heatedBedController.targetTemperatureC;  //  is float

    if (temp < UI_SET_MIN_HEATED_BED_TEMP)
      temp = 0;

    if (temp == 0 && encoderChange > 0)
      temp = UI_SET_MIN_HEATED_BED_TEMP;
    else
      temp += encoderChange;

    if (temp < UI_SET_MIN_HEATED_BED_TEMP)
      temp = 0;
    else if (temp > UI_SET_MAX_HEATED_BED_TEMP)
      temp = UI_SET_MAX_HEATED_BED_TEMP;

    Extruder::setHeatedBedTemperature(temp);
  }


  else if (action == UI_ACTION_BED_PREHEAT) {
    int16_t temp = (int)heatedBedController.preheatTemperature + encoderChange;

    if (temp < UI_SET_MIN_HEATED_BED_TEMP)
      temp = UI_SET_MIN_HEATED_BED_TEMP;

    if (temp > UI_SET_MAX_HEATED_BED_TEMP)
      temp = UI_SET_MAX_HEATED_BED_TEMP;

    heatedBedController.preheatTemperature = temp;
  }


  else if (action == UI_ACTION_EXT_TARGET) {
    int temp = (int)extruder[0].tempControl.targetTemperatureC;  //  is float

    if (temp < UI_SET_MIN_EXTRUDER_TEMP)
      temp = 0;

    if (temp == 0 && encoderChange > 0)
      temp = UI_SET_MIN_EXTRUDER_TEMP;
    else
      temp += encoderChange;

    if (temp < UI_SET_MIN_EXTRUDER_TEMP)
      temp = 0;

    if (temp > UI_SET_MAX_EXTRUDER_TEMP)
      temp = UI_SET_MAX_EXTRUDER_TEMP;

    Extruder::setTemperatureForExtruder(temp, 0);
  }


  else if (action == UI_ACTION_EXT_PREHEAT) {
    int16_t temp = extruder[0].tempControl.preheatTemperature + encoderChange;

    if (temp < UI_SET_MIN_EXTRUDER_TEMP)
      temp = UI_SET_MIN_EXTRUDER_TEMP;

    if (temp > UI_SET_MAX_EXTRUDER_TEMP)
      temp = UI_SET_MAX_EXTRUDER_TEMP;

    extruder[0].tempControl.preheatTemperature = temp;
  }


  else if (action == UI_ACTION_FANSPEED) {
    Commands::setFanSpeed(Printer::getFanSpeed() + encoderChange, true);
  }


  else if (action == UI_ACTION_FEEDRATE_MULTIPLY) {
    Commands::changeFeedrateMultiply(Printer::feedrateMultiply + encoderChange);
  }


  else if (action == UI_ACTION_FLOWRATE_MULTIPLY) {
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply + encoderChange);
  }





  ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;

  return(true);
}









#if 0


    case UI_ACTION_FAN2SPEED:
      Commands::setFan2Speed(Printer::getFan2Speed() + increment * 3);
      break;

    case UI_ACTION_MEASURE_ZP_REALZ:
      Printer::wizardStack[0].f += 0.01 * static_cast<float>(increment);
      break;

    case UI_ACTION_EPOSITION:
      if(!allowMoves) return false;
      PrintLine::moveRelativeDistanceInSteps(0, 0, 0, Printer::axisStepsPerMM[E_AXIS]*increment / Printer::extrusionFactor, UI_SET_EXTRUDER_FEEDRATE, true, false, false);
      Commands::printCurrentPosition();
      break;

#if FEATURE_BABYSTEPPING
    case UI_ACTION_Z_BABYSTEPS:
      {
        previousMillisCmd = HAL::timeInMilliseconds();

        int16_t diff =  encoderChange * BABYSTEP_MULTIPLICATOR;   //  DO NOT ACCELERATE!
        if(abs((int)Printer::zBabysteps + diff) < 30000 && abs(diff) < 2000) {
          InterruptProtectedBlock noint;
          Printer::zBabystepsMissing += diff;
          Printer::zBabysteps += diff;
        }
      }

      break;
#endif



    case UI_ACTION_STEPPER_INACTIVE: {
      uint8_t inactT = stepperInactiveTime / 60000;
      INCREMENT_MIN_MAX(inactT, 1, 0, 240);
      stepperInactiveTime = inactT * 60000;
    }
      break;

    case UI_ACTION_MAX_INACTIVE: {
      uint8_t inactT = maxInactiveTime / 60000;
      INCREMENT_MIN_MAX(inactT, 1, 0, 240);
      maxInactiveTime = inactT * 60000;
    }
      break;

    case UI_ACTION_PRINT_ACCEL_X:
    case UI_ACTION_PRINT_ACCEL_Y:
    case UI_ACTION_PRINT_ACCEL_Z:
      INCREMENT_MIN_MAX(Printer::maxAccelerationMMPerSquareSecond[action - UI_ACTION_PRINT_ACCEL_X], 100, 0, 10000);
      Printer::updateDerivedParameter();
      break;

    case UI_ACTION_MOVE_ACCEL_X:
    case UI_ACTION_MOVE_ACCEL_Y:
    case UI_ACTION_MOVE_ACCEL_Z:
      INCREMENT_MIN_MAX(Printer::maxTravelAccelerationMMPerSquareSecond[action - UI_ACTION_MOVE_ACCEL_X], 100, 0, 10000);
      Printer::updateDerivedParameter();
      break;

    case UI_ACTION_MAX_JERK:
      INCREMENT_MIN_MAX(Printer::maxJerk, 0.1, 1, 99.9);
      break;

    case UI_ACTION_STEPS_X:
    case UI_ACTION_STEPS_Y:
    case UI_ACTION_STEPS_Z:
      INCREMENT_MIN_MAX(Printer::axisStepsPerMM[action - UI_ACTION_STEPS_X], 0.1, 0, 999);
      Printer::updateDerivedParameter();
      break;

    case UI_ACTION_XOFF:
    case UI_ACTION_YOFF: {
      float tmp = -Printer::coordinateOffset[action - UI_ACTION_XOFF];
      INCREMENT_MIN_MAX(tmp, 1, -999, 999);
      Printer::coordinateOffset[action - UI_ACTION_XOFF] = -tmp;
    }
      break;

    case UI_ACTION_ZOFF: {
      float tmp = -Printer::coordinateOffset[Z_AXIS];
      INCREMENT_MIN_MAX(tmp, 0.01, -9.99, 9.99);
      Printer::coordinateOffset[Z_AXIS] = -tmp;
    }
      break;

    case UI_ACTION_BAUDRATE:
      {
        int16_t p = 0;
        int32_t rate;
        do {
          rate = pgm_read_dword(&(baudrates[(uint8_t)p]));
          if(rate == baudrate) break;
          p++;
        } while(rate != 0);
        if(rate == 0) p -= 2;
        p += increment;
        if(p < 0) p = 0;
        if(p > static_cast<int16_t>(sizeof(baudrates) / 4) - 2)
          p = sizeof(baudrates) / 4 - 2;
        baudrate = pgm_read_dword(&(baudrates[p]));
      }
      break;

    case UI_ACTION_PID_PGAIN:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidPGain, 0.1, 0, 200);
      break;

    case UI_ACTION_PID_IGAIN:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidIGain, 0.01, 0, 100);
      if(&Extruder::current->tempControl == currHeaterForSetup)
        Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_PID_DGAIN:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidDGain, 0.1, 0, 200);
      break;

    case UI_ACTION_DRIVE_MIN:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidDriveMin, 1, 1, 255);
      break;

    case UI_ACTION_DRIVE_MAX:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidDriveMax, 1, 1, 255);
      break;

    case UI_ACTION_PID_MAX:
      INCREMENT_MIN_MAX(currHeaterForSetup->pidMax, 1, 1, 255);
      break;

    case UI_ACTION_X_OFFSET:
      INCREMENT_MIN_MAX(Extruder::current->xOffset, RMath::max(static_cast<int32_t>(1), static_cast<int32_t>(Printer::axisStepsPerMM[X_AXIS] / 100)), -9999999, 9999999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_Y_OFFSET:
      INCREMENT_MIN_MAX(Extruder::current->yOffset, RMath::max(static_cast<int32_t>(1), static_cast<int32_t>(Printer::axisStepsPerMM[Y_AXIS] / 100)), -9999999, 9999999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_Z_OFFSET:
      INCREMENT_MIN_MAX(Extruder::current->zOffset, RMath::max(static_cast<int32_t>(1), static_cast<int32_t>(Printer::axisStepsPerMM[Z_AXIS] / 100)), -9999999, 9999999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_EXTR_STEPS:
      INCREMENT_MIN_MAX(Extruder::current->stepsPerMM, 0.1, 1, 99999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_EXTR_ACCELERATION:
      INCREMENT_MIN_MAX(Extruder::current->maxAcceleration, 10, 10, 99999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_EXTR_MAX_FEEDRATE:
      INCREMENT_MIN_MAX(Extruder::current->maxFeedrate, 1, 1, 999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_EXTR_START_FEEDRATE:
      INCREMENT_MIN_MAX(Extruder::current->maxStartFeedrate, 1, 1, 999);
      Extruder::selectExtruderById(Extruder::current->id);
      break;

    case UI_ACTION_EXTR_HEATMANAGER:
      INCREMENT_MIN_MAX(currHeaterForSetup->heatManager, 1, 0, 3);
      Printer::setMenuMode(MENU_MODE_FULL_PID, currHeaterForSetup->heatManager == 1); // show PIDS only with PID controller selected
      Printer::setMenuMode(MENU_MODE_DEADTIME, currHeaterForSetup->heatManager == 3);
      break;

    case UI_ACTION_EXTR_WATCH_PERIOD:
      INCREMENT_MIN_MAX(Extruder::current->watchPeriod, 1, 0, 999);
      break;

#if RETRACT_DURING_HEATUP
    case UI_ACTION_EXTR_WAIT_RETRACT_TEMP:
      INCREMENT_MIN_MAX(Extruder::current->waitRetractTemperature, 1, 100, UI_SET_MAX_EXTRUDER_TEMP);
      break;

    case UI_ACTION_EXTR_WAIT_RETRACT_UNITS:
      INCREMENT_MIN_MAX(Extruder::current->waitRetractUnits, 1, 0, 99);
      break;
#endif

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
    case UI_ACTION_ADVANCE_K:
      INCREMENT_MIN_MAX(Extruder::current->advanceK, 1, 0, 200);
      break;
#endif
    case UI_ACTION_ADVANCE_L:
      INCREMENT_MIN_MAX(Extruder::current->advanceL, 1, 0, 600);
      break;
#endif

#if FEATURE_AUTOLEVEL
    case UI_ACTION_AUTOLEVEL2:
      popMenu(true);
      break;
#endif

#if DISTORTION_CORRECTION
    case UI_ACTION_MEASURE_DISTORTION2:
      popMenu(true);
      break;
#endif



#endif
