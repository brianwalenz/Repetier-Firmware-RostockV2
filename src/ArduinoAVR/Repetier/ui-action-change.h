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



bool UIDisplay::nextPreviousAction(int16_t next, bool allowMoves) {
  if(Printer::isUIErrorMessage()) {
    Printer::setUIErrorMessage(false);
    // return true;
  }
  millis_t actTime = HAL::timeInMilliseconds();
  millis_t dtReal;
  millis_t dt = dtReal = actTime - lastNextPrev;
  lastNextPrev = actTime;
  if(dt < SPEED_MAX_MILLIS) dt = SPEED_MAX_MILLIS;
  if(dt > SPEED_MIN_MILLIS) {
    dt = SPEED_MIN_MILLIS;
    lastNextAccumul = 1;
  }
  float f = (float)(SPEED_MIN_MILLIS - dt) / (float)(SPEED_MIN_MILLIS - SPEED_MAX_MILLIS);
  lastNextAccumul = 1.0f + (float)SPEED_MAGNIFICATION * f * f;
#if UI_DYNAMIC_ENCODER_SPEED
  int16_t dynSp = lastNextAccumul / 16;
  if(dynSp < 1)  dynSp = 1;
  if(dynSp > 30) dynSp = 30;
  next *= dynSp;
#endif

  if(menuLevel == 0) {
    lastSwitch = HAL::timeInMilliseconds();
    if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
      menuPos[0]++;
      if(menuPos[0] >= UI_NUM_PAGES)
        menuPos[0] = 0;
    } else {
      menuPos[0] = (menuPos[0] == 0 ? UI_NUM_PAGES - 1 : menuPos[0] - 1);
    }
    return true;
  }
  UIMenu *men = (UIMenu*)menu[menuLevel];
  uint8_t nr = pgm_read_byte(&(men->numEntries));
  uint8_t mtype = HAL::readFlashByte((PGM_P) & (men->menuType)) & 127;
  UIMenuEntry **entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
  UIMenuEntry *ent = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
  UIMenuEntry *testEnt;
  // 0 = Info, 1 = Headline, 2 = sub menu ref, 3 = direct action command
  //uint8_t entType = HAL::readFlashByte((PGM_P)&(ent->entryType));
  unsigned int action = pgm_read_word(&(ent->entryAction));
  if(mtype == UI_MENU_TYPE_SUBMENU && activeAction == 0) { // browse through menu items
    if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
      while(menuPos[menuLevel] + 1 < nr) {
        menuPos[menuLevel]++;
        testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
        if(testEnt->showEntry())
          break;
      }
    } else if(menuPos[menuLevel] > 0) {
      while(menuPos[menuLevel] > 0) {
        menuPos[menuLevel]--;
        testEnt = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
        if(testEnt->showEntry())
          break;
      }
    }
    shift = -2; // reset shift position
    adjustMenuPos();
    return true;
  }

  if(mtype == UI_MENU_TYPE_FILE_SELECTOR) { // SD listing
    if((UI_INVERT_MENU_DIRECTION && next < 0) || (!UI_INVERT_MENU_DIRECTION && next > 0)) {
      menuPos[menuLevel] += 1; // abs(next);
      if(menuPos[menuLevel] > nFilesOnCard) menuPos[menuLevel] = nFilesOnCard;
    } else if(menuPos[menuLevel] > 0) {
      if(menuPos[menuLevel] > 1 /* abs(next) */)
        menuPos[menuLevel] -= 1; // abs(next);
      else
        menuPos[menuLevel] = 0;
    }
    if(menuTop[menuLevel] > menuPos[menuLevel]) {
      menuTop[menuLevel] = menuPos[menuLevel];
    } else if(menuTop[menuLevel] + UI_ROWS <= menuPos[menuLevel]) {
      menuTop[menuLevel] = (menuPos[menuLevel] + 1);
      menuTop[menuLevel] -= static_cast<uint16_t>(UI_ROWS); // DO NOT COMBINE IN ONE LINE - WILL NOT COMPILE CORRECTLY THEN!
    }
    shift = -2; // reset shift position
    return true;
  }

  if(mtype == UI_MENU_TYPE_MODIFICATION_MENU || mtype == UI_MENU_TYPE_WIZARD) action = pgm_read_word(&(men->menuAction));
  else action = activeAction;
  int16_t increment = next;

  switch(action) {
    case UI_ACTION_FANSPEED:
      Commands::setFanSpeed(Printer::getFanSpeed() + increment * 3, true);
      break;
    case UI_ACTION_FAN2SPEED:
      Commands::setFan2Speed(Printer::getFan2Speed() + increment * 3);
      break;
    case UI_ACTION_XPOSITION:
      if(!allowMoves) return false;
#if UI_SPEEDDEPENDENT_POSITIONING
      {
        float d = 0.01 * (float)increment * lastNextAccumul;
        if(fabs(d) * 1000 > Printer::maxFeedrate[X_AXIS] * dtReal)
          d *= Printer::maxFeedrate[X_AXIS] * dtReal / (1000 * fabs(d));
        long steps = (long)(d * Printer::axisStepsPerMM[X_AXIS]);
        steps = ( increment < 0 ? RMath::min(steps, (long)increment) : RMath::max(steps, (long)increment));
        PrintLine::moveRelativeDistanceInStepsReal(steps, 0, 0, 0, Printer::maxFeedrate[X_AXIS], false, false);
      }
#else
      PrintLine::moveRelativeDistanceInStepsReal(increment, 0, 0, 0, Printer::homingFeedrate[X_AXIS], false, false);
#endif
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_YPOSITION:
      if(!allowMoves) return false;
#if UI_SPEEDDEPENDENT_POSITIONING
      {
        float d = 0.01 * (float)increment * lastNextAccumul;
        if(fabs(d) * 1000 > Printer::maxFeedrate[Y_AXIS] * dtReal)
          d *= Printer::maxFeedrate[Y_AXIS] * dtReal / (1000 * fabs(d));
        long steps = (long)(d * Printer::axisStepsPerMM[Y_AXIS]);
        steps = ( increment < 0 ? RMath::min(steps, (long)increment) : RMath::max(steps, (long)increment));
        PrintLine::moveRelativeDistanceInStepsReal(0, steps, 0, 0, Printer::maxFeedrate[Y_AXIS], false, false);
      }
#else
      PrintLine::moveRelativeDistanceInStepsReal(0, increment, 0, 0, Printer::homingFeedrate[Y_AXIS], false, false);
#endif
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_ZPOSITION_NOTEST:
      if(!allowMoves) return false;
      Printer::setNoDestinationCheck(true);
      goto ZPOS1;
    case UI_ACTION_ZPOSITION:
      if(!allowMoves) return false;
    ZPOS1:
#if UI_SPEEDDEPENDENT_POSITIONING
      {
        float d = 0.01 * (float)increment * lastNextAccumul;
        if(fabs(d) * 1000 > Printer::maxFeedrate[Z_AXIS] * dtReal)
          d *= Printer::maxFeedrate[Z_AXIS] * dtReal / (1000 * fabs(d));
        long steps = (long)(d * Printer::axisStepsPerMM[Z_AXIS]);
        steps = ( increment < 0 ? RMath::min(steps, (long)increment) : RMath::max(steps, (long)increment));
        PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[Z_AXIS], false, false);
      }
#else
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, ((long)increment * Printer::axisStepsPerMM[Z_AXIS]) / 100, 0, Printer::homingFeedrate[Z_AXIS], false, false);
#endif
      Printer::setNoDestinationCheck(false);
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_XPOSITION_FAST:
      if(!allowMoves) return false;
      PrintLine::moveRelativeDistanceInStepsReal(Printer::axisStepsPerMM[X_AXIS] * increment, 0, 0, 0, Printer::homingFeedrate[X_AXIS], true, false);
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_YPOSITION_FAST:
      if(!allowMoves) return false;
      PrintLine::moveRelativeDistanceInStepsReal(0, Printer::axisStepsPerMM[Y_AXIS] * increment, 0, 0, Printer::homingFeedrate[Y_AXIS], true, false);
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_ZPOSITION_FAST_NOTEST:
      if(!allowMoves) return false;
      Printer::setNoDestinationCheck(true);
      goto ZPOS2;
    case UI_ACTION_ZPOSITION_FAST:
      if(!allowMoves) return false;
    ZPOS2:
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, Printer::axisStepsPerMM[Z_AXIS] * increment, 0, Printer::homingFeedrate[Z_AXIS], true, false);
      Printer::setNoDestinationCheck(false);
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_MEASURE_ZP_REALZ:
      Printer::wizardStack[0].f += 0.01 * static_cast<float>(increment);
      break;
    case UI_ACTION_EPOSITION:
      if(!allowMoves) return false;
      PrintLine::moveRelativeDistanceInSteps(0, 0, 0, Printer::axisStepsPerMM[E_AXIS]*increment / Printer::extrusionFactor, UI_SET_EXTRUDER_FEEDRATE, true, false, false);
      Commands::printCurrentPosition();
      break;
    case UI_ACTION_Z_BABYSTEPS:

#if FEATURE_BABYSTEPPING
      {
        previousMillisCmd = HAL::timeInMilliseconds();

#if UI_DYNAMIC_ENCODER_SPEED
        increment /= dynSp; // we need fixed speeds or we get in trouble here!
#endif

        int16_t diff =  increment * BABYSTEP_MULTIPLICATOR;
        if(abs((int)Printer::zBabysteps + diff) < 30000 && abs(diff) < 2000) {
          InterruptProtectedBlock noint;
          Printer::zBabystepsMissing += diff;
          Printer::zBabysteps += diff;
        }
      }

      break;
#endif

      //  nextPrevious()
      //  Set heated bed / extruder target temperature.


      //  Set target temperature for bed
    case UI_ACTION_BED_TARGET:
      {
        int temp = (int)heatedBedController.targetTemperatureC;

        if (temp < UI_SET_MIN_HEATED_BED_TEMP)
          temp = 0;

        if (temp == 0 && increment > 0)
          temp = UI_SET_MIN_HEATED_BED_TEMP;
        else
          temp += increment;

        if (temp < UI_SET_MIN_HEATED_BED_TEMP)
          temp = 0;
        else if (temp > UI_SET_MAX_HEATED_BED_TEMP)
          temp = UI_SET_MAX_HEATED_BED_TEMP;

        Extruder::setHeatedBedTemperature(temp);
      }
      break;

      //  Set preheat temperature for bed
    case UI_ACTION_BED_PREHEAT:
      {
        int temp = (int)heatedBedController.preheatTemperature + increment;

        if (temp < UI_SET_MIN_HEATED_BED_TEMP)
          temp = UI_SET_MIN_HEATED_BED_TEMP;

        if (temp > UI_SET_MAX_HEATED_BED_TEMP)
          temp = UI_SET_MAX_HEATED_BED_TEMP;

        heatedBedController.preheatTemperature = static_cast<int16_t>(temp);
      }
      break;

      //  Set target temperature for extruder
    case UI_ACTION_EXT_TARGET:
      {
        int temp = (int)extruder[0].tempControl.targetTemperatureC;

        if (temp < UI_SET_MIN_EXTRUDER_TEMP)
          temp = 0;

        if (temp == 0 && increment > 0)
          temp = UI_SET_MIN_EXTRUDER_TEMP;
        else
          temp += increment;

        if (temp < UI_SET_MIN_EXTRUDER_TEMP)
          temp = 0;

        if (temp > UI_SET_MAX_EXTRUDER_TEMP)
          temp = UI_SET_MAX_EXTRUDER_TEMP;

        Extruder::setTemperatureForExtruder(temp, 0);
      }
      break;

      //  Set preheat temperature for extruder
    case UI_ACTION_EXT_PREHEAT:
      {
        int temp = (int)extruder[0].tempControl.preheatTemperature + increment;

        if (temp < UI_SET_MIN_EXTRUDER_TEMP)
          temp = UI_SET_MIN_EXTRUDER_TEMP;

        if (temp > UI_SET_MAX_EXTRUDER_TEMP)
          temp = UI_SET_MAX_EXTRUDER_TEMP;

        extruder[0].tempControl.preheatTemperature = static_cast<int16_t>(temp);
      }
      break;






    case UI_ACTION_FEEDRATE_MULTIPLY: {
      int fr = Printer::feedrateMultiply;
      INCREMENT_MIN_MAX(fr, 1, 25, 500);
      Commands::changeFeedrateMultiply(fr);
    }
      break;
    case UI_ACTION_FLOWRATE_MULTIPLY: {
      INCREMENT_MIN_MAX(Printer::extrudeMultiply, 1, 25, 500);
      Commands::changeFlowrateMultiply(Printer::extrudeMultiply);
    }
      break;
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
    case UI_ACTION_HOMING_FEEDRATE_X:
    case UI_ACTION_HOMING_FEEDRATE_Y:
    case UI_ACTION_HOMING_FEEDRATE_Z:
      INCREMENT_MIN_MAX(Printer::homingFeedrate[action - UI_ACTION_HOMING_FEEDRATE_X], 1, 1, 1000);
      break;

    case UI_ACTION_MAX_FEEDRATE_X:
    case UI_ACTION_MAX_FEEDRATE_Y:
    case UI_ACTION_MAX_FEEDRATE_Z:
      INCREMENT_MIN_MAX(Printer::maxFeedrate[action - UI_ACTION_MAX_FEEDRATE_X], 1, 1, 1000);
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
    default:
      break;
  }

  ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;

  return true;
}



