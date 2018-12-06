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

extern UIDisplay uid;


bool
UIDisplay::doEncoderChange_file(int16_t encoderChange) {

  if ((encoderChange > 0) && (_menuPos < sd._nFilesOnCard + 1))
    _menuPos++;

  if ((encoderChange < 0) && (_menuPos > 0))
    _menuPos--;

  if (_menuTop > _menuPos)              //  If we back up past the previous top,
    _menuTop = _menuPos;                //  reset the first shown item.

  if (_menuTop + UI_ROWS <= _menuPos)   //  Likewise, if we go past the bottom,
    _menuTop = _menuPos + 1 - UI_ROWS;  //  reset the first shown item.

  return(true);
}



bool
UIDisplay::doEncoderChange_entry(int16_t encoderChange) {
  menuPage     *menu        = menuPagePtr(_menuPage);
  uint8_t       entriesLen  = menu->entriesLen();

  //  Search forward/backward for the next visible item.

  if (encoderChange > 0) {
    while (_menuPos + 1 < entriesLen) {
      if (menu->entry(++_menuPos)->visible())
        break;
    }
  }

  if (encoderChange < 0) {
    while (_menuPos > 0) {
      if (menu->entry(--_menuPos)->visible())
        break;
    }
  }

  return(true);
}



//  If a menu selector, move to the next/prev page based on the direction
//  of the encoder.
//
//  By construction, the first displayed item in each page is the menu
//  selector, but that's not necessarily the first entry in the list.  The
//  first page switches between "FILE TO PRINT" and "PRINTING", and those
//  are obviously different entries.
//
//  What this means is that when we change pages, we need to reset both
//  _menuTop and _menuPos, ideally, to the actual entryType_page being
//  shown.  We'll let adjustMenuPos() clean this up for us.

bool
UIDisplay::doEncoderChange_page(int16_t encoderChange) {

  //  Move forward/backward to the next visible menu.

  do {
    if (encoderChange > 0) {
      if (++_menuPage == UI_NUM_PAGES)
        _menuPage = 0;
    }

    if (encoderChange < 0) {
      if (_menuPage-- == 0)
        _menuPage = UI_NUM_PAGES-1;
    }
  } while (menuPagePtr(_menuPage)->visible() == false);


  //  Reset menuTop and menuPos, then find the actual valid menuTop and menuPos.
  //  Not absolutely necessary, adjustMenuPos() should do the same work.

  uint8_t  entriesLen = menuPagePtr(_menuPage)->entriesLen();

  _menuPos = 0;
  _menuSel = 0;
  _menuTop = 0;

  while ((_menuPos < entriesLen) &&
         (menuPagePtr(_menuPage)->entry(_menuPos)->visible() == false)) {
    _menuPos++;
    _menuSel++;
    _menuTop++;
  }

  return(true);
}




//  Called from ui.cpp slowAction().
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

  uint32_t thisEncoderTime = millis();
  uint32_t dtActual        = thisEncoderTime - lastEncoderTime;

  lastEncoderTime = thisEncoderTime;

  if (dtActual < 110)   encoderAccel *= 1.21;
  if (dtActual > 220)   encoderAccel /= 1.66;
  if (dtActual > 999)   encoderAccel  = 1.0;

  if (encoderAccel < 1.0) encoderAccel  = 1.0;

  //  Remember that we actually did something.

  _stopChangeTime = thisEncoderTime + (uint32_t)30 * 1000;
  _stopMenuTime   = thisEncoderTime + (uint32_t)60 * 1000;

  //  Figure out where we're at.

  menuPage  *menu        = menuPagePtr(_menuPage);
  uint8_t    menuType    = menu->type();
  uint8_t    entryType   = menu->entry(_menuPos)->type();
  uint16_t   entryAction = menu->entry(_menuPos)->action();

  //  If there isn't a selected entry, scroll up/down through the menu
  //  entries or file list.

  if (_menuSel == 255) {
    if (menuType == menuType_select)
      doEncoderChange_file(encoderChange);

    else
      doEncoderChange_entry(encoderChange);

    return(true);
  }

  //  Otherwise, maybe we want to change menu pages?

  else if (entryType == entryType_page) {
    doEncoderChange_page(encoderChange);
    return(true);
  }

  //  If a display, do nothing.

  else if (entryType == entryType_displ) {
    return(true);
  }

  //  If a toggle, toggle!  Should never get here, since toggle is handled in action_ok.

  else if (entryType == entryType_toggle) {
    return(true);
  }

  else {
  }

  //  Otherwise, entryType is entryType_action, and we need to do something!


  //  Extruder moves are special cased, but are basically the same.
  //  Well, not really.
  //  I want to make both the distance and extrude rate increase with acceleration.
  //
  if (entryAction == ACT_POS_E) {
    if (allowMoves == false)
      return(false);

    uint8_t axis = E_AXIS;

    float distance    = encoderChange * encoderAccel / 10.0;   //  Distance in mm
    float distanceMax = 1.0;

    if ((encoderChange > 0) && (distance > distanceMax))
      distance = distanceMax;

    if ((encoderChange < 0) && (distance < -distanceMax))
      distance = -distanceMax;

    int32_t steps = distance * Printer::axisStepsPerMM[axis];

    if ((encoderChange > 0) && (steps == 0))   steps =  1;
    if ((encoderChange < 0) && (steps == 0))   steps = -1;

    //  Now just move.

    Com::printf(PSTR("E: change:%d accel:%f distance:%f steps:%ld feedrate:%f -- "),
                encoderChange, encoderAccel, distance, steps, Printer::maxFeedrate[axis]);

    if (entryAction == ACT_POS_E) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, 0, steps, Printer::maxFeedrate[axis] / 10, false, false);
    }

    Commands::printCurrentPosition();

    return(true);
  }



  //  Move!
  //
  if ((entryAction == ACT_POS_X) ||
      (entryAction == ACT_POS_Y) ||
      (entryAction == ACT_POS_Z) ||
      (entryAction == ACT_POS_Z_OPEN)) {
    if (allowMoves == false)
      return(false);

    uint8_t  axis = Z_AXIS;

    if (entryAction == ACT_POS_X)   axis = X_AXIS;
    if (entryAction == ACT_POS_Y)   axis = Y_AXIS;

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

    if (entryAction == ACT_POS_X) {
      PrintLine::moveRelativeDistanceInStepsReal(steps, 0, 0, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (entryAction == ACT_POS_Y) {
      PrintLine::moveRelativeDistanceInStepsReal(0, steps, 0, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (entryAction == ACT_POS_Z) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[axis], false, false);
    }

    if (entryAction == ACT_POS_Z_OPEN) {
      PrintLine::moveRelativeDistanceInStepsReal(0, 0, steps, 0, Printer::maxFeedrate[axis], false, false);
      Printer::setNoDestinationCheck(true);
    }

    Commands::printCurrentPosition();

    return(true);
  }


  else if ((entryAction == ACT_EXT_T_TARGET) ||
           (entryAction == ACT_BED_T_TARGET)) {
    tempControl  *tc = NULL;

    if (entryAction == ACT_EXT_T_TARGET)   tc = &extruderTemp;
    if (entryAction == ACT_BED_T_TARGET)   tc = &bedTemp;

    float temp = tc->getTargetTemperature();
    float mint = tc->getTargetTemperatureMin();
    float maxt = tc->getTargetTemperatureMax();

    //Com::printf(PSTR("setTemp for %d to %.2f <= %.2f <= %.2f  encoderChange = %d\n"),
    //            tc->id(), mint, temp, maxt, encoderChange);

    if ((temp < mint) &&       //  If currently below the min, but a positive
        (encoderChange > 0))   //  change, set to the minimum.
      temp  = mint;            //
    else                       //  Otherwise, add the change to the current temp.
      temp += encoderChange;

    if (temp < mint)           //  If below the min, reset to OFF.
      temp = 0;

    if (temp > maxt)           //  If above the max, reset to max.
      temp = maxt;

    tc->setTargetTemperature(temp);
  }


  else if ((entryAction == ACT_EXTRUDER_FAN_CHANGE) ||
           (entryAction == ACT_LAYER_FAN_CHANGE)) {
    tempControl *tc = NULL;

    if (entryAction == ACT_EXTRUDER_FAN_CHANGE)   tc = &extruderTemp;
    if (entryAction == ACT_LAYER_FAN_CHANGE)      tc = &layerFan;

    uint8_t speed = tc->getFanSpeed();

    if      ((encoderChange < 0) && (speed <   0 - encoderChange))
      tc->setFanSpeed(0);

    else if ((encoderChange > 0) && (speed > 255 - encoderChange))
      tc->setFanSpeed(255);

    else
      tc->setFanSpeed(speed + encoderChange);
  }
    
  else if (entryAction == ACT_PID_P) {
    bedTemp._pidPGain += encoderChange * 0.1;
  }
  else if (entryAction == ACT_PID_I) {
    bedTemp._pidIGain += encoderChange * 0.1;
  }
  else if (entryAction == ACT_PID_D) {
    bedTemp._pidDGain += encoderChange * 0.1;
  }

  else if (entryAction == ACT_SPEED_CHANGE) {
    Commands::changeFeedrateMultiply(Printer::feedrateMultiply + encoderChange);
  }


  else if (entryAction == ACT_FLOW_CHANGE) {
    Commands::changeFlowrateMultiply(Printer::extrudeMultiply + encoderChange);
  }



  return(true);
}
