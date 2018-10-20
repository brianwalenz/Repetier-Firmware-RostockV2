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



void UIDisplay::finishAction(unsigned int action) {

  switch(action) {
    case UI_ACTION_BED_PREHEAT:
      {
        HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP, heatedBedController.preheatTemperature);
        EEPROM::updateChecksum();
      }
      break;

    case UI_ACTION_EXT_PREHEAT:
      {
        int i = 0;
        int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
        Extruder *e = &extruder[i];
        HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT, e->tempControl.preheatTemperature);
        EEPROM::updateChecksum();
      }
      break;
  }
}





// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.

int
UIDisplay::executeAction(unsigned int action, bool allowMoves) {
  int ret = 0;

  if(action & UI_ACTION_TOPMENU) { // Go to start menu
    menuLevel = 0;
  }

  action &= 8191; // strip out higher level flags

  if(action >= 2000 && action < 3000) {
    setStatusP(PSTR("Action:%la"));
  }

  switch(action) {
    case UI_ACTION_OK:
      ret = okAction(allowMoves);
      break;

    case UI_ACTION_BACK:
      popMenu(false);
      break;

    case UI_ACTION_MESSAGE:
      popMenu(true);
      break;

    case UI_ACTION_NEXT:
      if(!doEncoderChange(1, allowMoves))
        ret = UI_ACTION_NEXT;
      break;

    case UI_ACTION_PREVIOUS:
      if(!doEncoderChange(-1, allowMoves))
        ret = UI_ACTION_PREVIOUS;
      break;

    case UI_ACTION_MENU_UP:
      if(menuLevel > 0) menuLevel--;
      break;

    case UI_ACTION_TOP_MENU:
      menuLevel = 0;
      break;

    case UI_ACTION_EMERGENCY_STOP:
      Commands::emergencyStop();
      break;

    case UI_ACTION_HOME_ALL:
      if (!allowMoves)
        return UI_ACTION_HOME_ALL;

      Printer::homeAxis(true, true, true);
      Commands::printCurrentPosition();

      break;

    case UI_ACTION_SET_ORIGIN:
      if(!allowMoves) return UI_ACTION_SET_ORIGIN;
      Printer::setOrigin(-Printer::currentPosition[X_AXIS], -Printer::currentPosition[Y_AXIS], -Printer::currentPosition[Z_AXIS]);
      break;

    case UI_ACTION_DEBUG_ECHO:
      Printer::toggleEcho();
      break;

    case UI_ACTION_DEBUG_INFO:
      Printer::toggleInfo();
      break;

    case UI_ACTION_DEBUG_ERROR:
      Printer::toggleErrors();
      break;

    case UI_ACTION_DEBUG_ENDSTOP:
      Printer::toggleEndStop();
      break;

    case UI_ACTION_DEBUG_DRYRUN:
      Printer::toggleDryRun();
      if(Printer::debugDryrun()) { // simulate movements without printing
        for(int i = 0; i < NUM_EXTRUDER; i++)
          Extruder::setTemperatureForExtruder(0, i);
        Extruder::setHeatedBedTemperature(0);
      }
      break;

    case UI_ACTION_POWER:
#if PS_ON_PIN >= 0 // avoid compiler errors when the power supply pin is disabled
      {
        GCode gc; //not fully initialized but processMCode only needs M set for commands 80/81
        gc.M = Printer::isPowerOn() ? 81 : 80;
        Commands::processMCode(&gc);
      }
#endif
      break;


    case UI_ACTION_BED_PREHEAT_ON:
      //  Turn on BED preheat.
      Printer::setMenuMode(MENU_MODE_BED_HEAT, true);
      Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature);
      break;
    case UI_ACTION_BED_PREHEAT_OFF:
      //  Turn off BED preheat.
      Printer::setMenuMode(MENU_MODE_BED_HEAT, false);
      Extruder::setHeatedBedTemperature(0);
      break;

    case UI_ACTION_EXT_PREHEAT_ON:
      //  Turn on EXT preheat.
      Printer::setMenuMode(MENU_MODE_EXT_HEAT, true);
      Extruder::setTemperatureForExtruder(extruder[0].tempControl.preheatTemperature, 0);
      break;
    case UI_ACTION_EXT_PREHEAT_OFF:
      //  Turn off EXT preheat.
      Printer::setMenuMode(MENU_MODE_EXT_HEAT, false);
      Extruder::setTemperatureForExtruder(0, 0);
      break;

    case UI_ACTION_COOLDOWN:
      //  UI_ACTION_BED_PREHEAT_OFF
      Printer::setMenuMode(MENU_MODE_BED_HEAT, false);
      Extruder::setHeatedBedTemperature(0);

      //  UI_ACTION_EXT_PREHEAT_OFF
      Printer::setMenuMode(MENU_MODE_EXT_HEAT, false);
      Extruder::setTemperatureForExtruder(0, 0);
      break;



    case UI_ACTION_DISABLE_STEPPER:
      Printer::kill(true);
      break;
    case UI_ACTION_RESET_EXTRUDER:
      Printer::currentPositionSteps[E_AXIS] = 0;
      break;
    case UI_ACTION_EXTRUDER_RELATIVE:
      Printer::relativeExtruderCoordinateMode = !Printer::relativeExtruderCoordinateMode;
      break;
    case UI_ACTION_SELECT_EXTRUDER0:
      if(!allowMoves) return action;
      Extruder::selectExtruderById(static_cast<uint8_t>(action - UI_ACTION_SELECT_EXTRUDER0));
      currHeaterForSetup = &(Extruder::current->tempControl);
      Printer::setMenuMode(MENU_MODE_FULL_PID, currHeaterForSetup->heatManager == 1);
      Printer::setMenuMode(MENU_MODE_DEADTIME, currHeaterForSetup->heatManager == 3);
      break;
    case UI_ACTION_STORE_EEPROM:
      EEPROM::storeDataIntoEEPROM(false);
      pushMenu(&ui_menu_eeprom_saved, false);
      uiAlert();
      break;
    case UI_ACTION_LOAD_EEPROM:
      EEPROM::readDataFromEEPROM(true);
      Extruder::selectExtruderById(Extruder::current->id);
      pushMenu(&ui_menu_eeprom_loaded, false);
      uiAlert();
      break;

    case UI_ACTION_SD_DELETE:
      if(sd.sdactive) {
        pushMenu(&ui_menu_sd_fileselector, false);
      }
      break;
    case UI_ACTION_SD_PRINT:
      if(sd.sdactive) {
        pushMenu(&ui_menu_sd_fileselector, false);
      }
      break;
    case UI_ACTION_SD_PAUSE:
      if(!allowMoves)
        ret = UI_ACTION_SD_PAUSE;
      else
        sd.pausePrint(true);
      break;
    case UI_ACTION_SD_CONTINUE:
      if(!allowMoves) ret = UI_ACTION_SD_CONTINUE;
      else sd.continuePrint(true);
      break;
    case UI_ACTION_SD_PRI_PAU_CONT:
      if(!allowMoves) ret = UI_ACTION_SD_PRI_PAU_CONT;
      else {
        if(Printer::isMenuMode(MENU_MODE_PRINTING + MENU_MODE_PAUSED))
          sd.continuePrint(true);
        else if(Printer::isMenuMode(MENU_MODE_PRINTING))
          sd.pausePrint(true);
        else if(sd.sdactive)
          pushMenu(&ui_menu_sd_fileselector, false);
      }
      break;
    case UI_ACTION_SD_STOP:
      if(!allowMoves) ret = UI_ACTION_SD_STOP;
      else sd.stopPrint();
      break;
    case UI_ACTION_SD_UNMOUNT:
      sd.unmount();
      break;
    case UI_ACTION_SD_MOUNT:
      sd.mount();
      break;
    case UI_ACTION_MENU_SDCARD:
      //pushMenu(&ui_menu_sd, false);
      break;


    case UI_ACTION_STOP:
      Printer::stopPrint();
      break;
    case UI_ACTION_CONTINUE:
      Printer::continuePrint();
      break;


    case UI_ACTION_IGNORE_M106:
      Printer::flag2 ^= PRINTER_FLAG2_IGNORE_M106_COMMAND;
      break;


#if MAX_HARDWARE_ENDSTOP_Z
    case UI_ACTION_SET_MEASURED_ORIGIN: {
      Printer::updateCurrentPosition();
      Printer::zLength -= Printer::currentPosition[Z_AXIS];
      Printer::currentPositionSteps[Z_AXIS] = 0;
      Printer::updateDerivedParameter();
      transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
      Printer::updateCurrentPosition(true);
      Com::printFLN(PSTR("Printer height:"), Printer::zLength);
      EEPROM::storeDataIntoEEPROM(false);
      Com::printFLN(PSTR("EEPROM updated"));
      Commands::printCurrentPosition();
    }
      break;
#endif
    case UI_ACTION_SET_P1:
#if SOFTWARE_LEVELING
      for (uint8_t i = 0; i < 3; i++) {
        Printer::levelingP1[i] = Printer::currentPositionSteps[i];
      }
#endif
      break;
    case UI_ACTION_SET_P2:
#if SOFTWARE_LEVELING
      for (uint8_t i = 0; i < 3; i++) {
        Printer::levelingP2[i] = Printer::currentPositionSteps[i];
      }
#endif
      break;
    case UI_ACTION_SET_P3:
#if SOFTWARE_LEVELING
      for (uint8_t i = 0; i < 3; i++) {
        Printer::levelingP3[i] = Printer::currentPositionSteps[i];
      }
#endif
      break;
    case UI_ACTION_CALC_LEVEL:
#if SOFTWARE_LEVELING
      int32_t factors[4];
      PrintLine::calculatePlane(factors, Printer::levelingP1, Printer::levelingP2, Printer::levelingP3);
      Com::printFLN(PSTR("Leveling calc:"));
      Com::printFLN(PSTR("Tower 1:"), PrintLine::calcZOffset(factors, Printer::deltaAPosXSteps, Printer::deltaAPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
      Com::printFLN(PSTR("Tower 2:"), PrintLine::calcZOffset(factors, Printer::deltaBPosXSteps, Printer::deltaBPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
      Com::printFLN(PSTR("Tower 3:"), PrintLine::calcZOffset(factors, Printer::deltaCPosXSteps, Printer::deltaCPosYSteps) * Printer::invAxisStepsPerMM[Z_AXIS]);
#endif
      break;

#if FEATURE_Z_PROBE
    case UI_ACTION_MEASURE_ZPROBE_HEIGHT:
      Printer::wizardStackPos = 0;
      Printer::wizardStack[0].f = Printer::currentPosition[Z_AXIS];
      uid.pushMenu(&ui_menu_mzp, true);
      break;
    case UI_ACTION_MEASURE_ZPROBE_HEIGHT2:
      Printer::measureZProbeHeight(Printer::wizardStack[0].f);
      uid.popMenu(true);
      break;
#endif

    case UI_ACTION_KILL:
      Commands::emergencyStop();
      break;
    case UI_ACTION_RESET:
      HAL::resetHardware();
      break;
    case UI_ACTION_PAUSE:
      Printer::pausePrint();
      //Com::printFLN(PSTR("RequestPause:"));
      break;

#if FEATURE_AUTOLEVEL
    case UI_ACTION_AUTOLEVEL_ONOFF:
      Printer::setAutolevelActive(!Printer::isAutolevelActive());
      break;
#endif

#ifdef DEBUG_PRINT
    case UI_ACTION_WRITE_DEBUG:
      Com::printF(PSTR("Buf. Read Idx:"), (int)GCode::bufferReadIndex);
      Com::printF(PSTR(" Buf. Write Idx:"), (int)GCode::bufferWriteIndex);
      Com::printF(PSTR(" Comment:"), (int)GCode::commentDetected);
      Com::printF(PSTR(" Buf. Len:"), (int)GCode::bufferLength);
      Com::printF(PSTR(" Wait resend:"), (int)GCode::waitingForResend);
      Com::printFLN(PSTR(" Recv. Write Pos:"), (int)GCode::commandsReceivingWritePosition);
      //Com::printF(PSTR("Min. XY Speed:"),Printer::minimumSpeed);
      //Com::printF(PSTR(" Min. Z Speed:"),Printer::minimumZSpeed);
      Com::printF(PSTR(" Buffer:"), PrintLine::linesCount);
      Com::printF(PSTR(" Lines pos:"), (int)PrintLine::linesPos);
      Com::printFLN(PSTR(" Write Pos:"), (int)PrintLine::linesWritePos);
      Com::printFLN(PSTR("Wait loop:"), debugWaitLoop);
      Com::printF(PSTR("sd mode:"), (int)sd.sdmode);
      Com::printF(PSTR(" pos:"), sd.sdpos);
      Com::printFLN(PSTR(" of "), sd.filesize);
      break;
#endif

    case UI_ACTION_TEMP_DEFECT:
      Printer::setAnyTempsensorDefect();
      break;

#if FEATURE_AUTOLEVEL
    case UI_ACTION_AUTOLEVEL:
      //uid.pushMenu(&ui_msg_clearbed, true);  //  "be sure the heated bed is clear of any obstructions"
      break;
#endif

#if DISTORTION_CORRECTION
    case UI_ACTION_MEASURE_DISTORTION:
      //uid.pushMenu(&ui_msg_clearbed, true);
      break;
    case UI_ACTION_TOGGLE_DISTORTION:
      if(Printer::distortion.isEnabled())
        Printer::distortion.disable(true);
      else
        Printer::distortion.enable(true);
      break;
#endif
    default:
      break;
  }
  refreshPage();

  ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;

  return ret;
}



