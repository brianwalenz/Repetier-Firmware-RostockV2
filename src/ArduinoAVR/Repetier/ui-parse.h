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



//
// NOT DISPLAYING MENUS CORRECTLY - is this cache?
//

//
// WHY IS THE CURSOR STILL FUCKING BLINKING??
//

//
// IT ALSO SEEMS TO CALL initDisplay TWICE - the 'bri' scroller goes twice
//

//
// PROBABLY NEED TO FIGURE OUT SERIAL LOGGING
//



void
UIDisplay::parse(const char *txt, bool ram) {
  float          fvalue = 0;

  while(col < MAX_COLS) {
    char c0 = (ram) ? (*txt++) : (pgm_read_byte(txt++));

    //  Finished?
    if (c0 == 0)
      break;

    //  If the character isn't a format symbol, insert it verbatim.
    if (c0 != '%') {
      addChar(c0);
      continue;
    }

    //  Otherwise, parse the format string and add the requested value.
    //
    //  One control sequence '%%' does not use c2.  txt is decremented
    //  when that code is processed.

    char c1 = (ram) ? (*txt++) : (pgm_read_byte(txt++));
    char c2 = (ram) ? (*txt++)   : (pgm_read_byte(txt++));


    if      (c1 == '%') {
      addChar('%');
      txt--;   //  Because c2 was not used.
    }

    //  If %?x add an 'x' if the last character isn't 'x'.
    else if (c1 == '?') {
      if ((col > 0) && (printCols[col-1] != c2))
        addChar(c2);
    }

    //  Acceleration settings
    else if ((c1 == 'a') && ('x' <= c2) && (c2 <= 'z')) {
      addFloat(Printer::maxAccelerationMMPerSquareSecond[c2 - 'x'], 5, 0);
    }

    else if ((c1 == 'a') && ('X' <= c2) && (c2 <= 'Z')) {
      addFloat(Printer::maxTravelAccelerationMMPerSquareSecond[c2 - 'X'], 5, 0);
    }

    else if ((c1 == 'a') && (c2 == 'j')) {
      addFloat(Printer::maxJerk, 3, 1);
    }


    //  Extruder temperature
    //    eI - extruder temperature        %3d
    //    ec - extruder temperature        %5.1f
    //    e  - extruder temperature preset %3d
    //
    //    er - relative mode?
    //
    //    eb - build plate
    //    eB - build plate integer
    //
    else if ((c1 == 'e') && (c2 == 'I')) {
      //alue = extruder[c2 - '0'].tempControl.currentTemperatureC;
      fvalue = Extruder::current->tempControl.currentTemperatureC;
      addFloat(fvalue, 3, 0);
    }

    else if ((c1 == 'e') && (c2 == 'c')) {
      //alue = extruder[c2 - '0'].tempControl.currentTemperatureC;
      fvalue = Extruder::current->tempControl.currentTemperatureC;
      addFloat(fvalue, 3, 1);
    }

    else if ((c1 == 'e') && (c2 == 'B')) {
      fvalue = Extruder::getHeatedBedTemperature();
      addFloat(fvalue, 3, 1);
    }

    //    er - extruder in relative mode?
    else if ((c1 == 'e') && (c2 == 'r')) {
      addStringYesNo(Printer::relativeExtruderCoordinateMode);
    }

    //  Target temperatures
    //    Ec - extruder target
    //    Eb - build plate target
    //
    else if ((c1 == 'E') && (c2 == 'c')) {
      //alue = extruder[c2 - '0'].tempControl.targetTemperatureC;
      fvalue = Extruder::current->tempControl.targetTemperatureC;
      addFloat(fvalue, 3, 0 /*UI_TEMP_PRECISION*/);
    }

    else if ((c1 == 'E') && (c2 == 'b')) {
      fvalue = heatedBedController.targetTemperatureC;
      addFloat(fvalue, 3, 0 /*UI_TEMP_PRECISION*/);
    }

    //  Preheating
    //
    else if ((c1 == 'p') && (c2 == 'c')) {
      //dNumber(extruder[c2 - '0'].tempControl.preheatTemperature, 3, ' ');
      addNumber(Extruder::current->tempControl.preheatTemperature, 3, ' ');
    }

    else if ((c1 == 'p') && (c2 == 'b')) {
      addNumber(heatedBedController.preheatTemperature, 3, ' ');
    }






    // Endstop positions
    else if ((c1 == 's') && (c2 == 'x')) {
      addStringOnOff(Endstops::xMin());
    }

    else if ((c1 == 's') && (c2 == 'X')) {
      addStringOnOff(Endstops::xMax());
    }

    else if ((c1 == 's') && (c2 == 'y')) {
      addStringOnOff(Endstops::yMin());
    }

    else if ((c1 == 's') && (c2 == 'Y')) {
      addStringOnOff(Endstops::yMax());
    }

    else if ((c1 == 's') && (c2 == 'z')) {
      addStringOnOff(Endstops::zMin());
    }

    else if ((c1 == 's') && (c2 == 'Z')) {
      addStringOnOff(Endstops::zMax());
    }

    else if ((c1 == 's') && (c2 == 'P')) {
      addStringOnOff(Endstops::zProbe());
    }


    //  Fan speeds
    else if ((c1 == 'F') && (c2 == 's')) {
      addNumber(floor(Printer::getFanSpeed() * 100 / 255 + 0.5f), 3);
    }

    else if ((c1 == 'F') && (c2 == 'S')) {
      addNumber(floor(Printer::getFan2Speed() * 100 / 255 + 0.5f), 3);
    }

    else if ((c1 == 'F') && (c2 == 'i')) {
      addStringP((Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND) ? ui_selected : ui_unselected);
    }


    //  Feedrates

    else if ((c1 == 'f') && ('x' <= c2) && (c2 <= 'z')) {
      addFloat(Printer::maxFeedrate[c2 - 'x'], 5, 0);
    }

    else if ((c1 == 'f') && ('X' <= c2) && (c2 <= 'Z')) {
      addFloat(Printer::homingFeedrate[c2 - 'X'], 5, 0);
    }


    //'is') addNumber(stepperInactiveTime / 60000, 3);
    //'ip') addNumber(maxInactiveTime / 60000, 3);


    //  Print state

    else if ((c1 == 'P') && (c2 == 'n')) {   //  Name of the file being printed
      addString(Printer::printName);
    }

    else if ((c1 == 'P') && (c2 == 'l')) {   //  Current layer number
      addNumber(Printer::currentLayer, 0);
    }

    else if ((c1 == 'P') && (c2 == 'L')) {   //  Maximum layer number
      addNumber(Printer::maxLayer, 0);
    }

    else if ((c1 == 'P') && (c2 == 'p')) {   //  Print Progress
      addFloat(Printer::progress, 3, 1);
    }


    //  Status messages

    else if ((c1 == 'o') && (c2 == 's')) {
          if(sd.sdactive && sd.sdmode && !statusMsg[0]) {
            addStringP(PSTR("SD_Printed: "));
            float percent;
            if(sd.filesize < 2000000) percent = sd.sdpos * 100.0 / sd.filesize;
            else percent = (sd.sdpos >> 8) * 100.0 / (sd.filesize >> 8);
            addFloat(percent, 3, 1);
            if(col < MAX_COLS)
              printCols[col++] = '%';
          } else
            {
              parse(statusMsg, true);
            }
    }

    else if ((c1 == 'o') && (c2 == 'c')) {
          addNumber(baudrate, 6);
    }

    else if ((c1 == 'o') && (c2 == 'e')) {
          if(errorMsg != 0) addStringP((char PROGMEM *)errorMsg);
    }

    else if ((c1 == 'o') && (c2 == 'B')) {
          addNumber((int)PrintLine::linesCount, 2);
    }

    else if ((c1 == 'o') && (c2 == 'f')) {
          addNumber(Printer::extrudeMultiply, 3);
    }

    else if ((c1 == 'o') && (c2 == 'm')) {
          addNumber(Printer::feedrateMultiply, 3);
    }

    else if ((c1 == 'o') && (c2 == 'n')) {
          addNumber(Extruder::current->id + 1, 1);
    }

    else if ((c1 == 'o') && (c2 == 'Y')) {
          addFloat(static_cast<float>(Printer::zBabysteps) * Printer::invAxisStepsPerMM[Z_AXIS], 2, 2);
    }

    else if ((c1 == 'o') && (c2 == 'p')) {
    }

    else if ((c1 == 'o') && (c2 == 'b')) {
      addNumber(pwm_pos[heatedBedController.pwmIndex] * 100 / 255, 3);
      addChar('%');
    }

    else if ((c1 == 'o') && (c2 == 'C')) {
      addNumber(pwm_pos[Extruder::current->id] * 100 / 255, 3);
      addChar('%');
    }


    //  Usage
    //
    //  Previous version would try to add in the time of the current
    //  print job, but it looked bogus.
    //
    //  Would be nice if there was a report of the time of the current print,
    //  not just historical times.

    else if ((c1 == 'U') && (c2 == 't')) {   //  Printing time in HH:MM
      int32_t secs = HAL::eprGetInt32(EPR_PRINTING_TIME);

      int32_t days  = secs / 86400;  secs -= days  * 86400;
      int32_t hours = secs / 3600;   secs -= hours * 3600;
      int32_t mins  = secs / 60;

      addNumber(days, 1);
      addStringP(PSTR(" days "));
      addNumber(hours, 2, '0');
      addStringP(PSTR(":"));
      addNumber(mins, 2, '0');
    }

    else if ((c1 == 'U') && (c2 == 'h')) {   // Printing time in hours
      int32_t seconds = HAL::eprGetInt32(EPR_PRINTING_TIME);

      addNumber(seconds / 3600, 5);
    }

    else if ((c1 == 'U') && (c2 == 'f')) {   //  Filament usage
      float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);

      addFloat(dist, (dist > 9999 ? 6 : 4), (dist > 9999 ? 0 : 1));
    }

    else if ((c1 == 'U') && (c2 == 'k')) {   //  Filament usage in km
      float dist = (Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE)) * 0.001;

      addFloat(dist, (dist > 999 ? 5 : 3), (dist > 9999 ? 1 : 2));
    }










    else {
      addStringP(PSTR("UNKNOWN"));
      addChar(c1);
      addChar(c2);
    }
  }


  printCols[col] = 0;
}





#if 0

      case 'S':
        if(c2 >= 'x' && c2 <= 'z') addFloat(Printer::axisStepsPerMM[c2 - 'x'], 3, 1);
        if(c2 == 'e') addFloat(Extruder::current->stepsPerMM, 3, 1);
        break;


      case 'T': // Print offsets
        if(c2 == '2')
          addFloat(-Printer::coordinateOffset[Z_AXIS], 2, 2);
        else
          addFloat(-Printer::coordinateOffset[c2 - '0'], 4, 0);
        break;





      case 'x':
        if(c2 >= '0' && c2 <= '7') {
          if(c2 == '4') { // this sequence save 14 bytes of flash
            addFloat(Printer::filamentPrinted * 0.001, 3, 2);
            break;
          }

          if((c2 >= '0' && c2 <= '2') || (c2 >= '5' && c2 <= '7')) {
            if(Printer::isHoming()) {
              addStringP(PSTR(" Homing"));
              break;
            } else {
              if (Printer::isAnimation() &&
                  ((c2 == '0' && !Printer::isXHomed()) ||
                   (c2 == '1' && !Printer::isYHomed()) ||
                   (c2 == '2' && !Printer::isZHomed()))) {
                addStringP(PSTR("   z.??"));
                break;
              }
            }
          }
          if(c2 == '0')
            fvalue = Printer::realXPosition();
          else if(c2 == '1')
            fvalue = Printer::realYPosition();
          else if(c2 == '2')
            fvalue = Printer::realZPosition();

          //################ Workpiece Coordinates#########################################################

          else if(c2 == '5')
            fvalue = Printer::currentPosition[X_AXIS] + Printer::coordinateOffset[X_AXIS];
          else if(c2 == '6')
            fvalue = Printer::currentPosition[Y_AXIS] + Printer::coordinateOffset[Y_AXIS];
          else if(c2 == '7')
            fvalue = Printer::currentPosition[Z_AXIS] + Printer::coordinateOffset[Z_AXIS];

          //############ End Workpiece Coordinates #########################################################

          else
            fvalue = (float)Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];

          addFloat(fvalue, 4, 2);
        }

        else if(c2 >= 'a' && c2 <= 'f') {
          //  %xa-%xf : Extruder state icon 0x08 or 0x09 or 0x0a (off) - works only with graphic displays!
          fast8_t exid = c2 - 'a';
          TemperatureController &t = extruder[exid].tempControl;
          if(t.targetTemperatureC < 30)
            addChar(0x0a);
          else
            addChar((t.currentTemperatureC + 4 < t.targetTemperatureC) && Printer::isAnimation() ? 0x08 : 0x09);
          break;
        }
        else if(c2 == 'B') {
          //  %xB : Bed icon state 0x0c or 0x0d or 0x0b (off) Bed state - works only with graphic displays!
          if(heatedBedController.targetTemperatureC < 30)
            addChar(0x0b);
          else
            addChar((heatedBedController.currentTemperatureC + 2 < heatedBedController.targetTemperatureC) && Printer::isAnimation() ? 0x0c : 0x0d);
        }
        break;










      case 'X': // Extruder related
        if(c2 >= '0' && c2 <= '9') {
          addStringP(Extruder::current->id == c2 - '0' ? ui_selected : ui_unselected);
        } else if(c2 == 'i') {
          addFloat(currHeaterForSetup->pidIGain, 4, 2);
        } else if(c2 == 'p') {
          addFloat(currHeaterForSetup->pidPGain, 4, 2);
        } else if(c2 == 'd') {
          addFloat(currHeaterForSetup->pidDGain, 4, 2);
        } else if(c2 == 'm') {
          addNumber(currHeaterForSetup->pidDriveMin, 3);
        } else if(c2 == 'M') {
          addNumber(currHeaterForSetup->pidDriveMax, 3);
        } else if(c2 == 'D') {
          addNumber(currHeaterForSetup->pidMax, 3);
        } else if(c2 == 'w') {
          addNumber(Extruder::current->watchPeriod, 4);
        }
#if RETRACT_DURING_HEATUP
        else if(c2 == 'T') {
          addNumber(Extruder::current->waitRetractTemperature, 4);
        } else if(c2 == 'U') {
          addNumber(Extruder::current->waitRetractUnits, 2);
        }
#endif
        else if(c2 == 'h') {
          uint8_t hm = currHeaterForSetup->heatManager;
          if(hm == HTR_PID)
            addStringP(PSTR("PID"));
          else if(hm == HTR_DEADTIME)
            addStringP(PSTR("Dead time"));
          else if(hm == HTR_SLOWBANG)
            addStringP(PSTR("Slow bang"));
          else
            addStringP(PSTR("Bang bang"));
        }
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
        else if(c2 == 'a') {
          addFloat(Extruder::current->advanceK, 3, 0);
        }
#endif
        else if(c2 == 'l') {
          addFloat(Extruder::current->advanceL, 3, 0);
        }
#endif
        else if(c2 == 'x') {
          addFloat(Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS], 3, 2);
        } else if(c2 == 'y') {
          addFloat(Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS], 3, 2);
        } else if(c2 == 'z') {
          addFloat(Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS], 3, 2);
        } else if(c2 == 'f') {
          addFloat(Extruder::current->maxStartFeedrate, 5, 0);
        } else if(c2 == 'F') {
          addFloat(Extruder::current->maxFeedrate, 5, 0);
        } else if(c2 == 'A') {
          addFloat(Extruder::current->maxAcceleration, 5, 0);
        }
        break;










      case 'y':
        if(c2 >= '0' && c2 <= '3') fvalue = (float)Printer::currentNonlinearPositionSteps[c2 - '0'] * Printer::invAxisStepsPerMM[c2 - '0'];
        addFloat(fvalue, 3, 2);
        break;








      case 'z':
#if FEATURE_Z_PROBE
        if(c2 == 'h') { // write z probe height
          addFloat(EEPROM::zProbeHeight(), 3, 2);
          break;
        }
#endif
        if(c2 == '2')
          addFloat(-Printer::coordinateOffset[Z_AXIS], 2, 2);
        else
          addFloat(-Printer::coordinateOffset[c2 - '0'], 4, 0);
        break;






      case 'w':
        if(c2 >= '0' && c2 <= '7') {
          addNumber(Printer::wizardStack[c2 - '0'].l, 2);
        }
        break;





      case 'W':
        if(c2 >= '0' && c2 <= '7') {
          addFloat(Printer::wizardStack[c2 - '0'].f, 0, 2);
        } else if(c2 == 'A') {
          addFloat(Printer::wizardStack[0].f, 0, 1);
        } else if(c2 == 'B') {
          addFloat(Printer::wizardStack[1].f, 0, 1);
        }
        break;
    }
  }



#endif







#if 0


      case 'd':  // debug boolean
        if (c2 == 'o') addStringOnOff(Printer::debugEcho());
        if (c2 == 'i') addStringOnOff(Printer::debugInfo());
        if (c2 == 'e') addStringOnOff(Printer::debugErrors());
        if (c2 == 'd') addStringOnOff(Printer::debugDryrun());
        if (c2 == 'p') addStringOnOff(Printer::debugEndStop());
        if (c2 == 'x')
#if MIN_HARDWARE_ENDSTOP_X
          addStringP(Endstops::xMin() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        if (c2 == 'X')
#if MAX_HARDWARE_ENDSTOP_X
          addStringP(Endstops::xMax() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        if (c2 == 'y')
#if MIN_HARDWARE_ENDSTOP_Y
          addStringP(Endstops::yMin() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        if (c2 == 'Y')
#if MAX_HARDWARE_ENDSTOP_Y
          addStringP(Endstops::yMax() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        if (c2 == 'z')
#if MIN_HARDWARE_ENDSTOP_Z
          addStringP(Endstops::zMin() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        if (c2 == 'Z')
#if MAX_HARDWARE_ENDSTOP_Z
          addStringP(Endstops::zMax() ? ui_selected : ui_unselected);
#else
        addChar(' ');
#endif
        break;
      case 'D':
#if DISTORTION_CORRECTION
        if(c2 == 'e') {
          addStringOnOff((Printer::distortion.isEnabled()));        // Autolevel on/off
        }
#endif
        break;






      case 'l':
        if(c2 == 'a') addNumber(lastAction, 4);
#if FEATURE_AUTOLEVEL
        else if(c2 == 'l') addStringOnOff((Printer::isAutolevelActive()));        // Autolevel on/off
#endif
        break;




#endif
