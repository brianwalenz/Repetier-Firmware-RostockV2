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
#include "Eeprom.h"

extern UIDisplay uid;


//  extruder[exid].tempControl.targetTemperatureC
//  extruder[exid].tempControl.currentTemperatureC

//  heatedBedController.targetTemperatureC
//  heatedBedController.currentTemperatureC



void
UIDisplay::parse(const char *txt, bool ram) {

  while (rbp < MAX_COLS) {
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

    char c1 = (ram) ? (*txt++) : (pgm_read_byte(txt++));

    //  Printing an actual '%' is quite the special case.  All other codes are two letters,
    //  but we allow '%%', '% ' and '%' (at the end of the string).

    if (c1 == '%')  { addChar('%');                 continue; }
    if (c1 == ' ')  { addChar('%');  addChar(' ');  continue; }
    if (c1 ==  0)   { addChar('%');                 break;    }

    //  Read the final letter and finish parsing.

    char c2 = (ram) ? (*txt++) : (pgm_read_byte(txt++));

    //  If %?x add an 'x' if the last character isn't 'x'.
    if (c1 == '?') {
      if ((rbp > 0) && (rb[rbp-1] != c2))
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
    //    ec - extruder temperature        %5.1f
    //    e  - extruder temperature preset %3d
    //
    //    er - relative mode?
    //
    //    eb - build plate
    //
    else if ((c1 == 'e') && (c2 == 'c')) {
      //alue = extruder[c2 - '0'].tempControl.currentTemperatureC;
      addFloat(Extruder::current->tempControl.currentTemperatureC, 3, 1);
    }

    else if ((c1 == 'e') && (c2 == 'b')) {
      addFloat(Extruder::getHeatedBedTemperature(), 3, 1);
    }

    //  Target temperatures
    //    Ec - extruder target
    //    Eb - build plate target
    //
    else if ((c1 == 'E') && (c2 == 'c')) {
      //alue = extruder[c2 - '0'].tempControl.targetTemperatureC;
      addFloat(Extruder::current->tempControl.targetTemperatureC, 3, 0);
    }

    else if ((c1 == 'E') && (c2 == 'b')) {
      addFloat(heatedBedController.targetTemperatureC, 3, 0);
    }

    //  Heater PWM fraction.
    else if ((c1 == 'h') && (c2 == 'c')) {
      uint8_t  pwm = pwm_pos[Extruder::current->id];

      if        (pwm == 0) {
        addChar('o');
        addChar('f');
        addChar('f');

      } else if (pwm == 255) {
        addChar('m');
        addChar('a');
        addChar('x');

      } else {
        addFloat(100.0 * pwm / 255.0, 2, 0);
        addChar('%');
      }
    }

    else if ((c1 == 'h') && (c2 == 'b')) {
      uint8_t  pwm = pwm_pos[heatedBedController.pwmIndex];    //  Max might be 200, not 255

      if        (pwm == 0) {
        addChar('o');
        addChar('f');
        addChar('f');

      } else if (pwm == 255) {
        addChar('m');
        addChar('a');
        addChar('x');

      } else {
        addFloat(100.0 * pwm / 255.0, 2, 0);
        addChar('%');
      }
    }




    //  Fan speeds
    else if ((c1 == 'F') && (c2 == 's')) {
      addNumber(floor(Printer::getFanSpeed() * 100 / 255 + 0.5f), 3);
    }

    else if ((c1 == 'F') && (c2 == 'S')) {
      addNumber(floor(Printer::getFan2Speed() * 100 / 255 + 0.5f), 3);
    }

    else if ((c1 == 'F') && (c2 == 'i')) {
      if (Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)
        addChar(bSEL);
      else
        addChar(bUNSEL);
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

    else if ((c1 == 'P') && (c2 == 'e')) {   //  Time elapsed
      addTimeInHoursMinutesSeconds(0);
    }

    else if ((c1 == 'P') && (c2 == 'r')) {   //  Time remaining
      addTimeInHoursMinutesSeconds(0);
    }

    else if ((c1 == 'P') && (c2 == 'f')) {   //  Filament used
      //Printer::filamentPrinted * 0.001
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
      parse(statusMsg, true);
    }

#if 0
    else if ((c1 == 'o') && (c2 == 'c')) {
      addNumber(baudrate, 6);
    }
#endif

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



    //  Usage
    //
    //  Previous version would try to add in the time of the current
    //  print job, but it looked bogus.
    //
    //  Would be nice if there was a report of the time of the current print,
    //  not just historical times.
    //
    //    void Commands::reportPrinterUsage() {
    //      float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
    //      Com::printF(PSTR("Printed filament:"), dist, 2);
    //      Com::printF(PSTR("m "));
    //      bool alloff = true;
    //      for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    //        if(tempController[i]->targetTemperatureC > 15) alloff = false;
    //      int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
    //      int32_t tmp = seconds / 86400;
    //      seconds -= tmp * 86400;
    //      Com::printF(PSTR("Printing time:"), tmp);
    //      tmp = seconds / 3600;
    //      Com::printF(PSTR(" days "), tmp);
    //      seconds -= tmp * 3600;
    //      tmp = seconds / 60;
    //      Com::printF(PSTR(" hours "), tmp);
    //      Com::printF(PSTR(" min"));
    //      Com::printF(PSTR("\n"));
    //    }

    //  Total time printing:
    //    x days xx:xx
    else if ((c1 == 'U') && (c2 == 't')) {
      uint32_t seconds = HAL::eprGetInt32(EPR_PRINTING_TIME);

      addTimeInDaysHoursMinutes(seconds);
    }

    //  Filament usage:
    //    xxx.xx m
    //    x,xxx.xx m
    //
    else if ((c1 == 'U') && (c2 == 'f')) {   //  Filament usage
      float dist  = HAL::eprGetFloat(EPR_PRINTING_DISTANCE);  //  In meters?

      addFloat(dist);
      addStringP(PSTR(" m filament"));
    }


    //  POSITIONS
    //
    //  Would print ?.?? if Printer::isAnimation() && ! Printer::ixXHomed()
    //
    else if ((c1 == 'x') && (c2 == '0')) {   //  X position
      addFloat(Printer::realXPosition(), 4, 2);
    }

    else if ((c1 == 'x') && (c2 == '1')) {   //  Y position
      addFloat(Printer::realYPosition(), 4, 2);
    }

    else if ((c1 == 'x') && (c2 == '2')) {   //  Z position
      addFloat(Printer::realZPosition(), 4, 2);
    }

    else if ((c1 == 'x') && (c2 == '3')) {   //  E position
      addFloat(Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS], 4, 2);
    }

    else if ((c1 == 'x') && (c2 == '4')) {   //  Filament printed?
      addFloat(Printer::filamentPrinted * 0.001, 3, 2);
    }

    //  Workpiece Coordinates ??
    else if ((c1 == 'x') && (c2 == '5')) {   //  X position + offset
      addFloat(Printer::currentPosition[X_AXIS] + Printer::coordinateOffset[X_AXIS], 4, 2);
    }
    else if ((c1 == 'x') && (c2 == '6')) {   //  Y position + offset
      addFloat(Printer::currentPosition[Y_AXIS] + Printer::coordinateOffset[Y_AXIS], 4, 2);
    }
    else if ((c1 == 'x') && (c2 == '7')) {   //  Z position + offset
      addFloat(Printer::currentPosition[Z_AXIS] + Printer::coordinateOffset[Z_AXIS], 4, 2);
    }




    else {
      addStringP(PSTR("UNKNOWN"));
      addChar(c1);
      addChar(c2);
    }
  }


  rb[rbp] = 0;
}

