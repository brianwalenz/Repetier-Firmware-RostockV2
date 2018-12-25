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
#include "temperatures.h"

extern UIDisplay uid;


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
      addFloat(extruderTemp.getCurrentTemperature(), 3, 1);
    }

    else if ((c1 == 'e') && (c2 == 'b')) {
      addFloat(bedTemp.getCurrentTemperature(), 3, 1);
    }

    //  Target temperatures
    //    Ec - extruder target
    //    Eb - build plate target
    //
    else if ((c1 == 'E') && ((c2 == 'c') ||
                             (c2 == 'b'))) {
      tempControl  *tc = NULL;

      if (c2 == 'c')  tc = &extruderTemp;
      if (c2 == 'b')  tc = &bedTemp;

      addFloat(tc->getTargetTemperature(), 3, 0);
    }

    //  Heater PWM fraction.
    //    hc
    //    hb
    //
    else if ((c1 == 'h') && ((c2 == 'c') ||
                             (c2 == 'b'))) {
      tempControl  *tc = NULL;

      if (c2 == 'c')  tc = &extruderTemp;
      if (c2 == 'b')  tc = &bedTemp;

      uint8_t  pwm = tc->getHeaterDensity();

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


    else if ((c1 == 'K') && (c2 == 'p'))   { addFloat(extruderTemp._pidPGain, 6, 3); }
    else if ((c1 == 'K') && (c2 == 'i'))   { addFloat(extruderTemp._pidIGain, 6, 3); }
    else if ((c1 == 'K') && (c2 == 'd'))   { addFloat(extruderTemp._pidDGain, 6, 3); }

    else if ((c1 == 'K') && (c2 == 'P'))   { addFloat(bedTemp._pidPGain, 6, 3); }
    else if ((c1 == 'K') && (c2 == 'I'))   { addFloat(bedTemp._pidIGain, 6, 3); }
    else if ((c1 == 'K') && (c2 == 'D'))   { addFloat(bedTemp._pidDGain, 6, 3); }


    //  Fan speeds
    else if ((c1 == 'F') && (c2 == 'e')) {
      addNumber(floor(extruderTemp.getFanSpeed() * 100.0 / 255.0 + 0.5f), 3);
    }

    else if ((c1 == 'F') && (c2 == 'l')) {
      addNumber(floor(layerFan.getFanSpeed() * 100.0 / 255.0 + 0.5f), 3);
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
    //    Pn - print name
    //    Te - time elapsed       Tr - time remain
    //    Fu - filament used      Ft - filament remain
    //    Lp - lines printed      Lt - lines total
    //    Zh - current height     Zt - total height
    //
    else if ((c1 == 'P') && (c2 == 'n')) {   //  Name of the file being printed
      addString(sd.printName());
    }

    else if ((c1 == 'T') && (c2 == 'e')) {   //  Time elapsed
      addTimeInHoursMinutesSeconds(commandQueue.elapsedTime());
    }

    else if ((c1 == 'T') && (c2 == 'r')) {   //  Time remaining
      addTimeInHoursMinutesSeconds(sd.estBuildTime() - commandQueue.elapsedTime());
    }

    else if ((c1 == 'F') && (c2 == 'u')) {   //  Filament used
      addFloat(Printer::filamentPrinted * 0.001, 6, 0);
    }

    else if ((c1 == 'F') && (c2 == 'r')) {   //  Filament remaining
      addFloat(sd.estFilament() - Printer::filamentPrinted * 0.001, 6, 0);
    }

    else if ((c1 == 'L') && (c2 == 'p')) {   //  Current line printing
      addNumber(commandQueue.lineNumber(), 0);
    }

    else if ((c1 == 'L') && (c2 == 't')) {   //  Total lines
      addNumber(sd.nLines(), 0);
    }

    else if ((c1 == 'Z') && (c2 == 'h')) {   //  Z height printing
      addFloat(commandQueue.currentHeight(), 3, 3);
    }

    else if ((c1 == 'Z') && (c2 == 't')) {   //  Z height total
      addFloat(sd.maxHeight(), 3, 3);
    }


    //  Status messages

    else if ((c1 == 'o') && (c2 == 's')) {
      parse(statusMsg, true);
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
      addNumber(activeExtruder->id + 1, 1);
    }

    else if ((c1 == 'o') && (c2 == 'Y')) {
      addFloat(static_cast<float>(Printer::zBabysteps) * Printer::invAxisStepsPerMM[Z_AXIS], 2, 2);
    }

    else if ((c1 == 'o') && (c2 == 'p')) {
    }


#define EPR_PRINTING_TIME         125  // Time in seconds printing
#define EPR_PRINTING_DISTANCE     129  // Filament length printed

    //  Total time printing:
    //    x days xx:xx
    //
    else if ((c1 == 'U') && (c2 == 't')) {
      uint32_t seconds = eeprom_read_dword((uint32_t *)EPR_PRINTING_TIME);

      addTimeInDaysHoursMinutes(seconds);
    }

    //  Filament usage:
    //    xxx.xx m
    //    x,xxx.xx m
    //
    else if ((c1 == 'U') && (c2 == 'f')) {   //  Filament usage
      float dist  = eeprom_read_float((float *)EPR_PRINTING_DISTANCE);  //  In meters?

      addFloat(dist);
      addStringP(PSTR(" m filament"));
    }


    //  POSITIONS
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

