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
#include "gcode.h"



float
parseFloatValue(char *&s) {

  while ((*s == ' ') ||
         (*s == '\t'))
    s++;

  float f = strtod(s, &s);

  return(f);
}



long
parseLongValue(char *s) {

  while ((*s == ' ') ||
         (*s == '\t'))
    s++;

  long l = strtol(s, &s, 10);

  return(l);
}



//  Set EEPROM
//    T - type of data
//    P - eeprom address
//    S - integer data to write
//    X - float data to write
//

bool
gcodeCommand::parseCommand(char *line) {

  _has = 0;

  while (*line) {
    char c = *line++;

    if ((c == 'G') || (c == 'g'))  { G = parseLongValue(line) & 0xffff; setG(); }   //  gcode command
    if ((c == 'M') || (c == 'm'))  { M = parseLongValue(line) & 0xffff; setM(); }   //  reprap command
    if ((c == 'T') || (c == 't'))  { T = parseLongValue(line) & 0xff;   setT(); }   //  select tool

    if ((c == 'S') || (c == 's'))  { S = parseLongValue(line);          setS(); }   //  command parameter
    if ((c == 'P') || (c == 'p'))  { P = parseLongValue(line);          setP(); }   //  command parameter, proportional Kp in PID tuning

    if ((c == 'X') || (c == 'x'))  { X = parseFloatValue(line);         setX(); }   //  X coordinate
    if ((c == 'Y') || (c == 'y'))  { Y = parseFloatValue(line);         setY(); }   //  Y coordinate
    if ((c == 'Z') || (c == 'z'))  { Z = parseFloatValue(line);         setZ(); }   //  Z coordinate

    if ((c == 'I') || (c == 'i'))  { I = parseLongValue(line);          setI(); }   //  X offset in arc move, Integral Ki in PID tuning
    if ((c == 'J') || (c == 'j'))  { J = parseLongValue(line);          setJ(); }   //  Y offset in arc move

    if ((c == 'D') || (c == 'd'))  { D = parseLongValue(line);          setD(); }   //  Diameter, derivative Kd in PID tuning
    if ((c == 'H') || (c == 'h'))  { H = parseLongValue(line);          setH(); }   //  Heater ID in PID tuning

    if ((c == 'F') || (c == 'f'))  { F = parseFloatValue(line);         setF(); }   //  Feedrate
    if ((c == 'E') || (c == 'e'))  { E = parseFloatValue(line);         setE(); }   //  Extrude rate

    if ((c == 'R') || (c == 'r'))  { R = parseLongValue(line);          setR(); }   //  Temperature

    if ((c == 'N') || (c == 'n'))  { N = parseLongValue(line);          setN(); }   //  Line number for transmission retries
  }

  //  Return true if there is a command, false otherwise.
  return(hasG() || hasM() || hasT());
}





void
gcodeCommand::printCommand(void) {

  Com::printF(PSTR("Echo:"));

  if(hasG())  { Com::printF(PSTR(" G"), G); }
  if(hasM())  { Com::printF(PSTR(" M"), M); }
  if(hasT())  { Com::printF(PSTR(" T"), T); }

  if(hasS())  { Com::printF(PSTR(" S"), S); }
  if(hasP())  { Com::printF(PSTR(" P"), P); }

  if(hasX())  { Com::printF(PSTR(" X"), X, 4); }
  if(hasY())  { Com::printF(PSTR(" Y"), Y, 4); }
  if(hasZ())  { Com::printF(PSTR(" Z"), Z, 4); }

  if(hasI())  { Com::printF(PSTR(" I"), I); }
  if(hasJ())  { Com::printF(PSTR(" J"), J); }

  if(hasD())  { Com::printF(PSTR(" D"), D); }
  if(hasH())  { Com::printF(PSTR(" H"), H); }

  if(hasF())  { Com::printF(PSTR(" F"), F); }
  if(hasE())  { Com::printF(PSTR(" E"), E, 4); }

  if(hasR())  { Com::printF(PSTR(" R"), R); }
  if(hasN())  { Com::printF(PSTR(" N"), N); }

  Com::printF(PSTR("\n"));
}

