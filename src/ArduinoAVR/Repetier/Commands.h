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

#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "gcode.h"

class Commands
{
public:
  static void commandLoop();
  static void checkForPeriodicalActions(bool allowNewMoves);

  static void processArc(gcodeCommand *com);

  static void processG000(gcodeCommand *com);
  static void processG001(gcodeCommand *com);
  static void processG004(gcodeCommand *com);
  static void processG029(gcodeCommand *com);
  static void processG030(gcodeCommand *com);
  static void processG100(gcodeCommand *com);
  static void processG131(gcodeCommand *com);
  static void processG132(gcodeCommand *com);
  static void processG133(gcodeCommand *com);
  static void processGCode(gcodeCommand *com);

  static void processMCode(gcodeCommand *com);

  static void executeGCode(gcodeCommand *com);

  static void waitUntilEndOfAllMoves();
  static void waitUntilEndOfAllBuffers();
  static void printCurrentPosition();
  static void printTemperatures(bool showRaw = false);
  static void setFanSpeed(int speed, bool immediately = false); /// Set fan speed 0..255
  static void setFan2Speed(int speed); /// Set fan speed 0..255
  static void changeFeedrateMultiply(int factorInPercent);
  static void changeFlowrateMultiply(int factorInPercent);
  static void emergencyStop();
};

#endif // COMMANDS_H_INCLUDED
