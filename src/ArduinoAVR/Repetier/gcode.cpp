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
#include "Commands.h"
#include "motion.h"     //  PrintLine
#include "Printer.h"


gcodeQueue  commandQueue;


//  Read a command from the input (either serial or SD card file)
//  and add it to the queue of commands to run.
//
//  This is called from Commands::commandLoop(), which runs at full speed.
//
void
gcodeQueue::loadNext(void) {
  char     buf[MAX_CMD_SIZE];
  uint8_t  bufPos    = 0;
  uint8_t  inComment = 0;

  //  We only call loadNext() if the queue is in 'printing mode'.  If there is no data available,
  //  we must have just finished the print.

  if (dataAvailable() == false) {
    stopPrint();
    return;
  }

  //  Otherwise, read a line and process it.

  while (dataAvailable() == true) {                      //  Read a line of input.
    if (bufPos == MAX_CMD_SIZE) {                        //    Fail if we've exhausted the buffer.
      Com::printf(PSTR("Command string buffer full.\n"));
      stopPrint();
      return;
    }

    int8_t ch = readByte();                              //    Read a byte.

    //Com::printf(PSTR("Read '%c' bufPos=%u\n"), ch, bufPos);

    if ((ch == '\n') ||                                  //    If end-of-line, set it to
        (ch == '\r')) {                                  //    end-of-string.  Turn off any
      ch = 0;                                            //    in-comment flag so we append
      inComment = false;                                 //    the NUL byte.
    }

    if ((ch == ';') ||                                   //    Remember if we've just
        (ch == '('))                                     //    encountered a comment.
      inComment = true;                                  //

    if (inComment == false)                              //    If not in a comment, append
      buf[bufPos++] = ch;                                //    the letter.

    if (ch == ')')                                       //    If the end of a comment, allow
      inComment = false;                                 //    characters to be appended.

    if (ch == 0)                                         //    If at the end-of-line, break
      break;                                             //    out of the loop.
  }

  //if (buf[0] != 0)
  //  Com::printf(PSTR("COMMAND '%s'\n"), buf);

  _lineNumber++;

  gcodeCommand *cmd = _cmds + _cmdsIn;                   //  Grab the next available command

  if (cmd->parseCommand(buf) == false)                   //  Parse the command, aborting if
    return;                                              //  no command is found in the line.

  if ((cmd->hasM() == true) && (cmd->getM() == 112))     //  Handle emergency stops
    Commands::emergencyStop();                           //  IMMEDIATELY, regardless of queue.

  if ((cmd->hasZ() == true) &&                           //  If it's a move operation, remember
      (cmd->hasG() == true) &&                           //  the Z position for the UI.
      (cmd->G      == 1))
    _currentHeight = cmd->Z;

  _cmdsIn++;                                             //  Add the command to the queue.
  _cmdsLen++;                                            //

  if (_cmdsIn >= GCODE_QUEUE_LENGTH)                     //  Loop around the circle,
    _cmdsIn = 0;                                         //  if needed.
}



void
gcodeQueue::startPrint(void) {

  Com::printf(PSTR("Starting print.\n"));

  uid.setMenuMode(MODE_PRINTING);
  uid.clearMenuMode(MODE_PRINTED);

  Printer::maxLayer     = 0;
  Printer::currentLayer = 0;

  _startTime    = millis();
  _isPrinting   = true;

  if (sd.isAvailable() == true)
    sd.startFile();

  Com::printf(PSTR("Print started - active %d.\n"), sd.isAvailable());
}



void
gcodeQueue::stopPrint(void) {

  Com::printf(PSTR("Stopping print.  SD isA %d isP %d isF %d\n"),
              sd.isAvailable(), sd.isPrinting(), sd.isFinished());

  _stopTime     = millis();
  _isPrinting   = false;

  if (sd.isPrinting() == true)
    sd.stopFile();

  //  Execute some gcode on stopping.
  //GCode::executeFString(PSTR(SD_RUN_ON_STOP));

  Commands::waitUntilEndOfAllMoves();

  //if(Printer::currentPosition[Z_AXIS] < Printer::zMin + Printer::zLength - 15)
  //  PrintLine::moveRelativeDistanceInSteps(0, 0, 10 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);

  Printer::disableSteppers();
  Printer::disableHeaters();
  Printer::disablePower();

  uid.clearMenuMode(MODE_PRINTING);
  uid.setMenuMode(MODE_PRINTED);
}







#if 0


// Park position used when pausing from firmware side
#define PARK_POSITION_X (0)
#define PARK_POSITION_Y (70)
#define PARK_POSITION_Z_RAISE 10

void
Printer::moveToParkPosition() {
  if (isHomed() == false)
    return;

  moveToReal(EEPROM::parkX(),
             EEPROM::parkY(),
             IGNORE_COORDINATE,
             IGNORE_COORDINATE,
             Printer::maxFeedrate[X_AXIS],
             true);

  moveToReal(IGNORE_COORDINATE,
             IGNORE_COORDINATE,
             RMath::min(zMin + zLength, currentPosition[Z_AXIS] + EEPROM::parkZ()),
             IGNORE_COORDINATE,
             Printer::maxFeedrate[Z_AXIS],
             true);
}


void
SDCard::pausePrint(bool intern) {

  if (_sdActive == false)
    return;

  _sdMode = SDMODE_STOPPED; // finish running line

  uid.setMenuMode(MODE_PAUSED);

  commandQueue.pauseFile();

  if(intern) {
    Commands::waitUntilEndOfAllBuffers();

    Printer::MemoryPosition();

    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                        Printer::memoryE - RETRACT_ON_PAUSE,
                        Printer::maxFeedrate[E_AXIS] / 2);

    Printer::moveToParkPosition();

    Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];
    Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];
    Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];

    //commandQueue.executeFString(PSTR(PAUSE_START_COMMANDS));
  }
}



void
SDCard::continuePrint(bool intern) {

  if (_sdActive == false)
    return;

  if(intern) {
    //commandQueue.executeFString(PSTR(PAUSE_END_COMMANDS));

    Printer::GoToMemoryPosition(true, true, false, false, Printer::maxFeedrate[X_AXIS]);
    Printer::GoToMemoryPosition(false, false, true, false, Printer::maxFeedrate[Z_AXIS] / 2.0f);
    Printer::GoToMemoryPosition(false, false, false, true, Printer::maxFeedrate[E_AXIS] / 2.0f);
  }

  commandQueue.resumeFile();

  uid.clearMenuMode(MODE_PAUSED);

  _sdMode = SDMODE_PRINTING;
}


#endif
