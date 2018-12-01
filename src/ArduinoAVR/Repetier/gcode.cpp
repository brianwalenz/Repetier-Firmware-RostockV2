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
void
gcodeQueue::executeNext(void) {
  char     buf[MAX_CMD_SIZE];
  uint8_t  bufPos    = 0;
  uint8_t  inComment = 0;

  if (_cmdsLen >= GCODE_BUFFER_SIZE) {                   //  If the buffer is full, return
    keepAlive(GCODE_PROCESSING);                         //  without doing anything; wait
    return;                                              //  for the commands to execute.
  }

  if (dataAvailable() == false)                          //  If no data available, nothing
    return;                                              //  for us to do.
  
  //  CNC gcode uses () to encode comments.  Those are not supported.

  while (dataAvailable() == true) {                      //  Read a line of input.
    if (bufPos == MAX_CMD_SIZE) {                        //    Fail if we've exhausted
      fatalError(PSTR("Command string buffer full."));   //    the command buffer.
      return;
    }

    char ch = readByte();                                //    Read a byte.

#if 0
    Com::print("Read '");
    Com::print(ch);
    Com::print("'  bufPos=");
    Com::print(bufPos);
    Com::print("\n");
#endif

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

#if 0
  Com::print("COMMAND '");
  Com::print(buf);
  Com::print("'\n");
#endif

  gcodeCommand *cmd = _cmds + _cmdsIn;                   //  Grab the next available command

  if (cmd->parseCommand(buf) == false)                   //  Parse the command, aborting if
    return;                                              //  no command is found in the line.

  if ((cmd->hasM() == true) && (cmd->getM() == 112))     //  Handle emergency stops
    Commands::emergencyStop();                           //  IMMEDIATELY, regardless of queue.

  _cmdsIn++;                                             //  Add the command to the queue.
  _cmdsLen++;                                            //

  if (_cmdsIn >= GCODE_BUFFER_SIZE)                      //  Loop around the circle,
    _cmdsIn = 0;                                         //  if needed.

#if 0
  Com::printF(PSTR("ok\n"));                             //  Optionally append the line number that is being ACK'd.
#endif

  keepAlive(GCODE_NOT_BUSY);                             //  Update keep_alive status.
}



void
gcodeQueue::fatalError(FSTRINGPARAM(message)) {

  Printer::stopPrint();

  if(Printer::currentPosition[Z_AXIS] < Printer::zMin + Printer::zLength - 15)
    PrintLine::moveRelativeDistanceInSteps(0, 0, 10 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);

  Commands::waitUntilEndOfAllMoves();

  Printer::kill(false);

  Com::printF(PSTR("fatal:"));
  Com::printF(message);
  Com::printF(PSTR(" - Printer stopped and heaters disabled due to this error. Fix error and restart with M999.\n"));

  uid.setStatusP(message);
  uid.refreshPage();
}
