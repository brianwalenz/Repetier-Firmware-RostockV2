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
*/

#include "Repetier.h"
#include "HAL.h"
#include "Communication.h"
#include "gcode.h"
#include "Commands.h"
#include "Eeprom.h"
#include "motion.h"
#include "Printer.h"

#include "temperatures.h"
#include "Extruder.h"

#include "rmath.h"


// Microstep setting (Only functional when stepper driver microstep pins are connected to
// MCU. Currently only works for RAMBO boards

#define MICROSTEP_MODES {16,16,16,16,16} // [1,2,4,8,16]

// MS1 MS2 Stepper Driver Micro stepping mode table

#define MICROSTEP1  LOW,LOW
#define MICROSTEP2  HIGH,LOW
#define MICROSTEP4  LOW,HIGH
#define MICROSTEP8  HIGH,HIGH
#define MICROSTEP16 HIGH,HIGH
#define MICROSTEP32 HIGH,HIGH



void Commands::commandLoop() {

  if (Printer::isBlockingReceive() == false) {
    commandQueue.executeNext();
    executeGCode(commandQueue.popCommand());

  } else {
    commandQueue.keepAlive(GCODE_PAUSED);
  }

  checkForPeriodicalActions(true);

#if 0
  uint32_t curtime = millis();

  if(PrintLine::hasLines() || Printer::isMenuMode(MODE_PRINTING | MODE_PAUSED))
    previousMillisCmd = curtime;
  else {
    curtime -= previousMillisCmd;

    if(maxInactiveTime != 0 && curtime >  maxInactiveTime )
      Printer::kill(false);
    else
      Printer::setAllKilled(false); // prevent repeated kills

    if(stepperInactiveTime != 0 && curtime >  stepperInactiveTime )
      Printer::kill(true);
  }
#endif

}



//  Called explicitly when waiting for temperatures to stabilize - waitForTargetTemperature().
//  Called explicitly when waiting for the printer to print      - waitForXFreeLines()
//  Called explicitly when waiting for movements to end          - waitUntilEndOfAllMoves() (here)
//  Called explicitly when                                       -  waitUntilEndOfAllBuffers() (here)
//  Called explicitly in G004
//
void
Commands::checkForPeriodicalActions(bool allowNewMoves) {

  if (hal.execute100ms == 0)
    return;

  hal.execute100ms = 0;

  //Com::printf(PSTR("checkForPeriodicalAction()\n"), hal.counter100ms);

  extruderTemp.manageTemperature();
  bedTemp.manageTemperature();

  sd.automount();

  // If called from queueDelta etc. it is an error to start a new move since it
  // would invalidate old computation resulting in unpredicted behavior.
  // lcd controller can start new moves, so we disallow it if called from within
  // a move command.

  uid.slowAction(allowNewMoves);
}

/** \brief Waits until movement cache is empty.

    Some commands expect no movement, before they can execute. This function
    waits, until the steppers are stopped. In the meanwhile it buffers incoming
    commands and manages temperatures.
*/
void Commands::waitUntilEndOfAllMoves() {

  while(PrintLine::hasLines()) {
    checkForPeriodicalActions(false);
    commandQueue.keepAlive(GCODE_PROCESSING);
  }
}



void Commands::waitUntilEndOfAllBuffers() {
  gcodeCommand *code = NULL;

  while (PrintLine::hasLines() || (code != NULL)) {
    code = commandQueue.popCommand();

    if (code)
      executeGCode(code);

    checkForPeriodicalActions(false);
  }
}



void Commands::printCurrentPosition() {
  float x, y, z;
  Printer::realPosition(x, y, z);
  x += Printer::coordinateOffset[X_AXIS];
  y += Printer::coordinateOffset[Y_AXIS];
  z += Printer::coordinateOffset[Z_AXIS];
  Com::printF(PSTR("X:"), x * (Printer::unitIsInches ? 0.03937 : 1), 2);
  Com::printF(PSTR(" Y:"), y * (Printer::unitIsInches ? 0.03937 : 1), 2);
  Com::printF(PSTR(" Z:"), z * (Printer::unitIsInches ? 0.03937 : 1), 3);
  Com::printF(PSTR(" E:"), Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 4);
  Com::printF(PSTR("\n"));
#ifdef DEBUG_POS
  Com::printF(PSTR("OffX:"), Printer::offsetX); // to debug offset handling
  Com::printF(PSTR(" OffY:"), Printer::offsetY);
  Com::printF(PSTR(" OffZ:"), Printer::offsetZ);
  Com::printF(PSTR(" OffZ2:"), Printer::offsetZ2);
  Com::printF(PSTR(" XS:"), Printer::currentPositionSteps[X_AXIS]);
  Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
  Com::printF(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);
  Com::printF(PSTR("\n"));
#endif
}



//  Change the speed of the entire print - both the flow and printhead speed change.
void
Commands::changeFeedrateMultiply(int factor) {
  if (factor < 10)
    factor = 10;

  Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
  Printer::feedrateMultiply = factor;

  Com::printF(PSTR("SpeedMultiply:"), factor);
  Com::printF(PSTR("\n"));
}



//  Change the flow of filament to the nozzle, leaving the speed of the printhead constant.
void
Commands::changeFlowrateMultiply(int factor) {
  if (factor < 1)
    factor = 1;

  Printer::extrudeMultiply = factor;

  if (extruder.diameter <= 0)
    Printer::extrusionFactor = 0.01f * factor;
  else
    Printer::extrusionFactor = 0.04f * factor / (extruder.diameter * extruder.diameter * 3.141592654f);

  Com::printF(PSTR("FlowMultiply:"), factor);
  Com::printF(PSTR("\n"));
}










void microstepMS(uint8_t driver, int8_t ms1, int8_t ms2) {
  if(ms1 > -1) switch(driver) {
      case 0: WRITE( X_MS1_PIN, ms1); break;
      case 1: WRITE( Y_MS1_PIN, ms1); break;
      case 2: WRITE( Z_MS1_PIN, ms1); break;
      case 3: WRITE(E0_MS1_PIN, ms1); break;
      case 4: WRITE(E1_MS1_PIN, ms1); break;
    }
  if(ms2 > -1) switch(driver) {
      case 0: WRITE( X_MS2_PIN, ms2); break;
      case 1: WRITE( Y_MS2_PIN, ms2); break;
      case 2: WRITE( Z_MS2_PIN, ms2); break;
      case 3: WRITE(E0_MS2_PIN, ms2); break;
      case 4: WRITE(E1_MS2_PIN, ms2); break;
    }
}

void microstepMode(uint8_t driver, uint8_t stepping_mode) {
  switch(stepping_mode) {
    case 1: microstepMS(driver, MICROSTEP1); break;
    case 2: microstepMS(driver, MICROSTEP2); break;
    case 4: microstepMS(driver, MICROSTEP4); break;
    case 8: microstepMS(driver, MICROSTEP8); break;
    case 16: microstepMS(driver, MICROSTEP16); break;
    case 32: microstepMS(driver, MICROSTEP32); break;
  }
}

void microstepReadings() {
  Com::printF(PSTR("MS1,MS2 Pins\n"));

  Com::printF(PSTR("X:"), READ(X_MS1_PIN));
  Com::printF(PSTR(","), READ(X_MS2_PIN));
  Com::printF(PSTR("\n"));

  Com::printF(PSTR("Y:"), READ(Y_MS1_PIN));
  Com::printF(PSTR(","), READ(Y_MS2_PIN));
  Com::printF(PSTR("\n"));

  Com::printF(PSTR("Z:"), READ(Z_MS1_PIN));
  Com::printF(PSTR(","), READ(Z_MS2_PIN));
  Com::printF(PSTR("\n"));

  Com::printF(PSTR("E0:"), READ(E0_MS1_PIN));
  Com::printF(PSTR(","), READ(E0_MS2_PIN));
  Com::printF(PSTR("\n"));

  Com::printF(PSTR("E1:"), READ(E1_MS1_PIN));
  Com::printF(PSTR(","), READ(E1_MS2_PIN));
  Com::printF(PSTR("\n"));
}



void microstepInit() {
  const uint8_t microstep_modes[] = MICROSTEP_MODES;

  SET_OUTPUT(X_MS1_PIN);
  SET_OUTPUT(Y_MS1_PIN);
  SET_OUTPUT(Z_MS1_PIN);
  SET_OUTPUT(E0_MS1_PIN);
  SET_OUTPUT(E1_MS1_PIN);
  SET_OUTPUT(X_MS2_PIN);
  SET_OUTPUT(Y_MS2_PIN);
  SET_OUTPUT(Z_MS2_PIN);
  SET_OUTPUT(E0_MS2_PIN);
  SET_OUTPUT(E1_MS2_PIN);

  for(int i = 0; i <= 4; i++)
    microstepMode(i, microstep_modes[i]);
}






void
Commands::processG000(gcodeCommand *com) {
  processG001(com);
}


void
Commands::processG001(gcodeCommand *com) {

  com->printCommand();

  for (uint16_t i=0; i<20000; i++)
    ;

  //  S0 - ignore endstops
  //  S1 - check endstops
  //
  if (com->hasS())
    Printer::setNoDestinationCheck(com->S != 0);

  //  Set X, Y, Z, E and F from the gcodeCommand.
  //
  //  setDestination..() returns true if a move would occur,
  //                             false if no move is needed.

  if ((Printer::setDestinationStepsFromGCode(com) == true) &&
      (PrintLine::queueNonlinearMove(true, true, true) == false)) {
    Com::printF(PSTR("WARNING: executeGCode / queueDeltaMove returns error\n"));
  }

  //  Wait for the move to (mostly) finish.
  //
  //  The original allowed moves (last parameter true), but that applied only
  //  to moves made by the user interface.
  //
  //  The original comment was less than helpful (and large).

  PrintLine::waitForXFreeLines(1, false);
}


void
Commands::processG004(gcodeCommand *com) {

  Commands::waitUntilEndOfAllMoves();

  uint32_t  endTime = millis();   //  Overflows at 49.71 days.

  if (com->hasP())             //  Milliseconds to wait.
    endTime += com->P;

  if (com->hasS())             //  Seconds to wait.
    endTime += com->S * 1000;

  while (millis() < endTime) {
    commandQueue.keepAlive(GCODE_PROCESSING);
    Commands::checkForPeriodicalActions(true);
  }
}


void
Commands::processG100(gcodeCommand *com) {
#if 0 // DISABLE
  // G100 Calibrate floor or rod radius
  // Using manual control, adjust hot end to contact floor.
  // G100 <no arguments> No action. Avoid accidental floor reset.
  // G100 [X] [Y] [Z] set floor for argument passed in. Number ignored and may be absent.
  // G100 R with X Y or Z flag error, sets only floor or radius, not both.
  // G100 R[n] Add n to radius. Adjust to be above floor if necessary
  // G100 R[0] set radius based on current z measurement. Moves to (0,0,0)
  float currentZmm = Printer::currentPosition[Z_AXIS];
  if (currentZmm / Printer::zLength > 0.1) {
    Com::printF(PSTR("ERROR: Calibration code is limited to bottom 10% of Z height\n"));
    break;
  }
  if (com->hasR()) {
    if (com->hasX() || com->hasY() || com->hasZ())
      Com::printF(PSTR("ERROR: Cannot set radius and floor at same time.\n"));
    else if (com->R != 0) {
      //add r to radius
      if (abs(com->R) <= 10) EEPROM::incrementRodRadius(com->R);
      else Com::printF(PSTR("ERROR: Calibration movement is limited to 10mm.\n"));
    } else {
      // auto set radius. Head must be at 0,0 and touching
      // Z offset will be corrected for.
      if (Printer::currentPosition[X_AXIS] == 0
          && Printer::currentPosition[Y_AXIS] == 0) {
        if(Printer::isLargeMachine()) {
          // calculate radius assuming we are at surface
          // If Z is greater than 0 it will get calculated out for correct radius
          // Use either A or B tower as they anchor x Cartesian axis and always have
          // Radius distance to center in simplest set up.
          float h = Printer::deltaDiagonalStepsSquaredB.f;
          unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
          // The correct Rod Radius would put us here at z==0 and B height is
          // square root (rod length squared minus rod radius squared)
          // Reverse that to get calculated Rod Radius given B height
          h -= RMath::sqr((float)bSteps);
          h = sqrt(h);
          EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
        } else {
          // calculate radius assuming we are at surface
          // If Z is greater than 0 it will get calculated out for correct radius
          // Use either A or B tower as they anchor x Cartesian axis and always have
          // Radius distance to center in simplest set up.
          unsigned long h = Printer::deltaDiagonalStepsSquaredB.l;
          unsigned long bSteps = Printer::currentNonlinearPositionSteps[B_TOWER];
          // The correct Rod Radius would put us here at z==0 and B height is
          // square root (rod length squared minus rod radius squared)
          // Reverse that to get calculated Rod Radius given B height
          h -= RMath::sqr(bSteps);
          h = integerSqrt(h);
          EEPROM::setRodRadius(h * Printer::invAxisStepsPerMM[Z_AXIS]);
        }
      } else
        Com::printF(PSTR("ERROR: First move to touch at x,y=0,0 to auto-set radius.\n"));
    }
  } else {
    bool tooBig = false;
    if (com->hasX()) {
      if (abs(com->X) <= 10)
        EEPROM::setTowerXFloor(com->X + currentZmm + Printer::xMin);
      else tooBig = true;
    }
    if (com->hasY()) {
      if (abs(com->Y) <= 10)
        EEPROM::setTowerYFloor(com->Y + currentZmm + Printer::yMin);
      else tooBig = true;
    }
    if (com->hasZ()) {
      if (abs(com->Z) <= 10)
        EEPROM::setTowerZFloor(com->Z + currentZmm + Printer::zMin);
      else tooBig = true;
    }
    if (tooBig)
      Com::printF(PSTR("ERROR: Calibration movement is limited to 10mm.\n"));
  }
  // after adjusting zero, physical position is out of sync with memory position
  // this could cause jerky movement or push head into print surface.
  // moving gets back into safe zero'ed position with respect to newle set floor or Radius.
  Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, 12.0, IGNORE_COORDINATE, IGNORE_COORDINATE);
  break;
#endif // DISABLE
}

void
Commands::processG131(gcodeCommand *com) {
#if 0 // DISABLE
  // G131 Remove offset
  float cx, cy, cz;
  Printer::realPosition(cx, cy, cz);
  float oldfeedrate = Printer::feedrate;
  Printer::offsetX = 0;
  Printer::offsetY = 0;
  Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
  Printer::feedrate = oldfeedrate;
  Printer::updateCurrentPosition();
#endif // DISABLE
}

void
Commands::processG132(gcodeCommand *com) {
#if 0 // DISABLE
  // G132 Calibrate endstop offsets
  Printer::coordinateOffset[X_AXIS] = 0;
  Printer::coordinateOffset[Y_AXIS] = 0;
  Printer::coordinateOffset[Z_AXIS] = 0;
  // I think this is coded incorrectly, as it depends on the start position of the
  // of the hot end, and so should first move to x,y,z= 0,0,0, but as that may not
  // be possible if the printer is not in the homes/zeroed state, the printer
  // cannot safely move to 0 z coordinate without crashing into the print surface.
  // so other than commenting, I'm not meddling.
  // but you will always get different counts from different positions.
  Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
  int32_t m = RMath::max(Printer::stepsRemainingAtXHit, RMath::max(Printer::stepsRemainingAtYHit, Printer::stepsRemainingAtZHit));
  int32_t offx = m - Printer::stepsRemainingAtXHit;
  int32_t offy = m - Printer::stepsRemainingAtYHit;
  int32_t offz = m - Printer::stepsRemainingAtZHit;

  Com::printF(PSTR("Tower 1:"), offx);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Tower 2:"), offy);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Tower 3:"), offz);
  Com::printF(PSTR("\n"));

  if(com->hasS() && com->S > 0) {
    EEPROM::setDeltaTowerXOffsetSteps(offx);
    EEPROM::setDeltaTowerYOffsetSteps(offy);
    EEPROM::setDeltaTowerZOffsetSteps(offz);
  }
  PrintLine::moveRelativeDistanceInSteps(0, 0, -5 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);
  Printer::homeAxis(true, true, true);
#endif // DISABLE
}

void
Commands::processG133(gcodeCommand *com) {
#if 0 // DISABLE
  // G133 Measure steps to top
  Printer::currentPositionSteps[X_AXIS] = 0;
  Printer::currentPositionSteps[Y_AXIS] = 0;
  Printer::currentPositionSteps[Z_AXIS] = 0;
  Printer::coordinateOffset[X_AXIS] = 0;
  Printer::coordinateOffset[Y_AXIS] = 0;
  Printer::coordinateOffset[Z_AXIS] = 0;
  Printer::currentNonlinearPositionSteps[A_TOWER] = 0;
  Printer::currentNonlinearPositionSteps[B_TOWER] = 0;
  Printer::currentNonlinearPositionSteps[C_TOWER] = 0;
  // similar to comment above, this will get a different answer from any different starting point
  // so it is unclear how this is helpful. It must start at a well defined point.
  Printer::deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
  int32_t offx = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtXHit;
  int32_t offy = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtYHit;
  int32_t offz = HOME_DISTANCE_STEPS - Printer::stepsRemainingAtZHit;

  Com::printF(PSTR("Tower 1:"), offx);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Tower 2:"), offy);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Tower 3:"), offz);
  Com::printF(PSTR("\n"));

  PrintLine::moveRelativeDistanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -10, 0, Printer::homingFeedrate[Z_AXIS] / 4, true, false);
  Printer::homeAxis(true, true, true);
#endif // DISABLE
}




void
Commands::processGCode(gcodeCommand *com) {
  uint32_t codenum; //throw away variable

  if      (com->G == 0) {
    processG000(com);
  }

  else if (com->G == 1) {
    processG001(com);
  }

  else if (com->G == 4) {
    processG004(com);
  }

  // G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
  else if (com->G == 10) {
    extruder.retract(true, false);
  }

  // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
  else if (com->G == 11) {
    extruder.retract(false, false);
  }

  // G20 Units to inches
  else if (com->G == 20) {
    Printer::unitIsInches = 1;
  }

  // G21 Units to mm
  else if (com->G == 21) {
    Printer::unitIsInches = 0;
  }

  //  G28 move to origin.
  //
  //  Original version would set extruder position to zero if E was supplied.
  //  That's G92 though.
  //
  //  G28             - home the towers
  //  G28 E<anything> - set extruder current position to zero
  //  G28 X<anything> - ignored for delta, same as 'G28'
  //
  else if (com->G == 28) {
    Printer::homeAxis(true, true, true);
  }

  // G90 absolute positioning mode
  else if (com->G == 90) {
    Printer::relativeCoordinateMode = false;
  }

  // G91 relative positioning mode
  else if (com->G == 91) {
    Printer::relativeCoordinateMode = true;
  }

  // G92 set position
  else if (com->G == 92) {

    if (com->hasE()) {
      Printer::destinationSteps[E_AXIS]     = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
      Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
    }

    if (com->hasX() ||
        com->hasY() ||
        com->hasZ()) {
      float xOff = Printer::coordinateOffset[X_AXIS];
      float yOff = Printer::coordinateOffset[Y_AXIS];
      float zOff = Printer::coordinateOffset[Z_AXIS];

      if (com->hasX())  xOff = Printer::convertToMM(com->X) - Printer::currentPosition[X_AXIS];
      if (com->hasY())  yOff = Printer::convertToMM(com->Y) - Printer::currentPosition[Y_AXIS];
      if (com->hasZ())  zOff = Printer::convertToMM(com->Z) - Printer::currentPosition[Z_AXIS];

      Printer::setOrigin(xOff, yOff, zOff);

      Com::printF(PSTR("X_OFFSET: "), Printer::coordinateOffset[X_AXIS], 3);  Com::printF(PSTR("\n"));
      Com::printF(PSTR("Y_OFFSET: "), Printer::coordinateOffset[Y_AXIS], 3);  Com::printF(PSTR("\n"));
      Com::printF(PSTR("Z_OFFSET: "), Printer::coordinateOffset[Z_AXIS], 3);  Com::printF(PSTR("\n"));
    }
  }

  // G100 calibrate floor or rod radius
  else if (com->G == 100) {
    processG100(com);
  }

  // G131 remove offsets
  else if (com->G == 131) {
    processG131(com);
  }

  // G132 calibrate end stop offsets
  else if (com->G == 132) {
    processG132(com);
  }

  // G133 measure steps to top
  else if (com->G == 133) {
    processG133(com);
  }

  // G135 print stuff
  else if (com->G == 135) {
    Com::printF(PSTR("CompDelta:"), Printer::currentNonlinearPositionSteps[A_TOWER]);
    Com::printF(PSTR(","), Printer::currentNonlinearPositionSteps[B_TOWER]);
    Com::printF(PSTR(","), Printer::currentNonlinearPositionSteps[C_TOWER]);
    Com::printF(PSTR("\n"));
#ifdef DEBUG_REAL_POSITION
    Com::printF(PSTR("RealDelta:"), Printer::realDeltaPositionSteps[A_TOWER]);
    Com::printF(PSTR(","), Printer::realDeltaPositionSteps[B_TOWER]);
    Com::printF(PSTR(","), Printer::realDeltaPositionSteps[C_TOWER]);
    Com::printF(PSTR("\n"));
#endif
    Printer::updateCurrentPosition();
    Com::printF(PSTR("PosFromSteps:"));
    printCurrentPosition();
  }

  else {
    Com::printF(PSTR("Unknown command:"));
    com->printCommand();
  }

  previousMillisCmd = millis();
}







void
Commands::processMCode(gcodeCommand *com) {

  //  M18 disable all stepper motors
  if (com->M == 18) {
    Commands::waitUntilEndOfAllMoves();
    bool named = false;
    if(com->hasX()) {
      named = true;
      Printer::disableXStepper();
    }
    if(com->hasY()) {
      named = true;
      Printer::disableYStepper();
    }
    if(com->hasZ()) {
      named = true;
      Printer::disableZStepper();
    }
    if(com->hasE()) {
      named = true;
      extruder.disable();
    }
    if(!named) {
      Printer::disableXStepper();
      Printer::disableYStepper();
      Printer::disableZStepper();
      extruder.disable();
    }
  }

  //  M82 set extruder to absolute mode
  else if (com->M == 82) {
    Printer::relativeExtruderCoordinateMode = false;
  }

  //  M82 set extruder to relative mode
  else if (com->M == 83) {
    Printer::relativeExtruderCoordinateMode = true;
  }

  //  M84 stop idle hold
  //  M84 S10 - stop motors after 10 seconds of idle time
  else if (com->M == 84) {
    if(com->hasS()) {
      stepperInactiveTime = com->S * 1000;
    } else {
      Commands::waitUntilEndOfAllMoves();
      Printer::kill(true);
    }
  }

  //  M85 set inactivity timer
  else if (com->M == 85) {
    if(com->hasS())
      maxInactiveTime = (int32_t)com->S * 1000;
    else
      maxInactiveTime = 0;
  }

  //  M92 set axis_steps_per_unit
  else if (com->M == 92) {
    if(com->hasX()) Printer::axisStepsPerMM[X_AXIS] = com->X;
    if(com->hasY()) Printer::axisStepsPerMM[Y_AXIS] = com->Y;
    if(com->hasZ()) Printer::axisStepsPerMM[Z_AXIS] = com->Z;

    Printer::updateDerivedParameter();

    if(com->hasE()) {
      extruder.stepsPerMM = com->E;
      Printer::selectExtruderById(0);
    }
  }

  //  M104 set extruder temperature, return immediately
  //  M104 S<temp> T<tool>
  //  M104 S<temp> T<tool> H=1 - set preheat temperature - not supported!
  //
  //  M104 S<temp>  - set extruder temp and return immediately
  //  M109 S<temp>  - set extruder temp and wait for it to stabilize
  //
  //  M140 S<temp>  - set bed temperature and return immediately
  //  M190 S<temp>  - wait until bed reaches temperature
  //
  //  M116 P<tool> H<heater> C<chamber>  - wait for temps to stabilize

  else if (com->M == 104) {
    previousMillisCmd = millis();

    if ((com->hasS() && com->S == 0))
      Commands::waitUntilEndOfAllMoves();

    //  com->hasT() not used; picks extruder to set temperature for.

    if (com->hasS())
      extruderTemp.setTargetTemperature(com->S);
  }

  else if (com->M == 140) {
    previousMillisCmd = millis();

    if (com->hasS())
      bedTemp.setTargetTemperature(com->S);
  }

  //  Report current temperatures
  //    ok T:201 B:117
  //
  else if (com->M == 105) {
  }

  else if (com->M == 109) {
    previousMillisCmd = millis();

    Commands::waitUntilEndOfAllMoves();

    if (com->hasS())
      extruderTemp.setTargetTemperature(com->S);

    extruderTemp.waitForTargetTemperature();

    previousMillisCmd = millis();
  }

  else if (com->M == 190) {
    Commands::waitUntilEndOfAllMoves();

    if (com->hasS())
      bedTemp.setTargetTemperature(com->S);

    //  If the bed is within 5 degrees, don't wait.
    //
    //if (abs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) < 5)
    //  break;

    bedTemp.waitForTargetTemperature();

    previousMillisCmd = millis();
  }

#if 0
  else if (com->M == 155) {
    Printer::setAutoreportTemp((com->hasS() && com->S != 0) || !com->hasS() );
    Printer::lastTempReport = millis();
  }
#endif

  else if (com->M == 116) {
    bedTemp.waitForTargetTemperature();
    extruderTemp.waitForTargetTemperature();
  }

  else if (com->M == 106) {
    if(com->hasI()) {
      if(com->I != 0)
        Printer::flag2 |= PRINTER_FLAG2_IGNORE_M106_COMMAND;
      else
        Printer::flag2 &= ~PRINTER_FLAG2_IGNORE_M106_COMMAND;
    }
    if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
      layerFan.setFanSpeed(com->hasS() ? com->S : 255);
    }
  }

  else if (com->M == 107) {
    if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
      layerFan.setFanSpeed(0);
    }
  }

  //  M110 set current line number
  //  M110 N<line_number>
  else if (com->M == 110) {
  }

  //  M112 emergency stop.
  //
  //  This is (also) handled in gcodeQueue::executeNext(), so we can do it
  //  immediately, regardless of what's in the queue.
  //
  else if (com->M == 112) {
    Commands::emergencyStop();
  }

  else if (com->M == 115) {
    Com::printF(PSTR("Repetier_1.0.2(bri)"));
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("CAP: AUTOREPORT_TEMP:1\n"));
    Com::printF(PSTR("CAP: EEPROM:1\n"));
    Com::printF(PSTR("CAP: PAUSESTOP:1\n"));
    //reportPrinterUsage();
  }

  else if (com->M == 114) {
    printCurrentPosition();
    if(com->hasS() && com->S) {
      Com::printF(PSTR("XS:"), Printer::currentPositionSteps[X_AXIS]);
      Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
      Com::printF(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);
      Com::printF(PSTR("\n"));
    }
  }

  else if (com->M == 119) {
    Commands::waitUntilEndOfAllMoves();
    endstops.update();
    endstops.update(); // double test to get right signal. Needed for crosstalk protection.
    endstops.report();
  }



#if RAMP_ACCELERATION
  else if (com->M == 201) {
    if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
    if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
    if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
    if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
    Printer::updateDerivedParameter();
  }


  else if (com->M == 202) {
    if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
    if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
    if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
    if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
    Printer::updateDerivedParameter();
  }
#endif


  else {
    Com::printF(PSTR("Unknown command:"));
    com->printCommand();
  }

 endMcode:
  ;
}



void
Commands::executeGCode(gcodeCommand *com) {

  if (com == NULL)
    return;

  //com->printCommand();

  if      (com->hasG()) {
    processGCode(com);
  }

  else if (com->hasM()) {
    processMCode(com);
  }

  else if (com->hasT()) {
    Commands::waitUntilEndOfAllMoves();
    Printer::selectExtruderById(com->T);
  }

  else {
    Com::printF(PSTR("Unknown command:"));
    com->printCommand();
  }
}



void Commands::emergencyStop() {

  //  Kill by resetting the controller itself.  This assumes the reset will home
  //  the machine and reset heaters.

  hal.resetHardware();

  //  The alternate is to shutdown everyting manually and freeze.
  //  Nicer in that we can bump the head off the object and NOT
  //  home on the reset.

#if 0
  forbidInterrupts(); // Don't allow interrupts to do their work

  Printer::kill(false);

  extruderTemp.manageTemperature();
  bedTemp.manageTemperature();

  extruderTemp.disable();
  bedTemp.disable();

  layerFan.setFanSpeed(0);

  delay(200);

  InterruptProtectedBlock noInts;

  while(1)
    ;
#endif
}
