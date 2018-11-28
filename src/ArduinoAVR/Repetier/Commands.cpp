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



const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42



void Commands::commandLoop() {

#ifdef DEBUG_PRINT
  debugWaitLoop = 1;
#endif

  if (Printer::isBlockingReceive() == false) {
    commandQueue.executeNext();

    //uid.slowAction()  //  do slow events?  was disabled

    uid.mediumAction(); // do check encoder

    gcodeCommand *code = commandQueue.popCommand();

    if (code) {
      if (Printer::debugEcho())
        code->printCommand();

      Commands::executeGCode(code);
    }

  } else {
    commandQueue.keepAlive(GCODE_PAUSED);
    uid.mediumAction();
  }

  Printer::defaultLoopActions();
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {

  Printer::handleInterruptEvent();

  if(executePeriodical == false)
    return; // gets true every 100ms

  executePeriodical = 0;

  Extruder::manageTemperatures();

  if (--counter500ms == 0) {
    if(manageMonitor)
      writeMonitor();
    counter500ms = 5;
  }

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
#ifdef DEBUG_PRINT
  debugWaitLoop = 8;
#endif
  while(PrintLine::hasLines()) {
    checkForPeriodicalActions(false);
    commandQueue.keepAlive(GCODE_PROCESSING);
    uid.mediumAction();
  }
}

void Commands::waitUntilEndOfAllBuffers() {
  gcodeCommand *code = NULL;

#ifdef DEBUG_PRINT
  debugWaitLoop = 9;
#endif

  while(PrintLine::hasLines() || (code != NULL)) {
    uid.mediumAction(); // do check encoder

    code = commandQueue.popCommand();

    if (code) {
      if (Printer::debugEcho())
        code->printCommand();

      Commands::executeGCode(code);
    }

    Commands::checkForPeriodicalActions(false); // only called from memory
    uid.mediumAction();
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



void Commands::printTemperatures(bool showRaw) {
  int error;
  float temp = Extruder::current->tempControl.currentTemperatureC;
#if HEATED_BED_SENSOR_TYPE == 0
  Com::printF(PSTR("T:"), temp);
  Com::printF(PSTR(" /"), Extruder::current->tempControl.targetTemperatureC, 0);
#else
  Com::printF(PSTR("T:"), temp);
  Com::printF(PSTR(" /"), Extruder::current->tempControl.targetTemperatureC, 0);
  Com::printF(PSTR(" B:"), Extruder::getHeatedBedTemperature());
  Com::printF(PSTR(" /"), heatedBedController.targetTemperatureC, 0);
  if((error = heatedBedController.errorState()) > 0) {
    Com::printF(PSTR(" DB:"), error);
  }
  if(showRaw) {
    Com::printF(PSTR(" RAW"), (int)NUM_EXTRUDER);
    Com::printF(PSTR(":"), (1023 << (2 - ANALOG_REDUCE_BITS)) - heatedBedController.currentTemperature);
  }
  Com::printF(PSTR(" B@:"), (pwm_pos[heatedBedController.pwmIndex])); // Show output of auto tune when tuning!
#endif
  Com::printF(PSTR(" @:"), (autotuneIndex == 255 ? pwm_pos[Extruder::current->id] : pwm_pos[autotuneIndex])); // Show output of auto tune when tuning!
  if((error = extruder[0].tempControl.errorState()) > 0) {
    Com::printF(PSTR(" D0:"), error);
  }
  if(showRaw) {
    Com::printF(PSTR(" RAW"), (int)0);
    Com::printF(PSTR(":"), (1023 << (2 - ANALOG_REDUCE_BITS)) - extruder[0].tempControl.currentTemperature);
  }
  Com::printF(PSTR("\n"));
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

  if (Extruder::current->diameter <= 0)
    Printer::extrusionFactor = 0.01f * factor;
  else
    Printer::extrusionFactor = 0.04f * factor / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);

  Com::printF(PSTR("FlowMultiply:"), factor);
  Com::printF(PSTR("\n"));
}



#if FEATURE_FAN_CONTROL
uint8_t fanKickstart;
#endif
#if FEATURE_FAN2_CONTROL
uint8_t fan2Kickstart;
#endif

void Commands::setFanSpeed(int speed, bool immediately) {
#if FAN_PIN >- 1 && FEATURE_FAN_CONTROL
  if(Printer::fanSpeed == speed)
    return;
  speed = constrain(speed, 0, 255);
  //Printer::setMenuMode(MODE_FAN_RUNNING, speed != 0);
  Printer::fanSpeed = speed;
  if(PrintLine::linesCount == 0 || immediately) {
    if(Printer::mode == PRINTER_MODE_FFF) {
      for(fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++)
        PrintLine::lines[i].secondSpeed = speed;         // fill all printline buffers with new fan speed value
    }
    Printer::setFanSpeedDirectly(speed);
  }
  Com::printF(PSTR("Fanspeed:"), speed); // send only new values to break update loops!
  Com::printF(PSTR("\n"));
#endif
}
void Commands::setFan2Speed(int speed) {
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
  speed = constrain(speed, 0, 255);
  Printer::setFan2SpeedDirectly(speed);
  Com::printF(PSTR("Fanspeed2:"), speed); // send only new values to break update loops!
  Com::printF(PSTR("\n"));
#endif
}




// Digipot methods for controling current and microstepping

void digitalPotWrite(int address, uint16_t value) { // From Arduino DigitalPotControl example
  if(value > 255)
    value = 255;
  WRITE(DIGIPOTSS_PIN, LOW); // take the SS pin low to select the chip
  HAL::spiSend(address); //  send in the address and value via SPI:
  HAL::spiSend(value);
  WRITE(DIGIPOTSS_PIN, HIGH); // take the SS pin high to de-select the chip:
  //delay(10);
}

void setMotorCurrent(uint8_t driver, uint16_t current) {
  if(driver > 4) return;
  const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
  digitalPotWrite(digipot_ch[driver], current);
}

void setMotorCurrentPercent( uint8_t channel, float level) {
  uint16_t raw_level = ( level * 255 / 100 );
  setMotorCurrent(channel, raw_level);
}

void motorCurrentControlInit() { //Initialize Digipot Motor Current
  HAL::spiInit(0); //SPI.begin();
  SET_OUTPUT(DIGIPOTSS_PIN);
  const float digipot_motor_current[] = MOTOR_CURRENT_PERCENT;
  for(int i = 0; i <= 4; i++)
    //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
    setMotorCurrentPercent(i, digipot_motor_current[i]);
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

  uint32_t  endTime = HAL::timeInMilliseconds();   //  Overflows at 49.71 days.

  if (com->hasP())             //  Milliseconds to wait.
    endTime += com->P;

  if (com->hasS())             //  Seconds to wait.
    endTime += com->S * 1000;

  while (HAL::timeInMilliseconds() < endTime) {
    commandQueue.keepAlive(GCODE_PROCESSING);
    Commands::checkForPeriodicalActions(true);
  }
}


void
Commands::processG029(gcodeCommand *com) {
#if 0 // DISABLE
  Printer::prepareForProbing();

#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
  float actTemp[NUM_EXTRUDER];
  for(int i = 0; i < NUM_EXTRUDER; i++)
    actTemp[i] = extruder[i].tempControl.targetTemperatureC;
  Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, RMath::max(EEPROM::zProbeHeight(), static_cast<float>(ZHOME_HEAT_HEIGHT)), IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
  Commands::waitUntilEndOfAllMoves();

#if ZHOME_HEAT_ALL
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
  }
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
      Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
  }
#else
  if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif

  bool ok = true;
  Printer::startProbing(true);
  bool oldAutolevel = Printer::isAutolevelActive();
  Printer::setAutolevelActive(false);
  float sum = 0, last, oldFeedrate = Printer::feedrate;
  Printer::moveTo(EEPROM::zProbeX1(), EEPROM::zProbeY1(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
  sum = Printer::runZProbe(true, false, Z_PROBE_REPETITIONS, false);
  if(sum == ILLEGAL_Z_PROBE) ok = false;
  if(ok) {
    Printer::moveTo(EEPROM::zProbeX2(), EEPROM::zProbeY2(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
    last = Printer::runZProbe(false, false);
    if(last == ILLEGAL_Z_PROBE) ok = false;
    sum += last;
  }
  if(ok) {
    Printer::moveTo(EEPROM::zProbeX3(), EEPROM::zProbeY3(), IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
    last = Printer::runZProbe(false, true);
    if(last == ILLEGAL_Z_PROBE) ok = false;
    sum += last;
  }
  if(ok) {
    sum *= 0.33333333333333;
    Com::printF(PSTR("Z-probe average height:"), sum);
    Com::printF(PSTR("\n"));
    if(com->hasS() && com->S) {
#if MAX_HARDWARE_ENDSTOP_Z
#if DRIVE_SYSTEM == DELTA
      Printer::updateCurrentPosition();
      Printer::zLength += sum - Printer::currentPosition[Z_AXIS];
      Printer::updateDerivedParameter();
      Printer::homeAxis(true, true, true);
#else
      Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
      float zup = Printer::runZMaxProbe();
      if(zup == ILLEGAL_Z_PROBE) {
        ok = false;
      } else
        Printer::zLength = zup + sum - ENDSTOP_Z_BACK_ON_HOME;
#endif // DELTA
      Com::printF(PSTR("INFO Reset Z height\n"));
      Com::printF(PSTR("Printer height:"), Printer::zLength);
      Com::printF(PSTR("\n"));
#else
      Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
      Com::printF(PSTR("Adjusted z origin"));
      Com::printF(PSTR("\n"));
#endif // max z endstop
    }
    Printer::feedrate = oldFeedrate;
    Printer::setAutolevelActive(oldAutolevel);
    if(ok && com->hasS() && com->S == 2)
      EEPROM::storeDataIntoEEPROM();
  }
  Printer::updateCurrentPosition(true);
  printCurrentPosition();
  Printer::finishProbing();
  Printer::feedrate = oldFeedrate;
  if(!ok) {
    commandQueue.fatalError(PSTR("G29 leveling failed!"));
    break;
  }
#if defined(Z_PROBE_MIN_TEMPERATURE) && Z_PROBE_MIN_TEMPERATURE && Z_PROBE_REQUIRES_HEATING
#if ZHOME_HEAT_ALL
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, false);
  }
  for(int i = 0; i < NUM_EXTRUDER; i++) {
    if(extruder[i].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
      Extruder::setTemperatureForExtruder(RMath::max(actTemp[i], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), i, false, true);
  }
#else
  if(extruder[Extruder::current->id].tempControl.currentTemperatureC < ZPROBE_MIN_TEMPERATURE)
    Extruder::setTemperatureForExtruder(RMath::max(actTemp[Extruder::current->id], static_cast<float>(ZPROBE_MIN_TEMPERATURE)), Extruder::current->id, false, true);
#endif
#endif
#endif // DISABLE
}

void
Commands::processG030(gcodeCommand *com) {
#if 0 // DISABLE
  if (com->hasS()) {
    Printer::measureZProbeHeight(com->hasZ() ? com->Z : Printer::currentPosition[Z_AXIS]);
  } else {
    uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
    float z = Printer::runZProbe(p & 1, p & 2, Z_PROBE_REPETITIONS, true, false);
    if(z == ILLEGAL_Z_PROBE) {
      commandQueue.fatalError(PSTR("G30 probing failed!"));
      break;
    }
    if(com->hasR() || com->hasH()) {
      float h = Printer::convertToMM(com->hasH() ? com->H : 0);
      float o = Printer::convertToMM(com->hasR() ? com->R : h);
#if DISTORTION_CORRECTION
      // Undo z distortion correction contained in z
      float zCorr = 0;
      if(Printer::distortion.isEnabled()) {
        zCorr = Printer::distortion.correct(Printer::currentPositionSteps[X_AXIS], Printer::currentPositionSteps[Y_AXIS], Printer::zMinSteps) * Printer::invAxisStepsPerMM[Z_AXIS];
        z -= zCorr;
      }
#endif
      Printer::coordinateOffset[Z_AXIS] = o - h;
      Printer::currentPosition[Z_AXIS] = Printer::lastCmdPos[Z_AXIS] = z + h + Printer::zMin;
      Printer::updateCurrentPositionSteps();
      Printer::setZHomed(true);
      transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
    } else {
      Printer::updateCurrentPosition(p & 1);
    }
  }
#endif // DISABLE
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
          h = SQRT(h);
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
  // This has the probably unintended side effect of turning off leveling.
  Printer::setAutolevelActive(false); // don't let transformations change result!
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
  bool oldAuto = Printer::isAutolevelActive();
  Printer::setAutolevelActive(false); // don't let transformations change result!
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

  Printer::setAutolevelActive(oldAuto);
  PrintLine::moveRelativeDistanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
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
    Extruder::current->retract(true, false);
  }

  // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
  else if (com->G == 11) {
    Extruder::current->retract(false, false);
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

  // G29 3 points, build average or distortion compensation
  else if (com->G == 29) {
    processG029(com);
  }

  // G30 [Pn] [S]
  // G30 (the same as G30 P3) single probe set Z0
  // G30 S1 Z<real_z_pos> - measures probe height (P is ignored) assuming we are at real height Z
  // G30 H<height> R<offset> Make probe define new Z and z offset (R) at trigger point assuming z-probe measured an object of H height.
  else if (com->G == 30) {
    processG030(com);
  }

  // G31 display hall sensor output
  else if (com->G == 31) {
    Endstops::update();
    Com::printF(PSTR("Z-probe state:"));
    Com::printF(Endstops::zProbe() ? PSTR("H ") : PSTR("L "));
    Com::printF(PSTR("\n"));
  }

  // G32 Auto-Bed leveling
#if FEATURE_AUTOLEVEL
  else if (com->G == 32) {
    if(!runBedLeveling(com->hasS() ? com->S : -1)) {
      commandQueue.fatalError(PSTR("G32 leveling failed!"));
    }
  }
#endif

  // G33 distortion matrix stuff
#if DISTORTION_CORRECTION
  else if (com->G == 33) {
    if(com->hasL()) { // G33 L0 - List distortion matrix
      Printer::distortion.showMatrix();
    } else if(com->hasR()) { // G33 R0 - Reset distortion matrix
      Printer::distortion.resetCorrection();
    } else if(com->hasX() || com->hasY() || com->hasZ()) { // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
      if(com->hasX() && com->hasY() && com->hasZ()) {
        Printer::distortion.set(com->X, com->Y, com->Z);
      } else {
        Com::printF(PSTR("ERROR You need to define X, Y and Z to set a point!\n"));
      }
    } else { // G33
      Printer::measureDistortion();
    }
  }
#endif

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

  else if (Printer::debugErrors()) {
    Com::printF(PSTR("Unknown command:"));
    com->printCommand();
  }

  previousMillisCmd = HAL::timeInMilliseconds();
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
      Extruder::disableCurrentExtruderMotor();
    }
    if(!named) {
      Printer::disableXStepper();
      Printer::disableYStepper();
      Printer::disableZStepper();
      Extruder::disableAllExtruderMotors();
    }
  }

  //  M42 set hardware pin
#if 0
  else if (com->M == 42) {
    if (com->hasP()) {
      int pin_number = com->P;
      for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
        if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
          pin_number = -1;
          goto endMcode;
        }
      }
      if (pin_number > -1) {
        if(com->hasS()) {
          if(com->S >= 0 && com->S <= 255) {
            pinMode(pin_number, OUTPUT);
            digitalWrite(pin_number, com->S);
            analogWrite(pin_number, com->S);
            Com::printF(PSTR("Set output: "), pin_number);
            Com::printF(PSTR(" to "), (int)com->S);
            Com::printF(PSTR("\n"));
          } else
            Com::printF(PSTR("ERROR: Illegal S value for M42\n"));
        } else {
          pinMode(pin_number, INPUT_PULLUP);
          Com::printF(PSTR(" to "), pin_number);
          Com::printF(PSTR(" is "), digitalRead(pin_number));
          Com::printF(PSTR("\n"));
        }
      } else {
        Com::printF(PSTR("ERROR: Pin can not be set by M42, is in sensitive pins!\n"));
      }
    }
  }
#endif

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
      Extruder::current->stepsPerMM = com->E;
      Extruder::selectExtruderById(Extruder::current->id);
    }
  }

  //  M104 set extruder temperature, return immediately
  //  M104 S<temp> T<tool>
  //  M104 S<temp> T<tool> H=1 - set preheat temperature
  //
  else if (com->M == 104) {
    if(Printer::debugDryrun())
      goto endMcode;
    if(reportTempsensorError())
      goto endMcode;

    previousMillisCmd = HAL::timeInMilliseconds();


    if(com->hasP() || (com->hasS() && com->S == 0))
      Commands::waitUntilEndOfAllMoves();
    if (com->hasS()) {
      if(com->hasT() && com->T < NUM_EXTRUDER)
        Extruder::setTemperatureForExtruder(com->S, com->T, com->hasF() && com->F > 0);
      else
        Extruder::setTemperatureForExtruder(com->S, Extruder::current->id, com->hasF() && com->F > 0);
    } else if(com->hasH()) {
      if(com->hasT() && com->T < NUM_EXTRUDER)
        Extruder::setTemperatureForExtruder(extruder[com->T].tempControl.preheatTemperature, com->T, com->hasF() && com->F > 0);
      else
        Extruder::setTemperatureForExtruder(Extruder::current->tempControl.preheatTemperature, Extruder::current->id, com->hasF() && com->F > 0);
    }
  }

  else if (com->M == 140) {
    if(Printer::debugDryrun())
      goto endMcode;
    if(reportTempsensorError())
      goto endMcode;

    previousMillisCmd = HAL::timeInMilliseconds();

    if (com->hasS()) Extruder::setHeatedBedTemperature(com->S, com->hasF() && com->F > 0);
    else if(com->hasH()) Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature, com->hasF() && com->F > 0);
  }

  else if (com->M == 105) {
    printTemperatures(com->hasX());
  }

  else if (com->M == 109) {
    if(Printer::debugDryrun())
      goto endMcode;
    if(reportTempsensorError())
      goto endMcode;

    previousMillisCmd = HAL::timeInMilliseconds();

    Commands::waitUntilEndOfAllMoves();
    Extruder *actExtruder = Extruder::current;
    if(com->hasT() && com->T < NUM_EXTRUDER) actExtruder = &extruder[com->T];
    if (com->hasS()) Extruder::setTemperatureForExtruder(com->S, actExtruder->id, com->hasF() && com->F > 0, true);
    else if(com->hasH())  Extruder::setTemperatureForExtruder(actExtruder->tempControl.preheatTemperature, actExtruder->id, com->hasF() && com->F > 0, true);
    previousMillisCmd = HAL::timeInMilliseconds();
  }

  else if (com->M == 190) {
    if(Printer::debugDryrun())
      goto endMcode;
    if(reportTempsensorError())
      goto endMcode;

    Commands::waitUntilEndOfAllMoves();

    if (com->hasS()) Extruder::setHeatedBedTemperature(com->S, com->hasF() && com->F > 0);
    else if(com->hasH())  Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature, com->hasF() && com->F > 0);

    //  If the bed is within 5 degrees, don't wait.
    //
    //if (abs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) < 5)
    //  break;

    tempController[HEATED_BED_INDEX]->waitForTargetTemperature();

    previousMillisCmd = HAL::timeInMilliseconds();
  }

  else if (com->M == 155) {
    Printer::setAutoreportTemp((com->hasS() && com->S != 0) || !com->hasS() );
    Printer::lastTempReport = HAL::timeInMilliseconds();
  }

  else if (com->M == 116) {
    for(fast8_t h = 0; h <= HEATED_BED_INDEX; h++) {
      tempController[h]->waitForTargetTemperature();
    }
  }

  else if (com->M == 106) {
    if(com->hasI()) {
      if(com->I != 0)
        Printer::flag2 |= PRINTER_FLAG2_IGNORE_M106_COMMAND;
      else
        Printer::flag2 &= ~PRINTER_FLAG2_IGNORE_M106_COMMAND;
    }
    if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
      if(com->hasP() && com->P == 1)
        setFan2Speed(com->hasS() ? com->S : 255);
      else
        setFanSpeed(com->hasS() ? com->S : 255);
    }
  }

  else if (com->M == 107) {
    if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
      if(com->hasP() && com->P == 1)
        setFan2Speed(0);
      else
        setFanSpeed(0);
    }
  }

  //  M110 set current line number
  //  M110 N<line_number>
  else if (com->M == 110) {
  }

  //  M111 set debug level
  //
  else if (com->M == 111) {
    if(com->hasS()) Printer::setDebugLevel(static_cast<uint8_t>(com->S));
    if(com->hasP()) {
      if (com->P > 0) Printer::debugSet(static_cast<uint8_t>(com->P));
      else Printer::debugReset(static_cast<uint8_t>(-com->P));
    }
    if(Printer::debugDryrun()) { // simulate movements without printing
      Extruder::setTemperatureForExtruder(0, 0);
      Extruder::setHeatedBedTemperature(0, false);
    }
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
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    Com::printF(PSTR("CAP: AUTOLEVEL:1\n"));
#else
    Com::printF(PSTR("CAP: AUTOLEVEL:0\n"));
#endif
#if FEATURE_Z_PROBE
    Com::printF(PSTR("CAP: Z_PROBE:1\n"));
#else
    Com::printF(PSTR("CAP: Z_PROBE:0\n"));
#endif
    Com::printF(PSTR("CAP: PAUSESTOP:1\n"));
    Com::printF(PSTR("CAP: PREHEAT:1\n"));
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
    Endstops::update();
    Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
    Endstops::report();
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


  else if (com->M == 203) {
    if(com->hasS())
      manageMonitor = com->S != 255;
    else
      manageMonitor = 0;
  }

  //  M204 set PID parameters (non-standard)
  //  M204 X[Kp] Y[Ki] Z[Kd]
#if 0
  else if (com->M == 204) {
    TemperatureController *temp = &Extruder::current->tempControl;

    if(com->hasS()) {
      if(com->S < 0) break;
      if(com->S < NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
      else temp = &heatedBedController;
    }
    if(com->hasX())  temp->pidPGain = com->X;
    if(com->hasY())  temp->pidIGain = com->Y;
    if(com->hasZ())  temp->pidDGain = com->Z;

    temp->updateTempControlVars();
  }
#endif


  else {
    if(Printer::debugErrors()) {
      Com::printF(PSTR("Unknown command:"));
      com->printCommand();
    }
  }

 endMcode:
  ;
}



void
Commands::executeGCode(gcodeCommand *com) {

  if      (com->hasG()) {
    processGCode(com);
  }

  else if (com->hasM()) {
    processMCode(com);
  }

  else if (com->hasT()) {
    Commands::waitUntilEndOfAllMoves();
    Extruder::selectExtruderById(com->T);
  }

  else if (Printer::debugErrors()) {
    Com::printF(PSTR("Unknown command:"));
    com->printCommand();
  }
}



void Commands::emergencyStop() {

  //  Kill by resetting the controller itself.  This assumes the reset will home
  //  the machine and reset heaters.

  HAL::resetHardware();

  //  The alternate is to shutdown everyting manually and freeze.
#if 0
  //HAL::forbidInterrupts(); // Don't allow interrupts to do their work
  Printer::kill(false);
  Extruder::manageTemperatures();
  for(uint8_t i = 0; i < NUM_EXTRUDER + 3; i++)
    pwm_pos[i] = 0;

#if EXT0_HEATER_PIN > -1
  WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif

#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
  WRITE(FAN_PIN, 0);
#endif

  WRITE(HEATED_BED_HEATER_PIN, HEATER_PINS_INVERTED);

  uid.setStatusP(PSTR("Killed"));
  uid.refreshPage();

  HAL::delayMilliseconds(200);

  InterruptProtectedBlock noInts;

  while(1)
    ;
#endif
}
