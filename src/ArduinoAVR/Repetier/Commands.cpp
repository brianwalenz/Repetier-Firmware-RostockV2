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

const int8_t sensitive_pins[] PROGMEM = SENSITIVE_PINS; // Sensitive pin list for M42
int Commands::lowestRAMValue = MAX_RAM;
int Commands::lowestRAMValueSend = MAX_RAM;

void Commands::commandLoop() {
  //while(true) {
#ifdef DEBUG_PRINT
  debugWaitLoop = 1;
#endif
  if(!Printer::isBlockingReceive()) {
    GCode::readFromSerial();
    GCode *code = GCode::peekCurrentCommand();

    //uid.slowAction()  //  do slow events?  was disabled

    uid.mediumAction(); // do check encoder
    if(code) {
      if(sd.savetosd) {
        if(!(code->hasM() && code->M == 29))   // still writing to file
          sd.writeCommand(code);
        else
          sd.finishWrite();
#if ECHO_ON_EXECUTE
        code->echoCommand();
#endif
      } else
        Commands::executeGCode(code);
      code->popCurrentCommand();
    }
  } else {
    GCode::keepAlive(Paused);
    uid.mediumAction();
  }
  Printer::defaultLoopActions();
  //}
}

void Commands::checkForPeriodicalActions(bool allowNewMoves) {
  Printer::handleInterruptEvent();

  if(!executePeriodical) return; // gets true every 100ms
  executePeriodical = 0;

  Extruder::manageTemperatures();
  if(--counter500ms == 0) {
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
    //GCode::readFromSerial();
    checkForPeriodicalActions(false);
    GCode::keepAlive(Processing);
    uid.mediumAction();
  }
}

void Commands::waitUntilEndOfAllBuffers() {
  GCode *code = NULL;
#ifdef DEBUG_PRINT
  debugWaitLoop = 9;
#endif
  while(PrintLine::hasLines() || (code != NULL)) {
    //GCode::readFromSerial();
    code = GCode::peekCurrentCommand();
    uid.mediumAction(); // do check encoder
    if(code) {
      if(sd.savetosd) {
        if(!(code->hasM() && code->M == 29))   // still writing to file
          sd.writeCommand(code);
        else
          sd.finishWrite();
#if ECHO_ON_EXECUTE
        code->echoCommand();
#endif
      } else
        Commands::executeGCode(code);
      code->popCurrentCommand();
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
  Com::printFLN(PSTR(" E:"), Printer::currentPositionSteps[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS] * (Printer::unitIsInches ? 0.03937 : 1), 4);
#ifdef DEBUG_POS
  Com::printF(PSTR("OffX:"), Printer::offsetX); // to debug offset handling
  Com::printF(PSTR(" OffY:"), Printer::offsetY);
  Com::printF(PSTR(" OffZ:"), Printer::offsetZ);
  Com::printF(PSTR(" OffZ2:"), Printer::offsetZ2);
  Com::printF(PSTR(" XS:"), Printer::currentPositionSteps[X_AXIS]);
  Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
  Com::printFLN(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);

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
  Com::println();
}
void Commands::changeFeedrateMultiply(int factor) {
  if(factor < 25) factor = 25;
  if(factor > 500) factor = 500;
  Printer::feedrate *= (float)factor / (float)Printer::feedrateMultiply;
  Printer::feedrateMultiply = factor;
  Com::printFLN(PSTR("SpeedMultiply:"), factor);
}

void Commands::changeFlowrateMultiply(int factor) {
  if(factor < 25) factor = 25;
  if(factor > 200) factor = 200;
  Printer::extrudeMultiply = factor;
  if(Extruder::current->diameter <= 0)
    Printer::extrusionFactor = 0.01f * static_cast<float>(factor);
  else
    Printer::extrusionFactor = 0.01f * static_cast<float>(factor) * 4.0f / (Extruder::current->diameter * Extruder::current->diameter * 3.141592654f);
  Com::printFLN(PSTR("FlowMultiply:"), factor);
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
  Printer::setMenuMode(MENU_MODE_FAN_RUNNING, speed != 0);
  Printer::fanSpeed = speed;
  if(PrintLine::linesCount == 0 || immediately) {
    if(Printer::mode == PRINTER_MODE_FFF) {
      for(fast8_t i = 0; i < PRINTLINE_CACHE_SIZE; i++)
        PrintLine::lines[i].secondSpeed = speed;         // fill all printline buffers with new fan speed value
    }
    Printer::setFanSpeedDirectly(speed);
  }
  Com::printFLN(PSTR("Fanspeed:"), speed); // send only new values to break update loops!
#endif
}
void Commands::setFan2Speed(int speed) {
#if FAN2_PIN >- 1 && FEATURE_FAN2_CONTROL
  speed = constrain(speed, 0, 255);
  Printer::setFan2SpeedDirectly(speed);
  Com::printFLN(PSTR("Fanspeed2:"), speed); // send only new values to break update loops!
#endif
}

void Commands::reportPrinterUsage() {
  float dist = Printer::filamentPrinted * 0.001 + HAL::eprGetFloat(EPR_PRINTING_DISTANCE);
  Com::printF(PSTR("Printed filament:"), dist, 2);
  Com::printF(PSTR("m "));
  bool alloff = true;
  for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    if(tempController[i]->targetTemperatureC > 15) alloff = false;
  int32_t seconds = (alloff ? 0 : (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000) + HAL::eprGetInt32(EPR_PRINTING_TIME);
  int32_t tmp = seconds / 86400;
  seconds -= tmp * 86400;
  Com::printF(PSTR("Printing time:"), tmp);
  tmp = seconds / 3600;
  Com::printF(PSTR(" days "), tmp);
  seconds -= tmp * 3600;
  tmp = seconds / 60;
  Com::printF(PSTR(" hours "), tmp);
  Com::printFLN(PSTR(" min"));
}



// Digipot methods for controling current and microstepping

int digitalPotWrite(int address, uint16_t value) { // From Arduino DigitalPotControl example
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
  Com::printFLN(PSTR("MS1,MS2 Pins"));

  Com::printF(PSTR("X:"), READ(X_MS1_PIN));
  Com::printFLN(PSTR(","), READ(X_MS2_PIN));

  Com::printF(PSTR("Y:"), READ(Y_MS1_PIN));
  Com::printFLN(PSTR(","), READ(Y_MS2_PIN));

  Com::printF(PSTR("Z:"), READ(Z_MS1_PIN));
  Com::printFLN(PSTR(","), READ(Z_MS2_PIN));

  Com::printF(PSTR("E0:"), READ(E0_MS1_PIN));
  Com::printFLN(PSTR(","), READ(E0_MS2_PIN));

  Com::printF(PSTR("E1:"), READ(E1_MS1_PIN));
  Com::printFLN(PSTR(","), READ(E1_MS2_PIN));
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




/**
   \brief Execute the G command stored in com.
*/
void Commands::processGCode(GCode *com) {
  uint32_t codenum; //throw away variable
  switch(com->G) {
    case 0: // G0 -> G1
    case 1: // G1
      if(com->hasS()) Printer::setNoDestinationCheck(com->S != 0);
      if(Printer::setDestinationStepsFromGCode(com)) // For X Y Z E F
        if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, true)) {
          Com::printWarningFLN(PSTR("executeGCode / queueDeltaMove returns error"));
        }

      // ui can only execute motion commands if we are not waiting inside a move for an
      // old move to finish. For normal response times, we always leave one free after
      // sending a line. Drawback: 1 buffer line less for limited time. Since input cache
      // gets filled while waiting, the lost is neglectable.
      PrintLine::waitForXFreeLines(1, true);

#ifdef DEBUG_QUEUE_MOVE
      {
        InterruptProtectedBlock noInts;
        int lc = (int)PrintLine::linesCount;
        int lp = (int)PrintLine::linesPos;
        int wp = (int)PrintLine::linesWritePos;
        int n = (wp - lp);
        if(n < 0) n += PRINTLINE_CACHE_SIZE;
        noInts.unprotect();
        if(n != lc)
          Com::printFLN(PSTR("Buffer corrupted"));
      }
#endif

      break;
    case 4: // G4 dwell
      Commands::waitUntilEndOfAllMoves();
      codenum = 0;
      if(com->hasP()) codenum = com->P; // milliseconds to wait
      if(com->hasS()) codenum = com->S * 1000; // seconds to wait
      codenum += HAL::timeInMilliseconds();  // keep track of when we started waiting
      while((uint32_t)(codenum - HAL::timeInMilliseconds())  < 2000000000 ) {
        GCode::keepAlive(Processing);
        Commands::checkForPeriodicalActions(true);
      }
      break;
#if FEATURE_RETRACTION
    case 10: // G10 S<1 = long retract, 0 = short retract = default> retracts filament according to stored setting
      Extruder::current->retract(true, false);
      break;
    case 11: // G11 S<1 = long retract, 0 = short retract = default> = Undo retraction according to stored setting
      Extruder::current->retract(false, false);
      break;
#endif // FEATURE_RETRACTION
    case 20: // G20 Units to inches
      Printer::unitIsInches = 1;
      break;
    case 21: // G21 Units to mm
      Printer::unitIsInches = 0;
      break;
    case 28: { //G28 Home all Axis one at a time
      uint8_t homeAllAxis = (com->hasNoXYZ() && !com->hasE());
      if(com->hasE())
        Printer::currentPositionSteps[E_AXIS] = 0;
      if(homeAllAxis || !com->hasNoXYZ())
        Printer::homeAxis(homeAllAxis || com->hasX(), homeAllAxis || com->hasY(), homeAllAxis || com->hasZ());
    }
      break;
#if FEATURE_Z_PROBE
    case 29: { // G29 3 points, build average or distortion compensation
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
        Com::printFLN(PSTR("Z-probe average height:"), sum);
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
          Com::printInfoFLN(PSTR("Reset Z height"));
          Com::printFLN(PSTR("Printer height:"), Printer::zLength);
#else
          Printer::currentPositionSteps[Z_AXIS] = sum * Printer::axisStepsPerMM[Z_AXIS];
          Com::printFLN(PSTR("Adjusted z origin"));
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
        GCode::fatalError(PSTR("G29 leveling failed!"));
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
    }
      break;
    case 30: {
      // G30 [Pn] [S]
      // G30 (the same as G30 P3) single probe set Z0
      // G30 S1 Z<real_z_pos> - measures probe height (P is ignored) assuming we are at real height Z
      // G30 H<height> R<offset> Make probe define new Z and z offset (R) at trigger point assuming z-probe measured an object of H height.
      if (com->hasS()) {
        Printer::measureZProbeHeight(com->hasZ() ? com->Z : Printer::currentPosition[Z_AXIS]);
      } else {
        uint8_t p = (com->hasP() ? (uint8_t)com->P : 3);
        float z = Printer::runZProbe(p & 1, p & 2, Z_PROBE_REPETITIONS, true, false);
        if(z == ILLEGAL_Z_PROBE) {
          GCode::fatalError(PSTR("G30 probing failed!"));
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
    }
      break;
    case 31:  // G31 display hall sensor output
      Endstops::update();
      Endstops::update();
      Com::printF(PSTR("Z-probe state:"));
      Com::printF(Endstops::zProbe() ? PSTR("H ") : PSTR("L "));
      Com::println();
      break;
#if FEATURE_AUTOLEVEL
    case 32: // G32 Auto-Bed leveling
      if(!runBedLeveling(com->hasS() ? com->S : -1)) {
        GCode::fatalError(PSTR("G32 leveling failed!"));
      }
      break;
#endif
#if DISTORTION_CORRECTION
    case 33: {
      if(com->hasL()) { // G33 L0 - List distortion matrix
        Printer::distortion.showMatrix();
      } else if(com->hasR()) { // G33 R0 - Reset distortion matrix
        Printer::distortion.resetCorrection();
      } else if(com->hasX() || com->hasY() || com->hasZ()) { // G33 X<xpos> Y<ypos> Z<zCorrection> - Set correction for nearest point
        if(com->hasX() && com->hasY() && com->hasZ()) {
          Printer::distortion.set(com->X, com->Y, com->Z);
        } else {
          Com::printErrorFLN(PSTR("You need to define X, Y and Z to set a point!"));
        }
      } else { // G33
        Printer::measureDistortion();
      }
    }
      break;
#endif
#endif
    case 90: // G90
      Printer::relativeCoordinateMode = false;
      if(com->internalCommand)
        Com::printInfoFLN(PSTR("Absolute positioning"));
      break;
    case 91: // G91
      Printer::relativeCoordinateMode = true;
      if(com->internalCommand)
        Com::printInfoFLN(PSTR("Relative positioning"));
      break;
    case 92: { // G92
      float xOff = Printer::coordinateOffset[X_AXIS];
      float yOff = Printer::coordinateOffset[Y_AXIS];
      float zOff = Printer::coordinateOffset[Z_AXIS];
      if(com->hasX()) xOff = Printer::convertToMM(com->X) - Printer::currentPosition[X_AXIS];
      if(com->hasY()) yOff = Printer::convertToMM(com->Y) - Printer::currentPosition[Y_AXIS];
      if(com->hasZ()) zOff = Printer::convertToMM(com->Z) - Printer::currentPosition[Z_AXIS];
      Printer::setOrigin(xOff, yOff, zOff);
      if(com->hasE()) {
        Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] = Printer::convertToMM(com->E) * Printer::axisStepsPerMM[E_AXIS];
      }
      if(com->hasX() || com->hasY() || com->hasZ()) {
        Com::printF(PSTR("X_OFFSET:"), Printer::coordinateOffset[X_AXIS], 3);
        Com::printF(PSTR(" Y_OFFSET:"), Printer::coordinateOffset[Y_AXIS], 3);
        Com::printFLN(PSTR(" Z_OFFSET:"), Printer::coordinateOffset[Z_AXIS], 3);
      }
    }
      break;
#if DRIVE_SYSTEM == DELTA
    case 100: { // G100 Calibrate floor or rod radius
      // Using manual control, adjust hot end to contact floor.
      // G100 <no arguments> No action. Avoid accidental floor reset.
      // G100 [X] [Y] [Z] set floor for argument passed in. Number ignored and may be absent.
      // G100 R with X Y or Z flag error, sets only floor or radius, not both.
      // G100 R[n] Add n to radius. Adjust to be above floor if necessary
      // G100 R[0] set radius based on current z measurement. Moves to (0,0,0)
      float currentZmm = Printer::currentPosition[Z_AXIS];
      if (currentZmm / Printer::zLength > 0.1) {
        Com::printErrorFLN(PSTR("Calibration code is limited to bottom 10% of Z height"));
        break;
      }
      if (com->hasR()) {
        if (com->hasX() || com->hasY() || com->hasZ())
          Com::printErrorFLN(PSTR("Cannot set radius and floor at same time."));
        else if (com->R != 0) {
          //add r to radius
          if (abs(com->R) <= 10) EEPROM::incrementRodRadius(com->R);
          else Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
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
            Com::printErrorFLN(PSTR("First move to touch at x,y=0,0 to auto-set radius."));
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
          Com::printErrorFLN(PSTR("Calibration movement is limited to 10mm."));
      }
      // after adjusting zero, physical position is out of sync with memory position
      // this could cause jerky movement or push head into print surface.
      // moving gets back into safe zero'ed position with respect to newle set floor or Radius.
      Printer::moveTo(IGNORE_COORDINATE, IGNORE_COORDINATE, 12.0, IGNORE_COORDINATE, IGNORE_COORDINATE);
      break;
    }
    case 131: { // G131 Remove offset
      float cx, cy, cz;
      Printer::realPosition(cx, cy, cz);
      float oldfeedrate = Printer::feedrate;
      Printer::offsetX = 0;
      Printer::offsetY = 0;
      Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, Printer::homingFeedrate[X_AXIS]);
      Printer::feedrate = oldfeedrate;
      Printer::updateCurrentPosition();
    }
      break;
    case 132: { // G132 Calibrate endstop offsets
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
      Com::printFLN(PSTR("Tower 1:"), offx);
      Com::printFLN(PSTR("Tower 2:"), offy);
      Com::printFLN(PSTR("Tower 3:"), offz);
      if(com->hasS() && com->S > 0) {
        EEPROM::setDeltaTowerXOffsetSteps(offx);
        EEPROM::setDeltaTowerYOffsetSteps(offy);
        EEPROM::setDeltaTowerZOffsetSteps(offz);
      }
      PrintLine::moveRelativeDistanceInSteps(0, 0, -5 * Printer::axisStepsPerMM[Z_AXIS], 0, Printer::homingFeedrate[Z_AXIS], true, true);
      Printer::homeAxis(true, true, true);
    }
      break;
    case 133: { // G133 Measure steps to top
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
      Com::printFLN(PSTR("Tower 1:"), offx);
      Com::printFLN(PSTR("Tower 2:"), offy);
      Com::printFLN(PSTR("Tower 3:"), offz);
      Printer::setAutolevelActive(oldAuto);
      PrintLine::moveRelativeDistanceInSteps(0, 0, Printer::axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, false);
      Printer::homeAxis(true, true, true);
    }
      break;
    case 135: // G135
      Com::printF(PSTR("CompDelta:"), Printer::currentNonlinearPositionSteps[A_TOWER]);
      Com::printF(PSTR(","), Printer::currentNonlinearPositionSteps[B_TOWER]);
      Com::printFLN(PSTR(","), Printer::currentNonlinearPositionSteps[C_TOWER]);
#ifdef DEBUG_REAL_POSITION
      Com::printF(PSTR("RealDelta:"), Printer::realDeltaPositionSteps[A_TOWER]);
      Com::printF(PSTR(","), Printer::realDeltaPositionSteps[B_TOWER]);
      Com::printFLN(PSTR(","), Printer::realDeltaPositionSteps[C_TOWER]);
#endif
      Printer::updateCurrentPosition();
      Com::printF(PSTR("PosFromSteps:"));
      printCurrentPosition();
      break;

#endif // DRIVE_SYSTEM
    default:
      if(Printer::debugErrors()) {
        Com::printF(PSTR("Unknown command:"));
        com->printCommand();
      }
  }
  previousMillisCmd = HAL::timeInMilliseconds();
}
/**
   \brief Execute the G command stored in com.
*/
void Commands::processMCode(GCode *com) {

  switch( com->M ) {
    case 3: // Spindle/laser on
      break;
    case 4: // Spindle CCW
      break;
    case 5: // Spindle/laser off
      break;
    case 20: // M20 - list SD card
      sd.ls();
      break;
    case 21: // M21 - init SD card
      sd.mount();
      break;
    case 22: //M22 - release SD card
      sd.unmount();
      break;
    case 23: //M23 - Select file
      if(com->hasString()) {
        sd.fat.chdir();
        sd.selectFile(com->text);
      }
      break;
    case 24: //M24 - Start SD print
      sd.startPrint();
      break;
    case 25: //M25 - Pause SD print
      sd.pausePrint();
      break;
    case 26: //M26 - Set SD index
      if(com->hasS())
        sd.setIndex(com->S);
      break;
    case 27: //M27 - Get SD status
      sd.printStatus();
      break;
    case 28: //M28 - Start SD write
      if(com->hasString())
        sd.startWrite(com->text);
      break;
    case 29: //M29 - Stop SD write
      //processed in write to file routine above
      //savetosd = false;
      break;
    case 30: // M30 filename - Delete file
      if(com->hasString()) {
        sd.fat.chdir();
        sd.deleteFile(com->text);
      }
      break;
    case 32: // M32 directoryname
      if(com->hasString()) {
        sd.fat.chdir();
        sd.makeDirectory(com->text);
      }
      break;
    case 42: //M42 -Change pin status via gcode
      if (com->hasP()) {
        int pin_number = com->P;
        for(uint8_t i = 0; i < (uint8_t)sizeof(sensitive_pins); i++) {
          if (pgm_read_byte(&sensitive_pins[i]) == pin_number) {
            pin_number = -1;
            break;
          }
        }
        if (pin_number > -1) {
          if(com->hasS()) {
            if(com->S >= 0 && com->S <= 255) {
              pinMode(pin_number, OUTPUT);
              digitalWrite(pin_number, com->S);
              analogWrite(pin_number, com->S);
              Com::printF(PSTR("Set output: "), pin_number);
              Com::printFLN(PSTR(" to "), (int)com->S);
            } else
              Com::printErrorFLN(PSTR("Illegal S value for M42"));
          } else {
            pinMode(pin_number, INPUT_PULLUP);
            Com::printF(PSTR(" to "), pin_number);
            Com::printFLN(PSTR(" is "), digitalRead(pin_number));
          }
        } else {
          Com::printErrorFLN(PSTR("Pin can not be set by M42, is in sensitive pins! "));
        }
      }
      break;
    case 80: // M80 - ATX Power On
#if PS_ON_PIN > -1
      Commands::waitUntilEndOfAllMoves();
      previousMillisCmd = HAL::timeInMilliseconds();
      SET_OUTPUT(PS_ON_PIN); //GND
      Printer::setPowerOn(true);
      WRITE(PS_ON_PIN, LOW);
#endif
      break;
    case 81: // M81 - ATX Power Off
#if PS_ON_PIN > -1
      Commands::waitUntilEndOfAllMoves();
      SET_OUTPUT(PS_ON_PIN); //GND
      Printer::setPowerOn(false);
      WRITE(PS_ON_PIN, HIGH);
#endif
      break;
    case 82: // M82
      Printer::relativeExtruderCoordinateMode = false;
      break;
    case 83: // M83
      Printer::relativeExtruderCoordinateMode = true;
      break;
    case 18: // M18 is to disable named axis
      {
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
      break;
    case 84: // M84
      if(com->hasS()) {
        stepperInactiveTime = com->S * 1000;
      } else {
        Commands::waitUntilEndOfAllMoves();
        Printer::kill(true);
      }
      break;
    case 85: // M85
      if(com->hasS())
        maxInactiveTime = (int32_t)com->S * 1000;
      else
        maxInactiveTime = 0;
      break;
    case 92: // M92
      if(com->hasX()) Printer::axisStepsPerMM[X_AXIS] = com->X;
      if(com->hasY()) Printer::axisStepsPerMM[Y_AXIS] = com->Y;
      if(com->hasZ()) Printer::axisStepsPerMM[Z_AXIS] = com->Z;
      Printer::updateDerivedParameter();
      if(com->hasE()) {
        Extruder::current->stepsPerMM = com->E;
        Extruder::selectExtruderById(Extruder::current->id);
      }
      break;
    case 99: { // M99 S<time>
      millis_t wait = 10000;
      if(com->hasS())
        wait = 1000 * com->S;
      if(com->hasX())
        Printer::disableXStepper();
      if(com->hasY())
        Printer::disableYStepper();
      if(com->hasZ())
        Printer::disableZStepper();
      wait += HAL::timeInMilliseconds();
#ifdef DEBUG_PRINT
      debugWaitLoop = 2;
#endif
      while(wait - HAL::timeInMilliseconds() < 100000) {
        Printer::defaultLoopActions();
      }
      if(com->hasX())
        Printer::enableXStepper();
      if(com->hasY())
        Printer::enableYStepper();
      if(com->hasZ())
        Printer::enableZStepper();
    }
      break;

    case 104: // M104 temperature
      if(reportTempsensorError()) break;
      previousMillisCmd = HAL::timeInMilliseconds();
      if(Printer::debugDryrun()) break;
#ifdef EXACT_TEMPERATURE_TIMING
      Commands::waitUntilEndOfAllMoves();
#else
      if(com->hasP() || (com->hasS() && com->S == 0))
        Commands::waitUntilEndOfAllMoves();
#endif
      if (com->hasS()) {
        if(com->hasT() && com->T < NUM_EXTRUDER)
          Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), com->T, com->hasF() && com->F > 0);
        else
          Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), Extruder::current->id, com->hasF() && com->F > 0);
      } else if(com->hasH()) {
        if(com->hasT() && com->T < NUM_EXTRUDER)
          Extruder::setTemperatureForExtruder(extruder[com->T].tempControl.preheatTemperature + (com->hasO() ? com->O : 0), com->T, com->hasF() && com->F > 0);
        else
          Extruder::setTemperatureForExtruder(Extruder::current->tempControl.preheatTemperature + (com->hasO() ? com->O : 0), Extruder::current->id, com->hasF() && com->F > 0);
      }
      break;
    case 140: // M140 set bed temp
      if(reportTempsensorError()) break;
      previousMillisCmd = HAL::timeInMilliseconds();
      if(Printer::debugDryrun()) break;
      if (com->hasS()) Extruder::setHeatedBedTemperature(com->S + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
      else if(com->hasH()) Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
      break;
    case 105: // M105  get temperature. Always returns the current temperature, doesn't wait until move stopped
      Com::writeToAll = false;
      printTemperatures(com->hasX());
      break;
    case 109: // M109 - Wait for extruder heater to reach target.
      {
        if(reportTempsensorError()) break;
        previousMillisCmd = HAL::timeInMilliseconds();
        if(Printer::debugDryrun()) break;
        Commands::waitUntilEndOfAllMoves();
        Extruder *actExtruder = Extruder::current;
        if(com->hasT() && com->T < NUM_EXTRUDER) actExtruder = &extruder[com->T];
        if (com->hasS()) Extruder::setTemperatureForExtruder(com->S + (com->hasO() ? com->O : 0), actExtruder->id, com->hasF() && com->F > 0, true);
        else if(com->hasH())  Extruder::setTemperatureForExtruder(actExtruder->tempControl.preheatTemperature + (com->hasO() ? com->O : 0), actExtruder->id, com->hasF() && com->F > 0, true);
      }
      previousMillisCmd = HAL::timeInMilliseconds();
      break;
    case 190: { // M190 - Wait bed for heater to reach target.
      if(Printer::debugDryrun()) break;
      UI_STATUS_UPD_F(PSTR("Heating BED"));
      Commands::waitUntilEndOfAllMoves();
      if (com->hasS()) Extruder::setHeatedBedTemperature(com->S + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
      else if(com->hasH())  Extruder::setHeatedBedTemperature(heatedBedController.preheatTemperature + (com->hasO() ? com->O : 0), com->hasF() && com->F > 0);
#if defined(SKIP_M190_IF_WITHIN) && SKIP_M190_IF_WITHIN > 0
      if(abs(heatedBedController.currentTemperatureC - heatedBedController.targetTemperatureC) < SKIP_M190_IF_WITHIN) break;
#endif
      tempController[HEATED_BED_INDEX]->waitForTargetTemperature();
      UI_CLEAR_STATUS;
      previousMillisCmd = HAL::timeInMilliseconds();
    }
      break;
    case 155: // M155 S<1/0> Enable/disable auto report temperatures. When enabled firmware will emit temperatures every second.
      Printer::setAutoreportTemp((com->hasS() && com->S != 0) || !com->hasS() );
      Printer::lastTempReport = HAL::timeInMilliseconds();
      break;
#if NUM_TEMPERATURE_LOOPS > 0
    case 116: // Wait for temperatures to reach target temperature
      for(fast8_t h = 0; h <= HEATED_BED_INDEX; h++) {
        tempController[h]->waitForTargetTemperature();
      }
      break;
#endif
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
    case 106: // M106 Fan On
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
      break;
    case 107: // M107 Fan Off
      if(!(Printer::flag2 & PRINTER_FLAG2_IGNORE_M106_COMMAND)) {
        if(com->hasP() && com->P == 1)
          setFan2Speed(0);
        else
          setFanSpeed(0);
      }
      break;
#endif
    case 111: // M111 enable/disable run time debug flags
      if(com->hasS()) Printer::setDebugLevel(static_cast<uint8_t>(com->S));
      if(com->hasP()) {
        if (com->P > 0) Printer::debugSet(static_cast<uint8_t>(com->P));
        else Printer::debugReset(static_cast<uint8_t>(-com->P));
      }
      if(Printer::debugDryrun()) { // simulate movements without printing
        Extruder::setTemperatureForExtruder(0, 0);
        Extruder::setHeatedBedTemperature(0, false);
      }
      break;
    case 115: // M115
      Com::writeToAll = false;
      Com::printFLN(PSTR("Repetier_1.0.2(bri)"));
      Com::cap(PSTR("AUTOREPORT_TEMP:1"));
      Com::cap(PSTR("EEPROM:1"));
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
      Com::cap(PSTR("AUTOLEVEL:1"));
#else
      Com::cap(PSTR("AUTOLEVEL:0"));
#endif
#if FEATURE_Z_PROBE
      Com::cap(PSTR("Z_PROBE:1"));
#else
      Com::cap(PSTR("Z_PROBE:0"));
#endif
      Com::cap(PSTR("PAUSESTOP:1"));
      Com::cap(PSTR("PREHEAT:1"));
      reportPrinterUsage();
      Printer::reportPrinterMode();
      break;
    case 114: // M114
      Com::writeToAll = false;
      printCurrentPosition();
      if(com->hasS() && com->S) {
        Com::printF(PSTR("XS:"), Printer::currentPositionSteps[X_AXIS]);
        Com::printF(PSTR(" YS:"), Printer::currentPositionSteps[Y_AXIS]);
        Com::printFLN(PSTR(" ZS:"), Printer::currentPositionSteps[Z_AXIS]);
      }
      break;
    case 117: // M117 message to lcd
      if(com->hasString()) {
        UI_STATUS_UPD_RAM(com->text);
      }
      break;
    case 119: // M119
      Com::writeToAll = false;
      Commands::waitUntilEndOfAllMoves();
      Endstops::update();
      Endstops::update(); // double test to get right signal. Needed for crosstalk protection.
      Endstops::report();
      break;
    case 170: // preheat temperatures
      /* M170 - Set or retrieve preheat temperatures
         Parameter:
         B<bedPreheat> : Sets bed preheat temperature
         C<chamberPreheat> : Sets heated chamber temperature
         T<extruder> S<preheatTemp> : Sets preheat temperature for given extruder
         L0 : List preheat temperatures. Returns
         PREHEAT_BED:temp PREHEAT0:extr0 PREHEAT1:extr1 PREHEAT_CHAMBER:temp
      */
      {
        bool mod = false;
        if(com->hasB()) {
          mod |= heatedBedController.preheatTemperature != static_cast<int16_t>(com->B);
          heatedBedController.preheatTemperature = com->B;
        }
        if(com->hasT() && com->hasS() && com->T < NUM_EXTRUDER) {
          mod |= extruder[com->T].tempControl.preheatTemperature != static_cast<int16_t>(com->S);
          extruder[com->T].tempControl.preheatTemperature = com->S;
        }
        if(com->hasL()) {
          Com::printF(PSTR("PREHEAT_BED:"), heatedBedController.preheatTemperature);
          for(int i = 0; i < NUM_EXTRUDER; i++) {
            Com::printF(PSTR(" PREHEAT"), i);
            Com::printF(PSTR(":"), extruder[i].tempControl.preheatTemperature);
          }
          Com::println();
        }
        if(mod) {
          HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP, heatedBedController.preheatTemperature);
          for(int i = 0; i < NUM_EXTRUDER; i++) {
            int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
            Extruder *e = &extruder[i];
            HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT, e->tempControl.preheatTemperature);
          }
          EEPROM::updateChecksum();
        }
      }
      break;
    case 200: { // M200 T<extruder> D<diameter>
      uint8_t extruderId = Extruder::current->id;
      if(com->hasT() && com->T < NUM_EXTRUDER)
        extruderId = com->T;
      float d = 0;
      if(com->hasR())
        d = com->R;
      if(com->hasD())
        d = com->D;
      extruder[extruderId].diameter = d;
      if(extruderId == Extruder::current->id)
        changeFlowrateMultiply(Printer::extrudeMultiply);
      if(d == 0) {
        Com::printFLN(PSTR("Disabled volumetric extrusion for extruder "), static_cast<int>(extruderId));
      } else {
        Com::printF(PSTR("Set volumetric extrusion for extruder "), static_cast<int>(extruderId));
        Com::printFLN(PSTR(" to "), d);
      }
    }
      break;
#if RAMP_ACCELERATION
    case 201: // M201
      if(com->hasX()) Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = com->X;
      if(com->hasY()) Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
      if(com->hasZ()) Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
      if(com->hasE()) Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = com->E;
      Printer::updateDerivedParameter();
      break;
    case 202: // M202
      if(com->hasX()) Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = com->X;
      if(com->hasY()) Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = com->Y;
      if(com->hasZ()) Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = com->Z;
      if(com->hasE()) Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = com->E;
      Printer::updateDerivedParameter();
      break;
#endif
    case 203: // M203 Temperature monitor
      if(com->hasS())
        manageMonitor = com->S != 255;
      else
        manageMonitor = 0;
      break;
    case 204: { // M204
      TemperatureController *temp = &Extruder::current->tempControl;
      if(com->hasS()) {
        if(com->S < 0) break;
        if(com->S < NUM_EXTRUDER) temp = &extruder[com->S].tempControl;
        else temp = &heatedBedController;
      }
      if(com->hasX()) temp->pidPGain = com->X;
      if(com->hasY()) temp->pidIGain = com->Y;
      if(com->hasZ()) temp->pidDGain = com->Z;
      temp->updateTempControlVars();
    }
      break;
    case 205: // M205 Show EEPROM settings
      Com::writeToAll = false;
      EEPROM::writeSettings();
      break;
    case 206: // M206 T[type] P[pos] [Sint(long] [Xfloat]  Set eeprom value
      Com::writeToAll = false;
      EEPROM::update(com);
      break;
    case 207: // M207 X<XY jerk> Z<Z Jerk>
      if(com->hasX())
        Printer::maxJerk = com->X;
      if(com->hasE()) {
        Extruder::current->maxStartFeedrate = com->E;
        Extruder::selectExtruderById(Extruder::current->id);
      }
#if DRIVE_SYSTEM != DELTA
      if(com->hasZ())
        Printer::maxZJerk = com->Z;
      Com::printF(PSTR("Jerk:"), Printer::maxJerk);
      Com::printFLN(PSTR(" ZJerk:"), Printer::maxZJerk);
#else
      Com::printFLN(PSTR("Jerk:"), Printer::maxJerk);
#endif
      break;
    case 220: // M220 S<Feedrate multiplier in percent>
      changeFeedrateMultiply(com->getS(100));
      break;
    case 221: // M221 S<Extrusion flow multiplier in percent>
      changeFlowrateMultiply(com->getS(100));
      break;
    case 226: // M226 P<pin> S<state 0/1> - Wait for pin getting state S
      if(!com->hasS() || !com->hasP())
        break;
      {
        bool comp = com->S;
        if(com->hasX()) {
          if(com->X == 0)
            HAL::pinMode(com->P, INPUT);
          else
            HAL::pinMode(com->P, INPUT_PULLUP);
        }
        do {
          Commands::checkForPeriodicalActions(true);
          GCode::keepAlive(WaitHeater);
        } while(HAL::digitalRead(com->P) != comp);
      }
      break;
#if USE_ADVANCE
    case 223: // M223 Extruder interrupt test
      if(com->hasS()) {
        InterruptProtectedBlock noInts;
        Printer::extruderStepsNeeded += com->S;
      }
      break;
    case 232: // M232
      Com::printF(PSTR(" linear steps:"), maxadv2);
#if ENABLE_QUADRATIC_ADVANCE
      Com::printF(PSTR(" quadratic steps:"), maxadv);
#endif
      Com::printFLN(PSTR(", speed="), maxadvspeed);
#if ENABLE_QUADRATIC_ADVANCE
      maxadv = 0;
#endif
      maxadv2 = 0;
      maxadvspeed = 0;
      break;
#endif
#if USE_ADVANCE
    case 233: // M233
      if(com->hasY())
        Extruder::current->advanceL = com->Y;
      Com::printF(PSTR("linear L:"), Extruder::current->advanceL);
#if ENABLE_QUADRATIC_ADVANCE
      if(com->hasX())
        Extruder::current->advanceK = com->X;
      Com::printF(PSTR(" quadratic K:"), Extruder::current->advanceK);
#endif
      Com::println();
      Printer::updateAdvanceFlags();
      break;
#endif
#if Z_HOME_DIR > 0 && MAX_HARDWARE_ENDSTOP_Z
    case 251: // M251
      Printer::zLength -= Printer::currentPosition[Z_AXIS];
      Printer::currentPositionSteps[Z_AXIS] = 0;
      Printer::updateDerivedParameter();
      transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
      Printer::updateCurrentPosition();
      Com::printFLN(PSTR("Printer height:"), Printer::zLength);
      EEPROM::storeDataIntoEEPROM(false);
      Com::printFLN(PSTR("EEPROM updated"));
      Commands::printCurrentPosition();
      break;
#endif
    case 281: // Trigger watchdog
#if FEATURE_WATCHDOG
      {
        if(com->hasX()) {
          HAL::stopWatchdog();
          Com::printFLN(PSTR("Watchdog disabled"));
          break;
        }
        Com::printInfoFLN(PSTR("Triggering watchdog. If activated, the printer will reset."));
        Printer::kill(false);
        HAL::delayMilliseconds(200); // write output, make sure heaters are off for safety
#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
        InterruptProtectedBlock noInts;         // don't disable interrupts on mega2560 and mega1280 because of bootloader bug
#endif
        while(1) {} // Endless loop
      }
#else
      Com::printInfoFLN(PSTR("Watchdog feature was not compiled into this version!"));
#endif
      break;
#if FEATURE_BABYSTEPPING
    case 290: // M290 Z<babysteps> - Correct by adding baby steps for Z mm
      if(com->hasZ()) {
        if(abs(com->Z) < (32700 - labs(Printer::zBabystepsMissing)) * Printer::axisStepsPerMM[Z_AXIS])
          Printer::zBabystepsMissing += com->Z * Printer::axisStepsPerMM[Z_AXIS];
      }
      break;
#endif
    case 302: // M302 S<0 or 1> - allow cold extrusion. Without S parameter it will allow. S1 will allow, S0 will disallow.
      Printer::setColdExtrusionAllowed(!com->hasS() || (com->hasS() && com->S != 0));
      break;
    case 303: { // M303
#if NUM_TEMPERATURE_LOOPS > 0
      int temp = 150;
      int cont = 0;
      int cycles = 5;
      int method = 0;
      if(com->hasS()) temp = com->S;
      if(com->hasP()) cont = com->P;
      if(com->hasR()) cycles = static_cast<int>(com->R);
      if(com->hasC()) method = static_cast<int>(com->C);
      if(cont >= HEATED_BED_INDEX) cont = HEATED_BED_INDEX;
      if(cont < 0) cont = 0;
      tempController[cont]->autotunePID(temp, cont, cycles, com->hasX(), method);
#endif
    }
      break;

#if FEATURE_AUTOLEVEL
    case 320: // M320 Activate autolevel
      Printer::setAutolevelActive(true);
      if(com->hasS() && com->S) {
        EEPROM::storeDataIntoEEPROM();
      }
      break;
    case 321: // M321 Deactivate autoleveling
      Printer::setAutolevelActive(false);
      if(com->hasS() && com->S) {
        if(com->S == 3)
          Printer::resetTransformationMatrix(false);
        EEPROM::storeDataIntoEEPROM();
      }
      break;
    case 322: // M322 Reset auto leveling matrix
      Printer::resetTransformationMatrix(false);
      if(com->hasS() && com->S) {
        EEPROM::storeDataIntoEEPROM();
      }
      break;
#endif // FEATURE_AUTOLEVEL
#if DISTORTION_CORRECTION
    case 323: // M323 S0/S1 enable disable distortion correction P0 = not permanent, P1 = permanent = default
      if(com->hasS()) {
        if(com->S > 0)
          Printer::distortion.enable(com->hasP() && com->P == 1);
        else
          Printer::distortion.disable(com->hasP() && com->P == 1);
      } else {
        Printer::distortion.reportStatus();
      }
      break;
#endif // DISTORTION_CORRECTION
    case 350: { // M350 Set micro stepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
#if (defined(X_MS1_PIN) && X_MS1_PIN > -1)
      if(com->hasS()) for(int i = 0; i <= 4; i++) microstepMode(i, com->S);
      if(com->hasX()) microstepMode(0, (uint8_t)com->X);
      if(com->hasY()) microstepMode(1, (uint8_t)com->Y);
      if(com->hasZ()) microstepMode(2, (uint8_t)com->Z);
      if(com->hasE()) microstepMode(3, (uint8_t)com->E);
      if(com->hasP()) microstepMode(4, (uint8_t)com->P); // Original B but is not supported here
      if(com->hasR()) microstepMode(5, (uint8_t)com->R);
      microstepReadings();
#endif
    }
      break;
    case 360: // M360 - show configuration
      Com::writeToAll = false;
      Printer::showConfiguration();
      break;
    case 400: // M400 Finish all moves
      Commands::waitUntilEndOfAllMoves();
      break;
    case 401: // M401 Memory position
      Printer::MemoryPosition();
      break;
    case 402: // M402 Go to stored position
      Printer::GoToMemoryPosition(com->hasX(), com->hasY(), com->hasZ(), com->hasE(), (com->hasF() ? com->F : Printer::feedrate));
      break;
    case 450:
      Printer::reportPrinterMode();
      break;
    case 451:
      waitUntilEndOfAllMoves();
      Printer::mode = PRINTER_MODE_FFF;
      Printer::reportPrinterMode();
      break;
    case 452:
      Printer::reportPrinterMode();
      break;
    case 453:
      Printer::reportPrinterMode();
      break;
    case 500: { // M500
      EEPROM::storeDataIntoEEPROM(false);
      Com::printInfoFLN(PSTR("Configuration stored to EEPROM."));
    }
      break;
    case 501: { // M501
      EEPROM::readDataFromEEPROM(true);
      Extruder::selectExtruderById(Extruder::current->id);
      Com::printInfoFLN(PSTR("Configuration loaded from EEPROM."));
    }
      break;
    case 502: // M502
      EEPROM::restoreEEPROMSettingsFromConfiguration();
      break;
      //- M530 S<printing> L<layer> - Enables explicit printing mode (S1) or disables it (S0). L can set layer count
    case 530:
      if(com->hasL())
        Printer::maxLayer = static_cast<int>(com->L);
      if(com->hasS())
        Printer::setPrinting(static_cast<uint8_t>(com->S));
      else {
        Printer::setPrinting(0);
      }
      Printer::setMenuMode(MENU_MODE_PAUSED, false);
      UI_RESET_MENU
        break;
      //- M531 filename - Define filename being printed
    case 531:
      strncpy(Printer::printName, com->text, 20);
      Printer::printName[20] = 0;
      break;
      //- M532 X<percent> L<curLayer> - update current print state progress (X=0..100) and layer L
    case 532:
      if(com->hasX())
        Printer::progress = com->X;
      if(Printer::progress > 100.0)
        Printer::progress = 100.0;
      else if(Printer::progress < 0)
        Printer::progress = 0;
      if(com->hasL())
        Printer::currentLayer = static_cast<int>(com->L);
      break;
#ifdef DEBUG_QUEUE_MOVE
    case 533: { // M533 Write move data
      InterruptProtectedBlock noInts;
      int lc = (int)PrintLine::linesCount;
      int lp = (int)PrintLine::linesPos;
      int wp = (int)PrintLine::linesWritePos;
      int n = (wp - lp);
      if(n < 0) n += PRINTLINE_CACHE_SIZE;
      noInts.unprotect();
      if(n != lc)
        Com::printFLN(PSTR("Buffer corrupted"));
      Com::printF(PSTR("Buf:"), lc);
      Com::printF(PSTR(",LP:"), lp);
      Com::printFLN(PSTR(",WP:"), wp);
      if(PrintLine::cur == NULL) {
        Com::printFLN(PSTR("No move"));
        if(PrintLine::linesCount > 0) {
          PrintLine &cur = PrintLine::lines[PrintLine::linesPos];
          Com::printF(PSTR("JFlags:"), (int)cur.joinFlags);
          Com::printFLN(PSTR(" Flags:"), (int)cur.flags);
          if(cur.isWarmUp()) {
            Com::printFLN(PSTR(" warmup:"), (int)cur.getWaitForXLinesFilled());
          }
        }
      } else {
        Com::printF(PSTR("Rem:"), PrintLine::cur->stepsRemaining);
        Com::printFLN(PSTR(" Int:"), Printer::interval);
      }
    }
      break;
#endif // DEBUG_QUEUE_MOVE
#ifdef DEBUG_SEGMENT_LENGTH
    case 534: // M534
      Com::printFLN(PSTR("Max. segment size:"), Printer::maxRealSegmentLength);
      if(com->hasS())
        Printer::maxRealSegmentLength = 0;
      break;
#endif
#ifdef DEBUG_REAL_JERK
      Com::printFLN(PSTR("Max. jerk measured:"), Printer::maxRealJerk);
      if(com->hasS())
        Printer::maxRealJerk = 0;
      break;
#endif
    case 539:
      if(com->hasS()) {
        Printer::setSupportStartStop(com->S != 0);
      }
      if(com->hasP()) {
        if(com->P) {
          Printer::setMenuMode(MENU_MODE_PAUSED, true);
        } else {
          Printer::setMenuMode(MENU_MODE_PAUSED, false);
          UI_RESET_MENU
            }
      }
      break;
    case 601:
      if(com->hasS() && com->S > 0)
        Extruder::pauseExtruders(com->hasB() && com->B != 0);
      else
        Extruder::unpauseExtruders(com->hasP() && com->P != 1);
      break;
    case 670:
      if(com->hasS()) {
        HAL::eprSetByte(EPR_VERSION, static_cast<uint8_t>(com->S));
        HAL::eprSetByte(EPR_INTEGRITY_BYTE, EEPROM::computeChecksum());
      }
      break;
    case 907: { // M907 Set digital trimpot/DAC motor current using axis codes.
      // If "S" is specified, use that as initial default value, then update each axis w/ specific values as found later.
      if(com->hasS()) {
        for(int i = 0; i < 10; i++) {
          setMotorCurrentPercent(i, com->S);
        }
      }

      if(com->hasX()) setMotorCurrentPercent(0, (float)com->X);
      if(com->hasY()) setMotorCurrentPercent(1, (float)com->Y);
      if(com->hasZ()) setMotorCurrentPercent(2, (float)com->Z);
      if(com->hasE()) setMotorCurrentPercent(3, (float)com->E);
    }
      break;
    case 908: { // M908 Control digital trimpot directly.
      uint8_t channel, current;
      if(com->hasP() && com->hasS()) {
        setMotorCurrent((uint8_t)com->P, (unsigned int)com->S);
      }
    }
      break;
    case 909: { // M909 Read digital trimpot settings.
    }
      break;
    case 910: // M910 - Commit digipot/DAC value to external EEPROM
      break;
#if FEATURE_AUTOLEVEL && FEATURE_Z_PROBE
    case 890: {
      if(com->hasX() && com->hasY()) {
        float c = Printer::bendingCorrectionAt(com->X, com->Y);
        Com::printF(PSTR("Bending at ("), com->X);
        Com::printF(PSTR(","), com->Y);
        Com::printFLN(PSTR(") = "), c);
      }
    }
      break;
#endif
    case 998:
      uid.showMessage(com->S);
      break;
    case 999: // Stop fatal error take down
      if(com->hasS())
        GCode::fatalError(PSTR("Testing fatal error"));
      else
        GCode::resetFatalError();
      break;
    default:
      if(Printer::debugErrors()) {
        Com::writeToAll = false;
        Com::printF(PSTR("Unknown command:"));
        com->printCommand();
      }
  }
}

/**
   \brief Execute the command stored in com.
*/
void Commands::executeGCode(GCode *com) {
  // Set return channel for private commands. By default all commands send to all receivers.
  GCodeSource *actSource = GCodeSource::activeSource;
  GCodeSource::activeSource = com->source;
  Com::writeToAll = true;

  if (INCLUDE_DEBUG_COMMUNICATION) {
    if(Printer::debugCommunication()) {
      if(com->hasG() || (com->hasM() && com->M != 111)) {
        previousMillisCmd = HAL::timeInMilliseconds();
        GCodeSource::activeSource = actSource;
        return;
      }
    }
  }
  if(com->hasG()) processGCode(com);
  else if(com->hasM()) processMCode(com);
  else if(com->hasT()) {    // Process T code
    //com->printCommand(); // for testing if this the source of extruder switches
    Commands::waitUntilEndOfAllMoves();
    Extruder::selectExtruderById(com->T);
  } else {
    if(Printer::debugErrors()) {
      Com::printF(PSTR("Unknown command:"));
      com->printCommand();
    }
  }
#ifdef DEBUG_DRYRUN_ERROR
  if(Printer::debugDryrun()) {
    Com::printFLN(PSTR("Dryrun was enabled"));
    com->printCommand();
    Printer::debugReset(8);
  }
#endif
  GCodeSource::activeSource = actSource;
}

void Commands::emergencyStop() {
#if defined(KILL_METHOD) && KILL_METHOD == 1
  HAL::resetHardware();
#else
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

  UI_STATUS_UPD_F(PSTR("Killed"));
  HAL::delayMilliseconds(200);
  InterruptProtectedBlock noInts;
  while(1) {}
#endif
}

void Commands::checkFreeMemory() {
  int newfree = HAL::getFreeRam();
  if(newfree < lowestRAMValue)
    lowestRAMValue = newfree;
}

void Commands::writeLowestFreeRAM() {
  if(lowestRAMValueSend > lowestRAMValue) {
    lowestRAMValueSend = lowestRAMValue;
    Com::printFLN(PSTR("Free RAM:"), lowestRAMValue);
  }
}
