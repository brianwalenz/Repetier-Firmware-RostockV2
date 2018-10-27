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

#include "Repetier.h"

#if USE_ADVANCE
ufast8_t Printer::maxExtruderSpeed;            ///< Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//uint8_t Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases
#endif
uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.
//Stepper Movement Variables
float Printer::axisStepsPerMM[E_AXIS_ARRAY] = {XAXIS_STEPS_PER_MM, YAXIS_STEPS_PER_MM, ZAXIS_STEPS_PER_MM, 1}; ///< Number of steps per mm needed.
float Printer::invAxisStepsPerMM[E_AXIS_ARRAY]; ///< Inverse of axisStepsPerMM for faster conversion

float Printer::maxFeedrate[E_AXIS_ARRAY]    = { 300, 300, 300 };
float Printer::homingFeedrate[Z_AXIS_ARRAY] = { 120, 120, 120 };

#if RAMP_ACCELERATION
//  float max_start_speed_units_per_second[E_AXIS_ARRAY] = MAX_START_SPEED_UNITS_PER_SECOND; ///< Speed we can use, without acceleration.
float Printer::maxAccelerationMMPerSquareSecond[E_AXIS_ARRAY] = {MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts
float Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS_ARRAY] = {MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y, MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z}; ///< X, Y, Z max acceleration in mm/s^2 for travel moves
/** Acceleration in steps/s^3 in printing mode.*/
unsigned long Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS_ARRAY];
/** Acceleration in steps/s^2 in movement mode.*/
unsigned long Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS_ARRAY];
// uint32_t Printer::maxInterval;
#endif

long Printer::currentNonlinearPositionSteps[E_TOWER_ARRAY];
uint8_t lastMoveID = 0; // Last move ID

#if FEATURE_BABYSTEPPING
int16_t Printer::zBabystepsMissing = 0;
int16_t Printer::zBabysteps = 0;
#endif
uint8_t Printer::relativeCoordinateMode = false;  ///< Determines absolute (false) or relative Coordinates (true).
uint8_t Printer::relativeExtruderCoordinateMode = false;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

long Printer::currentPositionSteps[E_AXIS_ARRAY];
float Printer::currentPosition[Z_AXIS_ARRAY];
float Printer::lastCmdPos[Z_AXIS_ARRAY];
long Printer::destinationSteps[E_AXIS_ARRAY];
float Printer::coordinateOffset[Z_AXIS_ARRAY] = {0, 0, 0};
uint8_t Printer::flag0 = 0;
uint8_t Printer::flag1 = 0;
uint8_t Printer::flag2 = 0;
uint8_t Printer::flag3 = 0;
uint8_t Printer::debugLevel = 6; ///< Bitfield defining debug output. 1 = echo, 2 = info, 4 = error, 8 = dry run., 16 = Only communication, 32 = No moves
fast8_t Printer::stepsPerTimerCall = 1;
uint16_t Printer::menuMode = 0;
uint8_t Printer::mode = DEFAULT_PRINTER_MODE;
uint8_t Printer::fanSpeed = 0; // Last fan speed set with M106/M107
float Printer::extrudeMultiplyError = 0;
float Printer::extrusionFactor = 1.0;
uint8_t Printer::interruptEvent = 0;
int Printer::currentLayer = 0;
int Printer::maxLayer = -1; // -1 = unknown
char Printer::printName[21] = ""; // max. 20 chars + 0
float Printer::progress = 0;
millis_t Printer::lastTempReport = 0;

#if FEATURE_AUTOLEVEL
float Printer::autolevelTransformation[9]; ///< Transformation matrix
#endif

uint32_t Printer::interval = 30000;           ///< Last step duration in ticks.
uint32_t Printer::timer;              ///< used for acceleration/deceleration timing
uint32_t Printer::stepNumber;         ///< Step number in current move.

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
int32_t Printer::advanceExecuted;             ///< Executed advance steps
#endif
int Printer::advanceStepsSet;
#endif

int32_t Printer::maxDeltaPositionSteps;
floatLong Printer::deltaDiagonalStepsSquaredA;
floatLong Printer::deltaDiagonalStepsSquaredB;
floatLong Printer::deltaDiagonalStepsSquaredC;
float Printer::deltaMaxRadiusSquared;
float Printer::radius0;
int32_t Printer::deltaFloorSafetyMarginSteps = 0;
int32_t Printer::deltaAPosXSteps;
int32_t Printer::deltaAPosYSteps;
int32_t Printer::deltaBPosXSteps;
int32_t Printer::deltaBPosYSteps;
int32_t Printer::deltaCPosXSteps;
int32_t Printer::deltaCPosYSteps;
int32_t Printer::realDeltaPositionSteps[TOWER_ARRAY];
int16_t Printer::travelMovesPerSecond;
int16_t Printer::printMovesPerSecond;
int32_t Printer::stepsRemainingAtZHit;
int32_t Printer::stepsRemainingAtXHit;
int32_t Printer::stepsRemainingAtYHit;
#if SOFTWARE_LEVELING
int32_t Printer::levelingP1[3];
int32_t Printer::levelingP2[3];
int32_t Printer::levelingP3[3];
#endif
//float Printer::minimumSpeed;               ///< lowest allowed speed to keep integration error small
//float Printer::minimumZSpeed;
int32_t Printer::xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
int32_t Printer::yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
int32_t Printer::zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
int32_t Printer::xMinSteps;                   ///< For software endstops, limit of move in negative direction.
int32_t Printer::yMinSteps;                   ///< For software endstops, limit of move in negative direction.
int32_t Printer::zMinSteps;                   ///< For software endstops, limit of move in negative direction.
float Printer::xLength;
float Printer::xMin;
float Printer::yLength;
float Printer::yMin;
float Printer::zLength;
float Printer::zMin;
float Printer::feedrate;                   ///< Last requested feedrate.
int Printer::feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
unsigned int Printer::extrudeMultiply;     ///< Flow multiplier in percent (factor 1 = 100)
float Printer::maxJerk;                    ///< Maximum allowed jerk in mm/s
float Printer::offsetX;                     ///< X-offset for different extruder positions.
float Printer::offsetY;                     ///< Y-offset for different extruder positions.
float Printer::offsetZ;                     ///< Z-offset for different extruder positions.
float Printer::offsetZ2 = 0;                ///< Z-offset without rotation correction.
speed_t Printer::vMaxReached;               ///< Maximum reached speed
uint32_t Printer::msecondsPrinting;         ///< Milliseconds of printing time (means time with heated extruder)
float Printer::filamentPrinted;             ///< mm of filament printed since counting started
float Printer::memoryX = IGNORE_COORDINATE;
float Printer::memoryY = IGNORE_COORDINATE;
float Printer::memoryZ = IGNORE_COORDINATE;
float Printer::memoryE = IGNORE_COORDINATE;
float Printer::memoryF = -1;

#ifdef DEBUG_SEGMENT_LENGTH
float Printer::maxRealSegmentLength = 0;
#endif
#ifdef DEBUG_REAL_JERK
float Printer::maxRealJerk = 0;
#endif

#ifdef DEBUG_PRINT
int debugWaitLoop = 0;
#endif

#if FEATURE_Z_PROBE
fast8_t Printer::wizardStackPos;
wizardVar Printer::wizardStack[WIZARD_STACK_SIZE];
#endif

void Printer::setDebugLevel(uint8_t newLevel) {
  if(newLevel != debugLevel) {
    debugLevel = newLevel;
    if(debugDryrun()) {
      // Disable all heaters in case they were on
      Extruder::disableAllHeater();
    }
  }
  Com::printFLN(PSTR("DebugLevel:"), (int)newLevel);
}

void Printer::toggleEcho() {
  setDebugLevel(debugLevel ^ 1);
}

void Printer::toggleInfo() {
  setDebugLevel(debugLevel ^ 2);
}

void Printer::toggleErrors() {
  setDebugLevel(debugLevel ^ 4);
}

void Printer::toggleDryRun() {
  setDebugLevel(debugLevel ^ 8);
}

void Printer::toggleCommunication() {
  setDebugLevel(debugLevel ^ 16);
}

void Printer::toggleNoMoves() {
  setDebugLevel(debugLevel ^ 32);
}

void Printer::toggleEndStop() {
  setDebugLevel(debugLevel ^ 64);
}

bool Printer::isPositionAllowed(float x, float y, float z) {
  if(isNoDestinationCheck()) return true;
  bool allowed = true;
  if(!isHoming()) {
    allowed = allowed && (z >= 0) && (z <= zLength + 0.05 + ENDSTOP_Z_BACK_ON_HOME);
    allowed = allowed && (x * x + y * y <= deltaMaxRadiusSquared);
  }
  if(!allowed) {
    Printer::updateCurrentPosition(true);
    Commands::printCurrentPosition();
  }
  return allowed;
}

void Printer::setFanSpeedDirectly(uint8_t speed) {
	uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
  if(pwm_pos[PWM_FAN1] == trimmedSpeed)
    return;
#if FAN_KICKSTART_TIME
  if(fanKickstart == 0 && speed > pwm_pos[PWM_FAN1] && speed < 85) {
    if(pwm_pos[PWM_FAN1]) fanKickstart = FAN_KICKSTART_TIME / 100;
    else                  fanKickstart = FAN_KICKSTART_TIME / 25;
  }
#endif
  pwm_pos[PWM_FAN1] = trimmedSpeed;
#endif
}
void Printer::setFan2SpeedDirectly(uint8_t speed) {
	uint8_t trimmedSpeed = TRIM_FAN_PWM(speed);
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
  if(pwm_pos[PWM_FAN2] == trimmedSpeed)
    return;
#if FAN_KICKSTART_TIME
  if(fan2Kickstart == 0 && speed > pwm_pos[PWM_FAN2] && speed < 85) {
    if(pwm_pos[PWM_FAN2]) fan2Kickstart = FAN_KICKSTART_TIME / 100;
    else                  fan2Kickstart = FAN_KICKSTART_TIME / 25;
  }
#endif
  pwm_pos[PWM_FAN2] = trimmedSpeed;
#endif
}

void Printer::updateDerivedParameter() {
  travelMovesPerSecond = EEPROM::deltaSegmentsPerSecondMove();
  printMovesPerSecond = EEPROM::deltaSegmentsPerSecondPrint();
  if(travelMovesPerSecond < 15) travelMovesPerSecond = 15; // lower values make no sense and can cause serious problems
  if(printMovesPerSecond < 15) printMovesPerSecond = 15;
  axisStepsPerMM[X_AXIS] = axisStepsPerMM[Y_AXIS] = axisStepsPerMM[Z_AXIS];
  maxAccelerationMMPerSquareSecond[X_AXIS] = maxAccelerationMMPerSquareSecond[Y_AXIS] = maxAccelerationMMPerSquareSecond[Z_AXIS];
  homingFeedrate[X_AXIS] = homingFeedrate[Y_AXIS] = homingFeedrate[Z_AXIS];
  maxFeedrate[X_AXIS] = maxFeedrate[Y_AXIS] = maxFeedrate[Z_AXIS];
  maxTravelAccelerationMMPerSquareSecond[X_AXIS] = maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = maxTravelAccelerationMMPerSquareSecond[Z_AXIS];
  zMaxSteps = axisStepsPerMM[Z_AXIS] * (zLength);
  towerAMinSteps = axisStepsPerMM[A_TOWER] * xMin;
  towerBMinSteps = axisStepsPerMM[B_TOWER] * yMin;
  towerCMinSteps = axisStepsPerMM[C_TOWER] * zMin;
  //radius0 = EEPROM::deltaHorizontalRadius();
  float radiusA = radius0 + EEPROM::deltaRadiusCorrectionA();
  float radiusB = radius0 + EEPROM::deltaRadiusCorrectionB();
  float radiusC = radius0 + EEPROM::deltaRadiusCorrectionC();
  deltaAPosXSteps = floor(radiusA * cos(EEPROM::deltaAlphaA() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaAPosYSteps = floor(radiusA * sin(EEPROM::deltaAlphaA() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaBPosXSteps = floor(radiusB * cos(EEPROM::deltaAlphaB() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaBPosYSteps = floor(radiusB * sin(EEPROM::deltaAlphaB() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaCPosXSteps = floor(radiusC * cos(EEPROM::deltaAlphaC() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaCPosYSteps = floor(radiusC * sin(EEPROM::deltaAlphaC() * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaDiagonalStepsSquaredA.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionA() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
  deltaDiagonalStepsSquaredB.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionB() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
  deltaDiagonalStepsSquaredC.l = static_cast<uint32_t>((EEPROM::deltaDiagonalCorrectionC() + EEPROM::deltaDiagonalRodLength()) * axisStepsPerMM[Z_AXIS]);
  if(deltaDiagonalStepsSquaredA.l > 65534 || 2 * radius0 * axisStepsPerMM[Z_AXIS] > 65534) {
    setLargeMachine(true);
#ifdef SUPPORT_64_BIT_MATH
    deltaDiagonalStepsSquaredA.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredA.l));
    deltaDiagonalStepsSquaredB.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredB.l));
    deltaDiagonalStepsSquaredC.L = RMath::sqr(static_cast<uint64_t>(deltaDiagonalStepsSquaredC.l));
#else
    deltaDiagonalStepsSquaredA.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredA.l));
    deltaDiagonalStepsSquaredB.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredB.l));
    deltaDiagonalStepsSquaredC.f = RMath::sqr(static_cast<float>(deltaDiagonalStepsSquaredC.l));
#endif
  } else {
    setLargeMachine(false);
    deltaDiagonalStepsSquaredA.l = RMath::sqr(deltaDiagonalStepsSquaredA.l);
    deltaDiagonalStepsSquaredB.l = RMath::sqr(deltaDiagonalStepsSquaredB.l);
    deltaDiagonalStepsSquaredC.l = RMath::sqr(deltaDiagonalStepsSquaredC.l);
  }
  deltaMaxRadiusSquared = RMath::sqr(EEPROM::deltaMaxRadius());
  long cart[Z_AXIS_ARRAY], delta[TOWER_ARRAY];
  cart[X_AXIS] = cart[Y_AXIS] = 0;
  cart[Z_AXIS] = zMaxSteps;
  transformCartesianStepsToDeltaSteps(cart, delta);
  maxDeltaPositionSteps = delta[0];
  xMaxSteps = yMaxSteps = zMaxSteps;
  xMinSteps = yMinSteps = zMinSteps = 0;
  deltaFloorSafetyMarginSteps = DELTA_FLOOR_SAFETY_MARGIN_MM * axisStepsPerMM[Z_AXIS];

  for(uint8_t i = 0; i < E_AXIS_ARRAY; i++) {
    invAxisStepsPerMM[i] = 1.0f / axisStepsPerMM[i];
#ifdef RAMP_ACCELERATION
    /** Acceleration in steps/s^3 in printing mode.*/
    maxPrintAccelerationStepsPerSquareSecond[i] = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
    /** Acceleration in steps/s^2 in movement mode.*/
    maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
#endif
  }
	// For numeric stability we need to start accelerations at a minimum speed and hence ensure that the
	// jerk is at least 2 * minimum speed.

	// For xy moves the minimum speed is multiplied with 1.41 to enforce the condition also for diagonals since the
	// driving axis is the problematic speed.
  float accel = RMath::max(maxAccelerationMMPerSquareSecond[X_AXIS], maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
  float minimumSpeed = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[X_AXIS] * accel));
  accel = RMath::max(maxAccelerationMMPerSquareSecond[Y_AXIS], maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
  float minimumSpeed2 = 1.41 * accel * sqrt(2.0f / (axisStepsPerMM[Y_AXIS] * accel));
	if(minimumSpeed2 > minimumSpeed) {
		minimumSpeed = minimumSpeed2;
	}
  if(maxJerk < 2 * minimumSpeed) {// Enforce minimum start speed if target is faster and jerk too low
    maxJerk = 2 * minimumSpeed;
    Com::printFLN(PSTR("XY jerk was too low, setting to "), maxJerk);
  }
  accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS], maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#if DISTORTION_CORRECTION
  distortion.updateDerived();
#endif // DISTORTION_CORRECTION
  Printer::updateAdvanceFlags();
}

/**
   \brief Stop heater and stepper motors. Disable power,if possible.
*/
void Printer::kill(uint8_t onlySteppers) {
  if(areAllSteppersDisabled() && onlySteppers) return;
  if(Printer::isAllKilled()) return;
  disableXStepper();
  disableYStepper();
  disableZStepper();
  Extruder::disableAllExtruderMotors();
  setAllSteppersDiabled();
  unsetHomedAll();
  if(!onlySteppers) {
    for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
      Extruder::setTemperatureForExtruder(0, i);
    Extruder::setHeatedBedTemperature(0);

    uid.setStatusP(PSTR("Standby"));
    uid.refreshPage();

#if defined(PS_ON_PIN) && PS_ON_PIN>-1 && !defined(NO_POWER_TIMEOUT)
    //pinMode(PS_ON_PIN,INPUT);
    SET_OUTPUT(PS_ON_PIN); //GND
    WRITE(PS_ON_PIN, HIGH);
    Printer::setPowerOn(false);
#endif
    Printer::setAllKilled(true);
  } else {
    uid.setStatusP(PSTR("Stepper disabled"));
    uid.refreshPage();
  }

#if FAN_BOARD_PIN > -1
#if HAVE_HEATED_BED
  if(heatedBedController.targetTemperatureC < 15)      // turn off FAN_BOARD only if bed heater is off
#endif
    pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif // FAN_BOARD_PIN
  Commands::printTemperatures(false);
}

void Printer::updateAdvanceFlags() {
  Printer::setAdvanceActivated(false);
#if USE_ADVANCE
  for(uint8_t i = 0; i < NUM_EXTRUDER; i++) {
    if(extruder[i].advanceL != 0) {
      Printer::setAdvanceActivated(true);
    }
#if ENABLE_QUADRATIC_ADVANCE
    if(extruder[i].advanceK != 0) Printer::setAdvanceActivated(true);
#endif
  }
#endif
}

void Printer::moveToParkPosition() {
  if(Printer::isHomedAll()) { // for safety move only when homed!
    moveToReal(EEPROM::parkX(),EEPROM::parkY(),IGNORE_COORDINATE,IGNORE_COORDINATE, Printer::maxFeedrate[X_AXIS], true);
    moveToReal(IGNORE_COORDINATE,IGNORE_COORDINATE,RMath::min(zMin + zLength, currentPosition[Z_AXIS] + EEPROM::parkZ()),IGNORE_COORDINATE, Printer::maxFeedrate[Z_AXIS], true);
  }
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
uint8_t Printer::moveTo(float x, float y, float z, float e, float f) {
  if(x != IGNORE_COORDINATE)
    destinationSteps[X_AXIS] = (x + Printer::offsetX) * axisStepsPerMM[X_AXIS];
  if(y != IGNORE_COORDINATE)
    destinationSteps[Y_AXIS] = (y + Printer::offsetY) * axisStepsPerMM[Y_AXIS];
  if(z != IGNORE_COORDINATE)
    destinationSteps[Z_AXIS] = (z + Printer::offsetZ) * axisStepsPerMM[Z_AXIS];
  if(e != IGNORE_COORDINATE)
    destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
	else
		destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];
  if(f != IGNORE_COORDINATE)
    feedrate = f;
  // Disable software end stop or we get wrong distances when length < real length
  if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, true, false)) {
    Com::printWarningFLN(PSTR("moveTo / queueDeltaMove returns error"));
    return 0;
  }
  updateCurrentPosition(false);
  return 1;
}

uint8_t Printer::moveToReal(float x, float y, float z, float e, float f, bool pathOptimize) {
  if(x == IGNORE_COORDINATE)
    x = currentPosition[X_AXIS];
  else
    currentPosition[X_AXIS] = x;
  if(y == IGNORE_COORDINATE)
    y = currentPosition[Y_AXIS];
  else
    currentPosition[Y_AXIS] = y;
  if(z == IGNORE_COORDINATE)
    z = currentPosition[Z_AXIS];
  else
    currentPosition[Z_AXIS] = z;
  transformToPrinter(x + Printer::offsetX, y + Printer::offsetY, z + Printer::offsetZ, x, y, z);
  z += offsetZ2;
  // There was conflicting use of IGNOR_COORDINATE
  destinationSteps[X_AXIS] = static_cast<int32_t>(floor(x * axisStepsPerMM[X_AXIS] + 0.5f));
  destinationSteps[Y_AXIS] = static_cast<int32_t>(floor(y * axisStepsPerMM[Y_AXIS] + 0.5f));
  destinationSteps[Z_AXIS] = static_cast<int32_t>(floor(z * axisStepsPerMM[Z_AXIS] + 0.5f));
  if(e != IGNORE_COORDINATE && !Printer::debugDryrun()
#if MIN_EXTRUDER_TEMP > 30
     && (Extruder::current->tempControl.currentTemperatureC > MIN_EXTRUDER_TEMP || Printer::isColdExtrusionAllowed() || Extruder::current->tempControl.sensorType == 0)
#endif
     ) {
    destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
  } else {
		destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];
	}
  if(f != IGNORE_COORDINATE)
    feedrate = f;

  if (!PrintLine::queueNonlinearMove(ALWAYS_CHECK_ENDSTOPS, pathOptimize, true)) {
    Com::printWarningFLN(PSTR("moveToReal / queueDeltaMove returns error"));

#ifdef DEBUG
    Com::printF(PSTR(" x=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printFLN(PSTR(" mm"));
    Com::printF(PSTR(" y=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printFLN(PSTR(" mm"));
    Com::printF(PSTR(" z=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printFLN(PSTR(" mm"));
#endif

    return 0;
  }
  return 1;
}

void Printer::setOrigin(float xOff, float yOff, float zOff) {
  coordinateOffset[X_AXIS] = xOff;// offset from G92
  coordinateOffset[Y_AXIS] = yOff;
  coordinateOffset[Z_AXIS] = zOff;
}

/** Computes currentPosition from currentPositionSteps including correction for offset. */
void Printer::updateCurrentPosition(bool copyLastCmd) {
  currentPosition[X_AXIS] = static_cast<float>(currentPositionSteps[X_AXIS]) * invAxisStepsPerMM[X_AXIS];
  currentPosition[Y_AXIS] = static_cast<float>(currentPositionSteps[Y_AXIS]) * invAxisStepsPerMM[Y_AXIS];
  currentPosition[Z_AXIS] = static_cast<float>(currentPositionSteps[Z_AXIS]) * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
  transformFromPrinter(currentPosition[X_AXIS], currentPosition[Y_AXIS], currentPosition[Z_AXIS],
                       currentPosition[X_AXIS], currentPosition[Y_AXIS], currentPosition[Z_AXIS]);
  currentPosition[X_AXIS] -= Printer::offsetX; // Offset from active extruder or z probe
  currentPosition[Y_AXIS] -= Printer::offsetY;
  currentPosition[Z_AXIS] -= Printer::offsetZ;
  if(copyLastCmd) {
    lastCmdPos[X_AXIS] = currentPosition[X_AXIS];
    lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS];
    lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS];
  }
}

void Printer::updateCurrentPositionSteps() {
  float x_rotc, y_rotc, z_rotc;
  transformToPrinter(currentPosition[X_AXIS] + Printer::offsetX, currentPosition[Y_AXIS] + Printer::offsetY, currentPosition[Z_AXIS] +  Printer::offsetZ, x_rotc, y_rotc, z_rotc);
  z_rotc += offsetZ2;
  currentPositionSteps[X_AXIS] = static_cast<int32_t>(floor(x_rotc * axisStepsPerMM[X_AXIS] + 0.5f));
  currentPositionSteps[Y_AXIS] = static_cast<int32_t>(floor(y_rotc * axisStepsPerMM[Y_AXIS] + 0.5f));
  currentPositionSteps[Z_AXIS] = static_cast<int32_t>(floor(z_rotc * axisStepsPerMM[Z_AXIS] + 0.5f));
  transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);
}

/** \brief Sets the destination coordinates to values stored in com.

    Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
    position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
    \param com g-code with new destination position.
    \return true if it is a move, false if no move results from coordinates.
*/

uint8_t Printer::setDestinationStepsFromGCode(GCode *com) {
  register int32_t p;
  float x, y, z;
  bool posAllowed = true;
#if MOVE_X_WHEN_HOMED == 1 || MOVE_Y_WHEN_HOMED == 1 || MOVE_Z_WHEN_HOMED == 1
  if(!isNoDestinationCheck()) {
#if MOVE_X_WHEN_HOMED
    if(!isXHomed())
      com->unsetX();
#endif
#if MOVE_Y_WHEN_HOMED
    if(!isYHomed())
      com->unsetY();
#endif
#if MOVE_Z_WHEN_HOMED
    if(!isZHomed())
      com->unsetZ();
#endif
  }
#endif
#if DISTORTION_CORRECTION == 0
  if(!com->hasNoXYZ()) {
#endif
    if(!relativeCoordinateMode) {
      if(com->hasX()) lastCmdPos[X_AXIS] = currentPosition[X_AXIS] = convertToMM(com->X) - coordinateOffset[X_AXIS];
      if(com->hasY()) lastCmdPos[Y_AXIS] = currentPosition[Y_AXIS] = convertToMM(com->Y) - coordinateOffset[Y_AXIS];
      if(com->hasZ()) lastCmdPos[Z_AXIS] = currentPosition[Z_AXIS] = convertToMM(com->Z) - coordinateOffset[Z_AXIS];
    } else {
      if(com->hasX()) currentPosition[X_AXIS] = (lastCmdPos[X_AXIS] += convertToMM(com->X));
      if(com->hasY()) currentPosition[Y_AXIS] = (lastCmdPos[Y_AXIS] += convertToMM(com->Y));
      if(com->hasZ()) currentPosition[Z_AXIS] = (lastCmdPos[Z_AXIS] += convertToMM(com->Z));
    }
    transformToPrinter(lastCmdPos[X_AXIS] + Printer::offsetX, lastCmdPos[Y_AXIS] + Printer::offsetY, lastCmdPos[Z_AXIS] +  Printer::offsetZ, x, y, z);
    z += offsetZ2;
    destinationSteps[X_AXIS] = static_cast<int32_t>(floor(x * axisStepsPerMM[X_AXIS] + 0.5f));
    destinationSteps[Y_AXIS] = static_cast<int32_t>(floor(y * axisStepsPerMM[Y_AXIS] + 0.5f));
    destinationSteps[Z_AXIS] = static_cast<int32_t>(floor(z * axisStepsPerMM[Z_AXIS] + 0.5f));
    posAllowed = com->hasNoXYZ() || Printer::isPositionAllowed(lastCmdPos[X_AXIS], lastCmdPos[Y_AXIS], lastCmdPos[Z_AXIS]);
#if DISTORTION_CORRECTION == 0
  }
#endif
  if(com->hasE() && !Printer::debugDryrun()) {
    p = convertToMM(com->E * axisStepsPerMM[E_AXIS]);
    if(relativeCoordinateMode || relativeExtruderCoordinateMode) {
      if(
#if MIN_EXTRUDER_TEMP > 20
         (Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0) ||
#endif
         fabs(com->E) * extrusionFactor > EXTRUDE_MAXLENGTH)
        p = 0;
      destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS] + p;
    } else {
      if(
#if MIN_EXTRUDER_TEMP > 20
         (Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP  && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0) ||
#endif
         fabs(p - currentPositionSteps[E_AXIS]) * extrusionFactor > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS])
        currentPositionSteps[E_AXIS] = p;
      destinationSteps[E_AXIS] = p;
    }
  } else Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
  if(com->hasF() && com->F > 0.1) {
    if(unitIsInches)
      feedrate = com->F * 0.0042333f * (float)feedrateMultiply;  // Factor is 25.5/60/100
    else
      feedrate = com->F * (float)feedrateMultiply * 0.00016666666f;
  }
  if(!posAllowed) {
    currentPositionSteps[E_AXIS] = destinationSteps[E_AXIS];
    return false; // ignore move
  }
  return !com->hasNoXYZ() || (com->hasE() && destinationSteps[E_AXIS] != currentPositionSteps[E_AXIS]); // ignore unproductive moves
}

void Printer::setup() {
  HAL::stopWatchdog();

  for(uint8_t i = 0; i < NUM_PWM; i++)
    pwm_pos[i] = 0;


#if defined(MB_SETUP)
  MB_SETUP;
#endif

  //HAL::delayMilliseconds(500);  // add a delay at startup to give hardware time for initalization

#if defined(EEPROM_AVAILABLE) && defined(EEPROM_SPI_ALLIGATOR) && EEPROM_AVAILABLE == EEPROM_SPI_ALLIGATOR
  HAL::spiBegin();
#endif

  HAL::hwSetup();

#ifdef ANALYZER
  // Channel->pin assignments
#if ANALYZER_CH0>=0
  SET_OUTPUT(ANALYZER_CH0);
#endif
#if ANALYZER_CH1>=0
  SET_OUTPUT(ANALYZER_CH1);
#endif
#if ANALYZER_CH2>=0
  SET_OUTPUT(ANALYZER_CH2);
#endif
#if ANALYZER_CH3>=0
  SET_OUTPUT(ANALYZER_CH3);
#endif
#if ANALYZER_CH4>=0
  SET_OUTPUT(ANALYZER_CH4);
#endif
#if ANALYZER_CH5>=0
  SET_OUTPUT(ANALYZER_CH5);
#endif
#if ANALYZER_CH6>=0
  SET_OUTPUT(ANALYZER_CH6);
#endif
#if ANALYZER_CH7>=0
  SET_OUTPUT(ANALYZER_CH7);
#endif
#endif

#if defined(ENABLE_POWER_ON_STARTUP) && ENABLE_POWER_ON_STARTUP && (PS_ON_PIN>-1)
  SET_OUTPUT(PS_ON_PIN); //GND
  WRITE(PS_ON_PIN, LOW);
  Printer::setPowerOn(true);
#else
#if PS_ON_PIN > -1
  SET_OUTPUT(PS_ON_PIN); //GND
  WRITE(PS_ON_PIN, HIGH);
  Printer::setPowerOn(false);
#else
  Printer::setPowerOn(true);
#endif
#endif
  //power to SD reader
#if SDPOWER > -1
  SET_OUTPUT(SDPOWER);
  WRITE(SDPOWER, HIGH);
#endif
#if defined(SDCARDDETECT) && SDCARDDETECT > -1
  SET_INPUT(SDCARDDETECT);
  PULLUP(SDCARDDETECT, HIGH);
#endif


  //Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  SET_OUTPUT(Y_STEP_PIN);
  SET_OUTPUT(Z_STEP_PIN);
  endXYZSteps();
  //Initialize Dir Pins
#if X_DIR_PIN > -1
  SET_OUTPUT(X_DIR_PIN);
#endif
#if Y_DIR_PIN > -1
  SET_OUTPUT(Y_DIR_PIN);
#endif
#if Z_DIR_PIN > -1
  SET_OUTPUT(Z_DIR_PIN);
#endif

  //Steppers default to disabled.
#if X_ENABLE_PIN > -1
  SET_OUTPUT(X_ENABLE_PIN);
  WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
#if Y_ENABLE_PIN > -1
  SET_OUTPUT(Y_ENABLE_PIN);
  WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
#if Z_ENABLE_PIN > -1
  SET_OUTPUT(Z_ENABLE_PIN);
  WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif

	Endstops::setup();

#if FEATURE_Z_PROBE && Z_PROBE_PIN>-1
  SET_INPUT(Z_PROBE_PIN);
#if Z_PROBE_PULLUP
  PULLUP(Z_PROBE_PIN, HIGH);
#endif
#endif // FEATURE_FEATURE_Z_PROBE
#if FAN_PIN > -1 && FEATURE_FAN_CONTROL
  SET_OUTPUT(FAN_PIN);
  WRITE(FAN_PIN, LOW);
#endif
#if FAN2_PIN > -1 && FEATURE_FAN2_CONTROL
  SET_OUTPUT(FAN2_PIN);
  WRITE(FAN2_PIN, LOW);
#endif
#if FAN_BOARD_PIN>-1
  SET_OUTPUT(FAN_BOARD_PIN);
  WRITE(FAN_BOARD_PIN, LOW);
  pwm_pos[PWM_BOARD_FAN] = BOARD_FAN_MIN_SPEED;
#endif
#if defined(EXT0_HEATER_PIN) && EXT0_HEATER_PIN>-1
  SET_OUTPUT(EXT0_HEATER_PIN);
  WRITE(EXT0_HEATER_PIN, HEATER_PINS_INVERTED);
#endif

#if defined(EXT0_EXTRUDER_COOLER_PIN) && EXT0_EXTRUDER_COOLER_PIN>-1
  SET_OUTPUT(EXT0_EXTRUDER_COOLER_PIN);
  WRITE(EXT0_EXTRUDER_COOLER_PIN, LOW);
#endif

#if defined(UI_VOLTAGE_LEVEL) && defined(EXP_VOLTAGE_LEVEL_PIN) && EXP_VOLTAGE_LEVEL_PIN >-1
  SET_OUTPUT(EXP_VOLTAGE_LEVEL_PIN);
  WRITE(EXP_VOLTAGE_LEVEL_PIN, UI_VOLTAGE_LEVEL);
#endif // UI_VOLTAGE_LEVEL

  motorCurrentControlInit(); // Set current if it is firmware controlled

  microstepInit();
#if FEATURE_AUTOLEVEL
  resetTransformationMatrix(true);
#endif // FEATURE_AUTOLEVEL
  feedrate = 50; ///< Current feedrate in mm/s.
  feedrateMultiply = 100;
  extrudeMultiply = 100;
  lastCmdPos[X_AXIS] = lastCmdPos[Y_AXIS] = lastCmdPos[Z_AXIS] = 0;
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  advanceExecuted = 0;
#endif
  advanceStepsSet = 0;
#endif
  maxJerk = MAX_JERK;
  offsetX = offsetY = offsetZ = 0;
  interval = 5000;
  stepsPerTimerCall = 1;
  msecondsPrinting = 0;
  filamentPrinted = 0;
  flag0 = PRINTER_FLAG0_STEPPER_DISABLED;
  xLength = X_MAX_LENGTH;
  yLength = Y_MAX_LENGTH;
  zLength = Z_MAX_LENGTH;
  xMin = X_MIN_POS;
  yMin = Y_MIN_POS;
  zMin = Z_MIN_POS;
  radius0 = ROD_RADIUS;
#if USE_ADVANCE
  extruderStepsNeeded = 0;
#endif
  EEPROM::initBaudrate();
  HAL::serialSetBaudrate(baudrate);
  Com::printFLN(PSTR("start"));
  HAL::showStartReason();
  Extruder::initExtruder();
  // sets auto leveling in eeprom init
  EEPROM::init(); // Read settings from eeprom if wanted
  uid.initialize();
  for(uint8_t i = 0; i < E_AXIS_ARRAY; i++) {
    currentPositionSteps[i] = 0;
  }
  currentPosition[X_AXIS] = currentPosition[Y_AXIS] = currentPosition[Z_AXIS] =  0.0;
  //Commands::printCurrentPosition();
#if DISTORTION_CORRECTION
  distortion.init();
#endif // DISTORTION_CORRECTION

  updateDerivedParameter();
  Commands::checkFreeMemory();
  Commands::writeLowestFreeRAM();
  HAL::setupTimer();

#if FEATURE_WATCHDOG
  HAL::startWatchdog();
#endif // FEATURE_WATCHDOG
  sd.mount();

  transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);

#if DELTA_HOME_ON_POWER
  homeAxis(true, true, true);
#endif
  Commands::printCurrentPosition();

  Extruder::selectExtruderById(0);

#ifdef STARTUP_GCODE
  GCode::executeFString(PSTR(STARTUP_GCODE));
#endif
}

void Printer::defaultLoopActions() {
  Commands::checkForPeriodicalActions(true);  //check heater every n milliseconds
  uid.mediumAction(); // do check encoder
  millis_t curtime = HAL::timeInMilliseconds();
  if(PrintLine::hasLines() || isMenuMode(MODE_PRINTING | MODE_PAUSED))
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
  sd.automount();
#if defined(EEPROM_AVAILABLE) && EEPROM_AVAILABLE == EEPROM_SDCARD
  HAL::syncEEPROM();
#endif

  DEBUG_MEMORY;
}

void Printer::MemoryPosition() {
  Commands::waitUntilEndOfAllMoves();
  updateCurrentPosition(false);
  realPosition(memoryX, memoryY, memoryZ);
  memoryE = currentPositionSteps[E_AXIS] * invAxisStepsPerMM[E_AXIS];
  memoryF = feedrate;
}

void Printer::GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed) {
  if(memoryF < 0) return; // Not stored before call, so we ignore it
  bool all = !(x || y || z);
  moveToReal((all || x ? (lastCmdPos[X_AXIS] = memoryX) : IGNORE_COORDINATE)
             , (all || y ? (lastCmdPos[Y_AXIS] = memoryY) : IGNORE_COORDINATE)
             , (all || z ? (lastCmdPos[Z_AXIS] = memoryZ) : IGNORE_COORDINATE)
             , (e ? memoryE : IGNORE_COORDINATE),
             feed);
  feedrate = memoryF;
  updateCurrentPosition(false);
}


void Printer::deltaMoveToTopEndstops(float feedrate) {
  for (fast8_t i = 0; i < 3; i++)
    Printer::currentPositionSteps[i] = 0;
  Printer::stepsRemainingAtXHit = -1;
  Printer::stepsRemainingAtYHit = -1;
  Printer::stepsRemainingAtZHit = -1;
  setHoming(true);
  transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
  PrintLine::moveRelativeDistanceInSteps(0, 0, (zMaxSteps + EEPROM::deltaDiagonalRodLength()*axisStepsPerMM[Z_AXIS]) * 1.5, 0, feedrate, true, true);
  offsetX = offsetY = offsetZ = offsetZ2 = 0;
  setHoming(false);
}

void Printer::homeXAxis() {
  destinationSteps[X_AXIS] = 0;
  if (!PrintLine::queueNonlinearMove(true, false, false)) {
    Com::printWarningFLN(PSTR("homeXAxis / queueDeltaMove returns error"));
  }
}

void Printer::homeYAxis() {
  Printer::destinationSteps[Y_AXIS] = 0;
  if (!PrintLine::queueNonlinearMove(true, false, false)) {
    Com::printWarningFLN(PSTR("homeYAxis / queueDeltaMove returns error"));
  }
}

void Printer::homeZAxis() { // Delta z homing
  bool homingSuccess = false;
  Endstops::resetAccumulator();
  deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS]);
  // New safe homing routine by Kyrre Aalerud
  // This method will safeguard against sticky endstops such as may be gotten cheaply from china.
  // This can lead to head crashes and even fire, thus a safer algorithm to ensure the endstops actually respond as expected.
  //Endstops::report();
  // Check that all endstops (XYZ) were hit
  Endstops::fillFromAccumulator();
  if (Endstops::xMax() && Endstops::yMax() && Endstops::zMax()) {
    // Back off for retest
    PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_MOVE, 0, Printer::homingFeedrate[Z_AXIS] / ENDSTOP_X_RETEST_REDUCTION_FACTOR, true, true);
    //Endstops::report();
    // Check for proper release of all (XYZ) endstops
    if (!(Endstops::xMax() || Endstops::yMax() || Endstops::zMax())) {
      // Rehome with reduced speed
      Endstops::resetAccumulator();
      deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS] / ENDSTOP_Z_RETEST_REDUCTION_FACTOR);
      Endstops::fillFromAccumulator();
      //Endstops::report();
      // Check that all endstops (XYZ) were hit again
      if (Endstops::xMax() && Endstops::yMax() && Endstops::zMax()) {
        homingSuccess = true; // Assume success in case there is no back move
#if defined(ENDSTOP_Z_BACK_ON_HOME)
        if(ENDSTOP_Z_BACK_ON_HOME > 0) {
          PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -ENDSTOP_Z_BACK_ON_HOME, 0, homingFeedrate[Z_AXIS], true, true);
          //Endstops::report();
          // Check for missing release of any (XYZ) endstop
          if (Endstops::xMax() || Endstops::yMax() || Endstops::zMax()) {
            homingSuccess = false; // Reset success flag
          }
        }
#endif
      }
    }
  }
  // Check if homing failed.  If so, request pause!
  if (!homingSuccess) {
    setXHomed(false);
    setYHomed(false);
    setZHomed(false);
    GCodeSource::printAllFLN(PSTR("RequestPause:Homing failed!"));
  } else {
    setXHomed(true);
    setYHomed(true);
    setZHomed(true);
  }
  // Correct different end stop heights
  // These can be adjusted by two methods. You can use offsets stored by determining the center
  // or you can use the xyzMinSteps from G100 calibration. Both have the same effect but only one
  // should be measured as both have the same effect.
  long dx = -xMinSteps - EEPROM::deltaTowerXOffsetSteps();
  long dy = -yMinSteps - EEPROM::deltaTowerYOffsetSteps();
  long dz = -zMinSteps - EEPROM::deltaTowerZOffsetSteps();
  long dm = RMath::min(dx, dy, dz);
  //Com::printFLN(PSTR("Tower 1:"),dx);
  //Com::printFLN(PSTR("Tower 2:"),dy);
  //Com::printFLN(PSTR("Tower 3:"),dz);
  dx -= dm; // now all dxyz are positive
  dy -= dm;
  dz -= dm;
  currentPositionSteps[X_AXIS] = 0; // here we should be
  currentPositionSteps[Y_AXIS] = 0;
  currentPositionSteps[Z_AXIS] = zMaxSteps;
  transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
  currentNonlinearPositionSteps[A_TOWER] -= dx;
  currentNonlinearPositionSteps[B_TOWER] -= dy;
  currentNonlinearPositionSteps[C_TOWER] -= dz;
  PrintLine::moveRelativeDistanceInSteps(0, 0, dm, 0, homingFeedrate[Z_AXIS], true, false);
  currentPositionSteps[X_AXIS] = 0; // now we are really here
  currentPositionSteps[Y_AXIS] = 0;
  currentPositionSteps[Z_AXIS] = zMaxSteps; // Extruder is now exactly in the delta center
  coordinateOffset[X_AXIS] = 0;
  coordinateOffset[Y_AXIS] = 0;
  coordinateOffset[Z_AXIS] = 0;
  transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);
  realDeltaPositionSteps[A_TOWER] = currentNonlinearPositionSteps[A_TOWER];
  realDeltaPositionSteps[B_TOWER] = currentNonlinearPositionSteps[B_TOWER];
  realDeltaPositionSteps[C_TOWER] = currentNonlinearPositionSteps[C_TOWER];
  //maxDeltaPositionSteps = currentDeltaPositionSteps[X_AXIS];
#if defined(ENDSTOP_Z_BACK_ON_HOME)
  if(ENDSTOP_Z_BACK_ON_HOME > 0)
    maxDeltaPositionSteps += axisStepsPerMM[Z_AXIS] * ENDSTOP_Z_BACK_ON_HOME;
#endif
  Extruder::selectExtruderById(Extruder::current->id);
#if FEATURE_BABYSTEPPING
  Printer::zBabysteps = 0;
#endif
}


// This home axis is for delta
void Printer::homeAxis(bool xaxis, bool yaxis, bool zaxis) { // Delta homing code
  bool nocheck = isNoDestinationCheck();
  setNoDestinationCheck(true);
  bool autoLevel = isAutolevelActive();
  setAutolevelActive(false);
  if (!(X_MAX_PIN > -1 && Y_MAX_PIN > -1 && Z_MAX_PIN > -1
        && MAX_HARDWARE_ENDSTOP_X && MAX_HARDWARE_ENDSTOP_Y && MAX_HARDWARE_ENDSTOP_Z)) {
    Com::printErrorFLN(PSTR("Hardware setup inconsistent. Delta cannot home without max endstops."));
  }
  // The delta has to have home capability to zero and set position,
  // so the redundant check is only an opportunity to
  // gratuitously fail due to incorrect settings.
  // The following movements would be meaningless unless it was zeroed for example.

  uid.setStatusP(PSTR("Homing..."));
  uid.refreshPage();

  // Homing Z axis means that you must home X and Y
  homeZAxis();
  moveToReal(0, 0, Printer::zLength, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]); // Move to designed coordinates including translation
  updateCurrentPosition(true);
  updateHomedAll();

  uid.clearStatus();

  Commands::printCurrentPosition();
  setAutolevelActive(autoLevel);
  Printer::updateCurrentPosition();
  setNoDestinationCheck(nocheck);
}





/** \brief Execute a open baby step.

    If zBabystepsMissing is not 0 this will do a z step in the desired direction. The old movement directions
    get restored after execution.
*/
void Printer::zBabystep() {
#if FEATURE_BABYSTEPPING
  bool dir = zBabystepsMissing > 0;
  if(dir) zBabystepsMissing--;
  else zBabystepsMissing++;
  Printer::enableXStepper();
  Printer::enableYStepper();
  Printer::enableZStepper();
  Printer::unsetAllSteppersDisabled();
  bool xDir = Printer::getXDirection();
  bool yDir = Printer::getYDirection();
  bool zDir = Printer::getZDirection();
  Printer::setXDirection(dir);
  Printer::setYDirection(dir);
  Printer::setZDirection(dir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
  HAL::delayMicroseconds(DIRECTION_DELAY);
#else
  HAL::delayMicroseconds(10);
#endif
  startXStep();
  startYStep();
  startZStep();
  HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 2);
  Printer::endXYZSteps();
  HAL::delayMicroseconds(10);
  Printer::setXDirection(xDir);
  Printer::setYDirection(yDir);
  Printer::setZDirection(zDir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
  HAL::delayMicroseconds(DIRECTION_DELAY);
#endif
  //HAL::delayMicroseconds(STEPPER_HIGH_DELAY + 1);
#endif
}


void Printer::handleInterruptEvent() {
  if(interruptEvent == 0) return;
  int event = interruptEvent;
  interruptEvent = 0;
  switch(event) {
  }
}

#define START_EXTRUDER_CONFIG(i)     Com::printF(PSTR("Config:"));Com::printF(PSTR("Extr."),i+1);Com::print(':');
void Printer::showConfiguration() {
  Com::config(PSTR("Baudrate:"), baudrate);
#ifndef EXTERNALSERIAL
  Com::config(PSTR("InputBuffer:"), SERIAL_BUFFER_SIZE - 1);
#endif
  Com::config(PSTR("NumExtruder:"), NUM_EXTRUDER);
  Com::config(PSTR("HeatedBed:"), 1);
  Com::config(PSTR("SDCard:"), 1);
  Com::config(PSTR("Fan:"), FAN_PIN > -1 && FEATURE_FAN_CONTROL);
#if FEATURE_FAN2_CONTROL && defined(FAN2_PIN) && FAN2_PIN > -1
  Com::config(PSTR("Fan2:1"));
#else
  Com::config(PSTR("Fan2:0"));
#endif
  Com::config(PSTR("LCD:"), 1);
  Com::config(PSTR("SoftwarePowerSwitch:"), PS_ON_PIN > -1);
  Com::config(PSTR("XHomeDir:"), X_HOME_DIR);
  Com::config(PSTR("YHomeDir:"), Y_HOME_DIR);
  Com::config(PSTR("ZHomeDir:"), Z_HOME_DIR);
  Com::config(PSTR("XHomePos:"), 0, 2);
  Com::config(PSTR("YHomePos:"), 0, 2);
  Com::config(PSTR("ZHomePos:"), zMin + zLength, 3);
  Com::config(PSTR("SupportG10G11:"), FEATURE_RETRACTION);
  Com::config(PSTR("SupportLocalFilamentchange:"), FEATURE_RETRACTION);
  Com::config(PSTR("ZProbe:"), FEATURE_Z_PROBE);
  Com::config(PSTR("Autolevel:"), FEATURE_AUTOLEVEL);
  Com::config(PSTR("PrintlineCache:"), PRINTLINE_CACHE_SIZE);
  Com::config(PSTR("JerkXY:"), maxJerk);
  Com::config(PSTR("KeepAliveInterval:"), KEEP_ALIVE_INTERVAL);
#if FEATURE_RETRACTION
  Com::config(PSTR("RetractionLength:"), EEPROM_FLOAT(RETRACTION_LENGTH));
  Com::config(PSTR("RetractionLongLength:"), EEPROM_FLOAT(RETRACTION_LONG_LENGTH));
  Com::config(PSTR("RetractionSpeed:"), EEPROM_FLOAT(RETRACTION_SPEED));
  Com::config(PSTR("RetractionZLift:"), EEPROM_FLOAT(RETRACTION_Z_LIFT));
  Com::config(PSTR("RetractionUndoExtraLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LENGTH));
  Com::config(PSTR("RetractionUndoExtraLongLength:"), EEPROM_FLOAT(RETRACTION_UNDO_EXTRA_LONG_LENGTH));
  Com::config(PSTR("RetractionUndoSpeed:"), EEPROM_FLOAT(RETRACTION_UNDO_SPEED));
#endif // FEATURE_RETRACTION
  Com::config(PSTR("XMin:"), xMin);
  Com::config(PSTR("YMin:"), yMin);
  Com::config(PSTR("ZMin:"), zMin);
  Com::config(PSTR("XMax:"), xMin + xLength);
  Com::config(PSTR("YMax:"), yMin + yLength);
  Com::config(PSTR("ZMax:"), zMin + zLength);
  Com::config(PSTR("XSize:"), xLength);
  Com::config(PSTR("YSize:"), yLength);
  Com::config(PSTR("ZSize:"), zLength);
  Com::config(PSTR("XPrintAccel:"), maxAccelerationMMPerSquareSecond[X_AXIS]);
  Com::config(PSTR("YPrintAccel:"), maxAccelerationMMPerSquareSecond[Y_AXIS]);
  Com::config(PSTR("ZPrintAccel:"), maxAccelerationMMPerSquareSecond[Z_AXIS]);
  Com::config(PSTR("XTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
  Com::config(PSTR("YTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
  Com::config(PSTR("ZTravelAccel:"), maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
  Com::config(PSTR("PrinterType:Delta"));
  Com::config(PSTR("MaxBedTemp:"), HEATED_BED_MAX_TEMP);
  for(fast8_t i = 0; i < NUM_EXTRUDER; i++) {
    START_EXTRUDER_CONFIG(i)
      Com::printFLN(PSTR("Jerk:"), extruder[i].maxStartFeedrate);
    START_EXTRUDER_CONFIG(i)
      Com::printFLN(PSTR("MaxSpeed:"), extruder[i].maxFeedrate);
    START_EXTRUDER_CONFIG(i)
      Com::printFLN(PSTR("Acceleration:"), extruder[i].maxAcceleration);
    START_EXTRUDER_CONFIG(i)
      Com::printFLN(PSTR("Diameter:"), extruder[i].diameter);
    START_EXTRUDER_CONFIG(i)
      Com::printFLN(PSTR("MaxTemp:"), MAXTEMP);
  }
}



void Printer::pausePrint() {
  if(Printer::isMenuMode(MODE_PRINTING)) {
    sd.pausePrint(true);
  } else
    if(Printer::isMenuMode(MODE_PRINTING)) {
      GCodeSource::printAllFLN(PSTR("RequestPause:"));
      Printer::setMenuMode(MODE_PAUSED, true);
#if !defined(DISABLE_PRINTMODE_ON_PAUSE) || DISABLE_PRINTMODE_ON_PAUSE==1
      Printer::setPrinting(false);
#endif
    }
}

void Printer::continuePrint() {
  if(Printer::isMenuMode(MODE_PRINTING | MODE_PAUSED)) {
    sd.continuePrint(true);
  } else
    if(Printer::isMenuMode(MODE_PAUSED)) {
      GCodeSource::printAllFLN(PSTR("RequestContinue:"));
    }
	setMenuMode(MODE_PAUSED, false);
}

void Printer::stopPrint() {
  //flashSource.close(); // stop flash printing if busy

  if(Printer::isMenuMode(MODE_PRINTING)) {
    sd.stopPrint();
  } else
    {
      GCodeSource::printAllFLN(PSTR("RequestStop:"));
    }

	//if(!isUIErrorMessage()) {
  //  uid.menuLevel=0;
  //  uid.refreshPage();
  //    }
}
