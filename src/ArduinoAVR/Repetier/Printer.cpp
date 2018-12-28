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
#include "HAL.h"
#include "gcode.h"
#include "Commands.h"
#include "motion.h"
#include "Printer.h"
#include "Extruder.h"

#include "temperatures.h"

#include "rmath.h"


uint8_t Printer::maxExtruderSpeed;            ///< Timer delay for end extruder speed
volatile int Printer::extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
//uint8_t Printer::extruderAccelerateDelay;     ///< delay between 2 speec increases

uint8_t Printer::unitIsInches = 0; ///< 0 = Units are mm, 1 = units are inches.

//  Number of steps per mm.
//
//  92.4 is a magic number of the EZStruder.
//
float Printer::axisStepsPerMM[E_AXIS_ARRAY] = { MICRO_STEPS * STEPS_PER_ROTATION / PULLEY_CIRCUMFERENCE,
                                                MICRO_STEPS * STEPS_PER_ROTATION / PULLEY_CIRCUMFERENCE,
                                                MICRO_STEPS * STEPS_PER_ROTATION / PULLEY_CIRCUMFERENCE,
                                                92.4 };

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
int8_t Printer::stepsPerTimerCall = 1;
uint16_t Printer::menuMode = 0;
uint8_t Printer::mode = DEFAULT_PRINTER_MODE;
uint8_t Printer::fanSpeed = 0; // Last fan speed set with M106/M107
float Printer::extrudeMultiplyError = 0;
float Printer::extrusionFactor = 1.0;

int Printer::currentLayer = 0;
int Printer::maxLayer = -1; // -1 = unknown
char Printer::printName[21] = ""; // max. 20 chars + 0
float Printer::progress = 0;
uint32_t Printer::lastTempReport = 0;

uint32_t Printer::interval = 30000;           ///< Last step duration in ticks.
uint32_t Printer::timer;              ///< used for acceleration/deceleration timing
uint32_t Printer::stepNumber;         ///< Step number in current move.

int32_t Printer::advanceExecuted;             ///< Executed advance steps
int Printer::advanceStepsSet;

int32_t Printer::maxDeltaPositionSteps;
int32_t Printer::deltaDiagonalStepsSquaredA;
int32_t Printer::deltaDiagonalStepsSquaredB;
int32_t Printer::deltaDiagonalStepsSquaredC;
float Printer::deltaMaxRadiusSquared;
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
uint16_t Printer::vMaxReached;               ///< Maximum reached speed
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

bool Printer::isPositionAllowed(float x, float y, float z) {
  if(isNoDestinationCheck()) return true;
  bool allowed = true;
  if(!isHoming()) {
    allowed = allowed && (z >= 0) && (z <= zLength + 0.05 + 5);
    allowed = allowed && (x * x + y * y <= deltaMaxRadiusSquared);
  }
  if(!allowed) {
    Printer::updateCurrentPosition(true);
    Commands::printCurrentPosition();
  }
  return allowed;
}



void Printer::updateDerivedParameter() {

  travelMovesPerSecond = DELTA_SEGMENTS_PER_SECOND_MOVE;   //EEPROM::deltaSegmentsPerSecondMove();
  printMovesPerSecond  = DELTA_SEGMENTS_PER_SECOND_PRINT;  //EEPROM::deltaSegmentsPerSecondPrint();

  if (travelMovesPerSecond < 15)
    travelMovesPerSecond = 15;

  if (printMovesPerSecond < 15)
    printMovesPerSecond = 15;

  axisStepsPerMM[X_AXIS] = axisStepsPerMM[Y_AXIS] = axisStepsPerMM[Z_AXIS];

  maxAccelerationMMPerSquareSecond[X_AXIS] = maxAccelerationMMPerSquareSecond[Y_AXIS] = maxAccelerationMMPerSquareSecond[Z_AXIS];

  homingFeedrate[X_AXIS] = homingFeedrate[Y_AXIS] = homingFeedrate[Z_AXIS];

  maxFeedrate[X_AXIS] = maxFeedrate[Y_AXIS] = maxFeedrate[Z_AXIS];

  maxTravelAccelerationMMPerSquareSecond[X_AXIS] = maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = maxTravelAccelerationMMPerSquareSecond[Z_AXIS];

  zMaxSteps = axisStepsPerMM[Z_AXIS] * (zLength);

  xMinSteps = axisStepsPerMM[A_TOWER] * xMin;
  yMinSteps = axisStepsPerMM[B_TOWER] * yMin;
  zMinSteps = axisStepsPerMM[C_TOWER] * zMin;

  //  A large machine, and we overflow uint32_t, if
  //    deltaDiagonalStepsSquaredA.l            > 65534 OR
  //    2 * ROD_RADIUS * axisStepsPerMM[Z_AXIS] > 65534
  //
  //  People have reported overflow with 300mm rods, 400mm towers, 400 steps/turn with 16 microsteps.
  //    https://github.com/repetier/Repetier-Firmware/issues/104
  //
  //  My machine has a ROD_RADIUS of 142.68
  //                   tower height  350
  //                   steps/turn    200
  //                   microsteps    16
  //
  //  This gives a steps per mm of MICRO_STEPS * STEPS_PER_ROT / PULLEY_CIRCUM = 16 * 200 / 40 = 80
  //

  //ROD_RADIUS = EEPROM::deltaHorizontalRadius();

  float radiusA = ROD_RADIUS + 0;  //EEPROM::deltaRadiusCorrectionA();
  float radiusB = ROD_RADIUS + 0;  //EEPROM::deltaRadiusCorrectionB();
  float radiusC = ROD_RADIUS + 0;  //EEPROM::deltaRadiusCorrectionC();

  deltaAPosXSteps = floor(radiusA * cos(DELTA_ALPHA_A * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaAPosYSteps = floor(radiusA * sin(DELTA_ALPHA_A * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);

  deltaBPosXSteps = floor(radiusB * cos(DELTA_ALPHA_B * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaBPosYSteps = floor(radiusB * sin(DELTA_ALPHA_B * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);

  deltaCPosXSteps = floor(radiusC * cos(DELTA_ALPHA_C * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);
  deltaCPosYSteps = floor(radiusC * sin(DELTA_ALPHA_C * M_PI / 180.0f) * axisStepsPerMM[Z_AXIS] + 0.5f);

  float  diagRod = DELTA_DIAGONAL_ROD;  //EEPROM::deltaDiagonalRodLength()

  float  corrA = 0;  //EEPROM::deltaDiagonalCorrectionA()
  float  corrB = 0;  //EEPROM::deltaDiagonalCorrectionB()
  float  corrC = 0;  //EEPROM::deltaDiagonalCorrectionC()

  deltaDiagonalStepsSquaredA = (uint32_t)RMath::sqr((diagRod + corrA) * axisStepsPerMM[Z_AXIS]);
  deltaDiagonalStepsSquaredB = (uint32_t)RMath::sqr((diagRod + corrB) * axisStepsPerMM[Z_AXIS]);
  deltaDiagonalStepsSquaredC = (uint32_t)RMath::sqr((diagRod + corrC) * axisStepsPerMM[Z_AXIS]);

  deltaMaxRadiusSquared = DELTA_MAX_RADIUS * DELTA_MAX_RADIUS;



  long cart[Z_AXIS_ARRAY];
  long delta[TOWER_ARRAY];

  cart[X_AXIS] = 0;
  cart[Y_AXIS] = 0;
  cart[Z_AXIS] = zMaxSteps;

  transformCartesianStepsToDeltaSteps(cart, delta);

  maxDeltaPositionSteps = delta[0];

  xMaxSteps = yMaxSteps = zMaxSteps;
  xMinSteps = yMinSteps = zMinSteps = 0;

  deltaFloorSafetyMarginSteps = DELTA_FLOOR_SAFETY_MARGIN_MM * axisStepsPerMM[Z_AXIS];

  for (uint8_t i=0; i<4; i++) {
    invAxisStepsPerMM[i] = 1.0f / axisStepsPerMM[i];

    maxPrintAccelerationStepsPerSquareSecond[i]  = maxAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
    maxTravelAccelerationStepsPerSquareSecond[i] = maxTravelAccelerationMMPerSquareSecond[i] * axisStepsPerMM[i];
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
    Com::printF(PSTR("XY jerk was too low, setting to "));
    Com::print(maxJerk);
    Com::printF(PSTR("\n"));
  }
  accel = RMath::max(maxAccelerationMMPerSquareSecond[Z_AXIS], maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);

  Printer::updateAdvanceFlags();
}



void Printer::updateAdvanceFlags() {
  Printer::setAdvanceActivated(false);

  if(extruder.advanceL != 0)
    Printer::setAdvanceActivated(true);

  if(extruder.advanceK != 0)
    Printer::setAdvanceActivated(true);
}

// This is for untransformed move to coordinates in printers absolute Cartesian space
#if 0
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
  if (!PrintLine::queueNonlinearMove(true, true, false)) {
    Com::printF(PSTR("WARNING: moveTo / queueDeltaMove returns error\n"));
    return 0;
  }
  updateCurrentPosition(false);
  return 1;
}
#endif

uint8_t Printer::moveToReal(float x, float y, float z, float e, float f, bool pathOptimize) {

  Com::printf(PSTR("moveToReal()-- %f %f %f extruder %f feedrate %f\n"), x, y, z, e, f);

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
  destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];

#ifndef DONT_EXTRUDE
  if(e != IGNORE_COORDINATE)
    destinationSteps[E_AXIS] = e * axisStepsPerMM[E_AXIS];
#endif

  if(f != IGNORE_COORDINATE)
    feedrate = f;

  if (!PrintLine::queueNonlinearMove(true, pathOptimize, true)) {
    Com::printF(PSTR("WARNING: moveToReal / queueDeltaMove returns error\n"));

#ifdef DEBUG
    Com::printF(PSTR(" x=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printF(PSTR(" mm\n"));
    Com::printF(PSTR(" y=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printF(PSTR(" mm\n"));
    Com::printF(PSTR(" z=")); Com::print((long)x*80); Com::print(" steps  "); Com::print(x); Com::printF(PSTR(" mm\n"));
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
  currentPosition[X_AXIS] -= Printer::offsetX; // Offset from active extruder
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

uint8_t Printer::setDestinationStepsFromGCode(gcodeCommand *com) {
  register int32_t p;
  float x, y, z;
  bool posAllowed = true;
  if(!com->hasNoXYZ()) {
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
  }

  if(com->hasE()) {
    Com::printf(PSTR("setDestinationStepsFromGCode()-- E %f\n"), com->E);

    p = convertToMM(com->E * axisStepsPerMM[E_AXIS]);

    if(relativeCoordinateMode || relativeExtruderCoordinateMode) {
      Com::printf(PSTR("RELATIVE E\n"));

      if(fabs(com->E) * extrusionFactor > EXTRUDE_MAXLENGTH)
        p = 0;
#ifdef DONT_EXTRUDE
      p = 0;
#endif
      destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS] + p;

      Com::printf(PSTR("setDestinationStepsFromGCode()-- new destination %ld (with offset %ld)\n"), destinationSteps[E_AXIS], p);
    } else {
      Com::printf(PSTR("ABSOLUTE E\n"));

      if(fabs(p - currentPositionSteps[E_AXIS]) * extrusionFactor > EXTRUDE_MAXLENGTH * axisStepsPerMM[E_AXIS])
        currentPositionSteps[E_AXIS] = p;
#ifdef DONT_EXTRUDE
      currentPositionSteps[E_AXIS] = p;
#endif
      destinationSteps[E_AXIS] = p;
    }
  }

  //  No E specified, just a move.
  else {
    destinationSteps[E_AXIS] = currentPositionSteps[E_AXIS];
  }

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



void
Printer::setup() {

  Com::printf(PSTR("Printer Initializing.\n"));

  SET_OUTPUT(PS_ON_PIN); //GND
  WRITE(PS_ON_PIN, LOW);
  Printer::setPowerOn(true);

  SET_INPUT(SDCARDDETECT);
  PULLUP(SDCARDDETECT, HIGH);

  //
  //  Set motor current.
  //
  //  135 / 255 ~= 0.75A
  //  185 / 255 ~= 1.00A
  //

  SPI.begin();

  //  Omitting SPI.begin() and using just this results in a lockup here.
  //SPCR |= _BV(SPE) | _BV(MSTR);     //  Enable SPI and set us as the master.

  SET_OUTPUT(MOSI_PIN);         //  Master Out Slave In
  SET_INPUT (MISO_PIN);         //  Master In Slave Out
  SET_OUTPUT(SCK);              //  Serial Clock
  SET_OUTPUT(DIGIPOTSS_PIN);    //  Slave Select

  WRITE(DIGIPOTSS_PIN, HIGH);   //  Trun off slave selection until we want to talk.

  SPI.beginTransaction(SD_SCK_HZ(250000));

  WRITE(DIGIPOTSS_PIN, LOW);    //  Select the digipot.

  SPI.transfer(DIGIPOT_X_CH);    SPI.transfer(33);    //  140 / 255 is the standard for XYZ.
  SPI.transfer(DIGIPOT_Y_CH);    SPI.transfer(33);
  SPI.transfer(DIGIPOT_Z_CH);    SPI.transfer(33);
  SPI.transfer(DIGIPOT_E0_CH);   SPI.transfer(33);    //  130 / 255 is the standard for extruders.
  SPI.transfer(DIGIPOT_E1_CH);   SPI.transfer(0);

  WRITE(DIGIPOTSS_PIN, HIGH);   //  Hangup.

  SPI.endTransaction();

  //SPCR = 0;
  //SPI.end();

  //
  //  Set up TOWER stepper motors.
  //

  SET_OUTPUT(X_STEP_PIN);     SET_OUTPUT(Y_STEP_PIN);     SET_OUTPUT(Z_STEP_PIN);
  SET_OUTPUT(X_DIR_PIN);      SET_OUTPUT(Y_DIR_PIN);      SET_OUTPUT(Z_DIR_PIN);
  SET_OUTPUT(X_ENABLE_PIN);   SET_OUTPUT(Y_ENABLE_PIN);   SET_OUTPUT(Z_ENABLE_PIN);

  WRITE(X_ENABLE_PIN, !X_ENABLE_ON);          //  Disable steppers.
  WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
  WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);

  WRITE(X_DIR_PIN, !INVERT_X_DIR);            //  Set direction to forward.
  WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
  WRITE(Z_DIR_PIN, !INVERT_Z_DIR);

  WRITE(X_STEP_PIN, !START_STEP_WITH_HIGH);   //  Reset the stepping.
  WRITE(Y_STEP_PIN, !START_STEP_WITH_HIGH);
  WRITE(Z_STEP_PIN, !START_STEP_WITH_HIGH);





  microstepInit();


  feedrate = 50; ///< Current feedrate in mm/s.
  feedrateMultiply = 100;
  extrudeMultiply = 100;
  lastCmdPos[X_AXIS] = lastCmdPos[Y_AXIS] = lastCmdPos[Z_AXIS] = 0;

  advanceExecuted = 0;
  advanceStepsSet = 0;

  maxJerk = MAX_JERK;
  offsetX = offsetY = offsetZ = 0;
  interval = 5000;
  stepsPerTimerCall = 1;
  msecondsPrinting = 0;
  filamentPrinted = 0;

  xLength = X_MAX_LENGTH;
  yLength = Y_MAX_LENGTH;
  zLength = Z_MAX_LENGTH;

  xMin = X_MIN_POS;
  yMin = Y_MIN_POS;
  zMin = Z_MIN_POS;

  extruderStepsNeeded = 0;

  for(uint8_t i = 0; i < E_AXIS_ARRAY; i++) {
    currentPositionSteps[i] = 0;
  }
  currentPosition[X_AXIS] = 0.0;
  currentPosition[Y_AXIS] = 0.0;
  currentPosition[Z_AXIS] = 0.0;

  updateDerivedParameter();

  transformCartesianStepsToDeltaSteps(Printer::currentPositionSteps, Printer::currentNonlinearPositionSteps);

#if DELTA_HOME_ON_POWER
  homeAxis(true, true, true);
#endif

  Commands::printCurrentPosition();

  selectExtruderById(0);

  //  You can now send some initialization gcode to the printer.
  //commandQueue.executeFString(PSTR(STARTUP_GCODE));

  Com::printf(PSTR("Printer Initialized.\n"));
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


void
Printer::deltaMoveToTopEndstops(float feedrate) {

  Printer::currentPositionSteps[0] = 0;
  Printer::currentPositionSteps[1] = 0;
  Printer::currentPositionSteps[2] = 0;

  Printer::stepsRemainingAtXHit = -1;
  Printer::stepsRemainingAtYHit = -1;
  Printer::stepsRemainingAtZHit = -1;

  setHoming(true);

  transformCartesianStepsToDeltaSteps(currentPositionSteps, currentNonlinearPositionSteps);

  PrintLine::moveRelativeDistanceInSteps(0,
                                         0,
                                         (zMaxSteps + DELTA_DIAGONAL_ROD * axisStepsPerMM[Z_AXIS]) * 1.5,  //EEPROM::deltaDiagonalRodLength()
                                         0,
                                         feedrate,
                                         true,
                                         true);

  offsetX = offsetY = offsetZ = offsetZ2 = 0;

  setHoming(false);
}



void
Printer::homeZAxis(void) {
  bool homingSuccess = false;

  endstops.resetAccumulator();

  deltaMoveToTopEndstops(homingFeedrate[Z_AXIS]);

  endstops.fillFromAccumulator();


  // New safe homing routine by Kyrre Aalerud
  // This method will safeguard against sticky endstops such as may be gotten cheaply from china.
  // This can lead to head crashes and even fire, thus a safer algorithm to ensure the endstops actually respond as expected.
  //endstops.report();
  // Check that all endstops (XYZ) were hit

  if (endstops.xMax() && endstops.yMax() && endstops.zMax()) {
    // Back off for retest
    PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -10, 0, Printer::homingFeedrate[Z_AXIS] / 4, true, true);
    //endstops.report();
    // Check for proper release of all (XYZ) endstops
    if (!(endstops.xMax() || endstops.yMax() || endstops.zMax())) {

      // Rehome with reduced speed
      endstops.resetAccumulator();
      deltaMoveToTopEndstops(Printer::homingFeedrate[Z_AXIS] / 4);
      endstops.fillFromAccumulator();
      //endstops.report();
      // Check that all endstops (XYZ) were hit again
      if (endstops.xMax() && endstops.yMax() && endstops.zMax()) {
        homingSuccess = true; // Assume success in case there is no back move

        PrintLine::moveRelativeDistanceInSteps(0, 0, axisStepsPerMM[Z_AXIS] * -5, 0, homingFeedrate[Z_AXIS], true, true);
        //endstops.report();
        // Check for missing release of any (XYZ) endstop
        if (endstops.xMax() || endstops.yMax() || endstops.zMax()) {
          homingSuccess = false; // Reset success flag
        }

      }
    }
  }

  // Check if homing failed.  If so, request pause!

  if (homingSuccess == false)
    Com::printf(PSTR("RequestPause:Homing failed!\n"));


  setHomed(homingSuccess);


  // Correct different end stop heights
  // These can be adjusted by two methods. You can use offsets stored by determining the center
  // or you can use the xyzMinSteps from G100 calibration. Both have the same effect but only one
  // should be measured as both have the same effect.

  long dx = -xMinSteps - 0;  //EEPROM::deltaTowerXOffsetSteps();
  long dy = -yMinSteps - 0;  //EEPROM::deltaTowerYOffsetSteps();
  long dz = -zMinSteps - 0;  //EEPROM::deltaTowerZOffsetSteps();

  long dm = RMath::min(dx, dy, dz);

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
  maxDeltaPositionSteps += axisStepsPerMM[Z_AXIS] * 5;

  selectExtruderById(extruder.id);

#if FEATURE_BABYSTEPPING
  Printer::zBabysteps = 0;
#endif
}



void
Printer::homeTowers(void) {

  bool nocheck = isNoDestinationCheck();
  setNoDestinationCheck(true);

  homeZAxis();

  moveToReal(0, 0, Printer::zLength, IGNORE_COORDINATE, homingFeedrate[Z_AXIS]); // Move to designed coordinates including translation
  updateCurrentPosition(true);

  Commands::printCurrentPosition();
  Printer::updateCurrentPosition();

  setNoDestinationCheck(nocheck);
}



void
Printer::disableSteppers(void) {
  disableXStepper();
  disableYStepper();
  disableZStepper();

  extruder.disable();

  setHomed(false);
};



void
Printer::disableHeaters(void) {
  extruderTemp.setTargetTemperature(0);
  bedTemp.setTargetTemperature(0);
};



void
Printer::disablePower(void) {
  SET_OUTPUT(PS_ON_PIN);
  WRITE(PS_ON_PIN, HIGH);
};





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

  bool xDir = Printer::getXDirection();
  bool yDir = Printer::getYDirection();
  bool zDir = Printer::getZDirection();
  Printer::setXDirection(dir);
  Printer::setYDirection(dir);
  Printer::setZDirection(dir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
  delayMicroseconds(DIRECTION_DELAY);
#else
  delayMicroseconds(10);
#endif
  startXStep();
  startYStep();
  startZStep();
  delayMicroseconds(STEPPER_HIGH_DELAY + 2);
  Printer::endXYZSteps();
  delayMicroseconds(10);
  Printer::setXDirection(xDir);
  Printer::setYDirection(yDir);
  Printer::setZDirection(zDir);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
  delayMicroseconds(DIRECTION_DELAY);
#endif
  //delayMicroseconds(STEPPER_HIGH_DELAY + 1);
#endif
}





/** \brief Select extruder ext_num.

    This function changes and initializes a new extruder. This is also called, after the eeprom values are changed.
*/
void Printer::selectExtruderById(uint8_t extruderId) {
  return;

#if 0

  //  Formerly in Extruder, but doesn't belong there.

  float cx, cy, cz;
  Printer::realPosition(cx, cy, cz);

  Commands::waitUntilEndOfAllMoves();

  if (extruderId > 0)
    extruderId = 0;

  Extruder *current = extruder->current;
  Extruder *next = &extruder[extruderId];

  float oldfeedrate = Printer::feedrate;

  current->extrudePosition = Printer::currentPositionSteps[E_AXIS];

  if(Printer::isHomedAll() && next->zOffset < current->zOffset) { // prevent extruder from hitting bed - move bed down a bit
    Printer::offsetZ = -next->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
    Printer::setNoDestinationCheck(true);
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE, Printer::homingFeedrate[Z_AXIS]);
    Printer::setNoDestinationCheck(false);
    Commands::waitUntilEndOfAllMoves();
    Printer::updateCurrentPosition(true);
  }


  Extruder::current = next;

  // --------------------- Now new extruder is active --------------------

#ifdef SEPERATE_EXTRUDER_POSITIONS
  // Use separate extruder positions only if being told. Slic3r e.g. creates a continuous extruder position increment
  Printer::currentPositionSteps[E_AXIS] = extruder.extrudePosition;
#endif

  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS];
  Printer::axisStepsPerMM[E_AXIS] = extruder.stepsPerMM;
  Printer::invAxisStepsPerMM[E_AXIS] = 1.0f / Printer::axisStepsPerMM[E_AXIS];
  Printer::maxFeedrate[E_AXIS] = extruder.maxFeedrate;
  //   max_start_speed_units_per_second[E_AXIS] = extruder.maxStartFeedrate;
  Printer::maxAccelerationMMPerSquareSecond[E_AXIS] = Printer::maxTravelAccelerationMMPerSquareSecond[E_AXIS] = next->maxAcceleration;
  Printer::maxTravelAccelerationStepsPerSquareSecond[E_AXIS] =
    Printer::maxPrintAccelerationStepsPerSquareSecond[E_AXIS] = Printer::maxAccelerationMMPerSquareSecond[E_AXIS] * Printer::axisStepsPerMM[E_AXIS];
  Printer::maxExtruderSpeed = (uint8_t)floor(hal.maxExtruderTimerFrequency() / (extruder.maxFeedrate * next->stepsPerMM));
  if(Printer::maxExtruderSpeed > 15) Printer::maxExtruderSpeed = 15;
  float fmax = ((float)hal.maxExtruderTimerFrequency() / ((float)Printer::maxExtruderSpeed * Printer::axisStepsPerMM[E_AXIS])); // Limit feedrate to interrupt speed
  if(fmax < Printer::maxFeedrate[E_AXIS]) Printer::maxFeedrate[E_AXIS] = fmax;

  extruderTemp.updateTempControlVars();

  Printer::offsetX = -next->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
  Printer::offsetY = -next->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
  Printer::offsetZ = -next->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
  Commands::changeFlowrateMultiply(Printer::extrudeMultiply); // needed to adjust extrusionFactor to possibly different diameter

  resetExtruderDirection();

	if(Printer::isHomedAll()) {
		Printer::moveToReal(cx, cy, cz, IGNORE_COORDINATE, EXTRUDER_SWITCH_XY_SPEED);
	}

	Printer::feedrate = oldfeedrate;
	Printer::updateCurrentPosition(true);
#endif  //  DISABLED
}


