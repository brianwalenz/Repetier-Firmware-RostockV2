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

/**

   Coordinate system transformations:

   Level 1: G-code => Coordinates like send via g-codes.

   Level 2: Real coordinates => Coordinates corrected by coordinate shift via G92
   currentPosition and lastCmdPos are from this level.
   Level 3: Transformed and shifter => Include extruder offset and bed rotation.
   These variables are only stored temporary.

   Level 4: Step position => Level 3 converted into steps for motor position
   currentPositionSteps and destinationPositionSteps are from this level.

   Level 5: Nonlinear motor step position, only for nonlinear drive systems
   destinationDeltaSteps


*/

#ifndef PRINTER_H_INCLUDED
#define PRINTER_H_INCLUDED

#include "ui.h"   //  For MODE_*, at least.
#include "Communication.h"
#include "gcode.h"

union floatLong {
  float f;
  uint32_t l;
};

#define PRINTER_FLAG0_STEPPER_DISABLED      1
#define PRINTER_FLAG0_SEPERATE_EXTRUDER_INT 2
#define PRINTER_FLAG0_TEMPSENSOR_DEFECT     4
#define PRINTER_FLAG0_FORCE_CHECKSUM        8
#define PRINTER_FLAG0_MANUAL_MOVE_MODE      16
#define PRINTER_FLAG0_AUTOLEVEL_ACTIVE      32
#define PRINTER_FLAG0_ZPROBEING             64
#define PRINTER_FLAG0_LARGE_MACHINE         128
#define PRINTER_FLAG1_HOMED_ALL             1
#define PRINTER_FLAG1_ALLKILLED             8
#define PRINTER_FLAG1_UI_ERROR_MESSAGE      16
#define PRINTER_FLAG1_NO_DESTINATION_CHECK  32
#define PRINTER_FLAG1_POWER_ON              64
#define PRINTER_FLAG1_ALLOW_COLD_EXTRUSION  128

#define PRINTER_FLAG2_BLOCK_RECEIVING       1
#define PRINTER_FLAG2_RESET_FILAMENT_USAGE  4
#define PRINTER_FLAG2_IGNORE_M106_COMMAND   8
#define PRINTER_FLAG2_HOMING                64
#define PRINTER_FLAG2_ALL_E_MOTORS          128 // Set all e motors flag
#define PRINTER_FLAG3_X_HOMED               1
#define PRINTER_FLAG3_Y_HOMED               2
#define PRINTER_FLAG3_Z_HOMED               4
#define PRINTER_FLAG3_PRINTING              8 // set explicitly with M530
#define PRINTER_FLAG3_AUTOREPORT_TEMP       16
#define PRINTER_FLAG3_SUPPORTS_STARTSTOP    32

// define an integer number of steps more than large enough to get to end stop from anywhere
#define HOME_DISTANCE_STEPS (Printer::zMaxSteps-Printer::zMinSteps+1000)
#define HOME_DISTANCE_MM (HOME_DISTANCE_STEPS * invAxisStepsPerMM[Z_AXIS])

// Some defines to make clearer reading, as we overload these Cartesian memory locations for delta
#define towerAMaxSteps Printer::xMaxSteps
#define towerBMaxSteps Printer::yMaxSteps
#define towerCMaxSteps Printer::zMaxSteps
#define towerAMinSteps Printer::xMinSteps
#define towerBMinSteps Printer::yMinSteps
#define towerCMinSteps Printer::zMinSteps

class Plane {
public:
  // f(x, y) = ax + by + c
  float a, b, c;
  float z(float x, float y) {
    return a * x + y * b + c;
  }
};

#include "Distortion.h"

#include "Endstops.h"

extern bool runBedLeveling(int save); // save = S parameter in gcode

/**
   The Printer class is the main class for the control of the 3d printer. Here all
   movement related key variables are stored like positions, accelerations.

   ## Coordinates

   The firmware works with 4 different coordinate systems and understanding the
   dependencies between them is crucial to a good understanding on how positions
   are handled.

   ### Real world floating coordinates (RWC)

   These coordinates are the real floating positions with any offsets subtracted,
   which might be set with G92. This is used to show coordinates or for computations
   based on real positions. Any correction coming from rotation or distortion is
   not included in these coordinates. currentPosition and lastCmdPos use this coordinate
   system.

   When these coordinates need to be used for computation, the value of offsetX, offsetY and offsetZ
   is always added. These are the offsets of the currently active tool to virtual tool center
   (normally first extruder).

   ### Rotated floating coordinates (ROTC)

   If auto leveling is active, printing to the official coordinates is not possible. We have to assume
   that the bed is somehow rotated against the Cartesian mechanics from the printer. Applying
   _transformToPrinter_ to the real world coordinates, rotates them around the origin to
   be equal to the rotated bed. _transformFromPrinter_ would apply the opposite transformation.

   ### Cartesian motor position coordinates (CMC)

   The position of motors is stored as steps from 0. The reason for this is that it is crucial that
   no rounding errors ever cause addition of any steps. These positions are simply computed by
   multiplying the ROTC coordinates with the axisStepsPerMM.

   If distortion correction is enabled, there is an additional factor for the z position that
   gets added: _zCorrectionStepsIncluded_ This value is recalculated by every move added to
   reflect the distortion at any given xyz position.

   ### Nonlinear motor position coordinates (NMC)

   In case of a nonlinear mechanic like a delta printer, the CMC does not define the motor positions.
   An additional transformation converts the CMC coordinates into NMC.

   ### Transformations from RWC to CMC

   Given:
   - Target position for tool: x_rwc, y_rwc, z_rwc
   - Tool offsets: offsetX, offsetY, offsetZ
   - Offset from bed leveling: offsetZ2

   Step 1: Convert to ROTC

   transformToPrinter(x_rwc + Printer::offsetX, y_rwc + Printer::offsetY, z_rwc +  Printer::offsetZ, x_rotc, y_rotc, z_rotc);
   z_rotc += offsetZ2

   Step 2: Compute CMC

   x_cmc = static_cast<int32_t>(floor(x_rotc * axisStepsPerMM[X_AXIS] + 0.5f));
   y_cmc = static_cast<int32_t>(floor(y_rotc * axisStepsPerMM[Y_AXIS] + 0.5f));
   z_cmc = static_cast<int32_t>(floor(z_rotc * axisStepsPerMM[Z_AXIS] + 0.5f));

   ### Transformation from CMC to RWC

   Note: _zCorrectionStepsIncluded_ comes from distortion correction and gets set when a move is queued by the queuing function.
   Therefore it is not visible in the inverse transformation above. When transforming back, consider if the value was set or not!

   Step 1: Convert to ROTC

   x_rotc = static_cast<float>(x_cmc) * invAxisStepsPerMM[X_AXIS];
   y_rotc = static_cast<float>(y_cmc) * invAxisStepsPerMM[Y_AXIS];
   z_rotc = static_cast<float>(z_cmc * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
   z_rotc = static_cast<float>(z_cmc - zCorrectionStepsIncluded) * invAxisStepsPerMM[Z_AXIS] - offsetZ2;
   #endif

   Step 2: Convert to RWC

   transformFromPrinter(x_rotc, y_rotc, z_rotc,x_rwc, y_rwc, z_rwc);
   x_rwc -= Printer::offsetX; // Offset from active extruder or z probe
   y_rwc -= Printer::offsetY;
   z_rwc -= Printer::offsetZ;
*/

class Printer {
public:

  static volatile int extruderStepsNeeded; ///< This many extruder steps are still needed, <0 = reverse steps needed.
  static uint8_t maxExtruderSpeed;            ///< Timer delay for end extruder speed
  //static uint8_t extruderAccelerateDelay;     ///< delay between 2 speec increases
  static int advanceStepsSet;
  static long advanceExecuted;             ///< Executed advance steps

  static uint16_t menuMode;
  static float axisStepsPerMM[]; ///< Resolution of each axis in steps per mm.
  static float invAxisStepsPerMM[]; ///< 1/axisStepsPerMM for faster computation.
  static float maxFeedrate[]; ///< Maximum feedrate in mm/s per axis.
  static float homingFeedrate[]; // Feedrate in mm/s for homing.
  // static uint32_t maxInterval; // slowest allowed interval
  static float maxAccelerationMMPerSquareSecond[];
  static float maxTravelAccelerationMMPerSquareSecond[];
  static unsigned long maxPrintAccelerationStepsPerSquareSecond[];
  static unsigned long maxTravelAccelerationStepsPerSquareSecond[];
  static uint8_t relativeCoordinateMode;    ///< Determines absolute (false) or relative Coordinates (true).
  static uint8_t relativeExtruderCoordinateMode;  ///< Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.

  static uint8_t unitIsInches;
  static uint8_t mode;
  static uint8_t fanSpeed; // Last fan speed set with M106/M107
  static int8_t stepsPerTimerCall;
  static uint8_t flag0, flag1; // 1 = stepper disabled, 2 = use external extruder interrupt, 4 = temp Sensor defect, 8 = homed
  static uint8_t flag2, flag3;
  static uint32_t interval;    ///< Last step duration in ticks.
  static uint32_t timer;              ///< used for acceleration/deceleration timing
  static uint32_t stepNumber;         ///< Step number in current move.
  static float coordinateOffset[Z_AXIS_ARRAY];
  static int32_t currentPositionSteps[E_AXIS_ARRAY];     ///< Position in steps from origin.
  static float currentPosition[Z_AXIS_ARRAY]; ///< Position in global coordinates
  static float lastCmdPos[Z_AXIS_ARRAY]; ///< Last coordinates (global coordinates) send by g-codes
  static int32_t destinationSteps[E_AXIS_ARRAY];         ///< Target position in steps.
  static uint32_t lastTempReport;
  static float extrudeMultiplyError; ///< Accumulated error during extrusion
  static float extrusionFactor; ///< Extrusion multiply factor

  static int32_t maxDeltaPositionSteps;
  static int32_t currentNonlinearPositionSteps[E_TOWER_ARRAY];
  static floatLong deltaDiagonalStepsSquaredA;
  static floatLong deltaDiagonalStepsSquaredB;
  static floatLong deltaDiagonalStepsSquaredC;
  static float deltaMaxRadiusSquared;
  static int32_t deltaFloorSafetyMarginSteps;
  static int32_t deltaAPosXSteps;
  static int32_t deltaAPosYSteps;
  static int32_t deltaBPosXSteps;
  static int32_t deltaBPosYSteps;
  static int32_t deltaCPosXSteps;
  static int32_t deltaCPosYSteps;
  static int32_t realDeltaPositionSteps[TOWER_ARRAY];
  static int16_t travelMovesPerSecond;
  static int16_t printMovesPerSecond;
  static float radius0;

  static int32_t stepsRemainingAtZHit;
  static int32_t stepsRemainingAtXHit;
  static int32_t stepsRemainingAtYHit;
#if FEATURE_SOFTWARE_LEVELING
  static int32_t levelingP1[3];
  static int32_t levelingP2[3];
  static int32_t levelingP3[3];
#endif
#if FEATURE_AUTOLEVEL
  static float autolevelTransformation[9]; ///< Transformation matrix
#endif
#if FEATURE_BABYSTEPPING
  static int16_t zBabystepsMissing;
  static int16_t zBabysteps;
#endif
  //static float minimumSpeed;               ///< lowest allowed speed to keep integration error small
  //static float minimumZSpeed;              ///< lowest allowed speed to keep integration error small

  static int32_t xMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  static int32_t yMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  static int32_t zMaxSteps;                   ///< For software endstops, limit of move in positive direction.
  static int32_t xMinSteps;                   ///< For software endstops, limit of move in negative direction.
  static int32_t yMinSteps;                   ///< For software endstops, limit of move in negative direction.
  static int32_t zMinSteps;                   ///< For software endstops, limit of move in negative direction.

  static float xLength;
  static float xMin;
  static float yLength;
  static float yMin;
  static float zLength;
  static float zMin;
  static float feedrate;                   ///< Last requested feedrate.
  static int feedrateMultiply;             ///< Multiplier for feedrate in percent (factor 1 = 100)
  static unsigned int extrudeMultiply;     ///< Flow multiplier in percent (factor 1 = 100)
  static float maxJerk;                    ///< Maximum allowed jerk in mm/s

  static float offsetX;                     ///< X-offset for different tool positions.
  static float offsetY;                     ///< Y-offset for different tool positions.
  static float offsetZ;                     ///< Z-offset for different tool positions.
  static float offsetZ2;                    ///< Z-offset without rotation correction. Required for z probe corrections
  static uint16_t vMaxReached;               ///< Maximum reached speed
  static uint32_t msecondsPrinting;         ///< Milliseconds of printing time (means time with heated extruder)
  static float filamentPrinted;             ///< mm of filament printed since counting started
  static float memoryX;
  static float memoryY;
  static float memoryZ;
  static float memoryE;
  static float memoryF;
#ifdef DEBUG_SEGMENT_LENGTH
  static float maxRealSegmentLength;
#endif
#ifdef DEBUG_REAL_JERK
  static float maxRealJerk;
#endif
  // Print status related
  static int currentLayer;
  static int maxLayer; // -1 = unknown
  static char printName[21]; // max. 20 chars + 0
  static float progress;

  static INLINE void setMenuMode(uint16_t mode, bool on) {
    if(on)
      menuMode |= mode;
    else
      menuMode &= ~mode;
  }

  static INLINE bool isMenuMode(uint8_t mode) {
    return (menuMode & mode) == mode;
  }

  /** \brief Disable stepper motor for x direction. */
  static INLINE void disableXStepper() {
#if (X_ENABLE_PIN > -1)
    WRITE(X_ENABLE_PIN, !X_ENABLE_ON);
#endif
  }

  /** \brief Disable stepper motor for y direction. */
  static INLINE void disableYStepper() {
#if (Y_ENABLE_PIN > -1)
    WRITE(Y_ENABLE_PIN, !Y_ENABLE_ON);
#endif
  }
  /** \brief Disable stepper motor for z direction. */
  static INLINE void disableZStepper() {
#if (Z_ENABLE_PIN > -1)
    WRITE(Z_ENABLE_PIN, !Z_ENABLE_ON);
#endif
  }

  /** \brief Enable stepper motor for x direction. */
  static INLINE void  enableXStepper() {
#if (X_ENABLE_PIN > -1)
    WRITE(X_ENABLE_PIN, X_ENABLE_ON);
#endif
  }

  /** \brief Enable stepper motor for y direction. */
  static INLINE void  enableYStepper() {
#if (Y_ENABLE_PIN > -1)
    WRITE(Y_ENABLE_PIN, Y_ENABLE_ON);
#endif
  }
  /** \brief Enable stepper motor for z direction. */
  static INLINE void  enableZStepper() {
#if (Z_ENABLE_PIN > -1)
    WRITE(Z_ENABLE_PIN, Z_ENABLE_ON);
#endif
  }

  static INLINE void setXDirection(bool positive) {
    if(positive) {
      WRITE(X_DIR_PIN, !INVERT_X_DIR);
    } else {
      WRITE(X_DIR_PIN, INVERT_X_DIR);
    }
  }

  static INLINE void setYDirection(bool positive) {
    if(positive) {
      WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
    } else {
      WRITE(Y_DIR_PIN, INVERT_Y_DIR);
    }
  }
  static INLINE void setZDirection(bool positive) {
    if(positive) {
      WRITE(Z_DIR_PIN, !INVERT_Z_DIR);
    } else {
      WRITE(Z_DIR_PIN, INVERT_Z_DIR);
    }
  }

  static INLINE bool getZDirection() {
    return ((READ(Z_DIR_PIN) != 0) ^ INVERT_Z_DIR);
  }

  static INLINE bool getYDirection() {
    return((READ(Y_DIR_PIN) != 0) ^ INVERT_Y_DIR);
  }

  static INLINE bool getXDirection() {
    return((READ(X_DIR_PIN) != 0) ^ INVERT_X_DIR);
  }

  /** For large machines, the nonlinear transformation can exceed integer 32bit range, so floating point math is needed. */
  static INLINE uint8_t isLargeMachine() {
    return flag0 & PRINTER_FLAG0_LARGE_MACHINE;
  }

  static INLINE void setLargeMachine(uint8_t b) {
    flag0 = (b ? flag0 | PRINTER_FLAG0_LARGE_MACHINE : flag0 & ~PRINTER_FLAG0_LARGE_MACHINE);
  }

  static INLINE uint8_t isAdvanceActivated() {
    return flag0 & PRINTER_FLAG0_SEPERATE_EXTRUDER_INT;
  }

  static INLINE void setAdvanceActivated(uint8_t b) {
    flag0 = (b ? flag0 | PRINTER_FLAG0_SEPERATE_EXTRUDER_INT : flag0 & ~PRINTER_FLAG0_SEPERATE_EXTRUDER_INT);
  }

  static INLINE uint8_t isHomedAll() {
    return flag1 & PRINTER_FLAG1_HOMED_ALL;
  }

  static INLINE void unsetHomedAll() {
    flag1 &= ~PRINTER_FLAG1_HOMED_ALL;
    flag3 &= ~(PRINTER_FLAG3_X_HOMED | PRINTER_FLAG3_Y_HOMED | PRINTER_FLAG3_Z_HOMED);
  }

  static INLINE void updateHomedAll() {
    bool b = isXHomed() && isYHomed() && isZHomed();
    flag1 = (b ? flag1 | PRINTER_FLAG1_HOMED_ALL : flag1 & ~PRINTER_FLAG1_HOMED_ALL);
  }

  static INLINE uint8_t isXHomed() {
    return flag3 & PRINTER_FLAG3_X_HOMED;
  }

  static INLINE void setXHomed(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_X_HOMED : flag3 & ~PRINTER_FLAG3_X_HOMED);
    updateHomedAll();
  }

  static INLINE uint8_t isYHomed() {
    return flag3 & PRINTER_FLAG3_Y_HOMED;
  }

  static INLINE void setYHomed(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_Y_HOMED : flag3 & ~PRINTER_FLAG3_Y_HOMED);
    updateHomedAll();
  }

  static INLINE uint8_t isZHomed() {
    return flag3 & PRINTER_FLAG3_Z_HOMED;
  }

  static INLINE void setZHomed(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_Z_HOMED : flag3 & ~PRINTER_FLAG3_Z_HOMED);
    updateHomedAll();
  }
  static INLINE uint8_t isAutoreportTemp() {
    return flag3 & PRINTER_FLAG3_AUTOREPORT_TEMP;
  }

  static INLINE void setAutoreportTemp(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_AUTOREPORT_TEMP : flag3 & ~PRINTER_FLAG3_AUTOREPORT_TEMP);
  }

  static INLINE uint8_t isAllKilled() {
    return flag1 & PRINTER_FLAG1_ALLKILLED;
  }

  static INLINE void setAllKilled(uint8_t b) {
    flag1 = (b ? flag1 | PRINTER_FLAG1_ALLKILLED : flag1 & ~PRINTER_FLAG1_ALLKILLED);
  }

  static INLINE uint8_t isUIErrorMessage() {
    return flag1 & PRINTER_FLAG1_UI_ERROR_MESSAGE;
  }

  static INLINE void setUIErrorMessage(uint8_t b) {
    flag1 = (b ? flag1 | PRINTER_FLAG1_UI_ERROR_MESSAGE : flag1 & ~PRINTER_FLAG1_UI_ERROR_MESSAGE);
  }

  static INLINE uint8_t isNoDestinationCheck() {
    return flag1 & PRINTER_FLAG1_NO_DESTINATION_CHECK;
  }

  static INLINE void setNoDestinationCheck(uint8_t b) {
    flag1 = (b ? flag1 | PRINTER_FLAG1_NO_DESTINATION_CHECK : flag1 & ~PRINTER_FLAG1_NO_DESTINATION_CHECK);
  }

  static INLINE uint8_t isPowerOn() {
    return flag1 & PRINTER_FLAG1_POWER_ON;
  }

  static INLINE void setPowerOn(uint8_t b) {
    flag1 = (b ? flag1 | PRINTER_FLAG1_POWER_ON : flag1 & ~PRINTER_FLAG1_POWER_ON);
  }

  static INLINE uint8_t isColdExtrusionAllowed() {
    return flag1 & PRINTER_FLAG1_ALLOW_COLD_EXTRUSION;
  }

  static INLINE void setColdExtrusionAllowed(uint8_t b) {
    flag1 = (b ? flag1 | PRINTER_FLAG1_ALLOW_COLD_EXTRUSION : flag1 & ~PRINTER_FLAG1_ALLOW_COLD_EXTRUSION);
    if(b)
      Com::printF(PSTR("Cold extrusion allowed\n"));
    else
      Com::printF(PSTR("Cold extrusion disallowed\n"));
  }

  static INLINE uint8_t isBlockingReceive() {
    return flag2 & PRINTER_FLAG2_BLOCK_RECEIVING;
  }

  static INLINE void setBlockingReceive(uint8_t b) {
    flag2 = (b ? flag2 | PRINTER_FLAG2_BLOCK_RECEIVING : flag2 & ~PRINTER_FLAG2_BLOCK_RECEIVING);
    Com::printF(b ? PSTR("// action:pause\n") : PSTR("// action:resume\n"));
  }

  static INLINE uint8_t isPrinting() {
    return flag3 & PRINTER_FLAG3_PRINTING;
  }

  static INLINE void setPrinting(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_PRINTING : flag3 & ~PRINTER_FLAG3_PRINTING);
    Printer::setMenuMode(MODE_PRINTING, b);
  }

  static INLINE uint8_t isStartStopSupported() {
    return flag3 & PRINTER_FLAG3_SUPPORTS_STARTSTOP;
  }

  static INLINE void setSupportStartStop(uint8_t b) {
    flag3 = (b ? flag3 | PRINTER_FLAG3_SUPPORTS_STARTSTOP : flag3 & ~PRINTER_FLAG3_SUPPORTS_STARTSTOP);
  }

  static INLINE uint8_t isHoming() {
    return flag2 & PRINTER_FLAG2_HOMING;
  }

  static INLINE void setHoming(uint8_t b) {
    flag2 = (b ? flag2 | PRINTER_FLAG2_HOMING : flag2 & ~PRINTER_FLAG2_HOMING);
  }
  static INLINE uint8_t isAllEMotors() {
    return flag2 & PRINTER_FLAG2_ALL_E_MOTORS;
  }

  static INLINE void setAllEMotors(uint8_t b) {
    flag2 = (b ? flag2 | PRINTER_FLAG2_ALL_E_MOTORS : flag2 & ~PRINTER_FLAG2_ALL_E_MOTORS);
  }

  static INLINE float convertToMM(float x) {
    return (unitIsInches ? x * 25.4 : x);
  }
  static INLINE bool areAllSteppersDisabled() {
    return flag0 & PRINTER_FLAG0_STEPPER_DISABLED;
  }
  static INLINE void setAllSteppersDiabled() {
    flag0 |= PRINTER_FLAG0_STEPPER_DISABLED;
  }
  static INLINE void unsetAllSteppersDisabled() {
    flag0 &= ~PRINTER_FLAG0_STEPPER_DISABLED;
  }
  static INLINE bool isAnyTempsensorDefect() {
    return (flag0 & PRINTER_FLAG0_TEMPSENSOR_DEFECT);
  }
  static INLINE void setAnyTempsensorDefect() {
    flag0 |= PRINTER_FLAG0_TEMPSENSOR_DEFECT;
  }
  static INLINE void unsetAnyTempsensorDefect() {
    flag0 &= ~PRINTER_FLAG0_TEMPSENSOR_DEFECT;
  }
  static INLINE bool isManualMoveMode() {
    return (flag0 & PRINTER_FLAG0_MANUAL_MOVE_MODE);
  }
  static INLINE void setManualMoveMode(bool on) {
    flag0 = (on ? flag0 | PRINTER_FLAG0_MANUAL_MOVE_MODE : flag0 & ~PRINTER_FLAG0_MANUAL_MOVE_MODE);
  }
  static INLINE bool isAutolevelActive() {
    return (flag0 & PRINTER_FLAG0_AUTOLEVEL_ACTIVE) != 0;
  }
  static void setAutolevelActive(bool on);

  static INLINE void setZProbingActive(bool on) {
    flag0 = (on ? flag0 | PRINTER_FLAG0_ZPROBEING : flag0 & ~PRINTER_FLAG0_ZPROBEING);
  }
  static INLINE bool isZProbingActive() {
    return (flag0 & PRINTER_FLAG0_ZPROBEING);
  }

  static INLINE void executeXYGantrySteps() {}
  static INLINE void executeXZGantrySteps() {}

  static INLINE void startXStep() {WRITE(X_STEP_PIN, START_STEP_WITH_HIGH);}
  static INLINE void startYStep() {WRITE(Y_STEP_PIN, START_STEP_WITH_HIGH);}
  static INLINE void startZStep() {WRITE(Z_STEP_PIN, START_STEP_WITH_HIGH);}

  static INLINE void endXYZSteps() {
    WRITE(X_STEP_PIN, !START_STEP_WITH_HIGH);
    WRITE(Y_STEP_PIN, !START_STEP_WITH_HIGH);
    WRITE(Z_STEP_PIN, !START_STEP_WITH_HIGH);
  }

  static INLINE uint16_t updateStepsPerTimerCall(uint16_t vbase) {
    if(vbase > STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
      if(vbase > STEP_DOUBLER_FREQUENCY * 2) {
        Printer::stepsPerTimerCall = 4;
        return vbase >> 2;
      } else {
        Printer::stepsPerTimerCall = 2;
        return vbase >> 1;
      }
#else
      Printer::stepsPerTimerCall = 2;
      return vbase >> 1;
#endif
    } else {
      Printer::stepsPerTimerCall = 1;
    }
    return vbase;
  }
  static INLINE void disableAllowedStepper() {
  }
  static INLINE float realXPosition() {
    return currentPosition[X_AXIS];
  }

  static INLINE float realYPosition() {
    return currentPosition[Y_AXIS];
  }

  static INLINE float realZPosition() {
    return currentPosition[Z_AXIS];
  }
  /** \brief copies currentPosition to parameter. */
  static INLINE void realPosition(float &xp, float &yp, float &zp) {
    xp = currentPosition[X_AXIS];
    yp = currentPosition[Y_AXIS];
    zp = currentPosition[Z_AXIS];
  }
  static INLINE void insertStepperHighDelay() {
#if STEPPER_HIGH_DELAY>0
    delayMicroseconds(STEPPER_HIGH_DELAY);
#endif
  }
  static void updateDerivedParameter();
  /** If we are not homing or destination check being disabled, this will reduce _destinationSteps_ to a
      valid value. In other words this works as software endstop. */
  static void constrainDestinationCoords();
  /** Computes _currentposition_ from _currentPositionSteps_ considering all active transformations. If the _copyLastCmd_ flag is true, the
      result is also copied to _lastCmdPos_ . */
  static void updateCurrentPosition(bool copyLastCmd = false);
  static void updateCurrentPositionSteps();
  /** \brief Sets the destination coordinates to values stored in com.

      Extracts x,y,z,e,f from g-code considering active units. Converted result is stored in currentPosition and lastCmdPos. Converts
      position to destinationSteps including rotation and offsets, excluding distortion correction (which gets added on move queuing).
      \param com g-code with new destination position.
      \return true if it is a move, false if no move results from coordinates.
  */
  static uint8_t setDestinationStepsFromGCode(gcodeCommand *com);
  /** \brief Move to position without considering transformations.

      Computes the destinationSteps without rotating but including active offsets!
      The coordinates are in printer coordinates with no G92 offset.

      \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
      \return true if queuing was successful.
  */
  static uint8_t moveTo(float x, float y, float z, float e, float f);
  /** \brief Move to position considering transformations.

      Computes the destinationSteps including rotating and active offsets.
      The coordinates are in printer coordinates with no G92 offset.

      \param x Target x coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target y coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target z coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target e coordinate or IGNORE_COORDINATE if it should be ignored.
      \param x Target f feedrate or IGNORE_COORDINATE if it should use latest feedrate.
      \param pathOptimize true if path planner should include it in calculation, otherwise default start/end speed is enforced.
      \return true if queuing was successful.
  */
  static uint8_t moveToReal(float x, float y, float z, float e, float f, bool pathOptimize = true);
  static void kill(uint8_t only_steppers);
  static void updateAdvanceFlags();
  static void setup();

  static void homeAxis(bool xaxis, bool yaxis, bool zaxis); /// Home axis
  static void setOrigin(float xOff, float yOff, float zOff);
  /** \brief Tests if the target position is allowed.

      Tests if the test position lies inside the defined geometry. For Cartesian
      printers this is the defined cube defined by x,y,z min and length. For
      delta printers the cylindrical shape is tested.

      \param x X position in mm.
      \param x Y position in mm.
      \param x Z position in mm.
      \return true if position is valid and can be reached. */
  static bool isPositionAllowed(float x, float y, float z);

  static INLINE void setDeltaPositions(long xaxis, long yaxis, long zaxis) {
    currentNonlinearPositionSteps[A_TOWER] = xaxis;
    currentNonlinearPositionSteps[B_TOWER] = yaxis;
    currentNonlinearPositionSteps[C_TOWER] = zaxis;
  }
  static void deltaMoveToTopEndstops(float feedrate);

  static void transformToPrinter(float x, float y, float z, float &transX, float &transY, float &transZ);
  static void transformFromPrinter(float x, float y, float z, float &transX, float &transY, float &transZ);

#if FEATURE_AUTOLEVEL
  static void resetTransformationMatrix(bool silent);
  //static void buildTransformationMatrix(float h1,float h2,float h3);
  static void buildTransformationMatrix(Plane &plane);
#endif

#if DISTORTION_CORRECTION
  static void measureDistortion(void);
  static Distortion distortion;
#endif
  static void MemoryPosition();
  static void GoToMemoryPosition(bool x, bool y, bool z, bool e, float feed);
  static void zBabystep();

  //static void showConfiguration();

  static void homeZAxis();
  static void pausePrint();
  static void continuePrint();
  static void stopPrint();

	static void moveToParkPosition();

  static void selectExtruderById(uint8_t extruderId);
};

#endif // PRINTER_H_INCLUDED
