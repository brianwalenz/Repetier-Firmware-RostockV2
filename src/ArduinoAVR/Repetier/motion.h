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
  #
  Functions in this file are used to communicate using ascii or repetier protocol.
*/

#ifndef MOTION_H_INCLUDED
#define MOTION_H_INCLUDED

#include "Printer.h"
#include "Extruder.h"

/** Marks the first step of a new move */
#define FLAG_WARMUP 1
#define FLAG_NOMINAL 2
#define FLAG_DECELERATING 4
#define FLAG_ACCELERATION_ENABLED 8 // unused
#define FLAG_CHECK_ENDSTOPS 16
#define FLAG_ALL_E_MOTORS 32 // For mixed extruder move all motors instead of selected motor
#define FLAG_SKIP_DEACCELERATING 64 // unused
#define FLAG_BLOCKED 128

/** Are the step parameter computed */
#define FLAG_JOIN_STEPPARAMS_COMPUTED 1
/** The right speed is fixed. Don't check this block or any block to the left. */
#define FLAG_JOIN_END_FIXED 2
/** The left speed is fixed. Don't check left block. */
#define FLAG_JOIN_START_FIXED 4
/** Start filament retraction at move start */
#define FLAG_JOIN_START_RETRACT 8
/** Wait for filament push back, before ending move */
#define FLAG_JOIN_END_RETRACT 16
/** Disable retract for this line */
#define FLAG_JOIN_NO_RETRACT 32
/** Wait for the extruder to finish it's up movement */
#define FLAG_JOIN_WAIT_EXTRUDER_UP 64
/** Wait for the extruder to finish it's down movement */
#define FLAG_JOIN_WAIT_EXTRUDER_DOWN 128
// Printing related data

// Allow the delta cache to store segments for every line in line cache. Beware this gets big ... fast.

class PrintLine;
typedef struct {
  flag8_t dir;                                    ///< Direction of delta movement.
  uint16_t deltaSteps[TOWER_ARRAY];                       ///< Number of steps in move.
  inline bool checkEndstops(PrintLine *cur, bool checkall);
  inline void setXMoveFinished() {
    dir &= ~XSTEP;
  }
  inline void setYMoveFinished() {
    dir &= ~YSTEP;
  }
  inline void setZMoveFinished() {
    dir &= ~ZSTEP;
  }
  inline void setXYMoveFinished() {
    dir &= ~XY_STEP;
  }
  inline bool isXPositiveMove() {
    return (dir & X_STEP_DIRPOS) == X_STEP_DIRPOS;
  }
  inline bool isXNegativeMove() {
    return (dir & X_STEP_DIRPOS) == XSTEP;
  }
  inline bool isYPositiveMove() {
    return (dir & Y_STEP_DIRPOS) == Y_STEP_DIRPOS;
  }
  inline bool isYNegativeMove() {
    return (dir & Y_STEP_DIRPOS) == YSTEP;
  }
  inline bool isZPositiveMove() {
    return (dir & Z_STEP_DIRPOS) == Z_STEP_DIRPOS;
  }
  inline bool isZNegativeMove() {
    return (dir & Z_STEP_DIRPOS) == ZSTEP;
  }
  inline bool isEPositiveMove() {
    return (dir & E_STEP_DIRPOS) == E_STEP_DIRPOS;
  }
  inline bool isENegativeMove() {
    return (dir & E_STEP_DIRPOS) == ESTEP;
  }
  inline bool isXMove() {
    return (dir & XSTEP);
  }
  inline bool isYMove() {
    return (dir & YSTEP);
  }
  inline bool isXOrYMove() {
    return dir & XY_STEP;
  }
  inline bool isZMove() {
    return (dir & ZSTEP);
  }
  inline bool isEMove() {
    return (dir & ESTEP);
  }
  inline bool isEOnlyMove() {
    return (dir & XYZE_STEP) == ESTEP;
  }
  inline bool isNoMove() {
    return (dir & XYZE_STEP) == 0;
  }
  inline bool isXYZMove() {
    return dir & XYZ_STEP;
  }
  inline bool isMoveOfAxis(uint8_t axis) {
    return (dir & (XSTEP << axis));
  }
  inline void setMoveOfAxis(uint8_t axis) {
    dir |= XSTEP << axis;
  }
  inline void setPositiveMoveOfAxis(uint8_t axis) {
    dir |= X_STEP_DIRPOS << axis;
  }
  inline void setPositiveDirectionForAxis(uint8_t axis) {
    dir |= X_DIRPOS << axis;
  }
} NonlinearSegment;
extern uint8_t lastMoveID;



class UIDisplay;
class PrintLine { // RAM usage: 24*4+15 = 113 Byte
  friend class UIDisplay;
public:
  static ufast8_t linesPos; // Position for executing line movement
  static PrintLine lines[];
  static ufast8_t linesWritePos; // Position where we write the next cached line move
  ufast8_t joinFlags;
  volatile ufast8_t flags;
  uint8_t secondSpeed; // for laser intensity or fan control
private:
  fast8_t primaryAxis;
  ufast8_t dir;                       ///< Direction of movement. 1 = X+, 2 = Y+, 4= Z+, values can be combined.
  int32_t timeInTicks;
  int32_t delta[E_AXIS_ARRAY];                  ///< Steps we want to move.
  int32_t error[E_AXIS_ARRAY];                  ///< Error calculation for Bresenham algorithm
  float speedX;                   ///< Speed in x direction at fullInterval in mm/s
  float speedY;                   ///< Speed in y direction at fullInterval in mm/s
  float speedZ;                   ///< Speed in z direction at fullInterval in mm/s
  float speedE;                   ///< Speed in E direction at fullInterval in mm/s
  float fullSpeed;                ///< Desired speed mm/s
  float invFullSpeed;             ///< 1.0/fullSpeed for faster computation
  float accelerationDistance2;    ///< Real 2.0*distance*acceleration mm²/s²
  float maxJunctionSpeed;         ///< Max. junction speed between this and next segment
  float startSpeed;               ///< Starting speed in mm/s
  float endSpeed;                 ///< Exit speed in mm/s
  float minSpeed;
  float distance;
  uint8_t numNonlinearSegments;       ///< Number of delta segments left in line. Decremented by stepper timer.
  uint8_t moveID;                 ///< ID used to identify moves which are all part of the same line
  int32_t numPrimaryStepPerSegment;   ///< Number of primary Bresenham axis steps in each delta segment
  NonlinearSegment segments[DELTASEGMENTS_PER_PRINTLINE];
  ticks_t fullInterval;     ///< interval at full speed in ticks/step.
  uint32_t accelSteps;        ///< How much steps does it take, to reach the plateau.
  uint32_t decelSteps;        ///< How much steps does it take, to reach the end speed.
  uint32_t accelerationPrim; ///< Acceleration along primary axis
  uint32_t fAcceleration;    ///< accelerationPrim*262144/F_CPU
  speed_t vMax;              ///< Maximum reached speed in steps/s.
  speed_t vStart;            ///< Starting speed in steps/s.
  speed_t vEnd;              ///< End speed in steps/s
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  int32_t advanceRate;               ///< Advance steps at full speed
  int32_t advanceFull;               ///< Maximum advance at fullInterval [steps*65536]
  int32_t advanceStart;
  int32_t advanceEnd;
#endif
  uint16_t advanceL;         ///< Recomputed L value
#endif
#ifdef DEBUG_STEPCOUNT
  int32_t totalStepsRemaining;
#endif
public:
  int32_t stepsRemaining;            ///< Remaining steps, until move is finished
  static PrintLine *cur;
  static volatile ufast8_t linesCount; // Number of lines cached 0 = nothing to do
  inline bool areParameterUpToDate() {
    return joinFlags & FLAG_JOIN_STEPPARAMS_COMPUTED;
  }
  inline void invalidateParameter() {
    joinFlags &= ~FLAG_JOIN_STEPPARAMS_COMPUTED;
  }
  inline void setParameterUpToDate() {
    joinFlags |= FLAG_JOIN_STEPPARAMS_COMPUTED;
  }
  inline bool isStartSpeedFixed() {
    return joinFlags & FLAG_JOIN_START_FIXED;
  }
  inline void setStartSpeedFixed(bool newState) {
    joinFlags = (newState ? joinFlags | FLAG_JOIN_START_FIXED : joinFlags & ~FLAG_JOIN_START_FIXED);
  }
  inline void fixStartAndEndSpeed() {
    joinFlags |= FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
  }
  inline bool isEndSpeedFixed() {
    return joinFlags & FLAG_JOIN_END_FIXED;
  }
  inline void setEndSpeedFixed(bool newState) {
    joinFlags = (newState ? joinFlags | FLAG_JOIN_END_FIXED : joinFlags & ~FLAG_JOIN_END_FIXED);
  }
  inline bool isWarmUp() {
    return flags & FLAG_WARMUP;
  }
  inline uint8_t getWaitForXLinesFilled() {
    return primaryAxis;
  }
  inline void setWaitForXLinesFilled(uint8_t b) {
    primaryAxis = b;
  }
  inline bool isExtruderForwardMove() {
    return (dir & E_STEP_DIRPOS) == E_STEP_DIRPOS;
  }
  inline void block() {
    flags |= FLAG_BLOCKED;
  }
  inline void unblock() {
    flags &= ~FLAG_BLOCKED;
  }
  inline bool isBlocked() {
    return flags & FLAG_BLOCKED;
  }
  inline bool isAllEMotors() {
    return flags & FLAG_ALL_E_MOTORS;
  }
  inline bool isCheckEndstops() {
    return flags & FLAG_CHECK_ENDSTOPS;
  }
  inline bool isNominalMove() {
    return flags & FLAG_NOMINAL;
  }
  inline void setNominalMove() {
    flags |= FLAG_NOMINAL;
  }
  inline void checkEndstops() {
    if(isCheckEndstops()) {
      Endstops::update();
      if(Endstops::anyEndstopHit()) {
        if(isXNegativeMove() && Endstops::xMin())
          setXMoveFinished();
        else if(isXPositiveMove() && Endstops::xMax())
          setXMoveFinished();
        if(isYNegativeMove() && Endstops::yMin())
          setYMoveFinished();
        else if(isYPositiveMove() && Endstops::yMax())
          setYMoveFinished();
#if FEATURE_Z_PROBE
        if(Printer::isZProbingActive() && isZNegativeMove() && Endstops::zProbe()) {
          setZMoveFinished();
          Printer::stepsRemainingAtZHit = stepsRemaining;
        } else
#endif
          if(isZNegativeMove() && Endstops::zMin()
#if Z_MIN_PIN == Z_PROBE_PIN && FEATURE_Z_PROBE
             && Printer::isHoming()
#endif
             ) {
            setZMoveFinished();
          } else if(isZPositiveMove() && Endstops::zMax()) {
#if MAX_HARDWARE_ENDSTOP_Z
            Printer::stepsRemainingAtZHit = stepsRemaining;
#endif
            setZMoveFinished();
          }
      }
#if FEATURE_Z_PROBE
      else if(Printer::isZProbingActive() && isZNegativeMove()) {
        Endstops::update();
        if(Endstops::zProbe()) {
          setZMoveFinished();
          Printer::stepsRemainingAtZHit = stepsRemaining;
        }
      }
#endif
    }
  }

  inline void setXMoveFinished() {
    dir &= ~16;
  }
  inline void setYMoveFinished() {
    dir &= ~32;
  }
  inline void setZMoveFinished() {
    dir &= ~64;
  }
  inline void setXYMoveFinished() {
    dir &= ~48;
  }
  inline bool isXPositiveMove() {
    return (dir & X_STEP_DIRPOS) == X_STEP_DIRPOS;
  }
  inline bool isXNegativeMove() {
    return (dir & X_STEP_DIRPOS) == XSTEP;
  }
  inline bool isYPositiveMove() {
    return (dir & Y_STEP_DIRPOS) == Y_STEP_DIRPOS;
  }
  inline bool isYNegativeMove() {
    return (dir & Y_STEP_DIRPOS) == YSTEP;
  }
  inline bool isZPositiveMove() {
    return (dir & Z_STEP_DIRPOS) == Z_STEP_DIRPOS;
  }
  inline bool isZNegativeMove() {
    return (dir & Z_STEP_DIRPOS) == ZSTEP;
  }
  inline bool isEPositiveMove() {
    return (dir & E_STEP_DIRPOS) == E_STEP_DIRPOS;
  }
  inline bool isENegativeMove() {
    return (dir & E_STEP_DIRPOS) == ESTEP;
  }
  inline bool isXMove() {
    return (dir & XSTEP);
  }
  inline bool isYMove() {
    return (dir & YSTEP);
  }
  inline bool isXOrYMove() {
    return dir & XY_STEP;
  }
  inline bool isXOrZMove() {
    return dir & (XSTEP | ZSTEP);
  }
  inline bool isZMove() {
    return (dir & ZSTEP);
  }
  inline bool isEMove() {
    return (dir & ESTEP);
  }
  inline bool isEOnlyMove() {
    return (dir & XYZE_STEP) == ESTEP;
  }
  inline bool isNoMove() {
    return (dir & XYZE_STEP) == 0;
  }
  inline bool isXYZMove() {
    return dir & XYZ_STEP;
  }
  inline bool isMoveOfAxis(uint8_t axis) {
    return (dir & (XSTEP << axis));
  }
  inline void setMoveOfAxis(uint8_t axis) {
    dir |= XSTEP << axis;
  }
  inline void setPositiveDirectionForAxis(uint8_t axis) {
    dir |= X_DIRPOS << axis;
  }
  inline static void resetPathPlanner() {
    linesCount = 0;
    linesPos = linesWritePos;
    Printer::setMenuMode(MODE_PRINTING, Printer::isPrinting());
  }
  // Only called from bresenham -> inside interrupt handle
  inline void updateAdvanceSteps(speed_t v, uint8_t max_loops, bool accelerate) {
#if USE_ADVANCE
    if(!Printer::isAdvanceActivated()) return;
#if ENABLE_QUADRATIC_ADVANCE
    long advanceTarget = Printer::advanceExecuted;
    if(accelerate) {
      for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget += advanceRate;
      if(advanceTarget > advanceFull)
        advanceTarget = advanceFull;
    } else {
      for(uint8_t loop = 0; loop < max_loops; loop++) advanceTarget -= advanceRate;
      if(advanceTarget < advanceEnd)
        advanceTarget = advanceEnd;
    }
    long h = HAL::mulu16xu16to32(v, advanceL);
    int tred = ((advanceTarget + h) >> 16);
    HAL::forbidInterrupts();
    Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
    if(tred > 0 && Printer::advanceStepsSet <= 0)
      Printer::extruderStepsNeeded += Extruder::current->advanceBacklash;
    else if(tred < 0 && Printer::advanceStepsSet >= 0)
      Printer::extruderStepsNeeded -= Extruder::current->advanceBacklash;
    Printer::advanceStepsSet = tred;
    HAL::allowInterrupts();
    Printer::advanceExecuted = advanceTarget;
#else
    int tred = HAL::mulu6xu16shift16(v, advanceL);
    HAL::forbidInterrupts();
    Printer::extruderStepsNeeded += tred - Printer::advanceStepsSet;
    if(tred > 0 && Printer::advanceStepsSet <= 0)
      Printer::extruderStepsNeeded += (Extruder::current->advanceBacklash << 1);
    else if(tred < 0 && Printer::advanceStepsSet >= 0)
      Printer::extruderStepsNeeded -= (Extruder::current->advanceBacklash << 1);
    Printer::advanceStepsSet = tred;
    HAL::allowInterrupts();
#endif
#endif
  }
  INLINE bool moveDecelerating() {
    if(stepsRemaining <= static_cast<int32_t>(decelSteps)) {
      if (!(flags & FLAG_DECELERATING)) {
        Printer::timer = 0;
        flags |= FLAG_DECELERATING;
      }
      return true;
    } else return false;
  }
  INLINE bool moveAccelerating() {
    return Printer::stepNumber <= accelSteps;
  }
  INLINE void startXStep() {
    Printer::startXStep();
#ifdef DEBUG_STEPCOUNT
    totalStepsRemaining--;
#endif
  }
  INLINE void startYStep() {

    Printer::startYStep();

#ifdef DEBUG_STEPCOUNT
    totalStepsRemaining--;
#endif

  }
  INLINE void startZStep() {

    Printer::startZStep();

#ifdef DEBUG_STEPCOUNT
    totalStepsRemaining--;
#endif
  }
  void updateStepsParameter();
  float safeSpeed(fast8_t drivingAxis);
  void calculateMove(float axis_diff[], uint8_t pathOptimize, fast8_t distanceBase);
  void logLine();
  INLINE long getWaitTicks() {
    return timeInTicks;
  }
  INLINE void setWaitTicks(long wait) {
    timeInTicks = wait;
  }

  static INLINE bool hasLines() {
    return linesCount;
  }
  static INLINE void setCurrentLine() {
    cur = &lines[linesPos];
  }
  // Only called from within interrupts
  static INLINE void removeCurrentLineForbidInterrupt() {
    nextPlannerIndex(linesPos);
    cur = NULL;
    HAL::forbidInterrupts();
    --linesCount;
    if(!linesCount)
      Printer::setMenuMode(MODE_PRINTING, Printer::isPrinting());
  }
  static INLINE void pushLine() {
    nextPlannerIndex(linesWritePos);
    //Printer::setMenuMode(MODE_PRINTING, true);
    InterruptProtectedBlock noInts;
    linesCount++;
  }
  static uint8_t getLinesCount() {
    InterruptProtectedBlock noInts;
    return linesCount;
  }
  static PrintLine *getNextWriteLine() {
    return &lines[linesWritePos];
  }
  static inline void computeMaxJunctionSpeed(PrintLine *previous, PrintLine *current);
  static int32_t bresenhamStep();
  static void waitForXFreeLines(uint8_t b = 1, bool allowMoves = false);
  static inline void forwardPlanner(ufast8_t p);
  static inline void backwardPlanner(ufast8_t p, ufast8_t last);
  static void updateTrapezoids();
  static uint8_t insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines);

  static void moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd, bool check_endstop, bool pathOptimize = true);
  static void moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd, bool pathOptimize = true);
  static INLINE void previousPlannerIndex(ufast8_t &p) {
    p = (p ? p - 1 : PRINTLINE_CACHE_SIZE - 1);
  }
  static INLINE void nextPlannerIndex(ufast8_t& p) {
    p = (p >= PRINTLINE_CACHE_SIZE - 1 ? 0 : p + 1);
  }

  static uint8_t queueNonlinearMove(uint8_t check_endstops, uint8_t pathOptimize, uint8_t softEndstop);
  static inline void queueEMove(int32_t e_diff, uint8_t check_endstops, uint8_t pathOptimize);
  inline uint16_t calculateNonlinearSubSegments(uint8_t softEndstop);
  static inline void calculateDirectionAndDelta(int32_t difference[], ufast8_t *dir, int32_t delta[]);
  static inline uint8_t calculateDistance(float axis_diff[], uint8_t dir, float *distance);

#if (SOFTWARE_LEVELING)
  static void calculatePlane(int32_t factors[], int32_t p1[], int32_t p2[], int32_t p3[]);
  static float calcZOffset(int32_t factors[], int32_t pointX, int32_t pointY);
#endif
};


extern millis_t previousMillisCmd;
extern millis_t maxInactiveTime;
extern millis_t stepperInactiveTime;


#endif // MOTION_H_INCLUDED
