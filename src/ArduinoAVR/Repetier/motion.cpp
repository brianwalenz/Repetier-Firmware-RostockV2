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
#include "HAL.h"
#include "Commands.h"
#include "motion.h"
#include "Printer.h"
#include "Extruder.h"

#include "rmath.h"

// ================ Sanity checks ================


//Inactivity shutdown variables
uint32_t previousMillisCmd = 0;
uint32_t maxInactiveTime = MAX_INACTIVE_TIME * 1000L;
uint32_t stepperInactiveTime = STEPPER_INACTIVE_TIME * 1000L;

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
int maxadv = 0;
#endif
int maxadv2 = 0;
float maxadvspeed = 0;
#endif

uint8_t pwm_pos[NUM_PWM]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan
volatile int waitRelax = 0; // Delay filament relax at the end of print, could be a simple timeout

PrintLine PrintLine::lines[PRINTLINE_CACHE_SIZE]; ///< Cache for print moves.
PrintLine *PrintLine::cur = NULL;               ///< Current printing line
uint8_t PrintLine::linesWritePos = 0;            ///< Position where we write the next cached line move.
volatile uint8_t PrintLine::linesCount = 0;      ///< Number of lines cached 0 = nothing to do.
uint8_t PrintLine::linesPos = 0;                 ///< Position for executing line movement.

/**
   Move printer the given number of steps. Puts the move into the queue. Used by e.g. homing commands.
   Does not consider rotation but updates position correctly considering rotation. This can be used to
   correct positions when changing tools.

   \param x Distance in x direction in steps
   \param y Distance in y direction in steps
   \param z Distance in z direction in steps
   \param e Distance in e direction in steps
   \param feedrate Feed rate to be used in mm/s. Gets new active feedrate.
   \param waitEnd If true will block until move is finished.
   \param checkEndstop True if triggering endstop should stop move.
   \param pathOptimize If false start and end speeds get fixed to minimum values.
*/
void PrintLine::moveRelativeDistanceInSteps(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd, bool checkEndstop, bool pathOptimize) {
  if(Printer::debugDryrun() || (MIN_EXTRUDER_TEMP > 30 && Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0))
    e = 0; // should not be allowed for current temperature

  float savedFeedrate = Printer::feedrate;
  Printer::destinationSteps[X_AXIS] = Printer::currentPositionSteps[X_AXIS] + x;
  Printer::destinationSteps[Y_AXIS] = Printer::currentPositionSteps[Y_AXIS] + y;
  Printer::destinationSteps[Z_AXIS] = Printer::currentPositionSteps[Z_AXIS] + z;
  Printer::destinationSteps[E_AXIS] = Printer::currentPositionSteps[E_AXIS] + e;
  Printer::feedrate = feedrate;
  if (!queueNonlinearMove(checkEndstop, pathOptimize, false)) {
    Com::printF(PSTR("WARNING: moveRelativeDistanceInSteps / queueDeltaMove returns error\n"));
  }
  Printer::feedrate = savedFeedrate;
  Printer::updateCurrentPosition(false);
  if(waitEnd)
    Commands::waitUntilEndOfAllMoves();
  previousMillisCmd = HAL::timeInMilliseconds();
}

/** Adds the steps converted to mm to the lastCmdPos position and moves to that position using Printer::moveToReal.
    Will use Printer::isPositionAllowed to prevent illegal moves.

    \param x Distance in x direction in steps
    \param y Distance in y direction in steps
    \param z Distance in z direction in steps
    \param e Distance in e direction in steps
    \param feedrate Feed rate to be used in mm/s. Gets new active feedrate.
    \param waitEnd If true will block until move is finished.
    \param pathOptimize If false start and end speeds get fixed to minimum values.
*/
void PrintLine::moveRelativeDistanceInStepsReal(int32_t x, int32_t y, int32_t z, int32_t e, float feedrate, bool waitEnd, bool pathOptimize) {
  Printer::lastCmdPos[X_AXIS] += x * Printer::invAxisStepsPerMM[X_AXIS];
  Printer::lastCmdPos[Y_AXIS] += y * Printer::invAxisStepsPerMM[Y_AXIS];
  Printer::lastCmdPos[Z_AXIS] += z * Printer::invAxisStepsPerMM[Z_AXIS];

  if(!Printer::isPositionAllowed( Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS])) {
    return; // ignore move
  }

  if(Printer::debugDryrun() || (MIN_EXTRUDER_TEMP > 30 && Extruder::current->tempControl.currentTemperatureC < MIN_EXTRUDER_TEMP && !Printer::isColdExtrusionAllowed() && Extruder::current->tempControl.sensorType != 0))
    e = 0; // should not be allowed for current temperature

  Printer::moveToReal(Printer::lastCmdPos[X_AXIS], Printer::lastCmdPos[Y_AXIS], Printer::lastCmdPos[Z_AXIS],
                      (Printer::currentPositionSteps[E_AXIS] + e) * Printer::invAxisStepsPerMM[E_AXIS], feedrate, pathOptimize);
  Printer::updateCurrentPosition();
  if(waitEnd)
    Commands::waitUntilEndOfAllMoves();
  previousMillisCmd = HAL::timeInMilliseconds();
}



void PrintLine::calculateMove(float axisDistanceMM[], uint8_t pathOptimize, int8_t drivingAxis) {
  long axisInterval[VIRTUAL_AXIS_ARRAY]; // shortest interval possible for that axis

  //float timeForMove = (float)(F_CPU)*distance / (isXOrYMove() ? RMath::max(Printer::minimumSpeed, Printer::feedrate) : Printer::feedrate); // time is in ticks
  float timeForMove = (float)(F_CPU) * distance / Printer::feedrate; // time is in ticks
  //bool critical = Printer::isZProbingActive();
  if(linesCount < MOVE_CACHE_LOW && timeForMove < LOW_TICKS_PER_MOVE) { // Limit speed to keep cache full.
    //Com::printF(PSTR("L:"),(int)linesCount);
    //Com::printF(PSTR(" Old "),timeForMove);
    timeForMove += (3 * (LOW_TICKS_PER_MOVE - timeForMove)) / (linesCount + 1); // Increase time if queue gets empty. Add more time if queue gets smaller.
    //Com::printF(PSTR("Slow "),timeForMove);
    //Com::printF(PSTR("\n"));
    //critical = true;
  }
  timeInTicks = timeForMove;
  uid.mediumAction(); // do check encoder
  // Compute the slowest allowed interval (ticks/step), so maximum feedrate is not violated
  int32_t limitInterval0;
  int32_t limitInterval = limitInterval0 = timeForMove / stepsRemaining; // until not violated by other constraints it is your target speed
  float toTicks = static_cast<float>(F_CPU) / stepsRemaining;
  if(isXMove()) {
    axisInterval[X_AXIS] = axisDistanceMM[X_AXIS] * toTicks / (Printer::maxFeedrate[X_AXIS]); // mm*ticks/s/(mm/s*steps) = ticks/step
  } else axisInterval[X_AXIS] = 0;
  if(isYMove()) {
    axisInterval[Y_AXIS] = axisDistanceMM[Y_AXIS] * toTicks / Printer::maxFeedrate[Y_AXIS];
  } else axisInterval[Y_AXIS] = 0;
  if(isZMove()) { // normally no move in z direction
    axisInterval[Z_AXIS] = axisDistanceMM[Z_AXIS] * toTicks / Printer::maxFeedrate[Z_AXIS]; // must prevent overflow!
  } else axisInterval[Z_AXIS] = 0;
  if(isEMove()) {
    axisInterval[E_AXIS] = axisDistanceMM[E_AXIS] * toTicks / Printer::maxFeedrate[E_AXIS];
    limitInterval = RMath::max(axisInterval[E_AXIS], limitInterval);
  } else axisInterval[E_AXIS] = 0;
  if(axisDistanceMM[VIRTUAL_AXIS] >= 0) {// only for deltas all speeds in all directions have same limit
    axisInterval[VIRTUAL_AXIS] = axisDistanceMM[VIRTUAL_AXIS] * toTicks / (Printer::maxFeedrate[Z_AXIS]);
    limitInterval = RMath::max(axisInterval[VIRTUAL_AXIS], limitInterval);
  }
  fullInterval = limitInterval = limitInterval > LIMIT_INTERVAL ? limitInterval : LIMIT_INTERVAL; // This is our target speed
  if(limitInterval != limitInterval0) {
    // new time at full speed = limitInterval*p->stepsRemaining [ticks]
    timeForMove = (float)limitInterval * (float)stepsRemaining; // for large z-distance this overflows with long computation
  }
  float inverseTimeS = (float)F_CPU / timeForMove;
  if(isXMove()) {
    axisInterval[X_AXIS] = timeForMove / delta[X_AXIS];
    speedX = axisDistanceMM[X_AXIS] * inverseTimeS;
    if(isXNegativeMove()) speedX = -speedX;
  } else speedX = 0;
  if(isYMove()) {
    axisInterval[Y_AXIS] = timeForMove / delta[Y_AXIS];
    speedY = axisDistanceMM[Y_AXIS] * inverseTimeS;
    if(isYNegativeMove()) speedY = -speedY;
  } else speedY = 0;
  if(isZMove()) {
    axisInterval[Z_AXIS] = timeForMove / delta[Z_AXIS];
    speedZ = axisDistanceMM[Z_AXIS] * inverseTimeS;
    if(isZNegativeMove()) speedZ = -speedZ;
  } else speedZ = 0;
  if(isEMove()) {
    axisInterval[E_AXIS] = timeForMove / delta[E_AXIS];
    speedE = axisDistanceMM[E_AXIS] * inverseTimeS;
    if(isENegativeMove()) speedE = -speedE;
  }
  axisInterval[VIRTUAL_AXIS] = limitInterval; //timeForMove/stepsRemaining;
  fullSpeed = distance * inverseTimeS;
  //long interval = axis_interval[primary_axis]; // time for every step in ticks with full speed
  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
#if RAMP_ACCELERATION
  // slowest time to accelerate from v0 to limitInterval determines used acceleration
  // t = (v_end-v_start)/a
  float slowestAxisPlateauTimeRepro = 1e15; // 1/time to reduce division Unit: 1/s
  uint32_t *accel = (isEPositiveMove() ?  Printer::maxPrintAccelerationStepsPerSquareSecond : Printer::maxTravelAccelerationStepsPerSquareSecond);

  for(int8_t i = 0; i < E_AXIS_ARRAY ; i++) {
    if(isMoveOfAxis(i))
      // v = a * t => t = v/a = F_CPU/(c*a) => 1/t = c*a/F_CPU
      slowestAxisPlateauTimeRepro = RMath::min(slowestAxisPlateauTimeRepro, (float)axisInterval[i] * (float)accel[i]); //  steps/s^2 * step/tick  Ticks/s^2
  }

  // Errors for delta move are initialized in timer (except extruder)
  error[E_AXIS] = stepsRemaining >> 1;

  invFullSpeed = 1.0 / fullSpeed;
  accelerationPrim = slowestAxisPlateauTimeRepro / axisInterval[primaryAxis]; // a = v/t = F_CPU/(c*t): Steps/s^2
  //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
  fAcceleration = 262144.0 * (float)accelerationPrim / F_CPU; // will overflow without float!
  accelerationDistance2 = 2.0 * distance * slowestAxisPlateauTimeRepro * fullSpeed / ((float)F_CPU); // mm^2/s^2
  startSpeed = endSpeed = minSpeed = safeSpeed(drivingAxis);
  if(startSpeed > Printer::feedrate)
    startSpeed = endSpeed = minSpeed = Printer::feedrate;
  // Can accelerate to full speed within the line
  if (startSpeed * startSpeed + accelerationDistance2 >= fullSpeed * fullSpeed)
    setNominalMove();

  vMax = F_CPU / fullInterval; // maximum steps per second, we can reach
  // if(p->vMax>46000)  // gets overflow in N computation
  //   p->vMax = 46000;
  //p->plateauN = (p->vMax*p->vMax/p->accelerationPrim)>>1;
#if USE_ADVANCE
  if(!isXYZMove() || !isEPositiveMove()) {
#if ENABLE_QUADRATIC_ADVANCE
    advanceRate = 0; // No head move or E move only or sucking filament back
    advanceFull = 0;
#endif
    advanceL = 0;
  } else {
    float advlin = fabs(speedE) * Extruder::current->advanceL * 0.001 * Printer::axisStepsPerMM[E_AXIS];
    advanceL = (uint16_t)((65536L * advlin) / vMax); //advanceLscaled = (65536*vE*k2)/vMax
#if ENABLE_QUADRATIC_ADVANCE
    advanceFull = 65536 * Extruder::current->advanceK * speedE * speedE; // Steps*65536 at full speed
    long steps = (HAL::U16SquaredToU32(vMax)) / (accelerationPrim << 1); // v^2/(2*a) = steps needed to accelerate from 0-vMax
    advanceRate = advanceFull / steps;
    if((advanceFull >> 16) > maxadv) {
      maxadv = (advanceFull >> 16);
      maxadvspeed = fabs(speedE);
    }
#endif
    if(advlin > maxadv2) {
      maxadv2 = advlin;
      maxadvspeed = fabs(speedE);
    }
  }
#endif  //  USE_ADVANCE
  uid.mediumAction(); // do check encoder
  updateTrapezoids();
  // how much steps on primary axis do we need to reach target feedrate
  //p->plateauSteps = (long) (((float)p->acceleration *0.5f / slowest_axis_plateau_time_repro + p->vMin) *1.01f/slowest_axis_plateau_time_repro);
#else
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  advanceRate = 0; // No advance for constant speeds
  advanceFull = 0;
#endif
#endif
#endif


#ifdef DEBUG_QUEUE_MOVE
  if(Printer::debugEcho()) {
    logLine();
    Com::printF(PSTR("LimitInterval:"), limitInterval);
    Com::printF(PSTR("Move distance on the XYZ space:"), distance);
    Com::printF(PSTR("Commanded feedrate:"), Printer::feedrate);
    Com::printF(PSTR("Constant full speed move time:"), timeForMove);
  }
#endif
  // Make result permanent
  if (pathOptimize) waitRelax = 70;
  pushLine();
  //HAL::printFreeMemory();
}

/**
   This is the path planner.

   It goes from the last entry and tries to increase the end speed of previous moves in a fashion that the maximum jerk
   is never exceeded. If a segment with reached maximum speed is met, the planner stops. Everything left from this
   is already optimal from previous updates.
   The first 2 entries in the queue are not checked. The first is the one that is already in print and the following will likely to become active.

   The method is called before lines_count is increased!
*/
void PrintLine::updateTrapezoids() {
  uint8_t first = linesWritePos;
  PrintLine *firstLine;
  PrintLine *act = &lines[linesWritePos];
  InterruptProtectedBlock noInts;

  // First we find out how far back we could go with optimization.

  uint8_t maxfirst = linesPos; // first non fixed segment we might change
  if(maxfirst != linesWritePos)
    nextPlannerIndex(maxfirst); // don't touch the line printing
  // Now ignore enough segments to gain enough time for path planning
  uint32_t timeleft = 0;
  // Skip as many stored moves as needed to gain enough time for computation
#if PRINTLINE_CACHE_SIZE < 10
#define minTime 4500L * PRINTLINE_CACHE_SIZE
#else
#define minTime 45000L
#endif
  while(timeleft < minTime && maxfirst != linesWritePos) {
    timeleft += lines[maxfirst].timeInTicks;
    nextPlannerIndex(maxfirst);
  }
  // Search last fixed element
  while(first != maxfirst && !lines[first].isEndSpeedFixed())
    previousPlannerIndex(first);
  if(first != linesWritePos && lines[first].isEndSpeedFixed())
    nextPlannerIndex(first);
  // now first points to last segment before the end speed is fixed
  // so start speed is also fixed.

  if(first == linesWritePos) { // Nothing to plan, only new element present
    act->block(); // Prevent stepper interrupt from using this
    noInts.unprotect();
    act->setStartSpeedFixed(true);
    act->updateStepsParameter();
    act->unblock();
    return;
  }
  // now we have at least one additional move for optimization
  // that is not a wait move
  // First is now the new element or the first element with non fixed end speed.
  // anyhow, the start speed of first is fixed
  firstLine = &lines[first];
  firstLine->block(); // don't let printer touch this or following segments during update
  noInts.unprotect();
  uint8_t previousIndex = linesWritePos;
  previousPlannerIndex(previousIndex);
  PrintLine *previous = &lines[previousIndex]; // segment before the one we are inserting

  if(previous->isEOnlyMove() != act->isEOnlyMove()) {
    previous->maxJunctionSpeed = previous->endSpeed; // act->startSpeed; // maybe remove this. Previous should be at minimum and systems have nothing in common
    previous->setEndSpeedFixed(true);
    act->setStartSpeedFixed(true);
    act->updateStepsParameter();
    firstLine->unblock();
    return;
  } else {
    computeMaxJunctionSpeed(previous, act); // Set maximum junction speed if we have a real move before
  }
  // Increase speed if possible neglecting current speed
  backwardPlanner(linesWritePos, first);
  // Reduce speed to reachable speeds
  forwardPlanner(first);

#ifdef DEBUG_PLANNER
  if(Printer::debugEcho()) {
    Com::printF(PSTR("Planner: "), (int)linesCount);
    previousPlannerIndex(first);
    Com::printF(PSTR(" F "), lines[first].startSpeed, 1);
    Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
    Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
    Com::printF(PSTR(","), (int)lines[first].joinFlags);
    nextPlannerIndex(first);
  }
#endif
  // Update precomputed data
  do {
    lines[first].updateStepsParameter();
#ifdef DEBUG_PLANNER
    if(Printer::debugEcho()) {
      Com::printF(PSTR(" / "), lines[first].startSpeed, 1);
      Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
      Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
      Com::printF(PSTR(","), (int)lines[first].joinFlags);
#ifdef DEBUG_QUEUE_MOVE
      Com::print(PSTR("\n"));
#endif
    }
#endif
    //noInts.protect();
    lines[first].unblock();  // start with first block to release next used segment as early as possible
    nextPlannerIndex(first);
    lines[first].block();
    //noInts.unprotect();
  } while(first != linesWritePos);
  act->updateStepsParameter();
  act->unblock();
#ifdef DEBUG_PLANNER
  if(Printer::debugEcho()) {
    Com::printF(PSTR(" / "), lines[first].startSpeed, 1);
    Com::printF(PSTR(" - "), lines[first].endSpeed, 1);
    Com::printF(PSTR("("), lines[first].maxJunctionSpeed, 1);
    Com::printF(PSTR(","), (int)lines[first].joinFlags);
    Com::printF(PSTR("\n"));
  }
#endif
}

/* Computes the maximum junction speed of the newly added segment under
   optimal conditions. There is no guarantee that the previous move will be able to reach the
   speed at all, but if it could exceed it will never exceed this theoretical limit.

   if you define ALTERNATIVE_JERK the new jerk computations are used. These
   use the cosine of the angle and the maximum speed
   Jerk = (1-cos(alpha))*min(v1,v2)
   This sets jerk to 0 on zero angle change.

   Old               New
   0°:       0               0
   30°:     51,8             13.4
   45°:     76.53            29.3
   90°:    141               100
   180°:   200               200


   Speed from 100 to 200
   Old               New(min)   New(max)
   0°:     100               0          0
   30°:    123,9             13.4       26.8
   45°:    147.3             29.3       58.6
   90°:    223               100        200
   180°:   300               200        400

*/
inline void PrintLine::computeMaxJunctionSpeed(PrintLine *previous, PrintLine *current) {
#if USE_ADVANCE
  if(Printer::isAdvanceActivated()) {
    // if we start/stop extrusion we need to do so with lowest possible end speed
    // or advance would leave a drolling extruder and can not adjust fast enough.
    if(previous->isEMove() != current->isEMove()) {
      previous->setEndSpeedFixed(true);
      current->setStartSpeedFixed(true);
      previous->endSpeed = current->startSpeed = previous->maxJunctionSpeed = RMath::min(previous->endSpeed, current->startSpeed);
      previous->invalidateParameter();
      current->invalidateParameter();
      return;
    }
  }
#endif // USE_ADVANCE
  // if we are here we have to identical move types
  // either pure extrusion -> pure extrusion or
  // move -> move (with or without extrusion)
  // First we compute the normalized jerk for speed 1
  float factor = 1.0;
  float lengthFactor = 1.0;
#ifdef REDUCE_ON_SMALL_SEGMENTS
  if(previous->distance < MAX_JERK_DISTANCE)
    lengthFactor = static_cast<float>(MAX_JERK_DISTANCE * MAX_JERK_DISTANCE) / (previous->distance * previous->distance);
#endif
  float maxJoinSpeed = RMath::min(current->fullSpeed, previous->fullSpeed);

  // No point computing Z Jerk separately for delta moves
#ifdef ALTERNATIVE_JERK
  float jerk = maxJoinSpeed * lengthFactor * (1.0 - (current->speedX * previous->speedX + current->speedY * previous->speedY + current->speedZ * previous->speedZ) / (current->fullSpeed * previous->fullSpeed));
#else
  float dx = current->speedX - previous->speedX;
  float dy = current->speedY - previous->speedY;
  float dz = current->speedZ - previous->speedZ;
  float jerk = sqrt(dx * dx + dy * dy + dz * dz) * lengthFactor;
#endif // ALTERNATIVE_JERK

  if(jerk > Printer::maxJerk) {
    factor = Printer::maxJerk / jerk; // always < 1.0!
    if(factor * maxJoinSpeed * 2.0 < Printer::maxJerk)
      factor = Printer::maxJerk / (2.0 * maxJoinSpeed);
  }
  float eJerk = fabs(current->speedE - previous->speedE);
  if(eJerk > Extruder::current->maxStartFeedrate)
    factor = RMath::min(factor, Extruder::current->maxStartFeedrate / eJerk);

  previous->maxJunctionSpeed = maxJoinSpeed * factor; // set speed limit
#ifdef DEBUG_QUEUE_MOVE
  if(Printer::debugEcho()) {
    Com::printF(PSTR("ID:"), (int)previous);
    Com::printF(PSTR(" MJ:"), previous->maxJunctionSpeed);
    Com::printF(PSTR("\n"));
  }
#endif // DEBUG_QUEUE_MOVE
}

/** Update parameter used by updateTrapezoids

    Computes the acceleration/deceleration steps and advanced parameter associated.
*/
void PrintLine::updateStepsParameter() {
  if(areParameterUpToDate() || isWarmUp()) return;
  float startFactor = startSpeed * invFullSpeed;
  float endFactor   = endSpeed   * invFullSpeed;
  vStart = vMax * startFactor; //starting speed
  vEnd   = vMax * endFactor;

  uint32_t vmax2 = HAL::U16SquaredToU32(vMax);
  accelSteps = ((vmax2 - HAL::U16SquaredToU32(vStart)) / (accelerationPrim << 1)) + 1; // Always add 1 for missing precision
  decelSteps = ((vmax2 - HAL::U16SquaredToU32(vEnd))  / (accelerationPrim << 1)) + 1;

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  advanceStart = (float)advanceFull * startFactor * startFactor;
  advanceEnd   = (float)advanceFull * endFactor   * endFactor;
#endif
#endif
  if(static_cast<int32_t>(accelSteps + decelSteps) >= stepsRemaining) { // can't reach limit speed
    uint32_t red = (accelSteps + decelSteps - stepsRemaining) >> 1;
    accelSteps = accelSteps - RMath::min(static_cast<int32_t>(accelSteps), static_cast<int32_t>(red));
    decelSteps = decelSteps - RMath::min(static_cast<int32_t>(decelSteps), static_cast<int32_t>(red));
  }
  setParameterUpToDate();
#ifdef DEBUG_QUEUE_MOVE
  if(Printer::debugEcho()) {
    Com::printF(PSTR("ID:"), (int)this);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("vStart/End:"), (long)vStart);
    Com::printF(PSTR("/"), (long)vEnd);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("accel/decel steps:"), (long)accelSteps);
    Com::printF(PSTR("/"), (long)decelSteps);
    Com::printF(PSTR("/"), (long)stepsRemaining);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("st./end speed:"), startSpeed, 1);
    Com::printF(PSTR("/"), endSpeed, 1);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("Flags:"), (uint32_t)flags);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("joinFlags:"), (uint32_t)joinFlags);
    Com::printF(PSTR("\n"));
  }
#endif
}

/**
   Compute the maximum speed from the last entered move.
   The backwards planner traverses the moves from last to first looking at deceleration. The RHS of the accelerate/decelerate ramp.

   start = last line inserted
   last = last element until we check
*/
inline void PrintLine::backwardPlanner(uint8_t start, uint8_t last) {
  PrintLine *act = &lines[start], *previous;
  float lastJunctionSpeed = act->endSpeed; // Start always with safe speed

  //PREVIOUS_PLANNER_INDEX(last); // Last element is already fixed in start speed
  while(start != last) {
    previousPlannerIndex(start);
    previous = &lines[start];
    previous->block();
    // Avoid speed calculation once cruising in split delta move

    // Avoid speed calculations if we know we can accelerate within the line
    lastJunctionSpeed = (act->isNominalMove() ? act->fullSpeed : sqrt(lastJunctionSpeed * lastJunctionSpeed + act->accelerationDistance2)); // acceleration is acceleration*distance*2! What can be reached if we try?
    // If that speed is more that the maximum junction speed allowed then ...
    if(lastJunctionSpeed >= previous->maxJunctionSpeed) { // Limit is reached
      // If the previous line's end speed has not been updated to maximum speed then do it now
      if(previous->endSpeed != previous->maxJunctionSpeed) {
        previous->invalidateParameter(); // Needs recomputation
        previous->endSpeed = RMath::max(previous->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
      }
      // If actual line start speed has not been updated to maximum speed then do it now
      if(act->startSpeed != previous->maxJunctionSpeed) {
        act->startSpeed = RMath::max(act->minSpeed, previous->maxJunctionSpeed); // possibly unneeded???
        act->invalidateParameter();
      }
      lastJunctionSpeed = previous->endSpeed;
    } else {
      // Block previous end and act start as calculated speed and recalculate plateau speeds (which could move the speed higher again)
      act->startSpeed = RMath::max(act->minSpeed, lastJunctionSpeed);
      lastJunctionSpeed = previous->endSpeed = RMath::max(lastJunctionSpeed, previous->minSpeed);
      previous->invalidateParameter();
      act->invalidateParameter();
    }
    act = previous;
  } // while loop
}

void PrintLine::forwardPlanner(uint8_t first) {
  PrintLine *act;
  PrintLine *next = &lines[first];
  float vmaxRight;
  float leftSpeed = next->startSpeed;
  while(first != linesWritePos) { // All except last segment, which has fixed end speed
    act = next;
    nextPlannerIndex(first);
    next = &lines[first];
    // Avoid speed calculates if we know we can accelerate within the line.
    vmaxRight = (act->isNominalMove() ? act->fullSpeed : sqrt(leftSpeed * leftSpeed + act->accelerationDistance2));
    if(vmaxRight > act->endSpeed) { // Could be higher next run?
      if(leftSpeed < act->minSpeed) {
        leftSpeed = act->minSpeed;
        act->endSpeed = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
      }
      act->startSpeed = leftSpeed;
      next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
      if(act->endSpeed == act->maxJunctionSpeed) { // Full speed reached, don't compute again!
        act->setEndSpeedFixed(true);
        next->setStartSpeedFixed(true);
      }
      act->invalidateParameter();
    } else { // We can accelerate full speed without reaching limit, which is as fast as possible. Fix it!
      act->fixStartAndEndSpeed();
      act->invalidateParameter();
      if(act->minSpeed > leftSpeed) {
        leftSpeed = act->minSpeed;
        vmaxRight = sqrt(leftSpeed * leftSpeed + act->accelerationDistance2);
      }
      act->startSpeed = leftSpeed;
      act->endSpeed = RMath::max(act->minSpeed, vmaxRight);
      next->startSpeed = leftSpeed = RMath::max(RMath::min(act->endSpeed, act->maxJunctionSpeed), next->minSpeed);
      next->setStartSpeedFixed(true);
    }
  } // While
  next->startSpeed = RMath::max(next->minSpeed, leftSpeed); // This is the new segment, which is updated anyway, no extra flag needed.
}


inline float PrintLine::safeSpeed(int8_t drivingAxis) {
	float xyMin = Printer::maxJerk * 0.5;
	float mz = 0;
  float safe(xyMin);
  if(isEMove()) {
    if(isXYZMove())
      safe = RMath::min(safe, (float)(0.5 * Extruder::current->maxStartFeedrate * fullSpeed / fabs(speedE)));
    else
      safe = 0.5 * Extruder::current->maxStartFeedrate; // This is a retraction move
  }
  // Check for minimum speeds needed for numerical robustness
  if(drivingAxis == X_AXIS || drivingAxis == Y_AXIS || drivingAxis == Z_AXIS) // enforce minimum speed for numerical stability of explicit speed integration
    safe = RMath::max(xyMin, safe);
  return RMath::min(safe, fullSpeed);
}


/** Check if move is new. If it is insert some dummy moves to allow the path optimizer to work since it does
    not act on the first two moves in the queue. The stepper timer will spot these moves and leave some time for
    processing.
*/
uint8_t PrintLine::insertWaitMovesIfNeeded(uint8_t pathOptimize, uint8_t waitExtraLines) {
  if(linesCount == 0 && waitRelax == 0 && pathOptimize) { // First line after some time - warm up needed
    uint8_t w = 3;
    while(w--) {
      PrintLine *p = getNextWriteLine();
      p->flags = FLAG_WARMUP;
      p->joinFlags = FLAG_JOIN_STEPPARAMS_COMPUTED | FLAG_JOIN_END_FIXED | FLAG_JOIN_START_FIXED;
      p->dir = 0;
      p->setWaitForXLinesFilled(w + waitExtraLines);
      p->setWaitTicks(300000);
      p->moveID = lastMoveID++;
      pushLine();
    }
    //Com::printF(PSTR("InsertWait"));
    //Com::printF(PSTR("\n"));
    return 1;
  }
  return 0;
}

void PrintLine::logLine() {
#ifdef DEBUG_QUEUE_MOVE
  Com::printF(PSTR("ID:"), (int)this);
  Com::printF(PSTR("\n"));
  Com::printArrayF(PSTR("Delta"), delta, 4);
  Com::printF(PSTR("Dir:"), (uint32_t)dir);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Flags:"), (uint32_t)flags);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("fullSpeed:"), fullSpeed);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("vMax:"), (int32_t)vMax);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Acceleration:"), accelerationDistance2);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Acceleration Prim:"), (int32_t)accelerationPrim);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Remaining steps:"), stepsRemaining);
  Com::printF(PSTR("\n"));
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  Com::printF(PSTR("advanceFull:"), advanceFull >> 16);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("advanceRate:"), advanceRate);
  Com::printF(PSTR("\n"));
#endif
#endif
#endif // DEBUG_QUEUE_MOVE
}

void PrintLine::waitForXFreeLines(uint8_t b, bool allowMoves) {
  while(getLinesCount() + b > PRINTLINE_CACHE_SIZE) { // wait for a free entry in movement cache
    //GCode::readFromSerial();
    Commands::checkForPeriodicalActions(allowMoves);
  }

  //  This was at the end of Commands.cpp processG001(), but it seems here is better.
#ifdef DEBUG_QUEUE_MOVE
  InterruptProtectedBlock noInts;

  int lc = (int)PrintLine::linesCount;
  int lp = (int)PrintLine::linesPos;
  int wp = (int)PrintLine::linesWritePos;

  int n = (wp - lp);

  if(n < 0) n += PRINTLINE_CACHE_SIZE;

  noInts.unprotect();

  if(n != lc)
    Com::printF(PSTR("Buffer corrupted\n"));
#endif
}



/**
   Calculate the delta tower position from a Cartesian position
   @param cartesianPosSteps Array containing Cartesian coordinates.
   @param deltaPosSteps Result array with tower coordinates.
   @returns 1 if Cartesian coordinates have a valid delta tower position 0 if not.
*/

inline
unsigned long
absLong(long a) {
  return a >= 0 ? a : -a;
}


uint8_t transformCartesianStepsToDeltaSteps(int32_t cartesianPosSteps[], int32_t deltaPosSteps[]) {
  int32_t zSteps = cartesianPosSteps[Z_AXIS];

#if DISTORTION_CORRECTION
  static int cnt = 0;
  static int32_t lastZSteps = 9999999;
  static int32_t lastZCorrection = 0;
  cnt++;
  if(cnt >= DISTORTION_UPDATE_FREQUENCY || lastZSteps != zSteps) {
    cnt = 0;
    lastZSteps = zSteps;
    lastZCorrection = Printer::distortion.correct(cartesianPosSteps[X_AXIS], cartesianPosSteps[Y_AXIS], cartesianPosSteps[Z_AXIS]);
  }
  zSteps += lastZCorrection;
#endif

  if(Printer::isLargeMachine()) {
    float temp = Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS];
    float opt = Printer::deltaDiagonalStepsSquaredA.f - temp * temp;
    float temp2 = Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS];
    if ((temp = opt - temp2 * temp2) >= 0)
      deltaPosSteps[A_TOWER] = floor(0.5 + sqrt(temp) + zSteps);
    else
      return 0;
    if (deltaPosSteps[A_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

    temp = Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS];
    opt = Printer::deltaDiagonalStepsSquaredB.f - temp * temp;
    temp2 = Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS];
    if ((temp = opt - temp2 * temp2) >= 0)
      deltaPosSteps[B_TOWER] = floor(0.5 + sqrt(temp) + zSteps);
    else
      return 0;
    if (deltaPosSteps[B_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

    temp = Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS];
    opt = Printer::deltaDiagonalStepsSquaredC.f - temp * temp;
    temp2 = Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS];
    if ((temp = opt - temp2 * temp2) >= 0)
      deltaPosSteps[C_TOWER] = floor(0.5 + sqrt(temp) + zSteps);
    else
      return 0;
    if (deltaPosSteps[C_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) return 0;

    return 1;

  } else {
    // As we are right on the edge of many printers arm lengths, this is rewritten to use unsigned long
    // This allows 52% longer arms to be used without performance penalty
    // the code is a bit longer, because we cannot use negative to test for invalid conditions
    // Also, previous code did not check for overflow of squared result
    // Overflow is also detected as a fault condition

    const uint32_t LIMIT = 65534; // Largest squarable int without overflow;

    // A TOWER height
    uint32_t temp = absLong(Printer::deltaAPosYSteps - cartesianPosSteps[Y_AXIS]);
    uint32_t opt = Printer::deltaDiagonalStepsSquaredA.l;

    if (temp > LIMIT) {
      Com::printF(PSTR("Apos y steps\n"));
      return(0);
    }

    temp *= temp;

    if (opt < temp) {
      Com::printF(PSTR("Apos y square\n"));
      return(0);
    }

    opt -= temp;

    temp = absLong(Printer::deltaAPosXSteps - cartesianPosSteps[X_AXIS]);

    if (temp > LIMIT) {
      Com::printF(PSTR("Apos x steps\n"));
      return(0);
    }

    temp *= temp;

    if (opt < temp) {
      Com::printF(PSTR("Apos x square\n"));
      return(0);
    }

    deltaPosSteps[A_TOWER] = SQRT(opt - temp) + zSteps;

    if (deltaPosSteps[A_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) {
      Com::printF(PSTR("A hit floor\n"));
      return(0);
    }

    // B TOWER height
    temp = absLong(Printer::deltaBPosYSteps - cartesianPosSteps[Y_AXIS]);
    opt = Printer::deltaDiagonalStepsSquaredB.l;

    if (temp > LIMIT) {
      Com::printF(PSTR("Bpos y steps\n"));
      return(0);
    }

    temp *= temp;

    if (opt < temp) {
      Com::printF(PSTR("Bpos y square\n"));
      return(0);
    }

    opt -= temp;

    temp = absLong(Printer::deltaBPosXSteps - cartesianPosSteps[X_AXIS]);

    if (temp > LIMIT ) {
      Com::printF(PSTR("Bpos x steps\n"));
      return(0);
    }
    temp *= temp;

    if (opt < temp) {
      Com::printF(PSTR("Bpos x square\n"));
      return(0);
    }

    deltaPosSteps[B_TOWER] = SQRT(opt - temp) + zSteps ;

    if (deltaPosSteps[B_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) {
      Com::printF(PSTR("B hit floor\n"));
      return(0);
    }

    // C TOWER height
    temp = absLong(Printer::deltaCPosYSteps - cartesianPosSteps[Y_AXIS]);
    opt = Printer::deltaDiagonalStepsSquaredC.l ;

    if (temp > LIMIT) {
      Com::printF(PSTR("Cpos y steps\n"));
      return(0);
    }

    temp = temp * temp;

    if ( opt < temp ) {
      Com::printF(PSTR("Cpos y square\n"));
      return(0);
    }

    opt -= temp;

    temp = absLong(Printer::deltaCPosXSteps - cartesianPosSteps[X_AXIS]);

    if (temp > LIMIT) {
      Com::printF(PSTR("Cpos x steps\n"));
      return(0);
    }

    temp = temp * temp;

    if ( opt < temp ) {
      Com::printF(PSTR("Cpos x square\n"));
      return(0);
    }

    deltaPosSteps[C_TOWER] = SQRT(opt - temp) + zSteps;

    if (deltaPosSteps[C_TOWER] < Printer::deltaFloorSafetyMarginSteps && !Printer::isZProbingActive()) {
      Com::printF(PSTR("C hit floor\n"));
      return(0);
    }
  }
  return 1;
}





bool NonlinearSegment::checkEndstops(PrintLine *cur, bool checkall) {

  if (Printer::isZProbingActive()) {
    endstops.update();

    if(isZPositiveMove() && isXPositiveMove() && isYPositiveMove() && endstops.anyXYZMax()) {
        cur->setXMoveFinished();
        cur->setYMoveFinished();
        cur->setZMoveFinished();
        //dir = 0;
        Printer::stepsRemainingAtZHit = cur->stepsRemaining;
        return true;
      }

  } else if(checkall) {
    endstops.update(); // do not test twice
    if(!endstops.anyXYZ()) // very quick check for the normal case
      return false;
  }

  int8_t r = 0;

  if(checkall) {
    // endstops are per motor and do not depend on global axis movement
    if(isXPositiveMove() && endstops.xMax()) {
      if(Printer::stepsRemainingAtXHit < 0)
        Printer::stepsRemainingAtXHit = cur->stepsRemaining;
      setXMoveFinished();
      cur->setXMoveFinished();
      r++;
    }
    if(isYPositiveMove() && endstops.yMax()) {
      if(Printer::stepsRemainingAtYHit < 0)
        Printer::stepsRemainingAtYHit = cur->stepsRemaining;
      setYMoveFinished();
      cur->setYMoveFinished();
      r++;
    }
    if(isZPositiveMove() && endstops.zMax()) {
      if(Printer::stepsRemainingAtZHit)
        Printer::stepsRemainingAtZHit = cur->stepsRemaining;
      setZMoveFinished();
      cur->setZMoveFinished();
      r++;
    }
    if(isZNegativeMove() && endstops.zMin()) {
      setZMoveFinished();
      cur->setZMoveFinished();
      r++;
    }
    if(Printer::isHoming())
      return r == 3;
  }
  return r != 0;
}

void PrintLine::calculateDirectionAndDelta(int32_t difference[], uint8_t *dir, int32_t delta[]) {
  *dir = 0;
  //Find direction
  if(difference[X_AXIS] != 0) {
    if(difference[X_AXIS] < 0) {
      delta[X_AXIS] = -difference[X_AXIS];
      *dir |= XSTEP;
    } else {
      delta[X_AXIS] = difference[X_AXIS];
      *dir |= X_DIRPOS + XSTEP;
    }
  } else {
    delta[X_AXIS] = 0;
  }

  if(difference[Y_AXIS] != 0) {
    if(difference[Y_AXIS] < 0) {
      delta[Y_AXIS] = -difference[Y_AXIS];
      *dir |= YSTEP;
    } else {
      delta[Y_AXIS] = difference[Y_AXIS];
      *dir |= Y_DIRPOS + YSTEP;
    }
  } else {
    delta[Y_AXIS] = 0;
  }
  if(difference[Z_AXIS] != 0) {
    if(difference[Z_AXIS] < 0) {
      delta[Z_AXIS] = -difference[Z_AXIS];
      *dir |= ZSTEP;
    } else {
      delta[Z_AXIS] = difference[Z_AXIS];
      *dir |= Z_DIRPOS + ZSTEP;
    }
  } else {
    delta[Z_AXIS] = 0;
  }
  if(difference[E_AXIS] != 0) {
    if(difference[E_AXIS] < 0) {
      delta[E_AXIS] = -difference[E_AXIS];
      *dir |= ESTEP;
    } else {
      delta[E_AXIS] = difference[E_AXIS];
      *dir |= E_DIRPOS + ESTEP;
    }
  } else {
    delta[E_AXIS] = 0;
  }
}
/**
   Calculate and cache the delta robot positions of the Cartesian move in a line.
   @return The largest delta axis move in a single segment
   @param p The line to examine.
*/
inline uint16_t PrintLine::calculateNonlinearSubSegments(uint8_t softEndstop) {
  int8_t i;
  int32_t delta, diff;
  int32_t destinationSteps[Z_AXIS_ARRAY], destinationDeltaSteps[TOWER_ARRAY];
  // Save current position

#if !EXACT_DELTA_MOVES
  for(uint8_t i = 0; i < Z_AXIS_ARRAY; i++)
    destinationSteps[i] = Printer::currentPositionSteps[i];
#else
  float dx[Z_AXIS_ARRAY];
  for(int i = 0; i < Z_AXIS_ARRAY; i++)
    dx[i] = static_cast<float>(Printer::destinationSteps[i] - Printer::currentPositionSteps[i]) / static_cast<float>(numNonlinearSegments);
#endif

  //  out.print_byte_P(PSTR("Calculate delta segments:"), p->numDeltaSegments);
#ifdef DEBUG_STEPCOUNT
  totalStepsRemaining = 0;
#endif

  uint16_t maxAxisSteps = 0;
  for (int s = numNonlinearSegments; s > 0; s--) {
    NonlinearSegment *d = &segments[s - 1];

#if !EXACT_DELTA_MOVES
    for(i = 0; i < Z_AXIS_ARRAY; i++) {
      // End of segment in Cartesian steps

      // This method generates small waves which get larger with increasing number of delta segments. smaller?
      diff = Printer::destinationSteps[i] - destinationSteps[i];
      if(s == 1)
        destinationSteps[i] += diff;
      else if(s == 2)
        destinationSteps[i] += (diff >> 1);
      else if(s == 4)
        destinationSteps[i] += (diff >> 2);
      else if(diff < 0)
        destinationSteps[i] -= HAL::Div4U2U(-diff, s);
      else
        destinationSteps[i] += HAL::Div4U2U(diff, s);
    }
#else
    float segment = static_cast<float>(numNonlinearSegments - s + 1);
    for(i = 0; i < Z_AXIS_ARRAY; i++) // End of segment in Cartesian steps
      // Perfect approximation, but slower, so we limit it to faster processors like arm
      destinationSteps[i] = static_cast<int32_t>(floor(0.5 + dx[i] * segment)) + Printer::currentPositionSteps[i];
#endif
    // Verify that delta calculation has a solution
    if (transformCartesianStepsToDeltaSteps(destinationSteps, destinationDeltaSteps)) {
      d->dir = 0;
      if (softEndstop) {
        destinationDeltaSteps[A_TOWER] = RMath::min(destinationDeltaSteps[A_TOWER], Printer::maxDeltaPositionSteps);
        destinationDeltaSteps[B_TOWER] = RMath::min(destinationDeltaSteps[B_TOWER], Printer::maxDeltaPositionSteps);
        destinationDeltaSteps[C_TOWER] = RMath::min(destinationDeltaSteps[C_TOWER], Printer::maxDeltaPositionSteps);
      }
      for(i = 0; i < TOWER_ARRAY; i++) {
        delta = destinationDeltaSteps[i] - Printer::currentNonlinearPositionSteps[i];
        if (delta > 0) {
          d->setPositiveMoveOfAxis(i);
#ifdef DEBUG_DELTA_OVERFLOW
          if (delta > 65535) {
            Com::printF(PSTR("Delta overflow:"), delta);
            Com::printF(PSTR("\n"));
          }
#endif
          d->deltaSteps[i] = static_cast<uint16_t>(delta);
        } else {
          d->setMoveOfAxis(i);
#ifdef DEBUG_DELTA_OVERFLOW
          if (-delta > 65535) {
            Com::printF(PSTR("Delta overflow:"), delta);
            Com::printF(PSTR("\n"));
          }
#endif
          d->deltaSteps[i] = static_cast<uint16_t>(-delta);
        }
#ifdef DEBUG_STEPCOUNT
        totalStepsRemaining += d->deltaSteps[i];
#endif
        if(d->deltaSteps[i] > maxAxisSteps)
          maxAxisSteps = d->deltaSteps[i];
        Printer::currentNonlinearPositionSteps[i] = destinationDeltaSteps[i];
      }
    } else {
      // Illegal position - ignore move
      Com::printF(PSTR("WARNING: Invalid delta coordinate - move ignored"));
      Com::printF(PSTR(" x:"), destinationSteps[X_AXIS]);
      Com::printF(PSTR(" y:"), destinationSteps[Y_AXIS]);
      Com::printF(PSTR(" z:"), destinationSteps[Z_AXIS]);
      Com::printF(PSTR("\n"));
      d->dir = 0;
      d->deltaSteps[A_TOWER] = d->deltaSteps[B_TOWER] = d->deltaSteps[C_TOWER] = 0;
      return 65535; // flag error
    }
  }
#ifdef DEBUG_STEPCOUNT
  //      out.print_long_P(PSTR("initial StepsRemaining:"), p->totalStepsRemaining);
#endif
  return maxAxisSteps;
}

uint8_t PrintLine::calculateDistance(float axisDistanceMM[], uint8_t dir, float *distance) {
  // Calculate distance depending on direction
  if(dir & XYZ_STEP) {
    if(dir & XSTEP)
      *distance = axisDistanceMM[X_AXIS] * axisDistanceMM[X_AXIS];
    else
      *distance = 0;
    if(dir & YSTEP)
      *distance += axisDistanceMM[Y_AXIS] * axisDistanceMM[Y_AXIS];
    if(dir & ZSTEP)
      *distance += axisDistanceMM[Z_AXIS] * axisDistanceMM[Z_AXIS];
    *distance = RMath::max((float)sqrt(*distance), axisDistanceMM[E_AXIS]);
    return 1;
  } else {
    if(dir & ESTEP) {
      *distance = axisDistanceMM[E_AXIS];
      return 1;
    }
    *distance = 0;
    return 0;
  }
}

#if FEATURE_SOFTWARE_LEVELING
void PrintLine::calculatePlane(int32_t factors[], int32_t p1[], int32_t p2[], int32_t p3[]) {
  factors[0] = p1[1] * (p2[2] - p3[2]) + p2[1] * (p3[2] - p1[2]) + p3[1] * (p1[2] - p2[2]);
  factors[1] = p1[2] * (p2[0] - p3[0]) + p2[2] * (p3[0] - p1[0]) + p3[2] * (p1[0] - p2[0]);
  factors[2] = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]);
  factors[3] = p1[0] * ((p2[1] * p3[2]) - (p3[1] * p2[2])) + p2[0] * ((p3[1] * p1[2]) - (p1[1] * p3[2])) + p3[0] * ((p1[1] * p2[2]) - (p2[1] * p1[2]));
}

float PrintLine::calcZOffset(int32_t factors[], int32_t pointX, int32_t pointY) {
  return (factors[3] - factors[X_AXIS] * pointX - factors[Y_AXIS] * pointY) / (float) factors[2];
}
#endif

inline void PrintLine::queueEMove(int32_t extrudeDiff, uint8_t check_endstops, uint8_t pathOptimize) {
  Printer::unsetAllSteppersDisabled();
  waitForXFreeLines(1);
  uint8_t newPath = insertWaitMovesIfNeeded(pathOptimize, 1);
  PrintLine *p = getNextWriteLine();
  float axisDistanceMM[VIRTUAL_AXIS_ARRAY]; // Axis movement in mm
  if(check_endstops) p->flags = FLAG_CHECK_ENDSTOPS;
  else p->flags = 0;
  p->joinFlags = 0;
  if(!pathOptimize) p->setEndSpeedFixed(true);
  //Find direction
  for(uint8_t i = 0; i < Z_AXIS_ARRAY; i++) {
    p->delta[i] = 0;
    axisDistanceMM[i] = 0;
  }
  if (extrudeDiff >= 0) {
    p->delta[E_AXIS] = extrudeDiff;
    p->dir = E_STEP_DIRPOS;
  } else {
    p->delta[E_AXIS] = -extrudeDiff;
    p->dir = ESTEP;
  }
  Printer::currentPositionSteps[E_AXIS] = Printer::destinationSteps[E_AXIS];

  p->numNonlinearSegments = 0;
  //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  p->primaryAxis = E_AXIS;
  p->stepsRemaining = p->delta[E_AXIS];
  axisDistanceMM[E_AXIS] = p->distance = p->delta[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];
  axisDistanceMM[VIRTUAL_AXIS] = -p->distance;
  p->moveID = lastMoveID++;
  p->calculateMove(axisDistanceMM, pathOptimize, E_AXIS);
}

/**
   Split a line up into a series of lines with at most DELTASEGMENTS_PER_PRINTLINE delta segments.
   @param check_endstops Check endstops during the move.
   @param pathOptimize Run the path optimizer.
   @param softEndstop check if we go out of bounds.
*/
uint8_t PrintLine::queueNonlinearMove(uint8_t check_endstops, uint8_t pathOptimize, uint8_t softEndstop) {
  //if (softEndstop && Printer::destinationSteps[Z_AXIS] < 0) Printer::destinationSteps[Z_AXIS] = 0; // now constrained at entry level including cylinder test
  int32_t difference[E_AXIS_ARRAY];
  float axisDistanceMM[VIRTUAL_AXIS_ARRAY]; // Real cartesian axis movement in mm. Virtual axis in 4;
  uint8_t secondSpeed = Printer::fanSpeed;
  for(int8_t axis = 0; axis < E_AXIS_ARRAY; axis++) {
    difference[axis] = Printer::destinationSteps[axis] - Printer::currentPositionSteps[axis];
    if(axis == E_AXIS) {
      if(Printer::mode == PRINTER_MODE_FFF) {
        Printer::extrudeMultiplyError += (static_cast<float>(difference[E_AXIS]) * Printer::extrusionFactor);
        difference[E_AXIS] = static_cast<int32_t>(Printer::extrudeMultiplyError);
        Printer::extrudeMultiplyError -= difference[E_AXIS];
        axisDistanceMM[E_AXIS] = difference[E_AXIS] * Printer::invAxisStepsPerMM[E_AXIS];
        Printer::filamentPrinted += axisDistanceMM[E_AXIS];
        axisDistanceMM[E_AXIS] = fabs(axisDistanceMM[E_AXIS]);
      }
    } else
      axisDistanceMM[axis] = fabs(difference[axis] * Printer::invAxisStepsPerMM[axis]);
  }

  float cartesianDistance;
  uint8_t cartesianDir;
  int32_t cartesianDeltaSteps[E_AXIS_ARRAY];
  calculateDirectionAndDelta(difference, &cartesianDir, cartesianDeltaSteps);
  if (!calculateDistance(axisDistanceMM, cartesianDir, &cartesianDistance)) {
    // Appears the intent is to do nothing if no distance is detected.
    // This apparently is not an error condition, just early exit.
    return true;
  }

  if (!(cartesianDir & XYZ_STEP)) {
    queueEMove(difference[E_AXIS], check_endstops, pathOptimize);
    return true;
  }

  int16_t segmentCount;
  float feedrate = RMath::min(Printer::feedrate, Printer::maxFeedrate[Z_AXIS]);
  if (cartesianDir & XY_STEP) {
    // Compute number of seconds for move and hence number of segments needed
    //float seconds = 100 * cartesianDistance / (Printer::feedrate * Printer::feedrateMultiply); multiply in feedrate included
    float seconds = cartesianDistance / feedrate;
#ifdef DEBUG_SPLIT
    Com::printF(PSTR("Seconds:"), seconds);
    Com::printF(PSTR("\n"));
#endif
    float sps = static_cast<float>((cartesianDir & ESTEP) == ESTEP ? Printer::printMovesPerSecond : Printer::travelMovesPerSecond);
    segmentCount = RMath::max(1, static_cast<int16_t>(sps * seconds));
#ifdef DEBUG_SEGMENT_LENGTH
    float segDist = cartesianDistance / (float)segmentCount;
    if(segDist > Printer::maxRealSegmentLength) {
      Printer::maxRealSegmentLength = segDist;
      Com::printF(PSTR("SegmentsPerSecond:"), sps);
      Com::printF(PSTR("\n"));
      Com::printF(PSTR("New max. segment length:"), segDist);
      Com::printF(PSTR("\n"));
    }
#endif
    //Com::printF(PSTR("Segments:"),segmentCount);
    //Com::printF(PSTR("\n"));
  } else {
    // Optimize pure Z axis move. Since a pure Z axis move is linear all we have to watch out for is unsigned integer overruns in
    // the queued moves;
#ifdef DEBUG_SPLIT
    Com::printF(PSTR("Z delta:"), cartesianDeltaSteps[Z_AXIS]);
    Com::printF(PSTR("\n"));
#endif
    segmentCount = (cartesianDeltaSteps[Z_AXIS] + (uint32_t)43680) / (uint32_t)43679; // can not go to 65535 for rounding issues causing overflow later in some cases!
  }
  // Now compute the number of lines needed
  int numLines = (segmentCount + DELTASEGMENTS_PER_PRINTLINE - 1) / DELTASEGMENTS_PER_PRINTLINE;
  // There could be some error here but it doesn't matter since the number of segments will just be reduced slightly
  int segmentsPerLine = segmentCount / numLines;

  int32_t startPosition[E_AXIS_ARRAY], fractionalSteps[E_AXIS_ARRAY];
  if(numLines > 1) {
    for (int8_t i = 0; i < Z_AXIS_ARRAY; i++)
      startPosition[i] = Printer::currentPositionSteps[i];
    startPosition[E_AXIS] = 0;
    cartesianDistance /= static_cast<float>(numLines);
  }

#ifdef DEBUG_SPLIT
  Com::printF(PSTR("Segments:"), segmentCount);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("Num lines:"), numLines);
  Com::printF(PSTR("\n"));
  Com::printF(PSTR("segments_per_line:"), segmentsPerLine);
  Com::printF(PSTR("\n"));
#endif
  Printer::unsetAllSteppersDisabled(); // Motor is enabled now
  waitForXFreeLines(1);

  // Insert dummy moves if necessary
  // Need to leave at least one slot open for the first split move
  insertWaitMovesIfNeeded(pathOptimize, RMath::min(PRINTLINE_CACHE_SIZE - 4, numLines));
  uint32_t oldEDestination = Printer::destinationSteps[E_AXIS]; // flow and volumetric extrusion changed virtual target
  Printer::currentPositionSteps[E_AXIS] = 0;

  for (int lineNumber = 1; lineNumber <= numLines; lineNumber++) {
    waitForXFreeLines(1);
    PrintLine *p = getNextWriteLine();
    // Downside a comparison per loop. Upside one less distance calculation and simpler code.
    if (numLines == 1) {
      // p->numDeltaSegments = segmentCount; // not neede, gets overwritten further down
      p->dir = cartesianDir;
      for (int8_t i = 0; i < E_AXIS_ARRAY; i++) {
        p->delta[i] = cartesianDeltaSteps[i];
        fractionalSteps[i] = difference[i];
      }
      p->distance = cartesianDistance;
    } else {
      for (int8_t i = 0; i < E_AXIS_ARRAY; i++) {
        Printer::destinationSteps[i] = startPosition[i] + (difference[i] * lineNumber) / numLines;
        fractionalSteps[i] = Printer::destinationSteps[i] - Printer::currentPositionSteps[i];
        axisDistanceMM[i] = fabs(fractionalSteps[i] * Printer::invAxisStepsPerMM[i]);
      }
      calculateDirectionAndDelta(fractionalSteps, &p->dir, p->delta);
      p->distance = cartesianDistance;
    }

    p->joinFlags = 0;
    p->secondSpeed = secondSpeed;
    p->moveID = lastMoveID;

    // Only set fixed on last segment
    if (lineNumber == numLines && !pathOptimize)
      p->setEndSpeedFixed(true);

    p->flags = (check_endstops ? FLAG_CHECK_ENDSTOPS : 0);
    p->numNonlinearSegments = segmentsPerLine;

    uint16_t maxStepsPerSegment = p->calculateNonlinearSubSegments(softEndstop);
    if (maxStepsPerSegment == 65535) {
      Com::printF(PSTR("WARNING: in queueDeltaMove to calculateDeltaSubSegments returns error.\n"));
      return false;
    }
#ifdef DEBUG_SPLIT
    Com::printF(PSTR("Max DS:"), (int32_t)maxStepsPerSegment);
    Com::printF(PSTR("\n"));
#endif
    int32_t virtualAxisSteps = static_cast<int32_t>(maxStepsPerSegment) * segmentsPerLine;
    if (virtualAxisSteps == 0 && p->delta[E_AXIS] == 0) {
      if (numLines != 1) {
        Com::printF(PSTR("ERROR: No move in delta segment with > 1 segment. This should never happen and may cause a problem!\n"));
        return false;  // Line too short in low precision area
      }
    }
    int8_t drivingAxis = X_AXIS;
    p->primaryAxis = VIRTUAL_AXIS; // Virtual axis will lead Bresenham step either way
    if (virtualAxisSteps > p->delta[E_AXIS]) { // Is delta move or E axis leading
      p->stepsRemaining = virtualAxisSteps;
      axisDistanceMM[VIRTUAL_AXIS] = p->distance;  //virtual_axis_move * Printer::invAxisStepsPerMM[Z_AXIS]; // Steps/unit same as all the towers
      // Virtual axis steps per segment
      p->numPrimaryStepPerSegment = maxStepsPerSegment;
    } else {
      // Round up the E move to get something divisible by segment count which is greater than E move
      p->numPrimaryStepPerSegment = (p->delta[E_AXIS] + segmentsPerLine - 1) / segmentsPerLine;
      p->stepsRemaining = p->numPrimaryStepPerSegment * segmentsPerLine;
      axisDistanceMM[VIRTUAL_AXIS] = -p->distance; //p->stepsRemaining * Printer::invAxisStepsPerMM[Z_AXIS];
      drivingAxis = E_AXIS;
    }
#ifdef DEBUG_SPLIT
    Com::printF(PSTR("Steps Per Segment:"), p->numPrimaryStepPerSegment);
    Com::printF(PSTR("\n"));
    Com::printF(PSTR("Virtual axis steps:"), p->stepsRemaining);
    Com::printF(PSTR("\n"));
#endif
    p->calculateMove(axisDistanceMM, pathOptimize, drivingAxis);
    for (int8_t i = 0; i < E_AXIS_ARRAY; i++) {
      Printer::currentPositionSteps[i] += fractionalSteps[i];
    }
  }
  Printer::currentPositionSteps[E_AXIS] = Printer::destinationSteps[E_AXIS] = oldEDestination;
  lastMoveID++; // Will wrap at 255

  return true; // flag success
}



/**
   Moves the stepper motors one step. If the last step is reached, the next movement is started.
   The function must be called from a timer loop. It returns the time for the next call.
   This is a modified version that implements a Bresenham 'multi-step' algorithm where the dominant
   Cartesian axis steps may be less than the changing dominant delta axis.
*/


int lastblk = - 1;
int32_t cur_errupd;
// Current nonlinear segment
NonlinearSegment *curd;
// Current nonlinear segment primary error increment
int32_t curd_errupd, stepsPerSegRemaining;
int32_t PrintLine::bresenhamStep() { // Version for delta printer
  if(cur == NULL)
    {
      setCurrentLine();
      if(cur->isBlocked()) { // This step is in computation - shouldn't happen
        if(lastblk != (int)cur) {
          HAL::allowInterrupts();
          lastblk = (int)cur;
          Com::printF(PSTR("BLK "), (int32_t)linesCount);
          Com::printF(PSTR("\n"));
        }
        cur = NULL;
        return 2000;
      }
      HAL::allowInterrupts();
      lastblk = -1;

#if 1
      // Allows M111 so set bit 6 (32) which disables moves, at the first tried
      // step. In combination with a dry run, you can test the speed of path
      // computations, which are still performed.

      if(Printer::debugNoMoves()) { // simulate a move, but do nothing in reality
        removeCurrentLineForbidInterrupt();
        if(linesCount == 0) {
          uid.setStatusP(PSTR("Idle"));
          uid.refreshPage();
        }
        return 1000;
      }
#endif

      if(cur->isWarmUp()) {
        // This is a warm up move to initialize the path planner correctly. Just waste
        // a bit of time to get the planning up to date.
        if(linesCount <= cur->getWaitForXLinesFilled()) {
          cur = NULL;
          return 2000;
        }
        long wait = cur->getWaitTicks();
        removeCurrentLineForbidInterrupt();
        return(wait); // waste some time for path optimization to fill up
      } // End if WARMUP

      if(cur->isEMove()) {
        Extruder::enable();
      }
      cur->fixStartAndEndSpeed();
      // Set up delta segments
      if (cur->numNonlinearSegments) {

        // If there are delta segments point to them here
        curd = &cur->segments[--cur->numNonlinearSegments];
        // Enable axis - All axis are enabled since they will most probably all be involved in a move
        // Since segments could involve different axis this reduces load when switching segments and
        // makes disabling easier.
        Printer::enableXStepper();
        Printer::enableYStepper();
        Printer::enableZStepper();
        Printer::setXDirection(curd->isXPositiveMove());
        Printer::setYDirection(curd->isYPositiveMove());
        Printer::setZDirection(curd->isZPositiveMove());

        // Copy across movement into main direction flags so that endstops function correctly
        cur->dir |= curd->dir; // deltas need this for homing!
        // Initialize Bresenham for the first segment
        cur->error[X_AXIS] = cur->error[Y_AXIS] = cur->error[Z_AXIS] = cur->numPrimaryStepPerSegment >> 1;
        curd_errupd = cur->numPrimaryStepPerSegment;
        stepsPerSegRemaining = cur->numPrimaryStepPerSegment;
      } else curd = NULL;
      cur_errupd = cur->stepsRemaining;

      if(!cur->areParameterUpToDate()) { // should never happen, but with bad timings???
        cur->updateStepsParameter();
      }
      Printer::vMaxReached = cur->vStart;
      Printer::stepNumber = 0;
      Printer::timer = 0;
      HAL::forbidInterrupts();
#if USE_ADVANCE
      if(!Printer::isAdvanceActivated()) // Set direction if no advance/OPS enabled
#endif
        Extruder::setDirection(cur->isEPositiveMove());
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
      // HAL::delayMicroseconds(DIRECTION_DELAY); // We leave interrupt without step so no delay needed here
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
      Printer::advanceExecuted = cur->advanceStart;
#endif
      cur->updateAdvanceSteps(cur->vStart, 0, false);
#endif
      if(Printer::mode == PRINTER_MODE_FFF) {
        Printer::setFanSpeedDirectly(cur->secondSpeed);
      }
      return Printer::interval; // Wait an other 50% from last step to make the 100% full
    } // End cur=0
  HAL::allowInterrupts();

  if(curd != NULL) {
    if(curd->checkEndstops(cur, (cur->isCheckEndstops()))) { // should stop move
      cur->stepsRemaining = 0;
      curd = NULL;
      // eat up all following segments with moveID
      uint8_t delId = cur->moveID;
      removeCurrentLineForbidInterrupt();
      while(linesCount > 0) {
        setCurrentLine();
        if(cur->isBlocked() || cur->moveID != delId) {
          break;
        }
        removeCurrentLineForbidInterrupt();
      }
      cur = NULL;
      Printer::disableAllowedStepper();
      if(Printer::mode == PRINTER_MODE_FFF) {
        Printer::setFanSpeedDirectly(Printer::fanSpeed);
      }
      return Printer::interval;
    }
  }
  int maxLoops = (Printer::stepsPerTimerCall <= cur->stepsRemaining ? Printer::stepsPerTimerCall : cur->stepsRemaining);
  HAL::forbidInterrupts();
  for(int loop = 0; loop < maxLoops; loop++) {
#if STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY
    if(loop > 0)
      HAL::delayMicroseconds(STEPPER_HIGH_DELAY + DOUBLE_STEP_DELAY);
#endif
    if((cur->error[E_AXIS] -= cur->delta[E_AXIS]) < 0) {
#if USE_ADVANCE
      if(Printer::isAdvanceActivated()) { // Use interrupt for movement
        if(cur->isEPositiveMove())
          Printer::extruderStepsNeeded++;
        else
          Printer::extruderStepsNeeded--;
      } else
#endif
        Extruder::step();
      cur->error[E_AXIS] += cur_errupd;
    }
    if (curd) {
      // Take delta steps
      if(curd->isXMove())
        if((cur->error[X_AXIS] -= curd->deltaSteps[A_TOWER]) < 0) {
          cur->startXStep();
          cur->error[X_AXIS] += curd_errupd;
#ifdef DEBUG_REAL_POSITION
          Printer::realDeltaPositionSteps[A_TOWER] += curd->isXPositiveMove() ? 1 : -1;
#endif
#ifdef DEBUG_STEPCOUNT
          cur->totalStepsRemaining--;
#endif
        }

      if(curd->isYMove())
        if((cur->error[Y_AXIS] -= curd->deltaSteps[B_TOWER]) < 0) {
          cur->startYStep();
          cur->error[Y_AXIS] += curd_errupd;
#ifdef DEBUG_REAL_POSITION
          Printer::realDeltaPositionSteps[B_TOWER] += curd->isYPositiveMove() ? 1 : -1;
#endif
#ifdef DEBUG_STEPCOUNT
          cur->totalStepsRemaining--;
#endif
        }

      if(curd->isZMove())
        if((cur->error[Z_AXIS] -= curd->deltaSteps[C_TOWER]) < 0) {
          cur->startZStep();
          cur->error[Z_AXIS] += curd_errupd;
          Printer::realDeltaPositionSteps[C_TOWER] += curd->isZPositiveMove() ? 1 : -1;
#ifdef DEBUG_STEPCOUNT
          cur->totalStepsRemaining--;
#endif
        }
      stepsPerSegRemaining--;
    }
    Printer::insertStepperHighDelay();
    Printer::endXYZSteps();
#if USE_ADVANCE
    if(!Printer::isAdvanceActivated()) // Use interrupt for movement
#endif
      Extruder::unstep();
    if (!stepsPerSegRemaining) { // start new nonlinear segment
      if (cur->numNonlinearSegments && curd != NULL) {
#if FEATURE_BABYSTEPPING
        if(Printer::zBabystepsMissing/* && curd
                                        && (curd->dir & XYZ_STEP) == XYZ_STEP*/) {
          // execute a extra baby step
          Printer::zBabystep();
        }
#endif
        // Get the next delta segment
        curd = &cur->segments[--cur->numNonlinearSegments];

        // Initialize Bresenham for this segment (numPrimaryStepPerSegment is already correct for the half step setting)
        cur->error[X_AXIS] = cur->error[Y_AXIS] = cur->error[Z_AXIS] = cur->numPrimaryStepPerSegment >> 1;

        // Reset the counter of the primary steps. This is initialized in the line
        // generation so don't have to do this the first time.
        stepsPerSegRemaining = cur->numPrimaryStepPerSegment;

        // Change direction if necessary
        Printer::setXDirection(curd->dir & X_DIRPOS);
        Printer::setYDirection(curd->dir & Y_DIRPOS);
        Printer::setZDirection(curd->dir & Z_DIRPOS);
#if defined(DIRECTION_DELAY) && DIRECTION_DELAY > 0
        HAL::delayMicroseconds(DIRECTION_DELAY);
#endif

      } else
        curd = 0;// Release the last segment
      //deltaSegmentCount--;
    }
  } // for loop

  HAL::allowInterrupts(); // Allow interrupts for other types, timer1 is still disabled
#if RAMP_ACCELERATION
  //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
  if (cur->moveAccelerating()) {
    Printer::vMaxReached = HAL::ComputeV(Printer::timer, cur->fAcceleration) + cur->vStart;
    if(Printer::vMaxReached > cur->vMax) Printer::vMaxReached = cur->vMax;
    uint16_t v = Printer::updateStepsPerTimerCall(Printer::vMaxReached);
    Printer::interval = HAL::CPUDivU2(v);
    // if(Printer::maxInterval < Printer::interval) // fix timing for very slow speeds
    //    Printer::interval = Printer::maxInterval;
    Printer::timer += Printer::interval;
    cur->updateAdvanceSteps(Printer::vMaxReached, maxLoops, true);
    Printer::stepNumber += maxLoops; // is only used by moveAccelerating
  } else if (cur->moveDecelerating()) { // time to slow down
    uint16_t v = HAL::ComputeV(Printer::timer, cur->fAcceleration);
    if (v > Printer::vMaxReached)   // if deceleration goes too far it can become too large
      v = cur->vEnd;
    else {
      v = Printer::vMaxReached - v;
      if (v < cur->vEnd) v = cur->vEnd; // extra steps at the end of deceleration due to rounding errors
    }
    cur->updateAdvanceSteps(v, maxLoops, false);
    v = Printer::updateStepsPerTimerCall(v);
    Printer::interval = HAL::CPUDivU2(v);
    // if(Printer::maxInterval < Printer::interval) // fix timing for very slow speeds
    //    Printer::interval = Printer::maxInterval;
    Printer::timer += Printer::interval;
  } else {
    // If we had acceleration, we need to use the latest vMaxReached and interval
    // If we started full speed, we need to use cur->fullInterval and vMax
    cur->updateAdvanceSteps((!cur->accelSteps ? cur->vMax : Printer::vMaxReached), 0, true);
    if(!cur->accelSteps) {
      if(cur->vMax > STEP_DOUBLER_FREQUENCY) {
#if ALLOW_QUADSTEPPING
        if(cur->vMax > STEP_DOUBLER_FREQUENCY * 2) {
          Printer::stepsPerTimerCall = 4;
          Printer::interval = cur->fullInterval << 2;
        } else {
          Printer::stepsPerTimerCall = 2;
          Printer::interval = cur->fullInterval << 1;
        }
#else
        Printer::stepsPerTimerCall = 2;
        Printer::interval = cur->fullInterval << 1;
#endif
      } else {
        Printer::stepsPerTimerCall = 1;
        Printer::interval = cur->fullInterval;
      }
    }
  }
#else
  Printer::interval = cur->fullInterval; // without RAMPS always use full speed
#endif
  PrintLine::cur->stepsRemaining -= maxLoops;

  if(cur->stepsRemaining <= 0 || cur->isNoMove()) { // line finished
    // Release remaining delta segments
#ifdef DEBUG_STEPCOUNT
    if(cur->totalStepsRemaining || cur->numNonlinearSegments) {
      Com::printF(PSTR("Missed steps:"), cur->totalStepsRemaining);
      Com::printF(PSTR("\n"));
      Com::printF(PSTR("Step/seg r:"), stepsPerSegRemaining);
      Com::printF(PSTR("\n"));
      Com::printF(PSTR("NDS:"), (int) cur->numNonlinearSegments);
      Com::printF(PSTR("\n"));
    }
#endif
    removeCurrentLineForbidInterrupt();
    Printer::disableAllowedStepper();
    if(linesCount == 0) {
      if(!Printer::isPrinting()) {
        uid.setStatusP(PSTR("Idle"));
        uid.refreshPage();
      }
      if(Printer::mode == PRINTER_MODE_FFF) {
        Printer::setFanSpeedDirectly(Printer::fanSpeed);
      }
    }
    Printer::interval >>= 1; // 50% of time to next call to do cur=0
    //HAL::printFreeMemory();
  } // Do even
  return Printer::interval;
}

