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

#ifndef _REPETIER_H
#define _REPETIER_H

#include <math.h>
#include <stdint.h>

#include <avr/pgmspace.h>
#include <avr/io.h>

#undef min
#undef max

/** Uncomment, to see detailed data for every move. Only for debugging purposes! */
//#define DEBUG_QUEUE_MOVE

/** write infos about path planner changes */
//#define DEBUG_PLANNER

// Echo all ascii commands after receiving
//#define DEBUG_ECHO_ASCII

/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC

/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT

/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
//#define DEBUG_COM_ERRORS

/** Adds a menu point in quick settings to write debug informations to the host in case of hangs where the ui still works. */
//#define DEBUG_DELTA_OVERFLOW
//#define DEBUG_DELTA_REALPOS
//#define DEBUG_SPLIT

// Find the longest segment length during a print
//#define DEBUG_SEGMENT_LENGTH

// Find the maximum real jerk during a print
//#define DEBUG_REAL_JERK

// Debug reason for not mounting a sd card
//#define DEBUG_SD_ERROR

// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG


#define DELTA 3

#define IGNORE_COORDINATE 999999

#define HAS_PIN(x) (defined( x ## _PIN) && x > -1)

#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define E_AXIS 3
#define VIRTUAL_AXIS 4

// How big an array to hold X_AXIS..<MAX_AXIS>
#define Z_AXIS_ARRAY 3
#define E_AXIS_ARRAY 4
#define VIRTUAL_AXIS_ARRAY 5


#define A_TOWER 0
#define B_TOWER 1
#define C_TOWER 2
#define TOWER_ARRAY 3
#define E_TOWER_ARRAY 4

//direction flags
#define X_DIRPOS 1
#define Y_DIRPOS 2
#define Z_DIRPOS 4
#define E_DIRPOS 8
#define XYZ_DIRPOS 7

//step flags
#define XSTEP 16
#define YSTEP 32
#define ZSTEP 64
#define ESTEP 128

//combo's
#define XYZ_STEP 112
#define XY_STEP 48
#define XYZE_STEP 240
#define E_STEP_DIRPOS 136
#define Y_STEP_DIRPOS 34
#define X_STEP_DIRPOS 17
#define Z_STEP_DIRPOS 68

#define PRINTER_MODE_FFF 0

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "Configuration.h"

#ifndef MAX_JERK_DISTANCE
#define MAX_JERK_DISTANCE 0.6
#endif


#ifndef MAX_ROOM_TEMPERATURE
#define MAX_ROOM_TEMPERATURE 40
#endif


#ifndef FEATURE_BABYSTEPPING
#define FEATURE_BABYSTEPPING 0
#define BABYSTEP_MULTIPLICATOR 1
#endif

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
 */
#if !defined(ROD_RADIUS)
#define ROD_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)
#endif


//Step to split a circle in small Lines
#ifndef MM_PER_ARC_SEGMENT
#define MM_PER_ARC_SEGMENT 1
#define MM_PER_ARC_SEGMENT_BIG 3
#else
#define MM_PER_ARC_SEGMENT_BIG MM_PER_ARC_SEGMENT
#endif
//After this count of steps a new SIN / COS calculation is started to correct the circle interpolation
#define N_ARC_CORRECTION 25

// Test for shared cooler
#define SHARED_COOLER 0

#ifndef START_STEP_WITH_HIGH
#define START_STEP_WITH_HIGH 1
#endif











void manage_inactivity(uint8_t debug);

extern void finishNextSegment();

extern uint8_t transformCartesianStepsToDeltaSteps(long cartesianPosSteps[], long deltaPosSteps[]);

extern void linear_move(long steps_remaining[]);


extern void setupTimerInterrupt();
extern void microstepInit();




extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout

#define NUM_AXIS 4

#endif
