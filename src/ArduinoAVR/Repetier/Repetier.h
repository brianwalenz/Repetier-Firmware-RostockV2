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

/** Writes the free RAM to output, if it is less then at the last test. Should always return
    values >500 for safety, since it doesn't catch every function call. Nice to tweak cache
    usage or for searching for memory induced errors. Switch it off for production, it costs execution time. */
//#define DEBUG_FREE_MEMORY
//#define DEBUG_ADVANCE

/** If enabled, writes the created generic table to serial port at startup. */
//#define DEBUG_GENERIC

/** If enabled, steps to move and moved steps are compared. */
//#define DEBUG_STEPCOUNT

/** This enables code to make M666 drop an ok, so you get problems with communication. It is to test host robustness. */
//#define DEBUG_COM_ERRORS

/** Adds a menu point in quick settings to write debug informations to the host in case of hangs where the ui still works. */
//#define DEBUG_PRINT
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

#define ANALOG_REF_AREF 0
#define ANALOG_REF_AVCC _BV(REFS0)
#define ANALOG_REF_INT_1_1 _BV(REFS1)
#define ANALOG_REF_INT_2_56 _BV(REFS0) | _BV(REFS1)
#define ANALOG_REF ANALOG_REF_AVCC

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

#define ILLEGAL_Z_PROBE -888

// we can not prevent this as some configurations need a parameter and others not
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "Configuration.h"

#ifndef BOARD_FAN_SPEED
#define BOARD_FAN_SPEED
#endif

#ifndef MAX_JERK_DISTANCE
#define MAX_JERK_DISTANCE 0.6
#endif


#if FEATURE_Z_PROBE && Z_PROBE_PIN < 0
#error You need to define Z_PROBE_PIN to use z probe!
#endif

#if DISTORTION_CORRECTION
#if !FEATURE_Z_PROBE
#error Distortion correction requires the z probe feature to be enabled and configured!
#endif
#endif

#ifndef MAX_ROOM_TEMPERATURE
#define MAX_ROOM_TEMPERATURE 40
#endif

// MS1 MS2 Stepper Driver Micro stepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#define MICROSTEP16 HIGH,HIGH
#define MICROSTEP32 HIGH,HIGH

#define GCODE_BUFFER_SIZE 1

#ifndef FEATURE_BABYSTEPPING
#define FEATURE_BABYSTEPPING 0
#define BABYSTEP_MULTIPLICATOR 1
#endif

#if !defined(Z_PROBE_REPETITIONS) || Z_PROBE_REPETITIONS < 1
#define Z_PROBE_SWITCHING_DISTANCE 0.5 // Distance to safely untrigger probe
#define Z_PROBE_REPETITIONS 1
#endif

#define SOFTWARE_LEVELING ((FEATURE_SOFTWARE_LEVELING) && (DRIVE_SYSTEM==DELTA))
/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
 */
#if !defined(ROD_RADIUS) && DRIVE_SYSTEM == DELTA
#define ROD_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)
#endif


#ifdef FEATURE_Z_PROBE
#define MANUAL_CONTROL 1
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

// Test for shared coolers between extruders and mainboard
#if EXT0_EXTRUDER_COOLER_PIN > -1 && EXT0_EXTRUDER_COOLER_PIN == FAN_BOARD_PIN
#define SHARED_COOLER_BOARD_EXT 1
#else
#define SHARED_COOLER_BOARD_EXT 0
#endif

#ifndef DEBUG_FREE_MEMORY
#define DEBUG_MEMORY
#else
#define DEBUG_MEMORY Commands::checkFreeMemory();
#endif


#define EXT0_SENSOR_INDEX  0
#define BED_SENSOR_INDEX  1

#define NUM_ANALOG_TEMP_SENSORS 2
#define ANALOG_INPUTS           2

#define ANALOG_INPUT_CHANNELS {EXT0_TEMPSENSOR_PIN, HEATED_BED_SENSOR_PIN}








#if !defined(MAX_FAN_PWM) || MAX_FAN_PWM == 255
#define TRIM_FAN_PWM(x) x
#undef MAX_FAN_PWM
#define MAX_FAN_PWM 255
#else
#define TRIM_FAN_PWM(x) static_cast<uint8_t>(static_cast<unsigned int>(x) * MAX_FAN_PWM / 255)
#endif

extern const uint8_t osAnalogInputChannels[] PROGMEM;

#if ANALOG_INPUTS > 0
extern volatile uint16_t osAnalogInputValues[ANALOG_INPUTS];
#endif

#define PWM_HEATED_BED    NUM_EXTRUDER
#define PWM_BOARD_FAN     PWM_HEATED_BED + 1
#define PWM_FAN1          PWM_BOARD_FAN + 1
#define PWM_FAN2          PWM_FAN1 + 1
#define PWM_FAN_THERMO    PWM_FAN2 + 1
#define NUM_PWM           PWM_FAN_THERMO + 1

extern uint8_t pwm_pos[NUM_PWM]; // 0-NUM_EXTRUDER = Heater 0-NUM_EXTRUDER of extruder, NUM_EXTRUDER = Heated bed, NUM_EXTRUDER+1 Board fan, NUM_EXTRUDER+2 = Fan

#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
extern int maxadv;
#endif

extern int maxadv2;
extern float maxadvspeed;
#endif


void manage_inactivity(uint8_t debug);

extern void finishNextSegment();

extern uint8_t transformCartesianStepsToDeltaSteps(long cartesianPosSteps[], long deltaPosSteps[]);

#if SOFTWARE_LEVELING
extern void calculatePlane(long factors[], long p1[], long p2[], long p3[]);
extern float calcZOffset(long factors[], long pointX, long pointY);
#endif

extern void linear_move(long steps_remaining[]);


extern void setupTimerInterrupt();
extern void motorCurrentControlInit();
extern void microstepInit();

extern long baudrate;



extern unsigned int counterPeriodical;
extern volatile uint8_t executePeriodical;
extern uint8_t counter500ms;
extern void writeMonitor();

#if FEATURE_FAN_CONTROL
extern uint8_t fanKickstart;
#endif

#if FEATURE_FAN2_CONTROL
extern uint8_t fan2Kickstart;
#endif

extern volatile int waitRelax; // Delay filament relax at the end of print, could be a simple timeout

#ifdef DEBUG_PRINT
extern int debugWaitLoop;
#endif

#define NUM_AXIS 4

#endif
