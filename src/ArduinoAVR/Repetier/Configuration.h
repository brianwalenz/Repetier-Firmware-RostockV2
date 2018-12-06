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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* Some words on units:

   From 0.80 onwards the units used are unified for easier configuration, watch out when transferring from older configs!

   Speed is in mm/s
   Acceleration in mm/s^2
   Temperature is in degrees Celsius


   ##########################################################################################
   ##                                        IMPORTANT                                     ##
   ##########################################################################################

   For easy configuration, the default settings enable parameter storage in EEPROM.
   This means, after the first upload many variables can only be changed using the special
   M commands as described in the documentation. Changing these values in the configuration.h
   file has no effect. Parameters overridden by EEPROM settings are calibration values, extruder
   values except thermistor tables and some other parameter likely to change during usage
   like advance steps or ops mode.
   To override EEPROM settings with config settings, set EEPROM_MODE 0

*/


/** Number of extruders. Maximum 6 extruders. */
#define NUM_EXTRUDER 1

#include "pins.h"

//  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
//  this is enabled because it was enabled before
#define EXTERNALSERIAL


// ##########################################################################################
// ##                               Calibration                                            ##
// ##########################################################################################

#define BELT_PITCH 2         /** \brief Pitch in mm of drive belt. GT2 = 2mm */
#define PULLEY_TEETH 20      /** \brief Number of teeth on X, Y and Z tower pulleys */
#define PULLEY_CIRCUMFERENCE (BELT_PITCH * PULLEY_TEETH)

#define STEPS_PER_ROTATION 200  /** \brief Steps per rotation of stepper motor */

#define MICRO_STEPS 16    /** \brief Micro stepping rate of X, Y and Y tower stepper drivers */

// Calculations
#define AXIS_STEPS_PER_MM ((float)(MICRO_STEPS * STEPS_PER_ROTATION) / PULLEY_CIRCUMFERENCE)
#define XAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define YAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define ZAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM


// ##########################################################################################
// ##                           Extruder configuration                                     ##
// ##########################################################################################


// The firmware checks if the heater and sensor got decoupled, which is dangerous. Since it will never reach target
// temperature, the heater will stay on for every which can burn your printer or house.
// As an additional barrier to your smoke detectors (I hope you have one above your printer) we now
// do some more checks to detect if something got wrong.

// Set to 1 if you want firmware to kill print on decouple
#define KILL_IF_SENSOR_DEFECT 0

// Retraction for sd pause over lcd
#define RETRACT_ON_PAUSE 2

/* Speed in mm/s for extruder moves fom internal commands, e.g. switching extruder. */
#define EXTRUDER_SWITCH_XY_SPEED 100







/** If enabled you can select the distance your filament gets retracted during a
    M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP 1


/** auto-retract converts pure extrusion moves into retractions. Beware that
    simple extrusion e.g. over Repetier-Host will then not work! */

#define RETRACTION_LENGTH 3
#define RETRACTION_LONG_LENGTH 13
#define RETRACTION_SPEED 40
#define RETRACTION_Z_LIFT 0
#define RETRACTION_UNDO_EXTRA_LENGTH 0
#define RETRACTION_UNDO_EXTRA_LONG_LENGTH 0
#define RETRACTION_UNDO_SPEED 20


/** PID control only works target temperature +/- PID_CONTROL_RANGE.
    If you get much overshoot at the first temperature set, because the heater is going full power too long, you
    need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15.
*/

/** Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
    extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter! */
#define EXTRUDE_MAXLENGTH 100
/** Skip wait, if the extruder temperature is already within x degrees. Only fixed numbers, 0 = off */
#define SKIP_M109_IF_WITHIN 2



// How often the temperature of the heated bed is set (msec)
#define HEATED_BED_SET_INTERVAL 5000



#define DEFAULT_PRINTER_MODE PRINTER_MODE_FFF




// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0

// Inverting motor direction. Only in case of pure cartesian printers, this
// is also the axis you invert!
#define INVERT_X_DIR 0
#define INVERT_Y_DIR 1
#define INVERT_Z_DIR 0


// maximum positions in mm - only fixed numbers!
// For delta robot Z_MAX_LENGTH is the maximum travel of the towers and should be set to the distance between the hotend
// and the platform when the printer is at its home position.
// If EEPROM is enabled these values will be overridden with the values in the EEPROM
#define X_MAX_LENGTH 350.0
#define Y_MAX_LENGTH 350.0
#define Z_MAX_LENGTH 350.0
// Coordinates for the minimum axis. Can also be negative if you want to have the bed start at 0 and the printer can go to the left side
// of the bed. Maximum coordinate is given by adding the above X_MAX_LENGTH values.
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0

// Park position used when pausing from firmware side
#define PARK_POSITION_X (0)
#define PARK_POSITION_Y (70)
#define PARK_POSITION_Z_RAISE 10

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################


// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)

//#define MOTOR_CURRENT {140,140,140,130,0} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)  use 140 for xyz and 130 for the E if using Kysan motors and 175 xyz and 200 if using wantai motors
#define MOTOR_CURRENT_PERCENT {55,55,55,50,0}


/** \brief Number of segments to generate for delta conversions per second of move
 */
#define DELTA_SEGMENTS_PER_SECOND_PRINT 225 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 80 // Less accurate setting for other moves






#define DELTA_DIAGONAL_ROD 290.8 // 290.8 is for the new ball cup arms and the older spring u joint style is 269.0


/*  =========== Parameter essential for delta calibration ===================

    C, Y-Axis
    |                        |___| CARRIAGE_HORIZONTAL_OFFSET (recommend set it to 0)
    |                        |   \------------------------------------------
    |_________ X-axis        |    \                                        |
    / \                       |     \  DELTA_DIAGONAL_ROD (length)    Each move this Rod Height
    /   \                             \                                 is calculated
    /     \                             \    Carriage is at printer center!   |
    A      B                             \_____/--------------------------------
    |--| END_EFFECTOR_HORIZONTAL_OFFSET (recommend set it to 0)
    |----| ROD_RADIUS (Horizontal rod pivot to pivot measure)
    |-----------| PRINTER_RADIUS (recommend set it to ROD_RADIUS)

    Column angles are measured from X-axis counterclockwise
    "Standard" positions: alpha_A = 210, alpha_B = 330, alpha_C = 90
*/

/** \brief column positions - change only to correct build imperfections! */
#define DELTA_ALPHA_A 210
#define DELTA_ALPHA_B 330
#define DELTA_ALPHA_C 90

/** Correct radius by this value for each column. Perfect builds have 0 everywhere. */
#define DELTA_RADIUS_CORRECTION_A 0
#define DELTA_RADIUS_CORRECTION_B 0
#define DELTA_RADIUS_CORRECTION_C 0

/** Correction of the default diagonal size. Value gets added.*/
#define DELTA_DIAGONAL_CORRECTION_A 0
#define DELTA_DIAGONAL_CORRECTION_B 0
#define DELTA_DIAGONAL_CORRECTION_C 0

/** Max. radius (mm) the printer should be able to reach. */
#define DELTA_MAX_RADIUS 140

// Margin (mm) to avoid above tower minimum (xMin xMinsteps)
// If your printer can put its carriage low enough the rod is horizontal without hitting the floor
// set this to zero. Otherwise, measure how high the carriage is from horizontal rod
// Also, movement speeds are 10x to 20x cartesian speeds at tower bottom.
// You may need to leave a few mm for safety.
// Hitting floor at high speed can damage your printer (motors, drives, etc)
// THIS MAY NEED UPDATING IF THE HOT END HEIGHT CHANGES!
#define DELTA_FLOOR_SAFETY_MARGIN_MM 15

/** \brief Horizontal offset of the universal joints on the end effector (moving platform).
 */
#define END_EFFECTOR_HORIZONTAL_OFFSET 30.22  // 33.0 for old 1/8" axle and u joint style platforms

/** \brief Horizontal offset of the universal joints on the vertical carriages.
 */
#define CARRIAGE_HORIZONTAL_OFFSET 27.1  //  27.075 is calculated for new molded carriagges  38.4 is for the old lasercut melamine trucks w/608zz bearings

/** \brief Printer radius in mm,
    measured from the center of the print area to the vertical smooth tower.
    Alternately set this to the pivot to pivot horizontal rod distance, when head is at (0,0)
*/
#define PRINTER_RADIUS 200.0  //  198.25 for older v1 machines.  

/** 1 for more precise delta moves. 0 for faster computation.
    Needs a bit more computation time. */
#define EXACT_DELTA_MOVES 1

/* ========== END Delta calibration data ==============*/

/** When true the delta will home to z max when reset/powered over cord. That way you start with well defined coordinates.
    If you don't do it, make sure to home first before your first move.
*/
#define DELTA_HOME_ON_POWER 0

/** To allow software correction of misaligned endstops, you can set the correction in steps here. If you have EEPROM enabled
    you can also change the values online and autoleveling will store the results here. */
#define DELTA_X_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Y_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Z_ENDSTOP_OFFSET_STEPS 0



/** \brief Number of delta moves in each line. Moves that exceed this figure will be split into multiple lines.
    Increasing this figure can use a lot of memory since 7 bytes * size of line buffer * MAX_SELTA_SEGMENTS_PER_LINE
    will be allocated for the delta buffer.
    PrintLine PrintLine::lines[PRINTLINE_CACHE_SIZE (default 16?)];
    Printline is about 200 bytes + 7 * DELTASEGMENTS_PER_PRINTLINE
    or 16 * (200 + (7*22=154) = 354) = 5664 bytes! !1
    min is 5 * (200 + (7*10=70) =270) = 1350
    This leaves ~1K free RAM on an Arduino which has only 8k
    Mega. Used only for nonlinear systems like delta or tuga. */
#define DELTASEGMENTS_PER_PRINTLINE 22

/** After x seconds of inactivity, the stepper motors are disabled.
    Set to 0 to leave them enabled.
    This helps cooling the Stepper motors between two print jobs.
    Overridden if EEPROM activated.
*/
#define STEPPER_INACTIVE_TIME 360
/** After x seconds of inactivity, the system will go down as far it can.
    It will at least disable all stepper motors and heaters. If the board has
    a power pin, it will be disabled, too.
    Set value to 0 for disabled.
    Overridden if EEPROM activated.
*/
#define MAX_INACTIVE_TIME 1800


#define ZHOME_MIN_TEMPERATURE 0
#define ZPROBE_MIN_TEMPERATURE ZHOME_MIN_TEMPERATURE

// needs to heat all extruders (1) or only current extruder (0)
#define ZHOME_HEAT_ALL 1 
// Z-height for heating extruder during homing
#define ZHOME_HEAT_HEIGHT 20

// If your bed might bend while probing, because your sensor is the extruder tip
// you can define a predefined x,y position so bending is always the same and
// can be compensated. Set coordinate to 999999 to ignore positions and just
// use the position you are at.
#define ZHOME_X_POS IGNORE_COORDINATE
#define ZHOME_Y_POS IGNORE_COORDINATE



/** Comment this to disable ramp acceleration */
#define RAMP_ACCELERATION 1

/** If your stepper needs a longer high signal then given, you can add a delay here.
    The delay is realized as a simple loop wasting time, which is not available for other
    computations. So make it as low as possible. For the most common drivers no delay is needed, as the
    included delay is already enough.
*/
#define STEPPER_HIGH_DELAY 0

/** If your driver needs some additional delay between setting direction and first step signal,
    you can set this here. There are some commands between direction and signal, but some drivers
    might be even slower or you are using a fast Arduino board with slow driver. Normally 0 works.
    If you get skewed print, you might try 1 microsecond here.
*/
#define DIRECTION_DELAY 0


/** The firmware can only handle 16000Hz interrupt frequency cleanly. If you need higher speeds
    a faster solution is needed, and this is to double/quadruple the steps in one interrupt call.
    This is like reducing your 1/16th microstepping to 1/8 or 1/4. It is much cheaper then 1 or 3
    additional stepper interrupts with all it's overhead. As a result you can go as high as
    40000Hz.
*/
//  motion.cpp guarded this to be between 7000 and 20000
#define STEP_DOUBLER_FREQUENCY 12000


/** If you need frequencies off more then 30000 you definitely need to enable this. If you have only 1/8 stepping
    enabling this may cause to stall your moves when 20000Hz is reached.
*/
#define ALLOW_QUADSTEPPING 1
/** If you reach STEP_DOUBLER_FREQUENCY the firmware will do 2 or 4 steps with nearly no delay. That can be too fast
    for some printers causing an early stall.

*/
#define DOUBLE_STEP_DELAY 1 // time in microseconds

/** If the firmware is busy, it will send a busy signal to host signaling that
    everything is fine and it only takes a bit longer to finish. That way the 
    host can keep timeout short so in case of communication errors the resulting
    blobs are much smaller. Set to 0 to disable it. */
#define KEEP_ALIVE_INTERVAL 2000
//// Acceleration settings

/** \brief X, Y, Z max acceleration in mm/s^2 for printing moves or retracts. Make sure your printer can go that high!
    Overridden if EEPROM activated.
*/
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1850
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1850
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 1850

/** \brief X, Y, Z max acceleration in mm/s^2 for travel moves.  Overridden if EEPROM activated.*/
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 3000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 3000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 3000


/** \brief Maximum allowable jerk.

    Caution: This is no real jerk in a physical meaning.

    The jerk determines your start speed and the maximum speed at the join of two segments.
    Its unit is mm/s. If the printer is standing still, the start speed is jerk/2. At the
    join of two segments, the speed difference is limited to the jerk value.

    Examples:
    For all examples jerk is assumed as 40.

    Segment 1: vx = 50, vy = 0
    Segment 2: vx = 0, vy = 50
    v_diff = sqrt((50-0)^2+(0-50)^2) = 70.71
    v_diff > jerk => vx_1 = vy_2 = jerk/v_diff*vx_1 = 40/70.71*50 = 28.3 mm/s at the join

    Segment 1: vx = 50, vy = 0
    Segment 2: vx = 35.36, vy = 35.36
    v_diff = sqrt((50-35.36)^2+(0-35.36)^2) = 38.27 < jerk
    Corner can be printed with full speed of 50 mm/s

    Overridden if EEPROM activated.
*/
#define MAX_JERK 35.0
#define MAX_ZJERK 35.0


//  Number of moves we can cache in advance.
//
//  This number of moves can be cached in advance. If you want to cache more, increase
//  this. Especially on many very short moves the cache may go empty.
//
//  The minimum value is 5 or maybe 4.

#define PRINTLINE_CACHE_SIZE 16



/** \brief Low filled cache size.

    If the cache contains less then MOVE_CACHE_LOW segments, the time per segment is limited to LOW_TICKS_PER_MOVE clock cycles.
    If a move would be shorter, the feedrate will be reduced. This should prevent buffer underflows. Set this to 0 if you
    don't care about empty buffers during print.
*/
#define MOVE_CACHE_LOW 10
/** \brief Cycles per move, if move cache is low.

    This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
    if you are printing many very short segments at high speed. Higher delays here allow higher values in PATH_PLANNER_CHECK_SEGMENTS.
*/
#define LOW_TICKS_PER_MOVE 250000

// ##########################################################################################
// ##                           Extruder control                                           ##
// ##########################################################################################


/* \brief Minimum temperature for extruder operation

   This is a safety value. If your extruder temperature is below this temperature, no
   extruder steps are executed. This is to prevent your extruder to move unless the filament
   is at least molten. After having some complains that the extruder does not work, I leave
   it 0 as default.
*/

#define MIN_EXTRUDER_TEMP 10


// ##########################################################################################
// ##                           Communication configuration                                ##
// ##########################################################################################




/** \brief Sets time for echo debug

    You can set M111 1 which enables ECHO of commands sent. This define specifies the position,
    when it will be executed. In the original FiveD software, echo is done after receiving the
    command. With checksum you know, how it looks from the sending string. With this define
    uncommented, you will see the last command executed. To be more specific: It is written after
    execution. This helps tracking errors, because there may be 8 or more commands in the queue
    and it is elsewise difficult to know, what your reprap is currently doing.
*/
#define ECHO_ON_EXECUTE 1

/** \brief EEPROM storage mode

    Set the EEPROM_MODE to 0 if you always want to use the settings in this configuration file. If not,
    set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
    EEPROM settings with configuration defaults, just select an other value. On the first call to epr_init()
    it will detect a mismatch of the first byte and copy default values into EEPROM. If the first byte
    matches, the stored values are used to overwrite the settings.

    IMPORTANT: With mode <>0 some changes in Configuration.h are not set any more, as they are
    taken from the EEPROM.
*/
#define EEPROM_MODE 1




/* A watchdog resets the printer, if a signal is not send within predefined time limits. That way we can be sure that the board
   is always running and is not hung up for some unknown reason. */
#define FEATURE_WATCHDOG 1



/* Autoleveling allows it to z-probe 3 points to compute the inclination and compensates the error for the print.
   This feature requires a working z-probe and you should have z-endstop at the top not at the bottom.
   The same 3 points are used for the G29 command.
*/
#define FEATURE_AUTOLEVEL 0


/* DISTORTION_CORRECTION compensates the distortion caused by mechanical imprecisions of nonlinear (i.e. DELTA) printers
 * assumes that the floor is plain (i.e. glass plate)
 *     and that it is perpendicular to the towers
 *     and that the (0,0) is in center
 * requires z-probe
 * G33 measures the Z offset in matrix NxN points (due to nature of the delta printer, the corners are extrapolated instead of measured)
 * and compensate the distortion
 * more points means better compensation, but consumes more memory and takes more time
 * DISTORTION_CORRECTION_R is the distance of last row or column from center
 */
#define DISTORTION_CORRECTION         0


/** \brief Experimental calibration utility for delta printers
 * Change 1 to 0 to disable
 */
#define FEATURE_SOFTWARE_LEVELING 0



/* Babystepping allows to change z height during print without changing official z height */
#define FEATURE_BABYSTEPPING 1

/* If you have a threaded rod, you want a higher multiplicator to see an effect. Limit value to 50 or you get easily overflows.*/
#define BABYSTEP_MULTIPLICATOR 1

/** Show extended directory including file length. Don't use this with Pronterface! */
//  used in src/SdFat/FatLib/FatFilePrint.cpp
#define SD_EXTENDED_DIR 1

/** You can store the current position with M401 and go back to it with M402.
    This works only if feature is set to true. */
#define FEATURE_MEMORY_POSITION 1

/** Should support for fan control be compiled in. If you enable this make sure
    the FAN pin is not the same as for your second extruder. RAMPS e.g. has FAN_PIN in 9 which
    is also used for the heater if you have 2 extruders connected. */
#define FEATURE_FAN_CONTROL 1

/* You can have a second fan controlled by adding P1 to M106/M107 command. */
#define FEATURE_FAN2_CONTROL 0
//#define FAN2_PIN ORIG_FAN2_PIN



/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME 10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT 500
/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT 50
/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT 50




// ###############################################################################
// ##                         Values for menu settings                          ##
// ###############################################################################

// Extreme values
#define UI_SET_MIN_HEATED_BED_TEMP  25
#define UI_SET_MAX_HEATED_BED_TEMP 120
#define UI_SET_MIN_EXTRUDER_TEMP    50
#define UI_SET_MAX_EXTRUDER_TEMP   270
#define UI_SET_EXTRUDER_FEEDRATE 2 // mm/sec
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3 // mm

#endif  //  CONFIGURATION_H
