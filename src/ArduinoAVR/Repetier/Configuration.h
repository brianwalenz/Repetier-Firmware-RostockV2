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


// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

/** Number of extruders. Maximum 6 extruders. */
#define NUM_EXTRUDER 1

#include "pins.h"

// Override pin definitions from pins.h
//#define FAN_PIN   4  // Extruder 2 uses the default fan output, so move to an other pin


//  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.
//  this is enabled because it was enabled before
#define EXTERNALSERIAL


// Uncomment the following line if you are using Arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not being compatible!
//#define COMPAT_PRE1

/* Define the type of axis movements needed for your printer. The typical case
   is a full cartesian system where x, y and z moves are handled by separate motors.

   0 = full cartesian system, xyz have separate motors.
   1 = z axis + xy H-gantry (x_motor = x+y, y_motor = x-y)
   2 = z axis + xy H-gantry (x_motor = x+y, y_motor = y-x)
   3 = Delta printers (Rostock, Kossel, RostockMax, Cerberus, etc)
   4 = Tuga printer (Scott-Russell mechanism)
   5 = Bipod system (not implemented)
   8 = y axis + xz H-gantry (x_motor = x+z, z_motor = x-z)
   9 = y axis + xz H-gantry (x_motor = x+z, z_motor = z-x)
   Cases 1, 2, 8 and 9 cover all needed xy and xz H gantry systems. If you get results mirrored etc. you can swap motor connections for x and y.
   If a motor turns in the wrong direction change INVERT_X_DIR or INVERT_Y_DIR.
*/

#define DRIVE_SYSTEM DELTA    //  UNUSED
#define NONLINEAR_SYSTEM 1    //  UNUSED


// ##########################################################################################
// ##                               Calibration                                            ##
// ##########################################################################################

/** Drive settings for the Delta printers
 */
#if DRIVE_SYSTEM == DELTA
// ***************************************************
// *** These parameter are only for Delta printers ***
// ***************************************************

/** \brief Delta drive type: 0 - belts and pulleys, 1 - filament drive */
#define DELTA_DRIVE_TYPE 0

#if DELTA_DRIVE_TYPE == 0
/** \brief Pitch in mm of drive belt. GT2 = 2mm */
#define BELT_PITCH 2
/** \brief Number of teeth on X, Y and Z tower pulleys */
#define PULLEY_TEETH 20
#define PULLEY_CIRCUMFERENCE (BELT_PITCH * PULLEY_TEETH)
#elif DELTA_DRIVE_TYPE == 1
/** \brief Filament pulley diameter in millimeters */
#define PULLEY_DIAMETER 10
#define PULLEY_CIRCUMFERENCE (PULLEY_DIAMETER * 3.1415927)
#endif

/** \brief Steps per rotation of stepper motor */
#define STEPS_PER_ROTATION 200

/** \brief Micro stepping rate of X, Y and Y tower stepper drivers */
#define MICRO_STEPS 16

// Calculations
#define AXIS_STEPS_PER_MM ((float)(MICRO_STEPS * STEPS_PER_ROTATION) / PULLEY_CIRCUMFERENCE)
#define XAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define YAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define ZAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#else
// *******************************************************
// *** These parameter are for all other printer types ***
// *******************************************************

/** Drive settings for printers with cartesian drive systems */
/** \brief Number of steps for a 1mm move in x direction.
    For xy gantry use 2*belt moved!
    Overridden if EEPROM activated. */
#define XAXIS_STEPS_PER_MM 98.425196
/** \brief Number of steps for a 1mm move in y direction.
    For xy gantry use 2*belt moved!
    Overridden if EEPROM activated.*/
#define YAXIS_STEPS_PER_MM 98.425196
/** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
#define ZAXIS_STEPS_PER_MM 2560
#endif

// ##########################################################################################
// ##                           Extruder configuration                                     ##
// ##########################################################################################

// You can use either PWM (pulse width modulation) or PDM (pulse density modulation) for
// extruders or coolers. PDM will give more signal changes per second, so on average it gives
// the cleaner signal. The only advantage of PWM is giving signals at a fixed rate and never more
// then PWM.
#define PDM_FOR_EXTRUDER 1
#define PDM_FOR_COOLER 1

// The firmware checks if the heater and sensor got decoupled, which is dangerous. Since it will never reach target
// temperature, the heater will stay on for every which can burn your printer or house.
// As an additional barrier to your smoke detectors (I hope you have one above your printer) we now
// do some more checks to detect if something got wrong.

// If the temp. is on hold target, it may not sway more then this degrees celsius, or we mark
// sensor as defect.
#define DECOUPLING_TEST_MAX_HOLD_VARIANCE 20
// Minimum temp. rise we expect after the set duration of full heating is over.
// Always keep a good safety margin to get no false positives. If your period is e.g. 10 seconds
// because at startup you already need 7 seconds until heater starts to rise temp. for sensor
// then you have 3 seconds of increased heating to reach 1°C.
#define DECOUPLING_TEST_MIN_TEMP_RISE 1
// Set to 1 if you want firmware to kill print on decouple
#define KILL_IF_SENSOR_DEFECT 0
// for each extruder, fan will stay on until extruder temperature is below this value
#define EXTRUDER_FAN_COOL_TEMP 50
// Retraction for sd pause over lcd
#define RETRACT_ON_PAUSE 2

/* Speed in mm/s for extruder moves fom internal commands, e.g. switching extruder. */
#define EXTRUDER_SWITCH_XY_SPEED 100

// Extruder offsets in steps not mm!
#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
#define EXT0_Z_OFFSET 0
// for skeinforge 40 and later, steps to pull the plastic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT0_STEPS_PER_MM 92.4 // EZStruder
// What type of sensor is used?
// 0 is no thermistor/temperature control
// 1 is 100k thermistor (Epcos B57560G0107F000 - RepRap-Fab.org and many other)
// 2 is 200k thermistor
// 3 is mendel-parts thermistor (EPCOS G550)
// 4 is 10k thermistor
// 8 is ATC Semitec 104GT-2
// 12 is 100k RS thermistor 198-961
// 13 is PT100 for E3D/Ultimaker
// 14 is 100K NTC 3950
// 15 DYZE DESIGN 500°C Thermistor
// 16 is B3 innovations 500°C sensor
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 60 is AD8494, AD8495, AD8496 or AD8497 (5mV/degC and 1/4 the price of AD595 but only MSOT_08 package)
// 61 is AD8494, AD8495, AD8496 or AD8497 (5mV/degC and 1.25 Vref offset like adafruit breakout)
// 97 Generic thermistor table 1
// 98 Generic thermistor table 2
// 99 Generic thermistor table 3
// 100 is AD595     - no support
// 101 is MAX6675   - no support
// 102 is MAX31855  - no support
#define EXT0_TEMPSENSOR_TYPE 8 // E3D
// Analog input pin for reading temperatures or pin enabling SS for MAX6675
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
// Which pin enables the heater
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E0_STEP_PIN
#define EXT0_DIR_PIN E0_DIR_PIN
// set to false/true for normal / inverse direction
#define EXT0_INVERSE true
#define EXT0_ENABLE_PIN E0_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT0_ENABLE_ON 0
/* Set to 1 to mirror motor. Pins for mirrored motor are below */
#define EXT0_MIRROR_STEPPER 0
#define EXT0_STEP2_PIN E0_STEP_PIN
#define EXT0_DIR2_PIN E0_DIR_PIN
#define EXT0_INVERSE2 false
#define EXT0_ENABLE2_PIN E0_ENABLE_PIN
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use higher values.
//  Overridden if EEPROM activated.
#define EXT0_MAX_FEEDRATE 100
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT0_MAX_START_FEEDRATE 45
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT0_MAX_ACCELERATION 6500
/** Type of heat manager for this extruder.
    - 0 = Simply switch on/off if temperature is reached. Works always.
    - 1 = PID Temperature control. Is better but needs good PID values. Defaults are a good start for most extruder.
    - 3 = Dead-time control. PID_P becomes dead-time in seconds.
    Overridden if EEPROM activated.
*/
#define EXT0_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT0_WATCHPERIOD 3

/** \brief The maximum value, I-gain can contribute to the output.

    A good value is slightly higher then the output needed for your temperature.
    Values for starts:
    130 => PLA for temperatures from 170-180 deg C
    180 => ABS for temperatures around 240 deg C

    The precise values may differ for different nozzle/resistor combination.
    Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MAX 205
/** \brief lower value for integral part

    The I state should converge to the exact heater output needed for the target temperature.
    To prevent a long deviation from the target zone, this value limits the lower value.
    A good start is 30 lower then the optimal value. You need to leave room for cooling.
    Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MIN 60
/** P-gain.  Overridden if EEPROM activated. */
#define EXT0_PID_PGAIN_OR_DEAD_TIME   31.36
/** I-gain. Overridden if EEPROM activated.
 */
#define EXT0_PID_I   2.18
/** Dgain.  Overridden if EEPROM activated.*/
#define EXT0_PID_D 112.90
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT0_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
    K is the factor for the quadratic term, which is normally disabled in newer versions. If you want to use
    the quadratic factor make sure ENABLE_QUADRATIC_ADVANCE is defined.
    L is the linear factor and seems to be working better then the quadratic dependency.
*/
#define EXT0_ADVANCE_K 0.0f
#define EXT0_ADVANCE_L 0.0f
/* Motor steps to remove backlash for advance algorithm. These are the steps
   needed to move the motor cog in reverse direction until it hits the driving
   cog. Direct drive extruder need 0. */
#define EXT0_ADVANCE_BACKLASH_STEPS 0
/** \brief Temperature to retract filament when extruder is heating up. Overridden if EEPROM activated.
 */
#define EXT0_WAIT_RETRACT_TEMP 		150
/** \brief Units (mm/inches) to retract filament when extruder is heating up. Overridden if EEPROM activated. Set
    to 0 to disable.
*/
#define EXT0_WAIT_RETRACT_UNITS 	0

/** You can run any GCODE command on extruder deselect/select. Separate multiple commands with a new line \n.
    That way you can execute some mechanical components needed for extruder selection or retract filament or whatever you need.
    The codes are only executed for multiple extruder when changing the extruder. */
#define EXT0_SELECT_COMMANDS "M117 Extruder 1"
#define EXT0_DESELECT_COMMANDS ""
/** The extruder cooler is a fan to cool the extruder when it is heating. If you turn the extruder on, the fan goes on. */
#define EXT0_EXTRUDER_COOLER_PIN 7
/** PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT0_EXTRUDER_COOLER_SPEED 255
/** Time in ms between a heater action and test of success. Must be more then time between turning heater on and first temp. rise! 
 * 0 will disable decoupling test */
#define EXT0_DECOUPLE_TEST_PERIOD 18000
/* Temperature when using preheat */
#define EXT0_PREHEAT_TEMP 190






/** If enabled you can select the distance your filament gets retracted during a
    M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP 1

/** Allow retraction with G10/G11 removing requirement for retraction setting in slicer. Also allows filament change if lcd is configured. */
#define FEATURE_RETRACTION 1


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
#define PID_CONTROL_RANGE 20

/** Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
    extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter! */
#define EXTRUDE_MAXLENGTH 100
/** Skip wait, if the extruder temperature is already within x degrees. Only fixed numbers, 0 = off */
#define SKIP_M109_IF_WITHIN 2

/** \brief Set PID scaling

    PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by to methods.
    Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
    Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
    If your EXT0_PID_MAX is low, you should prefer the second method.
*/
#define SCALE_PID_TO_MAX 0


#define HEATER_PWM_SPEED 1 // How fast ist pwm signal 0 = 15.25Hz, 1 = 30.51Hz, 2 = 61.03Hz, 3 = 122.06Hz

/** Temperature range for target temperature to hold in M109 command. 5 means +/-5 degC

    Uncomment define to force the temperature into the range for given watch period.
*/
//#define TEMP_HYSTERESIS 5

/** Userdefined thermistor table

    There are many different thermistors, which can be combined with different resistors. This result
    in unpredictable number of tables. As a resolution, the user can define one table here, that can
    be used as type 5 for thermistor type in extruder/heated bed definition. Make sure, the number of entries
    matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
    each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
    increasing adc values. For more informations, read
    http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

    If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
    This firmware works with increased precision, so the value reads go from 0 to 4095 and the temperature is
    temperature*8.

    If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use thermistor types 50-52 instead of 5-7!
*/
/** Number of entries in the user thermistor table 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0 28
#define USER_THERMISTORTABLE0  {                                        \
    {1*4,864*8},{21*4,300*8},{25*4,290*8},{29*4,280*8},{33*4,270*8},{39*4,260*8},{46*4,250*8},{54*4,240*8},{64*4,230*8},{75*4,220*8}, \
    {90*4,210*8},{107*4,200*8},{128*4,190*8},{154*4,180*8},{184*4,170*8},{221*4,160*8},{265*4,150*8},{316*4,140*8},{375*4,130*8}, \
    {441*4,120*8},{513*4,110*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	}

/** Number of entries in the user thermistor table 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1  {}
/** Number of entries in the user thermistor table 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2  {}

/** If defined, creates a thermistor table at startup.

    If you don't feel like computing the table on your own, you can use this generic method. It is
    a simple approximation which may be not as accurate as a good table computed from the reference
    values in the datasheet. You can increase precision if you use a temperature/resistance for
    R0/T0, which is near your operating temperature. This will reduce precision for lower temperatures,
    which are not really important. The resistors must fit the following schematic:
    @code
    VREF ---- R2 ---+--- Termistor ---+-- GND
    |                 |
    +------ R1 -------+
    |                 |
    +---- Capacitor --+
    |
    V measured
    @endcode

    If you don't have R1, set it to 0.
    The capacitor is for reducing noise from long thermistor cable. If you don't have one, it's OK.

    If you need the generic table, uncomment the following define.
*/
//#define USE_GENERIC_THERMISTORTABLE_1

/* Some examples for different thermistors:

   EPCOS B57560G104+ : R0 = 100000  T0 = 25  Beta = 4036
   EPCOS 100K Thermistor (B57560G1104F) :  R0 = 100000  T0 = 25  Beta = 4092
   ATC Semitec 104GT-2 : R0 = 100000  T0 = 25  Beta = 4267
   Honeywell 100K Thermistor (135-104LAG-J01)  : R0 = 100000  T0 = 25  Beta = 3974

*/

/** Reference Temperature */
#define GENERIC_THERM1_T0 25
/** Resistance at reference temperature */
#define GENERIC_THERM1_R0 100000
/** Beta value of thermistor

    You can use the beta from the datasheet or compute it yourself.
    See http://reprap.org/wiki/MeasuringThermistorBeta for more details.
*/
#define GENERIC_THERM1_BETA 4450
/** Start temperature for generated thermistor table */
#define GENERIC_THERM1_MIN_TEMP -20
/** End Temperature for generated thermistor table */
#define GENERIC_THERM1_MAX_TEMP 300
#define GENERIC_THERM1_R1 0
#define GENERIC_THERM1_R2 4700

// The same for table 2 and 3 if needed

#define USE_GENERIC_THERMISTORTABLE_2
#define GENERIC_THERM2_T0 25
#define GENERIC_THERM2_R0 100000
#define GENERIC_THERM2_BETA 4367
#define GENERIC_THERM2_MIN_TEMP -20
#define GENERIC_THERM2_MAX_TEMP 300
#define GENERIC_THERM2_R1 0
#define GENERIC_THERM2_R2 4700

//#define USE_GENERIC_THERMISTORTABLE_3
#define GENERIC_THERM3_T0 170
#define GENERIC_THERM3_R0 1042.7
#define GENERIC_THERM3_BETA 4036
#define GENERIC_THERM3_MIN_TEMP -20
#define GENERIC_THERM3_MAX_TEMP 300
#define GENERIC_THERM3_R1 0
#define GENERIC_THERM3_R2 4700

/** Supply voltage to ADC, can be changed by setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VREF 5
/** Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
    Value is used for all generic tables created. */
#define GENERIC_THERM_NUM_ENTRIES 33

// ############# Heated bed configuration ########################

/** \brief Set true if you have a heated bed connected to your board, false if not - must be 0 or 1 */
#define HAVE_HEATED_BED 1

#define HEATED_BED_MAX_TEMP 120

// Select type of your heated bed. It's the same as for EXT0_TEMPSENSOR_TYPE
// set to 0 if you don't have a heated bed
#define HEATED_BED_SENSOR_TYPE 98
/** Analog pin of analog sensor to read temperature of heated bed.  */
#define HEATED_BED_SENSOR_PIN TEMP_1_PIN
/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_1_PIN
// How often the temperature of the heated bed is set (msec)
#define HEATED_BED_SET_INTERVAL 5000

/**
   Heat manager for heated bed:
   0 = Bang Bang, fast update
   1 = PID controlled
   2 = Bang Bang, limited check every HEATED_BED_SET_INTERVAL. Use this with relay-driven beds to save life time
   3 = dead time control
*/
#define HEATED_BED_HEAT_MANAGER 1
/** \brief The maximum value, I-gain can contribute to the output.
    The precise values may differ for different nozzle/resistor combination.
    Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 200
/** \brief lower value for integral part

    The I state should converge to the exact heater output needed for the target temperature.
    To prevent a long deviation from the target zone, this value limits the lower value.
    A good start is 30 lower then the optimal value. You need to leave room for cooling.
    Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
/** P-gain.  Overridden if EEPROM activated. */
#define HEATED_BED_PID_PGAIN_OR_DEAD_TIME   87.86
/** I-gain  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_IGAIN   3.01
/** Dgain.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_DGAIN 641.82
// maximum time the heater can be switched on. Max = 255.  Overridden if EEPROM activated.
#define HEATED_BED_PID_MAX 200
// Time to see a temp. change when fully heating. Consider that beds at higher temp. need longer to rise and cold
// beds need some time to get the temp. to the sensor. Time is in milliseconds! Set 0 to disable
#define HEATED_BED_DECOUPLE_TEST_PERIOD 300000

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
#define MAXTEMP 260   // E3D could go up to 300

#define HEATED_BED_PREHEAT_TEMP 55

/** Extreme values to detect defect thermistors. */
#define MIN_DEFECT_TEMPERATURE 10
#define MAX_DEFECT_TEMPERATURE 270

//How many milliseconds a hot end will preheat before starting to check the
//temperature. This value should NOT be set to the time it takes the
//hot end to reach the target temperature, but should be set to the time it 
//takes to reach the minimum temperature your thermistor can read. The lower
//the better/safer, and shouldn't need to be more than 30 seconds (30000) 
#define MILLISECONDS_PREHEAT_TIME 30000

#define DEFAULT_PRINTER_MODE PRINTER_MODE_FFF

// ##########################################################################################
// ##                            Endstop configuration                                     ##
// ##########################################################################################

/* By default all endstops are pulled up to HIGH. You need a pull-up if you
   use a mechanical endstop connected with GND. Set value to false for no pull-up
   on this endstop.
*/
#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_PULLUP_Z_MAX true

//set to true to invert the logic of the endstops
#define ENDSTOP_X_MIN_INVERTING true
#define ENDSTOP_Y_MIN_INVERTING true
#define ENDSTOP_Z_MIN_INVERTING true
#define ENDSTOP_X_MAX_INVERTING false
#define ENDSTOP_Y_MAX_INVERTING false
#define ENDSTOP_Z_MAX_INVERTING false

// Set the values true where you have a hardware endstop. The Pin number is taken from pins.h.

#define MIN_HARDWARE_ENDSTOP_X false
#define MIN_HARDWARE_ENDSTOP_Y false
#define MIN_HARDWARE_ENDSTOP_Z false
#define MAX_HARDWARE_ENDSTOP_X true
#define MAX_HARDWARE_ENDSTOP_Y true
#define MAX_HARDWARE_ENDSTOP_Z true

//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here

//// ADVANCED SETTINGS - to tweak parameters

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0

// Inverting motor direction. Only in case of pure cartesian printers, this
// is also the axis you invert!
#define INVERT_X_DIR 0
#define INVERT_Y_DIR 1
#define INVERT_Z_DIR 0

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR 1
#define Y_HOME_DIR 1
#define Z_HOME_DIR 1

// Delta robot radius endstop
#define max_software_endstop_r true

//If true, axis won't move to coordinates less than zero.
#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false

//If true, axis won't move to coordinates greater than the defined lengths below.
#define max_software_endstop_x true
#define max_software_endstop_y true
#define max_software_endstop_z true

// If during homing the endstop is reached, ho many mm should the printer move back for the second try
#define ENDSTOP_X_BACK_MOVE 10
#define ENDSTOP_Y_BACK_MOVE 10
#define ENDSTOP_Z_BACK_MOVE 10

// For higher precision you can reduce the speed for the second test on the endstop
// during homing operation. The homing speed is divided by the value. 1 = same speed, 2 = half speed
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 4
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 4
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 4

// When you have several endstops in one circuit you need to disable it after homing by moving a
// small amount back. This is also the case with H-belt systems.
#define ENDSTOP_X_BACK_ON_HOME 5
#define ENDSTOP_Y_BACK_ON_HOME 5
#define ENDSTOP_Z_BACK_ON_HOME 5
// If you do z min homing, you might want to rise extruder a bit after homing so it does not heat
// touching your bed.
#define Z_UP_AFTER_HOME 0

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
/** If you print on a moving bed, it can become more shaky the higher and bigger
    your print gets. Therefore it might be helpfull to reduce acceleration with
    increasing print height. You can define here how acceleration should change.
    You set ACCELERATION_FACTOR_TOP to the factor in percent for the top position
    of your printer. Acceleration will then be modified linear over height.
    INTERPOLATE_ACCELERATION_WITH_Z sets, which accelerations get changed:
    0 = do not interpolate at all
    1 = interpolate x and y acceleration
    2 = interpolate z acceleration
    3 = interpolate x,y and z acceleration
*/
#define INTERPOLATE_ACCELERATION_WITH_Z 0
#define ACCELERATION_FACTOR_TOP 100

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

/** \brief Number of moves we can cache in advance.

    This number of moves can be cached in advance. If you want to cache more, increase this. Especially on
    many very short moves the cache may go empty. The minimum value is 5.
*/
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

/** \brief Enable advance algorithm.

    Without a correct adjusted advance algorithm, you get blobs at points, where acceleration changes. The
    effect increases with speed and acceleration difference. Using the advance method decreases this effect.
    For more informations, read the wiki.
*/
#define USE_ADVANCE 1

/** \brief enables quadratic component.

    Set 1 to allow, 0 disallow a quadratic advance dependency. Linear is the dominant value, so no real need
    to activate the quadratic term. Only adds lots of computations and storage usage. */
#define ENABLE_QUADRATIC_ADVANCE 0


// ##########################################################################################
// ##                           Communication configuration                                ##
// ##########################################################################################

//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

/** \brief Communication speed.

    - 250000 : Fastest with error rate of 0% with 16 or 32 MHz - update wiring_serial.c in your board files. See boards/readme.txt
    - 115200 : Fast, but may produce communication errors on quite regular basis, Error rate -3,5%
    - 76800 : Best setting for Arduino with 16 MHz, Error rate 0,2% page 198 AVR1284 Manual. Result: Faster communication then 115200
    - 57600 : Should produce nearly no errors, on my gen 6 it's faster than 115200 because there are no errors slowing down the connection
    - 38600

    Overridden if EEPROM activated.
*/
//#define BAUDRATE 76800
//#define BAUDRATE 115200
#define BAUDRATE 250000

/**
   Some boards like Gen7 have a power on pin, to enable the ATX power supply. If this is defined,
   the power will be turned on without the need to call M80 if initially started.
*/
#define ENABLE_POWER_ON_STARTUP 1


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




/** Some fans won't start for low values, but would run if started with higher power at the beginning.
    This defines the full power duration before returning to set value. Time is in milliseconds */
#define FAN_KICKSTART_TIME  200
/** Defines the max. fan speed for M106 controlled fans. Normally 255 to use full range, but for
    12V fans on 24V this might help preventing a defect. For all other fans there is a explicit maximum PWM value
    you can set, so this is not used for other fans! */
#define MAX_FAN_PWM 255

/* A watchdog resets the printer, if a signal is not send within predefined time limits. That way we can be sure that the board
   is always running and is not hung up for some unknown reason. */
#define FEATURE_WATCHDOG 1

/* Z-Probing */

#define FEATURE_Z_PROBE 0
// Especially if you have more then 1 extruder acting as z probe this is important!
#define EXTRUDER_IS_Z_PROBE 0
// Disable all heaters before probing - required for inductive sensors
#define Z_PROBE_DISABLE_HEATERS 0
#define Z_PROBE_PIN 63
#define Z_PROBE_PULLUP 1
#define Z_PROBE_ON_HIGH 1
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
#define Z_PROBE_BED_DISTANCE 5.0 // Higher than max bed level distance error in mm

// Waits for a signal to start. Valid signals are probe hit and ok button.
// This is needful if you have the probe trigger by hand.
#define Z_PROBE_WAIT_BEFORE_TEST 0
/** Speed of z-axis in mm/s when probing */
#define Z_PROBE_SPEED 2
/** Delay before going down. Needed for piezo endstops to reload safely. */
#define Z_PROBE_DELAY 0
#define Z_PROBE_XY_SPEED 150
#define Z_PROBE_SWITCHING_DISTANCE 1.5 // Distance to safely switch off probe after it was activated
#define Z_PROBE_REPETITIONS 5 // Repetitions for probing at one point.
/** Distance between nozzle and bed when probe triggers. */
#define Z_PROBE_HEIGHT 39.91
/** Set 1 if you need a hot extruder for good probe results. Normally only required if nozzle is probe. */
#define Z_PROBE_REQUIRES_HEATING 0
/** Minimum extruder temperature for probing. If it is lower, it will be increased to that value. */
#define Z_PROBE_MIN_TEMPERATURE 150

/*
  Define how we measure the bed rotation. 
  All methods need at least 3 points to define the bed rotation correctly. The quality we get comes
  from the selection of the right points and method.

  BED_LEVELING_METHOD 0
  This method measures at the 3 probe points and creates a plane through these points. If you have
  a really planar bed this gives the optimum result. The 3 points must not be in one line and have
  a long distance to increase numerical stability.

  BED_LEVELING_METHOD 1
  This measures a grid. Probe point 1 is the origin and points 2 and 3 span a grid. We measure
  BED_LEVELING_GRID_SIZE points in each direction and compute a regression plane through all
  points. This gives a good overall plane if you have small bumps measuring inaccuracies.

  BED_LEVELING_METHOD 2
  Bending correcting 4 point measurement. This is for cantilevered beds that have the rotation axis
  not at the side but inside the bed. Here we can assume no bending on the axis and a symmetric
  bending to both sides of the axis. So probe points 2 and 3 build the symmetric axis and
  point 1 is mirrored to 1m across the axis. Using the symmetry we then remove the bending
  from 1 and use that as plane.
*/
#define BED_LEVELING_METHOD 0

// Grid size for grid based plane measurement
#define BED_LEVELING_GRID_SIZE 4

// Repetitions for motorized bed leveling
#define BED_LEVELING_REPETITIONS 5

/* These are the motor positions relative to bed origin. Only needed for
   motorized bed leveling */
#define BED_MOTOR_1_X 0
#define BED_MOTOR_1_Y 0
#define BED_MOTOR_2_X 200
#define BED_MOTOR_2_Y 0
#define BED_MOTOR_3_X 100
#define BED_MOTOR_3_Y 200

/* Autoleveling allows it to z-probe 3 points to compute the inclination and compensates the error for the print.
   This feature requires a working z-probe and you should have z-endstop at the top not at the bottom.
   The same 3 points are used for the G29 command.
*/
#define FEATURE_AUTOLEVEL 0
#define Z_PROBE_X1 100
#define Z_PROBE_Y1 20
#define Z_PROBE_X2 160
#define Z_PROBE_Y2 170
#define Z_PROBE_X3 20
#define Z_PROBE_Y3 170

/* Bending correction adds a value to a measured z-probe value. This may be
   required when the z probe needs some force to trigger and this bends the
   bed down. Currently the correction values A/B/C correspond to z probe
   positions 1/2/3. In later versions a bending correction algorithm might be
   introduced to give it other meanings.*/
#define BENDING_CORRECTION_A 0
#define BENDING_CORRECTION_B 0
#define BENDING_CORRECTION_C 0

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
#define DISTORTION_CORRECTION_POINTS  5
/** Max. distortion value to enter. Used to prevent dangerous errors with big values. */
#define DISTORTION_LIMIT_TO 2
/* For delta printers you simply define the measured radius around origin */
#define DISTORTION_CORRECTION_R       80
/* For all others you define the correction rectangle by setting the min/max coordinates. Make sure the the probe can reach all points! */
#define DISTORTION_XMIN 10
#define DISTORTION_YMIN 10
#define DISTORTION_XMAX 190
#define DISTORTION_YMAX 190

/** Uses EEPROM instead of ram. Allows bigger matrix (up to 22x22) without any ram cost.
    Especially on arm based systems with cached EEPROM it is good, on AVR it has a small
    performance penalty.
*/
#define DISTORTION_PERMANENT          1
/** Correction computation is not a cheap operation and changes are only small. So it
    is not necessary to update it for every sub-line computed. For example lets take DELTA_SEGMENTS_PER_SECOND_PRINT = 150
    and fastest print speed 100 mm/s. So we have a maximum segment length of 100/150 = 0.66 mm.
    Now lats say our point field is 200 x 200 mm with 9 x 9 points. So between 2 points we have
    200 / (9-1) = 25 mm. So we need at least 25 / 0.66 = 37 lines to move to the next measuring
    point. So updating correction every 15 calls gives us at least 2 updates between the
    measured points.
    NOTE: Explicit z changes will always trigger an update!
*/
#define DISTORTION_UPDATE_FREQUENCY   15
/** z distortion degrades to 0 from this height on. You should start after the first layer to get
    best bonding with surface. */
#define DISTORTION_START_DEGRADE 0.5
/** z distortion correction gets down to 0 at this height. */
#define DISTORTION_END_HEIGHT 1.5
/** If your corners measurement points are not measurable with given radius, you can
    set this to 1. It then omits the outer measurement points allowing a larger correction area.*/
#define DISTORTION_EXTRAPOLATE_CORNERS 0




/** \brief Experimental calibration utility for delta printers
 * Change 1 to 0 to disable
 */
#define FEATURE_SOFTWARE_LEVELING 0

/* Babystepping allows to change z height during print without changing official z height */
#define FEATURE_BABYSTEPPING 1

/* If you have a threaded rod, you want a higher multiplicator to see an effect. Limit value to 50 or you get easily overflows.*/
#define BABYSTEP_MULTIPLICATOR 1

//  SDSUPPORT is MAGIC, it's in FatLib/FatFile*cpp, and removing it breaks the build.
#define SDSUPPORT 1
#define SDCARDDETECT 81
//#define SDCARDDETECTINVERTED 0  // Change to true if you get a inserted message on removal.


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

/* By setting FAN_BOARD_PIN to a pin number you get a board cooler. That fan 
   goes on as soon as moves occur. Mainly to prevent overheating of stepper drivers. */
//#undef FAN_BOARD_PIN
//#define FAN_BOARD_PIN ORIG_FAN_PIN
/** Speed of board fan when on. 0 = off, 255 = max */
#define BOARD_FAN_SPEED 255
/* Speed when no cooling is required. Normally 0 but if you need slightly cooling
   it can be set here */
#define BOARD_FAN_MIN_SPEED 0





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
