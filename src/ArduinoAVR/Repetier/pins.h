#ifndef PINS_H
#define PINS_H

//  Pin assignments for RAMBO.

#ifndef __AVR_ATmega2560__
#error Oops!  Make sure you have 'Arduino Mega 2560' selected from the 'Tools -> Boards' menu.
#endif

#define X_STEP_PIN     37
#define X_DIR_PIN      48
#define X_MIN_PIN      12
#define X_MAX_PIN      24
#define X_ENABLE_PIN   29
#define X_MS1_PIN      40
#define X_MS2_PIN      41

#define Y_STEP_PIN     36
#define Y_DIR_PIN      49
#define Y_MIN_PIN      11
#define Y_MAX_PIN      23
#define Y_ENABLE_PIN   28
#define Y_MS1_PIN      69
#define Y_MS2_PIN      39

#define Z_STEP_PIN     35
#define Z_DIR_PIN      47
#define Z_MIN_PIN      10
#define Z_MAX_PIN      30
#define Z_ENABLE_PIN   27
#define Z_MS1_PIN      68
#define Z_MS2_PIN      67

//  These are digital pin numbers.
#define HEATER_0_PIN    9  //  "Ext 0 Heat 0" on board.
#define HEATER_1_PIN    7  //  USED FOR FAN!
#define BED_HEAT_PIN    3  //  "Heated Bed" on board (confirmed)

#define FAN_0_PIN       8  //  Board Pin 17 PH4 OC4C - Digital Pin 8 - "Ext 0 Fan" on board - Layer fan
#define FAN_1_PIN       6  //  Board Pin 15 PH3 OC4A - Digital Pin 6 - "Ext 1 Fan" on board - unused
#define FAN_2_PIN       2  //  Board Pin  6 PE4 OC3B - Digital Pin 2 - "Fan 2" on board

//  These are analog pin numbers.
#define TEMP_0_PIN      0  //  ADC0/PF0 - T0 om the board - Hotend ()
#define TEMP_1_PIN      1  //  ADC2/PF2 - T1 on the board -
#define TEMP_2_PIN      2  //  ADC1/PF1 - T2 on the board - Bed    (confirmed)
#define TEMP_3_PIN      7  //  ADC7/PF7 - T3 on the board -

#define E0_STEP_PIN    34
#define E0_DIR_PIN     43
#define E0_ENABLE_PIN  26
#define E0_MS1_PIN     65
#define E0_MS2_PIN     66

#define E1_STEP_PIN    33
#define E1_DIR_PIN     42
#define E1_ENABLE_PIN  25
#define E1_MS1_PIN     63
#define E1_MS2_PIN     64

#define DIGIPOTSS_PIN  38
#define DIGIPOT_CHANNELS {4,5,3,0,1} // X Y Z E0 E1 digipot channels to stepper driver mapping

#define DIGIPOT_X_CH    4
#define DIGIPOT_Y_CH    5
#define DIGIPOT_Z_CH    3
#define DIGIPOT_E0_CH   0
#define DIGIPOT_E1_CH   1

#define SDCARDDETECT   81

#define SDSS           53
#define LED_PIN        13
#define PS_ON_PIN       4

#define E0_PINS E0_STEP_PIN,E0_DIR_PIN,E0_ENABLE_PIN,E0_MS1_PIN,E0_MS2_PIN,
#define E1_PINS

#define SCK_PIN        52
#define MISO_PIN       50
#define MOSI_PIN       51
#define MAX6675_SS     53

#define BEEPER_PIN     79

#endif

