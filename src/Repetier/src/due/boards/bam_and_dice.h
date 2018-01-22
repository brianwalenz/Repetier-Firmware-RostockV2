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

/*****************************************************************
 * BAM&DICE Due Board with Arduino Due
 * http://www.2printbeta.de
 ******************************************************************/

#if MOTHERBOARD == 406
#ifndef __SAM3X8E__
#error Oops!  Make sure you have 'Arduino Due' selected from the 'Tools -> Boards' menu.
#endif

#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM

#define STEPPER_CURRENT_CONTROL  CURRENT_CONTROL_MANUAL

/*****************************************************************
* Arduino Due Pin Assignments
******************************************************************/

#define ORIG_X_STEP_PIN     54
#define ORIG_X_DIR_PIN      55
#define ORIG_X_MIN_PIN      3
#define ORIG_X_MAX_PIN      2
#define ORIG_X_ENABLE_PIN   38

#define ORIG_Y_STEP_PIN     60 
#define ORIG_Y_DIR_PIN      61
#define ORIG_Y_MIN_PIN      43
#define ORIG_Y_MAX_PIN      45
#define ORIG_Y_ENABLE_PIN   56

#define ORIG_Z_STEP_PIN     46
#define ORIG_Z_DIR_PIN      48
#define ORIG_Z_MIN_PIN      40
#define ORIG_Z_MAX_PIN      42
#define ORIG_Z_ENABLE_PIN   62

// Note that on the Due pin A0 on the board is channel 2 on the ARM chip
#define HEATER_0_PIN     10
// Due analog pin - SAM addressing (not arduino)
#define TEMP_0_PIN       11 

#define HEATER_1_PIN     8 
// Due analog pin - SAM addressing (not arduino)
#define TEMP_1_PIN       12 

#define HEATER_2_PIN     -1
#define TEMP_2_PIN       -1
#define HEATER_3_PIN     -1
#define TEMP_3_PIN       -1
#define HEATER_4_PIN     -1
#define TEMP_4_PIN       -1

#define ORIG_E0_STEP_PIN    26
#define ORIG_E0_DIR_PIN     28
#define ORIG_E0_ENABLE_PIN  24

#define ORIG_E1_STEP_PIN    36
#define ORIG_E1_DIR_PIN     34
#define ORIG_E1_ENABLE_PIN  30

#define SDSUPPORT      true
#define SDPOWER 	   -1
// 4,10,52 if using HW SPI.
#define SDSS		   53 
//#define SDSS		   -1
//#define SDCARDDETECT   -1
#define SDCARDDETECTINVERTED 0
#define LED_PIN 	   13
#define ORIG_FAN_PIN 	   9
#define ORIG_PS_ON_PIN    -1
#define KILL_PIN	   41
#define SUICIDE_PIN    -1  //PIN that has to be turned on right after start, to keep power flowing.

// 20 or 70
#define SDA_PIN 				20  	
// 21 or 71
#define SCL_PIN 				21  	


#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ          100000
// see eeprom device data sheet for the following values these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        128     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// specify size of eeprom address register
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE EEPROM_I2C
#endif

// Smart RAMPS without(408) and with EEPROM (413)
#if MOTHERBOARD == 408 || MOTHERBOARD == 413
#ifndef __SAM3X8E__
#erro oops! Be sure to have 'due Arduino' selected from the 'tools-> Boards menu'.
#endif
 
#define KNOWN_BOARD
#define CPU_ARCH ARCH_ARM
/*****************************************************************
* Arduino Due pin assignments
******************************************************************/
 
#define ORIG_X_STEP_PIN 54
#define ORIG_X_DIR_PIN 55
#define ORIG_X_MIN_PIN 3
#define ORIG_X_MAX_PIN 2
#define ORIG_X_ENABLE_PIN 38
 
#define ORIG_Y_STEP_PIN 60 
#define ORIG_Y_DIR_PIN 61
#define ORIG_Y_MIN_PIN 14
#define ORIG_Y_MAX_PIN 15
#define ORIG_Y_ENABLE_PIN 56
 
#define ORIG_Z_STEP_PIN 46
#define ORIG_Z_DIR_PIN 48
#define ORIG_Z_MIN_PIN 18
#define ORIG_Z_MAX_PIN 19
#define ORIG_Z_ENABLE_PIN 62
 
//Note that in due A0 pins on the board is channel 2 on the ARM chip
#define HEATER_0_PIN 10
// Because analog pin #
#define TEMP_0_PIN 11 
#define HEATER_1_PIN 8
// Because analog pin #
#define TEMP_1_PIN 12 
#define HEATER_2_PIN 9
// Because analog pin #
#define TEMP_2_PIN 13 
 
#define ORIG_E0_STEP_PIN 26
#define ORIG_E0_DIR_PIN 28
#define ORIG_E0_ENABLE_PIN 24
 
#define ORIG_E1_STEP_PIN 36
#define ORIG_E1_DIR_PIN 34
#define ORIG_E1_ENABLE_PIN 30
 
#define SDPOWER   -1
// 10 if using HW spi. 53 if using SW SPI
#define SDSS 53 
#define LED_PIN   13
#define ORIG_FAN_PIN   9
#define ORIG_PS_ON_PIN 12
#define KILL_PIN   -1
#define SUICIDE_PIN -1// Pin that has to be turned right after the start, to keep the power flowing.

#define E0_PINS ORIG_E0_STEP_PIN,ORIG_E0_DIR_PIN,ORIG_E0_ENABLE_PIN,
#define E1_PINS ORIG_E1_STEP_PIN,ORIG_E1_DIR_PIN,ORIG_E1_ENABLE_PIN,

#define TWI_CLOCK_FREQ          100000
#if MOTHERBOARD == 408
//	20 or 70
#define SDA_PIN 			-1 
// 21 or 71
#define SCL_PIN 			-1 
#define EEPROM_AVAILABLE EEPROM_NONE
#else
// this board supports eeprom
#define SDA_PIN     20      // 20 or 70
#define SCL_PIN     21      // 21 or 71

#define TWI_CLOCK_FREQ          400000
// see eeprom device data sheet for the following values, these are for 24xx256
#define EEPROM_SERIAL_ADDR      0x50   // 7 bit i2c address (without R/W bit)
#define EEPROM_PAGE_SIZE        64     // page write buffer size
#define EEPROM_PAGE_WRITE_TIME  7      // page write time in milliseconds (docs say 5ms but that is too short)
// TWI_MMR_IADRSZ_1_BYTE for 1 byte, or TWI_MMR_IADRSZ_2_BYTE for 2 byte
#define EEPROM_ADDRSZ_BYTES     TWI_MMR_IADRSZ_2_BYTE
#define EEPROM_AVAILABLE 1
#endif
#endif