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

#ifndef _EEPROM_H
#define _EEPROM_H

#include "Printer.h"

// Id to distinguish version changes
#define EEPROM_PROTOCOL_VERSION 19

/** Where to start with our data block in memory. Can be moved if you
    have problems with other modules using the eeprom */

#define EPR_MAGIC_BYTE              0
#define EPR_ACCELERATION_TYPE       1
#define EPR_XAXIS_STEPS_PER_MM      3
#define EPR_YAXIS_STEPS_PER_MM      7
#define EPR_ZAXIS_STEPS_PER_MM     11
#define EPR_MAX_JERK               39
//#define EPR_OPS_MIN_DISTANCE       43
#define EPR_MAX_ZJERK              47
#define EPR_X_MAX_ACCEL            51
#define EPR_Y_MAX_ACCEL            55
#define EPR_Z_MAX_ACCEL            59
#define EPR_X_MAX_TRAVEL_ACCEL     63
#define EPR_Y_MAX_TRAVEL_ACCEL     67
#define EPR_Z_MAX_TRAVEL_ACCEL     71
#define EPR_BAUDRATE               75
#define EPR_MAX_INACTIVE_TIME      79
#define EPR_STEPPER_INACTIVE_TIME  83
//#define EPR_OPS_RETRACT_DISTANCE   87
#define EPR_EXTRUDER_SPEED         95
//#define EPR_OPS_MOVE_AFTER         99
//#define EPR_OPS_MODE              103
#define EPR_INTEGRITY_BYTE        104   // Here the xored sum over eeprom is stored
#define EPR_VERSION               105   // Version id for updates in EEPROM storage
#define EPR_BED_HEAT_MANAGER      106
#define EPR_BED_DRIVE_MAX         107
#define EPR_BED_PID_PGAIN         108
#define EPR_BED_PID_IGAIN         112
#define EPR_BED_PID_DGAIN         116
#define EPR_BED_PID_MAX           120
#define EPR_BED_DRIVE_MIN         124
#define EPR_PRINTING_TIME         125  // Time in seconds printing
#define EPR_PRINTING_DISTANCE     129  // Filament length printed
#define EPR_X_HOME_OFFSET         133
#define EPR_Y_HOME_OFFSET         137
#define EPR_Z_HOME_OFFSET         141
#define EPR_X_LENGTH              145
#define EPR_Y_LENGTH              149
#define EPR_Z_LENGTH              153

#define EPR_Z_PROBE_X_OFFSET      800
#define EPR_Z_PROBE_Y_OFFSET      804
#define EPR_Z_PROBE_HEIGHT        808
#define EPR_Z_PROBE_SPEED         812
#define EPR_Z_PROBE_X1            816
#define EPR_Z_PROBE_Y1            820
#define EPR_Z_PROBE_X2            824
#define EPR_Z_PROBE_Y2            828
#define EPR_Z_PROBE_X3            832
#define EPR_Z_PROBE_Y3            836
#define EPR_Z_PROBE_XY_SPEED      840
#define EPR_AUTOLEVEL_MATRIX      844
#define EPR_AUTOLEVEL_ACTIVE      880
#define EPR_DELTA_DIAGONAL_ROD_LENGTH 881
#define EPR_DELTA_HORIZONTAL_RADIUS 885
#define EPR_DELTA_SEGMENTS_PER_SECOND_PRINT 889
#define EPR_DELTA_SEGMENTS_PER_SECOND_MOVE 891
#define EPR_DELTA_TOWERX_OFFSET_STEPS 893
#define EPR_DELTA_TOWERY_OFFSET_STEPS 895
#define EPR_DELTA_TOWERZ_OFFSET_STEPS 897
#define EPR_DELTA_ALPHA_A         901
#define EPR_DELTA_ALPHA_B         905
#define EPR_DELTA_ALPHA_C         909
#define EPR_DELTA_RADIUS_CORR_A   913
#define EPR_DELTA_RADIUS_CORR_B   917
#define EPR_DELTA_RADIUS_CORR_C   921
#define EPR_DELTA_MAX_RADIUS      925
#define EPR_Z_PROBE_BED_DISTANCE  929
#define EPR_DELTA_DIAGONAL_CORRECTION_A 933
#define EPR_DELTA_DIAGONAL_CORRECTION_B 937
#define EPR_DELTA_DIAGONAL_CORRECTION_C 941
#define EPR_TOUCHSCREEN           946 // - 975 = 30 byte for touchscreen calibration data


#define EPR_DISTORTION_CORRECTION_ENABLED      988
#define EPR_RETRACTION_LENGTH                  992
#define EPR_RETRACTION_LONG_LENGTH             996
#define EPR_RETRACTION_SPEED                  1000
#define EPR_RETRACTION_Z_LIFT                 1004
#define EPR_RETRACTION_UNDO_EXTRA_LENGTH      1008
#define EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH 1012
#define EPR_RETRACTION_UNDO_SPEED             1016
#define EPR_ACCELERATION_FACTOR_TOP           1032
#define EPR_BENDING_CORRECTION_A              1036
#define EPR_BENDING_CORRECTION_B              1040
#define EPR_BENDING_CORRECTION_C              1044
#define EPR_BED_PREHEAT_TEMP                  1048
#define EPR_PARK_X						      1056
#define EPR_PARK_Y                            1060
#define EPR_PARK_Z                            1064



#define EEPROM_FLOAT(x) HAL::eprGetFloat(EPR_##x)
#define EEPROM_INT32(x) HAL::eprGetInt32(EPR_##x)
#define EEPROM_BYTE(x) HAL::eprGetByte(EPR_##x)
#define EEPROM_SET_BYTE(x,val) HAL::eprSetByte(EPR_##x,val)

#define EEPROM_EXTRUDER_OFFSET 200
// bytes per extruder needed, leave some space for future development
#define EEPROM_EXTRUDER_LENGTH 100
// Extruder positions relative to extruder start
#define EPR_EXTRUDER_STEPS_PER_MM        0
#define EPR_EXTRUDER_MAX_FEEDRATE        4
// Feedrate from halted extruder in mm/s
#define EPR_EXTRUDER_MAX_START_FEEDRATE  8
// Acceleration in mm/s^2
#define EPR_EXTRUDER_MAX_ACCELERATION   12
#define EPR_EXTRUDER_HEAT_MANAGER       16
#define EPR_EXTRUDER_DRIVE_MAX          17
#define EPR_EXTRUDER_PID_PGAIN          18
#define EPR_EXTRUDER_PID_IGAIN          22
#define EPR_EXTRUDER_PID_DGAIN          26
#define EPR_EXTRUDER_DEADTIME EPR_EXTRUDER_PID_PGAIN
#define EPR_EXTRUDER_PID_MAX            30
#define EPR_EXTRUDER_X_OFFSET           31
#define EPR_EXTRUDER_Y_OFFSET           35
#define EPR_EXTRUDER_WATCH_PERIOD       39
#define EPR_EXTRUDER_ADVANCE_K          41
#define EPR_EXTRUDER_DRIVE_MIN          45
#define EPR_EXTRUDER_ADVANCE_L          46
#define EPR_EXTRUDER_WAIT_RETRACT_TEMP 50
#define EPR_EXTRUDER_WAIT_RETRACT_UNITS 52
#define EPR_EXTRUDER_COOLER_SPEED       54
// 55-57 free for byte sized parameter
#define EPR_EXTRUDER_MIXING_RATIOS  58 // 16*2 byte ratios = 32 byte -> end = 89
#define EPR_EXTRUDER_Z_OFFSET            90
#define EPR_EXTRUDER_PREHEAT             94 // maybe better temperature
#ifndef Z_PROBE_BED_DISTANCE
#define Z_PROBE_BED_DISTANCE 5.0
#endif

class EEPROM
{
  static void writeExtruderPrefix(uint16_t pos);
  static void writeFloat(uint16_t pos,PGM_P text,uint8_t digits = 3);
  static void writeLong(uint16_t pos,PGM_P text);
  static void writeInt(uint16_t pos,PGM_P text);
  static void writeByte(uint16_t pos,PGM_P text);
public:
  static uint8_t computeChecksum();
  static void updateChecksum();

public:
  static void init();
  static void initBaudrate();

  static void storeDataIntoEEPROM(uint8_t corrupted = 0);
  static void readDataFromEEPROM(bool includeExtruder);

  static void writeSettings();
  static void update(gcodeCommand *com);
  static void updatePrinterUsage();
  static inline void setVersion(uint8_t v) {
    HAL::eprSetByte(EPR_VERSION,v);
    HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
  }

#if FEATURE_Z_PROBE
  static inline void setZProbeHeight(float mm) {
    HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT, mm);
    Com::printF(PSTR("Z-Probe height set to: "),mm,3);
    Com::printF(PSTR("\n"));
    EEPROM::updateChecksum();
  }
#endif
    
  static inline float zProbeSpeed() {
    return HAL::eprGetFloat(EPR_Z_PROBE_SPEED);
  }
  static inline float zProbeXYSpeed() {
    return HAL::eprGetFloat(EPR_Z_PROBE_XY_SPEED);
  }
  static inline float zProbeXOffset() {
    return HAL::eprGetFloat(EPR_Z_PROBE_X_OFFSET);
  }
  static inline float zProbeYOffset() {
    return HAL::eprGetFloat(EPR_Z_PROBE_Y_OFFSET);
  }
  static inline float zProbeHeight() {
    return HAL::eprGetFloat(EPR_Z_PROBE_HEIGHT);
  }
  static inline float zProbeX1() {
    return HAL::eprGetFloat(EPR_Z_PROBE_X1);
  }
  static inline float zProbeY1() {
    return HAL::eprGetFloat(EPR_Z_PROBE_Y1);
  }
  static inline float zProbeX2() {
    return HAL::eprGetFloat(EPR_Z_PROBE_X2);
  }
  static inline float zProbeY2() {
    return HAL::eprGetFloat(EPR_Z_PROBE_Y2);
  }
  static inline float zProbeX3() {
    return HAL::eprGetFloat(EPR_Z_PROBE_X3);
  }
  static inline float zProbeY3() {
    return HAL::eprGetFloat(EPR_Z_PROBE_Y3);
  }
  static inline float zProbeBedDistance() {
    return HAL::eprGetFloat(EPR_Z_PROBE_BED_DISTANCE);
  }


  static inline int16_t deltaSegmentsPerSecondMove() {
    return HAL::eprGetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE);
  }
  static inline float deltaDiagonalRodLength() {
    return HAL::eprGetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH);
  }
  static inline int16_t deltaSegmentsPerSecondPrint() {
    return HAL::eprGetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT);
  }

  static inline float deltaHorizontalRadius() {
    return HAL::eprGetFloat(EPR_DELTA_HORIZONTAL_RADIUS);
  }
  static inline int16_t deltaTowerXOffsetSteps() {
    return HAL::eprGetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS);
  }
  static inline int16_t deltaTowerYOffsetSteps() {
    return HAL::eprGetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS);
  }
  static inline int16_t deltaTowerZOffsetSteps() {
    return HAL::eprGetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS);
  }

  static inline void setRodRadius(float mm) {
    Printer::radius0=mm;
    Printer::updateDerivedParameter();
    //This is an odd situation, the radius can only be changed if eeprom is on.
    // The radius is not saved to printer variable now, it is all derived parameters of
    // fetching the radius, which if EEProm is off returns the Configuration constant.
    HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, mm);
    EEPROM::updateChecksum();
  }
  static inline void incrementRodRadius(float mm) {
    setRodRadius(mm + deltaHorizontalRadius());
  }
  static inline void setTowerXFloor(float newZ) {
    Printer::xMin = newZ;
    Printer::updateDerivedParameter();
    //Com::printF(PSTR("X (A) tower floor set to: "),Printer::xMin,3);
    //Com::printF(PSTR("\n"));
    HAL::eprSetFloat(EPR_X_HOME_OFFSET,Printer::xMin);
    EEPROM::updateChecksum();
  }
  static inline void setTowerYFloor(float newZ) {
    Printer::yMin = newZ;
    Printer::updateDerivedParameter();
    //Com::printF(PSTR("Y (B) tower floor set to: "), Printer::yMin, 3);
    //Com::printF(PSTR("\n"));
    HAL::eprSetFloat(EPR_Y_HOME_OFFSET,Printer::yMin);
    EEPROM::updateChecksum();
  }
  static inline void setTowerZFloor(float newZ) {
    Printer::zMin = newZ;
    Printer::updateDerivedParameter();
    //Com::printF(PSTR("Z (C) tower floor set to: "), Printer::zMin, 3);
    //Com::printF(PSTR("\n"));
    HAL::eprSetFloat(EPR_Z_HOME_OFFSET,Printer::zMin);
    EEPROM::updateChecksum();
  }
  static inline void setDeltaTowerXOffsetSteps(int16_t steps) {
    HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,steps);
    EEPROM::updateChecksum();
  }
  static inline void setDeltaTowerYOffsetSteps(int16_t steps) {
    HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,steps);
    EEPROM::updateChecksum();
  }
  static inline void setDeltaTowerZOffsetSteps(int16_t steps) {
    HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,steps);
    EEPROM::updateChecksum();
  }
  static inline float deltaAlphaA() {
    return HAL::eprGetFloat(EPR_DELTA_ALPHA_A);
  }
  static inline float deltaAlphaB() {
    return HAL::eprGetFloat(EPR_DELTA_ALPHA_B);
  }
  static inline float deltaAlphaC() {
    return HAL::eprGetFloat(EPR_DELTA_ALPHA_C);
  }
  static inline float deltaRadiusCorrectionA() {
    return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_A);
  }
  static inline float deltaRadiusCorrectionB() {
    return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_B);
  }
  static inline float deltaRadiusCorrectionC() {
    return HAL::eprGetFloat(EPR_DELTA_RADIUS_CORR_C);
  }
  static inline float deltaDiagonalCorrectionA() {
    return EEPROM_FLOAT(DELTA_DIAGONAL_CORRECTION_A);
  }
  static inline float deltaDiagonalCorrectionB() {
    return EEPROM_FLOAT(DELTA_DIAGONAL_CORRECTION_B);
  }
  static inline float deltaDiagonalCorrectionC() {
    return EEPROM_FLOAT(DELTA_DIAGONAL_CORRECTION_C);
  }
  static inline float deltaMaxRadius() {
    return EEPROM_FLOAT(DELTA_MAX_RADIUS);
  }

  static void initalizeUncached();

  static void setZCorrection(int32_t c,int index);
  static inline int32_t getZCorrection(int index) {
    return HAL::eprGetInt32(2048 + (index << 2));
  }
  static inline void setZCorrectionEnabled(int8_t on) {
    if(isZCorrectionEnabled() == on) return;
    HAL::eprSetInt16(EPR_DISTORTION_CORRECTION_ENABLED, on);
    EEPROM::updateChecksum();
  }
  static inline int8_t isZCorrectionEnabled() {
    return HAL::eprGetByte(EPR_DISTORTION_CORRECTION_ENABLED);
  }
  static inline float bendingCorrectionA() {
    return HAL::eprGetFloat(EPR_BENDING_CORRECTION_A);
  }
  static inline float bendingCorrectionB() {
    return HAL::eprGetFloat(EPR_BENDING_CORRECTION_B);
  }
  static inline float bendingCorrectionC() {
    return HAL::eprGetFloat(EPR_BENDING_CORRECTION_C);
  }
  static inline float accelarationFactorTop() {
    return HAL::eprGetFloat(EPR_ACCELERATION_FACTOR_TOP);
  }
  static inline float parkX() {
    return HAL::eprGetFloat(EPR_PARK_X);
  }
  static inline float parkY() {
    return HAL::eprGetFloat(EPR_PARK_Y);
  }
  static inline float parkZ() {
    return HAL::eprGetFloat(EPR_PARK_Z);
  }

};
#endif
