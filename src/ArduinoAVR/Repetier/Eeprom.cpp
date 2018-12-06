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
#include "gcode.h"
#include "motion.h"
#include "Commands.h"
#include "Communication.h"
#include "Eeprom.h"
#include "Extruder.h"
#include "temperatures.h"

void EEPROM::update(gcodeCommand *com)
{
  if(com->hasT() && com->hasP())
    switch(com->T)
      {
        case 0:
          if(com->hasS())
            eprSetByte(com->P, (uint8_t)com->S);
          break;
        case 1:
          if(com->hasS())
            eprSetInt16(com->P, (int16_t)com->S);
          break;
        case 2:
          if(com->hasS())
            eprSetInt32(com->P, (int32_t)com->S);
          break;
        case 3:
          if(com->hasX())
            eprSetFloat(com->P, com->X);
          break;
      }

  uint8_t newcheck = computeChecksum();
  if(newcheck != eprGetByte(EPR_INTEGRITY_BYTE))
    eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
  bool includesEeprom = com->P >= EEPROM_EXTRUDER_OFFSET && com->P < EEPROM_EXTRUDER_OFFSET + 6 * EEPROM_EXTRUDER_LENGTH;
  readDataFromEEPROM(includesEeprom);

  Printer::selectExtruderById(0);
}





void EEPROM::storeDataIntoEEPROM(uint8_t corrupted)
{
  eprSetInt32(EPR_MAX_INACTIVE_TIME,maxInactiveTime);
  eprSetInt32(EPR_STEPPER_INACTIVE_TIME,stepperInactiveTime);
  //#define EPR_ACCELERATION_TYPE 1
  eprSetFloat(EPR_XAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[X_AXIS]);
  eprSetFloat(EPR_YAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Y_AXIS]);
  eprSetFloat(EPR_ZAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Z_AXIS]);

  eprSetFloat(EPR_MAX_JERK,Printer::maxJerk);
#if RAMP_ACCELERATION
  eprSetFloat(EPR_X_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
  eprSetFloat(EPR_Y_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
  eprSetFloat(EPR_Z_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
  eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
  eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
  eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#endif

  //eprSetByte(EPR_BED_DRIVE_MAX,bedTemp._pidDriveMax);
  //eprSetByte(EPR_BED_DRIVE_MIN,bedTemp._pidDriveMin);
  eprSetFloat(EPR_BED_PID_PGAIN,bedTemp._pidPGain);
  eprSetFloat(EPR_BED_PID_IGAIN,bedTemp._pidIGain);
  eprSetFloat(EPR_BED_PID_DGAIN,bedTemp._pidDGain);
  //eprSetByte(EPR_BED_PID_MAX,bedTemp._pidMax);

  eprSetFloat(EPR_X_HOME_OFFSET,Printer::xMin);
  eprSetFloat(EPR_Y_HOME_OFFSET,Printer::yMin);
  eprSetFloat(EPR_Z_HOME_OFFSET,Printer::zMin);
  eprSetFloat(EPR_X_LENGTH,Printer::xLength);
  eprSetFloat(EPR_Y_LENGTH,Printer::yLength);
  eprSetFloat(EPR_Z_LENGTH,Printer::zLength);
  eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, Printer::radius0);

#if FEATURE_AUTOLEVEL
  eprSetByte(EPR_AUTOLEVEL_ACTIVE,Printer::isAutolevelActive());
  for(uint8_t i = 0; i < 9; i++)
    eprSetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2),Printer::autolevelTransformation[i]);
#endif

  hal.pingWatchdog();

  // now the extruder
  int o = 0 * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;

  Extruder *e = &extruder;

  eprSetFloat(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
  eprSetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
  eprSetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
  eprSetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);

  //eprSetByte(o+EPR_EXTRUDER_DRIVE_MAX,    extruderTemp._pidDriveMax);
  //eprSetByte(o+EPR_EXTRUDER_DRIVE_MIN,    extruderTemp._pidDriveMin);
  eprSetFloat(o+EPR_EXTRUDER_PID_PGAIN,   extruderTemp._pidPGain);
  eprSetFloat(o+EPR_EXTRUDER_PID_IGAIN,   extruderTemp._pidIGain);
  eprSetFloat(o+EPR_EXTRUDER_PID_DGAIN,   extruderTemp._pidDGain);
  //eprSetByte(o+EPR_EXTRUDER_PID_MAX,      extruderTemp._pidMax);

  eprSetInt32(o+EPR_EXTRUDER_X_OFFSET,e->xOffset);
  eprSetInt32(o+EPR_EXTRUDER_Y_OFFSET,e->yOffset);
  eprSetInt32(o+EPR_EXTRUDER_Z_OFFSET,e->zOffset);
  eprSetInt16(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);
#if RETRACT_DURING_HEATUP
  eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,e->waitRetractTemperature);
  eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,e->waitRetractUnits);
#else
  eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_TEMP);
  eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,EXT0_WAIT_RETRACT_UNITS);
#endif
  eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
  eprSetFloat(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);

  if(corrupted)
    {
      eprSetInt32(EPR_PRINTING_TIME,0);
      eprSetFloat(EPR_PRINTING_DISTANCE,0);
      initalizeUncached();
    }
  // Save version and build checksum
  eprSetByte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
  eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
}





void EEPROM::initalizeUncached()
{
  eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,DELTA_SEGMENTS_PER_SECOND_PRINT);
  eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,DELTA_SEGMENTS_PER_SECOND_MOVE);
  eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH,DELTA_DIAGONAL_ROD);
  eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS,ROD_RADIUS);
  eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,DELTA_X_ENDSTOP_OFFSET_STEPS);
  eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,DELTA_Y_ENDSTOP_OFFSET_STEPS);
  eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,DELTA_Z_ENDSTOP_OFFSET_STEPS);
  eprSetFloat(EPR_DELTA_ALPHA_A,DELTA_ALPHA_A);
  eprSetFloat(EPR_DELTA_ALPHA_B,DELTA_ALPHA_B);
  eprSetFloat(EPR_DELTA_ALPHA_C,DELTA_ALPHA_C);
  eprSetFloat(EPR_DELTA_RADIUS_CORR_A,DELTA_RADIUS_CORRECTION_A);
  eprSetFloat(EPR_DELTA_RADIUS_CORR_B,DELTA_RADIUS_CORRECTION_B);
  eprSetFloat(EPR_DELTA_RADIUS_CORR_C,DELTA_RADIUS_CORRECTION_C);
  eprSetFloat(EPR_DELTA_MAX_RADIUS,DELTA_MAX_RADIUS);
  eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,DELTA_DIAGONAL_CORRECTION_A);
  eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,DELTA_DIAGONAL_CORRECTION_B);
  eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,DELTA_DIAGONAL_CORRECTION_C);
  eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED,0);

  eprSetFloat(EPR_RETRACTION_LENGTH,RETRACTION_LENGTH);
  eprSetFloat(EPR_RETRACTION_LONG_LENGTH,RETRACTION_LONG_LENGTH);
  eprSetFloat(EPR_RETRACTION_SPEED,RETRACTION_SPEED);
  eprSetFloat(EPR_RETRACTION_Z_LIFT,RETRACTION_Z_LIFT);
  eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,RETRACTION_UNDO_EXTRA_LENGTH);
  eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,RETRACTION_UNDO_EXTRA_LONG_LENGTH);
  eprSetFloat(EPR_RETRACTION_UNDO_SPEED,RETRACTION_UNDO_SPEED);

  eprSetFloat(EPR_PARK_X,PARK_POSITION_X);
  eprSetFloat(EPR_PARK_Y,PARK_POSITION_Y);
  eprSetFloat(EPR_PARK_Z,PARK_POSITION_Z_RAISE);
}




void EEPROM::readDataFromEEPROM(bool includeExtruder)
{
  uint8_t version = eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data nor set it to older versions!

  Com::printF(PSTR("Detected EEPROM version: "));
  Com::print(version);
  Com::printF(PSTR("\n"));

  maxInactiveTime = eprGetInt32(EPR_MAX_INACTIVE_TIME);
  stepperInactiveTime = eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
  //#define EPR_ACCELERATION_TYPE 1
  Printer::axisStepsPerMM[X_AXIS] = eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
  Printer::axisStepsPerMM[Y_AXIS] = eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
  Printer::axisStepsPerMM[Z_AXIS] = eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);

  Printer::maxJerk = eprGetFloat(EPR_MAX_JERK);
#if RAMP_ACCELERATION
  Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = eprGetFloat(EPR_X_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = eprGetFloat(EPR_Y_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = eprGetFloat(EPR_Z_MAX_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL);
#endif

#if 0
  bedTemp._pidDriveMax = eprGetByte(EPR_BED_DRIVE_MAX);
  bedTemp._pidDriveMin = eprGetByte(EPR_BED_DRIVE_MIN);
  bedTemp._pidPGain = eprGetFloat(EPR_BED_PID_PGAIN);
  bedTemp._pidIGain = eprGetFloat(EPR_BED_PID_IGAIN);
  bedTemp._pidDGain = eprGetFloat(EPR_BED_PID_DGAIN);
  bedTemp._pidMax = eprGetByte(EPR_BED_PID_MAX);

  Com::printf(PSTR("bedTemp._pidDriveMax %d\n"), bedTemp._pidDriveMax);
  Com::printf(PSTR("bedTemp._pidDriveMin %d\n"), bedTemp._pidDriveMin);
  Com::printf(PSTR("bedTemp._pidPGain    %f\n"), bedTemp._pidPGain);
  Com::printf(PSTR("bedTemp._pidIGain    %f\n"), bedTemp._pidIGain);
  Com::printf(PSTR("bedTemp._pidDGain    %f\n"), bedTemp._pidDGain);
  Com::printf(PSTR("bedTemp._pidMax      %d\n"), bedTemp._pidMax);
#endif

  Printer::xMin = eprGetFloat(EPR_X_HOME_OFFSET);
  Printer::yMin = eprGetFloat(EPR_Y_HOME_OFFSET);
  Printer::zMin = eprGetFloat(EPR_Z_HOME_OFFSET);
  Printer::xLength = eprGetFloat(EPR_X_LENGTH);
  Printer::yLength = eprGetFloat(EPR_Y_LENGTH);
  Printer::zLength = eprGetFloat(EPR_Z_LENGTH);
  Printer::radius0 = eprGetFloat(EPR_DELTA_HORIZONTAL_RADIUS);

#if FEATURE_AUTOLEVEL
  if(version > 2)
    {
      float sum = 0;
      for(uint8_t i = 0; i < 9; i++)
        Printer::autolevelTransformation[i] = eprGetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2));
      if(isnan(Printer::autolevelTransformation[0]))   // a bug caused storage of matrix at the wrong place. Read from old position instead.
        {
          for(uint8_t i = 0; i < 9; i++)
            Printer::autolevelTransformation[i] = eprGetFloat((EPR_AUTOLEVEL_MATRIX + (int)i) << 2);
        }
      for(uint8_t i = 0; i < 9; i++)
        {
          if(isnan(Printer::autolevelTransformation[i]))
            sum += 10;
          else
            sum += RMath::sqr(Printer::autolevelTransformation[i]);
        }
      if(sum < 2.7 || sum > 3.3)
        Printer::resetTransformationMatrix(false);
      Printer::setAutolevelActive(eprGetByte(EPR_AUTOLEVEL_ACTIVE));
      Com::printArrayF(PSTR("Transformation matrix:"),Printer::autolevelTransformation, 9, 6);
    }
#endif
  if(includeExtruder)
    {
      // now the extruder

      hal.pingWatchdog();

      int o = 0 * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;

      Extruder *e = &extruder;

      e->stepsPerMM = eprGetFloat(o+EPR_EXTRUDER_STEPS_PER_MM);
      e->maxFeedrate = eprGetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE);
      e->maxStartFeedrate = eprGetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
      e->maxAcceleration = eprGetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION);

#if 0
      extruderTemp._pidDriveMax = eprGetByte(o+EPR_EXTRUDER_DRIVE_MAX);
      extruderTemp._pidDriveMin = eprGetByte(o+EPR_EXTRUDER_DRIVE_MIN);
      extruderTemp._pidPGain    = eprGetFloat(o+EPR_EXTRUDER_PID_PGAIN);
      extruderTemp._pidIGain    = eprGetFloat(o+EPR_EXTRUDER_PID_IGAIN);
      extruderTemp._pidDGain    = eprGetFloat(o+EPR_EXTRUDER_PID_DGAIN);
      extruderTemp._pidMax      = eprGetByte(o+EPR_EXTRUDER_PID_MAX);

  Com::printf(PSTR("extruderTemp._pidDriveMax %d\n"), extruderTemp._pidDriveMax);
  Com::printf(PSTR("extruderTemp._pidDriveMin %d\n"), extruderTemp._pidDriveMin);
  Com::printf(PSTR("extruderTemp._pidPGain    %f\n"), extruderTemp._pidPGain);
  Com::printf(PSTR("extruderTemp._pidIGain    %f\n"), extruderTemp._pidIGain);
  Com::printf(PSTR("extruderTemp._pidDGain    %f\n"), extruderTemp._pidDGain);
  Com::printf(PSTR("extruderTemp._pidMax      %d\n"), extruderTemp._pidMax);
#endif

      e->xOffset = eprGetInt32(o+EPR_EXTRUDER_X_OFFSET);
      e->yOffset = eprGetInt32(o+EPR_EXTRUDER_Y_OFFSET);
      e->watchPeriod = eprGetInt16(o+EPR_EXTRUDER_WATCH_PERIOD);

#if RETRACT_DURING_HEATUP
      e->waitRetractTemperature = eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP);
      e->waitRetractUnits = eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif

      e->advanceK = eprGetFloat(o+EPR_EXTRUDER_ADVANCE_K);
      e->advanceL = eprGetFloat(o+EPR_EXTRUDER_ADVANCE_L);

      if(version < 13) {
        eprSetInt32(o+EPR_EXTRUDER_Z_OFFSET,e->zOffset);
      }
      e->zOffset = eprGetInt32(o + EPR_EXTRUDER_Z_OFFSET);
    }
  if(version != EEPROM_PROTOCOL_VERSION)
    {
      Com::printF(PSTR("INFO: Protocol version changed, upgrading\n"));
      if(version < 4)
        {
          eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,DELTA_SEGMENTS_PER_SECOND_PRINT);
          eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,DELTA_SEGMENTS_PER_SECOND_MOVE);
          eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH,DELTA_DIAGONAL_ROD);
          eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS,ROD_RADIUS);
          eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,DELTA_X_ENDSTOP_OFFSET_STEPS);
          eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,DELTA_Y_ENDSTOP_OFFSET_STEPS);
          eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,DELTA_Z_ENDSTOP_OFFSET_STEPS);
        }
      if(version < 5)
        {
          eprSetFloat(EPR_DELTA_ALPHA_A,DELTA_ALPHA_A);
          eprSetFloat(EPR_DELTA_ALPHA_B,DELTA_ALPHA_B);
          eprSetFloat(EPR_DELTA_ALPHA_C,DELTA_ALPHA_C);
        }
      if(version < 6)
        {
          eprSetFloat(EPR_DELTA_RADIUS_CORR_A,DELTA_RADIUS_CORRECTION_A);
          eprSetFloat(EPR_DELTA_RADIUS_CORR_B,DELTA_RADIUS_CORRECTION_B);
          eprSetFloat(EPR_DELTA_RADIUS_CORR_C,DELTA_RADIUS_CORRECTION_C);
        }
      if(version < 7)
        {
          eprSetFloat(EPR_DELTA_MAX_RADIUS,DELTA_MAX_RADIUS);
          eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,DELTA_DIAGONAL_CORRECTION_A);
          eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,DELTA_DIAGONAL_CORRECTION_B);
          eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,DELTA_DIAGONAL_CORRECTION_C);
        }
      if(version < 11)
        {
          eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);
        }
      if(version < 12)
        {
          eprSetFloat(EPR_RETRACTION_LENGTH,RETRACTION_LENGTH);
          eprSetFloat(EPR_RETRACTION_LONG_LENGTH,RETRACTION_LONG_LENGTH);
          eprSetFloat(EPR_RETRACTION_SPEED,RETRACTION_SPEED);
          eprSetFloat(EPR_RETRACTION_Z_LIFT,RETRACTION_Z_LIFT);
          eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,RETRACTION_UNDO_EXTRA_LENGTH);
          eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,RETRACTION_UNDO_EXTRA_LONG_LENGTH);
          eprSetFloat(EPR_RETRACTION_UNDO_SPEED,RETRACTION_UNDO_SPEED);
        }

      if(version < 19) {
		    eprSetFloat(EPR_PARK_X,PARK_POSITION_X);
		    eprSetFloat(EPR_PARK_Y,PARK_POSITION_Y);
		    eprSetFloat(EPR_PARK_Z,PARK_POSITION_Z_RAISE);
      }

      storeDataIntoEEPROM(false); // Store new fields for changed version
    }

  Com::printF(PSTR("\n\nSETTINGS\n\n"));
  writeSettings();
  Com::printF(PSTR("\n\nSETTINGS DONE\n\n"));

  delay(200);

  //Printer::updateDerivedParameter();

  //bedTemp.initialize();
}





void EEPROM::init()
{
  uint8_t check = computeChecksum();
  uint8_t storedcheck = eprGetByte(EPR_INTEGRITY_BYTE);
  if(eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check)
    {
      readDataFromEEPROM(true);
    }
  else
    {
      eprSetByte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make data change permanent
      initalizeUncached();
      storeDataIntoEEPROM(storedcheck != check);
    }
}





void EEPROM::updatePrinterUsage()
{
  if(Printer::filamentPrinted == 0 || (Printer::flag2 & PRINTER_FLAG2_RESET_FILAMENT_USAGE) != 0) return; // No miles only enabled
  uint32_t seconds = (millis() - Printer::msecondsPrinting) / 1000;
  seconds += eprGetInt32(EPR_PRINTING_TIME);
  eprSetInt32(EPR_PRINTING_TIME,seconds);
  eprSetFloat(EPR_PRINTING_DISTANCE,eprGetFloat(EPR_PRINTING_DISTANCE) + Printer::filamentPrinted * 0.001);
  Printer::flag2 |= PRINTER_FLAG2_RESET_FILAMENT_USAGE;
  Printer::msecondsPrinting = millis();
  updateChecksum();
  //Commands::reportPrinterUsage();
}





/** \brief Writes all eeprom settings to serial console.

    For each value stored, this function generates one line with syntax

    EPR: pos type value description

    With
    - pos = Position in EEPROM, the data starts.
    - type = Value type: 0 = byte, 1 = int, 2 = long, 3 = float
    - value = The value currently stored
    - description = Definition of the value
*/
void EEPROM::writeSettings()
{
  Com::printf(PSTR("EEPROM DUMP BEGIN\n"));

  writeFloat(EPR_PRINTING_DISTANCE, PSTR("Filament printed [m]"));
  writeLong(EPR_PRINTING_TIME, PSTR("Printer active [s]"));
  writeLong(EPR_MAX_INACTIVE_TIME, PSTR("Max. inactive time [ms,0=off]"));
  writeLong(EPR_STEPPER_INACTIVE_TIME, PSTR("Stop stepper after inactivity [ms,0=off]"));
  //#define EPR_ACCELERATION_TYPE 1
  writeFloat(EPR_ZAXIS_STEPS_PER_MM, PSTR("Steps per mm"), 4);
  writeFloat(EPR_MAX_JERK, PSTR("Max. jerk [mm/s]"));
  writeFloat(EPR_X_HOME_OFFSET, PSTR("X min pos [mm]"));
  writeFloat(EPR_Y_HOME_OFFSET, PSTR("Y min pos [mm]"));
  writeFloat(EPR_Z_HOME_OFFSET, PSTR("Z min pos [mm]"));
  writeFloat(EPR_X_LENGTH, PSTR("X max length [mm]"));
  writeFloat(EPR_Y_LENGTH, PSTR("Y max length [mm]"));
  writeFloat(EPR_Z_LENGTH, PSTR("Z max length [mm]"));
	writeFloat(EPR_PARK_X, PSTR("Park position X [mm]"));
	writeFloat(EPR_PARK_Y, PSTR("Park position Y [mm]"));
	writeFloat(EPR_PARK_Z, PSTR("Park position Z raise [mm]"));

  writeInt(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE, PSTR("Segments/s for travel"));
  writeInt(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT, PSTR("Segments/s for printing"));
#if RAMP_ACCELERATION
  //epr_out_float(EPR_X_MAX_START_SPEED,PSTR("X-axis start speed [mm/s]"));
  //epr_out_float(EPR_Y_MAX_START_SPEED,PSTR("Y-axis start speed [mm/s]"));
  //epr_out_float(EPR_Z_MAX_START_SPEED,PSTR("Z-axis start speed [mm/s]"));
  writeFloat(EPR_Z_MAX_ACCEL, PSTR("Acceleration [mm/s^2]"));
  writeFloat(EPR_Z_MAX_TRAVEL_ACCEL, PSTR("Travel acceleration [mm/s^2]"));

  writeFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH, PSTR("Diagonal rod length [mm]"));
  writeFloat(EPR_DELTA_HORIZONTAL_RADIUS, PSTR("Horizontal rod radius at 0,0 [mm]"));
  writeFloat(EPR_DELTA_MAX_RADIUS, PSTR("Max printable radius [mm]"));
  writeInt(EPR_DELTA_TOWERX_OFFSET_STEPS, PSTR("Tower X endstop offset [steps]"));
  writeInt(EPR_DELTA_TOWERY_OFFSET_STEPS, PSTR("Tower Y endstop offset [steps]"));
  writeInt(EPR_DELTA_TOWERZ_OFFSET_STEPS, PSTR("Tower Z endstop offset [steps]"));
  writeFloat(EPR_DELTA_ALPHA_A, PSTR("Alpha A(210):"));
  writeFloat(EPR_DELTA_ALPHA_B, PSTR("Alpha B(330):"));
  writeFloat(EPR_DELTA_ALPHA_C, PSTR("Alpha C(90):"));
  writeFloat(EPR_DELTA_RADIUS_CORR_A, PSTR("Delta Radius A(0):"));
  writeFloat(EPR_DELTA_RADIUS_CORR_B, PSTR("Delta Radius B(0):"));
  writeFloat(EPR_DELTA_RADIUS_CORR_C, PSTR("Delta Radius C(0):"));
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_A, PSTR("Corr. diagonal A [mm]"));
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_B, PSTR("Corr. diagonal B [mm]"));
  writeFloat(EPR_DELTA_DIAGONAL_CORRECTION_C, PSTR("Corr. diagonal C [mm]"));
#endif

#if FEATURE_AUTOLEVEL
  writeByte(EPR_AUTOLEVEL_ACTIVE, PSTR("Autolevel active (1/0)"));
#endif

  writeByte(EPR_BED_HEAT_MANAGER, PSTR("Bed Heat Manager [0-3]"));
  writeByte(EPR_BED_DRIVE_MAX, PSTR("Bed PID drive max"));
  writeByte(EPR_BED_DRIVE_MIN, PSTR("Bed PID drive min"));
  writeFloat(EPR_BED_PID_PGAIN, PSTR("Bed PID P-gain"));
  writeFloat(EPR_BED_PID_IGAIN, PSTR("Bed PID I-gain"));
  writeFloat(EPR_BED_PID_DGAIN, PSTR("Bed PID D-gain"));
  writeByte(EPR_BED_PID_MAX, PSTR("Bed PID max value [0-255]"));

  writeFloat(EPR_RETRACTION_LENGTH,PSTR("Retraction length [mm]"));
  writeFloat(EPR_RETRACTION_SPEED,PSTR("Retraction speed [mm/s]"));
  writeFloat(EPR_RETRACTION_Z_LIFT,PSTR("Retraction z-lift [mm]"));
  writeFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,PSTR("Extra extrusion on undo retract [mm]"));
  writeFloat(EPR_RETRACTION_UNDO_SPEED,PSTR("Retraction undo speed"));

  // now the extruder
  for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    {
      int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
      writeFloat(o + EPR_EXTRUDER_STEPS_PER_MM, PSTR("steps per mm"));
      writeFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, PSTR("max. feedrate [mm/s]"));
      writeFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, PSTR("start feedrate [mm/s]"));
      writeFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, PSTR("acceleration [mm/s^2]"));
      writeByte(o + EPR_EXTRUDER_HEAT_MANAGER, PSTR("heat manager [0-3]"));
      writeByte(o + EPR_EXTRUDER_DRIVE_MAX, PSTR("PID drive max"));
      writeByte(o + EPR_EXTRUDER_DRIVE_MIN, PSTR("PID drive min"));
      writeFloat(o + EPR_EXTRUDER_PID_PGAIN, PSTR("PID P-gain/dead-time"),4);
      writeFloat(o + EPR_EXTRUDER_PID_IGAIN, PSTR("PID I-gain"),4);
      writeFloat(o + EPR_EXTRUDER_PID_DGAIN, PSTR("PID D-gain"),4);
      writeByte(o + EPR_EXTRUDER_PID_MAX, PSTR("PID max value [0-255]"));
      writeLong(o + EPR_EXTRUDER_X_OFFSET, PSTR("X-offset [steps]"));
      writeLong(o + EPR_EXTRUDER_Y_OFFSET, PSTR("Y-offset [steps]"));
      writeLong(o + EPR_EXTRUDER_Z_OFFSET, PSTR("Z-offset [steps]"));
      writeInt(o + EPR_EXTRUDER_WATCH_PERIOD, PSTR("temp. stabilize time [s]"));
#if RETRACT_DURING_HEATUP
      writeInt(o + EPR_EXTRUDER_WAIT_RETRACT_TEMP, PSTR("temp. for retraction when heating [C]"));
      writeInt(o + EPR_EXTRUDER_WAIT_RETRACT_UNITS, PSTR("distance to retract when heating [mm]"));
#endif
      writeFloat(o + EPR_EXTRUDER_ADVANCE_K, PSTR("advance K [0=off]"));
      writeFloat(o + EPR_EXTRUDER_ADVANCE_L, PSTR("advance L [0=off]"));
    }

  Com::printf(PSTR("EEPROM DUMP END\n"));
}





uint8_t EEPROM::computeChecksum()
{
  unsigned int i;
  uint8_t checksum = 0;
  for(i = 0; i < 2048; i++)
    {
      if(i == EPR_INTEGRITY_BYTE) continue;
      checksum += eprGetByte(i);
    }
  return checksum;
}




void EEPROM::updateChecksum()
{
  uint8_t newcheck = computeChecksum();
  if(newcheck!=eprGetByte(EPR_INTEGRITY_BYTE))
    eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
}




void EEPROM::writeExtruderPrefix(uint16_t pos)
{
  if(pos < EEPROM_EXTRUDER_OFFSET || pos >= 800) return;
  int n = (pos - EEPROM_EXTRUDER_OFFSET) / EEPROM_EXTRUDER_LENGTH + 1;
  Com::printF(PSTR("Extr."), n);
  Com::print(' ');
}





void EEPROM::writeFloat(uint16_t pos,PGM_P text,uint8_t digits)
{
  Com::printF(PSTR("EPR:3 "), static_cast<int>(pos));
  Com::print(' ');
  Com::printFloat(eprGetFloat(pos),digits);
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printF(text);
  Com::printF(PSTR("\n"));
	delay(4); // reduces somehow transmission errors
}

void EEPROM::writeLong(uint16_t pos,PGM_P text)
{
  Com::printF(PSTR("EPR:2 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print(eprGetInt32(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printF(text);
  Com::printF(PSTR("\n"));
	delay(4); // reduces somehow transmission errors
}

void EEPROM::writeInt(uint16_t pos,PGM_P text)
{
  Com::printF(PSTR("EPR:1 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print(eprGetInt16(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printF(text);
  Com::printF(PSTR("\n"));
	delay(4); // reduces somehow transmission errors
}

void EEPROM::writeByte(uint16_t pos,PGM_P text)
{
  Com::printF(PSTR("EPR:0 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print((int)eprGetByte(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printF(text);
  Com::printF(PSTR("\n"));
	delay(4); // reduces somehow transmission errors
}

void EEPROM::setZCorrection(int32_t c,int index)
{
  eprSetInt32(2048 + (index << 2), c);
}


