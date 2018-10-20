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


void EEPROM::update(GCode *com)
{
  if(com->hasT() && com->hasP()) switch(com->T)
                                   {
                                     case 0:
                                       if(com->hasS())
                                         HAL::eprSetByte(com->P, (uint8_t)com->S);
                                       break;
                                     case 1:
                                       if(com->hasS())
                                         HAL::eprSetInt16(com->P, (int16_t)com->S);
                                       break;
                                     case 2:
                                       if(com->hasS())
                                         HAL::eprSetInt32(com->P, (int32_t)com->S);
                                       break;
                                     case 3:
                                       if(com->hasX())
                                         HAL::eprSetFloat(com->P, com->X);
                                       break;
                                   }
  uint8_t newcheck = computeChecksum();
  if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
    HAL::eprSetByte(EPR_INTEGRITY_BYTE, newcheck);
  bool includesEeprom = com->P >= EEPROM_EXTRUDER_OFFSET && com->P < EEPROM_EXTRUDER_OFFSET + 6 * EEPROM_EXTRUDER_LENGTH;
  readDataFromEEPROM(includesEeprom);
  Extruder::selectExtruderById(Extruder::current->id);
}

void EEPROM::storeDataIntoEEPROM(uint8_t corrupted)
{
  HAL::eprSetInt32(EPR_BAUDRATE,baudrate);
  HAL::eprSetInt32(EPR_MAX_INACTIVE_TIME,maxInactiveTime);
  HAL::eprSetInt32(EPR_STEPPER_INACTIVE_TIME,stepperInactiveTime);
  //#define EPR_ACCELERATION_TYPE 1
  HAL::eprSetFloat(EPR_XAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[X_AXIS]);
  HAL::eprSetFloat(EPR_YAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Y_AXIS]);
  HAL::eprSetFloat(EPR_ZAXIS_STEPS_PER_MM,Printer::axisStepsPerMM[Z_AXIS]);

  HAL::eprSetFloat(EPR_MAX_JERK,Printer::maxJerk);
#if RAMP_ACCELERATION
  HAL::eprSetFloat(EPR_X_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_MAX_ACCEL,Printer::maxAccelerationMMPerSquareSecond[Z_AXIS]);
  HAL::eprSetFloat(EPR_X_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS]);
  HAL::eprSetFloat(EPR_Y_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS]);
  HAL::eprSetFloat(EPR_Z_MAX_TRAVEL_ACCEL,Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS]);
#endif

  HAL::eprSetByte(EPR_BED_HEAT_MANAGER,heatedBedController.heatManager);
  HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP,heatedBedController.preheatTemperature);

  HAL::eprSetByte(EPR_BED_DRIVE_MAX,heatedBedController.pidDriveMax);
  HAL::eprSetByte(EPR_BED_DRIVE_MIN,heatedBedController.pidDriveMin);
  HAL::eprSetFloat(EPR_BED_PID_PGAIN,heatedBedController.pidPGain);
  HAL::eprSetFloat(EPR_BED_PID_IGAIN,heatedBedController.pidIGain);
  HAL::eprSetFloat(EPR_BED_PID_DGAIN,heatedBedController.pidDGain);
  HAL::eprSetByte(EPR_BED_PID_MAX,heatedBedController.pidMax);

  HAL::eprSetFloat(EPR_X_HOME_OFFSET,Printer::xMin);
  HAL::eprSetFloat(EPR_Y_HOME_OFFSET,Printer::yMin);
  HAL::eprSetFloat(EPR_Z_HOME_OFFSET,Printer::zMin);
  HAL::eprSetFloat(EPR_X_LENGTH,Printer::xLength);
  HAL::eprSetFloat(EPR_Y_LENGTH,Printer::yLength);
  HAL::eprSetFloat(EPR_Z_LENGTH,Printer::zLength);
  HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, Printer::radius0);

#if FEATURE_AUTOLEVEL
  HAL::eprSetByte(EPR_AUTOLEVEL_ACTIVE,Printer::isAutolevelActive());
  for(uint8_t i = 0; i < 9; i++)
    HAL::eprSetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2),Printer::autolevelTransformation[i]);
#endif
  // now the extruder
  for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    {
#if FEATURE_WATCHDOG
      HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

      int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
      Extruder *e = &extruder[i];
      HAL::eprSetFloat(o+EPR_EXTRUDER_STEPS_PER_MM,e->stepsPerMM);
      HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE,e->maxFeedrate);
      HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE,e->maxStartFeedrate);
      HAL::eprSetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION,e->maxAcceleration);
      HAL::eprSetByte(o+EPR_EXTRUDER_HEAT_MANAGER,e->tempControl.heatManager);
      HAL::eprSetInt16(o+EPR_EXTRUDER_PREHEAT,e->tempControl.preheatTemperature);
      HAL::eprSetByte(o+EPR_EXTRUDER_DRIVE_MAX,e->tempControl.pidDriveMax);
      HAL::eprSetByte(o+EPR_EXTRUDER_DRIVE_MIN,e->tempControl.pidDriveMin);
      HAL::eprSetFloat(o+EPR_EXTRUDER_PID_PGAIN,e->tempControl.pidPGain);
      HAL::eprSetFloat(o+EPR_EXTRUDER_PID_IGAIN,e->tempControl.pidIGain);
      HAL::eprSetFloat(o+EPR_EXTRUDER_PID_DGAIN,e->tempControl.pidDGain);
      HAL::eprSetByte(o+EPR_EXTRUDER_PID_MAX,e->tempControl.pidMax);
      HAL::eprSetInt32(o+EPR_EXTRUDER_X_OFFSET,e->xOffset);
      HAL::eprSetInt32(o+EPR_EXTRUDER_Y_OFFSET,e->yOffset);
      HAL::eprSetInt32(o+EPR_EXTRUDER_Z_OFFSET,e->zOffset);
      HAL::eprSetInt16(o+EPR_EXTRUDER_WATCH_PERIOD,e->watchPeriod);
#if RETRACT_DURING_HEATUP
      HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,e->waitRetractTemperature);
      HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,e->waitRetractUnits);
#else
      HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP,EXT0_WAIT_RETRACT_TEMP);
      HAL::eprSetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS,EXT0_WAIT_RETRACT_UNITS);
#endif
      HAL::eprSetByte(o+EPR_EXTRUDER_COOLER_SPEED,e->coolerSpeed);
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
      HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,e->advanceK);
#else
      HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,0);
#endif
      HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_L,e->advanceL);
#else
      HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_K,0);
      HAL::eprSetFloat(o+EPR_EXTRUDER_ADVANCE_L,0);
#endif
    }
  if(corrupted)
    {
      HAL::eprSetInt32(EPR_PRINTING_TIME,0);
      HAL::eprSetFloat(EPR_PRINTING_DISTANCE,0);
      initalizeUncached();
    }
  // Save version and build checksum
  HAL::eprSetByte(EPR_VERSION,EEPROM_PROTOCOL_VERSION);
  HAL::eprSetByte(EPR_INTEGRITY_BYTE,computeChecksum());
}
void EEPROM::initalizeUncached()
{
  HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
  HAL::eprSetFloat(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
  HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
  HAL::eprSetFloat(EPR_Z_PROBE_X_OFFSET,Z_PROBE_X_OFFSET);
  HAL::eprSetFloat(EPR_Z_PROBE_Y_OFFSET,Z_PROBE_Y_OFFSET);
  HAL::eprSetFloat(EPR_Z_PROBE_X1,Z_PROBE_X1);
  HAL::eprSetFloat(EPR_Z_PROBE_Y1,Z_PROBE_Y1);
  HAL::eprSetFloat(EPR_Z_PROBE_X2,Z_PROBE_X2);
  HAL::eprSetFloat(EPR_Z_PROBE_Y2,Z_PROBE_Y2);
  HAL::eprSetFloat(EPR_Z_PROBE_X3,Z_PROBE_X3);
  HAL::eprSetFloat(EPR_Z_PROBE_Y3,Z_PROBE_Y3);
  HAL::eprSetFloat(EPR_Z_PROBE_BED_DISTANCE,Z_PROBE_BED_DISTANCE);
  HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,DELTA_SEGMENTS_PER_SECOND_PRINT);
  HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,DELTA_SEGMENTS_PER_SECOND_MOVE);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH,DELTA_DIAGONAL_ROD);
  HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS,ROD_RADIUS);
  HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,DELTA_X_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,DELTA_Y_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,DELTA_Z_ENDSTOP_OFFSET_STEPS);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_A,DELTA_ALPHA_A);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_B,DELTA_ALPHA_B);
  HAL::eprSetFloat(EPR_DELTA_ALPHA_C,DELTA_ALPHA_C);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A,DELTA_RADIUS_CORRECTION_A);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B,DELTA_RADIUS_CORRECTION_B);
  HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C,DELTA_RADIUS_CORRECTION_C);
  HAL::eprSetFloat(EPR_DELTA_MAX_RADIUS,DELTA_MAX_RADIUS);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,DELTA_DIAGONAL_CORRECTION_A);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,DELTA_DIAGONAL_CORRECTION_B);
  HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,DELTA_DIAGONAL_CORRECTION_C);
  HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED,0);

  HAL::eprSetFloat(EPR_RETRACTION_LENGTH,RETRACTION_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_LONG_LENGTH,RETRACTION_LONG_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_SPEED,RETRACTION_SPEED);
  HAL::eprSetFloat(EPR_RETRACTION_Z_LIFT,RETRACTION_Z_LIFT);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,RETRACTION_UNDO_EXTRA_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,RETRACTION_UNDO_EXTRA_LONG_LENGTH);
  HAL::eprSetFloat(EPR_RETRACTION_UNDO_SPEED,RETRACTION_UNDO_SPEED);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_A,BENDING_CORRECTION_A);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_B,BENDING_CORRECTION_B);
  HAL::eprSetFloat(EPR_BENDING_CORRECTION_C,BENDING_CORRECTION_C);
  HAL::eprSetFloat(EPR_ACCELERATION_FACTOR_TOP,ACCELERATION_FACTOR_TOP);
  HAL::eprSetFloat(EPR_PARK_X,PARK_POSITION_X);
  HAL::eprSetFloat(EPR_PARK_Y,PARK_POSITION_Y);
  HAL::eprSetFloat(EPR_PARK_Z,PARK_POSITION_Z_RAISE);

}

void EEPROM::readDataFromEEPROM(bool includeExtruder)
{
  uint8_t version = HAL::eprGetByte(EPR_VERSION); // This is the saved version. Don't copy data nor set it to older versions!
  Com::printFLN(PSTR("Detected EEPROM version:"),(int)version);
  baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
  maxInactiveTime = HAL::eprGetInt32(EPR_MAX_INACTIVE_TIME);
  stepperInactiveTime = HAL::eprGetInt32(EPR_STEPPER_INACTIVE_TIME);
  //#define EPR_ACCELERATION_TYPE 1
  Printer::axisStepsPerMM[X_AXIS] = HAL::eprGetFloat(EPR_XAXIS_STEPS_PER_MM);
  Printer::axisStepsPerMM[Y_AXIS] = HAL::eprGetFloat(EPR_YAXIS_STEPS_PER_MM);
  Printer::axisStepsPerMM[Z_AXIS] = HAL::eprGetFloat(EPR_ZAXIS_STEPS_PER_MM);

  Printer::maxJerk = HAL::eprGetFloat(EPR_MAX_JERK);
#if RAMP_ACCELERATION
  Printer::maxAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_ACCEL);
  Printer::maxAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[X_AXIS] = HAL::eprGetFloat(EPR_X_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Y_AXIS] = HAL::eprGetFloat(EPR_Y_MAX_TRAVEL_ACCEL);
  Printer::maxTravelAccelerationMMPerSquareSecond[Z_AXIS] = HAL::eprGetFloat(EPR_Z_MAX_TRAVEL_ACCEL);
#endif

  heatedBedController.heatManager = HAL::eprGetByte(EPR_BED_HEAT_MANAGER);
  heatedBedController.preheatTemperature = HAL::eprGetInt16(EPR_BED_PREHEAT_TEMP);
  heatedBedController.pidDriveMax = HAL::eprGetByte(EPR_BED_DRIVE_MAX);
  heatedBedController.pidDriveMin = HAL::eprGetByte(EPR_BED_DRIVE_MIN);
  heatedBedController.pidPGain = HAL::eprGetFloat(EPR_BED_PID_PGAIN);
  heatedBedController.pidIGain = HAL::eprGetFloat(EPR_BED_PID_IGAIN);
  heatedBedController.pidDGain = HAL::eprGetFloat(EPR_BED_PID_DGAIN);
  heatedBedController.pidMax = HAL::eprGetByte(EPR_BED_PID_MAX);

  Printer::xMin = HAL::eprGetFloat(EPR_X_HOME_OFFSET);
  Printer::yMin = HAL::eprGetFloat(EPR_Y_HOME_OFFSET);
  Printer::zMin = HAL::eprGetFloat(EPR_Z_HOME_OFFSET);
  Printer::xLength = HAL::eprGetFloat(EPR_X_LENGTH);
  Printer::yLength = HAL::eprGetFloat(EPR_Y_LENGTH);
  Printer::zLength = HAL::eprGetFloat(EPR_Z_LENGTH);
  Printer::radius0 = HAL::eprGetFloat(EPR_DELTA_HORIZONTAL_RADIUS);

#if FEATURE_AUTOLEVEL
  if(version > 2)
    {
      float sum = 0;
      for(uint8_t i = 0; i < 9; i++)
        Printer::autolevelTransformation[i] = HAL::eprGetFloat(EPR_AUTOLEVEL_MATRIX + (((int)i) << 2));
      if(isnan(Printer::autolevelTransformation[0]))   // a bug caused storage of matrix at the wrong place. Read from old position instead.
        {
          for(uint8_t i = 0; i < 9; i++)
            Printer::autolevelTransformation[i] = HAL::eprGetFloat((EPR_AUTOLEVEL_MATRIX + (int)i) << 2);
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
      Printer::setAutolevelActive(HAL::eprGetByte(EPR_AUTOLEVEL_ACTIVE));
      Com::printArrayFLN(PSTR("Transformation matrix:"),Printer::autolevelTransformation, 9, 6);
    }
#endif
  if(includeExtruder)
    {
      // now the extruder
      for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
        {
#if FEATURE_WATCHDOG
          HAL::pingWatchdog();
#endif // FEATURE_WATCHDOG

          int o=i*EEPROM_EXTRUDER_LENGTH+EEPROM_EXTRUDER_OFFSET;
          Extruder *e = &extruder[i];
          e->stepsPerMM = HAL::eprGetFloat(o+EPR_EXTRUDER_STEPS_PER_MM);
          e->maxFeedrate = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_FEEDRATE);
          e->maxStartFeedrate = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_START_FEEDRATE);
          e->maxAcceleration = HAL::eprGetFloat(o+EPR_EXTRUDER_MAX_ACCELERATION);
          e->tempControl.heatManager = HAL::eprGetByte(o+EPR_EXTRUDER_HEAT_MANAGER);
          e->tempControl.preheatTemperature = HAL::eprGetInt16(o+EPR_EXTRUDER_PREHEAT);
          e->tempControl.pidDriveMax = HAL::eprGetByte(o+EPR_EXTRUDER_DRIVE_MAX);
          e->tempControl.pidDriveMin = HAL::eprGetByte(o+EPR_EXTRUDER_DRIVE_MIN);
          e->tempControl.pidPGain = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_PGAIN);
          e->tempControl.pidIGain = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_IGAIN);
          e->tempControl.pidDGain = HAL::eprGetFloat(o+EPR_EXTRUDER_PID_DGAIN);
          e->tempControl.pidMax = HAL::eprGetByte(o+EPR_EXTRUDER_PID_MAX);
          e->xOffset = HAL::eprGetInt32(o+EPR_EXTRUDER_X_OFFSET);
          e->yOffset = HAL::eprGetInt32(o+EPR_EXTRUDER_Y_OFFSET);
          e->watchPeriod = HAL::eprGetInt16(o+EPR_EXTRUDER_WATCH_PERIOD);
#if RETRACT_DURING_HEATUP
          e->waitRetractTemperature = HAL::eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_TEMP);
          e->waitRetractUnits = HAL::eprGetInt16(o+EPR_EXTRUDER_WAIT_RETRACT_UNITS);
#endif
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
          e->advanceK = HAL::eprGetFloat(o+EPR_EXTRUDER_ADVANCE_K);
#endif
          e->advanceL = HAL::eprGetFloat(o+EPR_EXTRUDER_ADVANCE_L);
#endif
          if(version > 1)
            e->coolerSpeed = HAL::eprGetByte(o+EPR_EXTRUDER_COOLER_SPEED);
          if(version < 13) {
            HAL::eprSetInt32(o+EPR_EXTRUDER_Z_OFFSET,e->zOffset);
          }
          e->zOffset = HAL::eprGetInt32(o + EPR_EXTRUDER_Z_OFFSET);
        }
    }
  if(version != EEPROM_PROTOCOL_VERSION)
    {
      Com::printInfoFLN(PSTR("Protocol version changed, upgrading"));
      if(version < 3)
        {
          HAL::eprSetFloat(EPR_Z_PROBE_HEIGHT,Z_PROBE_HEIGHT);
          HAL::eprSetFloat(EPR_Z_PROBE_SPEED,Z_PROBE_SPEED);
          HAL::eprSetFloat(EPR_Z_PROBE_XY_SPEED,Z_PROBE_XY_SPEED);
          HAL::eprSetFloat(EPR_Z_PROBE_X_OFFSET,Z_PROBE_X_OFFSET);
          HAL::eprSetFloat(EPR_Z_PROBE_Y_OFFSET,Z_PROBE_Y_OFFSET);
          HAL::eprSetFloat(EPR_Z_PROBE_X1,Z_PROBE_X1);
          HAL::eprSetFloat(EPR_Z_PROBE_Y1,Z_PROBE_Y1);
          HAL::eprSetFloat(EPR_Z_PROBE_X2,Z_PROBE_X2);
          HAL::eprSetFloat(EPR_Z_PROBE_Y2,Z_PROBE_Y2);
          HAL::eprSetFloat(EPR_Z_PROBE_X3,Z_PROBE_X3);
          HAL::eprSetFloat(EPR_Z_PROBE_Y3,Z_PROBE_Y3);
        }
      if(version < 4)
        {
          HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_PRINT,DELTA_SEGMENTS_PER_SECOND_PRINT);
          HAL::eprSetInt16(EPR_DELTA_SEGMENTS_PER_SECOND_MOVE,DELTA_SEGMENTS_PER_SECOND_MOVE);
          HAL::eprSetFloat(EPR_DELTA_DIAGONAL_ROD_LENGTH,DELTA_DIAGONAL_ROD);
          HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS,ROD_RADIUS);
          HAL::eprSetInt16(EPR_DELTA_TOWERX_OFFSET_STEPS,DELTA_X_ENDSTOP_OFFSET_STEPS);
          HAL::eprSetInt16(EPR_DELTA_TOWERY_OFFSET_STEPS,DELTA_Y_ENDSTOP_OFFSET_STEPS);
          HAL::eprSetInt16(EPR_DELTA_TOWERZ_OFFSET_STEPS,DELTA_Z_ENDSTOP_OFFSET_STEPS);
        }
      if(version < 5)
        {
          HAL::eprSetFloat(EPR_DELTA_ALPHA_A,DELTA_ALPHA_A);
          HAL::eprSetFloat(EPR_DELTA_ALPHA_B,DELTA_ALPHA_B);
          HAL::eprSetFloat(EPR_DELTA_ALPHA_C,DELTA_ALPHA_C);
        }
      if(version < 6)
        {
          HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A,DELTA_RADIUS_CORRECTION_A);
          HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B,DELTA_RADIUS_CORRECTION_B);
          HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C,DELTA_RADIUS_CORRECTION_C);
        }
      if(version < 7)
        {
          HAL::eprSetFloat(EPR_DELTA_MAX_RADIUS,DELTA_MAX_RADIUS);
          HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A,DELTA_DIAGONAL_CORRECTION_A);
          HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B,DELTA_DIAGONAL_CORRECTION_B);
          HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C,DELTA_DIAGONAL_CORRECTION_C);
        }
      if(version < 8)
        {
          HAL::eprSetFloat(EPR_Z_PROBE_BED_DISTANCE,Z_PROBE_BED_DISTANCE);
        }
      if(version < 11)
        {
          HAL::eprSetByte(EPR_DISTORTION_CORRECTION_ENABLED, 0);
        }
      if(version < 12)
        {
          HAL::eprSetFloat(EPR_RETRACTION_LENGTH,RETRACTION_LENGTH);
          HAL::eprSetFloat(EPR_RETRACTION_LONG_LENGTH,RETRACTION_LONG_LENGTH);
          HAL::eprSetFloat(EPR_RETRACTION_SPEED,RETRACTION_SPEED);
          HAL::eprSetFloat(EPR_RETRACTION_Z_LIFT,RETRACTION_Z_LIFT);
          HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,RETRACTION_UNDO_EXTRA_LENGTH);
          HAL::eprSetFloat(EPR_RETRACTION_UNDO_EXTRA_LONG_LENGTH,RETRACTION_UNDO_EXTRA_LONG_LENGTH);
          HAL::eprSetFloat(EPR_RETRACTION_UNDO_SPEED,RETRACTION_UNDO_SPEED);
        }
      if(version < 16) {
        HAL::eprSetFloat(EPR_BENDING_CORRECTION_A,BENDING_CORRECTION_A);
        HAL::eprSetFloat(EPR_BENDING_CORRECTION_B,BENDING_CORRECTION_B);
        HAL::eprSetFloat(EPR_BENDING_CORRECTION_C,BENDING_CORRECTION_C);
        HAL::eprSetFloat(EPR_ACCELERATION_FACTOR_TOP,ACCELERATION_FACTOR_TOP);
      }
      if(version < 17) {
        heatedBedController.preheatTemperature = HEATED_BED_PREHEAT_TEMP;
        extruder[0].tempControl.preheatTemperature = EXT0_PREHEAT_TEMP;
      }
      if(version < 19) {
		    HAL::eprSetFloat(EPR_PARK_X,PARK_POSITION_X);
		    HAL::eprSetFloat(EPR_PARK_Y,PARK_POSITION_Y);
		    HAL::eprSetFloat(EPR_PARK_Z,PARK_POSITION_Z_RAISE);
      }

      storeDataIntoEEPROM(false); // Store new fields for changed version
    }
  Printer::updateDerivedParameter();
  Extruder::initHeatedBed();
}

void EEPROM::initBaudrate()
{
  // Invariant - baudrate is initialized with or without eeprom!
  baudrate = BAUDRATE;
  if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE)
    {
      baudrate = HAL::eprGetInt32(EPR_BAUDRATE);
    }
}

#ifndef USE_CONFIGURATION_BAUD_RATE
#define USE_CONFIGURATION_BAUD_RATE 0
#endif // USE_CONFIGURATION_BAUD_RATE

void EEPROM::init()
{
  uint8_t check = computeChecksum();
  uint8_t storedcheck = HAL::eprGetByte(EPR_INTEGRITY_BYTE);
  if(HAL::eprGetByte(EPR_MAGIC_BYTE) == EEPROM_MODE && storedcheck == check)
    {
      readDataFromEEPROM(true);
      if (USE_CONFIGURATION_BAUD_RATE)
        {
          // Used if eeprom gets unusable baud rate set and communication wont work at all.
          if(HAL::eprGetInt32(EPR_BAUDRATE) != BAUDRATE)
            {
              HAL::eprSetInt32(EPR_BAUDRATE,BAUDRATE);
              baudrate = BAUDRATE;
              uint8_t newcheck = computeChecksum();
              if(newcheck != HAL::eprGetByte(EPR_INTEGRITY_BYTE))
                HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
            }
          Com::printFLN(PSTR("EEPROM baud rate restored from configuration."));
          Com::printFLN(PSTR("RECOMPILE WITH USE_CONFIGURATION_BAUD_RATE == 0 to alter baud rate via EEPROM"));
        }
    }
  else
    {
      HAL::eprSetByte(EPR_MAGIC_BYTE,EEPROM_MODE); // Make data change permanent
      initalizeUncached();
      storeDataIntoEEPROM(storedcheck != check);
    }
}

void EEPROM::updatePrinterUsage()
{
  if(Printer::filamentPrinted == 0 || (Printer::flag2 & PRINTER_FLAG2_RESET_FILAMENT_USAGE) != 0) return; // No miles only enabled
  uint32_t seconds = (HAL::timeInMilliseconds() - Printer::msecondsPrinting) / 1000;
  seconds += HAL::eprGetInt32(EPR_PRINTING_TIME);
  HAL::eprSetInt32(EPR_PRINTING_TIME,seconds);
  HAL::eprSetFloat(EPR_PRINTING_DISTANCE,HAL::eprGetFloat(EPR_PRINTING_DISTANCE) + Printer::filamentPrinted * 0.001);
  Printer::flag2 |= PRINTER_FLAG2_RESET_FILAMENT_USAGE;
  Printer::msecondsPrinting = HAL::timeInMilliseconds();
  updateChecksum();
  Commands::reportPrinterUsage();
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
  writeLong(EPR_BAUDRATE, PSTR("Baudrate"));
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
#if defined(INTERPOLATE_ACCELERATION_WITH_Z) && INTERPOLATE_ACCELERATION_WITH_Z != 0
  writeFloat(EPR_ACCELERATION_FACTOR_TOP, PSTR("Acceleration factor at top [%,100=like bottom]"));
#endif
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

#if FEATURE_Z_PROBE
  writeFloat(EPR_Z_PROBE_HEIGHT, PSTR("Z-probe height [mm]"));
  writeFloat(EPR_Z_PROBE_BED_DISTANCE, PSTR("Max. z-probe - bed dist. [mm]"));
  writeFloat(EPR_Z_PROBE_SPEED, PSTR("Z-probe speed [mm/s]"));
  writeFloat(EPR_Z_PROBE_XY_SPEED, PSTR("Z-probe x-y-speed [mm/s]"));
  writeFloat(EPR_Z_PROBE_X_OFFSET, PSTR("Z-probe offset x [mm]"));
  writeFloat(EPR_Z_PROBE_Y_OFFSET, PSTR("Z-probe offset y [mm]"));
  writeFloat(EPR_Z_PROBE_X1, PSTR("Z-probe X1 [mm]"));
  writeFloat(EPR_Z_PROBE_Y1, PSTR("Z-probe Y1 [mm]"));
  writeFloat(EPR_Z_PROBE_X2, PSTR("Z-probe X2 [mm]"));
  writeFloat(EPR_Z_PROBE_Y2, PSTR("Z-probe Y2 [mm]"));
  writeFloat(EPR_Z_PROBE_X3, PSTR("Z-probe X3 [mm]"));
  writeFloat(EPR_Z_PROBE_Y3, PSTR("Z-probe Y3 [mm]"));
  writeFloat(EPR_BENDING_CORRECTION_A, PSTR("Z-probe bending correction A [mm]"));
  writeFloat(EPR_BENDING_CORRECTION_B, PSTR("Z-probe bending correction B [mm]"));
  writeFloat(EPR_BENDING_CORRECTION_C, PSTR("Z-probe bending correction C [mm]"));
#endif
#if FEATURE_AUTOLEVEL
  writeByte(EPR_AUTOLEVEL_ACTIVE, PSTR("Autolevel active (1/0)"));
#endif



  writeInt(EPR_BED_PREHEAT_TEMP, PSTR("Bed Preheat temp. [°C]"));
  writeByte(EPR_BED_HEAT_MANAGER, PSTR("Bed Heat Manager [0-3]"));
  writeByte(EPR_BED_DRIVE_MAX, PSTR("Bed PID drive max"));
  writeByte(EPR_BED_DRIVE_MIN, PSTR("Bed PID drive min"));
  writeFloat(EPR_BED_PID_PGAIN, PSTR("Bed PID P-gain"));
  writeFloat(EPR_BED_PID_IGAIN, PSTR("Bed PID I-gain"));
  writeFloat(EPR_BED_PID_DGAIN, PSTR("Bed PID D-gain"));
  writeByte(EPR_BED_PID_MAX, PSTR("Bed PID max value [0-255]"));

#if FEATURE_RETRACTION
  writeFloat(EPR_RETRACTION_LENGTH,PSTR("Retraction length [mm]"));
  writeFloat(EPR_RETRACTION_SPEED,PSTR("Retraction speed [mm/s]"));
  writeFloat(EPR_RETRACTION_Z_LIFT,PSTR("Retraction z-lift [mm]"));
  writeFloat(EPR_RETRACTION_UNDO_EXTRA_LENGTH,PSTR("Extra extrusion on undo retract [mm]"));
  writeFloat(EPR_RETRACTION_UNDO_SPEED,PSTR("Retraction undo speed"));
#endif
  // now the extruder
  for(uint8_t i = 0; i < NUM_EXTRUDER; i++)
    {
      int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;
      writeFloat(o + EPR_EXTRUDER_STEPS_PER_MM, PSTR("steps per mm"));
      writeFloat(o + EPR_EXTRUDER_MAX_FEEDRATE, PSTR("max. feedrate [mm/s]"));
      writeFloat(o + EPR_EXTRUDER_MAX_START_FEEDRATE, PSTR("start feedrate [mm/s]"));
      writeFloat(o + EPR_EXTRUDER_MAX_ACCELERATION, PSTR("acceleration [mm/s^2]"));
      writeInt(o + EPR_EXTRUDER_PREHEAT, PSTR("Preheat temp. [°C]"));
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
      writeByte(o + EPR_EXTRUDER_COOLER_SPEED, PSTR("extruder cooler speed [0-255]"));
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
      writeFloat(o + EPR_EXTRUDER_ADVANCE_K, PSTR("advance K [0=off]"));
#endif
      writeFloat(o + EPR_EXTRUDER_ADVANCE_L, PSTR("advance L [0=off]"));
#endif
    }
}


uint8_t EEPROM::computeChecksum()
{
  unsigned int i;
  uint8_t checksum = 0;
  for(i = 0; i < 2048; i++)
    {
      if(i == EEPROM_OFFSET + EPR_INTEGRITY_BYTE) continue;
      checksum += HAL::eprGetByte(i);
    }
  return checksum;
}

void EEPROM::updateChecksum()
{
  uint8_t newcheck = computeChecksum();
  if(newcheck!=HAL::eprGetByte(EPR_INTEGRITY_BYTE))
    HAL::eprSetByte(EPR_INTEGRITY_BYTE,newcheck);
}

void EEPROM::writeExtruderPrefix(uint pos)
{
  if(pos < EEPROM_EXTRUDER_OFFSET || pos >= 800) return;
  int n = (pos - EEPROM_EXTRUDER_OFFSET) / EEPROM_EXTRUDER_LENGTH + 1;
  Com::printF(PSTR("Extr."), n);
  Com::print(' ');
}

void EEPROM::writeFloat(uint pos,PGM_P text,uint8_t digits)
{
  Com::printF(PSTR("EPR:3 "), static_cast<int>(pos));
  Com::print(' ');
  Com::printFloat(HAL::eprGetFloat(pos),digits);
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeLong(uint pos,PGM_P text)
{
  Com::printF(PSTR("EPR:2 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print(HAL::eprGetInt32(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeInt(uint pos,PGM_P text)
{
  Com::printF(PSTR("EPR:1 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print(HAL::eprGetInt16(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::writeByte(uint pos,PGM_P text)
{
  Com::printF(PSTR("EPR:0 "), static_cast<int>(pos));
  Com::print(' ');
  Com::print((int)HAL::eprGetByte(pos));
  Com::print(' ');
  writeExtruderPrefix(pos);
  Com::printFLN(text);
	HAL::delayMilliseconds(4); // reduces somehow transmission errors
}

void EEPROM::setZCorrection(int32_t c,int index)
{
  HAL::eprSetInt32(2048 + (index << 2), c);
}


