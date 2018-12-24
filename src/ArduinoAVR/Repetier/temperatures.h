

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

#ifndef TEMPERATURES_H
#define TEMPERATURES_H

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#include "Repetier.h"
#include "HAL.h"

#include "Communication.h"

#include "gcode.h"

#include "Commands.h"

#define TEMPCONTROL_FLAG_SENSOR_DEFECT     0x01
#define TEMPCONTROL_FLAG_SENSOR_DECOUPLED  0x02


class tempControl {
public:
  tempControl(uint8_t idx);
  ~tempControl() {};

private:
  uint8_t  _id;

  uint8_t  _sensorType;              // Type of temperature sensor.
  uint8_t  _sensorPin;               // Pin to read extruder temperature.

  int16_t  _currentRaw;              // Current temperature value read from sensor.
  float    _current;                 // Current temperature in degC.

  float    _target;                  // Target temperature in degC.
  float    _targetMin;
  float    _targetMax;

  int8_t   _heaterPin;
  uint8_t  _heaterState;
  uint8_t  _heaterDensity;
  uint8_t  _heaterDensityMax;
  int16_t  _heaterError;

  //  If density changes from 0-255 to 0-100, search for heaterDensity to update print commands
  //  If density changes from 0-255 to 0-100, search for fanDensity to update print commands
public:
  uint8_t   id(void) {
    return(_id);
  };

  void      heaterPDM(void) {

    if (_heaterPin == -1)
      return;

    if (_heaterDensity > _heaterError) {
      if (_heaterState == 0) {
        _heaterState = 1;
        digitalWrite(_heaterPin, 1);
      }
      _heaterError += 255;
      _heaterError -= _heaterDensity;
    } else {
      if (_heaterState == 1) {
        _heaterState = 0;
        digitalWrite(_heaterPin, 0);
      }
      _heaterError -= _heaterDensity;
    }
  };

  uint8_t   getHeaterDensity(void) {
    return(_heaterDensity);
  };


private:
  int8_t   _fanPin;
  uint8_t  _fanState;
  uint8_t  _fanDensity;
  int16_t  _fanError;
  uint8_t  _fanKick;

public:
  void      fanPDM(void) {

    if (_fanPin == -1)
      return;

    if ((_fanKick > 0) && (_fanState == 0)) {
      _fanState = 1;
      digitalWrite(_fanPin, 1);
      return;
    }

    if (_fanDensity > _fanError) {
      if (_fanState == 0) {
        _fanState = 1;
        digitalWrite(_fanPin, 1);
      }
      _fanError += 255;
      _fanError -= _fanDensity;
    } else {
      if (_fanState == 1) {
        _fanState = 0;
        digitalWrite(_fanPin, 0);
      }
      _fanError -= _fanDensity;
    }
  };

  void setFanSpeed(uint8_t speed) {

    if (_fanDensity == speed)
      return;

    if ((_fanDensity < 15) &&    //  Fan will run at 100% speed for this many tenths-of-seconds.
        (_fanDensity < speed))   //  It's just needed to make the fan initially spin.
      _fanKick = 5;

    _fanDensity = speed;
  };

  uint8_t   getFanSpeed(void) {
    return(_fanDensity);
  };

  void      fanKickTick(void) {
    if (_fanKick > 0)
      _fanKick--;
  }


public:
  float    _pidPGain;                // Pgain (proportional gain) for PID temperature control [0,01 Units].
  float    _pidIGain;                // Igain (integral) for PID temperature control [0,01 Units].
  float    _pidDGain;                // Dgain (damping) for PID temperature control [0,01 Units].

private:
	uint8_t  _pidTimer;
  float    _pidTempPrev;
	float    _pidTemp;

  float    _tempIState;              // Temp. var. for PID computation.

  uint32_t _lastDecoupleTime;        // Last time of decoupling sensor-heater test
  float    _lastDecoupleTemp;        // Temperature on last test

  uint32_t _decoupleTestPeriod;      // Time between setting and testing decoupling.

  uint8_t  _flags;

public:
  void initialize(void) {

    Com::printf(PSTR("tempControl::initialize() id %d\n"), _id);

    if (_sensorPin > -1) {
    }

    if (_heaterPin > -1) {
      pinMode(_heaterPin, OUTPUT);
      digitalWrite(_heaterPin, 0);
    }

    if (_fanPin > -1) {
      pinMode(_fanPin, OUTPUT);
      digitalWrite(_fanPin, 0);
    }
  };

  void printTemperature(void) {
    Com::printf(PSTR("Temp: BED %5.1f/%5.1f RAW %6d ERROR %3d PWM %4d\n"),
                _current, _target,
                _currentRaw,
                _flags,
                _heaterDensity);
  };

  void setTargetTemperature(float temp) {

    Com::printf(PSTR("setTemp TO  %.2f, currently set to %.2f\n"), temp, _target);

    if (temp > _targetMax)    temp = _targetMax;
    if (temp < _targetMin)    temp = 0;

    if ((temp > 0 &&                  //  If the new target is positive, and the previous
         (_target == 0))) {           //  was zero, initialize the decouple test.
      _lastDecoupleTime = millis();
      _lastDecoupleTemp = _current;
    }

    if (temp == 0) {                  //  If the new target is zero, disable
      _lastDecoupleTime = 0;          //  the decouple test.
      _lastDecoupleTemp = 0;
    }

    _target = temp;
  };

  void disable(void) {
    _target              = 0;
    _heaterDensity       = 0;
  }

  void waitForTargetTemperature(void) {
    //if (_target < 30)              return;

    //  RETRACT_DURING_HEATUP
    //if (actExtruder->waitRetractUnits > 0 && !retracted && dirRising && actExtruder->tempControl.current > actExtruder->waitRetractTemperature) {
    //  PrintLine::moveRelativeDistanceInSteps(0, 0, 0, -actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
    //  retracted = 1;
    //}

    Com::printf(PSTR("waitForTargetTemperature()  id %d target %d current %d\n"), _id, _target, _current);

    if (_target == 0)
      return;

    while (fabs(_target - _current) > 1.0) {
      commandQueue.keepAlive(GCODE_WAIT_HEATER);

      Commands::checkForPeriodicalActions(true);
    }

    //RETRACT_DURING_HEATUP
    //if (retracted && actExtruder) {
    //  PrintLine::moveRelativeDistanceInSteps(0, 0, 0, actExtruder->waitRetractUnits * Printer::axisStepsPerMM[E_AXIS], actExtruder->maxFeedrate / 4, false, false);
    //}
  };



  float   getCurrentTemperature  (void)   { return(_current);   };
  float   getTargetTemperature   (void)   { return(_target);    };
  float   getTargetTemperatureMin(void)   { return(_targetMin); };
  float   getTargetTemperatureMax(void)   { return(_targetMax); };

  void    updateCurrentTemperature(void);

  uint8_t checkInvalidTemperatures(void);
  uint8_t checkDecoupledTemperatures(void);
  void    manageTemperature(void);

  bool    isSensorDefect(void)    { return(_flags & TEMPCONTROL_FLAG_SENSOR_DEFECT);    };
  bool    isSensorDecoupled(void) { return(_flags & TEMPCONTROL_FLAG_SENSOR_DECOUPLED); };
};


extern tempControl extruderTemp;
extern tempControl bedTemp;
extern tempControl layerFan;

#endif  //  TEMPERATURES_H
