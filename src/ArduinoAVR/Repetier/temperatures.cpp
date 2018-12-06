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


#include "temperatures.h"

tempControl extruderTemp(0);
tempControl bedTemp(1);
tempControl layerFan(2);


#define TEMPTABLE_NUM 35
const int16_t bits4267[35] PROGMEM = {    0,      22,      25,      28,      33,      38,      44,      51,      60,      70,      83,      98,     117,     140,     168,     201,     242,     290,     346,     410,     481,     557,     635,     712,     782,     844,     895,     935,     965,     986,    1000,    1010,    1015,    1019,    1024    };
const float   temp4267[35] PROGMEM = {  300.00,  300.00,  290.00,  280.00,  270.00,  260.00,  250.00,  240.00,  230.00,  220.00,  210.00,  200.00,  190.00,  180.00,  170.00,  160.00,  150.00,  140.00,  130.00,  120.00,  110.00,  100.00,   90.00,   80.00,   70.00,   60.00,   50.00,   40.00,   30.00,   20.00,   10.00,    0.00,  -10.00,  -20.00,  -20.00 };


tempControl::tempControl(uint8_t idx) {

  //  Setup for the extruder.
  if (idx == 0) {
    _id                    = 0;

    _sensorType            = 8;  //  E3D
    _sensorPin             = TEMP_0_PIN;

    _currentRaw            = 0;
    _current               = 0;

    _target                = 0;
    _targetMin             = 25;
    _targetMax             = 260;   //  E3D can go up to 300.

    _heaterPin             = HEATER_0_PIN;
    _heaterState           = 0;
    _heaterDensity         = 0;
    _heaterDensityMax      = 255;
    _heaterError           = 0;

    _fanPin                = HEATER_1_PIN;  //  XXX NON STANDARD!  Fan needs LOTS of power to run.
    _fanState              = 0;
    _fanDensity            = 0;
    _fanError              = 0;
    _fanKick               = 0;


    _pidPGain              = 23.5;   //  A little bit (3-4 deg) overshoot, but stabilizes quickly.
    _pidIGain              =  3.3;
    _pidDGain              = 45.0;

    _pidTimer              = 0;
    _pidTempPrev           = 0;
    _pidTemp               = 0;

    _tempIState            = 0;

    _flags                 = 0;
    _lastDecoupleTime      = 0;
    _lastDecoupleTemp      = 0;
    _decoupleTestPeriod    = 8000;  //  Every 8 seconds?
  }

  //  Setup for the bed.
  if (idx == 1) {
    _id                    = 1;

    _sensorType            = 98;
    _sensorPin             = TEMP_2_PIN;

    _currentRaw            = 0;
    _current               = 0;

    _target                = 0;
    _targetMin             = 25;
    _targetMax             = 120;

    _heaterPin             = BED_HEAT_PIN;
    _heaterState           = 0;
    _heaterDensity         = 0;
    _heaterDensityMax      = 255;
    _heaterError           = 0;

    _fanPin                = -1;
    _fanState              = 0;
    _fanDensity            = 0;
    _fanError              = 0;
    _fanKick               = 0;


    _pidPGain              = 50.0;   //  1 degree overshoot, then 0.8 under, Really doesn't matter much.  This overshoots a
    _pidIGain              =  4.3;   //  small bit, and takes a while to stabilize.
    _pidDGain              = 80.0;

    _pidTimer              = 0;
    _pidTempPrev           = 0;
    _pidTemp               = 0;

    _tempIState            = 0;

    _flags                 = 0;
    _lastDecoupleTime      = 0;
    _lastDecoupleTemp      = 0;
    _decoupleTestPeriod    = 300000;  //  Every five minutes?
  }


  //  Setup for the layer fan.
  if (idx == 2) {
    _id                    = 2;

    _sensorType            = 0;
    _sensorPin             = -1;

    _currentRaw            = 0;
    _current               = 0;

    _target                = 0;
    _targetMin             = 0;
    _targetMax             = 0;

    _heaterPin             = -1;
    _heaterState           = 0;
    _heaterDensity         = 0;
    _heaterDensityMax      = 255;
    _heaterError           = 0;

    _fanPin                = FAN_0_PIN;   //  XXX NON STANDARD
    _fanState              = 0;
    _fanDensity            = 0;
    _fanError              = 0;
    _fanKick               = 0;


    _pidPGain              = 0;
    _pidIGain              = 0;
    _pidDGain              = 0;

    _pidTimer              = 0;
    _pidTempPrev           = 0;
    _pidTemp               = 0;

    _tempIState            = 0;

    _flags                 = 0;
    _lastDecoupleTime      = 0;
    _lastDecoupleTemp      = 0;
    _decoupleTestPeriod    = 0;
  }
};




#define CELSIUS_EXTRA_BITS 3
#define TEMP_INT_TO_FLOAT(temp) ((float)(temp) / (float) (1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)  ((temp) * (1<<CELSIUS_EXTRA_BITS)))


//  I've only got two sensors,
//    EXT0_TEMPSENSOR_TYPE    -  8 == analog
//    HEATED_BED_SENSOR_TYPE  - 98 == analog

void
tempControl::updateCurrentTemperature(void) {

  //  Get raw temperature.  Assume all sensors use the analog inputs.

  _currentRaw = analogRead(_sensorPin);

  //  Do something.

  int16_t newraw  = 0, oldraw  = pgm_read_word (&bits4267[0]);
  float   newtemp = 0, oldtemp = pgm_read_float(&temp4267[0]);
  float   result  = 300;

  for (uint8_t i=1; i<TEMPTABLE_NUM; i++) {
    newraw  = pgm_read_word (&bits4267[i]);
    newtemp = pgm_read_float(&temp4267[i]);

    //Com::printf(PSTR("raw %5d-%5d temp %7.2f-%7.2f -- currentRaw %5d\n"), oldraw, newraw, oldtemp, newtemp, _currentRaw);

    if ((oldraw <= _currentRaw) && (_currentRaw < newraw)) {
      float  deltaT = (newtemp - oldtemp) / (newraw - oldraw);

      result = oldtemp + (_currentRaw - oldraw) * deltaT;

      //Com::printf(PSTR("TEMP %.4fC - target %.4fC\n"), result, _target);
    }

    oldtemp = newtemp;
    oldraw  = newraw;
  }

  _current = result;
}

