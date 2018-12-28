
#include "Repetier.h"
#include "HAL.h"
#include "Commands.h"
#include "motion.h"
#include "Printer.h"
#include "Extruder.h"
#include "temperatures.h"


//
//  Update temperatures and heater power every 100ms.
//



  //  Check for broken sensors.
  //   - below  15C (59F)
  //   - above 270C
  //
  //  Open circuit on E3D sensor reads 11.8.
uint8_t
tempControl::checkInvalidTemperatures(void) {
  uint8_t newDefectFound = false;

  if ((_current < 15) ||
      (_current > 270)) {
    Com::printf(PSTR("Abnormal temperature %.2f detected (raw %d).  FAIL!"), _current, _currentRaw);

    _flags |= TEMPCONTROL_FLAG_SENSOR_DEFECT;

    newDefectFound = true;
  }

  return(newDefectFound);
}




  //  Check if the sensor is reading expected temperature changes.
  //
  //  Failure modes are:
  //    Open circuit (physical failure)
  //    Drift in actual value
  //    Short circuit (needs a power spike to fuse it together)
  //
  //  An open circuit results in a temperature reading of around 15 degrees C.
  //
  //  When initially heating up, the temperature should rise.  (at least 1C every decoupleTestPeriod ms).
  //  When holding stable, the temperature should be stable.   (at most 20C every decoupleTestPeriod ms).
  //
#define MIN_RISE      1
#define MAX_VARIANCE 20


uint8_t
tempControl::checkDecoupledTemperatures(void) {
  uint8_t newDefectFound = false;

  //  If we're below the target temperature, assume we're heating up.
  //  Fail if the current temperature drops (significantly) below the last
  //  recorded temperature.

  if (_current < _target - MAX_VARIANCE) {
    if (_current - _lastDecoupleTemp < MIN_RISE) {
      Com::printf(PSTR("Abnormal variance current %.2f vs target %.2f detected.  FAIL!"), _current, _target);

      _flags |= TEMPCONTROL_FLAG_SENSOR_DECOUPLED;

      newDefectFound = true;
    }
  }

  //  Otherwise, we're supposed to be steady state, so make sure we're not
  //  wildly different than the last time.

  else if (fabs(_current - _lastDecoupleTemp) > MAX_VARIANCE) {
    Com::printf(PSTR("Abnormal variance current %.2f vs last %.2f detected.  FAIL!"), _current, _lastDecoupleTemp);

    _flags |= TEMPCONTROL_FLAG_SENSOR_DECOUPLED;

    newDefectFound = true;
  }

  return(newDefectFound);
}




//  This is called once every 100ms.
//
void
tempControl::manageTemperature(void) {

  //  Refresh temperature reading.  This is the only place we call this function!

  updateCurrentTemperature();
  
  //  Turn off any cooling fan if we're cold and not heating up.

  if ((_current < 45) &&    //  THIS WORKS.
      (_target  < 45))
    _fanDensity = 0;

  //  Turn on any cooling fan if we're hot.

  if ((_current > 55))      //  THIS DIDN'T.
    _fanDensity = 255;

  //  If the target is zero, turn off the heaters and return.

  if (_target < _targetMin) {
    _heaterDensity = 0;
    return;
  }

  //  Likewise, if we're too hot, turn off heaters and return.

  if (_current > _targetMax) {
    _heaterDensity = 0;
    return;
  }

  //  Check for faulty sensors.


#if 0
  uint32_t time = millis(); // compare time for decouple tests

  if (time - _lastDecoupleTime > _decoupleTestPeriod) {
    Com::printf(PSTR("Decouple test for id %d with lastTemp %.2f\n"), _id, _lastDecoupleTemp);

    if (checkInvalidTemperatures() ||
        checkDecoupledTemperatures()) {
      Com::printF(PSTR("Disabling all heaters due to detected sensor defect.\n"));

      extruderTemp.disable();
      bedTemp.disable();

      commandQueue.stopPrint();
    }

    _lastDecoupleTime = time;
    _lastDecoupleTemp = _current;
  }
#endif


  //  Figure out how much power to give the heater.

  float    density = 0;
  float    tDelta   = _target - _current;

  //  Use fancy math when the current temperature is within PID_CONTROL_RANGE
  //  of the target temperature.  Otherwise, full on or full off.
#define PID_CONTROL_RANGE  20
#define PID_INTEGRAL_RANGE 10

  //  Woah!  Way too cold.
  if (tDelta > PID_CONTROL_RANGE) {
    density = 255;
    _tempIState = 0;
  }

  //  Woah!  Way too hot.
  else if (-tDelta > PID_CONTROL_RANGE) {
    density = 0;
    _tempIState = 0;
  }

  //  Just right.  Be fancy.
  //
  //  PID values assume a usable range from 0-255. This can be further
  //  limited to _pidMax by two methods.
  //
  //   - threshold to _pidMax.
  //   - scale by _pidMax/256, then threshold to _pidMax.  Good for small _pidMax.
  //
  //  Assumed to be called every 0.1 seconds.
  //
  else {
    if ((-PID_INTEGRAL_RANGE <= tDelta) &&      //  Only accumulate I when we're
        (tDelta < PID_INTEGRAL_RANGE))          //  very close to stable.
      _tempIState += tDelta;

    if (_tempIState < 50)
      _tempIState = 50;

    if (_tempIState > 500)
      _tempIState = 500;

    density = ((_pidPGain * tDelta) +
               (_pidIGain * _tempIState * 0.1) +            //  0.1 -> 10 Hz
               (_pidDGain * (_pidTempPrev - _pidTemp)));    //  dgain
  }

  Com::printf(PSTR("PID target %.2f current %.2f delta %.2f density %.2f tempIState %.4f\n"),
              _target, _current, tDelta, density, _tempIState);

  //  Threshold to sane values.

  if (density < 0)     density = 0;
  if (density > 255)   density = 255;

  //  Done!  Update the density.

  _heaterDensity = (uint8_t)density;

  //
  //  Update PID temperatures every second.
  //

  if (++_pidTimer == 10) {
    _pidTimer    = 0;
    _pidTempPrev = _pidTemp;
    _pidTemp     = _current;
  }

  //Com::printf(PSTR("SET %d density to %d based on temp %f (raw %d)\n"), _id, density, _current, _currentRaw);

  //  What is this lighting?
  //WRITE(LED_PIN, on);
}
