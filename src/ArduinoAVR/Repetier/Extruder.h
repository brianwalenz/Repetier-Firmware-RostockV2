#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#include "HAL.h"

#define CELSIUS_EXTRA_BITS 3
#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions

// Updates the temperature of all extruders and heated bed if it's time.
// Toggles the heater power if necessary.
extern bool reportTempsensorError(); ///< Report defect sensors
extern uint8_t manageMonitor;
#define HTR_OFF 0
#define HTR_PID 1
#define HTR_SLOWBANG 2
#define HTR_DEADTIME 3

#define TEMPERATURE_CONTROLLER_FLAG_ALARM 1
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL 2    ///< Full heating enabled
#define TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD 4    ///< Holding target temperature
#define TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT    8    ///< Indicating sensor defect
#define TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED 16   ///< Indicating sensor decoupling
#define TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN      64   ///< Indicates a slowed down extruder

#ifndef PID_TEMP_CORRECTION
#define PID_TEMP_CORRECTION 2.0
#endif

/** TemperatureController manages one heater-temperature sensor loop. You can have up to
    4 loops allowing pid/bang bang for up to 3 extruder and the heated bed.

*/
class TemperatureController
{
public:
  uint8_t pwmIndex; ///< pwm index for output control. 0-2 = Extruder, 3 = Fan, 4 = Heated Bed
  uint8_t sensorType; ///< Type of temperature sensor.
  uint8_t sensorPin; ///< Pin to read extruder temperature.
  int8_t heatManager; ///< How is temperature controlled. 0 = on/off, 1 = PID-Control, 3 = dead time control
  int16_t currentTemperature; ///< Current temperature value read from sensor.
  //int16_t targetTemperature; ///< Target temperature value in units of sensor.
  float currentTemperatureC; ///< Current temperature in degC.
  float targetTemperatureC; ///< Target temperature in degC.
	float temperatureC; ///< For 1s updates temperature and last build a short time history
	float lastTemperatureC; ///< Used to compute D errors.
  uint32_t lastTemperatureUpdate; ///< Time in millis of the last temperature update.
  float tempIState; ///< Temp. var. for PID computation.
  uint8_t pidDriveMax; ///< Used for windup in PID calculation.
  uint8_t pidDriveMin; ///< Used for windup in PID calculation.
#define deadTime pidPGain
  // deadTime is logically different value but physically overlays pidPGain for saving space
  float pidPGain; ///< Pgain (proportional gain) for PID temperature control [0,01 Units].
  float pidIGain; ///< Igain (integral) for PID temperature control [0,01 Units].
  float pidDGain;  ///< Dgain (damping) for PID temperature control [0,01 Units].
  uint8_t pidMax; ///< Maximum PWM value, the heater should be set.
  float tempIStateLimitMax;
  float tempIStateLimitMin;
  uint8_t flags;
  uint32_t lastDecoupleTest;  ///< Last time of decoupling sensor-heater test
  float  lastDecoupleTemp;  ///< Temperature on last test
  uint32_t decoupleTestPeriod; ///< Time between setting and testing decoupling.

  void setTargetTemperature(float target);
  void updateCurrentTemperature();
  void updateTempControlVars();
  inline bool isAlarm()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_ALARM;
  }
  inline void setAlarm(bool on)
  {
    if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_ALARM;
    else flags &= ~TEMPERATURE_CONTROLLER_FLAG_ALARM;
  }
  inline bool isDecoupleFull()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
  }
	inline void removeErrorStates() {
    flags &= ~(TEMPERATURE_CONTROLLER_FLAG_ALARM | TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT | TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED);
	}
  inline bool isDecoupleFullOrHold()
  {
    return flags & (TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
  }
  inline void setDecoupleFull(bool on)
  {
    flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL;
  }
  inline bool isDecoupleHold()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
  }
  inline void setDecoupleHold(bool on)
  {
    flags &= ~(TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_FULL | TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD);
    if(on) flags |= TEMPERATURE_CONTROLLER_FLAG_DECOUPLE_HOLD;
  }
  inline void startFullDecouple(uint32_t &t)
  {
    if(isDecoupleFull()) return;
    lastDecoupleTest = t;
    lastDecoupleTemp = currentTemperatureC;
    setDecoupleFull(true);
  }
  inline void startHoldDecouple(uint32_t &t)
  {
    if(isDecoupleHold()) return;
    if(fabs(currentTemperatureC - targetTemperatureC) + 1 > DECOUPLING_TEST_MAX_HOLD_VARIANCE) return;
    lastDecoupleTest = t;
    lastDecoupleTemp = targetTemperatureC;
    setDecoupleHold(true);
  }
  inline void stopDecouple()
  {
    setDecoupleFull(false);
  }
  inline bool isSensorDefect()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDEFECT;
  }
  inline bool isSensorDecoupled()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_SENSDECOUPLED;
  }
	static void resetAllErrorStates();
	int8_t errorState();
  inline bool isSlowedDown()
  {
    return flags & TEMPERATURE_CONTROLLER_FLAG_SLOWDOWN;
  }
  void waitForTargetTemperature();
  void autotunePID(float temp,uint8_t controllerId,int maxCycles,bool storeResult, int method);
};
class Extruder;
extern Extruder extruder[];

#define EXTRUDER_FLAG_RETRACTED 1

/** \brief Data to drive one extruder.

    This structure contains all definitions for an extruder and all
    current state variables, like current temperature, feeder position etc.
*/
class Extruder   // Size: 12*1 Byte+12*4 Byte+4*2Byte = 68 Byte
{
public:
  static Extruder *current;
  uint8_t id;
  int32_t xOffset;
  int32_t yOffset;
  int32_t zOffset;
  float stepsPerMM;        ///< Steps per mm.
  int8_t enablePin;          ///< Pin to enable extruder stepper motor.
  uint8_t enableOn;
  float maxFeedrate;      ///< Maximum feedrate in mm/s.
  float maxAcceleration;  ///< Maximum acceleration in mm/s^2.
  float maxStartFeedrate; ///< Maximum start feedrate in mm/s.
  int32_t extrudePosition;   ///< Current extruder position in steps.
  int16_t watchPeriod;        ///< Time in seconds, a M109 command will wait to stabilize temperature
  int16_t waitRetractTemperature; ///< Temperature to retract the filament when waiting for heat up
  int16_t waitRetractUnits;   ///< Units to retract the filament when waiting for heat up
#if USE_ADVANCE
#if ENABLE_QUADRATIC_ADVANCE
  float advanceK;         ///< Coefficient for advance algorithm. 0 = off
#endif
  float advanceL;
  int16_t advanceBacklash;  // always zero
#endif // USE_ADVANCE
  TemperatureController tempControl;
  const char * PROGMEM selectCommands;
  const char * PROGMEM deselectCommands;
  uint8_t coolerSpeed; ///< Speed to use when enabled
  uint8_t coolerPWM; ///< current PWM setting
  float diameter;
  uint8_t flags;

  // Methods here

  static void step();
  static void unstep();
  static void setDirection(uint8_t dir);
  static void enable();
#if FEATURE_RETRACTION
  inline bool isRetracted() {return (flags & EXTRUDER_FLAG_RETRACTED) != 0;}
  inline void setRetracted(bool on) {
    flags = (flags & (255 - EXTRUDER_FLAG_RETRACTED)) | (on ? EXTRUDER_FLAG_RETRACTED : 0);
  }
  void retract(bool isRetract,bool isLong);
  void retractDistance(float dist,bool extraLength = false);
#endif

  static void manageTemperatures();
  static void disableCurrentExtruderMotor();
  static void disableAllExtruderMotors();
  static void selectExtruderById(uint8_t extruderId);
  static void disableAllHeater();
  static void initExtruder();

  static void initHeatedBed();
  static void setHeatedBedTemperature(float temp_celsius);
  static float getHeatedBedTemperature();

  static void setTemperatureForExtruder(float temp_celsius,uint8_t extr,bool beep = false,bool wait = false);
  static void pauseExtruders(bool bed = false);
  static void unpauseExtruders(bool wait = true);
};


#define HEATED_BED_INDEX NUM_EXTRUDER
#define NUM_TEMPERATURE_LOOPS HEATED_BED_INDEX+1

extern TemperatureController heatedBedController;


#define TEMP_INT_TO_FLOAT(temp) ((float)(temp)/(float)(1<<CELSIUS_EXTRA_BITS))
#define TEMP_FLOAT_TO_INT(temp) ((int)((temp)*(1<<CELSIUS_EXTRA_BITS)))

#if NUM_TEMPERATURE_LOOPS > 0
extern TemperatureController *tempController[NUM_TEMPERATURE_LOOPS];
#endif

extern uint8_t autotuneIndex;


#endif // EXTRUDER_H_INCLUDED
