#ifndef EXTRUDER_H_INCLUDED
#define EXTRUDER_H_INCLUDED

#include "HAL.h"

#define VIRTUAL_EXTRUDER 16 // don't change this to more then 16 without modifying the eeprom positions

#define EXTRUDER_FLAG_RETRACTED 1


// Data to drive one extruder.
//
//    This structure contains all definitions for an extruder and all
//    current state variables, like current temperature, feeder position etc.

class Extruder {
public:
  Extruder(uint8_t idx);
  ~Extruder() {};

public:
  uint8_t  id;

  int32_t  xOffset;                 //  Offsets in STEPS, not MM!
  int32_t  yOffset;
  int32_t  zOffset;

  uint8_t  invert;

  float    stepsPerMM;              //  Steps per mm.

  int8_t   enablePin;               //  Pin to enable extruder stepper motor.
  uint8_t  enableOn;

  int8_t   dirPin;
  int8_t   stepPin;

  float    maxFeedrate;             //  Maximum feedrate in mm/s.
  float    maxAcceleration;         //  Maximum acceleration in mm/s^2.
  float    maxStartFeedrate;        //  Maximum start feedrate in mm/s.
  int32_t  extrudePosition;         //  Current extruder position in steps.
  int16_t  watchPeriod;             //  Time in seconds, a M109 command will wait to stabilize temperature

  int16_t  waitRetractTemperature;  //  Temperature to retract the filament when waiting for heat up
  int16_t  waitRetractUnits;        //  Units to retract the filament when waiting for heat up; 0 = disable

  //  Factors for the advance algorithm.
  //  K is the factor for the quadratic term, which is normally disabled in newer versions.
  //  L is the linear factor and seems to be working better then the quadratic dependency.
  //
  float    advanceK;                //  Coefficient for advance algorithm. 0 = off
  float    advanceL;

  //  Motor steps to remove backlash for advance algorithm. These are the steps
  //  needed to move the motor cog in reverse direction until it hits the driving
  //  cog. Direct drive extruder need 0.
  //
  int16_t  advanceBacklash;         //  always zero

  float    diameter;
  uint8_t  flags;

  const char * PROGMEM selectCommands;
  const char * PROGMEM deselectCommands;

public:
  void initialize(void) {

    Com::printF(PSTR("Extruder initialize.\n"));

    if (dirPin    > -1)   pinMode(dirPin, OUTPUT);
    if (stepPin   > -1)   pinMode(stepPin, OUTPUT);
    if (enablePin > -1)   pinMode(enablePin, OUTPUT);

    if (enablePin > -1)
      digitalWrite(enablePin, !enableOn);
  };

  void enable(void)  {  digitalWrite(enablePin,  enableOn);        };
  void disable(void) {  digitalWrite(enablePin, !enableOn);        };

  void step(void)    {  digitalWrite(stepPin,    START_STEP_WITH_HIGH);  };
  void unstep(void)  {  digitalWrite(stepPin,   !START_STEP_WITH_HIGH);  };

  void setDirection(uint8_t dir) {
    if (dir)
      digitalWrite(dirPin, !invert);
    else
      digitalWrite(dirPin,  invert);
  };


  bool isRetracted()           { return (flags & EXTRUDER_FLAG_RETRACTED) != 0; }
  void setRetracted(bool on)   { flags = (flags & (255 - EXTRUDER_FLAG_RETRACTED)) | (on ? EXTRUDER_FLAG_RETRACTED : 0); }

private:
  void retractDistance(float dist,
                      bool extraLength = false);

public:
  //  This is G10 and G11.
  void retract(bool isRetract,
               bool isLong);
};


extern Extruder  extruder;
extern Extruder *activeExtruder;

#endif // EXTRUDER_H_INCLUDED
