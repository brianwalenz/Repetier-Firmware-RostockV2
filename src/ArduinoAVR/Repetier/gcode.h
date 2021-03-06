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
#ifndef _GCODE_H
#define _GCODE_H

#include "SDCard.h"

class Commands;

//#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))

// This class defines the general interface to handle gcode communication with the firmware. This
// allows it to connect to different data sources and handle them all inside the same data structure.
// If several readers are active, the first one sending a byte pauses all other inputs until the command
// is complete. Only then the next reader will be queried. New queries are started in round robin fashion
// so every channel gets the same chance to send commands.
//
// Available source types are:
// - serial communication port
// - sd card


#define GCODE_QUEUE_LENGTH  3    //  Number of commands we buffer.
#define MAX_CMD_SIZE       96    //  Maximum length of a command line.

//  Previous enum FirmwareState
#define GCODE_NOT_BUSY      0
#define GCODE_PROCESSING    1
#define GCODE_PAUSED        2
#define GCODE_WAIT_HEATER   3


#define GCODE_N           0x00000001
#define GCODE_M           0x00000002
#define GCODE_G           0x00000004
#define GCODE_X           0x00000008
#define GCODE_Y           0x00000010
#define GCODE_Z           0x00000020
#define GCODE_E           0x00000040
#define GCODE_F           0x00000080
#define GCODE_T           0x00000100
#define GCODE_S           0x00000200
#define GCODE_P           0x00000400
#define GCODE_xs          0x00000800  //  Former string
#define GCODE_I           0x00001000
#define GCODE_J           0x00002000
#define GCODE_R           0x00004000
#define GCODE_D           0x00008000
#define GCODE_C           0x00010000
#define GCODE_H           0x00020000
#define GCODE_A           0x00040000
#define GCODE_B           0x00080000
#define GCODE_K           0x00100000
#define GCODE_L           0x00200000
#define GCODE_O           0x00400000
#define GCODE_x1          0x00800000
#define GCODE_x2          0x01000000
#define GCODE_x3          0x02000000
#define GCODE_x4          0x04000000
#define GCODE_x5          0x08000000
#define GCODE_x6          0x10000000
#define GCODE_x7          0x20000000
#define GCODE_x8          0x40000000
#define GCODE_x9          0x80000000




class gcodeCommand {
public:
  inline bool hasG(void)           { return((_has & GCODE_G) != 0); };
  inline bool hasM(void)           { return((_has & GCODE_M) != 0); };
  inline bool hasT(void)           { return((_has & GCODE_T) != 0); };

  inline bool hasS(void)           { return((_has & GCODE_S) != 0); };
  inline bool hasP(void)           { return((_has & GCODE_P) != 0); };

  inline bool hasX(void)           { return((_has & GCODE_X) != 0); };
  inline bool hasY(void)           { return((_has & GCODE_Y) != 0); };
  inline bool hasZ(void)           { return((_has & GCODE_Z) != 0); };

  inline bool hasI(void)           { return((_has & GCODE_I) != 0); };
  inline bool hasJ(void)           { return((_has & GCODE_J) != 0); };

  inline bool hasD(void)           { return((_has & GCODE_D) != 0); };
  inline bool hasH(void)           { return((_has & GCODE_H) != 0); };

  inline bool hasF(void)           { return((_has & GCODE_F) != 0); };
  inline bool hasE(void)           { return((_has & GCODE_E) != 0); };

  inline bool hasR(void)           { return((_has & GCODE_R) != 0); };

  inline bool hasN(void)           { return((_has & GCODE_N) != 0); };

  inline bool hasNoXYZ(void) {
    return((hasX() == false) && (hasY() == false) && (hasZ() == false));
  };

  inline void setG(void)           { _has |= GCODE_G; };
  inline void setM(void)           { _has |= GCODE_M; };
  inline void setT(void)           { _has |= GCODE_T; };

  inline void setS(void)           { _has |= GCODE_S; };
  inline void setP(void)           { _has |= GCODE_P; };

  inline void setX(void)           { _has |= GCODE_X; };
  inline void setY(void)           { _has |= GCODE_Y; };
  inline void setZ(void)           { _has |= GCODE_Z; };

  inline void setI(void)           { _has |= GCODE_I; };
  inline void setJ(void)           { _has |= GCODE_J; };

  inline void setD(void)           { _has |= GCODE_D; };
  inline void setH(void)           { _has |= GCODE_H; };

  inline void setF(void)           { _has |= GCODE_F; };
  inline void setE(void)           { _has |= GCODE_E; };

  inline void setR(void)           { _has |= GCODE_R; };

  inline void setN(void)           { _has |= GCODE_N; };


  inline void unsetX(void)         { _has &= ~GCODE_X; };
  inline void unsetY(void)         { _has &= ~GCODE_Y; };
  inline void unsetZ(void)         { _has &= ~GCODE_Z; };


  inline uint16_t getM(void)       { return(M);                };
  inline long     getS(long def)   { return(hasS() ? S : def); };
  inline long     getP(long def)   { return(hasP() ? P : def); };

private:
  uint32_t _has;

public:
  uint16_t G;
  uint16_t M;
  uint8_t  T;

  int32_t  S;
  int32_t  P;

  float    X;
  float    Y;
  float    Z;

  float    I;
  float    J;

  float    D;
  float    H;

  float    F;
  float    E;

  float    R;

  uint16_t N;  //  Line number

  //uint16_t LN;     //  Line number this command should be

public:
  bool parseCommand(char *line);
  void printCommand(void);
};










//  Read data from either the serial port or the SD Card.
//  When an SD card file is selected via the interface, _sdCardActive must be set.
//  We'll read bytes from there until the file is empty, then go back to
//  waiting for serial port commands.


class gcodeQueue {
public:
  gcodeQueue() {
    _isPrinting     = false;
    _startTime      = 0;
    _stopTime       = 0;
    _lineNumber     = 0;
    _currentHeight  = 0.0;

    _cmdsLen        = 0;
    _cmdsOut        = 0;
    _cmdsIn         = 0;
  };
  ~gcodeQueue() {
  };


  //  Return next command, if it exists.
  //
  gcodeCommand   *popCommand(void) {
    gcodeCommand *out = NULL;

    if (_cmdsLen > 0) {
      out = _cmds + _cmdsOut;

      _cmdsOut++;
      _cmdsLen--;

      if (_cmdsOut == GCODE_QUEUE_LENGTH)
        _cmdsOut = 0;
    }

    return(out);
  };


private:
  bool      dataAvailable(void) {          //  Returns true if there is data to read.
    if ((sd.isPrinting() == true) &&       //  If the SD card isn't finished, data is available.
        (sd.isFinished() == false))
      return(true);

    if (Serial.available() == true)        //  If the serial port has data, data is available.
      return(true);

    return(false);
  };

  int8_t      readByte(void) {             //  Returns a byte from the input.
    if ((sd.isPrinting() == true) &&       //  If the SD card isn't finished, data is available.
        (sd.isFinished() == false))
      return(sd.readByte());

    if (Serial.available() == true)        //  If the serial port has data, data is available.
      return(Serial.read());

    return(0);
  };


public:
  void      loadNext(void);
  void      executeNext(void) {
    if (_isPrinting == false)               //  If not printing, return without doing anything.
      return;

    if (_cmdsLen >= GCODE_QUEUE_LENGTH)     //  If the buffer is full, return without doing
      return;                               //  anything; wait for the commands to execute.

    loadNext();                             //  Otherwise, load the next command.
  };

  void      startPrint(void);
  void      stopPrint(void);

  void      pausePrint(void)  {};
  void      resumePrint(void) {};


  uint32_t  elapsedTime(void)    { return(millis()  - _startTime); };
  uint32_t  totalTime(void)      { return(_stopTime - _startTime); };
  uint32_t  lineNumber(void)     { return(_lineNumber);            };
  float     currentHeight(void)  { return(_currentHeight);         };

private:
  uint8_t       _isPrinting;
  uint32_t      _startTime;
  uint32_t      _stopTime;
  uint32_t      _lineNumber;
  float         _currentHeight;

private:
  gcodeCommand  _cmds[GCODE_QUEUE_LENGTH];   //  Buffer of received commands.
  uint8_t       _cmdsLen;                    //
  uint8_t       _cmdsOut;                    //  Index of the next buffer to output to the processor.
  uint8_t       _cmdsIn;                     //  Index of the next buffer to fill from the input file.
};


extern gcodeQueue  commandQueue;


#endif
