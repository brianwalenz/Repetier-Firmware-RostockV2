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

#define MAX_CMD_SIZE 96
#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))

enum FirmwareState {
  NotBusy=0,
  Processing,
  Paused,
  WaitHeater
};

#include "SDCard.h"

class Commands;







#ifndef MAX_DATA_SOURCES
#define MAX_DATA_SOURCES 4
#endif

/** This class defines the general interface to handle gcode communication with the firmware. This
    allows it to connect to different data sources and handle them all inside the same data structure.
    If several readers are active, the first one sending a byte pauses all other inputs until the command
    is complete. Only then the next reader will be queried. New queries are started in round robin fashion
    so every channel gets the same chance to send commands.

    Available source types are:
    - serial communication port
    - sd card
    - flash memory
*/

class GCodeSource {
  static fast8_t      numSources;
  static GCodeSource *sources[MAX_DATA_SOURCES];

public:
  static GCodeSource *activeSource;

  static void registerSource(GCodeSource *newSource);
  static void removeSource(GCodeSource *delSource);
  static void rotateSource();


  uint32_t lastLineNumber;
  uint8_t  wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
  millis_t timeOfLastDataPacket;
  int8_t   waitingForResend; ///< Waiting for line to be resend. -1 = no wait.

  GCodeSource() {
    lastLineNumber                 = 0;
    wasLastCommandReceivedAsBinary = false;
    timeOfLastDataPacket           = 0;
    waitingForResend               = -1;
  };
  virtual ~GCodeSource() {}

  virtual bool isOpen(void) = 0;
  virtual bool closeOnError(void) = 0; // return true if the channel can not interactively correct errors.
  virtual bool dataAvailable(void) = 0; // would read return a new byte?
  virtual int  readByte(void) = 0;
  virtual void close(void) = 0;
};



class SerialGCodeSource: public GCodeSource {
  Stream *stream;
public:
  SerialGCodeSource(Stream *p)      { stream = p;                  };
  virtual bool isOpen(void)         { return(true);                };
  virtual bool closeOnError(void)   { return(false);               };
  virtual bool dataAvailable(void)  { return(stream->available()); };
  virtual int  readByte(void)       { return(stream->read());      };
  virtual void close(void)          { return;                      };
};


class SDCardGCodeSource: public GCodeSource {
public:
  virtual bool isOpen(void)          { return(sd.isOpen()); };
  virtual bool closeOnError(void)    { return(true); };
  virtual bool dataAvailable(void) {
    if (sd.isPrinting() == false)
      return(false);

    if (sd.isFinished() == false)
      return(true);

    close();
    return(false);
  };
  virtual int  readByte(void) {
    int8_t n = sd.file.read();

    if (n == -1)
      close();

    return(n);
  };
  virtual void close(void) {
    sd.stopPrint();

    //  stopPrint() removes the source too.
    //GCodeSource::removeSource(this);  

    Com::printF(PSTR("Done printing file.\n"));
  };
};


extern SerialGCodeSource serial0Source;
extern SDCardGCodeSource sdSource;











class GCode   // 52 uint8_ts per command needed
{
  uint16_t params;
  uint16_t params2;

public:
  uint16_t N;  //  Line number
  uint16_t M;
  uint16_t G;
  float    X;
  float    Y;
  float    Z;
  float    E;
  float    F;
  int32_t  S;
  int32_t  P;
  float    I;
  float    J;
  float    R;
  float    D;
  float    C;
  float    H;
  float    A;
  float    B;
  float    K;
  float    L;
  float    O;
  char    *text;   //  Message
  uint8_t  T;

  // True if origin did not come from serial console. That way we can send status messages to
  // a host only if he would normally not know about the mode switch.
  bool internalCommand;

  inline bool hasN()           { return ((params & 1) != 0);}
  inline bool hasM()           { return ((params & 2) != 0);}
  inline bool hasG()           { return ((params & 4) != 0);}
  inline bool hasX()           { return ((params & 8) != 0);}
  inline bool hasY()           { return ((params & 16) != 0);}
  inline bool hasZ()           { return ((params & 32) != 0);}
  inline bool hasNoXYZ()       { return ((params & 56) == 0);}
  inline bool hasE()           { return ((params & 64) != 0);}
  inline bool hasF()           { return ((params & 256) != 0);}
  inline bool hasT()           { return ((params & 512) != 0);}
  inline bool hasS()           { return ((params & 1024) != 0);}
  inline bool hasP()           { return ((params & 2048) != 0);}
  inline bool isV2()           { return ((params & 4096) != 0);}
  inline bool hasString()      { return ((params & 32768) != 0);}
  inline bool hasI()           { return ((params2 & 1) != 0);}
  inline bool hasJ()           { return ((params2 & 2) != 0);}
  inline bool hasR()           { return ((params2 & 4) != 0);}
  inline bool hasD()           { return ((params2 & 8) != 0);}
  inline bool hasC()           { return ((params2 & 16) != 0);}
  inline bool hasH()           { return ((params2 & 32) != 0);}
  inline bool hasA()           { return ((params2 & 64) != 0);}
  inline bool hasB()           { return ((params2 & 128) != 0);}
  inline bool hasK()           { return ((params2 & 256) != 0);}
  inline bool hasL()           { return ((params2 & 512) != 0);}
  inline bool hasO()           { return ((params2 & 1024) != 0);}
  inline long getS(long def)   { return (hasS() ? S : def);}
  inline long getP(long def)   { return (hasP() ? P : def);}

  inline void unsetX()         { params &= ~8;}
  inline void unsetY()         { params &= ~16;}
  inline void unsetZ()         { params &= ~32;}

  inline void setFormatError() { params2 |= 32768;}
  inline bool hasFormatError() { return ((params2 & 32768) != 0);}

	static FSTRINGPARAM(fatalErrorMsg);

	inline static bool hasFatalError() {return fatalErrorMsg != NULL;}

  void printCommand();

  bool parseBinary(uint8_t *buffer,bool fromSerial);
  bool parseAscii(char *line,bool fromSerial);

  void popCurrentCommand();
  void echoCommand();

  /** Get next command in command buffer. After the command is processed, call gcode_command_finished() */
  static GCode *peekCurrentCommand();

  /** Frees the cache used by the last command fetched. */
  static void readFromSerial();
  static void pushCommand();
  static void executeFString(FSTRINGPARAM(cmd));
  static uint8_t computeBinarySize(char *ptr);
	static void fatalError(FSTRINGPARAM(message));
	static void reportFatalError();
	static void resetFatalError();
	static void keepAlive(enum FirmwareState state);
	static uint32_t keepAliveInterval;

  //friend class SDCard;
  //friend class UIDisplay;

  friend class GCodeSource;    

protected:
  void debugCommandBuffer();
  void checkAndPushCommand();

  static void requestResend();

  inline float parseFloatValue(char *s) {
    char *endPtr;
    while(*s == 32) s++; // skip spaces
    float f = (strtod(s, &endPtr));
    if(s == endPtr) f=0.0; // treat empty string "x " as "x0"
    return f;
  }

  inline long parseLongValue(char *s) {
    char *endPtr;
    while(*s == 32) s++; // skip spaces
    long l = (strtol(s, &endPtr, 10));
    if(s == endPtr) l=0; // treat empty string argument "p " as "p0"
    return l;
  }

  static GCode   commandsBuffered[GCODE_BUFFER_SIZE]; ///< Buffer for received commands.
  static uint8_t bufferReadIndex; ///< Read position in gcode_buffer.
  static uint8_t bufferWriteIndex; ///< Write position in gcode_buffer.
  static uint8_t commandReceiving[MAX_CMD_SIZE]; ///< Current received command.
  static uint8_t commandsReceivingWritePosition; ///< Writing position in gcode_transbuffer.
  static uint8_t sendAsBinary; ///< Flags the command as binary input.
  static uint8_t commentDetected; ///< Flags true if we are reading the comment part of a command.
  static uint8_t binaryCommandSize; ///< Expected size of the incoming binary command.
  static bool    waitUntilAllCommandsAreParsed; ///< Don't read until all commands are parsed. Needed if gcode_buffer is misused as storage for strings.
  static uint32_t actLineNumber; ///< Line number of current command.
  static volatile uint8_t bufferLength; ///< Number of commands stored in gcode_buffer
  static uint8_t formatErrors; ///< Number of sequential format errors
	static millis_t lastBusySignal; ///< When was the last busy signal

public:
  GCodeSource *source;    
};



#endif

