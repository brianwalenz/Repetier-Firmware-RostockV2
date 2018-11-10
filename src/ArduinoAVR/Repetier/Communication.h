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
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

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
  static fast8_t numSources; ///< Number of data sources available
  static fast8_t numWriteSources;
  static GCodeSource *sources[MAX_DATA_SOURCES];
  static GCodeSource *writeableSources[MAX_DATA_SOURCES];

public:
  static GCodeSource *activeSource;

  static void registerSource(GCodeSource *newSource);
  static void removeSource(GCodeSource *delSource);
  static void rotateSource(); ///< Move active to next source

  static void writeToAll(uint8_t byte) { ///< Write to all listening sources 
    fast8_t i;

    for(i = 0; i < numWriteSources; i++) {
      writeableSources[i]->writeByte(byte);
    }
  };



  uint32_t lastLineNumber;
  uint8_t wasLastCommandReceivedAsBinary; ///< Was the last successful command in binary mode?
  millis_t timeOfLastDataPacket;
  int8_t waitingForResend; ///< Waiting for line to be resend. -1 = no wait.

  GCodeSource();
  virtual ~GCodeSource() {}
  virtual bool isOpen() = 0;
  virtual bool supportsWrite() = 0; ///< true if write is a non dummy function
  virtual bool closeOnError() = 0; // return true if the channel can not interactively correct errors.
  virtual bool dataAvailable() = 0; // would read return a new byte?
  virtual int readByte() = 0;
  virtual void close() = 0;
  virtual void writeByte(uint8_t byte) = 0;
};



class Com {
public:
  static void printF(FSTRINGPARAM(text));
  static void print(const char *text);

  static void printF(FSTRINGPARAM(text), const char *msg) {
    printF(text);
    print(msg);
  };
  static void printF(FSTRINGPARAM(text), int value) {
    printF(text);
    print((int32_t)value);
  };
  static void printF(FSTRINGPARAM(text), int32_t value) {
    printF(text);
    print(value);
  };
  static void printF(FSTRINGPARAM(text), uint32_t value) {
    printF(text);
    printNumber(value);
  };
  static void printF(FSTRINGPARAM(text), float value, uint8_t digits=2) {
    printF(text);
    printFloat(value, digits);
  };

  static void print(char c)          { GCodeSource::writeToAll(c); }

  static void print(uint8_t value)   { print((uint32_t)value); };
  static void print( int8_t value)   { print(( int32_t)value); };

  static void print(uint16_t value)  { print((uint32_t)value); };
  static void print( int16_t value)  { print(( int32_t)value); };

  static void print(uint32_t value)  { printNumber(value);     };
  static void print( int32_t value)  {
    if (value < 0) {
      print('-');
      value = -value;
    }
    printNumber((uint32_t)value);
  };

  static void print(float number)    { printFloat(number, 6); }



  static void printNumber(uint32_t n);
  static void printFloat(float number, uint8_t digits);

  static void printArrayF(FSTRINGPARAM(text),float *arr,uint8_t n=4,uint8_t digits=2);
  static void printArrayF(FSTRINGPARAM(text),long *arr,uint8_t n=4);
};

#endif // COMMUNICATION_H
