

#ifndef SDCARD_H
#define SDCARD_H

#define SDMODE_IDLE       0
#define SDMODE_PRINTING   1
#define SDMODE_STOPPED    2
#define SDMODE_FAILED   255

#define MAX_FILENAME_LEN     48  //  could overflow in src/SdFat/FatLib/FatFileLFN.cpp
#define MAX_CWD_LEN         128


#include "Repetier.h"
#include "HAL.h"
#include "Communication.h"
#include "src/SdFat/SdFat.h"

class SDCard {
public:
  SDCard();

public:
  void automount(void);
private:
  void mount(void);
  void unmount(void);

public:
  FatFile *getvwd(void) {
    return(_fat.vwd());
  };

  void    goDir(const char *path);

  uint8_t scanCard(uint8_t filePos = 255, char *filename = NULL);

private:
  bool    openFile(const char *filename);
  void    savePrintName(const char *filename);
  void    countLines(void);
  void    findStatistics(void);

public:
  bool    printFile(const char *filename);

  void    startPrint(void);
  void    pausePrint(bool intern = false);
  void    continuePrint(bool intern = false);
  void    stopPrint(void);

  //  To delete a file:    _fat.remove(filename)
  //  To delete a dir:     _fat.rmdir(filename)
  //  To create a dir:     _fat.mkdir(filename)

  //  Used in SDCardGCodeSouce::isOpen()
  bool    isOpen(void)       {  return((_sdMode != SDMODE_IDLE) &&
                                       (_sdMode != SDMODE_FAILED));  };
  bool    isPrinting(void)   {  return(_sdMode == SDMODE_PRINTING);  };
  bool    isFinished(void)   {  return(_filePos == _fileSize);    };

  int8_t  readByte(void) {
    return(_file.read());
  }

  //int8_t  readLine(char *line, uint8_t maxLen);

  //  For the file being printed.
public:
  char       *printName(void)      { return(_printName);    };
  uint32_t    nLines(void)         { return(_nLines);       };
  uint32_t    estBuildTime(void)   { return(_estBuildTime); };
  float       estFilament(void)    { return(_estFilament);  };
  float       estWeight(void)      { return(_estWeight);    };
  float       maxHeight(void)      { return(_maxHeight);    };

private:
  char        _printName[41];
  uint32_t    _nLines;
  uint32_t    _estBuildTime;
  float       _estFilament;
  float       _estWeight;
  float       _maxHeight;

private:
  SdFat       _fat;
  uint8_t     _sdActive;
  uint8_t     _sdMode;  // 1 if we are printing from sd card, 2 = stop accepting new commands

private:
  char        _cwd[MAX_CWD_LEN + 1];

public:
  uint8_t     _nFilesOnCard;

private:
  SdFile      _file;
  uint32_t    _fileSize;   //  size of current file?
  uint32_t    _filePos;    //  position we're printing at in the current file
};

extern SDCard sd;

#endif  //  SDCARD_H
