

#ifndef SDCARD_H
#define SDCARD_H

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
  void    closeFile(void);
  void    savePrintName(const char *filename);
  void    countLines(void);
  void    findStatistics(void);

public:
  bool    analyzeFile(const char *filename);

  void    startFile(void)   { _filePrinting  = true;                  };
  void    stopFile(void)    { _filePrinting  = false;   closeFile();  };
  void    abortFile(void)   { _filePrinting  = false;   closeFile();  };

  //  To delete a file:    _fat.remove(filename)
  //  To delete a dir:     _fat.rmdir(filename)
  //  To create a dir:     _fat.mkdir(filename)

  bool    isAvailable(void)  {  return(_cardPresent  == true);        };
  bool    isPrinting(void)   {  return(_filePrinting == true);        };
  bool    isFinished(void)   {  return(_filePos      == _fileSize);   };

  int8_t  readByte(void) {
    int8_t  c = _file.read();     //  Read a byte from the file.

    if (c == -1) {                //  If an error, abort the file;
      abortFile();                //    sets filePos to fileLength, changes
      c = 0;                      //    mode to idle.
    }

    return(c);
  }

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
  uint8_t     _cardPresent;    //  True if there is a card present.
  uint8_t     _cardFailed;     //  True if the card failed to mount.
  uint8_t     _filePrinting;   //  True if we're reading bytes from the card.

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
