


#define SDMODE_IDLE       0
#define SDMODE_PRINTING   1
#define SDMODE_STOPPED    2
#define SDMODE_FAILED   255


class SDCard {
public:
  SDCard();

public:
  void automount();
private:
  void mount();
  void unmount();

public:
  void chdir(const char *path) {
    _fat.chdir(path);
  };

  FatFile *getvwd(void) {
    return(_fat.vwd());
  };

  bool   selectFile(const char *filename,bool silent=false);

  void   startPrint(void);
  void   pausePrint(bool intern = false);
  void   continuePrint(bool intern = false);
  void   stopPrint(void);

  //  To delete a file:    _fat.remove(filename)
  //  To delete a dir:     _fat.rmdir(filename)
  //  To create a dir:     _fat.mkdir(filename)

  //  Used in SDCardGCodeSouce::isOpen()
  bool   isOpen(void)       {  return((_sdMode != SDMODE_IDLE) &&
                                    (_sdMode != SDMODE_FAILED));  };
  bool   isPrinting(void)   {  return(_sdMode == SDMODE_PRINTING);  };
  bool   isFinished(void)   {  return(sdpos == filesize);    };

  int8_t readByte(void);

private:
  SdFat       _fat;
  uint8_t     _sdActive;
  uint8_t     _sdMode;  // 1 if we are printing from sd card, 2 = stop accepting new commands

public:
  SdFile      file;

private:
  uint32_t    filesize;   //  size of current file?
  uint32_t    sdpos;      //  position we're printing at in the current file
};

extern SDCard sd;
