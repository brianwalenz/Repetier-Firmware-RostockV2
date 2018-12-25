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

#include "Repetier.h"
#include "HAL.h"
#include "gcode.h"
#include "Commands.h"
#include "Printer.h"

SDCard            sd;



SDCard::SDCard() {

  _printName[0] = 0;
  _nLines       = 0;
  _estBuildTime = 0;
  _estFilament  = 0.0;
  _maxHeight    = 0.0;

  _sdActive     = false;
  _sdMode       = SDMODE_IDLE;

  _cwd[0]       = '/';
  _cwd[1]       = 0;

  _nFilesOnCard = 0;

  _fileSize     = 0;
  _filePos      = 0;
}



//  Called in commandLoop()
void
SDCard::automount(void) {
  uint8_t  inserted = (READ(SDCARDDETECT) == 0);   //  If inserted, pin is low.

  //  If the card is there, and we're already active, just return.
  //  If the card is there, and we've failed to mount it, just return.

  if ((inserted == true)  && (_sdActive == true)) {
    return;
  }

  if ((inserted == true)  && (_sdMode == SDMODE_FAILED)) {
    return;
  }

  //  If the card isn't there, and we're flagged as failed, reset status.

  if ((inserted == false) && (_sdMode == SDMODE_FAILED)) {
    _sdActive = false;
    _sdMode   = SDMODE_IDLE;
    return;
  }

  //  If the card isn't there, but we're marked as active, unount the card.

  if ((inserted == false) && (_sdActive == true)) {
    Com::printF(PSTR("SD card removed.\n"));
    unmount();
    //uid.refreshPage();
  }

  //  If the card is there, but we're not active, mount the card.

  if ((inserted == true) && (_sdActive == false)) {
    Com::printF(PSTR("SD card inserted.\n"));
    mount();
    //uid.refreshPage();
  }
}



void
SDCard::mount() {

  _sdActive = false;
  _sdMode   = SDMODE_IDLE;

  //  If the SDCARDDETECT pin is low, there's no card present.
  if (READ(SDCARDDETECT) != 0)
    return;

  hal.pingWatchdog();
  delay(50);

  _fat.begin(SDSS, SD_SCK_MHZ(50)); // dummy init of SD_CARD (???)

  delay(50);

  hal.pingWatchdog();

  //  Try to initialize the card.  If it fals, alert the user.

  if (_fat.begin(SDSS, SD_SCK_MHZ(50)) == false) {
    _sdMode = SDMODE_FAILED;

    //  If an error code (we failed to mount the card), alert the user.
    if (_fat.card()->errorCode()) {
      uid.uiAlert(1);
      return;
    }

    //  If not a valid FAT16/FAT32 partition, alert the user.
    if (_fat.vol()->fatType() == 0) {
      uid.uiAlert(2);
      return;
    }

    //  If no root directory, alert the user.
    if (!_fat.vwd()->isOpen()) {
      uid.uiAlert(3);
      return;
    }

    return;
  }

  Com::printF(PSTR("Card successfully initialized.\n"));

  _sdActive = true;

  Printer::setMenuMode(MODE_CARD_PRESENT, true);

  hal.pingWatchdog();

  _fat.chdir();

  scanCard();

  //  If there's an init.g on the card, print it.
  //sd.printFile("init.g");
}



void
SDCard::unmount() {

  _sdActive = false;
  _sdMode   = SDMODE_IDLE;

  Printer::setMenuMode(MODE_CARD_PRESENT, false);
  Printer::setMenuMode(MODE_PAUSED,       false);
  Printer::setMenuMode(MODE_PRINTING,     false);

  _cwd[0] = '/';
  _cwd[1] = 0;
}



void
SDCard::goDir(const char *name) {
  char *p = _cwd;

  //  Skip to the end of the string.

  while (*p)
    p++;

  //  If no name supplied, move up one level.

  if (name == NULL) {
    if ((_cwd[0] == '/') && (_cwd[1] == 0))
      return;

    p--;  //  Move back off the NUL, now on a /
    p--;  //  Move back off the /.   now on the last letter in the directory name

    while (*p != '/')  //  Search back for the next /
      p--;

    *++p = 0;    //  and make it the end of the string.
  }

  //  Otherwise, copy the directory name onto our path.

  else {
    while (*name)
      *p++ = *name++;

    *p = 0;
  }

  //  Now set the directory and scan the files in it.

  _fat.chdir(_cwd);

  scanCard();
}



uint8_t
SDCard::scanCard(uint8_t filePos, char *filename) {
  char     cardname[MAX_FILENAME_LEN + 1];
  dir_t   *p    = NULL;
  FatFile *root = getvwd();
  FatFile  file;

  uint8_t  nFiles = 0;
  uint8_t  isDir  = 0;

  root->rewind();

#ifdef SHOW_SCANCARD
  Com::printf(PSTR("\nscanCard\n"));
#endif

  while (_file.openNext(root, O_READ)) {
    hal.pingWatchdog();

    _file.getName(cardname, MAX_FILENAME_LEN);
    _file.close();

    //  Skip dot files.

    if ((cardname[0] == '.') && (cardname[1] != '.'))
      continue;

    //  If this is the file we want to get the name for, copy the name.

    if ((filename != NULL) && (nFiles == filePos)) {
      uint8_t  pos = 0;

      for (pos=0; cardname[pos] != 0; pos++)
        filename[pos] = cardname[pos];

      if (_file.isDir()) {
        isDir = 1;
        filename[pos++] = '/';
      }

      filename[pos] = 0;

#ifdef SHOW_SCANCARD
      Com::printf(PSTR("scanCard-- %s - MATCH index %u\n"), cardname, filePos);
    } else {
      Com::printf(PSTR("scanCard-- %s\n"), cardname);
#endif
    }

    nFiles++;
  }

  if (filename == NULL) {
    _nFilesOnCard = nFiles;
#ifdef SHOW_SCANCARD
    Com::printf(PSTR("scanCard-- %d files.\n"), _nFilesOnCard);
#endif
  }

  return(isDir);
}











void
SDCard::startPrint() {

  if (_sdActive == false)
    return;

  _sdMode = SDMODE_PRINTING;

  Printer::setMenuMode(MODE_PRINTING, true);
  Printer::setMenuMode(MODE_PAUSED,   false);

  Printer::setPrinting(true);

  Printer::maxLayer     = 0;
  Printer::currentLayer = 0;

  uid.clearStatus();

  commandQueue.startFile();
}



void
SDCard::pausePrint(bool intern) {

  if (_sdActive == false)
    return;

  _sdMode = SDMODE_STOPPED; // finish running line

  Printer::setMenuMode(MODE_PAUSED, true);
  Printer::setPrinting(false);

  commandQueue.pauseFile();

  if(intern) {
    Commands::waitUntilEndOfAllBuffers();

    Printer::MemoryPosition();
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                        Printer::memoryE - RETRACT_ON_PAUSE,
                        Printer::maxFeedrate[E_AXIS] / 2);

    //Printer::moveToParkPosition();

    Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];
    Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];
    Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];

    //commandQueue.executeFString(PSTR(PAUSE_START_COMMANDS));
  }
}



void
SDCard::continuePrint(bool intern) {

  if (_sdActive == false)
    return;

  if(intern) {
    //commandQueue.executeFString(PSTR(PAUSE_END_COMMANDS));

    Printer::GoToMemoryPosition(true, true, false, false, Printer::maxFeedrate[X_AXIS]);
    Printer::GoToMemoryPosition(false, false, true, false, Printer::maxFeedrate[Z_AXIS] / 2.0f);
    Printer::GoToMemoryPosition(false, false, false, true, Printer::maxFeedrate[E_AXIS] / 2.0f);
  }

  commandQueue.resumeFile();

  Printer::setPrinting(true);
  Printer::setMenuMode(MODE_PAUSED, false);

  _sdMode = SDMODE_PRINTING;
}



void
SDCard::stopPrint() {

  if (_sdActive == false)
    return;

  if (_sdMode == SDMODE_PRINTING)
    Com::printF(PSTR("SD print stopped by user.\n"));

  _sdMode = SDMODE_IDLE;

  Printer::setMenuMode(MODE_PRINTING, false);
  Printer::setMenuMode(MODE_PAUSED,   false);
  Printer::setPrinting(false);

  commandQueue.stopFile();

  //  Execute some gcode on stopping.
  //GCode::executeFString(PSTR(SD_RUN_ON_STOP));

  Commands::waitUntilEndOfAllMoves();

  //  Disable heaters and motors
  Printer::kill(false);
}



bool
SDCard::openFile(const char *filename) {

  //  Close any open file.
  _file.close();

  //  Open the file.
  if (_file.open(_fat.vwd(), filename, O_READ) == false) {
    Com::printf(PSTR("Failed to open file '%s'.\n"), filename);
    return(false);
  }

  _filePos  = 0;
  _fileSize = _file.fileSize();

  return(true);
}



void
SDCard::savePrintName(const char *filename) {
  uint8_t  pp = 0;
  uint8_t  ff = 0;

  for (pp=0; (pp < 40); pp++)
    _printName[pp] = 0;

  for (pp=0, ff=0; ((filename[ff] != 0) && (pp < 40)); ff++) {
    _printName[pp++] = filename[ff];

    if (filename[ff] == '/')
      pp = 0;
  }

  _printName[pp] = 0;
}



void
SDCard::countLines(void) {
  char    buf[96];
  int8_t  len;

  //                     --------------------
  uid.printRowP(0, PSTR("Scanning file."));
  uid.printRowP(1, PSTR(" xxxxxxxxx lines."));
  uid.printRowP(2, PSTR(" -----.--- mm tall."));
  uid.printRowP(3, PSTR(""));

  //  For 33651 lines:
  //    No update  -  3472 ms, 3472 ms, 3472 ms
  //    every 1023 -  3746 ms
  //    every  255 -  3989 ms
  //    every line - 91534 ms
  //
  //  For 
  //    No update  - 222325 ms
  //    Every 4095 - 
  //    Every 1023 - 238514 ms
  //

  _nLines = 0;

  return;

  _file.seekSet(0);

  len = _file.read(buf, 96);

  while (len > 0) {
    for (int8_t pp=0; pp<len; pp++) {
      if (buf[pp] == '\n') {
        _nLines++;

        if ((_nLines & 0x0fff) == 0) {
          snprintf(buf, 96, " %9lu lines.", _nLines);
          uid.printRow(1, buf);
        }
      }
    }

    len = _file.read(buf, 96);
  }
  
  snprintf(buf, 96, " %9lu lines.", _nLines);
  uid.printRow(1, buf);

  _file.seekSet(0);
}



//
//  PARSING STATS FROM FILE, ASSUMES SIMPLIFY3D:
//
//  Look for "G1 Z#" to get current layer (also includes moves where the head is raised).
//  Look for comments "; layer #, Z = #"
//
//  At the end of the file:
//    ; Build Summary
//    ;   Build time: 0 hours 34 minutes
//    ;   Filament length: 3542.7 mm (3.54 m)
//    ;   Plastic volume: 8521.23 mm^3 (8.52 cc)
//    ;   Plastic weight: 10.65 g (0.02 lb)
//    ;   Material cost: 0.49
//
//  Returns:
//    nLines
//    maxHeight
//    estBuildTime
//    estFilament
//


//  In gcodecommand.cpp
extern float parseFloatValue(char *&s);
extern long  parseLongValue(char *s);

void
SDCard::findStatistics(void) {
  uint32_t   rewind = 32768;

  if (_fileSize > rewind)
    _file.seekSet(_fileSize - rewind);
  else
    _file.seekSet(0);

  char    line[MAX_CMD_SIZE];
  int8_t  len = _file.fgets(line, MAX_CMD_SIZE);

  while (len > 0) {

    //  Extract maximum Z height.
    //
    if ((line[0] == 'G') &&
        (line[1] == '1') &&
        (line[2] == ' ') &&
        (line[3] == 'Z')) {
      char *L = line + 4;
      float Z = parseFloatValue(L);

      if (_maxHeight < Z)
        _maxHeight = Z;

      snprintf(line, MAX_CMD_SIZE, " %9.4f mm tall.", (double)_maxHeight);
      uid.printRow(2, line);
    }

    //  Extract estimated build time.
    //
    else if ((line[ 0] == ';') &&
             (line[ 4] == 'B') &&
             (line[ 8] == 'd') &&
             (line[10] == 't')) {
      char *L = line + 15;

      uint32_t hours   = parseLongValue(L);

      while (*L != 's')
        L++;
      L++;

      uint32_t minutes = parseLongValue(L);

      _estBuildTime  = 3600 * hours;
      _estBuildTime +=   60 * minutes;
      _estBuildTime *= 1000;
    }

    //  Extract estimated filament usage.
    //
    else if ((line[ 0] == ';') &&
             (line[ 4] == 'F') &&
             (line[ 8] == 'm') &&
             (line[10] == 'n')) {
      char *L = line + 21;

      _estFilament = parseFloatValue(L);
    }

    //  Extract estimated filament weight.
    //
    else if ((line[ 0] == ';') &&
             (line[ 4] == 'P') &&
             (line[ 8] == 't') &&
             (line[12] == 'w')) {
      char *L = line + 20;

      _estWeight = parseFloatValue(L);
    }

    len = _file.fgets(line, MAX_CMD_SIZE);
  }

  _filePos = 0;
  _file.seekSet(0);
}



bool
SDCard::printFile(const char *filename) {

  //  If no SD card, fail.

  if (_sdActive == false)
    return(false);

  //  Clear the display

  openFile(filename);

  //  Scan the file for statistics.

  _printName[0]  = 0;
  _nLines        = 0;
  _estBuildTime  = 0;
  _estFilament   = 0.0;
  _estWeight     = 0.0;
  _maxHeight    = 0.0;

  savePrintName(filename);
  countLines();
  findStatistics();

  Com::printf(PSTR("Filename:                  '%s'\n"),         _printName);
  Com::printf(PSTR("Number of lines:           %ld.\n"),         _nLines);
  Com::printf(PSTR("Maximum Z height:          %.4f mm.\n"),     _maxHeight);
  Com::printf(PSTR("Estimated build time:      %ld seconds.\n"), _estBuildTime);
  Com::printf(PSTR("Estimated filament usage:  %.2f mm.\n"),     _estFilament);
  Com::printf(PSTR("Estimated filament weight: %.2f g.\n"),      _estWeight);

  //  Start the print.

  startPrint();

  return(true);
}
