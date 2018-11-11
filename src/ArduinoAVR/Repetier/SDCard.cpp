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



SDCardGCodeSource sdSource;
SDCard            sd;



SDCard::SDCard() {
  _sdActive = false;
  _sdMode   = SDMODE_IDLE;

  _cwd[0] = '/';
  _cwd[1] = 0;

  _nFilesOnCard = 0;

  filesize = 0;
  sdpos = 0;
}



//  This is called from Printer::defaultLoopActions()
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

  HAL::pingWatchdog();
  HAL::delayMilliseconds(50);

  _fat.begin(SDSS, SD_SCK_MHZ(50)); // dummy init of SD_CARD (???)

  HAL::delayMilliseconds(50);

  HAL::pingWatchdog();

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

  HAL::pingWatchdog();

  _fat.chdir();

  uid.scanSDcard();

  //  If there's an init.g on the card, print it.
  if (selectFile("init.g", true))
    startPrint();
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

  GCodeSource::registerSource(&sdSource);
}



void
SDCard::pausePrint(bool intern) {

  if (_sdActive == false)
    return;

  _sdMode = SDMODE_STOPPED; // finish running line

  Printer::setMenuMode(MODE_PAUSED, true);
  Printer::setPrinting(false);

  GCodeSource::removeSource(&sdSource);

  if(intern) {
    Commands::waitUntilEndOfAllBuffers();

    Printer::MemoryPosition();
    Printer::moveToReal(IGNORE_COORDINATE, IGNORE_COORDINATE, IGNORE_COORDINATE,
                        Printer::memoryE - RETRACT_ON_PAUSE,
                        Printer::maxFeedrate[E_AXIS] / 2);

    Printer::moveToParkPosition();

    Printer::lastCmdPos[X_AXIS] = Printer::currentPosition[X_AXIS];
    Printer::lastCmdPos[Y_AXIS] = Printer::currentPosition[Y_AXIS];
    Printer::lastCmdPos[Z_AXIS] = Printer::currentPosition[Z_AXIS];

    GCode::executeFString(PSTR(PAUSE_START_COMMANDS));
  }
}



void
SDCard::continuePrint(bool intern) {

  if (_sdActive == false)
    return;

  if(intern) {
    GCode::executeFString(PSTR(PAUSE_END_COMMANDS));

    Printer::GoToMemoryPosition(true, true, false, false, Printer::maxFeedrate[X_AXIS]);
    Printer::GoToMemoryPosition(false, false, true, false, Printer::maxFeedrate[Z_AXIS] / 2.0f);
    Printer::GoToMemoryPosition(false, false, false, true, Printer::maxFeedrate[E_AXIS] / 2.0f);
  }

  GCodeSource::registerSource(&sdSource);

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

  GCodeSource::removeSource(&sdSource);

  //  Execute some gcode on stopping.
  //GCode::executeFString(PSTR(SD_RUN_ON_STOP));

  Commands::waitUntilEndOfAllMoves();

  //  Disable heaters and motors
  Printer::kill(false);
}



bool
SDCard::selectFile(const char* filename, bool silent) {

  if (_sdActive == false)
    return false;

  const char* oldP = filename;

  _sdMode = SDMODE_IDLE;

  file.close();

  // Filename for progress view
  strncpy(Printer::printName, filename, 20);

  Printer::printName[20] = 0;

  if (file.open(_fat.vwd(), filename, O_READ) == false) {
    if (!silent)
      Com::printF(PSTR("file.open failed.\n"));
    return false;
  }

  if ((oldP = strrchr(filename, '/')) != NULL)
    oldP++;
  else
    oldP = filename;

  if (!silent) {
    Com::printF(PSTR("Opened '"));
    Com::print(oldP);
    Com::printF(PSTR("' of size "));
    Com::print(file.fileSize());
    Com::printF(PSTR(" bytes\n"));
  }

  sdpos = 0;
  filesize = file.fileSize();

  Com::printF(PSTR("File selected.\n"));

  return true;
}





int8_t
SDCard::readByte(void) {
  int8_t  n = file.read();

  if (n == -1) {
    file.seekSet(sdpos);

    n = file.read();

    if (n == -1) {
      Com::printF(PSTR("ERROR: SD read error at position "));
      Com::print(sdpos);
      Com::printF(PSTR("\n"));

      return(-1);
    }
  }

  sdpos++;

  return(n);
}
