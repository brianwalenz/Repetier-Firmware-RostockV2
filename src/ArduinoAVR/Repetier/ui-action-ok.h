



void
UIDisplay::okAction_selectFile(uint8_t filePos) {
  char    filename[MAX_FILENAME_LEN + 1 + 1];   //  Needs one extra byte for an appended '/', and nul terminator.

  uiChirp();

  //  Grab the name of the file at postion filePos.

  uint8_t  isDir = sd.scanCard(filePos, filename);

  Com::printf(PSTR("selectFile()-- found file %d '%s'\n"), filePos, filename);

  //  If a directory was selected, go into it.

  if (isDir) {
    sd.goDir(filename);

    _menuPos = 2;      //  Leave on the first file; UIDisplay::sdrefresh()
    _menuSel = 255;    //  will reset if the directory is empty.
    _menuTop = 0;

    refreshPage();

    return;
  }

  //  Otherwise, a file.  Print it and reset the display if it starts.

  if (sd.analyzeFile(filename) == true) {
    commandQueue.startPrint();

    _menuPage = 1;
    _menuPos  = 0;
    _menuSel  = 255;
    _menuTop  = 0;

    refreshPage();
  }

  else {
    uiAlert();
  }
}



void
UIDisplay::okAction_start(bool allowMoves) {
  menuPage     *menu        = menuPagePtr(_menuPage);
  uint8_t       menuType    = menu->type();
  uint8_t       entryType   = menu->entry(_menuPos)->type();
  uint16_t      entryAction = menu->entry(_menuPos)->action();

  //  For all menus, if we're on a page title, set that as the active item.
  //  Then, the wheel will scroll through available menus.
  //
  if (entryType == entryType_page) {
    _menuSel = _menuPos;
  }

  //  In a file selection menu, but selected the first item, which is always
  //  'up directory', even if there is no up directory.
  else if ((menuType == menuType_select) && (_menuPos == 1)) {
    sd.goDir(NULL);

    _menuPos = 1;     //  Leave on the up-directory entry.
    _menuSel = 255;
    _menuTop = 0;

    refreshPage();
  }

  //  In a menuSelect, and selected a file or directory.
  else if (menuType == menuType_select) {
    okAction_selectFile(_menuPos - 2);
  }

  //  On a display item.  Do nothing.  Maybe chirp.  But just do nothing.
  else if (entryType == entryType_displ) {
  }

  //  On a toggle item.  Toggle!
  else if (entryType == entryType_toggle) {
    if      (entryAction == ACT_ABORT_PRINT) {
      commandQueue.stopPrint();
    }

    else if (entryAction == ACT_HOME) {
      Printer::homeTowers();
      Commands::printCurrentPosition();
    }

    else if (entryAction == ACT_POS_Z_SET) {
      Printer::setOrigin(-Printer::currentPosition[X_AXIS],
                         -Printer::currentPosition[Y_AXIS],
                         -Printer::currentPosition[Z_AXIS]);
      Commands::printCurrentPosition();
    }

    else if (entryAction == ACT_REL_MOTORS) {
      Printer::disableSteppers();
    }
  }

  //  If we're on an action item, this is actually the easy case, just set the
  //  active item.  All the rest is done in doEncoderChange().

  else if (entryType == entryType_action) {
    _menuSel = _menuPos;
  }
}



void
UIDisplay::okAction_stop(bool allowMoves) {
  menuPage     *menu        = menuPagePtr(_menuPage);
  uint8_t       menuType    = menu->type();
  uint8_t       entryType   = menu->entry(_menuPos)->type();
  uint16_t      entryAction = menu->entry(_menuPos)->action();

  //  A page selection.  Nothing to do, since we're already on the page to show.
  if ((entryType   == entryType_page) ||
      (entryAction == ACT_MENU_CHANGE)) {
  }

  //  finish up setting parameter ...
  else if (entryAction == 99) {
  }

  //  finish up setting parameter ...
  else if (entryAction == 98) {
  }

  //  Now just turn off the selection.
  _menuSel = 255;
}



//  This is a button press.
void
UIDisplay::okAction(bool allowMoves) {

  uiChirp();

  if (_menuSel == 255)
    okAction_start(allowMoves);   //  If nothing active, start some action.
  else
    okAction_stop(allowMoves);    //  If something currently active, terminate the action.

  refreshPage();
}
