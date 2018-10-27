








void
UIDisplay::okAction_selectFile(uint8_t filePos) {
  char    filename[LONG_FILENAME_LENGTH + 2];   //  Needs one extra byte for an appended '/', and nul terminator.

  //  Grab the name of the file at postion filePos.

  scanSDcard(filePos, filename);

  //  If a directory was selected, go into it.

  if (isDirname(filename)) {
    goDir(filename);


    _menuPos = 2;
    _menuSel = 255;
    _menuTop = 0;

    refreshPage();
 }

  //  Otherwise, a file.  If we can select it, start printing
  //  and reset the display.

  else if (sd.selectFile(filename, false) == true) {
    sd.startPrint();

    uiAlert();

    _menuPage = 0;
    _menuPos  = 0;
    _menuSel  = 255;
    _menuTop  = 0;

    refreshPage();
  }

  //  the original seems to do these here - they seem extraneous
  //sd.file.close();
  //sd.fat.chdir(cwd);
}





//  DELETE A FILE
//
//  if(sd.sdactive) {
//    sd.sdmode = 0;
//    sd.file.close();
//    if(sd.fat.remove(filename)) {
//      Com::printFLN(PSTR("File deleted"));
//      uiAlert();
//      if(_menuPos > 0)
//        _menuPos--;
//      updateSDFileCount();
//    } else {
//      Com::printFLN(PSTR("Deletion failed"));
//    }
//  }




void
UIDisplay::okAction_start(bool allowMoves) {
  menuPage     *menu        = (menuPage   *)pgm_read_ptr (&menuPages[_menuPage]);
  uint8_t       menuType    =               pgm_read_byte(&menu->menuType);
  menuEntry   **entries     = (menuEntry **)pgm_read_ptr (&menu->menuEntries);
  uint8_t       entriesLen  =               pgm_read_byte(&menu->menuEntriesLen);
  menuEntry    *entry       = (menuEntry  *)pgm_read_ptr (&entries[_menuPos]);
  uint8_t       entryType   =               pgm_read_byte(&entry->entryType);
  uint8_t       entryAction =               pgm_read_word(&entry->entryAction);

#if 0
  Com::print("okAction_start _menuPos=");
  Com::print(_menuPos);
  Com::print(" menuType=");
  Com::print(menuType);
  Com::print(" entryType=");
  Com::print(entryType);
  Com::print(" entryAction=");
  Com::print(entryAction);
  Com::print("\n");
#endif

  //  For all menus, if we're on a page title, set that as the active item.
  //  Then, the wheel will scroll through available menus.
  //
  if (entryType == entryType_page) {
    _menuSel = 0;
  }

  //  In a menuSelect, but selected the menu itself.  Cancel!  Start changing menus.
  else if ((menuType == menuType_fileSelect) && (entryType == entryType_page)) {
    _menuSel = 0;  //  actually handled above....
  }

  //  In a fileSelect, but selected the first item, which is always 'up directory',
  //  even if there is no up directory.
  else if ((menuType == menuType_fileSelect) && (_menuPos == 1)) {
    goDir(NULL);

    _menuPos = 1;
    _menuSel = 255;
    _menuTop = 0;

    refreshPage();
  }

  //  In a menuSelect, and selected a file or directory.
  else if (menuType == menuType_fileSelect) {
    okAction_selectFile(_menuPos - 2);
  }

  //  On a display item.  Do nothing.  Maybe chirp.  But just do nothing.
  else if (entryType == entryType_displ) {
  }

  //  On a toggle item.  Toggle!
  else if (entryType == entryType_toggle) {
    if      (entryAction == 0) {
    }

    else if (entryAction == 1) {
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
  menuPage     *menu        = (menuPage   *)pgm_read_ptr (&menuPages[_menuPage]);
  //uint8_t       menuType    =               pgm_read_byte(&menu->menuType);
  menuEntry   **entries     = (menuEntry **)pgm_read_ptr (&menu->menuEntries);
  //uint8_t       entriesLen  =               pgm_read_byte(&menu->menuEntriesLen);
  menuEntry    *entry       = (menuEntry  *)pgm_read_ptr (&entries[_menuPos]);
  uint8_t       entryType   =               pgm_read_byte(&entry->entryType);
  uint8_t       entryAction =               pgm_read_word(&entry->entryAction);

#if 0
  Com::print("okAction_stop _menuSel=");
  Com::print(_menuSel);
  Com::print(" entryAction=");
  Com::print(entryAction);
  Com::print("\n");
#endif


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

  if (Printer::isUIErrorMessage()) {
    Printer::setUIErrorMessage(false);
    // return 0;
  }

  uiChirp();

#if 0
  Com::print("okAction _menuSel=");
  Com::print(_menuSel);
  Com::print("\n");
#endif

  if (_menuSel == 255)
    okAction_start(allowMoves);   //  If nothing active, start some action.
  else
    okAction_stop(allowMoves);    //  If something currently active, terminate the action.

  refreshPage();
}



#if 0

  if (entType == UI_MENU_TYPE_SUBMENU) {
    pushMenu((UIMenu*)action, false);

    currHeaterForSetup = &(Extruder::current->tempControl);

    Printer::setMenuMode(MODE_FULL_PID, currHeaterForSetup->heatManager == 1);
    Printer::setMenuMode(MODE_DEADTIME, currHeaterForSetup->heatManager == 3);

    return(0);
  }

  if (entType == UI_MENU_TYPE_MODIFICATION_MENU) {
    return executeAction(action, allowMoves);
  }

#endif
