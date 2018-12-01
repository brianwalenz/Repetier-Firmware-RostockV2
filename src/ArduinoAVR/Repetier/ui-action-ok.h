

void
UIDisplay::okAction_selectFile(uint8_t filePos) {
  char    filename[MAX_FILENAME_LEN + 1 + 1];   //  Needs one extra byte for an appended '/', and nul terminator.

  //  Grab the name of the file at postion filePos.

  scanSDcard(filePos, filename);

  //  If a directory was selected, go into it.

  if (isDirname(filename)) {
    goDir(filename);

    _menuPos = 2;      //  Leave on the first file; UIDisplay::sdrefresh()
    _menuSel = 255;    //  will reset if the directory is empty.
    _menuTop = 0;

    refreshPage();
 }

  //  Otherwise, a file.  If we can select it, start printing
  //  and reset the display.

  else if (sd.selectFile(filename, false) == true) {
    sd.startPrint();

    uiAlert();

    _menuPage = 1;
    _menuPos  = 0;
    _menuSel  = 255;
    _menuTop  = 0;

    refreshPage();
  }

  //  the original seems to do these here - they seem extraneous
  //sd.file.close();
  //sd.chdir(cwd);
}



void
UIDisplay::okAction_start(bool allowMoves) {
  menuPage     *menu        = menuPagePtr(_menuPage);
  uint8_t       menuType    = menu->type();
  uint8_t       entryType   = menu->entry(_menuPos)->type();
  uint16_t      entryAction = menu->entry(_menuPos)->action();

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
    _menuSel = _menuPos;
  }

  //  In a file selection menu, but selected the first item, which is always
  //  'up directory', even if there is no up directory.
  else if ((menuType == menuType_select) && (_menuPos == 1)) {
    goDir(NULL);

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
      sd.stopPrint();
      //Printer::stopPrint();       //  What's the difference??
      //Printer::continuePrint();
    }

    else if (entryAction == ACT_HOME) {
      Printer::homeAxis(true, true, true);
      Commands::printCurrentPosition();
    }

    else if (entryAction == ACT_POS_Z_SET) {
      Printer::setOrigin(-Printer::currentPosition[X_AXIS],
                         -Printer::currentPosition[Y_AXIS],
                         -Printer::currentPosition[Z_AXIS]);
      Commands::printCurrentPosition();
    }

    else if (entryAction == ACT_REL_MOTORS) {
      Printer::kill(true);
    }

#if 0
    case UI_ACTION_STORE_EEPROM:
      EEPROM::storeDataIntoEEPROM(false);
      //pushMenu(&ui_menu_eeprom_saved, false);
      uiAlert();
      break;
    case UI_ACTION_LOAD_EEPROM:
      EEPROM::readDataFromEEPROM(true);
      Extruder::selectExtruderById(Extruder::current->id);
      //pushMenu(&ui_menu_eeprom_loaded, false);
      uiAlert();
      break;
#endif
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


#if 0
  //  former finishAction
  if (action == ACT_EXT_T_PREHEAT) {
    int i = 0;
    int o = i * EEPROM_EXTRUDER_LENGTH + EEPROM_EXTRUDER_OFFSET;

    Extruder *e = &extruder[i];

    HAL::eprSetInt16(o + EPR_EXTRUDER_PREHEAT, e->tempControl.preheatTemperature);
    EEPROM::updateChecksum();
  }

  if (action == ACT_BED_T_PREHEAT) {
    HAL::eprSetInt16(EPR_BED_PREHEAT_TEMP, heatedBedController.preheatTemperature);
    EEPROM::updateChecksum();
  }
#endif
