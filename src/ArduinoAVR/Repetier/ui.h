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

#ifndef _ui_h
#define _ui_h


#define UI_DISPLAY_CHARSET 1
#define UI_COLS 20
#define UI_ROWS 4




// ----------------------------------------------------------------------------
//                          Action codes
// 1-999     : Autorepeat
// 1000-1999 : Execute
// 2000-2999 : Write code
// 4000-4999 : Show menu
//
// Add UI_ACTION_TOPMENU to show a menu as top menu
// Add UI_ACTION_NO_AUTORETURN to prevent auto return to start display
// ----------------------------------------------------------------------------

#define UI_ACTION_TOPMENU 8192
#define UI_ACTION_NO_AUTORETURN 16384

#define UI_ACTION_NEXT 1
#define UI_ACTION_PREVIOUS 2

#define UI_ACTION_X_UP                 100
#define UI_ACTION_X_DOWN               101
#define UI_ACTION_Y_UP                 102
#define UI_ACTION_Y_DOWN               103
#define UI_ACTION_Z_UP                 104
#define UI_ACTION_Z_DOWN               105
#define UI_ACTION_EXTRUDER_UP          106
#define UI_ACTION_EXTRUDER_DOWN        107
#define UI_ACTION_EXTRUDER_TEMP_UP     108
#define UI_ACTION_EXTRUDER_TEMP_DOWN   109
#define UI_ACTION_HEATED_BED_UP        110
#define UI_ACTION_HEATED_BED_DOWN      111
#define UI_ACTION_FAN_UP               112
#define UI_ACTION_FAN_DOWN             113
// 700-999 reserved for custom events.


#define UI_ACTION_DUMMY 10000
#define UI_ACTION_BACK                  1000
#define UI_ACTION_OK                    1001
#define UI_ACTION_MENU_UP               1002
#define UI_ACTION_TOP_MENU              1003
#define UI_ACTION_EMERGENCY_STOP        1004
#define UI_ACTION_XPOSITION             1005
#define UI_ACTION_YPOSITION             1006
#define UI_ACTION_ZPOSITION             1007
#define UI_ACTION_EPOSITION             1008
#define UI_ACTION_BED_TEMP              1009
#define UI_ACTION_EXTRUDER_TEMP         1010
#define UI_ACTION_SD_DELETE             1012
#define UI_ACTION_SD_PRINT              1013
#define UI_ACTION_SD_PAUSE              1014
#define UI_ACTION_SD_CONTINUE           1015
#define UI_ACTION_SD_UNMOUNT            1016
#define UI_ACTION_SD_MOUNT              1017
#define UI_ACTION_HOME_ALL              1021
#define UI_ACTION_STORE_EEPROM          1030
#define UI_ACTION_LOAD_EEPROM           1031
#define UI_ACTION_PRINT_ACCEL_X         1032
#define UI_ACTION_PRINT_ACCEL_Y         1033
#define UI_ACTION_PRINT_ACCEL_Z         1034
#define UI_ACTION_MOVE_ACCEL_X          1035
#define UI_ACTION_MOVE_ACCEL_Y          1036
#define UI_ACTION_MOVE_ACCEL_Z          1037
#define UI_ACTION_MAX_JERK              1038
#define UI_ACTION_MAX_ZJERK             1039
#define UI_ACTION_BAUDRATE              1040
#define UI_ACTION_STEPS_X               1047
#define UI_ACTION_STEPS_Y               1048
#define UI_ACTION_STEPS_Z               1049
#define UI_ACTION_FAN_OFF               1050
#define UI_ACTION_FAN_25                1051
#define UI_ACTION_FAN_50                1052
#define UI_ACTION_FAN_75                1053
#define UI_ACTION_FAN_FULL              1054
#define UI_ACTION_FEEDRATE_MULTIPLY     1055
#define UI_ACTION_STEPPER_INACTIVE      1056

#define UI_ACTION_PID_PGAIN             1058
#define UI_ACTION_PID_IGAIN             1059
#define UI_ACTION_PID_DGAIN             1060
#define UI_ACTION_DRIVE_MIN             1061
#define UI_ACTION_DRIVE_MAX             1062
#define UI_ACTION_X_OFFSET              1063
#define UI_ACTION_Y_OFFSET              1064
#define UI_ACTION_EXTR_STEPS            1065
#define UI_ACTION_EXTR_ACCELERATION     1066
#define UI_ACTION_EXTR_MAX_FEEDRATE     1067
#define UI_ACTION_EXTR_START_FEEDRATE   1068
#define UI_ACTION_EXTR_HEATMANAGER      1069
#define UI_ACTION_EXTR_WATCH_PERIOD     1070
#define UI_ACTION_PID_MAX               1071
#define UI_ACTION_ADVANCE_K             1072
#define UI_ACTION_SET_ORIGIN            1073

#define UI_ACTION_POWER                 1078
#define UI_ACTION_PREHEAT_SINGLE        1079
#define UI_ACTION_COOLDOWN              1080
#define UI_ACTION_HEATED_BED_OFF        1081
#define UI_ACTION_EXTRUDER0_OFF         1082
#define UI_ACTION_EXTRUDER1_OFF         1083
#define UI_ACTION_EXTRUDER2_OFF         1084
#define UI_ACTION_EXTRUDER3_OFF         1085
#define UI_ACTION_EXTRUDER4_OFF         1086
#define UI_ACTION_EXTRUDER5_OFF         1087
#define UI_ACTION_OPS_OFF               1088
#define UI_ACTION_OPS_CLASSIC           1089
#define UI_ACTION_OPS_FAST              1090
#define UI_ACTION_DISABLE_STEPPER       1091
#define UI_ACTION_RESET_EXTRUDER        1092
#define UI_ACTION_EXTRUDER_RELATIVE     1093
#define UI_ACTION_ADVANCE_L             1094
#define UI_ACTION_PREHEAT_ALL           1095
#define UI_ACTION_FLOWRATE_MULTIPLY     1096
#define UI_ACTION_KILL                  1097
#define UI_ACTION_RESET                 1098
#define UI_ACTION_PAUSE                 1099
#define UI_ACTION_EXTR_WAIT_RETRACT_TEMP 1100
#define UI_ACTION_EXTR_WAIT_RETRACT_UNITS 1101
#define UI_ACTION_WRITE_DEBUG           1105
#define UI_ACTION_FANSPEED              1106
#define UI_ACTION_LIGHTS_ONOFF          1107
#define UI_ACTION_SD_STOP               1108
#define UI_ACTION_ZPOSITION_NOTEST      1109
#define UI_ACTION_ZPOSITION_FAST_NOTEST 1110
#define UI_ACTION_Z_BABYSTEPS           1111
#define UI_ACTION_MAX_INACTIVE          1112
#define UI_ACTION_TEMP_DEFECT           1113
#define UI_ACTION_BED_HEATMANAGER       1114
#define UI_ACTION_BED_PGAIN             1115
#define UI_ACTION_BED_IGAIN             1116
#define UI_ACTION_BED_DGAIN             1117
#define UI_ACTION_BED_DRIVE_MIN         1118
#define UI_ACTION_BED_DRIVE_MAX         1119
#define UI_ACTION_BED_MAX               1120
#define UI_ACTION_HEATED_BED_TEMP       1121
#define UI_ACTION_EXTRUDER0_TEMP        1122
#define UI_ACTION_EXTRUDER1_TEMP        1123
#define UI_ACTION_EXTRUDER2_TEMP        1124
#define UI_ACTION_EXTRUDER3_TEMP        1125
#define UI_ACTION_EXTRUDER4_TEMP        1126
#define UI_ACTION_EXTRUDER5_TEMP        1127
#define UI_ACTION_SELECT_EXTRUDER0      1128
#define UI_ACTION_SELECT_EXTRUDER1      1129
#define UI_ACTION_SELECT_EXTRUDER2      1130
#define UI_ACTION_SELECT_EXTRUDER3      1131
#define UI_ACTION_SELECT_EXTRUDER4      1132
#define UI_ACTION_SELECT_EXTRUDER5      1133

#define UI_ACTION_DEBUG_ECHO            1150
#define UI_ACTION_DEBUG_INFO            1151
#define UI_ACTION_DEBUG_ERROR           1152
#define UI_ACTION_DEBUG_DRYRUN          1153
#define UI_ACTION_DEBUG_ENDSTOP         1154

#define UI_ACTION_SD_PRI_PAU_CONT       1200
#define UI_ACTION_FAN_SUSPEND           1201
#define UI_ACTION_AUTOLEVEL_ONOFF       1202
#define UI_ACTION_IGNORE_M106           1204

#define UI_ACTION_KAPTON                1205
#define UI_ACTION_BLUETAPE              1206
#define UI_ACTION_NOCOATING             1207
#define UI_ACTION_PETTAPE               1208
#define UI_ACTION_GLUESTICK             1209
#define UI_ACTION_RESET_MATRIX          1210
#define UI_ACTION_CALIBRATE             1211
#define UI_ACTION_BED_LED_CHANGE        1212
#define UI_ACTION_COATING_CUSTOM        1213
#define UI_ACTION_BUILDTAK              1214

#define UI_ACTION_CONTINUE              1220
#define UI_ACTION_STOP                  1221
#define UI_ACTION_STOP_CONFIRMED        1222
#define UI_ACTION_FAN2SPEED             1223
#define UI_ACTION_AUTOLEVEL             1224
#define UI_ACTION_MEASURE_DISTORTION    1225
#define UI_ACTION_TOGGLE_DISTORTION     1226
#define UI_ACTION_MESSAGE               1227
#define UI_ACTION_STATE                 1228
#define UI_ACTION_AUTOLEVEL2            1229
#define UI_ACTION_MEASURE_DISTORTION2   1230
//#define UI_ACTION_BED_PREHEAT           1231
#define UI_ACTION_EXT0_PREHEAT          1232
#define UI_ACTION_MEASURE_ZPROBE_HEIGHT	1238
#define UI_ACTION_MEASURE_ZPROBE_HEIGHT2 1239
#define UI_ACTION_MEASURE_ZP_REALZ      1240
#define UI_ACTION_Z_OFFSET              1241

#define UI_ACTION_BED_TARGET            1250   //  Set target temperature for bed
#define UI_ACTION_BED_PREHEAT           1251   //  Set preheat temperature for bed
#define UI_ACTION_BED_PREHEAT_ON        1252   //  Turn on preheat for bed
#define UI_ACTION_BED_PREHEAT_OFF       1253   //  Turn off preheat for bed

#define UI_ACTION_EXT_TARGET            1260
#define UI_ACTION_EXT_PREHEAT           1261
#define UI_ACTION_EXT_PREHEAT_ON        1262
#define UI_ACTION_EXT_PREHEAT_OFF       1263

// 1500-1699 reserved for custom actions

#define UI_ACTION_MENU_XPOS             4000
#define UI_ACTION_MENU_YPOS             4001
#define UI_ACTION_MENU_ZPOS             4002
#define UI_ACTION_MENU_XPOSFAST         4003
#define UI_ACTION_MENU_YPOSFAST         4004
#define UI_ACTION_MENU_ZPOSFAST         4005
#define UI_ACTION_MENU_SDCARD           4006
#define UI_ACTION_MENU_QUICKSETTINGS    4007
#define UI_ACTION_MENU_EXTRUDER         4008
#define UI_ACTION_MENU_POSITIONS        4009
//#define UI_ACTION_SHOW_MEASUREMENT		4010
//#define UI_ACTION_RESET_MEASUREMENT		4011
#define UI_ACTION_SET_MEASURED_ORIGIN	4012
#define UI_ACTION_SET_P1				4013
#define UI_ACTION_SET_P2				4014
#define UI_ACTION_SET_P3				4015
#define UI_ACTION_CALC_LEVEL			4016
#define UI_ACTION_XOFF                  4020
#define UI_ACTION_YOFF                  4021
#define UI_ACTION_ZOFF                  4022

#define UI_ACTION_SHOW_USERMENU1        4101
#define UI_ACTION_SHOW_USERMENU2        4102
#define UI_ACTION_SHOW_USERMENU3        4103
#define UI_ACTION_SHOW_USERMENU4        4104
#define UI_ACTION_SHOW_USERMENU5        4105
#define UI_ACTION_SHOW_USERMENU6        4106
#define UI_ACTION_SHOW_USERMENU7        4107
#define UI_ACTION_SHOW_USERMENU8        4108
#define UI_ACTION_SHOW_USERMENU9        4109
#define UI_ACTION_SHOW_USERMENU10       4110





//  For all, +128 == sticky, no autoreturn to main menu after timeout.
#define UI_MENU_TYPE_INFO              0
#define UI_MENU_TYPE_FILE_SELECTOR     1
#define UI_MENU_TYPE_SUBMENU           2
#define UI_MENU_TYPE_MODIFICATION_MENU 3
#define UI_MENU_TYPE_CHANGEACTION      4

struct UIMenuEntry_t {
  const char   *entryText;   // Menu text
  uint8_t       entryType;   // Menu type, as above
  uint16_t      entryAction; // 
  uint16_t      filter;      // allows dynamic menu filtering based on Printer::menuMode bits set.
  uint16_t      nofilter;    // Hide if one of these bits are set

  bool          showEntry(void) const;
};
typedef const UIMenuEntry_t UIMenuEntry;

struct UIMenu_t {
  uint8_t   menuType;
  int16_t   menuAction;
  uint8_t   numEntries;

  const UIMenuEntry * const * entries;
};
typedef const UIMenu_t UIMenu;




#define UI_STRING(name,text) const char PROGMEM name[] = text

#define UI_PAGE4(name,row1,row2,row3,row4)                              \
  UI_STRING(name ## _1txt,row1);                                        \
  UI_STRING(name ## _2txt,row2);                                        \
  UI_STRING(name ## _3txt,row3);                                        \
  UI_STRING(name ## _4txt,row4);                                        \
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};              \
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};              \
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0};              \
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0};              \
  const UIMenuEntry * const name ## _entries [] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4}; \
  const UIMenu name PROGMEM = {UI_MENU_TYPE_INFO, 0, 4, name ## _entries};

#define UI_MENU_ACTION4(name,action,row1,row2,row3,row4)                \
  UI_STRING(name ## _1txt,row1);                                        \
  UI_STRING(name ## _2txt,row2);                                        \
  UI_STRING(name ## _3txt,row3);                                        \
  UI_STRING(name ## _4txt,row4);                                        \
  UIMenuEntry name ## _1 PROGMEM ={name ## _1txt,0,0,0,0};              \
  UIMenuEntry name ## _2 PROGMEM ={name ## _2txt,0,0,0,0};              \
  UIMenuEntry name ## _3 PROGMEM ={name ## _3txt,0,0,0,0};              \
  UIMenuEntry name ## _4 PROGMEM ={name ## _4txt,0,0,0,0};              \
  const UIMenuEntry * const name ## _entries[] PROGMEM = {&name ## _1,&name ## _2,&name ## _3,&name ## _4}; \
  const UIMenu name PROGMEM = {UI_MENU_TYPE_MODIFICATION_MENU, action, 4, name ## _entries};

#define UI_MENU_HEADLINE(name,text)                                      UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 1, 0,      0,        0};
#define UI_MENU_HEADLINE_FILTER(name,text,filter,nofilter)               UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 1, 0, filter, nofilter};

#define UI_MENU_CHANGEACTION(name,text,action)                           UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 4, action,      0,        0};
#define UI_MENU_CHANGEACTION_FILTER(name,text,action,filter,nofilter)    UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 4, action, filter, nofilter};

#define UI_MENU_ACTIONCOMMAND(name,text,action)                          UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 3, action,      0,        0};
#define UI_MENU_ACTIONCOMMAND_FILTER(name,text,action,filter,nofilter)   UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 3, action, filter, nofilter};

#define UI_MENU_SUBMENU(name,text,entries)                               UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 2, (uint16_t)&entries,      0,        0};
#define UI_MENU_SUBMENU_FILTER(name,text,entries,filter,nofilter)        UI_STRING(name ## _txt,text);  UIMenuEntry name PROGMEM = {name ## _txt, 2, (uint16_t)&entries, filter, nofilter};

#define UI_MENU(name,items,itemsCnt)            const UIMenuEntry * const name ## _entries[] PROGMEM = items;  const UIMenu name PROGMEM = {2,     0, itemsCnt, name ## _entries};
#define UI_STICKYMENU(name,items,itemsCnt)      const UIMenuEntry * const name ## _entries[] PROGMEM = items;  const UIMenu name PROGMEM = {2+128, 0, itemsCnt, name ## _entries};
#define UI_MENU_FILESELECT(name,items,itemsCnt) const UIMenuEntry * const name ## _entries[] PROGMEM = items;  const UIMenu name PROGMEM = {1,     0, itemsCnt, name ## _entries};



#define MAX_COLS                   28   // Maximum size of a row - if row is larger, text gets scrolled

#define UI_MENU_MAXLEVEL            7

#define UI_FLAG_FAST_KEY_ACTION     1
#define UI_FLAG_SLOW_KEY_ACTION     2
#define UI_FLAG_SLOW_ACTION_RUNNING 4
#define UI_FLAG_KEY_TEST_RUNNING    8

class GCode;

class UIDisplay {
public:
  UIDisplay() {
  };

  void initialize(void);


  void addNumber(int32_t value, int8_t digits, char fillChar = ' ');

  void addFloat(float number, char fixdigits=-9, uint8_t digits=2);

  void addStringP(PGM_P text);
  void addString(const char * text);

  void addStringOnOff(uint8_t on) {
    addStringP(on ? PSTR("On") : PSTR("Off"));
  };

  void addStringYesNo(uint8_t yes) {
    addStringP(yes ? PSTR("Yes") : PSTR("No"));
  };

  void addChar(const char c) {
    if (col < UI_COLS)
      printCols[col++] = c;
  };



  int okAction(bool allowMoves);

  bool doEncoderChange(int16_t next, bool allowMoves);

#if 0
  void waitForKey(void) {
    uint16_t nextAction = 0;

    lastButtonAction = 0;

    while (lastButtonAction == nextAction)
      uiCheckKeys(nextAction);
  };
#endif

  void printRow (uint8_t r, char *txt);
  void printRowP(uint8_t r, PGM_P txt);

  void parse(const char *txt, bool ram); /// Parse output and write to printCols;
  void refreshPage();
  int  executeAction(unsigned int action, bool allowMoves);
  void finishAction(unsigned int action);
  void slowAction(bool allowMoves);
  void fastAction();
  void mediumAction();
  void pushMenu(const UIMenu *men, bool refresh);
  void popMenu(bool refresh);

  void adjustMenuPos(void);

  void setStatusP(PGM_P txt, bool error = false);
  void setStatus(const char *txt, bool error = false);
  void clearStatus(void);

  inline void setOutputMaskBits(unsigned int bits) {
    outputMask |= bits;
  }

  inline void unsetOutputMaskBits(unsigned int bits) {
    outputMask &= ~bits;
  }

  //  SD CARD

  void     updateSDFileCount();
  void     goDir(char *name);
  bool     isDirname(char *name);
  uint8_t  sdrefresh(char cache[UI_ROWS][MAX_COLS + 1]);


  bool isSticky(void) {
    UIMenu *men = (UIMenu*)menu[menuLevel];
    uint8_t mt = pgm_read_byte(&(men->menuType));
    return ((mt & 128) == 128) || mt == 5;
  }




  volatile uint8_t flags; // 1 = fast key action, 2 = slow key action, 4 = slow action running, 8 = key test running

  uint8_t       col; // current col for buffer pre fill


  uint8_t       menuLevel; // current menu level, 0 = info, 1 = group, 2 = groupdata select, 3 = value change

  uint16_t      menuPos[UI_MENU_MAXLEVEL]; // Positions in menu
  uint16_t      menuTop[UI_MENU_MAXLEVEL]; // Top row in menu
  const UIMenu *menu   [UI_MENU_MAXLEVEL]; // Menus active


  int8_t        shift; // Display shift for scrolling text
  int           pageDelay; // Counter. If 0 page is refreshed if menuLevel is 0.
  void         *errorMsg;
  uint16_t      activeAction; // action for ok/next/previous
  uint16_t      lastAction;
  uint16_t      delayedAction;
  millis_t      lastRefresh;
  uint16_t      lastButtonAction;
  millis_t      lastButtonStart;
  millis_t      nextRepeat; // Time of next autorepeat
  unsigned int  outputMask; // Output mask for back light, leds etc.
  int           repeatDuration; // Time between to actions if autorepeat is enabled

  millis_t      lastEncoderTime;     //  Time of the last encoder change event.
  float         encoderAccel;

  uint8_t       encoderStartScreen;  //  timer on aborting a menu

  char          printCols[MAX_COLS + 1];


  char statusMsg[21];
  int8_t encoderPos;
  int8_t encoderLast;


  char    cwd[SD_MAX_FOLDER_DEPTH * LONG_FILENAME_LENGTH + 2];
  uint16_t nFilesOnCard;
};



extern UIDisplay uid;



#endif

