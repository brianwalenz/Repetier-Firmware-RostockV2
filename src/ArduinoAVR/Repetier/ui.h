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

#include "HAL.h"


//  The display can show 4 rows of 20 columns each.
#define UI_COLS  20
#define UI_ROWS   4

//  But we allow the rows to be up to 28 letters long and
//  will scroll each row to show the missing columns.
#define MAX_COLS 28





#define ACT_NOTHING              (uint16_t)0
#define ACT_OK                   (uint16_t)1  //  Encoder button pushed.
#define ACT_KILL                 (uint16_t)2  //  Kill button pushed.

#define ACT_MENU_CHANGE          (uint16_t)3
#define ACT_SPEED_CHANGE         (uint16_t)4
#define ACT_FLOW_CHANGE          (uint16_t)5
#define ACT_EXTRUDER_FAN_CHANGE  (uint16_t)6
#define ACT_LAYER_FAN_CHANGE     (uint16_t)7
#define ACT_ABORT_PRINT          (uint16_t)8
#define ACT_EXT_T_TARGET         (uint16_t)9
#define ACT_unused_1             (uint16_t)10
#define ACT_BED_T_TARGET         (uint16_t)11
#define ACT_unused_2             (uint16_t)12
#define ACT_HOME                 (uint16_t)13
#define ACT_POS_E                (uint16_t)14
#define ACT_POS_X                (uint16_t)15
#define ACT_POS_Y                (uint16_t)16
#define ACT_POS_Z                (uint16_t)17
#define ACT_POS_Z_OPEN           (uint16_t)18
#define ACT_POS_Z_SET            (uint16_t)19
#define ACT_REL_MOTORS           (uint16_t)20

#define ACT_PID_P                (uint16_t)30
#define ACT_PID_I                (uint16_t)31
#define ACT_PID_D                (uint16_t)32

#define MODE_PRINTING            (uint16_t)1
#define MODE_PAUSED              (uint16_t)2
#define MODE_CARD_PRESENT        (uint16_t)4
#define MODE_FULL_PID            (uint16_t)8
#define MODE_DEADTIME            (uint16_t)16
#define MODE_BED_HEAT            (uint16_t)32
#define MODE_EXT_HEAT            (uint16_t)64

#define entryType_page           (uint8_t)30      //  Scroll left/right through menu pages.
#define entryType_file           (uint8_t)31      //  Placeholder to populate file list from SD card.
#define entryType_displ          (uint8_t)32      //  A display-only item.  No action possible.
#define entryType_toggle         (uint8_t)33      //  An on/off entry.
#define entryType_action         (uint8_t)34      //  A value entry.

#define menuType_normal          (uint8_t)21
#define menuType_select          (uint8_t)22

#if 1
//  OLD ACTIONS
#define UI_FLAG_FAST_KEY_ACTION     1
#define UI_FLAG_SLOW_KEY_ACTION     2
#define UI_FLAG_SLOW_ACTION_RUNNING 4
#define UI_FLAG_KEY_TEST_RUNNING    8
#endif


class menuEntry_t {
public:
  bool         visible(void)     const;

  const char  *text(void)        const  {  return((const char *)pgm_read_ptr (&_entryText));      };
  uint8_t      type(void)        const  {  return((uint8_t)     pgm_read_byte(&_entryType));      };
  uint16_t     action(void)      const  {  return((uint16_t)    pgm_read_byte(&_entryAction));    };

public:
  const char  *_entryText;
  uint8_t      _entryType;
  uint16_t     _entryAction;
  uint16_t     _doShow;        //  hide if all of these are missing
  uint16_t     _noShow;        //  hide if any of these are present
};

typedef const menuEntry_t           menuEntry;
typedef const menuEntry_t * const   menuEnArr;



class menuPage_t {
public:
  bool         visible(void)     const;

  uint8_t      type(void)        const  {  return((uint8_t)     pgm_read_byte(&_menuType));         };
  uint8_t      entriesLen(void)  const  {  return((uint8_t)     pgm_read_byte(&_menuEntriesLen));   };
  menuEntry  **entries(void)     const  {  return((menuEntry **)pgm_read_ptr (&_menuEntries));      };
  menuEntry   *entry(uint8_t ee) const  {  return((menuEntry *) pgm_read_ptr (&entries()[ee]));     };

public:
  uint8_t     _menuType;
  uint8_t     _menuEntriesLen;
  menuEnArr  *_menuEntries;
};

typedef const menuPage_t     menuPage;





class UIDisplay {
public:
  UIDisplay() {
  };

  void initialize(void);

  //  Alerts

  void  uiChirp(void);
  void  uiAlert(uint8_t n=4);

  //  Display

  void printRow (uint8_t r, char *txt);
  void printRowP(uint8_t r, PGM_P txt);

  void addChar(const char c) {
    if (rbp < MAX_COLS)
      rb[rbp++] = c;
  };

  void addStringP(PGM_P text);
  void addString(const char * text);

  void addOnOff(uint8_t on) {
    addStringP(on ? PSTR("On") : PSTR("Off"));
  };

  void addNumber(int32_t value, int8_t digits, char fillChar = ' ');
  void addFloat(float number, char fixdigits=-9, uint8_t digits=2);

  void addTimeInDaysHoursMinutes(uint32_t seconds);
  void addTimeInHoursMinutesSeconds(uint32_t seconds);

  void setStatusP(PGM_P txt, bool error = false);
  void setStatus(const char *txt, bool error = false);

  void clearStatus(void) {;
    sm[0] = 0;
  }

  void        parse(const char *txt, bool ram);

  void        adjustMenuPos(void);
  void        refreshPage();


  void        okAction_selectFile(uint8_t filePos);
  void        okAction_start(bool allowMoves);
  void        okAction_stop(bool allowMoves);
  void        okAction(bool allowMoves);

  bool        doEncoderChange_file (int16_t encoderChange);
  bool        doEncoderChange_entry(int16_t encoderChange);
  bool        doEncoderChange_page (int16_t encoderChange);
  bool        doEncoderChange(int16_t encoderChange, bool allowMoves);

  uint16_t    executeAction(uint16_t action, bool allowMoves);

  void        slowAction(bool allowMoves);
  void        fastAction();





  inline void setOutputMaskBits  (unsigned int bits)   { outputMask |=  bits; }
  inline void unsetOutputMaskBits(unsigned int bits)   { outputMask &= ~bits; }

  //  SD CARD

  void     scanSDcard(uint16_t filePos = 65535, char *filename = NULL);

  void     goDir(char *name);
  bool     isDirname(char *name);
  uint8_t  sdrefresh(char cache[UI_ROWS][MAX_COLS + 1]);



  //  1 - fast key action
  //  2 - slow key action
  //  4 - slow action running
  //  8 - key test running
#ifdef NEW_ACTIONS
  volatile uint8_t   slowKeyAction;
  volatile uint8_t   fastButtonStateChange;
  volatile uint8_t   slowActionRunning;
  volatile uint8_t   keyTestRunning;
#else
  volatile uint8_t flags; // 1 = fast key action, 2 = slow key action, 4 = slow action running, 8 = key test running
#endif

  //  What page and item are we showing?
  //
  uint8_t      _menuPage;        //  Active menu page.
  uint8_t      _menuPos;         //  Menu entry the cursor is at.
  uint8_t      _menuSel;         //  Menu entry selected for action.
  uint8_t      _menuTop;         //  First displayed element.

  //uint8_t      _displayShift;    //  Shift the display to show long rows.

  //  A few timers.
  //   - exiting a value change operation when idle.
  //   - returning to the print status page when idle.
  //
  uint32_t     _stopChangeTime;
  uint32_t     _stopMenuTime;
  uint32_t     _refreshPageTime;

  //  A buffer for constructing lines for the display.
  //
  uint8_t       rbp;                //  That's Row Buffer Position.
  char          rb[MAX_COLS + 1];   //

  //  A buffer for holding a printer status message.
  //
  uint8_t       smp;                //  That's Status Message Position.
  char          sm[MAX_COLS + 1];   //

  //  Variables for accessing the selector wheel.
  //  The first two are set in ui.cpp uiCheckKeys().
  //  The last two are set in ui-action-change.h doEncoderChange().

  int8_t        encoderLast;        //  Last state of the encoder wheel.
  int8_t        encoderPos;         //  Current number of clicks encoder wheel has turned.
  
  int8_t        grabEncoder(void) {
    uint8_t  sreg = SREG;
    int8_t   ret;

    cli();                     //  Turn off interrupts.
    ret        = encoderPos;   //  Save current encoder value.
    encoderPos = 0;            //  And reset it.
    SREG       = sreg;         //  Reenable interrupts.

    return(ret);
  }

  //  Variables for using the selector wheel.

  uint32_t      lastEncoderTime;    //  Time of the last encoder change event.
  float         encoderAccel;       //  Amount of acceleration to apply to the encoder count.

  //  Variables for using the buttons.

#ifdef NEW_ACTIONS
  uint32_t     _buttonTime;
  uint8_t      _buttonAction;
  uint8_t      _executedAction;       //  was lastAction

  //uint16_t      activeAction; // action for ok/next/previous
  //uint16_t      delayedAction;
  //uint32_t      lastRefresh;
#else
  uint16_t activeAction; // action for ok/next/previous
  uint16_t lastAction;
  uint16_t delayedAction;
  uint32_t lastSwitch; // Last time display switched pages
  uint32_t lastRefresh;
  uint16_t lastButtonAction;
  uint32_t lastButtonStart;
  uint32_t nextRepeat; // Time of next autorepeat
  int repeatDuration; // Time between to actions if autorepeat is enabled
#endif

  //int8_t        shift; // Display shift for scrolling text

  int           pageDelay; // Counter. If 0 page is refreshed if menuLevel is 0.

  void         *errorMsg;

  unsigned int  outputMask; // Output mask for back light, leds etc.


  uint8_t       encoderStartScreen;  //  timer on aborting a menu

  char          statusMsg[21];
};



extern UIDisplay uid;



#endif

