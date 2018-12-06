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

#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
#include <ctype.h>

#include "Repetier.h"
#include "HAL.h"

#include "Communication.h"
#include "Commands.h"

#include "motion.h"
#include "temperatures.h"

#include "Extruder.h"

#include "uimenu.h"       //  Menu definitions.


//
//  RAMBO controller
//

#define UI_DISPLAY_RS_PIN      70
#define UI_DISPLAY_RW_PIN      -1
#define UI_DISPLAY_ENABLE_PIN  71
#define UI_DISPLAY_D0_PIN      -1
#define UI_DISPLAY_D1_PIN      -1
#define UI_DISPLAY_D2_PIN      -1
#define UI_DISPLAY_D3_PIN      -1
#define UI_DISPLAY_D4_PIN      72
#define UI_DISPLAY_D5_PIN      73
#define UI_DISPLAY_D6_PIN      74
#define UI_DISPLAY_D7_PIN      75
#define UI_ENCODER_A           76
#define UI_ENCODER_B           77
#define UI_ENCODER_CLICK       78
#define UI_KILL_PIN            80
#define UI_DELAYPERCHAR       50


#define UI_INVERT_MENU_DIRECTION 0

//Symbolic character values for specific symbols.

#define cUP     "\001"
#define cDEG    "\002"
#define cSEL    "\003"
#define cUNSEL  "\004"
#define cTEMP   "\005"
#define cFOLD   "\006"
#define cARROW  "\176"
#define cARROWc  0x3e

#define bSEL    3
#define bUNSEL  4
#define bFOLD   6

#define CHAR_SELECTOR 0xa5  //'>'
#define CHAR_SELECTED '*'

//UI_STRING(ui_selected,   cSEL);
//UI_STRING(ui_unselected, cUNSEL);

UIDisplay uid;



//  This should be inlined in ui.h, but it depends on Printer:: which isn't known until late in Repetier.h

bool
menuEntry_t::visible(void) const {
  uint16_t  ft = pgm_read_word(&_doShow);
  uint16_t  nf = pgm_read_word(&_noShow);

  if ((ft != 0) &&
      ((ft & Printer::menuMode) == 0))  //  Do not show if all of the 'do-show'
    return(false);                      //  filter bits are missing in the mode menu.

  if ((nf & Printer::menuMode) != 0)    //  Do not show if any of the 'no-show'
    return(false);                      //  filter bits are set in the menu mode.

  return(true);                         //  Default to showing.
}


//  Scan the entries in this menu.  If all are disabled, don't show
//  the menu (by skipping over it in doEncoderChange_page().
bool
menuPage_t::visible(void) const {

  for (uint8_t ee=0; ee<entriesLen(); ee++) {
    if (entry(ee)->visible() == true)
      return(true);
  }

  return(false);
}






uint16_t
uiCheckKeys(void) {

  //  Shift the encoder one position.

  uid.encoderLast <<= 2;
  uid.encoderLast  &= 0x0f;

  //  And add in the new value.

  if (READ(UI_ENCODER_A) == 0)    //  Active low.
    uid.encoderLast |= 2;

  if (READ(UI_ENCODER_B) == 0)    //  Active low.
    uid.encoderLast |= 1;

  //  To change direction of the encoder, swap increment (+=) and decrement (-=).
  //
  //  To change the speed of the encoder, test against different values.
  //     FAST =  1,  7,  8, 14   MEDIUM =  7,  8   SLOW = 14
  //     FAST =  2,  4, 11, 13   MEDIUM =  2, 13   SLOW = 11

  if ((uid.encoderLast ==  7) || (uid.encoderLast ==  8))
    uid.encoderPos += 1;
  if ((uid.encoderLast ==  2) || (uid.encoderLast == 13))
    uid.encoderPos -= 1;

  //  Check for the two buttons, one on the encoder, and the estop/reset.

  if (READ(UI_ENCODER_CLICK) == 0)    //  If ENCODER_CLICK is pushed (active low)
    return(ACT_OK);                   //  make the action be "OK".

  if (READ(UI_KILL_PIN) == 0)         //  If KILL is pushed (active low)
    return(ACT_KILL);                 //  kill ourself.

  return(0);
}



void
UIDisplay::uiChirp(void) {

  SET_OUTPUT(BEEPER_PIN);

  WRITE(BEEPER_PIN, HIGH);   delay(1);
  WRITE(BEEPER_PIN, LOW);    delay(1);
}



void
UIDisplay::uiAlert(uint8_t n) {

  SET_OUTPUT(BEEPER_PIN);

  for (uint8_t i=0; i<n; i++) {
    WRITE(BEEPER_PIN, HIGH);   delay(50);
    WRITE(BEEPER_PIN, LOW);    delay(100);
  }
}




#define lcdPutChar(value) lcdWriteByte(value,1)
#define lcdCommand(value) lcdWriteByte(value,0)

#define lcdWriteBytes(A,B,C,D,E,F,G,H) {        \
    lcdWriteByte(A,1);                          \
    lcdWriteByte(B,1);                          \
    lcdWriteByte(C,1);                          \
    lcdWriteByte(D,1);                          \
    lcdWriteByte(E,1);                          \
    lcdWriteByte(F,1);                          \
    lcdWriteByte(G,1);                          \
    lcdWriteByte(H,1);                          \
  }



void lcdWriteNibble(uint8_t value) {

WRITE(UI_DISPLAY_D4_PIN, value & 1);
 WRITE(UI_DISPLAY_D5_PIN, value & 2);
 WRITE(UI_DISPLAY_D6_PIN, value & 4);
 WRITE(UI_DISPLAY_D7_PIN, value & 8);
 delayMicroseconds(1);

 WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
 delayMicroseconds(2);        //  The enable pulse must be more than 450ns.  We wait 2000ns.

 WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
 delayMicroseconds(UI_DELAYPERCHAR);
}


void lcdWriteByte(uint8_t c, uint8_t rs) {

  WRITE(UI_DISPLAY_RS_PIN, rs);     //  Data (1) or Command (0)?

  WRITE(UI_DISPLAY_D4_PIN, c & 0x10);
  WRITE(UI_DISPLAY_D5_PIN, c & 0x20);
  WRITE(UI_DISPLAY_D6_PIN, c & 0x40);
  WRITE(UI_DISPLAY_D7_PIN, c & 0x80);
  delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
  delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

  WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
  WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
  WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
  WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
  delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
  delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
  delayMicroseconds(100);
}



void
UIDisplay::initialize() {

#ifdef NEW_ACTIONS
  slowKeyAction = 0;
  fastButtonStateChange = 0;
  slowActionRunning = 0;
  keyTestRunning = 0;
#else
  flags = 0;
#endif

  _menuPage = 0;      //  Showing the first page.
  _menuPos  = 0;      //  With the first entry highlighted.
  _menuSel  = 255;    //  But no action selected.
  _menuTop  = 0;      //

  uint32_t timeNow = millis();

  _stopChangeTime  = timeNow;
  _stopMenuTime    = timeNow;
  _refreshPageTime = timeNow;

  rbp = 0;
  rb[0] = 0;

  smp = 0;
  sm[0] = 0;

  encoderLast = 0;
  encoderPos  = 0;

  lastEncoderTime = timeNow;
  encoderAccel    = 1.0;

#ifdef NEW_ACTIONS
  _buttonTime     = 0;
  _buttonAction   = 0;
  _executedAction = 0;

  //uint16_t      activeAction; // action for ok/next/previous
  //uint16_t      delayedAction;
  //uint32_t      lastRefresh;
#else
  activeAction = 0;
  lastAction = 0;
  delayedAction = 0;
  lastSwitch = 0;
  lastRefresh = 0;
  lastButtonAction = 0;
  lastButtonStart = 0;
  nextRepeat = 0;
  repeatDuration = 0;
#endif

  //shift = 0;
  pageDelay = 0;
  errorMsg = NULL;
  outputMask = 0;
  encoderStartScreen = 0;
  statusMsg[0] = 0;



  //
  //  Initialize the encoder and buttons.
  //

  SET_INPUT(UI_ENCODER_A);   PULLUP(UI_ENCODER_A, HIGH);   //  Make the encoder be active low (I think).
  SET_INPUT(UI_ENCODER_B);   PULLUP(UI_ENCODER_B, HIGH);

  SET_INPUT(UI_ENCODER_CLICK);       //  Initialize the encoder clicker
  PULLUP(UI_ENCODER_CLICK, HIGH);    //  to be active low (I think).

  SET_INPUT(UI_KILL_PIN);            //  Initialize the e-stop
  PULLUP(UI_KILL_PIN, HIGH);         //  to be active low (I think).

  //
  //  Set up the LCD!
  //

  delay(235);

  SET_OUTPUT(UI_DISPLAY_D4_PIN);
  SET_OUTPUT(UI_DISPLAY_D5_PIN);
  SET_OUTPUT(UI_DISPLAY_D6_PIN);
  SET_OUTPUT(UI_DISPLAY_D7_PIN);
  SET_OUTPUT(UI_DISPLAY_RS_PIN);

#if (UI_DISPLAY_RW_PIN > -1)
  SET_OUTPUT(UI_DISPLAY_RW_PIN);
#endif

  SET_OUTPUT(UI_DISPLAY_ENABLE_PIN);

  //  Wait for at least 15ms after Vcc rises to 4.5 V.
  //  Probably not necessary, since the time to initialize the CPU
  //  should be at least as long.

  delay(100);

  //  Pull both RS and R/W low to begin commands.  We don't have an R/W pin
  //  so assume it's tied to LOW.

  WRITE(UI_DISPLAY_RS_PIN,     LOW);
  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

  delayMicroseconds(100);

  //  Put LCD into 4-bit mode.
  //
  //  To do this, send the high nibble of the first four initialization
  //  commands:
  //    
  lcdWriteNibble(0x03);    delayMicroseconds(4100);   //  Function set.  Wait at least 4.1ms
  lcdWriteNibble(0x03);    delayMicroseconds(100);    //  Function set.  Wait at least 0.1ms
  lcdWriteNibble(0x03);    delayMicroseconds(100);    //  Function set.  No wait specified, but we'll still wait.
  lcdWriteNibble(0x02);    delayMicroseconds(100);    //  Function set, set interface to 4-bit mode.

  //
  //  See page 24, Table 6 for instructions.
  //

  //             v------- FUNCTION SET
  //              v------ interface mode: 1=8-bit    0=4-bit
  //               v----- num lines:      1=2 lines  0=1 line
  //                v---- format:         1=5x10     0=5x7
  //                 xx-- don't care
  lcdCommand(0b00101000);
  delayMicroseconds(200);

  //                  v-- CLEAR DISPLAY - wait for at least 1.52ms, else the next command is lost.
  lcdCommand(0b00000001);
  delayMicroseconds(2500);

  //                 v--- DISPLAY AND CURSOR HOME (moves cursor to home, unshifts display)
  //                  v-- don't care
  lcdCommand(0b00000010);
  delayMicroseconds(200);

  //               v----- DISPLAY ON/OFF and CURSOR MODE
  //                v---- display on           1=on    0=off
  //                 v--- cursor underlining:  1=on    0=off
  //                  v-- cursor blinking:     1=blink 0=off
  lcdCommand(0b00001100);
  delayMicroseconds(200);

  //                v---- CHARACTER ENTRY MOVEMENT and DISPLAY SHIFT
  //                 v--- direction         1=increment 0=decrement
  //                  v-- display shift     1=shift     0=off
  lcdCommand(0b00000110);
  delayMicroseconds(200);

  // MenuBack  Degrees   Selected  Unsel     Temp      Folder    Ready   
  // ..*..  4  ..*..  4  .....  0  .....  0  ..*..  4  .....  0  *...* 17
  // .***. 14  .*.*. 10  ***** 31  ***** 31  .*.*. 10  ***.. 28  .*.*. 10
  // *.*.* 21  ..*..  4  ***** 31  *...* 17  .*.*. 10  ***** 31  ..*..  4
  // ..*..  4  .....  0  ***** 31  *...* 17  .*.*. 10  *...* 17  *...* 17
  // ..*..  4  .....  0  ***** 31  *...* 17  .***. 14  *...* 17  ..*..  4
  // ..*..  4  .....  0  ***** 31  *...* 17  ***** 31  ***** 31  .*.*. 10
  // ***.. 28  .....  0  ***** 31  ***** 31  ***** 31  .....  0  *...* 17
  // .....  0  .....  0  .....  0  .....  0  .***. 14  .....  0  *...* 17
  //
  //            v----- Set CGRAM address, CGRAM data is sent after this instruction.
  //             vvv-- CGRAM address
  lcdCommand(0b01001000);  lcdWriteBytes( 4, 14, 21,  4,  4,  4, 28,  0);  //  Char 1
  lcdCommand(0b01010000);  lcdWriteBytes( 4, 10,  4,  0,  0,  0,  0,  0);  //  Char 2
  lcdCommand(0b01011000);  lcdWriteBytes( 0, 31, 31, 31, 31, 31,  0,  0);  //  Char 3
  lcdCommand(0b01100000);  lcdWriteBytes( 0, 31, 17, 17, 17, 31,  0,  0);  //  Char 4
  lcdCommand(0b01101000);  lcdWriteBytes( 4, 10, 10, 10, 14, 31, 31, 14);  //  Char 5
  lcdCommand(0b01110000);  lcdWriteBytes( 0, 28, 31, 17, 17, 31,  0,  0);  //  Char 6
  lcdCommand(0b01111000);  lcdWriteBytes(17, 10,  4, 17,  4, 10, 17, 17);  //  Char 7

  //
  //  And display a nice greeting.
  //

  setStatusP(PSTR("Printer ready."));

  printRowP(0, PSTR("Rostock Max v2"));
  printRowP(1, PSTR("Repetier 1.0.2[bri]"));
  printRowP(2, PSTR(""));
  printRowP(3, PSTR(""));

  //  Show the start screen for 2 seconds.
#if 1
  delay(2000);
#else
  delay(282);   printRowP(3, PSTR("                   b"));
  delay(282);   printRowP(3, PSTR("                  br"));
  delay(282);   printRowP(3, PSTR("                 bri"));
  delay(282);   printRowP(3, PSTR("                bri"));
  delay(282);   printRowP(3, PSTR("               bri"));
  delay(282);   printRowP(3, PSTR("              bri"));
  delay(282);   printRowP(3, PSTR("             bri"));
  delay(282);   printRowP(3, PSTR("            bri"));
  delay(282);   printRowP(3, PSTR("           bri"));
  delay(282);   printRowP(3, PSTR("          bri"));
  delay(282);   printRowP(3, PSTR("         bri"));
  delay(282);   printRowP(3, PSTR("        bri"));
  delay(282);   printRowP(3, PSTR("       bri"));
  delay(282);   printRowP(3, PSTR("      bri"));
  delay(282);   printRowP(3, PSTR("     bri"));
  delay(282);   printRowP(3, PSTR("    bri"));
  delay(282);   printRowP(3, PSTR("   bri"));
  delay(282);   printRowP(3, PSTR("  bri"));
  delay(282);   printRowP(3, PSTR(" bri"));
  delay(282);   printRowP(3, PSTR("bri"));
  delay(282);   printRowP(3, PSTR("ri"));
  delay(282);   printRowP(3, PSTR("i"));
  delay(282);   printRowP(3, PSTR(""));
#endif
}



void
UIDisplay::printRow(uint8_t r, char *text) {
  uint8_t c = 0;

  //  Set the cursor to      v-------  Set DRAM address
  //  the start of a line     vvvvvvv  DRAM address 
  if (r == 0)   lcdCommand(0b10000000 | 0x00);
  if (r == 1)   lcdCommand(0b10000000 | 0x40);
  if (r == 2)   lcdCommand(0b10000000 | 0x14);
  if (r == 3)   lcdCommand(0b10000000 | 0x54);
  if (r >= 4)   return;

  for (; (c < UI_COLS) && (text[c] != 0); c++)
    lcdPutChar(text[c]);

  for (; c < UI_COLS; c++)
    lcdPutChar(' ');
}



void
UIDisplay::printRowP(uint8_t r, const char *text) {
  uint8_t c = 0;

  //  Set the cursor to      v-------  Set DRAM address
  //  the start of a line     vvvvvvv  DRAM address 
  if (r == 0)   lcdCommand(0b10000000 | 0x00);
  if (r == 1)   lcdCommand(0b10000000 | 0x40);
  if (r == 2)   lcdCommand(0b10000000 | 0x14);
  if (r == 3)   lcdCommand(0b10000000 | 0x54);
  if (r >= 4)   return;

  for (; (c < UI_COLS) && (pgm_read_byte(&text[c]) != 0); c++)
    lcdPutChar(pgm_read_byte(&text[c]));

  for (; c < UI_COLS; c++)
    lcdPutChar(' ');
}





void
UIDisplay::addStringP(const char *text) {
  while (rbp < MAX_COLS) {
    uint8_t c = pgm_read_byte(text++);

    if (c == 0)
      return;

    rb[rbp++] = c;
  }
}



void
UIDisplay::addString(const char *text) {
  while (rbp < MAX_COLS) {
    uint8_t c = *text++;

    if (c == 0)
      return;

    rb[rbp++] = c;
  }
}



void
UIDisplay::addNumber(int32_t number, int8_t digits, char fillChar) {
  char     buf[12];
  int8_t   pos = 0;
  int8_t   neg = 0;

  if (number < 0) {
    number = -number;
    neg    = 1;
  }

  int32_t  m;
  char     c;

  do {
    m = number;

    number /= 10;

    buf[pos++] = m - 10 * number + '0';
  } while (number > 0);

  if (neg)
    buf[pos++] = '-';

  while (pos < digits)
    buf[pos++] = fillChar;

  while ((rbp < MAX_COLS) && (pos > 0))
    rb[rbp++] = buf[--pos];

  rb[rbp] = 0;
}



void
UIDisplay::addFloat(float number, char wholeDigits, uint8_t fractDigits) {

  if (rbp >= MAX_COLS)
    return;

  //  If negative, append a minus sign and make a positive number.

  if (number < 0.0) {
    rb[rbp++] = '-';
    number = -number;

    wholeDigits--;
  }

  if (rbp >= MAX_COLS)
    return;

  //  Round.

  if (fractDigits == 0)  number += 0.5;
  if (fractDigits == 1)  number += 0.05;
  if (fractDigits == 2)  number += 0.005;
  if (fractDigits == 3)  number += 0.0005;
  if (fractDigits == 4)  number += 0.00005;
  if (fractDigits == 5)  number += 0.000005;
  if (fractDigits == 6)  number += 0.0000005;

  //  Extract the integer part of the number and display it.

  uint32_t  whole = (uint32_t)number;
  float     fract = number - (float)whole;

  addNumber(whole, wholeDigits);

  if (rbp >= MAX_COLS)
    return;

  //  Display the fractional part, if requested.

  if (fractDigits == 0)
    return;

  rb[rbp++] = '.';

  for (uint8_t d=0; d<fractDigits; d++)
    fract *= 10;

  addNumber(fract, fractDigits, '0');
}



//  Print a time in seconds as "x days xx:xx".
void
UIDisplay::addTimeInDaysHoursMinutes(uint32_t seconds) {
  uint32_t days    = seconds / 86400;  seconds -= days  * 86400;
  uint32_t hours   = seconds / 3600;   seconds -= hours * 3600;
  uint32_t minutes = seconds / 60;

  addNumber(days, 1);
  addStringP(PSTR(" days "));
  addNumber(hours, 2, '0');
  addStringP(PSTR(":"));
  addNumber(minutes, 2, '0');
}



//  Print a time in seconds as "xx:xx:xx".
void
UIDisplay::addTimeInHoursMinutesSeconds(uint32_t seconds) {
  uint32_t hours   = seconds / 3600;   seconds -= hours * 3600;
  uint32_t minutes = seconds / 60;     seconds -= minutes  * 60;

  addNumber(hours, 2, '0');
  addStringP(PSTR(":"));
  addNumber(minutes, 2, '0');
  addStringP(PSTR(":"));
  addNumber(seconds, 2, '0');
}



void
UIDisplay::setStatusP(const char *text, bool error) {

  if ((error == false) && Printer::isUIErrorMessage())   //  Don't change if we're
    return;                                              //  in an error state.

  smp = 0;

  while (smp < MAX_COLS) {
    uint8_t c = pgm_read_byte(text++);

    if (c == 0)
      return;

    sm[smp++] = c;
  }

  sm[smp] = 0;

  if (error)
    Printer::setUIErrorMessage(true);
}



void
UIDisplay::setStatus(const char *text, bool error) {

  if ((error == false) && Printer::isUIErrorMessage())   //  Don't change if we're
    return;                                              //  in an error state.

  smp = 0;

  while (smp < MAX_COLS) {
    uint8_t c = *text++;

    if (c == 0)
      return;

    sm[smp++] = c;
  }

  sm[smp] = 0;

  if (error)
    Printer::setUIErrorMessage(true);
}








#include "ui-action-change.h"
#include "ui-action-execute.h"
#include "ui-action-ok.h"
#include "ui-parse.h"
#include "ui-sdcard.h"




//  Adjust _menuPos so that we're always on a visible item.
//
//  Adjust _menuTop so that (a) the display is as full as possible, and
//                          (b) the active element is displayed.
//
void
UIDisplay::adjustMenuPos(void) {
  menuPage     *menu       = menuPagePtr(_menuPage);
  uint8_t       menuType   = menu->type();
  uint8_t       entriesLen = menu->entriesLen();

  //  Find the next visible entry.  Usually, it's the one we're sitting on,
  //  but if not, search up in the list (decreasing indices), then down
  //  (increasing indices), until we find something visible.  And make that
  //  be the position we're at.

#undef DEBUG_ADJUST_MENU_POS

#ifdef DEBUG_ADJUST_MENU_POS
  Com::print("adjust menuMode=");
  Com::print(Printer::menuMode);
  Com::print(" menuPos=");
  Com::print(_menuPos);
#endif

  while (_menuPos > 0) {
    if (menu->entry(_menuPos)->visible() == true)
      break;
    else
      _menuPos--;
  }

#ifdef DEBUG_ADJUST_MENU_POS
  Com::print(" menuPos=");
  Com::print(_menuPos);
#endif

  while (_menuPos < entriesLen - 1) {
    if (menu->entry(_menuPos)->visible() == true)
      break;
    else
      _menuPos++;
  }

#ifdef DEBUG_ADJUST_MENU_POS
  Com::print(" menuPos=");
  Com::print(_menuPos);
#endif

  //  If the active position is before the top, reset the top
  //  to show the active position.

  if (_menuPos < _menuTop)
    _menuTop = _menuPos;

  //  But now some pain.  We need to move the top down the list (increase the value)
  //  if the position we're at is more than UI_ROWS _visible_ elements away.
  //
  //  There's probably a way to do this by counting backwards from _menuPos.

  uint8_t  nVisible = UI_ROWS + 1;

  while (nVisible > UI_ROWS) {
    nVisible = 0;

    for (uint8_t vv=_menuTop; vv<=_menuPos; vv++) {
      if (menu->entry(_menuPos)->visible() == true)
        nVisible++;
    }

    if (nVisible > UI_ROWS)
      _menuTop++;
  }

#ifdef DEBUG_ADJUST_MENU_POS
  Com::print(" menuTop=");
  Com::print(_menuTop);
  Com::print("\n");
#endif
}




void
UIDisplay::refreshPage(void) {
  char    cache[UI_ROWS][MAX_COLS + 1] = {0};

  //  Really doesn't belong here.
#warning no endstops.update in refreshPage.
  endstops.update();

  //  Figure out what to display.

  //  If the current menu page isn't visible, move to the next.

  while (menuPagePtr(_menuPage)->visible() == false)
    doEncoderChange_page(1);

  menuPage    *menu       = menuPagePtr(_menuPage);
  uint8_t      menuType   = menu->type();
  uint8_t      entriesLen = menu->entriesLen();

  uint8_t       rowi       = 0;
  uint8_t       enti       = 0;

#undef DEBUG_REFRESH_PAGE
#undef DEBUG_REFRESH_PAGE_FULL

#ifdef DEBUG_REFRESH_PAGE
  Com::print("refreshPage _menuPage=");
  Com::print(_menuPage);
  Com::print(" menuType=");
  Com::print(menuType);
  Com::print(" entriesLen=");
  Com::print(entriesLen);
  Com::print(" _menuTop=");
  Com::print(_menuTop);
  Com::print(" _menuPos=");
  Com::print(_menuPos);
  Com::print(" _menuSel=");
  Com::print(_menuSel);
  Com::print(" isPrinting==");
  Com::print(Printer::isPrinting());
  Com::print("\n");
#endif

  //  If the current menu page is a file selector, get a file list and show it.

  if ((menuType == menuType_select) &&
      (Printer::menuMode & MODE_CARD_PRESENT)) {
      rowi = sdrefresh(cache);
  }

  //  But if it's a menu, show those items.
  //    (menuType == menuType_normal)

  else {
    adjustMenuPos();

    rowi = 0;
    enti = _menuTop;

    while ((enti < entriesLen) && (rowi < UI_ROWS)) {
      menuEntry   *entry     = menu->entry(enti);
      const char  *entryText = entry->text();
      uint8_t      entryType = entry->type();

#ifdef DEBUG_REFRESH_PAGE_FULL
      Com::print("entryTextRaw=");
      Com::print((uint16_t)entry->entryText);
      Com::print(" text='");
      Com::printF(entryText);
      Com::print("'\n");
#endif

      if (entry->visible() == false) {
        enti++;
        continue;
      }

      rbp = 0;
      parse(entryText, false);

      //  Copy the parsed string to our display buffer.

      for (uint8_t c=0; c<MAX_COLS; c++)
        cache[rowi][c] = rb[c];

      //  If an entryType_page, add arrows on the ends

      if (entryType == entryType_page) {
        cache[rowi][1]         = 0x7f;
        cache[rowi][UI_COLS-1] = 0x7e;
      }

      //  Set the SELECTOR / SELECTED icon if the row is active or selected.

      if      (enti == _menuSel)
        cache[rowi][0] = CHAR_SELECTED;

      else if (enti == _menuPos)
        cache[rowi][0] = CHAR_SELECTOR;

      else
        cache[rowi][0] = ' ';

      rowi++;
      enti++;
    }
  }

  //  Blank out any remaning rows.

  while (rowi < UI_ROWS)
    cache[rowi++][0] = 0;

  cache[0][UI_COLS] = 0;
  cache[1][UI_COLS] = 0;
  cache[2][UI_COLS] = 0;
  cache[3][UI_COLS] = 0;

  //  Do we need to scroll the display?  Nope.

  uint8_t off[UI_ROWS] = {0};

#if 0
  uint8_t off0 = (shift <= 0 ? 0 : shift);

  for (uint8_t y=0; y<UI_ROWS; y++) {
    uint8_t len = strlen(cache[y]); // length of line content

    off[y] = 0;
    off[y] = (len > UI_COLS) ? RMath::min(len - UI_COLS, off0) : 0;

    if(len > UI_COLS) {
      off[y] = RMath::min(len - UI_COLS, off0);

      // Copy first char to front
      if ((menType == UI_MENU_TYPE_FILE_SELECTOR) ||
          (menType == UI_MENU_TYPE_SUBMENU)) {
        cache[y][off[y]] = cache[y][0];
      }

    } else off[y] = 0;
  }
#endif

  //  Show it!

  for (uint8_t y=0; y<UI_ROWS; y++)
    printRow(y, cache[y] + off[y]);
}
























// Gets calls from main tread only
void UIDisplay::slowAction(bool allowMoves) {
  uint32_t time    = millis();
  uint8_t  refresh = 0;

  // delayed action open?

  if (allowMoves && delayedAction != 0) {
    executeAction(delayedAction, true);
    delayedAction = 0;
  }

  // Update key buffer

  InterruptProtectedBlock noInts;

  if((flags & (UI_FLAG_FAST_KEY_ACTION + UI_FLAG_KEY_TEST_RUNNING)) == 0) {
    flags |= UI_FLAG_KEY_TEST_RUNNING;
    noInts.unprotect();

    uint16_t nextAction = 0;

    if(lastButtonAction != nextAction) {
      lastButtonStart = time;
      lastButtonAction = nextAction;

      noInts.protect();

      flags |= UI_FLAG_SLOW_KEY_ACTION; // Mark slow action
    }

    noInts.protect();

    flags &= ~UI_FLAG_KEY_TEST_RUNNING;
  }

  noInts.protect();

  if ((flags & UI_FLAG_SLOW_ACTION_RUNNING) == 0) {
    flags |= UI_FLAG_SLOW_ACTION_RUNNING;

    // Reset click encoder

    noInts.protect();

    int16_t encodeChange = encoderPos;

    encoderPos = 0;

    noInts.unprotect();

    int newAction;

    //  If the encoder changed, update whaever we're doing.

    if(encodeChange) {
      doEncoderChange(encodeChange, allowMoves);

      //uiChirp();
      refresh = 1;
    }

    //  

    if(lastAction != lastButtonAction) {
      if(lastButtonAction == 0) {
        if(lastAction >= 2000 && lastAction < 3000)
          statusMsg[0] = 0;
        lastAction = 0;
        noInts.protect();
        flags &= ~(UI_FLAG_FAST_KEY_ACTION + UI_FLAG_SLOW_KEY_ACTION);
      } else if(time - lastButtonStart > UI_KEY_BOUNCETIME) { // New key pressed
        lastAction = lastButtonAction;
        uiChirp();
        if((newAction = executeAction(lastAction, allowMoves)) == 0) {
          nextRepeat = time + UI_KEY_FIRST_REPEAT;
          repeatDuration = UI_KEY_FIRST_REPEAT;
        } else {
          if(delayedAction == 0)
            delayedAction = newAction;
        }
      }
    } else if(lastAction < 1000 && lastAction) { // Repeatable key
      if(time - nextRepeat < 10000) {
        if(delayedAction == 0)
          delayedAction = executeAction(lastAction, allowMoves);
        else
          executeAction(lastAction, allowMoves);
        repeatDuration -= UI_KEY_REDUCE_REPEAT;
        if(repeatDuration < UI_KEY_MIN_REPEAT) repeatDuration = UI_KEY_MIN_REPEAT;
        nextRepeat = time + repeatDuration;
        uiChirp();
      }
    }
    noInts.protect();
    flags &= ~UI_FLAG_SLOW_ACTION_RUNNING;
  }

  noInts.unprotect();

  // Go to top menu after x seconds

  //if(menuLevel > 0 && ui_autoreturn_time < time && !uid.isSticky()) {
  //  menuLevel = 0;
  //  activeAction = 0;
  //}

  //  Refresh the display, every second for the main, and 0.8 seconds for non-main menus.

  if (time - lastRefresh >= 800)
    refresh = 1;

#if 0
  if (refresh) {
    if (menuLevel > 1 || Printer::isAutomount()) {
      shift++;

      if(shift + UI_COLS > MAX_COLS + 1)
        shift = -2;
    } else {
      shift = -2;
    }
  }
#endif

  if (refresh) {
    refreshPage();
    lastRefresh = time;
  }
}





// Gets called from inside an interrupt with interrupts allowed!
//
void
UIDisplay::fastAction(void) {

  InterruptProtectedBlock noInts;

  if ((flags & (UI_FLAG_KEY_TEST_RUNNING + UI_FLAG_SLOW_KEY_ACTION)) == 0) {
    flags |= UI_FLAG_KEY_TEST_RUNNING;

    uint16_t nextAction = uiCheckKeys();

    if (lastButtonAction != nextAction) {
      lastButtonStart = millis();
      lastButtonAction = nextAction;

      flags |= UI_FLAG_FAST_KEY_ACTION;
    }

    flags &= ~UI_FLAG_KEY_TEST_RUNNING;
  }
}



