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

#include "uimenu.h"       //  Menu definitions.

UIDisplay uid;


static TemperatureController *currHeaterForSetup;    // pointer to extruder or heatbed temperature controller


const UIMenu * const ui_pages[UI_NUM_PAGES] PROGMEM = UI_PAGES;



//  If idle in a sub menu for this many milliseconds, return to the
//  first status page.
#define UI_AUTORETURN_TO_MENU_AFTER 30000
millis_t ui_autoreturn_time = 0;

UI_STRING(ui_selected,   cSEL);
UI_STRING(ui_unselected, cUNSEL);




void
uiCheckKeys(uint16_t &action) {

  //  Shift the encoder one position.

  uid.encoderLast <<= 2;
  uid.encoderLast  &= 0x0f;

  //  And add in the new value.

  if (READ(UI_ENCODER_A) == 0)    //  Active low.
    uid.encoderLast |= 2;

  if (READ(UI_ENCODER_B) == 0)    //  Active low.
    uid.encoderLast |= 1;

  //  To change direction of the encoder, swap increment and decrement.
  //
  //  To change the speed of the encoder, test against different values.
  //     FAST =  1,  7,  8, 14   MEDIUM =  7,  8   SLOW = 14
  //     FAST =  2,  4, 11, 13   MEDIUM =  2, 13   SLOW = 11

  if ((uid.encoderLast ==  7) || (uid.encoderLast ==  8))
    uid.encoderPos -= 1;
  if ((uid.encoderLast ==  2) || (uid.encoderLast == 13))
    uid.encoderPos += 1;

  //  Check for the two buttons, one on the encoder, and the estop/reset.

  if (READ(UI_ENCODER_CLICK) == 0)    //  If ENCODER_CLICK is pushed (active low)
    action = UI_ACTION_OK;            //  make the action be "OK".

  if (READ(UI_KILL_PIN) == 0)         //  If KILL is pushed (active low)
    action = UI_ACTION_KILL;          //  kill ourself.
}



void
uiChirp(void) {

  SET_OUTPUT(BEEPER_PIN);

  WRITE(BEEPER_PIN, HIGH);   HAL::delayMilliseconds(1);
  WRITE(BEEPER_PIN, LOW);    HAL::delayMilliseconds(1);
}



void
uiAlert(void) {

  SET_OUTPUT(BEEPER_PIN);

  for (uint8_t i=0; i<4; i++) {
    WRITE(BEEPER_PIN, HIGH);   HAL::delayMilliseconds(1);
    WRITE(BEEPER_PIN, LOW);    HAL::delayMilliseconds(2);
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
 HAL::delayMicroseconds(1);

 WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
 HAL::delayMicroseconds(2);        //  The enable pulse must be more than 450ns.  We wait 2000ns.

 WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
 HAL::delayMicroseconds(UI_DELAYPERCHAR);
  }


void lcdWriteByte(uint8_t c, uint8_t rs) {

  WRITE(UI_DISPLAY_RS_PIN, rs);     //  Data (1) or Command (0)?

  WRITE(UI_DISPLAY_D4_PIN, c & 0x10);
  WRITE(UI_DISPLAY_D5_PIN, c & 0x20);
  WRITE(UI_DISPLAY_D6_PIN, c & 0x40);
  WRITE(UI_DISPLAY_D7_PIN, c & 0x80);
  HAL::delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
  HAL::delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

  WRITE(UI_DISPLAY_D4_PIN, c & 0x01);
  WRITE(UI_DISPLAY_D5_PIN, c & 0x02);
  WRITE(UI_DISPLAY_D6_PIN, c & 0x04);
  WRITE(UI_DISPLAY_D7_PIN, c & 0x08);
  HAL::delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, HIGH);
  HAL::delayMicroseconds(2);

  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);
  HAL::delayMicroseconds(100);
}
















bool
UIMenuEntry::showEntry(void) const {

  //  Get the value of 'filter', store in f.
  uint16_t  ft = pgm_read_word(&filter);
  uint16_t  nf = pgm_read_word(&nofilter);

  //  Do not show if all of the 'do-show' filter bits are missing in the mode menu.
  if ((ft != 0) && ((ft & Printer::menuMode) == 0))
    return(false);

  //  Do not show if any of the 'no-show' filter bits are set in the menu mode.
  if ((nf & Printer::menuMode) != 0)
    return(false);

  //  Default to showing.
  return(true);
}



void
UIDisplay::printRow(uint8_t r, char *txt) {
  uint8_t c = 0;

  //  Set the cursor to the start of the line.
  //
  //                         v-------  Set DRAM address
  //                          vvvvvvv  DRAM address 

  if (r == 0)   lcdCommand(0b10000000 | 0x00);
  if (r == 1)   lcdCommand(0b10000000 | 0x40);
  if (r == 2)   lcdCommand(0b10000000 | 0x14);
  if (r == 3)   lcdCommand(0b10000000 | 0x54);
  if (r >= 4)   return;

  for (; (c < UI_COLS) && (txt[c] != 0); c++)
    lcdPutChar(txt[c]);

  for (; c < UI_COLS; c++)
    lcdPutChar(' ');
}



void
UIDisplay::printRowP(uint8_t r, PGM_P txt) {

  col = 0;

  addStringP(txt);

  printCols[col] = 0;

  printRow(r, printCols);
}






void UIDisplay::initialize() {

  flags            = 0;
  menuLevel        = 0;
  shift            = -2;
  menuPos[0]       = 0;
  lastAction       = 0;
  delayedAction    = 0;
  lastButtonAction = 0;
  activeAction     = 0;
  statusMsg[0]     = 0;

  cwd[0]           = '/';
  cwd[1]           = 0;

  lastSwitch  = HAL::timeInMilliseconds();
  lastRefresh = HAL::timeInMilliseconds();



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

  HAL::delayMilliseconds(235);

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

  HAL::delayMilliseconds(100);

  //  Pull both RS and R/W low to begin commands.  We don't have an R/W pin
  //  so assume it's tied to LOW.

  WRITE(UI_DISPLAY_RS_PIN,     LOW);
  WRITE(UI_DISPLAY_ENABLE_PIN, LOW);

  HAL::delayMicroseconds(100);

  //  Put LCD into 4-bit mode.
  //
  //  To do this, send the high nibble of the first four initialization
  //  commands:
  //    
  lcdWriteNibble(0x03);    HAL::delayMicroseconds(4100);   //  Function set.  Wait at least 4.1ms
  lcdWriteNibble(0x03);    HAL::delayMicroseconds(100);    //  Function set.  Wait at least 0.1ms
  lcdWriteNibble(0x03);    HAL::delayMicroseconds(100);    //  Function set.  No wait specified, but we'll still wait.
  lcdWriteNibble(0x02);    HAL::delayMicroseconds(100);    //  Function set, set interface to 4-bit mode.

  //
  //  See page 24, Table 6 for instructions.
  //

  //             v------- FUNCTION SET
  //              v------ interface mode: 1=8-bit    0=4-bit
  //               v----- num lines:      1=2 lines  0=1 line
  //                v---- format:         1=5x10     0=5x7
  //                 xx-- don't care
  lcdCommand(0b00101000);
  HAL::delayMicroseconds(200);

  //                  v-- CLEAR DISPLAY - wait for at least 1.52ms, else the next command is lost.
  lcdCommand(0b00000001);
  HAL::delayMicroseconds(2500);

  //                 v--- DISPLAY AND CURSOR HOME (moves cursor to home, unshifts display)
  //                  v-- don't care
  lcdCommand(0b00000010);
  HAL::delayMicroseconds(200);

  //               v----- DISPLAY ON/OFF and CURSOR MODE
  //                v---- display on           1=on    0=off
  //                 v--- cursor underlining:  1=on    0=off
  //                  v-- cursor blinking:     1=blink 0=off
  lcdCommand(0b00001100);
  HAL::delayMicroseconds(200);

  //                v---- CHARACTER ENTRY MOVEMENT and DISPLAY SHIFT
  //                 v--- direction         1=increment 0=decrement
  //                  v-- display shift     1=shift     0=off
  lcdCommand(0b00000110);
  HAL::delayMicroseconds(200);

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

  UI_STATUS_F(PSTR("Printer ready."));

  printRowP(0, PSTR("Rostock Max v2"));
  printRowP(1, PSTR("Repetier 1.0.2[bri]"));
  printRowP(2, PSTR(""));
  printRowP(3, PSTR(""));

  //  Show the start screen for 2 seconds.
#if 1
  HAL::delayMilliseconds(2000);
#else
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("                   b"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("                  br"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("                 bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("                bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("               bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("              bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("             bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("            bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("           bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("          bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("         bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("        bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("       bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("      bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("     bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("    bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("   bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("  bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR(" bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("bri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("ri"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR("i"));
  HAL::delayMilliseconds(282);   printRowP(3, PSTR(""));
#endif
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

  while ((col < MAX_COLS) && (pos > 0))
    printCols[col++] = buf[--pos];

  printCols[col] = 0;
}





void
UIDisplay::addFloat(float number, char wholeDigits, uint8_t fractDigits) {

  if (col >= MAX_COLS)
    return;

  //  If negative, append a minus sign and make a positive number.

  if (number < 0.0) {
    printCols[col++] = '-';
    number = -number;

    wholeDigits--;
  }

  if (col >= MAX_COLS)
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

  if (col >= UI_COLS)
    return;

  //  Display the fractional part, if requested.

  if (fractDigits == 0)
    return;

  printCols[col++] = '.';

  for (uint8_t d=0; d<fractDigits; d++)
    fract *= 10;

  addNumber(fract, fractDigits, '0');
}



void
UIDisplay::addStringP(FSTRINGPARAM(text)) {
  while (col < MAX_COLS) {
    uint8_t c = pgm_read_byte(text++);
    
    if(c == 0)
      return;

    printCols[col++] = c;
  }
}



void
UIDisplay::addString(const char *text) {
  while (col < MAX_COLS) {
    uint8_t c = *text++;

    if(c == 0)
      return;

    printCols[col++] = c;
  }
}



void
UIDisplay::setStatusP(PGM_P txt, bool error) {

  if (!error && Printer::isUIErrorMessage())
    return;

  uint8_t i = 0;

  while(i < 20) {
    uint8_t c = pgm_read_byte(txt++);
    if(!c) break;
    statusMsg[i++] = c;
  }

  statusMsg[i] = 0;

  if(error)
    Printer::setUIErrorMessage(true);
}



void
UIDisplay::setStatus(const char *txt, bool error) {

  if (!error && Printer::isUIErrorMessage())
    return;

  uint8_t i = 0;

  while(*txt && i < 20)
    statusMsg[i++] = *txt++;

  statusMsg[i] = 0;

  if(error)
    Printer::setUIErrorMessage(true);
}









#include "ui-action-change.h"
#include "ui-action-execute.h"
#include "ui-parse.h"
#include "ui-sdcard.h"










//  Find the 'next' visible and useful menu item to show.
//  Sets the 
void
UIDisplay::adjustMenuPos(void) {

  //  If not in a menu, there is no position to adjust.

  if (menuLevel == 0)
    return;

  //  Figure out what menu we're in.

  UIMenu       *men        = (UIMenu *)menu[menuLevel];
  UIMenuEntry **entries    = (UIMenuEntry **)pgm_read_ptr(&(men->entries));
  UIMenuEntry  *entry      = NULL;
  uint8_t       mtype      = pgm_read_byte(&(men->menuType)) & 127;
  uint8_t       numEntries = pgm_read_byte(&(men->numEntries));

  if (mtype != UI_MENU_TYPE_SUBMENU)
    return;

  //  Move up until we reach a visible position.

  while (menuPos[menuLevel] > 0) {
    UIMenuEntry *entry = (UIMenuEntry *)pgm_read_ptr(&(entries[menuPos[menuLevel]]));

    if (pgm_read_byte(&(entry->entryType)) == 1) // skip headlines
      menuPos[menuLevel]--;

    else if (entry->showEntry() == true)   //  Found something to show!
      break;

    else
      menuPos[menuLevel]--;
  }

  //  With bad luck the only visible option was in the opposite direction
  //  So go down until we reach a visible position.

  while (menuPos[menuLevel] < numEntries - 1) {
    UIMenuEntry *entry = (UIMenuEntry *)pgm_read_ptr(&(entries[menuPos[menuLevel]]));

    if(pgm_read_byte(&(entry->entryType)) == 1) // skip headlines
      menuPos[menuLevel]++;

    else if (entry->showEntry() == true)   //  Found something to show!
      break;

    else
      menuPos[menuLevel]++;
  }

  //  Adjust the top to keep the list
  //  from scrolling up past the bottom
  //  (I think)

  uint16_t skipped = 0;
  bool     modified;

  if(menuTop[menuLevel] > menuPos[menuLevel])
    menuTop[menuLevel] = menuPos[menuLevel];

  do {
    skipped  = 0;
    modified = false;

    for (uint8_t r=menuTop[menuLevel]; r<menuPos[menuLevel]; r++) {
      UIMenuEntry *entry = (UIMenuEntry *)pgm_read_ptr(&(entries[r]));

      if (entry->showEntry() == false)
        skipped++;
    }

    if ((menuTop[menuLevel] + skipped + UI_ROWS) - 1 < menuPos[menuLevel]) {
      menuTop[menuLevel]  = menuPos[menuLevel] + 1;
      menuTop[menuLevel] -= UI_ROWS;
      modified = true;
    }

  } while(modified);
}



// Refresh current menu page
void
UIDisplay::refreshPage(void) {
  char    cache[UI_ROWS][MAX_COLS + 1] = {0};

  Endstops::update();

  //  This does what?

  adjustMenuPos();

  // Reset timeout on menu back when user active on menu

  if (encoderLast != encoderStartScreen)
    ui_autoreturn_time = HAL::timeInMilliseconds() + UI_AUTORETURN_TO_MENU_AFTER;

  encoderStartScreen = encoderLast;

  // Copy result into cache

  Endstops::update();

  //  Figure out what to display.

  uint8_t    rowi    = 0;
  uint8_t    enti    = 0;

  UIMenu    *men     = NULL;
  uint16_t   menType = 0;

#if 0
  Com::print("refreshPage menuLevel=");
  Com::print(menuLevel);
  Com::print(" menuPos[0]==");
  Com::print(menuPos[0]);
  Com::print(" isPrinting==");
  Com::print(Printer::isPrinting());
  Com::print("\n");
#endif

  //
  //  If not in a menu, and printing, show a default display.
  //
  //  isPrinting() is a bit flag, currently returns 8 if true.

  if ((menuLevel == 0) && (menuPos[0] == 0) && (Printer::isPrinting() != false)) {
    //Com::print("printing-display\n");

    if(sd.sdactive && sd.sdmode)
      Printer::progress = (static_cast<float>(sd.sdpos) * 100.0) / static_cast<float>(sd.filesize);

    col = 0;
    parse(PSTR("%Pn"), false);     //  FILENAME
    for (uint8_t c=0; c<MAX_COLS; c++)
      cache[rowi][c] = printCols[c];
    rowi++;

    col = 0;
    parse(PSTR("              %Pp%%"), false);  //  PROGRESS
    for (uint8_t c=0; c<MAX_COLS; c++)
      cache[rowi][c] = printCols[c];
    rowi++;

    col = 0;
    parse(PSTR("Layer %Pl/%PL"), false);
    for (uint8_t c=0; c<MAX_COLS; c++)
      cache[rowi][c] = printCols[c];
    rowi++;

    col = 0;
    parse(PSTR("%os"), false);       //  STATUS
    for (uint8_t c=0; c<MAX_COLS; c++)
      cache[rowi][c] = printCols[c];
    rowi++;
  }

  //
  //  If not in a menu and not printing OR
  //     not in a menu and on a secondary status page, show one of the main status pages.
  //
  //  (status page 0 is replaced with the above page)
  //
  else if (((menuLevel == 0) && (Printer::isPrinting() == false)) ||
           ((menuLevel == 0) && (menuPos[0] > 0))) {
    //Com::print("status-display\n");

    UIMenu       *men        = (UIMenu*)      pgm_read_ptr(&(ui_pages[menuPos[0]]));
    UIMenuEntry **entries    = (UIMenuEntry**)pgm_read_ptr(&(men->entries));
    uint8_t       entriesLen =                pgm_read_byte(&(men->numEntries));

    rowi = 0;
    enti = 0;

    while ((enti < entriesLen) && (rowi < UI_ROWS)) {
      UIMenuEntry *ent = (UIMenuEntry *)pgm_read_ptr(&(entries[enti]));

      col = 0;
      parse((char*)pgm_read_ptr(&(ent->entryText)), false);

      for (uint8_t c=0; c<MAX_COLS; c++)
        cache[rowi][c] = printCols[c];

      rowi++;
      enti++;
    }
  }

  //
  //  If in a menu, show the menu.
  //
  else if (menuLevel > 0) {

    men     = (UIMenu*)menu[menuLevel];
    menType = pgm_read_byte(&(men->menuType));

    if (menType == UI_MENU_TYPE_FILE_SELECTOR) {
      //Com::print("sd-card\n");
      rowi = sdrefresh(cache);
    }

    else {
      //Com::print("menu-display\n");

      UIMenuEntry **entries    = (UIMenuEntry**)pgm_read_ptr(&(men->entries));
      uint8_t       entriesLen =                pgm_read_byte(&(men->numEntries));

      rowi = 0;
      enti = menuTop[menuLevel];;

      while ((rowi < UI_ROWS) &&
             (enti < entriesLen)) {
        UIMenuEntry *ent       = (UIMenuEntry *)pgm_read_ptr(&(entries[enti]));
        uint8_t      entType   =                pgm_read_byte(&(ent->entryType)) & 127;
        uint16_t     entAction =                pgm_read_word(&(ent->entryAction));
        char        *entText   = (char *)       pgm_read_ptr(&(ent->entryText));

        //  If the menu entry is hidden, don't show it.

        if (ent->showEntry() == false) {
          enti++;
          continue;
        }

        //  Othwewise, add the menu entry to the display list

        col = 0;

        if ((entType == UI_MENU_TYPE_SUBMENU) ||
            (entType == UI_MENU_TYPE_MODIFICATION_MENU)) {
          if (enti == menuPos[menuLevel] && activeAction != entAction)
            printCols[col++] = CHAR_SELECTOR;

          else if(activeAction == entAction)
            printCols[col++] = CHAR_SELECTED;

          else
            printCols[col++] = ' ';
        }

        parse(entText, false);

        //  Draw sub menu marker at the right side

        if (entType == UI_MENU_TYPE_SUBMENU) {
          while (col < UI_COLS - 1)
            printCols[col++] = ' ';

          printCols[UI_COLS - 1] = cARROWc;
          printCols[UI_COLS    ] = 0;
        }

        //  Copy the parsed string to our display cache.

        for (uint8_t c=0; c<MAX_COLS; c++)
          cache[rowi][c] = printCols[c];

        //  Move to the next row.

        rowi++;
        enti++;
      }
    }
  }


  else {
  }








  //  Blank out any remaning rows.

  while (rowi < UI_ROWS)
    cache[rowi++][0] = 0;



  //  compute line scrolling values

  uint8_t off0 = (shift <= 0 ? 0 : shift);
  uint8_t off[UI_ROWS] = {0};

#if 0
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

  //  And do something.

  Printer::toggleAnimation();
}





void
UIDisplay::pushMenu(const UIMenu *men, bool refresh) {

  //  If we're trying to go to the same menu, just refresh the page.

  if (men == menu[menuLevel]) {
    refreshPage();
    return;
  }

  //  If too many menus, gah, abort!

  if (menuLevel + 1 >= UI_MENU_MAXLEVEL)
    return;

  //  Push the new menu onto our list of menus.

  menuLevel++;

  menu   [menuLevel] = men;
  menuTop[menuLevel] = 0;
  menuPos[menuLevel] = 0;

  //  If the menu is a file selector, reload the directory and update.

  if (pgm_read_byte(&(men->menuType)) == UI_MENU_TYPE_FILE_SELECTOR) {
    updateSDFileCount();

    if (nFilesOnCard > 0)
      menuPos[menuLevel] = 1;  //  Top entry is 'back', default to the first real entry if one exists.
  }

  //  Otherwise, we're just a normal menu.

  else {
    UIMenuEntry **entries  = (UIMenuEntry**)pgm_read_ptr(&(men->entries));
    UIMenuEntry *entry     = (UIMenuEntry *)pgm_read_ptr(&(entries[0]));
    uint16_t     entAction                = pgm_read_word(&(entry->entryAction));

    if (entAction == UI_ACTION_BACK)
      menuPos[menuLevel] = 1;  //  Top entry is 'back', default to the first real entry.
  }

  //  Refresh if requested.

  if(refresh)
    refreshPage();
}




void UIDisplay::popMenu(bool refresh) {

  if (menuLevel > 0)
    menuLevel--;

  Printer::setAutomount(false);

  activeAction = 0;

  if (refresh)
    refreshPage();
}





void UIDisplay::showMessage(int id) {

  menuLevel = 0;

  Printer::setUIErrorMessage(true);

  switch(id) {
    case 1:
      pushMenu(&ui_msg_leveling_error, true);
      break;
    case 2:
      pushMenu(&ui_msg_defectsensor, true);
      break;
    case 3:
      pushMenu(&ui_msg_decoupled, true);
      break;
    case 4:
      pushMenu(&ui_msg_slipping, true);
      break;
  }
}










int
UIDisplay::okAction(bool allowMoves) {

  if (Printer::isUIErrorMessage()) {
    Printer::setUIErrorMessage(false);
    // return 0;
  }

  uiChirp();

  //  Enter the main menu if we're not at it.  This action is a button press from the status display.

  if (menuLevel == 0) {
    menuLevel   = 1;
    menuTop[1]  = 0;
    menuPos[1]  = 1;  //  Assume top item is back, default to first useful item.
    menu[1]     = &ui_menu_main;

    return(0);
  }

  //  Do something.

  const UIMenu   *men     = menu[menuLevel];
  uint8_t         mentype = pgm_read_byte(&(men->menuType)) & 63;

  UIMenuEntry   **entries;
  UIMenuEntry    *ent;
  unsigned char   entType;
  unsigned int    action;

  //  If a file selector, and 'back' was selected, go back.

  if ((mentype == UI_MENU_TYPE_FILE_SELECTOR) &&
      (menuPos[menuLevel] == 0)) {

    //  If the first level fileselector menu, return to the main menu.
    if ((cwd[0] == '/') && (cwd[1] == 0))
      return(executeAction(UI_ACTION_BACK, allowMoves));

    //  Otherwise, go up a directory.

    goDir(NULL);

    menuTop[menuLevel] = 0;
    menuPos[menuLevel] = 1;

    refreshPage();

    return(0);
  }

  //  If a file selector, but no SD card is active, return.

  if ((mentype == UI_MENU_TYPE_FILE_SELECTOR) &&
      (sd.sdactive == false))
    return(0);

  //  If a file selector, ....

  if (mentype == UI_MENU_TYPE_FILE_SELECTOR) {
    uint8_t filePos = menuPos[menuLevel] - 1;
    char    filename[LONG_FILENAME_LENGTH + 2];   //  Needs one extra byte for an appended '/', and nul terminator.

    getSDFilenameAt(filePos, filename);

    //  If a directory was selected, go into it.

    if (isDirname(filename)) {
      goDir(filename);

      menuTop[menuLevel] = 0;
      menuPos[menuLevel] = 1;

      refreshPage();

      return(0);
    }

    if (Printer::isAutomount()) {
      action = UI_ACTION_SD_PRINT;
    }

    else {
      men     = menu[menuLevel - 1];
      entries = (UIMenuEntry **)pgm_read_ptr(&(men->entries));
      ent     = (UIMenuEntry  *)pgm_read_ptr(&(entries[menuPos[menuLevel - 1]]));
      action  =                 pgm_read_word(&(ent->entryAction));
    }

    sd.file.close();
    sd.fat.chdir(cwd);

    switch (action) {
      case UI_ACTION_SD_PRINT:
        if (sd.selectFile(filename, false)) {
          sd.startPrint();
          uiAlert();
          menuLevel = 0;
        }
        break;
      case UI_ACTION_SD_DELETE:
        if(sd.sdactive) {
          sd.sdmode = 0;
          sd.file.close();
          if(sd.fat.remove(filename)) {
            Com::printFLN(Com::tFileDeleted);
            uiAlert();
            if(menuPos[menuLevel] > 0)
              menuPos[menuLevel]--;
            updateSDFileCount();
          } else {
            Com::printFLN(Com::tDeletionFailed);
          }
        }
        break;
    }

    return(0);
  }  //  UI_MENU_TYPE_FILE_SELECTOR



  entries = (UIMenuEntry**)pgm_read_word(&(men->entries));
  ent     = (UIMenuEntry *)pgm_read_word(&(entries[menuPos[menuLevel]]));
  entType =                pgm_read_byte(&(ent->entryType));
  action  =                pgm_read_word(&(ent->entryAction));

  //  If a modification menu, grab the action from the menu, do it, then go back to where we came from.

  if (mentype == UI_MENU_TYPE_MODIFICATION_MENU) {
    action = pgm_read_word(&(men->menuAction));

    finishAction(action);

    return(executeAction(UI_ACTION_BACK, true));
  }

  //  If a submenu, with entry type 4??

  if ((mentype == UI_MENU_TYPE_SUBMENU) &&
      (entType == 4)) { // Modify action
    if (activeAction) {
      finishAction(action);
      activeAction = 0;
    } else {
      activeAction = action;
    }

    return(0);
  }

  //  If a wizard, 

  if(mentype == UI_MENU_TYPE_WIZARD) {
    action = pgm_read_word(&(men->menuAction));

    switch(action) {
      case UI_ACTION_MESSAGE:
        popMenu(true);
        break;

      case UI_ACTION_STATE:
        break;

#if FEATURE_AUTOLEVEL & FEATURE_Z_PROBE
      case UI_ACTION_AUTOLEVEL2:
        uid.popMenu(false);
        uid.pushMenu(&ui_msg_calibrating_bed, true);
        runBedLeveling(2);
        uid.popMenu(true);
        break;
#endif

#if DISTORTION_CORRECTION
      case UI_ACTION_MEASURE_DISTORTION2:
        uid.pushMenu(&ui_msg_calibrating_bed, true);
        Printer::measureDistortion();
        uid.popMenu(true);
        break;
#endif

      default:
        break;
    }

    return(0);
  }

  //  If the entry is a submenu, go into the submenu.

  if (entType == UI_MENU_TYPE_SUBMENU) {
    pushMenu((UIMenu*)action, false);

    currHeaterForSetup = &(Extruder::current->tempControl);

    Printer::setMenuMode(MENU_MODE_FULL_PID, currHeaterForSetup->heatManager == 1);
    Printer::setMenuMode(MENU_MODE_DEADTIME, currHeaterForSetup->heatManager == 3);

    return(0);
  }

  //  

  if (entType == UI_MENU_TYPE_MODIFICATION_MENU) {
    return executeAction(action, allowMoves);
  }

  //  Otherwise, go back.

  return executeAction(UI_ACTION_BACK, allowMoves);
}





















// Gets calls from main tread only
void UIDisplay::slowAction(bool allowMoves) {
  millis_t time    = HAL::timeInMilliseconds();
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
      Com::writeToAll = true;

      nextPreviousAction(encodeChange, allowMoves);

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
        Com::writeToAll = true;
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

  if(menuLevel > 0 && ui_autoreturn_time < time && !uid.isSticky()) {
    lastSwitch = time;
    menuLevel = 0;
    activeAction = 0;
  }

  // prevent stepper/heater disable from timeout during active wizard

  if(uid.isWizardActive())
    previousMillisCmd = HAL::timeInMilliseconds();

  //  Refresh the display, every second for the main, and 0.8 seconds for non-main menus.

  if (menuLevel == 0) {
    if (time - lastRefresh >= 1000)
      refresh = 1;
  } else {
    if (time - lastRefresh >= 800)
      refresh = 1;
  }

  if (refresh) {
    if (menuLevel > 1 || Printer::isAutomount()) {
      shift++;

      if(shift + UI_COLS > MAX_COLS + 1)
        shift = -2;
    } else {
      shift = -2;
    }
  }

  if (refresh) {
    refreshPage();
    lastRefresh = time;
  }
}




void UIDisplay::mediumAction() {
}



// Gets called from inside an interrupt with interrupts allowed!
//
void
UIDisplay::fastAction(void) {

  InterruptProtectedBlock noInts;

  if ((flags & (UI_FLAG_KEY_TEST_RUNNING + UI_FLAG_SLOW_KEY_ACTION)) == 0) {
    flags |= UI_FLAG_KEY_TEST_RUNNING;

    uint16_t nextAction = 0;

    uiCheckKeys(nextAction);

    if (lastButtonAction != nextAction) {
      lastButtonStart = HAL::timeInMilliseconds();
      lastButtonAction = nextAction;

      flags |= UI_FLAG_FAST_KEY_ACTION;
    }

    flags &= ~UI_FLAG_KEY_TEST_RUNNING;
  }
}


