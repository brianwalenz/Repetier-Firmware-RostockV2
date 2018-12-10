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

#ifndef _UI_MENU_H
#define _UI_MENU_H

//  OLD print status page was
//    %Pn            FILE NAME
//    %Pp%%          PERCENT DONE
//    Layer %Pl/%PL  LAYER (not supported)
//    %os            STATUS

//  Nothing updates the printer progress:
//
//    if ((sd.sdactive) && (sd.sdmode))
//      Printer::progress = (float)sd.sdpos * 100.0 / (float)sd.filesize;


//  FIRST PAGE - IF NO CARD INSERTED and NOT PRINTING
//               IF CARD INSERTED, it shows files on the card; special case in code.
const char page01_01[] PROGMEM = "    FILE TO PRINT   ";
const char page01_02[] PROGMEM = "   <insert  card>   ";

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry01_01 PROGMEM = { page01_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry01_02 PROGMEM = { page01_02, entryType_displ,  0,                0,                    MODE_PRINTING | MODE_CARD_PRESENT };

//  SECOND PAGE - IF PRINTING
//  This is the default page to show just after selecting something to print.
const char page02_01[] PROGMEM = "      PRINTING      ";
const char page02_02[] PROGMEM = " %Pn";                  //  filename
const char page02_03[] PROGMEM = " %Pe   %Pr";            //  timeUsed  timeLeft
const char page02_04[] PROGMEM = "               %Pp%";   //            percPrinted
const char page02_05[] PROGMEM = " Speed:   %om%%     ";  //   %om 10 - 999   set to zero to pause
const char page02_06[] PROGMEM = " Flow:    %of%%     ";  //   %of 10 - 999
const char page02_07[] PROGMEM = " Lay Fan: %Fl%%   ";    //   %Fl  0 - 100 (fan speed)
const char page02_08[] PROGMEM = " -- ABORT  PRINT -- ";

//                               text       type              action                   hide-if-any-missing   hide-if-any-present
menuEntry entry02_01 PROGMEM = { page02_01, entryType_page,   ACT_MENU_CHANGE,         MODE_PRINTING,        0 };
menuEntry entry02_02 PROGMEM = { page02_02, entryType_displ,  0,                       MODE_PRINTING,        0 };
menuEntry entry02_03 PROGMEM = { page02_03, entryType_displ,  0,                       MODE_PRINTING,        0 };
menuEntry entry02_04 PROGMEM = { page02_04, entryType_displ,  0,                       MODE_PRINTING,        0 };
menuEntry entry02_05 PROGMEM = { page02_05, entryType_action, ACT_SPEED_CHANGE,        MODE_PRINTING,        0 };
menuEntry entry02_06 PROGMEM = { page02_06, entryType_action, ACT_FLOW_CHANGE,         MODE_PRINTING,        0 };
menuEntry entry02_07 PROGMEM = { page02_07, entryType_action, ACT_LAY_FAN_CHANGE,      MODE_PRINTING,        0 };
menuEntry entry02_08 PROGMEM = { page02_08, entryType_toggle, ACT_ABORT_PRINT,         MODE_PRINTING,        0 };

//  THIRD PAGE - TEMPERATURES
//  the preset temp is what?  the default preset temp or the current target temp?
const char page03_01[] PROGMEM = "    TEMPERATURES    ";   // 
const char page03_02[] PROGMEM = " EXT %ec/%Ec %hc";   //   %ec %Ec %hc    %hc - "off" or "##%" PWM percentage
const char page03_03[] PROGMEM = " BED %eb/%Eb %hb";   //   %eb %Eb %hb

//                               text       type              action                   hide-if-any-missing   hide-if-any-present
menuEntry entry03_01 PROGMEM = { page03_01, entryType_page,   ACT_MENU_CHANGE,         0,                    0 };
menuEntry entry03_02 PROGMEM = { page03_02, entryType_action, ACT_EXT_T_TARGET,        0,                    0 };
menuEntry entry03_03 PROGMEM = { page03_03, entryType_action, ACT_BED_T_TARGET,        0,                    0 };



//  FOURTH PAGE - IF NOT PRINTING
const char page04_01[] PROGMEM = "      POSITION      ";
const char page04_02[] PROGMEM = " Move to home       ";
const char page04_03[] PROGMEM = " E %x3 mm       ";  //  %x3
const char page04_04[] PROGMEM = " X %x0 mm       ";  //  %x0
const char page04_05[] PROGMEM = " Y %x1 mm       ";  //  %x1
const char page04_06[] PROGMEM = " Z %x2 mm       ";  //  %x2
const char page04_07[] PROGMEM = " Z %x2 mm (free)";  //  %x2 -- MAKE open mode limit to 0.01 move per click when z < 0.50mm
const char page04_08[] PROGMEM = " Set new Z=0.00     ";
const char page04_09[] PROGMEM = " Release Motors     ";

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry04_01 PROGMEM = { page04_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry04_02 PROGMEM = { page04_02, entryType_toggle, ACT_HOME,         0,                    MODE_PRINTING };
menuEntry entry04_03 PROGMEM = { page04_03, entryType_action, ACT_POS_E,        0,                    MODE_PRINTING };
menuEntry entry04_04 PROGMEM = { page04_04, entryType_action, ACT_POS_X,        0,                    MODE_PRINTING };
menuEntry entry04_05 PROGMEM = { page04_05, entryType_action, ACT_POS_Y,        0,                    MODE_PRINTING };
menuEntry entry04_06 PROGMEM = { page04_06, entryType_action, ACT_POS_Z,        0,                    MODE_PRINTING };
menuEntry entry04_07 PROGMEM = { page04_07, entryType_action, ACT_POS_Z_OPEN,   0,                    MODE_PRINTING };
menuEntry entry04_08 PROGMEM = { page04_08, entryType_toggle, ACT_POS_Z_SET,    0,                    MODE_PRINTING };
menuEntry entry04_09 PROGMEM = { page04_09, entryType_toggle, ACT_REL_MOTORS,   0,                    MODE_PRINTING };

//  FIFTH PAGE - IF PRINTING
const char page05_01[] PROGMEM = "      POSITION      ";
const char page05_02[] PROGMEM = " X %x0   E %x3";  //  %x0  %x3;
const char page05_03[] PROGMEM = " Y %x1           ";  //  %x1;
const char page05_04[] PROGMEM = " Z %x2    (in mm)";  //  %x2;

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry05_01 PROGMEM = { page05_01, entryType_page,   ACT_MENU_CHANGE,  MODE_PRINTING,        0 };
menuEntry entry05_02 PROGMEM = { page05_02, entryType_displ,  0,                MODE_PRINTING,        0 };
menuEntry entry05_03 PROGMEM = { page05_03, entryType_displ,  0,                MODE_PRINTING,        0 };
menuEntry entry05_04 PROGMEM = { page05_04, entryType_displ,  0,                MODE_PRINTING,        0 };


//  SIXTH PAGE
const char page06_01[] PROGMEM = "      SETTINGS      ";
const char page06_02[] PROGMEM = "      1             ";
const char page06_03[] PROGMEM = "       2            ";
const char page06_04[] PROGMEM = "        3           ";

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry06_01 PROGMEM = { page06_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry06_02 PROGMEM = { page06_02, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry06_03 PROGMEM = { page06_03, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry06_04 PROGMEM = { page06_04, entryType_displ,  0,                0,                    MODE_PRINTING };

//  SEVENTH PAGE
//  Printer::measureDistortion() was the entry point from okAction
const char page07_01[] PROGMEM = "       PROBING      ";
const char page07_02[] PROGMEM = "       1            ";
const char page07_03[] PROGMEM = "        2           ";
const char page07_04[] PROGMEM = "         3          ";

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry07_01 PROGMEM = { page07_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry07_02 PROGMEM = { page07_02, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry07_03 PROGMEM = { page07_03, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry07_04 PROGMEM = { page07_04, entryType_displ,  0,                0,                    MODE_PRINTING };

//  EIGHTH PAGE - PID TUNING
const char page08_01[] PROGMEM = "     HEATER PID     ";   // 
const char page08_02[] PROGMEM = " EXT %ec/%Ec %hc";   //   %ec %Ec %hc    %hc - "off" or "##%" PWM percentage
const char page08_03[] PROGMEM = " EXT P %Kp";
const char page08_04[] PROGMEM = " EXT I %Ki";
const char page08_05[] PROGMEM = " EXT D %Kd";
const char page08_06[] PROGMEM = " BED %eb/%Eb %hb";   //   %eb %Eb %hb
const char page08_07[] PROGMEM = " BED P %KP";
const char page08_08[] PROGMEM = " BED I %KI";
const char page08_09[] PROGMEM = " BED D %KD";

//                               text       type              action                   hide-if-any-missing   hide-if-any-present
menuEntry entry08_01 PROGMEM = { page08_01, entryType_page,   ACT_MENU_CHANGE,         0,                    0 };
menuEntry entry08_02 PROGMEM = { page08_02, entryType_action, ACT_EXT_T_TARGET,        0,                    0 };
menuEntry entry08_03 PROGMEM = { page08_03, entryType_action, ACT_EXT_PID_P,           0,                    0 };
menuEntry entry08_04 PROGMEM = { page08_04, entryType_action, ACT_EXT_PID_I,           0,                    0 };
menuEntry entry08_05 PROGMEM = { page08_05, entryType_action, ACT_EXT_PID_D,           0,                    0 };
menuEntry entry08_06 PROGMEM = { page08_06, entryType_action, ACT_BED_T_TARGET,        0,                    0 };
menuEntry entry08_07 PROGMEM = { page08_07, entryType_action, ACT_BED_PID_P,           0,                    0 };
menuEntry entry08_08 PROGMEM = { page08_08, entryType_action, ACT_BED_PID_I,           0,                    0 };
menuEntry entry08_09 PROGMEM = { page08_09, entryType_action, ACT_BED_PID_D,           0,                    0 };

//  NINTH PAGE - ACCELERATION SETTINGS
const char page09_01[] PROGMEM = "    ACCELERATION    ";
const char page09_02[] PROGMEM = " Print  %az         ";  // ACT_ACTION_PRINT_ACCEL_Z);
const char page09_03[] PROGMEM = " Move   %aZ         ";  // ACT_ACTION_MOVE_ACCEL_Z);
const char page09_04[] PROGMEM = " Jerk   %aj         ";  // ACT_ACTION_MAX_JERK);
const char page09_05[] PROGMEM = " ZJerk  %aJ         ";  // ACT_ACTION_MAX_JERK);

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry09_01 PROGMEM = { page09_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry09_02 PROGMEM = { page09_02, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry09_03 PROGMEM = { page09_03, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry09_04 PROGMEM = { page09_04, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry09_05 PROGMEM = { page09_05, entryType_displ,  0,                0,                    MODE_PRINTING };



//  TENTH PAGE - STATISTICS
//  --------------------
//  xxx.x days xx:xx:xx
//  xxxx:xx:xx hours
//  xxxx.xx m used
//  xxxx jobs  xxxx aborts
//  
const char page10_01[] PROGMEM = "      STATISTICS  9 ";
const char page10_02[] PROGMEM = " Total Printing";        //  " Total Printing     "
const char page10_03[] PROGMEM = " %Ut";                   //  " x days xx:xx:xx    "
const char page10_04[] PROGMEM = " %Uf";                   //  " xxx.xx m filament  "

//                               text       type              action            hide-if-any-missing   hide-if-any-present
menuEntry entry10_01 PROGMEM = { page10_01, entryType_page,   ACT_MENU_CHANGE,  0,                    MODE_PRINTING };
menuEntry entry10_02 PROGMEM = { page10_02, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry10_03 PROGMEM = { page10_03, entryType_displ,  0,                0,                    MODE_PRINTING };
menuEntry entry10_04 PROGMEM = { page10_04, entryType_displ,  0,                0,                    MODE_PRINTING };




menuEnArr entry01[2]  PROGMEM = { &entry01_01, &entry01_02 };
menuEnArr entry02[8]  PROGMEM = { &entry02_01, &entry02_02, &entry02_03, &entry02_04, &entry02_05, &entry02_06, &entry02_07, &entry02_08 };
menuEnArr entry03[3]  PROGMEM = { &entry03_01, &entry03_02, &entry03_03 };
menuEnArr entry04[9]  PROGMEM = { &entry04_01, &entry04_02, &entry04_03, &entry04_04, &entry04_05, &entry04_06, &entry04_07, &entry04_08, &entry04_09 };
menuEnArr entry05[4]  PROGMEM = { &entry05_01, &entry05_02, &entry05_03, &entry05_04 };
menuEnArr entry06[4]  PROGMEM = { &entry06_01, &entry06_02, &entry06_03, &entry06_04 };
menuEnArr entry07[4]  PROGMEM = { &entry07_01, &entry07_02, &entry07_03, &entry07_04 };
menuEnArr entry08[9]  PROGMEM = { &entry08_01, &entry08_02, &entry08_03, &entry08_04, &entry08_05, &entry08_06, &entry08_07, &entry08_08, &entry08_09 };
menuEnArr entry09[5]  PROGMEM = { &entry09_01, &entry09_02, &entry09_03, &entry09_04, &entry09_05 };
menuEnArr entry10[4]  PROGMEM = { &entry10_01, &entry10_02, &entry10_03, &entry10_04 };

menuPage  page01      PROGMEM = { menuType_select, 2, entry01 };
menuPage  page02      PROGMEM = { menuType_normal, 8, entry02 };
menuPage  page03      PROGMEM = { menuType_normal, 3, entry03 };
menuPage  page04      PROGMEM = { menuType_normal, 9, entry04 };
menuPage  page05      PROGMEM = { menuType_normal, 4, entry05 };
menuPage  page06      PROGMEM = { menuType_normal, 4, entry06 };
menuPage  page07      PROGMEM = { menuType_normal, 4, entry07 };
menuPage  page08      PROGMEM = { menuType_normal, 9, entry08 };
menuPage  page09      PROGMEM = { menuType_normal, 5, entry09 };
menuPage  page10      PROGMEM = { menuType_normal, 4, entry10 };


#define UI_NUM_PAGES   10

menuPage * const menuPages[UI_NUM_PAGES] PROGMEM = { &page01, &page02, &page03, &page04, &page05, &page06, &page07, &page08, &page09, &page10 };

menuPage   *menuPagePtr(uint8_t mm)   {  return((menuPage *) pgm_read_ptr(&menuPages[mm]));      };


#endif // __UI_MENU_H
