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
#ifndef RF_DISPLAY
#define RF_DISPLAY

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
#define bFOLD   6
#define cARROW  "\176"
#define cARROWc  0x3e

#define CHAR_SELECTOR '>'
#define CHAR_SELECTED '*'


#define UI_STATUS(status) uid.setStatusP(PSTR(status));
#define UI_STATUS_F(status) uid.setStatusP(status);
#define UI_STATUS_UPD(status) {uid.setStatusP(PSTR(status));uid.refreshPage();}
#define UI_STATUS_UPD_F(status) {uid.setStatusP(status);uid.refreshPage();}
#define UI_STATUS_RAM(status) uid.setStatus(status);
#define UI_STATUS_UPD_RAM(status) {uid.setStatus(status);uid.refreshPage();}

#define UI_ERROR(status) uid.setStatusP(PSTR(status),true);
#define UI_ERROR_P(status) uid.setStatusP(status,true);
#define UI_ERROR_UPD(status) {uid.setStatusP(PSTR(status),true);uid.refreshPage();}
#define UI_ERROR_RAM(status) uid.setStatus(status,true);
#define UI_ERROR_UPD_RAM(status) {uid.setStatus(status,true);uid.refreshPage();}

#define UI_CLEAR_STATUS {uid.statusMsg[0]=0;}
#define UI_RESET_MENU {uid.menuLevel=0;uid.refreshPage();}


#endif
