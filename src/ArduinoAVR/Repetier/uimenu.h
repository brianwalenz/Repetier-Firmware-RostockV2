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

////////////////////////////////////////
//
//  Pages
//
//  If not in a menu, there can be multiple pages of read-only information.
//
//  Define pages with
//    UI_PAGE4(name,row1,row2,row3,row4);

//UI_PAGE4(ui_page1,
//         cTEMP "%ec/%Ec" cDEG "B%eB/%Eb" cDEG,
//         "Z:%x2  Buf : %oB",
//         "Mul: %om   E:%x4",
//         "%os")
UI_PAGE4(ui_page1,
         "               Temps",
         "     EXT: %ec/%Ec" cDEG,
         "     BED: %eB/%Eb" cDEG,
         "%os")

UI_PAGE4(ui_page2, "X:%x0 mm %dx%dX",                     "Y:%x1 mm %dy%dY",                   "Z:%x2 mm %dz%dZ",     "%os")
UI_PAGE4(ui_page3, " E:%ec/%Ec" cDEG "C" cARROW "%oC",    " B:%eb/%Eb" cDEG "C" cARROW "%ob",  "",                    "%os")
UI_PAGE4(ui_page4, "Total Printing Time",                 "%Ut",                               "Total Filament Used", "%Uf m")

#define UI_PAGES      {&ui_page1, &ui_page2, &ui_page3, &ui_page4}
#define UI_NUM_PAGES  4

////////////////////////////////////////
//
//  Menus
//
//  Each display line has a set of actions associated with it.
//  Turns of the encoder knob change the value ("up" or "down").
//  Pushes of the knob select 'OK' and return to the previous menu.
//
//  Menus are defined from leaves up, for some reason.

UI_MENU_ACTIONCOMMAND(ui_menu_back,  "<-" cUP, UI_ACTION_BACK)
UI_MENU_HEADLINE     (ui_menu_empty, "")

// Error menu
//UI_MENU_ACTION2_T(ui_menu_error, UI_ACTION_DUMMY, UI_TEXT_ERROR_ID, UI_TEXT_ERRORMSG_ID)

//
//  MESSAGES
//

  UI_WIZARD4(ui_msg_decoupled,         UI_ACTION_MESSAGE,             "Notification:",        "Heater decoupled",           "",     ">>> OK <<<")
UI_WIZARD4(ui_msg_defectsensor,      UI_ACTION_MESSAGE,             "Notification:",        "Temp. sensor defect",        "",     ">>> OK <<<")
UI_WIZARD4(ui_msg_slipping,          UI_ACTION_MESSAGE,             "Notification:",        "Filament slipping",          "",     ">>> OK <<<")
UI_WIZARD4(ui_msg_leveling_error,    UI_ACTION_MESSAGE,             "Notification:",        "Leveling error",             "",     ">>> OK <<<")
//UI_WIZARD4(ui_msg_calibration_error, UI_ACTION_MESSAGE,             "Notification:",        "Calibration Error",          "",     ">>> OK <<<")
UI_WIZARD4(ui_msg_clearbed,          UI_ACTION_MEASURE_DISTORTION2, "Make sure the heated", "bed is clear of any", "obstructions", ">>> OK <<<")
UI_WIZARD4(ui_msg_calibrating_bed,   UI_ACTION_STATE,               "",                     "Calibrating bed",                "",     "*** Please wait ***")
UI_WIZARD4(ui_msg_homing,            UI_ACTION_STATE,               "",                     "Homing...",                     "",     "*** Please wait ***")

//  Probing

#if 1
UI_MENU_HEADLINE      (ui_menu_mzp_head,    "Meas. Probe Height")
UI_MENU_CHANGEACTION  (ui_menu_mzp_realz,   "Real Z Pos:%W0mm",  UI_ACTION_MEASURE_ZP_REALZ)
UI_MENU_ACTIONCOMMAND (ui_menu_mzp_cont,    "Continue",          UI_ACTION_MEASURE_ZPROBE_HEIGHT2)
UI_MENU_ACTIONCOMMAND (ui_menu_mzp_close,   "Close",             UI_ACTION_BACK)

#define UI_MENU_MZP_ITEMS {                     \
    &ui_menu_mzp_head,                          \
      &ui_menu_mzp_realz,                       \
      &ui_menu_mzp_cont,                        \
      &ui_menu_mzp_close}

UI_STICKYMENU(ui_menu_mzp,UI_MENU_MZP_ITEMS,4)
#endif




// Bed leveling menu - FEATURE_SOFTWARE_LEVELLING
//
//  If FEATURE_SOFTWARE_LEVELLING, add &ui_menu_conf_level to to MENU_CONFIGURATION BELOW
//
//  Auto levelling - DISABLED
//  Add &ui_menu_autolevelbed and &ui_menu_toggle_autolevel to MENU_GENERAL if enabled (and MENU_SETUP?).

#if SOFTWARE_LEVELING
#define UI_MENU_LEVEL {                         \
    &ui_menu_back,                              \
      &ui_menu_set_p1,                          \
      &ui_menu_set_p2,                          \
      &ui_menu_set_p3,                          \
      &ui_menu_calculate_leveling,              \
      &ui_menu_go_xpos,                         \
      &ui_menu_go_ypos,                         \
      &ui_menu_go_zpos}

UI_MENU(ui_menu_level, UI_MENU_LEVEL, 8)
UI_MENU_SUBMENU(ui_menu_conf_level, "Level delta", ui_menu_level)
#endif







//  PRINTING CONTROL - pause and cancel.
//

//  SD CARD
//
//  SD card insert pulls up "sd card inserted" and shows file select menu.  UI_TEXT_SD_INSERTED_EN
//  rmemoving  - UI_TEXT_SD_REMOVED_EN - and drops back to first status page
//
//  Other actions:
//     UI_ACTION_STOP UI_ACTION_PAUSE UI_ACTION_CONTINUE
//     UI_ACTION_SD_DELETE

//                                                                                                          HIDE IF NOT ALL                        HIDE IF ANY
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_pause,            "Pause print",       UI_ACTION_SD_PAUSE,          MENU_MODE_PRINTING,                    MENU_MODE_PAUSED)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_continue,         "Continue print",    UI_ACTION_SD_CONTINUE,       MENU_MODE_PAUSED,                      0)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_stop,             "Stop print",        UI_ACTION_SD_STOP,           MENU_MODE_PRINTING | MENU_MODE_PAUSED, 0)

UI_MENU_FILESELECT(ui_menu_sd_fileselector, {&ui_menu_back}, 1)   //  Used in ui.cpp

#define UI_MENU_PRINTING_SUB {                  \
    &ui_menu_back,                              \
      &ui_menu_sd_pause,                        \
      &ui_menu_sd_continue,                     \
      &ui_menu_sd_stop }
UI_MENU(ui_menu_printing_sub, UI_MENU_PRINTING_SUB, 4)


//  These appear in the main menu.
//                                                                                             HIDE IF NOT ALL     HIDE IF ANY
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_printfile, "Print file...",     UI_ACTION_SD_PRINT,    MENU_MODE_MOUNTED,  MENU_MODE_PRINTING)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_sd_insert,    "<insert SD card>",  UI_ACTION_DUMMY,       0,                  MENU_MODE_MOUNTED)
UI_MENU_SUBMENU_FILTER      (ui_menu_printing,     "Job Control...",    ui_menu_printing_sub,  MENU_MODE_PRINTING, 0)



//  PREHEATING WHEN IDLE and TEMP CHANGE WHEN PRINTING
//
//  %e - current temperature
//  %p - preheat temperature
//  %E - target temperature (when printing)
//
//  Other commands:
//    UI_ACTION_FAN_OFF
//    UI_ACTION_FAN_25
//    UI_ACTION_FAN_50
//    UI_ACTION_FAN_75
//    UI_ACTION_FAN_FULL
//
//    UI_ACTION_FAN2SPEED - not sure what this is, was hoping it'd be the hot end fan, but it does nothing.

//                                                                                                        HIDE IF NOT ALL     HIDE IF ANY
UI_MENU_CHANGEACTION (ui_menu_preheat_bed,            "Bed:%eb/%pb " cDEG "C", UI_ACTION_BED_PREHEAT)
UI_MENU_CHANGEACTION (ui_menu_preheat_ext,            "Ext:%e0/%p0 " cDEG "C", UI_ACTION_EXT_PREHEAT)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_preheat_bed_on,  "Preheat BED",           UI_ACTION_BED_PREHEAT_ON,  0,                  MENU_MODE_BED_HEAT)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_preheat_bed_off, "Stop BED preheat",      UI_ACTION_BED_PREHEAT_OFF, MENU_MODE_BED_HEAT, 0)

UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_preheat_ext_on,  "Preheat EXT",           UI_ACTION_EXT_PREHEAT_ON,  0,                  MENU_MODE_EXT_HEAT)
UI_MENU_ACTIONCOMMAND_FILTER(ui_menu_preheat_ext_off, "Stop EXT preheat",      UI_ACTION_EXT_PREHEAT_OFF, MENU_MODE_EXT_HEAT, 0)

UI_MENU_ACTIONCOMMAND(ui_menu_preheat_cooldown,       "Turn Heaters Off",      UI_ACTION_COOLDOWN)

#define UI_MENU_PREHEAT_SUB {                   \
    &ui_menu_preheat_bed,                       \
      &ui_menu_preheat_ext,                     \
      &ui_menu_preheat_bed_on,                  \
      &ui_menu_preheat_bed_off,                 \
      &ui_menu_preheat_ext_on,                  \
      &ui_menu_preheat_ext_off,                 \
      &ui_menu_preheat_cooldown,                \
      &ui_menu_back}
UI_MENU(ui_menu_preheat_sub, UI_MENU_PREHEAT_SUB, 8)

UI_MENU_CHANGEACTION  (ui_menu_bed_temp,       "BED:%eb/%Eb " cDEG "C", UI_ACTION_BED_TARGET)
UI_MENU_CHANGEACTION  (ui_menu_ext_temp,       "EXT:%e0/%E0 " cDEG "C", UI_ACTION_EXT_TARGET)
UI_MENU_CHANGEACTION  (ui_menu_fan_fanspeed,   "LAY fan:%Fs%%",         UI_ACTION_FANSPEED)
UI_MENU_ACTIONCOMMAND (ui_menu_fan_ignoreM106, "LAY fan forced %Fi",    UI_ACTION_IGNORE_M106)    // %Fi shows a checkbox with status of the flag

#define UI_MENU_TEMPERATURES_SUB {              \
    &ui_menu_back,                              \
      &ui_menu_bed_temp,                        \
      &ui_menu_ext_temp,                        \
      &ui_menu_fan_fanspeed,                    \
      &ui_menu_fan_ignoreM106}
UI_MENU(ui_menu_temperatures_sub, UI_MENU_TEMPERATURES_SUB, 5)

UI_MENU_CHANGEACTION        (ui_menu_quick_speedmultiply, " Speed  %om%%", UI_ACTION_FEEDRATE_MULTIPLY)
UI_MENU_CHANGEACTION        (ui_menu_quick_flowmultiply,  " Flow   %of%%", UI_ACTION_FLOWRATE_MULTIPLY)

#define UI_MENU_SPEED_SUB {                     \
    &ui_menu_back,                              \
      &ui_menu_quick_speedmultiply,             \
      &ui_menu_quick_flowmultiply }
UI_MENU(ui_menu_speed_sub, UI_MENU_SPEED_SUB, 3)

//  These appear in the main menu.
//                                                                                         HIDE IF NOT ALL     HIDE IF ANY
UI_MENU_SUBMENU_FILTER(ui_menu_preheat,      "Preheat...",      ui_menu_preheat_sub,       0,                  MENU_MODE_PRINTING)
UI_MENU_SUBMENU_FILTER(ui_menu_temperatures, "Temperatures...", ui_menu_temperatures_sub,  MENU_MODE_PRINTING, 0)
UI_MENU_SUBMENU_FILTER(ui_menu_speed,        "Speed...",        ui_menu_speed_sub,         MENU_MODE_PRINTING, 0)




//
//  Positioning.
//
//  The 'fast' variants are unused, but need to be defined.  See ui.cpp.
//
//  If FEATURE_Z_PROBE, add &ui_menu_measure_zprobe_height to the z height calibration.
//

UI_MENU_ACTIONCOMMAND(ui_menu_home_all,        "Home.",           UI_ACTION_HOME_ALL)
UI_MENU_CHANGEACTION (ui_menu_go_epos,         " E %x3 mm",        UI_ACTION_EPOSITION)
UI_MENU_CHANGEACTION (ui_menu_go_xpos,         " X %x0 mm",        UI_ACTION_XPOSITION)
UI_MENU_CHANGEACTION (ui_menu_go_ypos,         " Y %x1 mm",        UI_ACTION_YPOSITION)
UI_MENU_CHANGEACTION (ui_menu_go_zpos,         " Z %x2 mm",        UI_ACTION_ZPOSITION)
UI_MENU_CHANGEACTION (ui_menu_go_zpos_notest,  " Z %x2 mm (free)", UI_ACTION_ZPOSITION_NOTEST)
UI_MENU_ACTIONCOMMAND(ui_menu_set_z_origin,    "Set new Z=0.00",  UI_ACTION_SET_MEASURED_ORIGIN)
UI_MENU_ACTIONCOMMAND(ui_menu_release_stepper, "Release Motors",  UI_ACTION_DISABLE_STEPPER)

UI_MENU_ACTIONCOMMAND(ui_menu_measure_zprobe_height, "Meas. Probe Height", UI_ACTION_MEASURE_ZPROBE_HEIGHT)

#define UI_MENU_POSITION_SUB {                  \
    &ui_menu_back,                              \
      &ui_menu_home_all,                        \
      &ui_menu_go_epos,                         \
      &ui_menu_go_xpos,                         \
      &ui_menu_go_ypos,                         \
      &ui_menu_go_zpos,                         \
      &ui_menu_go_zpos_notest,                  \
      &ui_menu_set_z_origin,                    \
      &ui_menu_release_stepper }

UI_MENU               (ui_menu_position_sub, UI_MENU_POSITION_SUB, 9)
UI_MENU_SUBMENU_FILTER(ui_menu_position, "Position...", ui_menu_position_sub, 0, MENU_MODE_PRINTING)







UI_MENU_ACTIONCOMMAND(ui_menu_conf_to_eeprom,   "Store to EEPROM",  UI_ACTION_STORE_EEPROM)
UI_MENU_ACTIONCOMMAND(ui_menu_conf_from_eeprom, "Load from EEPROM", UI_ACTION_LOAD_EEPROM)

UI_MENU_ACTION4(ui_menu_eeprom_saved,  UI_ACTION_DUMMY, "Configuration", "stored in EEPROM", "", "");
UI_MENU_ACTION4(ui_menu_eeprom_loaded, UI_ACTION_DUMMY, "Configuration", "loaded from EEPROM", "", "");



//
//  SETTINGS
//
//  UI_ACTION_STEPPER_INACTIVE -- set inactive time to disable stepper motors
//  UI_ACTION_MAX_INACTIVE     -- set inactive time to disable as much as possible
//



//  ACCELERATION

UI_MENU_CHANGEACTION(ui_menu_accel_printz,  "Print  %az", UI_ACTION_PRINT_ACCEL_Z)
UI_MENU_CHANGEACTION(ui_menu_accel_travelz, "Move   %aZ", UI_ACTION_MOVE_ACCEL_Z)
UI_MENU_CHANGEACTION(ui_menu_accel_jerk,    "Jerk   %aj", UI_ACTION_MAX_JERK)
UI_MENU_CHANGEACTION(ui_menu_accel_z_jerk,  "ZJerk  %aJ", UI_ACTION_MAX_JERK)

#define UI_MENU_SETTINGS_ACCEL_SUB {            \
    &ui_menu_back,                              \
      &ui_menu_accel_printz,                    \
      &ui_menu_accel_travelz,                   \
      &ui_menu_accel_jerk,                      \
      &ui_menu_accel_z_jerk}
UI_MENU(ui_menu_settings_accel_sub, UI_MENU_SETTINGS_ACCEL_SUB, 4)

//  FEED RATE

UI_MENU_CHANGEACTION(ui_menu_feedrate_maxz,  "Max %fz",  UI_ACTION_MAX_FEEDRATE_Z)
UI_MENU_CHANGEACTION(ui_menu_feedrate_homez, "Home %fZ", UI_ACTION_HOMING_FEEDRATE_Z)

#define UI_MENU_SETTINGS_FEED_SUB {             \
    &ui_menu_back,                              \
      &ui_menu_feedrate_maxz,                   \
      &ui_menu_feedrate_homez}
UI_MENU(ui_menu_settings_feed_sub, UI_MENU_SETTINGS_FEED_SUB, 3)

//  EXTRUDER SETTINGS

UI_MENU_CHANGEACTION(ui_menu_cext_steps,          "Steps/MM:%Se",              UI_ACTION_EXTR_STEPS)
UI_MENU_CHANGEACTION(ui_menu_cext_start_feedrate, "Start FR:%Xf",              UI_ACTION_EXTR_START_FEEDRATE)
UI_MENU_CHANGEACTION(ui_menu_cext_max_feedrate,   "Max FR:%XF",                UI_ACTION_EXTR_MAX_FEEDRATE)
UI_MENU_CHANGEACTION(ui_menu_cext_acceleration,   "Accel:%XA",                 UI_ACTION_EXTR_ACCELERATION)
UI_MENU_CHANGEACTION(ui_menu_cext_watch_period,   "Stab.Time:%Xw",             UI_ACTION_EXTR_WATCH_PERIOD)
UI_MENU_CHANGEACTION(ui_menu_ext_wait_units,      "Wait retr.:%XU mm",         UI_ACTION_EXTR_WAIT_RETRACT_UNITS)
UI_MENU_CHANGEACTION(ui_menu_ext_wait_temp,       "Wait temp. %XT" cDEG "C",   UI_ACTION_EXTR_WAIT_RETRACT_TEMP)
UI_MENU_CHANGEACTION(ui_menu_cext_advancel,       "Advance lin:%Xl",           UI_ACTION_ADVANCE_L)   //  "Advance method" to decrease blobs at points where acceleartion changes
UI_MENU_CHANGEACTION(ui_menu_cext_advancek,       "Advance quad:%Xa",          UI_ACTION_ADVANCE_K)

#define UI_MENU_SETTINGS_EXT_SUB {              \
    &ui_menu_back,                              \
      &ui_menu_cext_steps,                      \
      &ui_menu_cext_start_feedrate,             \
      &ui_menu_cext_max_feedrate,               \
      &ui_menu_cext_acceleration,               \
      &ui_menu_cext_watch_period,               \
      &ui_menu_ext_wait_units,                  \
      &ui_menu_ext_wait_temp,                   \
      &ui_menu_cext_advancel,                   \
      &ui_menu_cext_advancek }
UI_MENU(ui_menu_settings_ext_sub, UI_MENU_SETTINGS_EXT_SUB, 10)



UI_MENU_CHANGEACTION       (ui_menu_cext_manager,  "Control:%Xh",       UI_ACTION_EXTR_HEATMANAGER)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_pgain,    "PID P:%Xp",         UI_ACTION_PID_PGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_igain,    "PID I:%Xi",         UI_ACTION_PID_IGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_dgain,    "PID D:%Xd",         UI_ACTION_PID_DGAIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_dmin,     "Drive Min:%Xm",     UI_ACTION_DRIVE_MIN,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_dmax,     "Drive Max:%XM",     UI_ACTION_DRIVE_MAX,  MENU_MODE_FULL_PID, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_pgain_dt, "Deadtime:%Xp",      UI_ACTION_PID_PGAIN,  MENU_MODE_DEADTIME, 0)
UI_MENU_CHANGEACTION_FILTER(ui_menu_cext_dmax_dt,  "Control DPWM:%XM",  UI_ACTION_DRIVE_MAX,  MENU_MODE_DEADTIME, 0)
UI_MENU_CHANGEACTION       (ui_menu_cext_pmax,     "PID PMax:%XD",      UI_ACTION_PID_MAX)

//  EXT TEMP SETTINGS
#define UI_MENU_SETTINGS_EXT_TEMP_SUB {         \
    &ui_menu_back,                              \
      &ui_menu_cext_manager,                    \
      &ui_menu_cext_pgain,                      \
      &ui_menu_cext_igain,                      \
      &ui_menu_cext_dgain,                      \
      &ui_menu_cext_dmin,                       \
      &ui_menu_cext_dmax,                       \
      &ui_menu_cext_pgain_dt,                   \
      &ui_menu_cext_dmax_dt,                    \
      &ui_menu_cext_pmax}
UI_MENU(ui_menu_settings_ext_temp_sub, UI_MENU_SETTINGS_EXT_TEMP_SUB, 10)

//  BED TEMP SETTINGS
#define UI_MENU_SETTINGS_BED_TEMP_SUB {         \
    &ui_menu_back,                              \
      &ui_menu_cext_manager,                    \
      &ui_menu_cext_pgain,                      \
      &ui_menu_cext_igain,                      \
      &ui_menu_cext_dgain,                      \
      &ui_menu_cext_dmin,                       \
      &ui_menu_cext_dmax,                       \
      &ui_menu_cext_pgain_dt,                   \
      &ui_menu_cext_dmax_dt,                    \
      &ui_menu_cext_pmax}
UI_MENU(ui_menu_settings_bed_temp_sub, UI_MENU_SETTINGS_BED_TEMP_SUB, 10)



UI_MENU_CHANGEACTION(ui_menu_general_baud,        "Baud: %oc",       UI_ACTION_BAUDRATE)
UI_MENU_SUBMENU     (ui_menu_settings_accel,      "Acceleration...", ui_menu_settings_accel_sub)
UI_MENU_SUBMENU     (ui_menu_settings_feed,       "Feedrate...",     ui_menu_settings_feed_sub)
UI_MENU_SUBMENU     (ui_menu_settings_ext,        "Extruder...",     ui_menu_settings_ext_sub)
UI_MENU_SUBMENU     (ui_menu_settings_ext_temp,   "EXT Temp Sens...",   ui_menu_settings_ext_temp_sub)
UI_MENU_SUBMENU     (ui_menu_settings_bed_temp,   "BED Temp Sens...",   ui_menu_settings_bed_temp_sub)

#define UI_MENU_SETTINGS_SUB {                  \
    &ui_menu_back,                              \
      &ui_menu_general_baud,                    \
      &ui_menu_settings_accel,                  \
      &ui_menu_settings_feed,                   \
      &ui_menu_settings_ext,                    \
      &ui_menu_settings_ext_temp,               \
      &ui_menu_settings_bed_temp,               \
      &ui_menu_conf_to_eeprom,                  \
      &ui_menu_conf_from_eeprom}
UI_MENU(ui_menu_settings_sub, UI_MENU_SETTINGS_SUB, 9)
UI_MENU_SUBMENU_FILTER(ui_menu_settings, "Settings...", ui_menu_settings_sub, 0, MENU_MODE_PRINTING)



//
//  DEBUGGING
//

UI_MENU_ACTIONCOMMAND(ui_menu_quick_debug,   "Write Debug", UI_ACTION_WRITE_DEBUG)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_echo,    "Echo    %do", UI_ACTION_DEBUG_ECHO)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_info,    "Info    %di", UI_ACTION_DEBUG_INFO)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_error,   "Errors  %de", UI_ACTION_DEBUG_ERROR)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_dryrun,  "Dry run %dd", UI_ACTION_DEBUG_DRYRUN)
UI_MENU_ACTIONCOMMAND(ui_menu_debug_endstop, "EndStop %dp", UI_ACTION_DEBUG_ENDSTOP)

#define UI_MENU_DEBUGGING_SUB {                 \
    &ui_menu_back,                              \
      &ui_menu_debug_echo,                      \
      &ui_menu_debug_info,                      \
      &ui_menu_debug_error,                     \
      &ui_menu_debug_dryrun,                    \
      &ui_menu_debug_endstop}

UI_MENU(ui_menu_debugging_sub, UI_MENU_DEBUGGING_SUB, 6)
UI_MENU_SUBMENU(ui_menu_debugging, "Debugging...", ui_menu_debugging_sub)




#define UI_MENU_MAIN {                          \
    &ui_menu_back,                              \
                                                \
      &ui_menu_sd_printfile,                    \
      &ui_menu_sd_insert,                       \
      &ui_menu_printing,                        \
                                                \
      &ui_menu_preheat,                         \
      &ui_menu_temperatures,                    \
      &ui_menu_speed,                           \
                                                \
      &ui_menu_position,                        \
                                                \
      &ui_menu_settings,                        \
      &ui_menu_debugging  }

UI_MENU(ui_menu_main, UI_MENU_MAIN, 10)



#endif // __UI_MENU_H
