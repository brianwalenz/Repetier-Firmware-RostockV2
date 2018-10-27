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


extern UIDisplay uid;



void
UIDisplay::finishAction(uint16_t action) {

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
}





// Actions are events from user input. Depending on the current state, each
// action can behave differently. Other actions do always the same like home, disable extruder etc.

uint16_t
UIDisplay::executeAction(uint16_t action, bool allowMoves) {
  int ret = 0;

  Com::print("executeAction action=");
  Com::print(action);
  Com::print("\n");

  if      (action == ACT_OK) {
    okAction(allowMoves);
  }

  else if (action == ACT_KILL) {
    Commands::emergencyStop();
  }

  return(ret);
}

