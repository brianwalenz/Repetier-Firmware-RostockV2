#include "Repetier.h"
#include "HAL.h"
#include "Eeprom.h"
#include "Printer.h"
//#include "Commands.h"

#include "Extruder.h"
#include "temperatures.h"

//#include <SPI.h>


void
setup(void) {

  Serial.begin(250000);

  Com::printf(PSTR("\n"));
  Com::printf(PSTR("Booting.\n"));
  Com::printf(PSTR("\n"));

  // sets auto leveling in eeprom init
  EEPROM::init(); // Read settings from eeprom if wanted

  Com::printf("Free RAM: %d bytes.\n", hal.getFreeRAM());

  hal.setup();

  //hal.startWatchdog();

	endstops.setup();

  extruderTemp.initialize();
  bedTemp.initialize();
  layerFan.initialize();

  extruder.initialize();

  Printer::setup();

  uid.initialize();

  Com::printf(PSTR("\n"));
  Com::printf(PSTR("Booting Finished.  Free RAM: %d bytes.\n"), hal.getFreeRAM());
  Com::printf(PSTR("\n"));
}



void
loop(void) {
  Commands::commandLoop();
}
