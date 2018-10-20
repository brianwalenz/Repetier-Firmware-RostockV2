#include "Repetier.h"
#include <SPI.h>


void setup()
{
    Printer::setup();
}

void loop()
{
    Commands::commandLoop();
}
