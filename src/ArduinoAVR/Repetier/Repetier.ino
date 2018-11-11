#include "Repetier.h"
#include "Printer.h"
#include "Commands.h"

#include <SPI.h>


void setup()
{
    Printer::setup();
}

void loop()
{
    Commands::commandLoop();
}
