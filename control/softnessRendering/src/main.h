#include <Arduino.h>
#include "Timing.cpp"           // Custom class to implement accurate timing functions
#include "pouch.h"
#include "softnessRendering.h"


double minPressure = MIN_PRESSURE, maxPressure = MAX_PRESSURE; // [kPa]
unsigned long now, period;
