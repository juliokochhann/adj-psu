#include <stdbool.h>
#include <stdint.h>
#include "instruments.h"

/* Define meters */
Meter_t voltmeter = {VOLTMETER, 'U', 0, 0, 16.0};
Meter_t ammeter = {AMMETER, 'A', 0, 0, 2.048};
//Meter wattmeter = {WATTMETER, 'P', 0, 0, 32.0};
//Meter thermometer = {THERMOMETER, 'C', 0, 0, 100.0};