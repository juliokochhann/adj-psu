#ifndef INSTRUMENTS_H
#define INSTRUMENTS_H

/* Enumerate instruments */
enum {
    VOLTMETER,
    AMMETER
    //THERMOMETER,
    //WATTMETER
};

typedef struct {
    const uint8_t meter;
    const char unit;
    float val_read;
    float val_set;
    const float range;
} Meter_t;

/* Forward declaration of meters */
extern Meter_t voltmeter;
extern Meter_t ammeter;
/* extern Meter_t wattmeter;
extern Meter_t thermometer; */

#endif