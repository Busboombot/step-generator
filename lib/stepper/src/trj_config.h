#pragma once

#include <Arduino.h>
#include <limits.h>
#include "trj_const.h"



// Main Configuration class, 68 bytes
struct Config {

    uint8_t n_axes = 0;         // Number of axes
    uint8_t interrupt_delay=INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
    uint8_t segment_complete_pin=0; // Pin on which to signal that a segment is complete
    bool debug_print = true;
    bool debug_tick = true;
};
