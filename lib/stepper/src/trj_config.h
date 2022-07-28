#pragma once

#include <Arduino.h>
#include <limits.h>
#include "trj_const.h"

// Configuration record for one axis
// 8 Bytes
struct AxisConfig {

    uint8_t axis;           // Axis number
    uint8_t step_pin;       // Step output, or quadture b
    uint8_t direction_pin;  // Direction output, or quadrature b
    uint8_t enable_pin;
    uint32_t v_max;
    uint32_t a_max;
};

// Main Configuration class
struct Config {

    uint8_t n_axes = 0;         // Number of axes
    uint8_t interrupt_delay=INTERRUPT_DELAY;    // How often interrupt is called, in microseconds
    uint8_t segment_complete_pin=0; // Pin on which to signal that a segment is complete
    uint8_t enable_active=0; // 
    bool debug_print = true;
    bool debug_tick = true;
};
