#pragma once

#include <Arduino.h>

extern Stream &debug_serial;


#define DEBUG_PIN_1 5
#define DEBUG_PIN_2 6

#ifdef DEBUG_PIN_1
extern int debug_state_1;
#define DEBUG_SET_1 digitalWriteFast(DEBUG_PIN_1, HIGH);
#define DEBUG_CLEAR_1 digitalWriteFast(DEBUG_PIN_1, LOW);
#define DEBUG_TOG_1 digitalWriteFast(DEBUG_PIN_1, debug_state_1=!debug_state_1);
#else
#define DEBUG_SET_1
#define DEBUG_CLEAR_1
#define DEBUG_TOG_1 
#endif

// This should be a macro, or something, 
// but I got lazy trying to figure out macro varargs
#define SER_PRINT_ENABLED false

void ser_printf(const char* fmt, ...);