#pragma once

#include <Arduino.h>

extern Stream &debug_serial;


#define DEBUG_SET_1
#define DEBUG_CLEAR_1
#define DEBUG_TOG_1 

#define DEBUG_SET_2
#define DEBUG_CLEAR_2
#define DEBUG_TOG_2

#define DEBUG_SET_3
#define DEBUG_CLEAR_3
#define DEBUG_TOG_3

// This should be a macro, or something, 
// but I got lazy trying to figure out macro varargs
#define SER_PRINT_ENABLED false

void ser_printf(const char* fmt, ...);