/*
6 Axis step generator and trajectory planner

Reads packets data from the serial port that represents 
6 axis motion segments, determines the optimal acelleration and
velocity for each segment, and generates step and direction pulses

*/

#include <Arduino.h>
#include <limits.h>
#include "trj_loop.h"
#include "trj_debug.h"
#include "trj_const.h"

Stream &debug_serial = Serial1;

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;


// https://github.com/arduino/Arduino/issues/9413#issue-520272978
extern "C" {
  // This must exist to keep the linker happy but is never called.
  int _gettimeofday( struct timeval *tv, void *tzvp )
  {
    return 0;  // return non-zero for error
  } // end _gettimeofday()
}

Loop mainLoop(Serial);

/**
 * @brief Run-once setup
 * 
 */
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);

#ifdef  DEBUG_PIN_1
pinMode(DEBUG_PIN_1, OUTPUT);
#endif

  Serial.begin(115200); 
  Serial1.begin(115200);
  Serial1.println("Debug Starting");
  delay(200);

  mainLoop.setup();
}

void loop(){
  mainLoop.loopOnce();
}
