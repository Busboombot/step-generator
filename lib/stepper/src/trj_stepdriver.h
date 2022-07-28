#pragma once
#include <Arduino.h>

#include <limits.h>

#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_stepper.h"
#include "trj_ringbuffer.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_planner.h"

#define ITR_DELAY 2

#define EEPROM_OFFSET 1

class StepDriver {

public:

    StepDriver() : running(false) {}

    void stop() { running = false; }

    void start() { running = true; }

    void enable();

    void disable();

    void setAxisConfig(AxisConfig* as);

    inline StepInterface& getStepper(uint8_t n){ return *steppers[n]; }

    inline StepperState& getState(uint8_t n){ return state[n]; }

    int stepAll();

    inline int step(uint8_t n){   return state[n].step(getStepper(n)); }

    void update();

    void pushMoves(CommandCode code, int seq, Moves *m);

private:

    int n_axes = 6;

    bool running = false;
    bool enabled = false;
    bool finished_phase = false;

    Planner planner;
 
    uint32_t now;
  
    StepperState state[N_AXES];

    StepInterface* steppers[N_AXES] = {0,0,0,0,0,0};

    StepInterface* newStepper(AxisConfig* as);

    void feedSteppers();

    bool isEmpty(){ return planner.isEmpty(); }

    bool nextPhase();


};