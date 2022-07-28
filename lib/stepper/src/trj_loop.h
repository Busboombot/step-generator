#pragma once
#include <Arduino.h>

#include <limits.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_sdstepper.h"
#include "trj_ringbuffer.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_planner.h"

#define ITR_DELAY 2

#define EEPROM_OFFSET 1

class Loop {

public:

    Loop(Stream& serial, unsigned int period) :  sdp(serial, *this), sd(StepDriver(period))  {}

    void setup();

    void loopOnce();

    void setConfig(Config* config);

    void setAxisConfig(AxisConfig* config);

    void stop();

    void start();

    inline Config& getConfig(){ return config;}

    inline StepDirectionStepper *getStepper(uint8_t n){ return steppers[n]; }

    void reset();

    void zero();

    void enable() {}
    void disable() {}

    void signalSegmentComplete();
    void clearSegmentComplete();

    void processMove(const uint8_t* buffer_, size_t size);

    int getLastSegNum(){
        return last_seg_num;
    }

    bool isEmpty(){ return sd.isEmpty(); }

    void printInfo();

private:

    Config config;
    AxisConfig axes_config[N_AXES];

    IntervalTimer segmentCompleteTimer; // Clears the segment complete signal pin. 

    MessageProcessor sdp;

    StepDirectionStepper *steppers[N_AXES];

    StepDriver sd;

    CurrentState current_state;
    
    int last_seg_num = 0;
    bool running = false;


};