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


class Loop {

public:

    Loop(Stream& serial) :  sdp(serial, *this)  {
        sd.setPeriod(INTERRUPT_DELAY);
    }

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
    void enable();
    void disable();

    
    void signalSegmentComplete(); // Toggle pin to tell encoders that a segment is done
    void clearSegmentComplete();

    // Read encoder signal that a limit has been hit. 
    void limitChanged(); 

    void processMove(const uint8_t* buffer_, size_t size);

    int getLastSegNum(){
        return last_seg_num;
    }

    bool isEmpty(){ return sd.isEmpty(); }

    void printInfo();

    MessageProcessor &getMessageProcessor(){ return sdp;}
    

private:

    bool is_stopped = false;
    int last_seg_num = 0;
    bool running = false;

    // Incremented when the encoders signal that a limit has been changed. 
    int limitChanges = 0;

    Config config;
    AxisConfig axes_config[N_AXES];

    IntervalTimer segmentCompleteTimer; // Clears the segment complete signal pin. 

    StepDirectionStepper *steppers[N_AXES] = {0};

    StepDriver sd;

    CurrentState current_state;

public:  
    MessageProcessor sdp;



};