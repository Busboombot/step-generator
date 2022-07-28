#pragma once
#include <Arduino.h>
#include <limits.h>

#include "trj_jointss.h"
#include "trj_planner_const.h" // For N_AXES
#include "trj_planner.h"

typedef enum
{
    CCW = -1,  ///< Clockwise
    STOP = 0,  ///< Clockwise
    CW  = 1   ///< Counter-Clockwise

} Direction;


// 2,000 rpm for a 1.8deg stepper is 400,000 steps per min, 7K steps 
// per sec. For a 10 ustep driver, 70KHz step pulse. 

#define TIMEBASE 1000000.0 // Microseconds

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

/**
 * @brief Track the current conditions for the queue and positions, as of the time
 * of ACKs and DONE messages. 
 * 
 */
struct CurrentState {
  int32_t queue_length = 0;
  uint32_t queue_time = 0;
  int32_t positions[N_AXES] = {0};
  int32_t planner_positions[N_AXES] = {0};
}; 



class Stepper {

protected:

    int8_t axis;
    int8_t direction = 0;
    int32_t position = 0;
    
public:

   
    Stepper(int8_t axis) : axis(axis){};
    void writeStep(){ position += direction;}
    void clearStep(){};
    void toggle(){};
    void enable(){};
    void enable(Direction dir){setDirection(dir);enable();}
    void disable() { setDirection(STOP); }
    void setDirection(Direction dir){direction = dir;};
    int getPosition(){return position;};
    void setPosition(int64_t v){position=v;};
     
};


class StepperState {
    
protected: 

    Direction direction=STOP; 
    uint32_t  stepsLeft=0;

    float delay_counter;
    float delay;
    float delay_inc;
    float v;
    float a;
    float t;   // Running time
    float t_s; // segment time, in sections
    float v_i; // Initial velocity

    int period; 

public:

    StepperState() {}
    
    StepperState(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int32_t period) :period(period) {
        setParams(segment_time, v0, v1, x, period);
    }
    
    inline uint32_t getStepsLeft(){
        return stepsLeft;
    }
    
    inline int getVelocity(){
        return v;
    }
    
    inline Direction getDirection() { return direction; }
    
    inline void setParams(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int32_t period){

        if (x > 0){
            direction = CW;
        } else if (x < 0){
            direction = CCW;
        } else {
            direction = STOP;
        }
        t = 0; // cumulative time
        delay = 0;
        delay_counter = 0;
    
        if(v1 + v0 != 0 and abs(x) != 0){
            stepsLeft = abs(x);
            // This is the Dave Austin algorithm, via the AccelStepper arduino library. 
            t_s =  fabs(2.0 * ((float)x)) / ( ((float)v1) + ((float)v0 ) );
            v_i = (float)v0;
            a = ((float)v1-(float)v0) / t_s;
        } else {
            stepsLeft = 0;
            t_s = 0;
            v_i = 0;
            a = 0;
        }
        
        delay_inc = ((float)period)/((float)TIMEBASE);
        
    }
   

    inline bool step(Stepper& stepper){


        if (stepsLeft == 0){
            return 0;
        }

        if (delay_counter >= delay){

            delay_counter -= delay;
            stepsLeft--;
            stepper.writeStep();
        }

        v = a * t + v_i;

        if(v != 0){
            delay = 1.0/v;
        } else {
            delay = 0;
        }
        delay_counter += delay_inc;
        t += delay_inc;

        return stepsLeft;
    }
        
};

class SubSegment {
public:
    uint16_t seq = 0;
    uint16_t code = 0;
    uint32_t segment_time = 0; // total segment time, in microseconds 
    uint32_t lastTime;
    StepperState axes[N_AXES];
public:
    SubSegment() {

    }
};

class StepDriver {

public:

    StepDriver(uint16_t period) :  period(period) {}
    ~StepDriver(){}

    void stop() { running = false; }

    void start() { running = true; }

    void enable();

    void disable();

    void setAxisConfig(AxisConfig* as);

    inline Stepper& getStepper(uint8_t n){ return *steppers[n]; }

    inline StepperState& getState(uint8_t n){ return state[n]; }

    int stepAll();

    inline int step(uint8_t n){   return state[n].step(getStepper(n)); }

    void update();

    void pushMoves(Move::MoveType move_type, int seq, uint32_t segment_time,  int32_t x[N_AXES]);

protected:

    int n_axes = 6;

    
    bool running = false;
    bool enabled = false;
    bool finished_phase = false;
    uint16_t period= 4; // Inter-interrupt time

    Planner planner;
 
    uint32_t now;
  
    StepperState state[N_AXES];

    Stepper* steppers[N_AXES] = {0,0,0,0,0,0};

    Stepper* newStepper(AxisConfig* as){
        return new Stepper(as->axis);
        //return new StepDirectionStepper(as->axis, as->step_pin, as->direction_pin, as->enable_pin);
    }

    void feedSteppers();

    bool isEmpty(){ return planner.isEmpty(); }

    bool nextPhase();

};

