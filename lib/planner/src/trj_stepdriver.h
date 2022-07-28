#pragma once
#include <Arduino.h>
#include <limits.h>

#include "trj_jointss.h"
#include "trj_planner_const.h" // For N_AXES
#include "trj_planner.h"
#include "trj_util.h"


typedef enum
{
    CCW = -1,  ///< Clockwise
    STOP = 0,  ///< Clockwise
    CW  = 1   ///< Counter-Clockwise

} Direction;


// 2,000 rpm for a 1.8deg stepper is 400,000 steps per min, 7K steps 
// per sec. For a 10 ustep driver, 70KHz step pulse. 


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
    bool enabled = false;
    Direction direction = Direction::STOP;
     
public:

    Stepper() : axis(0), enabled(false){};
    Stepper(int8_t axis) : axis(axis), enabled(false){};
    void writeStep(){ }
    void clearStep(){};
    void toggle(){};
    void enable(){enabled = true;};
    void enable(Direction dir){setDirection(dir);enable();}
    void disable() { setDirection(STOP); enabled = false;}
    void setDirection(Direction dir){direction = dir;};

};

class StepperState {
    
protected: 

    Direction direction=STOP; 
    uint32_t  stepsLeft=0;
    long position = 0;

    int period; 

    float delay_counter;
    float delay;
    float delay_inc;
    float v;
    float a;
    float t;   // Running time
    float t_s; // segment time, in sections
    float v_i; // Initial velocity


public:

    StepperState() {}
    
    
    inline uint32_t getStepsLeft(){ return stepsLeft;}
    
    inline int getVelocity(){ return v;}
    
    inline Direction getDirection() { return direction; }

    inline long getPosition(){ return position; }

    inline void setPosition(long p){ position = p; }
    
    void setParams(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int period);

    int step(Stepper *stepper);

   

};

class SubSegment {
public:
    uint16_t seq = 0;
    uint16_t code = 0;
    uint32_t segment_time = 0; // total segment time, in microseconds 
    uint32_t lastTime;
    StepperState axes[N_AXES];
public:
    SubSegment() {}
};


class StepDriver {

public:

    StepDriver(uint16_t period) :  period(period){}
    ~StepDriver(){}

    void stop() { running = false; }

    void start() { running = true; }

    void enable();

    void disable();

    void setAxisConfig(uint8_t axis, unsigned int v_max, unsigned int a_max);

    void setStepper(uint8_t axis, Stepper* stepper){steppers[axis] = stepper;}

    inline StepperState& getState(uint8_t n){ return state[n]; }
    inline Planner& getPlanner(){ return planner;}

    inline int step(uint8_t n){ return state[n].step(steppers[n]); }

    inline void clear(uint8_t n){
        if (steppers[n] != 0){
            steppers[n]->clearStep(); 
        }
    }

    int update(); // Run all of the stepping and state updates

    int loadNextPhase();

    bool isEmpty(){ return planner.isEmpty(); }

    void push(Move m);


    double sincePhaseStart();

    void clear(){planner.clear();}

    void zero(){
        for (StepperState ss : state){
            ss.setPosition(0);
        }
    }

    void setNAxes(int n){ n_axes = n; }



protected:

    int n_axes = 0; // Gets increased in setAxisConfig

    bool running = false;
    bool enabled = false;
    bool phaseIsActive = false;

    long nextUpdate; // microseconds since start of current phase for next update to steppers
    long nextClear; // microsecond until next clear of step pins. 

    uint16_t period; // Inter-interrupt time

    Planner planner;
    uint32_t now;
  
    StepperState state[N_AXES];

    Stepper *steppers[N_AXES] = {nullptr};

    bool nextPhase();

 private:
        friend std::ostream & operator<<(std::ostream &os, const StepDriver& sd);
       

};

