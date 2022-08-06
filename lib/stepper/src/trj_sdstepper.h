#pragma once 

#include <Arduino.h>
#include <stdlib.h>     /* abs */
#include <math.h>       /* sqrt and fabs  */
#include <limits.h>     /* LONG_MAX */
#include <binary.h>
#include "trj_bithacks.h"
#include "trj_fastset.h"
#include "trj_debug.h"
#include "trj_config.h"  // for INTERRUPT_DELAY
#include "trj_stepdriver.h"


class StepDirectionStepper : public Stepper {
    
protected:
    
    bool pinState = true;
    
    int8_t direction = 0;
    int32_t position = 0;

    uint8_t enable_active = LOW;
    
public:
    
     inline StepDirectionStepper(AxisConfig config) :  Stepper(config) {
        pinMode(config.step_pin, config.step_output_mode);
        pinMode(config.direction_pin, config.direction_output_mode);
        pinMode(config.enable_pin, config.enable_output_mode); 
    }

    ~StepDirectionStepper(){}
    
    inline void writeStep(){
        digitalWriteFast(config.step_pin, config.step_high_value);
        position += direction;
    }
   
    inline void clearStep(){
        digitalWriteFast(config.step_pin, !config.step_high_value);
    }
    
    
    inline void enable(Direction dir){  
        setDirection(dir);
        enable();
    }
    
    inline void disable(){
        digitalWriteFast(config.enable_pin, !config.enable_high_value);
        setDirection(STOP);
    }
    
    inline void setDirection(Direction dir){

        if (dir == CW){
            digitalWriteFast(config.direction_pin, config.direction_high_value);
            direction = CW;
        } else if (dir == CCW){
            digitalWriteFast(config.direction_pin, !config.direction_high_value);
            direction = CCW;
        } else {
            direction = STOP;
        }
    }
    
    inline int getPosition(){ return position; }
    inline void setPosition(int64_t v){ position = v; }
     
private: 

    inline void enable(){
        digitalWriteFast(config.enable_pin, config.enable_high_value);
    }
     
};





