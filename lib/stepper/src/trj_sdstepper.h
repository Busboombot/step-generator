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
    
    uint8_t stepPin;
    uint8_t directionPin;
    uint8_t enablePin;

    int8_t direction = 0;
    int32_t position = 0;

    uint8_t enable_active = LOW;
    

public:
    
    inline StepDirectionStepper(uint8_t axis_n, uint8_t stepPin, uint8_t directionPin, uint8_t enablePin): 
        Stepper(axis_n),
        stepPin(stepPin), directionPin(directionPin), enablePin(enablePin) {
        pinMode(stepPin, OUTPUT);
        pinMode(directionPin, OUTPUT);
        pinMode(enablePin, OUTPUT); 
    }
 
    ~StepDirectionStepper(){}
    
    inline void writeStep(){
        digitalWriteFast(stepPin, HIGH);
        position += direction;
    }
   
    inline void clearStep(){
        digitalWriteFast(stepPin, LOW);
    }
    
    
    inline void enable(Direction dir){  
        setDirection(dir);
        enable();
    }
    
    inline void disable(){
        digitalWriteFast(enablePin, !enable_active);// Active low
        setDirection(STOP);
    }
    
    inline void setDirection(Direction dir){

        if (dir == CW){
            fastSet(directionPin);
            direction = CW;
        } else if (dir == CCW){
            fastClear(directionPin);
            direction = CCW;
        } else {
            direction = STOP;
        }
    }
    
    inline int getPosition(){ return position; }
    inline void setPosition(int64_t v){ position = v; }
     
private: 

    inline void enable(){
        digitalWriteFast(enablePin, enable_active);  // Usually active low 
    }
     
};





