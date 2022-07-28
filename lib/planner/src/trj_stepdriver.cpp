#include <limits>
#include <chrono>

#include "trj_stepdriver.h"
#include "trj_util.h"
#include "trj_debug.h"


CurrentState current_state;
auto lastPhaseTime = steadyClock::now();

void StepDriver::setAxisConfig(uint8_t axis, unsigned int v_max, unsigned int a_max){

  if(axis>=n_axes){
    n_axes = axis+1;
    planner.setNJoints(n_axes);
  }

  Joint joint(axis,static_cast< float >(v_max), static_cast< float >(a_max));
  planner.setJoint(joint);
 
}

void StepDriver::enable(){
  enabled = true;
  switch (n_axes){
      case 6: steppers[5]->enable(state[5].getDirection());
      case 5: steppers[4]->enable(state[4].getDirection()); 
      case 4: steppers[3]->enable(state[3].getDirection()); 
      case 3: steppers[2]->enable(state[2].getDirection()); 
      case 2: steppers[1]->enable(state[1].getDirection());
      case 1: steppers[0]->enable(state[0].getDirection()); 
  }
}

void StepDriver::disable(){
  enabled = false;
  switch (n_axes){
      case 6: steppers[5]->disable();
      case 5: steppers[4]->disable();
      case 4: steppers[3]->disable();
      case 3: steppers[2]->disable();
      case 2: steppers[1]->disable();
      case 1: steppers[0]->disable();
  }
}

void StepDriver::push(Move move){

    for (int axis = 0; axis < n_axes; axis++){
      if(move.move_type == Move::MoveType::absolute){
        current_state.planner_positions[axis] = move.x[axis];
      } else {
        current_state.planner_positions[axis] += move.x[axis];
      }
    }
    
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    planner.push(move);

}

int StepDriver::loadNextPhase(){
  const PhaseJoints&  pj = planner.getNextPhase();
  cout << pj << endl;
  int active_axes = 0;

  for (int axis = 0; axis < n_axes; axis++){
    const JointSubSegment &jss = pj.moves[axis];
    if(jss.x != 0){
      active_axes++;
      state[axis].setParams(jss.t, jss.v_0, jss.v_1, jss.x, period); // Omits config for INTERRUPT_DELAY
      if (steppers[axis] != nullptr){
        steppers[axis]->enable(state[axis].getDirection());
      }
    } else {
      state[axis].setParams(0, 0, 0, 0, 0);
    }
  }

  return active_axes;
}


int StepDriver::update(){

  static unsigned long  stepsLeft = 0;

  double t = sincePhaseStart(); // microseconds since start of the current phase

  return 0 ;

  if(!phaseIsActive & !isEmpty() ){
    int active_axes = loadNextPhase();
    
    if( active_axes > 0){
        phaseIsActive = true;
        lastPhaseTime = steadyClock::now();
        t = sincePhaseStart();
        nextUpdate = t ;
        nextClear = 0;

    }
  }


  if(phaseIsActive and (t >= nextUpdate) ){
    nextUpdate += period;
    nextClear = nextUpdate + period/2;
    stepsLeft = 0;

    switch (n_axes) {
      case 6: stepsLeft += step(5);
      case 5: stepsLeft += step(4);
      case 4: stepsLeft += step(3);
      case 3: stepsLeft += step(2);
      case 2: stepsLeft += step(1);
      case 1: stepsLeft += step(0);
      case 0: ;
    }

    if(stepsLeft == 0){
      phaseIsActive = false;
    }
  }

  if (t >= nextClear){
    switch (n_axes) {
      case 6: clear(5);
      case 5: clear(4);
      case 4: clear(3);
      case 3: clear(2);
      case 2: clear(1);
      case 1: clear(0);
      case 0: ;
    }
    nextClear = 0;
  }

  return stepsLeft;

}

double StepDriver::sincePhaseStart(){ 
  return usince(lastPhaseTime); 
}



void StepperState::setParams(uint32_t segment_time, uint32_t v0, uint32_t v1, int32_t x, int period){

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
            // This is from the Dave Austin algorithm, via the AccelStepper arduino library. 
            t_s =  fabs(2.0 * ((float)x)) / ( ((float)v1) + ((float)v0 ) );
            v_i = (float)v0;
            a = ((float)v1-(float)v0) / t_s;
        } else {
            stepsLeft = 0;
            t_s = 0;
            v_i = 0;
            a = 0;
        }
        
        delay_inc = ((float)period) / ((float)TIMEBASE);
       
    }

  int StepperState::step(Stepper *stepper){


        if (stepsLeft == 0){
            return 0;
        }

        if (delay_counter >= delay){

            delay_counter -= delay;
            stepsLeft--;
            if (stepper != nullptr){
              stepper->writeStep();
            }
          
            position += direction;
        }

        // Note that t is always increasing, even if there isn't a step, so 
        // v is always changing. If the first step from a v0 == 0 is very large, delay can be
        // so large that the first step may not be schedule for several seconds, but if this function
        // is called regularly, t will increase, and v will increase, so delay will decrease. After
        // a couple of calls, delay will be reasonable again. 
        //
        // Alternatively, we could set a minimum velocity, but that is probably not necessary.

        v = a * t + v_i;

        if(v != 0){
            delay = 1.0/abs(v);

        } else {
            delay = 0;
        }

        delay_counter += delay_inc;
        t += delay_inc;

        
        return stepsLeft;
  }
        

ostream &operator<<( ostream &output, const StepDriver &sd ) { 

    output << sd.planner << endl;

    return output;
}


