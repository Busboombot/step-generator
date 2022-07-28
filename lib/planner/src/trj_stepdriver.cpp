#include "trj_stepdriver.h"


CurrentState current_state;


void StepDriver::setAxisConfig(AxisConfig* as){
  
  int pos = 0;

  if(as->axis < n_axes){

    // Clear out any old stepper instance
    if (steppers[as->axis] != 0)
      pos = steppers[as->axis]->getPosition(); // But save the position
      delete steppers[as->axis];

    // Then make a new one. 
    steppers[as->axis] = newStepper(as);

    steppers[as->axis]->setPosition(pos);

    Joint joint(as->axis,static_cast< float >(as->v_max), static_cast< float >(as->a_max));
    planner.setJoint(joint);
  }
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

int StepDriver::stepAll(){

    unsigned long  steps_left = 0;

    if(finished_phase)
      return -1;

    //clearTimer.begin(clearISR,1.55);  // 1.55 is the minimum on teensy4 == 36 cycles. 

    switch (n_axes) {
      case 6: steps_left += step(5);
      case 5: steps_left += step(4);
      case 4: steps_left += step(3);
      case 3: steps_left += step(2);
      case 2: steps_left += step(1);
      case 1: steps_left += step(0);
      case 0: ;
    }

    return steps_left;
}

void StepDriver::pushMoves(Move::MoveType move_type, int seq, uint32_t segment_time,  int32_t x[]){

  
    Move move(n_axes, seq, segment_time, 0);

    move.move_type = move_type;
    
    for (int axis = 0; axis < n_axes; axis++){
      move.x[axis] = x[axis];

      if(move_type == Move::MoveType::absolute){
        current_state.planner_positions[axis] = x[axis];
      } else {
        current_state.planner_positions[axis] += x[axis];
      }
    }
    
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    planner.push(move);

}

void StepDriver::update(){

  if (!enabled){
    return;
  }

  if (finished_phase){
    stop(); // running = false;


    if(planner.isEmpty()){
      //sdp.sendEmpty(planner.getCurrentPhase().seq, current_state);
      //disable();
    }  
    finished_phase = false;
  }

  if (finished_phase == false and !running and planner.getQueueSize() > 0){

      int active_axes = 0;

      if(!planner.isEmpty()){

        const PhaseJoints&  pj = planner.getNextPhase();

        for (int axis = 0; axis < n_axes; axis++){
          const JointSubSegment &jss = pj.moves[axis];
          if(jss.x != 0){
            active_axes++;
            state[axis].setParams(jss.t, jss.v_0, jss.v_1, jss.x, period); // Omits config for INTERRUPT_DELAY
          } else {
            state[axis].setParams(0, 0, 0, 0, 0);
          }
        }

      }

    if(active_axes > 0){
      start(); // running = true
    }
  }
}
