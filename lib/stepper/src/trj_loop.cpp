#include <Arduino.h>

#include <sstream>
#include <functional>
#include <limits.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "trj_loop.h"
#include "trj_messageprocessor.h"
#include "trj_fastset.h"
#include "trj_sdstepper.h"
#include "trj_ringbuffer.h"
#include "trj_bithacks.h"
#include "trj_config.h"
#include "trj_segment.h"
#include "trj_debug.h"

extern Loop  mainLoop;


void clearSegmentCompleteISR(){ mainLoop.clearSegmentComplete();}

inline void Loop::signalSegmentComplete(){

  for(int i = 0; i < config.n_axes; i++){
    current_state.positions[i] = sd.getState(i).getPosition();
  }

  auto& planner = sd.getPlanner();

  current_state.queue_length = planner.getQueueSize();
  current_state.queue_time = planner.getQueueTime();

  sdp.sendDone(planner.getCurrentPhase().seq, current_state);

  if(config.segment_complete_pin > 0){
    digitalWriteFast(config.segment_complete_pin, HIGH);
    segmentCompleteTimer.begin(clearSegmentCompleteISR,2); 
  }
}

inline void Loop::clearSegmentComplete(){
  if(config.segment_complete_pin > 0){
    digitalWriteFast(config.segment_complete_pin, LOW);
  }
}

void Loop::setup(){
  return;
}

/* Run one iteration of the main loop
*/
void Loop::loopOnce(){
    // Blink the LED and toggle a debugging pin, to show we're not crashed. 
    
  
    static unsigned long last = millis();
    static bool ledToggle = true;
    
    // Fast tick for running, slow for idle
    if( millis() - last > (running ? 10 : 500)  ){
      digitalWrite(LED_BUILTIN, (ledToggle = !ledToggle));
      last = millis();
    }

    sdp.update();  // Get serial data and update queues 
    
    sd.tick(); // sd.update(); // Update the steppers

    int seq = sd.checkIsDone();
    if (seq >=0){
         signalSegmentComplete();
    }

    if (sd.checkIsEmpty()){
      sdp.sendEmpty(sd.getPlanner().getCurrentPhase().seq, current_state);
    }

}


// Start the timers, if there are segments available. 
void Loop::start(){ 
  running = true;
}

void Loop::stop(){ 
  running = false;
}


/**
 * @brief Remove all of the segments from the queue
 * 
 */
void Loop::reset(){
  sd.clear();
}

void Loop::zero(){
  sd.zero();
}

void Loop::setConfig(Config* config_){
  
  config.interrupt_delay = config_->interrupt_delay;
  config.n_axes = config_->n_axes;
  config.debug_print = config_->debug_print;
  config.debug_tick = config_->debug_tick;
  config.segment_complete_pin = config_->segment_complete_pin;

  if (config.segment_complete_pin > 0){
    pinMode(config.segment_complete_pin, OUTPUT);
  }

  sd.setNAxes(config.n_axes);
  sd.setPeriod(config.interrupt_delay);
}

/**
 * @brief Configure a stepper for an axis. Creates a new StepperInterface object for the axis
 * 
 * @param as Axis configuration object
 * @param eeprom_write If true, write positions to the eeprom. 
 */
void Loop::setAxisConfig(AxisConfig* as){

  if(as->axis < config.n_axes){
    sd.setAxisConfig(as->axis, as->v_max, as->a_max);
    if ( steppers[as->axis] != nullptr){
      delete steppers[as->axis];
    }

    steppers[as->axis] = new StepDirectionStepper(as->axis,as->step_pin,as->direction_pin,as->enable_pin);
  
    sd.setStepper(as->axis, steppers[as->axis]);
  
    axes_config[as->axis] = *as;
  }

}

// Turn a move command into a move and add it to the planner
void Loop::processMove(const uint8_t* buffer_, size_t size){

    PacketHeader *ph = (PacketHeader*)buffer_;
    Moves *m = (Moves*)(buffer_ + sizeof(PacketHeader));
  
    Move move(getConfig().n_axes, ph->seq, m->segment_time, 0);


    switch(ph->code){
        case CommandCode::RMOVE:
     
        move.move_type = Move::MoveType::relative;
        break;
        
        case CommandCode::AMOVE:
        move.move_type = Move::MoveType::absolute;
        break;
        
        case CommandCode::JMOVE:
        move.move_type = Move::MoveType::jog;
        break;
        
        default: ; 
    }



    for (int axis = 0; axis < getConfig().n_axes; axis++){
      move.x[axis] = m->x[axis];
      // FIXME! This position update will only work for relative moves
      current_state.planner_positions[axis] += m->x[axis];
    }
    
    auto &planner = sd.getPlanner();
    current_state.queue_length = planner.getQueueSize();
    current_state.queue_time = planner.getQueueTime();

    sd.push(move);
}

void Loop::printInfo(){

  auto& planner = sd.getPlanner();

  sdp.printf("===========\n"
            "Queue Size : %d\r\n"
            "Queue Time : %d\r\n"
            "Running    : %d\r\n"
            "N Axes     : %d\r\n"
            "Joints     : %d\r\n"
            "Intr Delay : %d\r\n"
            "ClrSeg Pin : %d\r\n"
            "Debug print: %d\r\n"
            "Debug tick : %d\r\n",
           planner.getQueueSize(), planner.getQueueTime(), running, 
           config.n_axes,planner.getJoints().size(), config.interrupt_delay, 
           config.segment_complete_pin,config.debug_print, config.debug_tick) ;
  
  for(const Joint &j : planner.getJoints() ){

    if(j.n>=config.n_axes){
      continue;
    }

    AxisConfig as = axes_config[j.n];
    StepperState &state = sd.getState(j.n);

    sdp.printf("-- Axis %d \r\n"
            "SDE        : %d %d %d\r\n"
            "A V Max    : %d %d\r\n"
            "Position   : %d\r\n",
            j.n, as.step_pin, as.direction_pin, as.enable_pin, 
            static_cast<int>(j.a_max), static_cast<int>(j.v_max),
            state.getPosition()); 
  }
  
  stringstream ss;
  ss << sd << endl;
  
  std::string line;
  while (std::getline(ss, line, '\n')) {
    sdp.sendMessage(line.c_str());
  }

}
