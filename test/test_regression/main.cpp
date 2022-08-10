#include <iostream>
#include <unity.h>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <time.h>

#include "trj_segment.h"
#include "trj_stepdriver.h"
#include "trj_util.h"
#include "trj_debug.h"

// Can't get platform.io to load this library
// #include "FastCRC.h"

using namespace std;

extern time_t time(time_t *);

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;

void ser_printf(const char* fmt, ...){}

// Fixes bug 
// https://github.com/arduino/Arduino/issues/9413
// https://github.com/arduino-libraries/ArduinoBearSSL/issues/54
extern "C" {
  // This must exist to keep the linker happy but is never called.
  int _gettimeofday( struct timeval *tv, void *tzvp )
  {
    return 0;  // return non-zero for error
  } // end _gettimeofday()
}

void setUp(){
    
}
void tearDown(){
    
}

vector<Move> get2Moves(){
    return std::vector<Move>{
        Move( 0, {10000,100}),
        Move( 0, {10000,100})
    };
}

vector<Joint> getJoints(){
    return std::vector<Joint>{
        Joint(0, 10e3, 300e3),
        Joint(1, 10e3, 300e3)
     
    };
}


struct RunOut {
    int n = 0;
    uint32_t t = 0; // total_time
    uint32_t crc = 0;
};

struct RunOut run_out(Planner &p, bool print = false){

    struct RunOut ro;
    
    do {
        const PhaseJoints pj = p.getNextPhase();

        if(print)
            cout << pj << " ql="<< p.getQueueSize()<< " qt="<< p.getQueueTime() << endl;
        ro.n++;
        ro.t += pj.t;

     
            
    } while (p.getQueueSize() > 0);

    return ro;
}



void test_driver(){


    StepDriver sd = StepDriver();
    sd.setPeriod(10);

    int v_max = 5000;
    int a_max = 50000;

    int x = 6400;

    sd.setAxisConfig(0, v_max, a_max);

    sd.push(Move(0,1e5, Move::MoveType::relative, {x}));
    sd.push(Move(1,1e5, Move::MoveType::relative, {-x}));
    sd.push(Move(2,1e5, Move::MoveType::relative, {x}));

    cout  << sd << endl;
    int seq = 0;
    while(!sd.isEmpty()){

        int sl = sd.update() ;
        
        seq = sd.checkIsDone();
        if (seq >=0){
            cout << "DONE "<<seq<<endl;
        }

        if (sd.checkIsEmpty()){
            cout << "EMPTY "<<endl;
        }

        if(sl == 0){
            cout << "============ " << sd.sincePhaseStart() << endl;

            cout << "END " << sd.getState(0) << endl;
        }
    }
    
    sd.disable();

    cout  << sd << endl;

}
/* Triangle segments, which have a zero cruise time (v_c) 
*  because either their acceleation time is too long, or the total segment
* is too short, dont' get their velocities linked. 
Possibly related to:
*   void JointSegment::update_end_velocity_limit(bool is_last)
*       The check  x_a + x_d >= x forces final velocity to zero. 
*/

void test_low_rpm(){
    {
    Planner p = Planner( {Joint(0, 33333, 66666)} );
    
    int x = 5000;
    
    p.push(Move(0,0, Move::MoveType::relative, {x}));
    p.push(Move(1,0, Move::MoveType::relative, {x}));

    
    cout << p <<  endl;

    run_out(p, true);
    }

    cout << endl << "XXXXXXXXXXXXXXXX"  << endl  << endl;

    {
    Planner p = Planner( {Joint(0, 33333, 6666600)} );
    
    int x = 5000;
    
    p.push(Move(0,0, Move::MoveType::relative, {x}));
    p.push(Move(1,0, Move::MoveType::relative, {x}));

    
    cout << p <<  endl;

    run_out(p, true);
    }

}



int main(){

    UNITY_BEGIN();     
    //RUN_TEST(test_driver);
    RUN_TEST(test_low_rpm);
    UNITY_END();
    return 0;
}