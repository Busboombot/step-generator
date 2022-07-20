#include <iostream>
#include <unity.h>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <time.h>


#include "trj_planner.h"
#include "trj_segment.h"

// Can't get platform.io to load this library
// #include "FastCRC.h"

using namespace std;

extern time_t time(time_t *);

// These are supposed to be defined somewhere, but aren't
unsigned __exidx_start;
unsigned __exidx_end;

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



void test_basic(){
    Planner p = Planner( {Joint(0, 10e3, 300e3)} );
    
    int x = 1000;
    
    p.push(Move(0,1e5, Move::MoveType::relative, {x}));
    TEST_ASSERT_EQUAL(1000, p.getPosition()[0]);
    
    cout << p <<  endl;

    p.push(Move(0,1e5, Move::MoveType::relative, {x}));
    TEST_ASSERT_EQUAL(2000, p.getPosition()[0]);
    
    cout << p <<  endl;

    p.push(Move(0,1e5, Move::MoveType::relative, {x}));
    TEST_ASSERT_EQUAL(3000, p.getPosition()[0]);
    
    cout << p <<  endl;

}



Move jog(int x){
    return Move(0,1e5, Move::MoveType::jog, {x});
}

void test_jog_ql(){
    Planner p = Planner( {Joint(0, 10e3, 300e3)} );
    
    p.push(jog(1000));

    cout << p <<  endl;
    
    p.push(jog(1000));

    cout << p <<  endl;

    p.push(jog(2000));
   
    cout << p <<  endl;

    p.push(jog(3000));
   
    cout << p <<  endl;

}

// With low accelleration, this routine produces triangle segments
// that have no Cruise subsegment. This sort of segment does not link up correctly,
// and the final velocity will be 0 even if there is a following segment. 
void test_basic_triangle(){
    Planner p = Planner( {Joint(0, 10e3, 1e6)} );
    
    int x = 2000;
    
    p.push(Move(0,1e4, Move::MoveType::relative, {x}));
    p.push(Move(0,1e4, Move::MoveType::relative, {x}));
    
    cout << p <<  endl;
}

int main(){

    UNITY_BEGIN();   

    RUN_TEST(test_jog_ql);
    RUN_TEST(test_basic);
    RUN_TEST(test_basic_triangle);
    
   
    UNITY_END();
    return 0;
}