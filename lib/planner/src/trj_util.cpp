#include "trj_util.h"

bool same_sign(float a, float b){
    return (a == 0) or (b == 0) or (sgn(a) == sgn(b));
}


double  usince(steadyClock::time_point start){

    duration elapsed = steadyClock::now() - start;
    return ((double)elapsed.count())/1000.0;
}