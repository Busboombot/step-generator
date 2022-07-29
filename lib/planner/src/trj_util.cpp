#include "trj_util.h"

bool same_sign(float a, float b){
    return (a == 0) or (b == 0) or (sgn(a) == sgn(b));
}


double  usince(steadyClock::time_point start){

    duration elapsed = steadyClock::now() - start;
    return ((double)elapsed.count())/1000.0;
}

std::vector<std::string> splitString(const std::string& str){
    std::vector<std::string> tokens;
 
    std::string::size_type pos = 0;
    std::string::size_type prev = 0;
    while ((pos = str.find('\n', prev)) != std::string::npos) {
        tokens.push_back(str.substr(prev, pos - prev));
        prev = pos + 1;
    }
    tokens.push_back(str.substr(prev));
 
    return tokens;
}