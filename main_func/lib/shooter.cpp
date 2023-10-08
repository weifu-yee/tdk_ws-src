#include "shooter.h"

int SHOOTER::compute_angle(){
    if(MAP::des_baseball.empty())   return 0;
    int des = MAP::des_baseball.top();
    MAP::des_baseball.pop();
    return 1;
}