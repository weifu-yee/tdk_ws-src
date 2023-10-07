#include "cammera.h"
#include "odom.h"

set<int> CAM::numbers;
bool CAM::cease = 0;
double CAM::capt_x[] = {120, 320, 430};

void CAM::what_to_erase(int a, int b){
    auto eraseBox = [&](int u){
        if(u < 7)
            MAP::eraseEdge(u, u+3);
        else{
            if(u == 7)  MAP::eraseEdge(7, 12);
            else if(u == 8)  MAP::eraseEdge(8, 11);
            else if(u == 9)  MAP::eraseEdge(9, 10);
        }
    };
    eraseBox(a);
    eraseBox(b);
    auto secondColumn = [&](){
        return (ODOM::odometry.y > (49 + 139)/2 && ODOM::odometry.y < (139 + 229)/2);
    };
    if(a == 1 && b == 3)    MAP::eraseEdge(2, 3);
    if(a == 2 && b == 3)    MAP::eraseEdge(0, 2);
    if(a == 5 && b == 6 && secondColumn())    MAP::eraseEdge(5, 6);
    if(a == 8 && b == 9 && secondColumn())    MAP::eraseEdge(8, 9);
}