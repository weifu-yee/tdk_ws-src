#include "cammera.h"
#include "odom.h"

int firstNum1 = 0;
int firstNum2 = 0;
int secondNum1 = 0;
int secondNum2 = 0;
int thirdNum1 = 0;
int thirdNum2 = 0;

set<int> CAM::numbers;
bool CAM::cease = 0;
double CAM::capt_x[] = {125, 300, 420};

vector<vector<int>> CAM::predict_numbers(4);

void CAM::initPredictNumbers(){
    predict_numbers[1].clear();
    predict_numbers[2].clear();
    predict_numbers[3].clear();
    if(firstNum1)   predict_numbers[1].push_back(firstNum1);
    else   predict_numbers[1].push_back(1);
    if(firstNum2)   predict_numbers[1].push_back(firstNum2);
    else   predict_numbers[1].push_back(3);
    if(secondNum1)   predict_numbers[2].push_back(secondNum1);
    else   predict_numbers[2].push_back(4);
    if(secondNum2)   predict_numbers[2].push_back(secondNum2);
    else   predict_numbers[2].push_back(6);
    if(thirdNum1)   predict_numbers[3].push_back(thirdNum1);
    else   predict_numbers[3].push_back(7);
    if(thirdNum2)   predict_numbers[3].push_back(thirdNum2);
    else   predict_numbers[3].push_back(9);
}
void CAM::what_to_insert(int t, int a, int b){
    if(a && b);
    else{
        if(!a || a != CAM::predict_numbers[t][0]){
            numbers.insert(CAM::predict_numbers[t][0]);
            ROS_ERROR("numbers.insert(%d);",CAM::predict_numbers[t][0]);
        }
        if(!a && !b || a == CAM::predict_numbers[t][0]){
            numbers.insert(CAM::predict_numbers[t][1]);
            ROS_ERROR("numbers.insert(%d);",CAM::predict_numbers[t][1]);
        }
    }
}
void CAM::midway_reset_debug(){
    int firstNum1 = 0;
    int firstNum2 = 0;
    int secondNum1 = 0;
    int secondNum2 = 0;
    int thirdNum1 = 0;
    int thirdNum2 = 0;
}
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