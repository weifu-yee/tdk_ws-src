#include "odom.h"
using namespace ODOM;

Odometry::Odometry(double _x, double _y, double _theta):
    x(_x), y(_y), theta(_theta/180*PI){};

void Odometry::update(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if(std::abs(dt) > 1)  dt = 0;

    double x_car, y_car;
    // if(ODOM::oriNow >= 10 || ODOM::oriNow % 2 == 0)
        x_car = ins_vel->linear.x * (dt);
    // if(ODOM::oriNow >= 10 || ODOM::oriNow % 2 == 1)
        y_car = ins_vel->linear.y * (dt);

    int xMat[] = {1, 0, -1, 0};
    int yMat[] = {0, 1, 0, -1};
    int i = -ODOM::faceTo + 4;   i %= 4;
    int j = -ODOM::faceTo + 5;   j %= 4;
    x += xMat[i] * x_car + yMat[i] * y_car;
    y += xMat[j] * x_car + yMat[j] * y_car;

    if(ODOM::oriNow == 4 || ODOM::oriNow == 5)
        theta += ins_vel->angular.z * (dt);
    while(theta > PI)  theta -= 2*PI;
    while(theta <= -PI) theta += 2*PI;
    last_time = current_time;
    return;
}


Odometry ODOM::odometry(0, 0, 0);
int ODOM::oriNow = 0;
int ODOM::faceTo = 0;
bool ODOM::slow_mode = false;
std::stack<int> ODOM::slow_points;
void ODOM::initSlowPoints(){
    while(!ODOM::slow_points.empty())   ODOM::slow_points.pop();
    ODOM::slow_points.push(46);
    ODOM::slow_points.push(45);
    ODOM::slow_points.push(44);
    ODOM::slow_points.push(43);
    ODOM::slow_points.push(42);
    ODOM::slow_points.push(41);
}
void ODOM::SECinitSlowPoints(){
    while(!ODOM::slow_points.empty())   ODOM::slow_points.pop();
    ODOM::slow_points.push(46);
    ODOM::slow_points.push(45);
}
bool ODOM::slow(int nodeToGo){
    if(MAP::disToOdom(nodeToGo) < decelerationZone)     return true;
    return false;
}