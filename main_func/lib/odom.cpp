#include "odom.h"
using namespace ODOM;

Odometry::Odometry(double _x, double _y, double _theta):
    x(_x), y(_y), theta(_theta/180*PI){};

void Odometry::update(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if(std::abs(dt) > 1)  dt = 0;

    double x_car, y_car;
    if(ODOM::oriNow >= 10 || ODOM::oriNow % 2 == 0)
        x_car = ins_vel->linear.x * (dt);
    if(ODOM::oriNow >= 10 || ODOM::oriNow % 2 == 1)
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
double Odometry::vel_World2Car(char coor, double Vx_world, double Vy_world){
	if(coor == 'x')
        return Vx_world * cos(theta) - Vy_world * sin(theta);
	else
		return Vx_world * cos(theta) + Vy_world * sin(theta);
}

Odometry ODOM::odometry(0, 0, 0);
int ODOM::oriNow = 0;
int ODOM::faceTo = 0;
bool ODOM::slow(int nodeToGo){
    if(nodeNow != -1){
        double ux = MAP::node[nodeToGo].second.first;
        double uy = MAP::node[nodeToGo].second.second;
        double vx = MAP::node[MAP::nodeNow].second.first;
        double vy = MAP::node[MAP::nodeNow].second.second;
        double x_diff = fabs(ux - vx);
        double y_diff = fabs(uy - vy);
        if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
            return true;
    }else{
        double ux = MAP::node[0].second.first;
        double uy = MAP::node[0].second.second;
        double vx = 60;
        double vy = 180;
        double x_diff = fabs(ux - vx);
        double y_diff = fabs(uy - vy);
        if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
            return true;
    }
    return false;
}