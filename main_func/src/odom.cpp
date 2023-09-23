#include "odom.h"
using namespace ODOM;

Odometry::Odometry(double _x, double _y, double _theta):
    x(_x), y(_y), theta(_theta/180*PI){};

void Odometry::update(const geometry_msgs::Twist::ConstPtr& ins_vel){
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    if(std::abs(dt) > 1)  dt = 0;
    if(ODOM::oriNow % 2 == 0)
        x += ins_vel->linear.x * (dt);
    if(ODOM::oriNow % 2 == 1)
        y += ins_vel->linear.y * (dt);
    theta += ins_vel->angular.z * (dt);
    while(theta > PI)  theta -= 2*PI;        
    while(theta < -PI) theta += 2*PI;
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