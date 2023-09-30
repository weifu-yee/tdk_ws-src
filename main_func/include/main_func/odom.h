#ifndef _ODOM_H_
#define _ODOM_H_

#include "map.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#define PI 3.1415926
#define wheelRadius 0.0498      //m
#define carWidth 0.23704           //m
#define carLength 0.196          //m

class Odometry{
private:
    ros::Time current_time, last_time;
    double dt;
public:
    Odometry(double _x, double _y, double _theta);
    double x, y, theta;
    void update(const geometry_msgs::Twist::ConstPtr& ins_vel);
    double vel_World2Car(char coor, double Vx_world, double Vy_world);
};

namespace ODOM{
    extern Odometry odometry;
    extern int oriNow;
    extern int faceTo;
    bool slow(int nodeToGo);
}
using namespace ODOM;

#endif