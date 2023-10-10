#ifndef _ODOM_H_
#define _ODOM_H_

#include "map.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <stack>

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
};

namespace ODOM{
    extern Odometry odometry;
    extern int oriNow;
    extern int faceTo;
    extern bool slow_mode;
    extern std::stack<int>   slow_points;
    void initSlowPoints(void);
    bool slow(int nodeToGo);
}
using namespace ODOM;

#endif