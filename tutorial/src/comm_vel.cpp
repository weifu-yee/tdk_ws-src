#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <cmath>
#include "std_msgs/Int8.h"

// #define deceleration_coeff 0.93
// #define deceleration_minSpeed 5.0
// #define deceleration_minSpeed 5

//to control which cmd_vel can be send

//main_function arguments --from main_func & script & navi
geometry_msgs::Twist vel;
geometry_msgs::Twist vel_last;
int ori = -1, ori_last = -1;

void vel_callback(const geometry_msgs::TwistConstPtr& msg){
    // if(ori != -2){
        vel.linear.x = msg->linear.x;
        vel.linear.y = msg->linear.y;
        vel.angular.z = msg->angular.z;
    // }
}

void ori_callback(const std_msgs::Int8::ConstPtr& msg){
    ori = msg->data;
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "comm_vel");
    ros::NodeHandle nh;

    double span=0.05;
    double softRate = 1;
    nh.getParam("/span",span);
    nh.getParam("/softRate",softRate);
    ros::Subscriber sub_dir = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, ori_callback);
    ros::Subscriber sub_vel = nh.subscribe<geometry_msgs::Twist>("vel", 10, vel_callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    
    
    std_msgs::Int8 script;
    double deceleration_coeff, deceleration_minSpeed;

    vel_last.linear.x = 0;
    vel_last.linear.y = 0;
    vel_last.angular.z = 0;

    while(ros::ok())
    {   
        ros::spinOnce();
        
        nh.getParam("/deceleration_coeff",deceleration_coeff);
        nh.getParam("/deceleration_minSpeed",deceleration_minSpeed);

        // if(fabs(vel.linear.x - vel_last.linear.x) > softRate){
        //     vel.linear.x = vel_last.linear.x + (vel.linear.x > vel_last.linear.x)?softRate:-softRate;
        //     vel_last.linear.x = vel.linear.x;
        // }
        // if(fabs(vel.linear.y - vel_last.linear.y) > softRate){
        //     vel.linear.y = vel_last.linear.y + (vel.linear.y > vel_last.linear.y)?softRate:-softRate;
        //     vel_last.linear.y = vel.linear.y;
        // }
        // if(fabs(vel.angular.z - vel_last.angular.z) > softRate){
        //     vel.angular.z = vel_last.angular.z + (vel.angular.z > vel_last.angular.z)?softRate:-softRate;
        //     vel_last.angular.z = vel.angular.z;
        // }

        vel.linear.x = softRate * vel_last.linear.x + (1 - softRate) * vel.linear.x;
        vel.linear.y = softRate * vel_last.linear.y + (1 - softRate) * vel.linear.y;
        // vel.angular.z = softRate * vel_last.angular.z + (1 - softRate) * vel.angular.z;

        vel_last.linear.x = vel.linear.x;
        vel_last.linear.y = vel.linear.y;
        vel_last.angular.z = vel.angular.z;
                    
        if(ori == -1){
            vel.linear.x = 0;
            vel.linear.y = 0;
            vel.angular.z = 0;
            pub.publish(vel);
        }
        else if(ori == 6){

        }
        else if(ori == 7){

        }
        else if(ori == 8){

        }
        else if(ori >= 0 && ori < 4 || ori == -2){
            pub.publish(vel);
        }

        // ROS_INFO("dir: %d ",dir);
        // ROS_INFO("dv: %f  dtheta: %f",det_vel.linear.y,det_vel.angular.z);
        
        ROS_INFO("ori:%d , Vx: %.1lf  Vy: %.1lf  w: %.1lf",ori,vel.linear.x,vel.linear.y,vel.angular.z);

        ros::Duration(span).sleep();
    }
    return 0;
}