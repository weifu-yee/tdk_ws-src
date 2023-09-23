#include "ros/ros.h"
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

std_msgs::Bool node_point;

void detect_callback(const std_msgs::Bool::ConstPtr& msg){
    node_point.data = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "debug");
    ros::NodeHandle nh;
    ros::Publisher pub1 = nh.advertise<std_msgs::Int8>("dir", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Int8MultiArray>("tracker_data", 10);
    ros::Subscriber sub1 = nh.subscribe<std_msgs::Bool>("node_detect",10,detect_callback);
    ros::Rate loop_rate(1);
    
    int number = 0;
    std_msgs::Int8 msg1;
    std_msgs::Int8MultiArray msg2;

    for (int i = 0; i < 20; ++i) {
        msg2.data.push_back(0); // 添加整数数据到消息中
    }
    
    while(ros::ok())
    {   
        ros::spinOnce();
        
        msg1.data = 0;
        pub1.publish(msg1);

        msg2.data[7]=1;
        msg2.data[8]=number%2;
        msg2.data[1]=1;
        msg2.data[10]=1;
        // msg2.data[4]=1;
        // msg2.data[10]=1;
        // msg2.data[5]=1;

        //ROS_INFO("%d",number);
        //ROS_INFO("%d", msg1.data);
        // 输出 msg2.data 中的每个元素
        // for (size_t i = 0; i < msg2.data.size(); ++i) {
        //     ROS_INFO("msg2.data[%zu]: %d", i, msg2.data[i]);
        // }
        ROS_INFO("node detect: %d",node_point.data);
        
        pub2.publish(msg2);

        
        loop_rate.sleep();

        ++number;
    }
    return 0;
}