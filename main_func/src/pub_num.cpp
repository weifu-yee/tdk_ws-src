#include <iostream>
#include <fstream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_num");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("/numbers", 10);
    ros::Rate rate(1);  // 1 Hz
    std::vector<int> arr = {1,2,3,4,5,6,7,8,9};


    try {
        // Load the YAML file
        YAML::Node config = YAML::LoadFile("/home/ditrobotics/tdk_ws/src/main_func/params/pub_num.yaml");

        // Access values by key
        int passThrough1 = config["passThrough1"].as<int>();
        int passThrough2 = config["passThrough2"].as<int>();
        int passThrough3 = config["passThrough3"].as<int>();

        arr[passThrough1 - 1] = 0;
        arr[passThrough2 - 1] = 0;
        arr[passThrough3 - 1] = 0;

        // Print the values
        std::cout << "passThrough1: " << passThrough1 << std::endl;
        std::cout << "passThrough2: " << passThrough2 << std::endl;
        std::cout << "passThrough3: " << passThrough3 << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return 1;
    }

    std_msgs::Int32MultiArray multi_array_msg;
    multi_array_msg.data = arr;  // Example data
    while (ros::ok())
    {
        pub.publish(multi_array_msg);
        rate.sleep();
    }

    return 0;
}
