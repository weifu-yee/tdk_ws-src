#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>

using namespace cv;
#define ATTEMPT 5

ros::Subscriber cam_sub;
ros::Publisher detect_pub;
std_msgs::Bool detect_msg;
VideoCapture cap;

bool flag1=true, flag2=true, flag3=true;

bool openCam(){
    cap.open(0, CAP_V4L2);
    if (cap.isOpened()) {
        ROS_INFO("[1] Camera opened.");
        return true;
    } else {
        ROS_ERROR("[1] Could not open the camera. Retrying...");
        return false;
    } 
}

bool closeCam(){
    if (cap.isOpened()) {
        cap.release();
        ROS_INFO("[2] Camera closed.");
        return true;
    } else {
        ROS_ERROR("[2] Could not close the camera. Retrying...");
        return false;
    }
}

bool detect(){
    cap.open(0, CAP_V4L2);
    Mat frame;
    cap >> frame; 
    cap.release();

/* ADDITIONAL SAVE*/
    // Get the current time
    std::time_t now = std::time(nullptr);
    char buf[20];
    strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&now));

    // Create the directory if it doesn't exist
    struct stat st;
    if(stat("new-train-set", &st) == -1) mkdir("new-train-set", 0700);
    
    // Create the filename
    std::ostringstream img_path_stream_save;
    img_path_stream_save << "/home/ditrobotics/tdk_ws/src/yolov8/new-train-set/capture_" << buf << ".jpg";
    std::string img_path_save = img_path_stream_save.str();

    bool success_save = cv::imwrite(img_path_save, frame);  // Save the image
    if (success_save) ROS_INFO("[3] Image_save captured and saved.");
    else ROS_ERROR("[3] Failed to save the image_save. Retrying...");
/* ADDITIONAL SAVE*/

/* ORIGINAL SAVE*/
    // add number on file name
    std::ostringstream img_path_stream;
    img_path_stream << "/home/ditrobotics/tdk_ws/src/yolov8/jpg/capture.jpg";
    std::string img_path = img_path_stream.str();

    ros::Duration(0.2).sleep();

    bool success = cv::imwrite(img_path, frame);  // Save the image
    if (success) ROS_INFO("[3] Image captured and saved.");
    else ROS_ERROR("[3] Failed to save the image. Retrying...");
    
    ros::Duration(0.5).sleep();
    
    // call detect.py
    detect_msg.data = true;
    ros::Rate rate(20);
    for(int i = 1; i <= 20; i++){
        detect_pub.publish(detect_msg);
        ros::spinOnce();
        rate.sleep();
    }

    detect_msg.data = false;
    detect_pub.publish(detect_msg);

    frame.release();
    return success;
}
/* ORIGINAL SAVE*/

void modeCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int att, mode = msg->data;
    if (mode == 1 && flag1) {
        flag1 = 0;
        att = 0;
        while(!openCam() && att<ATTEMPT) att++;
    } else if (mode == 2 && flag2) {
        flag2 = 0;
        att = 0;
        while(!closeCam() && att<ATTEMPT) att++;
    } else if (mode == 3 && flag3) {
        flag3 = 0;
        att = 0;
        while(!detect() && att<ATTEMPT) att++;
    } else if (mode == 4) {
        flag1 = true;
        flag2 = true;
        flag3 = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_cpp_node");
    ros::NodeHandle nh;

    cam_sub = nh.subscribe("/mode", 10, modeCallback);
    detect_pub = nh.advertise<std_msgs::Bool>("/detect", 10);
    ros::spin();

    return 0;
}