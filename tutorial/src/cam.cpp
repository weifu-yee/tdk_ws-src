#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <cstdlib>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
#define ATTEMPT 5

ros::Subscriber cam_sub;
VideoCapture cap;

int idx=0;

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
    // add number on file name
    std::ostringstream img_path_stream;
    img_path_stream << "/home/ditrobotics/tdk_ws/src/tutorial/src/capture_" << idx << ".jpg";
    std::string img_path = img_path_stream.str();

    Mat frame;
    cap >> frame; 
    bool success = cv::imwrite(img_path, frame);  // Save the image
    if (success) ROS_INFO("[3] Image captured and saved.");
    else ROS_ERROR("[3] Failed to save the image. Retrying...");
    
    // Update the system call to include idx as an argument
    std::ostringstream command_stream;
    command_stream << "python3 /home/ditrobotics/tdk_ws/src/tutorial/src/detect.py " << idx;
    std::string command = command_stream.str();
    system(command.c_str());

    idx++;
    ROS_INFO("[3] Detect Finish.");
    return success;
}

void modeCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int att, mode = msg->data;
    if (mode == 1) { 
        att = 0;
        while(!openCam() && att<ATTEMPT) att++;
    } else if (mode == 2) {
        att = 0;
        while(!closeCam() && att<ATTEMPT) att++;
    } else if (mode == 3) {
        att = 0;
        while(!detect() && att<ATTEMPT) att++;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_cpp_node");
    ros::NodeHandle nh;

    ros::Subscriber cam_sub = nh.subscribe("/mode", 10, modeCallback);
    ros::spin();

    return 0;
}
