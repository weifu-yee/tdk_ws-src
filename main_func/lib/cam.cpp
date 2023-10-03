#include "cam.h"
#include "odom.h"

set<int> CAM::numbers;
bool CAM::cease = 0;

double secondCapt = 300;
double thirdCapt = 430;
int _pub4 = 0;

void CAM::capture_n_detect(int op, ros::Publisher& cam_pub, 
        ros::Publisher& orientation_pub, ros::NodeHandle& nh){
    std_msgs::Int32 cam_mode;
    cam_mode.data = 3;
    std_msgs::Int8 cease;
    cease.data = -1;

    ROS_WARN("cease!!");
    CAM::cease = 1;

    ros::Rate rate(20); //20Hz
    int flag = 0;
    int _a = 0, _b = 0;
    do{     //the capture and detect process
        ros::spinOnce();
        if(_pub4 > 60)   cam_mode.data = 4;
        else    cam_mode.data = 3;
        cam_pub.publish(cam_mode);
        orientation_pub.publish(cease);
        int a = 0, b = 0;
        for(int i = op; i <= op + 2; i++){
            if(numbers.find(i) != numbers.end()){
                if(!a){
                    a = i;      flag = 0;
                }
                else if(!b){
                    b = i;
                    flag = 1;
                }
                else{
                    flag = 0;   
                    CAM::numbers.clear();
                }
            }
        }
        _a = a;     _b = b;
        _pub4 ++;        
        rate.sleep();
    }while(!flag && nh.ok());
    what_to_erase(_a, _b);
    CAM::cease = 0;
}
void CAM::what_to_erase(int a, int b){
    auto eraseBox = [&](int u){
        if(u < 7)
            MAP::eraseEdge(u, u+3);
        else{
            if(u == 7)  MAP::eraseEdge(7, 12);
            else if(u == 8)  MAP::eraseEdge(8, 11);
            else if(u == 9)  MAP::eraseEdge(9, 10);
        }
    };
    eraseBox(a);
    eraseBox(b);
    auto secondColumn = [&](){
        return (ODOM::odometry.y > (49 + 139)/2 && ODOM::odometry.y < (139 + 229)/2);
    };
    if(a == 1 && b == 3)    MAP::eraseEdge(2, 3);
    if(a == 2 && b == 3)    MAP::eraseEdge(0, 2);
    if(a == 5 && b == 6 && secondColumn())    MAP::eraseEdge(5, 6);
    if(a == 8 && b == 9 && secondColumn())    MAP::eraseEdge(8, 9);
}