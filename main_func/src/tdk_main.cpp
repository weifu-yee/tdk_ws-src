#include "map.h"
#include "cam.h"
#include "odom.h"
#include "reset.h"
using namespace std;

#define att 20
enum Action{
    waiting,
    tutorial_move,
    odom_move,
    rotate,
    capture_n_detect,
    over_hurdles,
    calibration,
    binBaiYa,
    dustBox
};

Action individual_action;
string robot_state;

//global vars
int start_now = -1, start_togo = 0;
bool isNodeLast = false;
bool onNode = false;
double xNow, xLast = -1;
bool nodeLoseConp = 0;
int capt_ed_times = 0;
bool rotate_ed = 0;
int time_6 = 30, time_7 = 15;
// double after_6_shift = 40;
double after_6_shift = 40;
bool stick = 0;
bool secRESET = 0;
int cam_mode_1 = 0;
int ori6State = 0;
int op;
int cam_flag = 0, _a = 0, _b = 0, _pub4 = 0;
double degrees[] = {0, PI/2, PI, -PI/2};
double thetaToGo;
int after_6_shift_state = 0;
//publisher
ros::Publisher orientation_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher cam_pub;
ros::Publisher node_detect_pub;
ros::Publisher laji_pub;
//subscriber
ros::Subscriber node_sub;
ros::Subscriber number_sub;
ros::Subscriber odom_sub;
ros::Subscriber stickOnLine_sub;
ros::Subscriber reset_sub;
//msgs
std_msgs::Int8 orientation;
geometry_msgs::Twist cmd_vel;
std_msgs::Int32 cam_mode;
std_msgs::Int8 cmd_laji;
geometry_msgs::Twist rotate_ang;
std_msgs::Bool ONE;

//initialization
void runInit(ros::NodeHandle& nh);

//Callback
void nodeCallback(const std_msgs::Bool::ConstPtr& is_node);
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers);
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel);
void stickCallback(const std_msgs::Bool::ConstPtr& sitck_);
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data);

bool amoungDeg(double a, double b);

//main
int main(int argc, char **argv){
    ros::init(argc, argv, "tdk_main");
    ros::NodeHandle nh;

    MAP::buildNode();
    MAP::initBuildEdge();

    runInit(nh);

    ros::Rate rate(freq);
    
    
    while(nh.ok()){

        ROS_WARN_THROTTLE(1, "robot_state: %s",robot_state.c_str());
        // ROS_WARN("robot_state: %s",robot_state.c_str());

        switch(RESET::state){
            case 0:{
                robot_state = "waiting";
                individual_action = Action::waiting;
                MAP::initBuildEdge();
                MAP::startPointInit(-1, 0);
                break;
            }
            case 1:{
                if(robot_state == "waiting"){
                    robot_state = "tutorial_move";
                }
                if(!secRESET){
                    //開相機
                    if(cam_mode_1 < att){
                        cam_mode.data = 1;
                        cam_pub.publish(cam_mode);
                        cam_mode_1 ++;
                    }
                    //跨坎
                    if(ori6State < (int)time_6*freq && odometry.x >= 140 + 20 && 
                        (MAP::nodeNow >= 1 && MAP::nodeNow <= 3) && 
                        (MAP::nodeToGo >= 4 && MAP::nodeToGo <= 6)){   
                        robot_state = "over_hurdles";
                    }
                    //三次辨識
                    for(int i = 0; i <= 2; i++){
                        if(capt_ed_times == i && odometry.x >= CAM::capt_x[i]){
                            op = 3*i + 1;
                            ROS_WARN("cease!!");
                            cam_flag = 0, _a = 0, _b = 0, _pub4 = 0;
                            robot_state = "capture_n_detect";
                            capt_ed_times++;
                        }
                    }
                    //節點12旋轉
                    if(MAP::nodeNow == 12 && !rotate_ed){
                        thetaToGo = degrees[(ODOM::faceTo + 1) % 4];
                        ROS_WARN("---------------------------------");
                        ROS_WARN("SCRIPT::rotateCCW, faceTo: %f -> %f",degrees[ODOM::faceTo],thetaToGo);

                        robot_state = "rotate";
                        rotate_ed = 1;
                    }
                    //第二重製區偏移
                    if(ODOM::faceTo == 1 && (odometry.x < 710 || odometry.y < 370)){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("sec RESET shifting (odom_move)");
                        robot_state = "odom_move";
                    }


                    if(robot_state == "tutorial_move"){
                        individual_action = Action::tutorial_move;
                    }
                    else if(robot_state == "over_hurdles"){

                        if(!ori6State){
                            ROS_WARN("---------------------------------");
                            ROS_WARN("over_hurdles");
                        }
                        individual_action = Action::over_hurdles;
                        ori6State++ ;
                        if(ori6State == time_6*freq){
                            robot_state = "calibration";
                            stick = 0;
                        }
                    }
                    else if(robot_state == "calibration"){
                        individual_action = Action::calibration;
                        if(stick == true){
                            ROS_WARN("stick on the line!!");
                            robot_state = "tutorial_move";
                        }
                    }
                    else if(robot_state == "capture_n_detect"){
                        // ROS_INFO("capture_n_detect!!");
                        individual_action = Action::capture_n_detect;
                        
                        if(cam_flag == 1){
                            CAM::what_to_erase(_a, _b);
                            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                            orientation_pub.publish(orientation);
                            robot_state = "tutorial_move";
                            individual_action = Action::tutorial_move;
                        }
                    }
                    else if(robot_state == "rotate"){
                        individual_action = Action::rotate;
                            
                        if(!amoungDeg(ODOM::odometry.theta,thetaToGo)){
                            ODOM::faceTo ++;
                            ROS_INFO("ODOM::faceTo: %d",ODOM::faceTo);
                            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                            orientation_pub.publish(orientation);
                            robot_state = "tutorial_move";
                            individual_action = Action::tutorial_move;
                        }
                        
                    }
                    else if(robot_state == "odom_move"){
                        individual_action = Action::odom_move;

                        ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                        if(odometry.x < 710)    cmd_vel.linear.y = -2;
                        else    cmd_vel.linear.y = 0;
                        if(odometry.y < 370)    cmd_vel.linear.x = 15;
                        else    cmd_vel.linear.x = 0;

                        if(!(odometry.x < 710 || odometry.y < 370)){
                            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                            orientation_pub.publish(orientation);
                            robot_state = "tutorial_move";
                            individual_action = Action::tutorial_move;
                            secRESET = true;
                        }
                    }
                    
                }
                // ROS_WARN("**************** pass 1st Level!! ****************");
            }
            case 2:
            break;
        }

        switch(individual_action){
            case Action::waiting:{
                ROS_ERROR_THROTTLE(1, "~~~ Waiting for the switch on ~~~");
                break;
            }
            case Action::tutorial_move:{
                if(onNode){
                    // ROS_INFO("+++++++++++++++++ onNode: %d",MAP::nodeToGo);

                    //檢查odom是否在node一定範圍內
                    if(MAP::check_onNode(MAP::nodeToGo) == 0){
                        ROS_ERROR("Node misjudgment!!");
                        onNode = false;
                        continue;
                    }
                    //更新現在的node
                    MAP::nodeNow = MAP::nodeToGo;
                    odometry.x = MAP::node[MAP::nodeNow].second.first;
                    odometry.y = MAP::node[MAP::nodeNow].second.second;
                    //更新要去的node
                    auto arr = MAP::adj_list[MAP::nodeNow];
                    int max = -1;
                    for(auto it = arr.begin(); it != arr.end(); ++it)   max = (max<*it)?*it:max;
                    if(max == -1){
                        ROS_ERROR("NoWay!!");
                        break;
                    }
                    MAP::nodeToGo = max;
                    //決定方向
                    ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                    //刪除來的路徑
                    if(MAP::nodeToGo != 16 && MAP::nodeToGo != 17)
                        MAP::eraseEdge(MAP::nodeNow, MAP::nodeToGo);
                    //重製"在節點上"
                    onNode = false;

                    ROS_WARN("On %d, -> %d ; go ahead: %d",MAP::nodeNow,MAP::nodeToGo,orientation.data);

                }

                //靠近node時減速
                if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

                //節點補償
                if(MAP::nodeLoseConp())     onNode = 1;

                orientation_pub.publish(orientation);
                break;
            }
            case Action::odom_move:{
                orientation_pub.publish(orientation);
                cmd_vel_pub.publish(cmd_vel);
                break;
            }
            case Action::rotate:{
                ODOM::oriNow = orientation.data = 4;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::capture_n_detect:{
                if(_pub4 > att)   cam_mode.data = 4;
                else    cam_mode.data = 3;
                cam_pub.publish(cam_mode);

                orientation.data = -1;
                orientation_pub.publish(orientation);

                int a = 0, b = 0;
                for(int i = op; i <= op + 2; i++){
                    if(numbers.find(i) != numbers.end()){
                        if(!a){
                            a = i;      cam_flag = 0;
                        }
                        else if(!b){
                            b = i;
                            cam_flag = 1;
                        }
                        else{
                            cam_flag = 0;   
                            CAM::numbers.clear();
                        }
                    }
                }
                _a = a;     _b = b;
                _pub4 ++;
                break;
            }
            case Action::over_hurdles:{
                ROS_WARN_THROTTLE(1," \"6\" /cmd_ori: %d, %d / %d (sec)",orientation.data ,ori6State/20 + 1, time_6);
                ODOM::oriNow = orientation.data = 6;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::calibration:{
                ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                if(after_6_shift_state == 0){
                    if(ODOM::odometry.y > MAP::node[MAP::nodeNow].second.second - after_6_shift){
                        cmd_vel.linear.y = -15;
                    }else{
                        after_6_shift_state ++;
                        ROS_WARN("switch");
                    }
                }else if(after_6_shift_state == 1){
                    if(ODOM::odometry.y < MAP::node[MAP::nodeNow].second.second + after_6_shift){
                        cmd_vel.linear.y = 15;
                    }else{
                        after_6_shift_state ++;
                        ROS_WARN("switch");
                    }
                }else{
                    ROS_ERROR("can't stick the line ... QQ");
                    ODOM::oriNow = orientation.data = 0;
                    robot_state = "tutorial_move";
                }
                orientation_pub.publish(orientation);
                cmd_vel_pub.publish(cmd_vel);
                break;
            }
            case Action::binBaiYa:{
                break;
            }
            case Action::dustBox:{
                break;
            }
        }


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

bool amoungDeg(double a, double b){
    if(b == PI)     return a < b - 0.2;
    if(b >= 0)  return a < b;
    return a < b || a > PI - 0.2;
}

void runInit(ros::NodeHandle& nh){
    orientation_pub = nh.advertise<std_msgs::Int8>("/cmd_ori", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    node_sub = nh.subscribe("/node_detect",1,nodeCallback);
    cam_pub = nh.advertise<std_msgs::Int32>("/mode", 1);
    number_sub = nh.subscribe("/numbers",1,numberCallback);

    odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    // odom_sub = nh.subscribe("/realspeed",1,odomCallback);

    node_detect_pub = nh.advertise<std_msgs::Bool>("/node_detect", 1);
    laji_pub = nh.advertise<std_msgs::Int8>("/cmd_laji", 1);
    stickOnLine_sub = nh.subscribe("/stickOnLine",1,stickCallback);
    reset_sub = nh.subscribe("/reset",1,reset_callback);

    ONE.data = 1;

    nh.getParam("/freq",freq);
    nh.getParam("/tolerence",tolerence);
    nh.getParam("/decelerationZone",decelerationZone);
    nh.getParam("/nodeLoseConpDELAY",nodeLoseConpDELAY);
    nh.getParam("/start_now",start_now);
    nh.getParam("/start_togo",start_togo);
    nh.getParam("/time_6",time_6);
    nh.getParam("/time_7",time_7);
    nh.getParam("/after_6_shift",after_6_shift);
}

void nodeCallback(const std_msgs::Bool::ConstPtr& is_node){
    bool isNode = is_node->data;
    if(isNode != isNodeLast && isNode){
        onNode = true;
        ROS_WARN("onNode!");
    }
    // ROS_WARN("nodeCall, isNode:%d, isNodeLast:%d",isNode,isNodeLast);
    isNodeLast = isNode;
}
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers){
    for(auto i:the_numbers->data){
        CAM::numbers.insert(i);
    }
    _pub4 = 0;
}
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    odometry.update(ins_vel);
    if(CAM::cease)  return;
    if(!RESET::state)   return;
    ROS_INFO_THROTTLE(1, "{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d faceTo: %d",MAP::nodeNow,MAP::nodeToGo,odometry.x, odometry.y, odometry.theta, ODOM::oriNow, ODOM::faceTo);
}
void stickCallback(const std_msgs::Bool::ConstPtr& stick_){
    stick = stick_->data;
}
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    RESET::state = reset_data->data;
    if(RESET::state == 1 && robot_state == "waiting")    robot_state = "tutorial_move";
}