#include "map.h"
#include "cam.h"
#include "odom.h"
#include "reset.h"
using namespace std;

#define att 20

enum Reset{
    wait,
    all_run,
    sec_all_run,
    sec_pass_binBaiYa,
    sec_pass_dustBox,
    sec_pass_badminton,
    sec_last_binBaiYa,
    sec_last_dustBox,
    sec_last_badminton,
};
enum Level{
    the_wait,
    first,
    sec_move_1,
    binBaiYa,
    sec_move_2,
    baseball,
    badminton,
    sec_move_3,
    complete
};
namespace Done{
    bool _first = 0;
    bool _sec_move_1 = 0;
    bool _binBaiYa = 0;
    bool _sec_move_2 = 0;
    bool _baseball = 0;
    bool _badminton = 0;
    bool _sec_move_3 = 0;
};
enum Action{
    waiting,
    tutorial_move,
    odom_move,
    rotate,
    capture_n_detect,
    over_hurdles,
    calibration,
    script_binBaiYa,
    dustBox
};

Reset reset_state = Reset::wait;
Reset last_reset_state = Reset::wait;
Level level_ing = Level::the_wait;
Level last_level_ing = Level::the_wait;
Action individual_action = Action::waiting;
string robot_state = "waiting";

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
int stick_times = 0;
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

        switch(reset_state){
            case Reset::wait:{
                if(last_reset_state != reset_state){       //中途重置
                    cout<<endl; ROS_ERROR("\nmidway reset Q_Q\n"); cout<<endl;
                }

                level_ing = Level::the_wait;
                Done::_first = 0;
                Done::_sec_move_1 = 0;
                Done::_binBaiYa = 0;
                Done::_sec_move_2 = 0;
                Done::_baseball = 0;
                Done::_badminton = 0;
                Done::_sec_move_3 = 0;
                robot_state = "waiting";
                individual_action = Action::waiting;
                break;
            }
            case Reset::all_run:{
                if(last_reset_state != reset_state){       //初始化
                    cout<<endl; ROS_ERROR("Reset::all_run"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(-1, 0);
                    MAP::initBuildEdge();
                    capt_ed_times = 0;
                    ori6State = 0;
                    rotate_ed = 0;
                }

                level_ing = Level::first;
                if(Done::_first)    level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            
        }
        last_reset_state = reset_state;

        switch(level_ing){
            case Level::the_wait:{
                break;
            }
            case Level::first:{
                if(last_level_ing != level_ing){
                    cout<<endl; ROS_WARN("Level::first"); cout<<endl;
                }
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
                //跨坎補償
                //三次辨識
                for(int i = 0; i <= 2; i++){
                    if(capt_ed_times == i && odometry.x >= CAM::capt_x[i]){
                        op = 3*i + 1;
                        ROS_ERROR("cease ... to capture & detect!!!");
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
                if(ODOM::faceTo == 1 && (odometry.x < 710 || odometry.y < 370) && odometry.y > 330){
                    if(robot_state != "odom_move"){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("sec RESET shifting (odom_move)");
                    }
                    robot_state = "odom_move";
                }

                //----------------------------------------------------------------------------------------

                if(robot_state == "tutorial_move"){     //給導航走
                    individual_action = Action::tutorial_move;
                }
                else if(robot_state == "capture_n_detect"){     //拍照辨識
                    individual_action = Action::capture_n_detect;
                    
                    if(cam_flag == 1){      //辨識成功
                        CAM::what_to_erase(_a, _b);
                        ROS_ERROR("CAM::what_to_erase(%d, %d)",_a,_b);
                        ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                        orientation_pub.publish(orientation);
                        robot_state = "tutorial_move";
                        individual_action = Action::tutorial_move;
                        cam_flag = 0;
                        CAM::cease = 0;
                    }
                }
                else if(robot_state == "over_hurdles"){     //跨坎
                    individual_action = Action::over_hurdles;
                    if(!ori6State){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("over_hurdles");
                    }
                    ODOM::odometry.x = 270;
                    //跨坎補償
                    if(ori6State++ == time_6*freq){
                        robot_state = "calibration";
                        stick = 0;
                        stick_times = 0;
                    }
                }
                else if(robot_state == "calibration"){      //跨坎補償
                    if(individual_action == Action::over_hurdles){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("calibration");
                    }
                    individual_action = Action::calibration;
                    if(stick == true && stick_times > 5 || after_6_shift_state == 2){
                        if(after_6_shift_state == 2)    ROS_ERROR("can't stick the line ... QQ");
                        else    ROS_WARN("stick on the line!!");
                        ODOM::oriNow = orientation.data = 0;
                        orientation_pub.publish(orientation);
                        ODOM::odometry.x = 270;
                        ODOM::odometry.y = MAP::node[MAP::nodeNow].second.second;
                        robot_state = "tutorial_move";
                        individual_action = Action::tutorial_move;
                    }
                }
                else if(robot_state == "rotate"){       //節點12旋轉
                    individual_action = Action::rotate;
                        
                    if(!amoungDeg(ODOM::odometry.theta,thetaToGo)){
                        ODOM::faceTo ++;
                        ROS_ERROR("ODOM::faceTo: %d",ODOM::faceTo);
                        ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                        orientation_pub.publish(orientation);
                        robot_state = "tutorial_move";
                        individual_action = Action::tutorial_move;
                    }
                    
                }
                else if(robot_state == "odom_move"){        //第二重製區偏移
                    individual_action = Action::odom_move;

                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    if(odometry.x < 710)    cmd_vel.linear.y = -2;
                    else    cmd_vel.linear.y = 0;
                    if(odometry.y < 370)    cmd_vel.linear.x = 15;
                    else    cmd_vel.linear.x = 0;

                    if(!(odometry.x < 710 || odometry.y < 370)){
                        ROS_WARN("---------------------------------");
                        ROS_ERROR("Done::_first = true!!");
                        robot_state = "waiting";
                        Done::_first = true;
                    }
                }
                break;
            }
            case Level::sec_move_1:{
                //偏移至第二重置框框外
                if(odometry.y < 420){
                    if(robot_state != "odom_move"){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("shifting to stick on Line(odom_move)");
                    }
                    robot_state = "odom_move";
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = 15;
                }else{
                    if(robot_state != "tutorial_move"){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("switch to tutorial_move");
                    }
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                    ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                }
                //開到13
                if(MAP::nodeNow == 13){
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_sec_move_1 = true!!");
                    robot_state = "waiting";
                    Done::_sec_move_1 = true;
                }
                break;
            }
            case Level::binBaiYa:{
                break;
            }
            case Level::sec_move_2:{
                break;
            }
            case Level::baseball:{
                break;
            }
            case Level::badminton:{
                break;
            }
            case Level::sec_move_3:{
                break;
            }
            case Level::complete:{
                break;
            }
        }
        last_level_ing = level_ing;

        switch(individual_action){
            case Action::waiting:{
                // ROS_ERROR_THROTTLE(1, "~~~ Waiting for the switch on ~~~");
                ODOM::oriNow = orientation.data = -1;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::tutorial_move:{

                //節點補償
                if(MAP::nodeLoseConp())     onNode = 1;

                //屆節點
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
                        ROS_ERROR_THROTTLE(1,"NoWay!!");
                        ros::shutdown();
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

                    ROS_ERROR("<<<<<<< On %d(%.0f,%.0f), -> %d(%.0f,%.0f) ; go ahead: %d >>>>>>>",
                        MAP::nodeNow,MAP::node[MAP::nodeNow].second.first,MAP::node[MAP::nodeNow].second.second,
                        MAP::nodeToGo,MAP::node[MAP::nodeToGo].second.first,MAP::node[MAP::nodeToGo].second.second,
                        orientation.data);
                }

                //靠近node時減速
                if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

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

                ODOM::oriNow = orientation.data = -1;
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
                        ROS_WARN("calibration_done");
                    }
                }
                orientation_pub.publish(orientation);
                cmd_vel_pub.publish(cmd_vel);
                break;
            }
            case Action::script_binBaiYa:{
                break;
            }
            case Action::dustBox:{
                break;
            }
        }

        ROS_WARN_THROTTLE(1, "robot_state: %s",robot_state.c_str());
        // ROS_WARN("robot_state: %s",robot_state.c_str());


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
    if(CAM::cease)  return;
    if(reset_state == Reset::wait)   return;
    if(robot_state == "waiting")    return;
    odometry.update(ins_vel);
    ROS_INFO_THROTTLE(0.25, "{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d faceTo: %d",MAP::nodeNow,MAP::nodeToGo,odometry.x, odometry.y, odometry.theta, orientation.data, ODOM::faceTo);
}
void stickCallback(const std_msgs::Bool::ConstPtr& stick_){
    stick = stick_->data;
    if(stick == false)  stick_times = 0;
    else    stick_times ++;
    // ROS_ERROR("stick: %d",stick);
}
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    switch(reset_data->data){
        case 0:     reset_state = Reset::wait;      break;
        case 1:     reset_state = Reset::all_run;      break;
        case 2:     reset_state = Reset::sec_all_run;      break;
        case 3:     reset_state = Reset::sec_pass_binBaiYa;      break;
        case 4:     reset_state = Reset::sec_pass_dustBox;      break;
        case 5:     reset_state = Reset::sec_pass_badminton;      break;
        case 6:     reset_state = Reset::sec_last_binBaiYa;      break;
        case 7:     reset_state = Reset::sec_last_dustBox;      break;
        case 8:     reset_state = Reset::sec_last_badminton;      break;
    }

    // RESET::state = reset_data->data;
    // if(RESET::state == 1 && robot_state == "waiting")    robot_state = "tutorial_move";
}