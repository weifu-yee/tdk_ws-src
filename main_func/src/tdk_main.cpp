#include "map.h"
#include "cammera.h"
#include "odom.h"
#include "reset.h"
#include "shooter.h"
using namespace std;

#define att 40

enum Reset{
    wait,
    all_run,
    sec_all_run,
    sec_pass_binBaiYa,
    sec_pass_baseball,
    sec_pass_badminton,
    sec_last_binBaiYa,
    sec_last_baseball,
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
    back_rotate,
    capture_n_detect,
    over_hurdles,
    calibration,
    script_binBaiYa,
    script_badminton,
    teleop      //just for debug's convenience
};

namespace sideAction{
    bool SHOOTER = false;
    bool DUSTBOX = false;
}

Reset reset_state = Reset::wait;
Reset last_reset_state = Reset::wait;
Level level_ing = Level::the_wait;
Level last_level_ing = Level::the_wait;
Action individual_action = Action::waiting;
string robot_state = "waiting";

//parameters
int odom_mode = 1;
int start_now = 31, start_togo = 0;
int sec_start_now = 32, sec_start_togo = 13;
double after_6_shift = 20;
double X_after_over_hurdles = 314;
double Y_shifting_after_binBaiYa = 110;
double Y_shifting_dustBox = 0;
double Y_badmiton_start_shift_right = 12.5;

//variables
int midway_reset_pub_0_times = 0;
bool onNode = false;
int capt_ed_times = 0;
bool rotate_ed = 0;
int cam_mode_1 = 0;
int camNum_op;
int ori6State = 0;
int ori7State = 0;
int cam_flag , _a , _b , _pub4;
double thetaToGo;
int after_6_shift_state = 0;
int stick_times = 0;
int laji_process_state = 0;
int laji_ok_state = 0;
int lajiOKLast = 1;
int pitches_state = -1;
int shooter_state = -1;
int stOKLast = 0;
bool shooter_ok = false;
int num_of_badminton = 0;
bool dis_state = false;
int disLast = 0;
int badminton_process_state = 0;
bool badminton_ok_state = false;
int badminton_okLast = 0;
int sec_move_3_process = 0;
int calibration_delay = 0;

int steal_rotate_times = 30;
int capture_rotate_times = 0;

int detectFailTimes = 0;

//variables_last
bool isNodeLast = false;

//callback data
bool stick = 0;

//constants
double degrees[] = {0, PI/2, PI, -PI/2};

//publisher
ros::Publisher orientation_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher cam_pub;
ros::Publisher node_detect_pub;
ros::Publisher cmd_laji_pub;
ros::Publisher cmd_angle_pub;
//subscriber
ros::Subscriber node_sub;
ros::Subscriber number_sub;
ros::Subscriber odom_sub;
ros::Subscriber stickOnLine_sub;
ros::Subscriber reset_sub;
ros::Subscriber laji_ok_sub;
ros::Subscriber pitches_sub;
ros::Subscriber st_ok_sub;
ros::Subscriber dis_sub;
ros::Subscriber badminton_ok_sub;

//pub msgs
std_msgs::Int8 orientation;
geometry_msgs::Twist cmd_vel;
std_msgs::Int32 cam_mode;
std_msgs::Int8 cmd_laji;
geometry_msgs::Point cmd_angle;
geometry_msgs::Twist rotate_ang;


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
    _pub4 = -2;
}
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    if(CAM::cease)  return;
    if(reset_state == Reset::wait)   return;
    if(robot_state == "waiting")    return;
    odometry.update(ins_vel);
    if(ins_vel->linear.z == 1)  ori6State = 1;
    if(ins_vel->linear.z == 2)  ori7State = 1;
    ROS_INFO_THROTTLE(0.25, "{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d faceTo: %d",MAP::nodeNow,MAP::nodeToGo,odometry.x, odometry.y, odometry.theta, orientation.data, ODOM::faceTo);
}
void stickCallback(const std_msgs::Bool::ConstPtr& stick_){
    stick = stick_->data;
    if(stick == false)  stick_times = 0;
    else    stick_times ++;
}
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    switch(reset_data->data){
        case 0:     reset_state = Reset::wait;      break;
        case 1:     reset_state = Reset::all_run;      break;
        case 2:     reset_state = Reset::sec_all_run;      break;
        case 3:     reset_state = Reset::sec_pass_binBaiYa;      break;
        case 4:     reset_state = Reset::sec_pass_baseball;      break;
        case 5:     reset_state = Reset::sec_pass_badminton;      break;
        case 6:     reset_state = Reset::sec_last_binBaiYa;      break;
        case 7:     reset_state = Reset::sec_last_baseball;      break;
        case 8:     reset_state = Reset::sec_last_badminton;      break;
    }
}
void laji_ok_callback(const std_msgs::Int8::ConstPtr& laji_ok_data){
    bool lajiOK = laji_ok_data->data;
    if(lajiOK != lajiOKLast && lajiOK){
        laji_ok_state = true;
        ROS_WARN("lajiOK!");
    }
    lajiOKLast = lajiOK;        
}
void pitches_callback(const std_msgs::Int8::ConstPtr& pitches_data){
    pitches_state = pitches_data->data;
}
void st_ok_callback(const std_msgs::Int8::ConstPtr& st_ok_data){
    int stOK = st_ok_data->data;
    if(stOK != stOKLast && stOK == 1){
        shooter_ok = true;
        ROS_WARN("shooter_ok!");
    }
    stOKLast = stOK;
}
void dis_callback(const std_msgs::Int8::ConstPtr& dis_data){
    int disNow = dis_data->data;
    if(disNow != disLast && disNow == 1){
        dis_state = true;
        ROS_WARN("dis_state = 1 !!!");
    }
    disLast = disNow;
}
void badminton_ok_callback(const std_msgs::Int8::ConstPtr& badminton_ok_data){
    int badminton_ok = badminton_ok_data->data;
    if(badminton_ok != badminton_okLast && badminton_ok == 1){
        badminton_ok_state = true;
        ROS_WARN("dis_state = 1 !!!");
    }
    badminton_okLast = badminton_ok;
}

bool amoungDeg(double a, double b){
    if(b == PI)     return a < b - 0.2;
    if(b >= 0)  return a < b;
    return a < b || a > PI - 0.2;
}
void GetParam(ros::NodeHandle& nh){
    nh.getParam("/odom_mode",odom_mode);
    nh.getParam("/freq",freq);
    nh.getParam("/tolerence",tolerence);
    nh.getParam("/decelerationZone",decelerationZone);
    nh.getParam("/nodeLoseConpDELAY",nodeLoseConpDELAY);
    nh.getParam("/start_now",start_now);
    nh.getParam("/start_togo",start_togo);
    nh.getParam("/sec_start_now",sec_start_now);
    nh.getParam("/sec_start_togo",sec_start_togo);
    nh.getParam("/after_6_shift",after_6_shift);
    nh.getParam("/X_after_over_hurdles",X_after_over_hurdles);
    nh.getParam("/Y_shifting_after_binBaiYa",Y_shifting_after_binBaiYa);
    nh.getParam("/Y_shifting_dustBox",Y_shifting_dustBox);
    nh.getParam("/Y_badmiton_start_shift_right",Y_badmiton_start_shift_right);
    nh.getParam("/steal_rotate_times",steal_rotate_times);

    if(!odom_mode){
        ROS_ERROR("^^^^^^^^^^^^^^^^^^fake_odom !!!^^^^^^^^^^^^^^^^^^");
        odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    }else{
        ROS_ERROR("^^^^^^^^^^^^^^^^^^realspeed !!!^^^^^^^^^^^^^^^^^^");
        odom_sub = nh.subscribe("/realspeed",1,odomCallback);   //realspeed
    }
}
void DebugNum(ros::NodeHandle& nh){
    nh.getParam("/firstNum1",firstNum1);
    nh.getParam("/firstNum2",firstNum2);
    nh.getParam("/secondNum1",secondNum1);
    nh.getParam("/secondNum2",secondNum2);
    nh.getParam("/thirdNum1",thirdNum1);
    nh.getParam("/thirdNum2",thirdNum2);
}

void pubSubInit(ros::NodeHandle& nh){
    orientation_pub = nh.advertise<std_msgs::Int8>("/cmd_ori", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    cam_pub = nh.advertise<std_msgs::Int32>("/mode", 1);
    node_detect_pub = nh.advertise<std_msgs::Bool>("/node_detect", 1);
    cmd_laji_pub = nh.advertise<std_msgs::Int8>("/cmd_laji", 1);
    cmd_angle_pub = nh.advertise<geometry_msgs::Point>("/cmd_angle", 1);

    odom_sub = nh.subscribe("/realspeed",1,odomCallback);   //realspeed
    // odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    number_sub = nh.subscribe("/numbers",1,numberCallback);
    node_sub = nh.subscribe("/node_detect",1,nodeCallback);
    stickOnLine_sub = nh.subscribe("/stickOnLine",1,stickCallback);
    reset_sub = nh.subscribe("/reset",1,reset_callback);
    laji_ok_sub = nh.subscribe("/laji_ok",1,laji_ok_callback);
    pitches_sub = nh.subscribe("/pitches",1,pitches_callback);
    st_ok_sub = nh.subscribe("/st_ok",1,st_ok_callback);
    dis_sub = nh.subscribe("/dis",1,dis_callback);
    badminton_ok_sub = nh.subscribe("/badminton_ok",1,badminton_ok_callback);
}
void variable_reset(void){
    level_ing = Level::the_wait;

    CAM::numbers.clear();
    Done::_first = 0;
    Done::_sec_move_1 = 0;
    Done::_binBaiYa = 0;
    Done::_sec_move_2 = 0;
    Done::_baseball = 0;
    Done::_badminton = 0;
    Done::_sec_move_3 = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    sideAction::DUSTBOX = false;
    sideAction::SHOOTER = false;

    onNode = false;
    capt_ed_times = 0;
    rotate_ed = 0;
    cam_mode_1 = 0;
    ori6State = 0;
    ori7State = 0;
    after_6_shift_state = 0;
    stick_times = 0;
    laji_process_state = 0;
    laji_ok_state = 0;
    lajiOKLast = 1;
    pitches_state = -1;
    shooter_state = -1;
    stOKLast = 0;
    shooter_ok = false;
    num_of_badminton = 0;
    dis_state = false;
    disLast = 0;
    badminton_process_state = 0;
    sec_move_3_process = 0;
    badminton_ok_state = false;
    badminton_okLast = 0;
    capture_rotate_times = 0;
    calibration_delay = 0;

    isNodeLast = false;
    stick = 0;

    cam_mode.data = 4;
    cam_pub.publish(cam_mode);
}
void reset__sec_init(){
    ODOM::oriNow = orientation.data = MAP::startPointInit(sec_start_now, sec_start_togo);
    onNode = false;
    if(sec_start_now == 14 && sec_start_togo == 17){
        Done::_sec_move_1 = true;
        Done::_sec_move_2 = true;
    }
}

//debug print functons
void Reset_print(string s){
    cout<<endl; 
    ROS_ERROR("............... Reset::%s ...............",s.c_str());
}
void Level_print(string s){
    cout<<endl; ROS_WARN("_______________ Level::%s _______________",s.c_str()); cout<<endl;
}
void Process_print(string s){
    ROS_WARN("---------------------------------");
    ROS_ERROR("%s !!!",s.c_str());
}
void Done_print(string s){
    cout<<endl;
    ROS_ERROR("*   *   *   *   *   *   *");
    ROS_WARN("Done::_%s = true!!",s.c_str());
    ROS_ERROR("*   *   *   *   *   *   *");
}


//main
int main(int argc, char **argv){
    ros::init(argc, argv, "tdk_main");
    ros::NodeHandle nh;
    MAP::buildNode();
    MAP::initBuildEdge();
    DebugNum(nh);
    CAM::initPredictNumbers();
    pubSubInit(nh);
    GetParam(nh);
    ros::Rate rate(freq);
    while(nh.ok()){
        
        //重置狀態
        switch(reset_state){
            case Reset::wait:{
                if(last_reset_state != reset_state){       //中途重置
                    cout<<endl; ROS_ERROR("################### midway reset Q_Q ###################"); cout<<endl;
                    MAP::initBuildEdge();
                    midway_reset_pub_0_times = 0;
                }

                variable_reset();
                DebugNum(nh);
                CAM::initPredictNumbers();
                // cout<<endl; ROS_INFO("%d,%d,%d,%d,%d,%d",firstNum1, firstNum2,secondNum1,secondNum2,thirdNum1,thirdNum2);   cout<<endl;


                robot_state = "waiting";
                individual_action = Action::waiting;

                if(midway_reset_pub_0_times++ > att){
                    individual_action = Action::teleop;
                }
                break;
            }
            case Reset::all_run:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("all_run");
                    ODOM::oriNow = orientation.data = MAP::startPointInit(start_now, start_togo);

                    for(int i = 0; i <= 2; i++){
                        if(ODOM::odometry.x < CAM::capt_x[i]){
                            capt_ed_times = i;
                            break;
                        }
                        else    capt_ed_times = i + 1;
                    }

                    if(start_now <= 3 || start_now == 31){
                        ori6State = 0;
                        after_6_shift_state = 0;
                    }
                    else{
                        ori6State = 1;
                        after_6_shift_state = 2;
                    }

                    if(start_now < 12 || start_now == 31)  rotate_ed = 0;
                    else rotate_ed = 1;

                    ROS_ERROR("capt_ed_times: %d, ori6State: %d, rotate_ed:%d",capt_ed_times,ori6State,rotate_ed);
                    
                    onNode = false;
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
            case Reset::sec_all_run:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_all_run");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_pass_binBaiYa:{         //動了順序for測試
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_pass_binBaiYa");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::sec_move_3;
                if(Done::_sec_move_3)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_pass_baseball:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_pass_baseball");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_pass_badminton:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_pass_badminton");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_last_binBaiYa:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_last_binBaiYa");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_last_baseball:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_last_baseball");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_last_badminton:{
                if(last_reset_state != reset_state){       //初始化
                    Reset_print("sec_last_badminton");
                    reset__sec_init();
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            
        }
        last_reset_state = reset_state;
        //執行中的階段
        switch(level_ing){
            case Level::the_wait:{
                break;
            }
            case Level::first:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("first");
                    robot_state = "tutorial_move";
                    ori6State = 0;
                }
                //開相機
                if(cam_mode_1 < att){
                    cam_mode.data = 1;
                    cam_pub.publish(cam_mode);
                    cam_mode_1 ++;
                }
                //跨坎
                if(!ori6State&& odometry.x >= 140 + 20 && 
                    (MAP::nodeNow >= 1 && MAP::nodeNow <= 3) && 
                    (MAP::nodeToGo >= 4 && MAP::nodeToGo <= 6)){   
                    robot_state = "over_hurdles";
                }
                //三次辨識
                for(int i = 0; i <= 2; i++){
                    if(capt_ed_times == i && odometry.x >= CAM::capt_x[i]){
                        if(i == 1 && !(after_6_shift_state == 3))    continue;
                        CAM::numbers.clear();       
                        camNum_op = 3*i + 1;
                        ROS_ERROR("cease ... to capture & detect!!!");
                        cam_flag = 0, _a = 0, _b = 0, _pub4 = 0;
                        if(MAP::node_y(MAP::nodeNow) == 229 && capture_rotate_times < steal_rotate_times){
                            robot_state = "steal_back_rotate";
                            individual_action = Action::back_rotate;
                            capture_rotate_times ++;
                        }
                        else if(MAP::node_y(MAP::nodeNow) == 49 && capture_rotate_times < steal_rotate_times){
                            robot_state = "steal_rotate";
                            individual_action = Action::rotate;
                            capture_rotate_times ++;
                        }
                        else{
                            robot_state = "capture_n_detect";
                            // capture_rotate_times = 0;
                            capt_ed_times++;
                        }
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
                        if(capture_rotate_times > 0){
                            if(ODOM::odometry.theta < 0){
                                individual_action = Action::rotate;
                            }else{
                                individual_action = Action::back_rotate;
                            }
                            capture_rotate_times --;
                        }else{
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
                }
                else if(robot_state == "over_hurdles"){     //跨坎
                    if(individual_action != Action::over_hurdles){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("over_hurdles");
                    }
                    individual_action = Action::over_hurdles;
                    ODOM::odometry.x = X_after_over_hurdles;
                    //跨坎補償
                    if(ori6State){
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
                        ODOM::odometry.x = X_after_over_hurdles;
                        ODOM::odometry.y = MAP::node[MAP::nodeNow].second.second;
                        after_6_shift_state = 3;
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
                    if(odometry.x < 715)    cmd_vel.linear.y = -5;
                    else    cmd_vel.linear.y = 0;
                    if(odometry.y < 370)    cmd_vel.linear.x = 15;
                    else    cmd_vel.linear.x = 0;

                    if(!(odometry.x < 715 || odometry.y < 370)){
                        Done_print("first");
                        Done::_first = true;
                    }
                }
                break;
            }
            case Level::sec_move_1:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("sec_move_1");
                }
                //偏移至第二重置框框外
                if(odometry.y < 420){
                    robot_state = "odom_move";
                    individual_action = Action::odom_move;
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = 15;
                    cmd_vel.linear.y = 0;
                }else{
                    if(robot_state != "tutorial_move"){
                        ROS_WARN("---------------------------------");
                        ROS_WARN("switch to tutorial_move");
                    }
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                    ODOM::oriNow = orientation.data = 0;
                }
                //開到13
                if(MAP::nodeNow == 13){
                    Done_print("sec_move_1");
                    robot_state = "waiting";
                    Done::_sec_move_1 = true;
                }
                break;
            }
            case Level::binBaiYa:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("binBaiYa");
                }

                if(ori7State && robot_state == "script_binBaiYa"){
                    ROS_ERROR("script_binBaiYa --- complete!!");
                    ODOM::oriNow = orientation.data = 2;
                    orientation_pub.publish(orientation);
                    ODOM::odometry.y = MAP::node_y(16) + Y_shifting_after_binBaiYa;
                }

                robot_state = "tutorial_move";
                individual_action = Action::tutorial_move;

                if(MAP::nodeNow == 16 && !ori7State){
                    robot_state = "script_binBaiYa";
                    individual_action = Action::script_binBaiYa;
                }

                if(MAP::nodeNow == 13 && MAP::nodeToGo == 14){
                    Done_print("binBaiYa");
                    robot_state = "waiting";
                    Done::_binBaiYa = true;
                }

                break;
            }
            case Level::sec_move_2:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("sec_move_2");

                    ODOM::oriNow = orientation.data = MAP::startPointInit(13, 14);
                    MAP::eraseEdge(13, 16);

                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                }

                //節點13,14旋轉
                if(MAP::nodeNow == 13 && ODOM::faceTo == 1 || MAP::nodeNow == 14){
                    if(robot_state != "rotate"){
                        thetaToGo = degrees[(ODOM::faceTo + 1) % 4];
                        ROS_WARN("---------------------------------");
                        ROS_WARN("SCRIPT::rotateCCW, faceTo: %f -> %f",degrees[ODOM::faceTo],thetaToGo);
                    }
                    robot_state = "rotate";
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
                
                //開到14
                if(MAP::nodeNow == 14 && ODOM::faceTo == 3){
                    Done_print("sec_move_2");
                    robot_state = "waiting";
                    Done::_sec_move_2 = true;
                }
                break;
            }
            case Level::baseball:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("baseball");
                }

                //14開到17      同時放下母車
                if(laji_process_state == 0){
                    sideAction::DUSTBOX = true;
                    cmd_laji.data = 1;
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                    if(MAP::nodeNow == 17){
                        laji_process_state ++;
                        ROS_WARN("---------------------------------");
                        ROS_ERROR("waiting the dustBox set up to grab");
                    }
                }
                //到17等待laji_ok
                if(laji_process_state == 1){
                    robot_state = "waiting";
                    individual_action = Action::waiting;
                    if(laji_ok_state){
                        laji_process_state ++;
                        ROS_WARN("---------------------------------");
                        ROS_ERROR("the dustBox is setted up to grab, now go ahead a little !!!");
                        laji_ok_state = false;
                    }
                }
                //往前走一小段
                if(laji_process_state == 2){
                    robot_state = "odom_move";
                    individual_action = Action::odom_move;
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = -5;
                    cmd_vel.linear.y = 0;
                    if(ODOM::odometry.y >= MAP::node_y(17) + Y_shifting_dustBox){
                        laji_process_state ++;
                        ROS_WARN("---------------------------------");
                        ROS_ERROR("waiting the dustBox pull the balls on car");
                    }
                }
                //等待倒球
                if(laji_process_state == 3){
                    robot_state = "waiting";
                    individual_action = Action::waiting;
                    cmd_laji.data = 2;

                    if(laji_ok_state){
                        ODOM::oriNow = orientation.data = 0;
                        laji_process_state ++;
                        sideAction::SHOOTER = true;
                        cmd_angle.x = 2;

                        ROS_WARN("---------------------------------");
                        ROS_ERROR("put down the box and go !!!");
                        laji_ok_state = false;
                    }
                }
                //放下箱子，拖著走
                if(laji_process_state == 4){
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                    cmd_laji.data = 1;
                }

                if(laji_process_state == 4 && MAP::nodeNow == 14){
                    Done_print("baseball");
                    robot_state = "waiting";
                    individual_action = Action::waiting;        //to Delete
                    Done::_baseball = true;
                }
                break;
            }
            case Level::badminton:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("badminton");
                    num_of_badminton = 0;
                    badminton_process_state = 0;
                    MAP::nodeNow = 14;
                    MAP::nodeToGo = 15;
                    ODOM::oriNow = orientation.data = 3;
                }
                if(badminton_process_state == 0){
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                    if(MAP::nodeNow == 15){
                        badminton_process_state++;
                        Process_print("go right to get first badminton");
                    }
                }
                if(badminton_process_state == 1){
                    robot_state = "odom_move";
                    individual_action = Action::odom_move;
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = 15;
                    cmd_vel.linear.y = 0;
                    if(odometry.y < MAP::node_y(18)){
                        badminton_process_state++;
                        Process_print("start do the script_badminton");
                    }
                }
                if(badminton_process_state == 2){
                    robot_state = "odom_move";
                    individual_action = Action::odom_move;
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = -10;
                    cmd_vel.linear.y = 0;
                    //距離夠近，原地等待發射結束
                    if(dis_state == 1){
                        robot_state = "script_badminton";
                        individual_action = Action::script_badminton;
                        //發射結束
                        if(badminton_ok_state == 1){
                            num_of_badminton++;
                            ROS_WARN("---------------------------------");
                            ROS_WARN("the %dth badminton done, let go next badminton !!!",num_of_badminton);
                            dis_state = 0;
                            badminton_ok_state = 0;
                        }
                    }
                    //射完四顆
                    if(num_of_badminton >= 4){
                        Done_print("badminton");
                        robot_state = "waiting";
                        individual_action = Action::waiting;        //to Delete
                        Done::_badminton = true;
                    }
                }
                
                break;
            }
            case Level::sec_move_3:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("sec_move_3");
                    sec_move_3_process = 0;
                }
                if(sec_move_3_process == 0){
                    robot_state = "odom_move";
                    individual_action = Action::odom_move;
                    ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                    cmd_vel.linear.x = 15;
                    cmd_vel.linear.y = 0;
                    if(odometry.y < MAP::node_y(15)){
                        Process_print("back on node 15 -> tutorial_move");
                        sec_move_3_process++;
                    }
                }
                if(sec_move_3_process == 1){
                    ODOM::oriNow = orientation.data = MAP::cmd_ori(15, 14);
                    robot_state = "tutorial_move";
                    individual_action = Action::tutorial_move;
                }

                if(MAP::nodeNow == 14){
                    Done_print("sec_move_3");
                    Done::_sec_move_3 = true;
                }
                break;
            }
            case Level::complete:{
                if(last_level_ing != level_ing){    //初始化
                    Level_print("complete");
                }
                robot_state = "waiting";
                individual_action = Action::waiting;        //to Delete
                break;
            }
        }
        last_level_ing = level_ing;
        //獨立的動作們
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
                    odometry.x = MAP::node_x(MAP::nodeNow);
                    odometry.y = MAP::node_y(MAP::nodeNow);
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
                    if(MAP::nodeToGo != 16 && MAP::nodeToGo != 17 && MAP::nodeToGo != 15)
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
            case Action::back_rotate:{
                ODOM::oriNow = orientation.data = 5;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::capture_n_detect:{
                if(_pub4 > att || _pub4 < 0)   cam_mode.data = 4;
                else    cam_mode.data = 3;
                cam_pub.publish(cam_mode);

                ODOM::oriNow = orientation.data = -1;
                orientation_pub.publish(orientation);

                int a = 0, b = 0;
                for(int i = camNum_op; i <= camNum_op + 2; i++){
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
                if(_pub4 == -2){
                    if(!b && a != CAM::predict_numbers[capt_ed_times][0]){
                        numbers.insert(CAM::predict_numbers[capt_ed_times][0]);
                        ROS_ERROR("numbers.insert(%d);",CAM::predict_numbers[capt_ed_times][0]);
                    }
                    if(!a || a == CAM::predict_numbers[capt_ed_times][0]){
                        numbers.insert(CAM::predict_numbers[capt_ed_times][1]);
                        ROS_ERROR("numbers.insert(%d);",CAM::predict_numbers[capt_ed_times][1]);
                    }
                }
                _pub4 ++;
                break;
            }
            case Action::over_hurdles:{
                ODOM::oriNow = orientation.data = 6;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::calibration:{
                ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                if(calibration_delay++ > 10){
                    if(after_6_shift_state == 0){
                        if(ODOM::odometry.y < MAP::node_y(MAP::nodeNow) + after_6_shift){
                            cmd_vel.linear.y = 8;
                        }else{
                            after_6_shift_state ++;
                            ROS_WARN("switch");
                        }
                    }else if(after_6_shift_state == 1){
                        if(ODOM::odometry.y > MAP::node[MAP::nodeNow].second.second - after_6_shift){
                            cmd_vel.linear.y = -8;
                        }else{
                            after_6_shift_state ++;
                            ROS_WARN("calibration_done");
                        }
                    }
                }
                orientation_pub.publish(orientation);
                cmd_vel_pub.publish(cmd_vel);
                break;
            }
            case Action::script_binBaiYa:{
                ODOM::oriNow = orientation.data = 7;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::script_badminton:{
                ODOM::oriNow = orientation.data = -1;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::teleop:{
                ODOM::oriNow = orientation.data = 10;
                orientation_pub.publish(orientation);
            }
        }

        //同時進行動作們        sideAction
        if(sideAction::DUSTBOX){
            cmd_laji_pub.publish(cmd_laji);
            ROS_WARN_THROTTLE(1,"cmd_laji: %d",cmd_laji.data);
            if(Done::_baseball && laji_ok_state)    sideAction::DUSTBOX = false;
        }
        if(sideAction::SHOOTER){
            ROS_WARN_THROTTLE(1,"cmd_angle: (%f, %f, %f)",cmd_angle.x, cmd_angle.y, cmd_angle.z);
            cmd_angle_pub.publish(cmd_angle);

            //確認接收到"初次填裝"命令
            if(shooter_state == -1 && pitches_state == 0){
                ROS_ERROR("shooter switch on !!!");
                shooter_state = 0;
            }
            //等待填裝完成
            if(shooter_state == 0){
                if(shooter_ok){
                    SHOOTER::compute_angle();
                    shooter_state = 1;
                    shooter_ok = false;
                }
            }
        }


        ROS_WARN_THROTTLE(1, "robot_state: %s",robot_state.c_str());
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}