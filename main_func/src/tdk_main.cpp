#include "map.h"
#include "cammera.h"
#include "odom.h"
#include "reset.h"
using namespace std;

#define att 20

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
int odom_mode = 1;
int odom_mode_last = 1;
int start_now = 31, start_togo = 0;
bool isNodeLast = false;
bool onNode = false;
double xNow, xLast = -1;
bool nodeLoseConp = 0;
int capt_ed_times = 0;
bool rotate_ed = 0;
double X_after_over_hurdles = 300;
double after_6_shift = 40;
bool stick = 0;
bool secRESET = 0;
int cam_mode_1 = 0;
int ori6State = 0;
int ori7State = 0;
int op;
int cam_flag = 0, _a = 0, _b = 0, _pub4 = 0;
double degrees[] = {0, PI/2, PI, -PI/2};
double thetaToGo;
int after_6_shift_state = 0;
int stick_times = 0;

int laji_ok_state = -1;
int shooter_ok_state = -1;
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
ros::Subscriber shooter_ok_sub;
//msgs
std_msgs::Int8 orientation;
geometry_msgs::Twist cmd_vel;
std_msgs::Int32 cam_mode;
std_msgs::Int8 cmd_laji;
geometry_msgs::Twist rotate_ang;

//initialization
void runInit(ros::NodeHandle& nh);

//Callback
void nodeCallback(const std_msgs::Bool::ConstPtr& is_node);
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers);
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel);
void stickCallback(const std_msgs::Bool::ConstPtr& sitck_);
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data);
void laji_ok_callback(const std_msgs::Int8::ConstPtr& laji_ok_data);
void shooter_ok_callback(const std_msgs::Int8::ConstPtr& shooter_ok_data);

bool amoungDeg(double a, double b);
void GetParam(ros::NodeHandle& nh){
    nh.getParam("/odom_mode",odom_mode);
    nh.getParam("/freq",freq);
    nh.getParam("/tolerence",tolerence);
    nh.getParam("/decelerationZone",decelerationZone);
    nh.getParam("/nodeLoseConpDELAY",nodeLoseConpDELAY);
    nh.getParam("/start_now",start_now);
    nh.getParam("/start_togo",start_togo);
    nh.getParam("/after_6_shift",after_6_shift);
    nh.getParam("/X_after_over_hurdles",X_after_over_hurdles);
    if(!odom_mode){
        ROS_ERROR("^^^^^^^^^^^^^^^^^^fake_odom !!!^^^^^^^^^^^^^^^^^^");
        odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    }else{
        ROS_ERROR("^^^^^^^^^^^^^^^^^^realspeed !!!^^^^^^^^^^^^^^^^^^");
        odom_sub = nh.subscribe("/realspeed",1,odomCallback);   //realspeed
    }
}

//main
int main(int argc, char **argv){
    ros::init(argc, argv, "tdk_main");
    ros::NodeHandle nh;
    MAP::buildNode();
    MAP::initBuildEdge();
    runInit(nh);
    GetParam(nh);
    ros::Rate rate(freq);
    while(nh.ok()){

        //重置狀態
        switch(reset_state){
            case Reset::wait:{
                if(last_reset_state != reset_state){       //中途重置
                    cout<<endl; ROS_ERROR("\nmidway reset Q_Q\n"); cout<<endl;
                    MAP::initBuildEdge();
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

                    ODOM::oriNow = orientation.data = MAP::startPointInit(start_now, start_togo);
                    MAP::eraseEdge(start_now, start_togo);

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
                    cout<<endl; ROS_ERROR("Reset::sec_all_run"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_pass_binBaiYa:{
                if(last_reset_state != reset_state){       //初始化
                    cout<<endl; ROS_ERROR("Reset::sec_pass_binBaiYa"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::badminton;
                if(Done::_badminton)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_pass_baseball:{
                if(last_reset_state != reset_state){       //初始化
                    cout<<endl; ROS_ERROR("Reset::sec_pass_baseball"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
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
                    cout<<endl; ROS_ERROR("Reset::sec_pass_badminton"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
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
                    cout<<endl; ROS_ERROR("Reset::sec_last_binBaiYa"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::binBaiYa;
                if(Done::_binBaiYa)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_last_baseball:{
                if(last_reset_state != reset_state){       //初始化
                    cout<<endl; ROS_ERROR("Reset::sec_last_baseball"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
                }

                level_ing = Level::sec_move_1;
                if(Done::_sec_move_1)    level_ing = Level::sec_move_2;
                if(Done::_sec_move_2)    level_ing = Level::baseball;
                if(Done::_baseball)    level_ing = Level::complete;
                break;
            }
            case Reset::sec_last_badminton:{
                if(last_reset_state != reset_state){       //初始化
                    cout<<endl; ROS_ERROR("Reset::sec_last_badminton"); cout<<endl;

                    ODOM::oriNow = orientation.data = MAP::startPointInit(32, 13);
                    MAP::nodeNow = 12;
                    onNode = false;
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
                    cout<<endl; ROS_WARN("____________Level::first____________"); cout<<endl;
                    robot_state = "tutorial_move";
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
                    if(odometry.x < 715)    cmd_vel.linear.y = -5;
                    else    cmd_vel.linear.y = 0;
                    if(odometry.y < 370)    cmd_vel.linear.x = 15;
                    else    cmd_vel.linear.x = 0;

                    if(!(odometry.x < 715 || odometry.y < 370)){
                        ROS_WARN("---------------------------------");
                        ROS_ERROR("Done::_first = true!!");
                        Done::_first = true;
                    }
                }
                break;
            }
            case Level::sec_move_1:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::sec_move_1____________"); cout<<endl;
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
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_sec_move_1 = true!!");
                    robot_state = "waiting";
                    Done::_sec_move_1 = true;
                }
                break;
            }
            case Level::binBaiYa:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::binBaiYa____________"); cout<<endl;
                }

                if(ori7State && robot_state == "script_binBaiYa"){
                    ROS_WARN("script_binBaiYa --- complete!!");
                    ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
                    orientation_pub.publish(orientation);
                }

                robot_state = "tutorial_move";
                individual_action = Action::tutorial_move;

                if(MAP::nodeNow == 16 && !ori7State){
                    robot_state = "script_binBaiYa";
                    individual_action = Action::script_binBaiYa;
                }

                if(MAP::nodeNow == 13 && MAP::nodeToGo == 14){
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_binBaiYa = true!!");
                    robot_state = "waiting";
                    Done::_binBaiYa = true;
                }

                break;
            }
            case Level::sec_move_2:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::sec_move_2____________"); cout<<endl;

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
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_sec_move_2 = true!!");
                    robot_state = "waiting";
                    Done::_sec_move_2 = true;
                }
                break;
            }
            case Level::baseball:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::baseball____________"); cout<<endl;
                }
                cmd_laji.data = 1;
                cmd_laji_pub.publish(cmd_laji);
                
                if(1){
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_baseball = true!!");
                    robot_state = "waiting";
                    individual_action = Action::waiting;        //to Delete
                    Done::_baseball = true;
                }
                break;
            }
            case Level::badminton:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::badminton____________"); cout<<endl;
                }
                if(1){
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_badminton = true!!");
                    robot_state = "waiting";
                    individual_action = Action::waiting;        //to Delete
                    Done::_badminton = true;
                }
                break;
            }
            case Level::sec_move_3:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::sec_move_3____________"); cout<<endl;
                }
                if(1){
                    ROS_WARN("---------------------------------");
                    ROS_ERROR("Done::_sec_move_3 = true!!");
                    robot_state = "waiting";
                    individual_action = Action::waiting;        //to Delete
                    Done::_sec_move_3 = true;
                }
                break;
            }
            case Level::complete:{
                if(last_level_ing != level_ing){    //初始化
                    cout<<endl; ROS_WARN("____________Level::complete____________"); cout<<endl;
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
                ODOM::oriNow = orientation.data = 6;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::calibration:{
                ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                if(after_6_shift_state == 0){
                    if(ODOM::odometry.y > MAP::node_y(MAP::nodeNow) - after_6_shift){
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
                ODOM::oriNow = orientation.data = 7;
                orientation_pub.publish(orientation);
                break;
            }
            case Action::dustBox:{
                break;
            }
        }

        //同時進行動作們
        if(1){}
        ROS_WARN_THROTTLE(1, "robot_state: %s",robot_state.c_str());
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
    cam_pub = nh.advertise<std_msgs::Int32>("/mode", 1);
    node_detect_pub = nh.advertise<std_msgs::Bool>("/node_detect", 1);
    cmd_laji_pub = nh.advertise<std_msgs::Int8>("/cmd_laji", 1);
    cmd_angle_pub = nh.advertise<std_msgs::Int8>("/cmd_angle", 1);

    odom_sub = nh.subscribe("/realspeed",1,odomCallback);   //realspeed
    // odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    number_sub = nh.subscribe("/numbers",1,numberCallback);
    node_sub = nh.subscribe("/node_detect",1,nodeCallback);
    stickOnLine_sub = nh.subscribe("/stickOnLine",1,stickCallback);
    reset_sub = nh.subscribe("/reset",1,reset_callback);
    laji_ok_sub = nh.subscribe("/laji_ok",1,laji_ok_callback);
    shooter_ok_sub = nh.subscribe("/shooter_ok",1,shooter_ok_callback);
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
    laji_ok_state = laji_ok_data->data;
}
void shooter_ok_callback(const std_msgs::Int8::ConstPtr& shooter_ok_data){
    shooter_ok_state = shooter_ok_data->data;
}