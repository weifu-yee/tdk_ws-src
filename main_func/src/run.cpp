#include "map.h"
#include "cam.h"
#include "odom.h"
#include "reset.h"
using namespace std;

//global vars
int start_now = -1, start_togo = 0;
bool isNodeLast = false;
bool onNode = false;
double xNow, xLast = -1;
bool nodeLoseConp = 0;
int capt_ed_times = 0;
bool rotate_ed = 0;
int time_6 = 30, time_7 = 15;
double after_6_shift = 40;
bool stick = 0;
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

//scripts
namespace SCRIPT{
    void firstLevel(ros::NodeHandle& nh);
    void binBaiYa(ros::NodeHandle& nh);
    void dustBox(ros::NodeHandle& nh);
    void badminton(ros::NodeHandle& nh);
    
    void overHurdles(ros::NodeHandle& nh, int& ori6State);
    void rotateCCW(ros::NodeHandle& nh);
    void from_A_To_B(ros::NodeHandle& nh, int A, int B);

    void testLine(ros::NodeHandle& nh);
}

//initialization
void nh_Init(ros::NodeHandle& nh);

//Callback
void nodeCallback(const std_msgs::Bool::ConstPtr& is_node);
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers);
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel);
void stickCallback(const std_msgs::Bool::ConstPtr& sitck_);
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data);

//main
int main(int argc, char **argv){
    ros::init(argc, argv, "main_func");
    ros::NodeHandle nh;

    nh_Init(nh);

    MAP::buildNode();
    MAP::initBuildEdge();

    ODOM::oriNow = orientation.data = MAP::startPointInit(start_now,start_togo);
    ros::Rate rate(freq);

    while(nh.ok()){
        switch(RESET::state){
            case 0:
                while(nh.ok() && !RESET::state){
                    ros::spinOnce();
                    ROS_ERROR("~~~ Waiting for the switch on ~~~");
                    rate.sleep();
                }
                break;
            case 1:
                SCRIPT::firstLevel(nh);
                ROS_WARN("**************** pass 1st Level!! ****************");
            case 2:            
                SCRIPT::from_A_To_B(nh, -2, 13);
                SCRIPT::binBaiYa(nh);
                ROS_WARN("**************** pass binBaiYa!! ****************");
            case 3:
                SCRIPT::from_A_To_B(nh, 13, 14);
                SCRIPT::rotateCCW(nh);
                SCRIPT::rotateCCW(nh);
                SCRIPT::dustBox(nh);
                ROS_WARN("**************** pass dustBox!! ****************");
            case 4:
                SCRIPT::badminton(nh);
                ROS_WARN("**************** pass badminton!! ****************");
            default:
                ROS_ERROR("**************** Allpass!! ****************");
                RESET::state = 0;
                break;
        }
        rate.sleep();
    }
    return 0;
}

//scripts
void SCRIPT::firstLevel(ros::NodeHandle& nh){
    ROS_WARN("---------------------------------");
    ROS_WARN("\n\nFirstLevel:\tOn %d, -> %d ; go ahead: %d\n",start_now,start_togo,ODOM::oriNow);

    ros::Rate rate(freq);
    cam_mode.data = 1;

    int ori6State = 0;
    if(MAP::nodeNow > 3 || MAP::nodeNow == -2)    ori6State = time_6*20;
    bool secRESET = false;
    while(nh.ok() && RESET::state && !secRESET){
        // std::cout<<"secRESET:"<<secRESET<<" rotate_ed:"<<rotate_ed<<" ODOM::faceTo:"<<ODOM::faceTo<<endl;
        cam_pub.publish(cam_mode);

        //在node上
        if(onNode){
            //檢查odom是否在node一定範圍內
            if(MAP::check_onNode(MAP::nodeToGo) == 0){
                ROS_ERROR("Node misjudgment!!");
                onNode = false;
                continue;
            }
            //第一次辨識
            if(MAP::nodeNow == -1){
                CAM::capture_n_detect(1, cam_pub, orientation_pub, nh);
                capt_ed_times ++;
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
                return;
            }
            MAP::nodeToGo = max;
            //發布方向
            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
            //刪除來的路徑
            MAP::eraseEdge(MAP::nodeNow, MAP::nodeToGo);
            //重製"在節點上"
            onNode = false;

            ROS_WARN("On %d, -> %d ; go ahead: %d",MAP::nodeNow,MAP::nodeToGo,orientation.data);

        }

        //靠近node時減速
        if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

        //跨坎劇本
        SCRIPT::overHurdles(nh, ori6State);

        // publish /cmd_ori
        orientation_pub.publish(orientation);
        
        //節點補償
        if(MAP::nodeLoseConp())     node_detect_pub.publish(ONE);

        //逆時針轉90度
        if(MAP::nodeNow == 12 && !rotate_ed){
            SCRIPT::rotateCCW(nh);
            rotate_ed = 1;
        }

        //第二重製區偏移
        if(rotate_ed && (odometry.x < 710 || odometry.y < 370)){
            ROS_WARN("---------------------------------");
            ROS_WARN("sec RESET shifting");
            while(nh.ok() && RESET::state){
                ros::spinOnce();
                ODOM::oriNow = orientation.data = 10;  //不讓comm_vel發布
                if(odometry.x < 710)    cmd_vel.linear.y = -2;
                else    cmd_vel.linear.y = 0;
                if(odometry.y < 370)    cmd_vel.linear.x = 15;
                else    cmd_vel.linear.x = 0;

                if(odometry.x >= 710 && odometry.y >= 370){
                    odometry.x = 710;
                    odometry.y = 370;
                    secRESET = true;
                    ROS_WARN("secRESET done!!");
                    break;
                }

                orientation_pub.publish(orientation);
                cmd_vel_pub.publish(cmd_vel);
                rate.sleep();
            }
        }

        //第二次辨識
        if(odometry.x >= secondCapt && capt_ed_times == 1){
            CAM::capture_n_detect(4, cam_pub, orientation_pub, nh);
            capt_ed_times++;
        }
        //第三次辨識
        if(odometry.x >= thirdCapt && MAP::nodeNow > 3 && capt_ed_times == 2){
            CAM::capture_n_detect(7, cam_pub, orientation_pub, nh);
            capt_ed_times++;
            cam_mode.data = 2;
        }

        //20Hz
        ros::spinOnce();
        rate.sleep();
    }
}
void SCRIPT::binBaiYa(ros::NodeHandle& nh){
    ROS_WARN("---------------------------------");
    ROS_WARN("binBaiYa");
    ros::Rate rate(freq);
    
    int cmdori_7_times = 0;
    while(nh.ok() && RESET::state && MAP::nodeToGo != 14){
        ros::spinOnce();
        //在node上
        if(onNode){
            //檢查odom是否在node一定範圍內
            if(MAP::check_onNode(MAP::nodeToGo) == 0){
                ROS_WARN("Node misjudgment!!");
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
                return;
            }
            MAP::nodeToGo = max;

            //發布方向
            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
            //刪除來的路徑
            MAP::eraseEdge(MAP::nodeNow, MAP::nodeToGo);
            //重製"在節點上"
            onNode = false;

            ROS_WARN("On %d, -> %d ; go ahead: %d",MAP::nodeNow,MAP::nodeToGo,orientation.data);
        }

        if(MAP::nodeNow == 16 && cmdori_7_times == 0){
            int k = 0, klast = -1;
            while(nh.ok() && RESET::state && cmdori_7_times++ < time_7*20){
                if(cmdori_7_times < 40){
                    orientation.data = 7;
                }
                else    orientation.data = -1;
                orientation_pub.publish(orientation);
                k = cmdori_7_times/20;
                if(k != klast){
                    ROS_WARN(" \"7\" /cmd_ori: %d, %d / %d (sec)",orientation.data ,cmdori_7_times/20, time_7);
                    klast = k;
                }
                rate.sleep();
            }
            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
        }

        //靠近node時減速
        if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

        //節點補償
        if(MAP::nodeLoseConp())     node_detect_pub.publish(ONE);
        
        //publish /cmd_ori
        orientation_pub.publish(orientation);

        //20Hz
        rate.sleep();
    }
}
void SCRIPT::from_A_To_B(ros::NodeHandle& nh, int A, int B){
    ROS_WARN("---------------------------------");
    ROS_WARN("SCRIPT::from_%d_To_%d",A,B);
    ODOM::oriNow = orientation.data = MAP::startPointInit(A,B);
    if(A == -2)     MAP::eraseEdge(12, 13);
    if(A == 14)     MAP::eraseEdge(14, 13);

    ros::Rate rate(freq);
    while(nh.ok() && RESET::state && MAP::nodeNow != B){
        ros::spinOnce();

        //在node上
        if(onNode){
            //檢查odom是否在node一定範圍內
            if(MAP::check_onNode(MAP::nodeToGo) == 0){
                ROS_WARN("Node misjudgment!!");
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
                return;
            }
            MAP::nodeToGo = max;
            //發布方向
            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
            //刪除來的路徑
            if(max != 16 && max != 17){
                MAP::eraseEdge(MAP::nodeNow, MAP::nodeToGo);
            }
            //重製"在節點上"
            onNode = false;

            ROS_WARN("On %d, -> %d ; go ahead: %d",MAP::nodeNow,MAP::nodeToGo,orientation.data);
        }

        //靠近node時減速
        if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

        //節點補償
        if(MAP::nodeLoseConp())     node_detect_pub.publish(ONE);
        
        //publish /cmd_ori
        orientation_pub.publish(orientation);

        //20Hz
        rate.sleep();
    }
    ROS_WARN("Now on the node %d",B);
}
void SCRIPT::dustBox(ros::NodeHandle& nh){
    ROS_WARN("---------------------------------");
    ROS_WARN("dustBox");

    //Go left to get balls
    SCRIPT::from_A_To_B(nh, 14, 17);

    //do the script of dustBox
    ROS_WARN("---------------------------------");
    ROS_WARN("dustBox -- upper & lowerSTM");
    // cmd_laji.data = 1;
    // laji_pub.publish(cmd_laji);

    //Go Back to 14 and shoot
    SCRIPT::from_A_To_B(nh, 17, 14);

    //shoot
    ROS_WARN("---------------------------------");
    ROS_WARN("shoot -- upperSTM");
}
void SCRIPT::badminton(ros::NodeHandle& nh){
    ROS_WARN("---------------------------------");
    ROS_WARN("SCRIPT::badminton");

    //Go ahead to the badminton platform
    SCRIPT::from_A_To_B(nh, 14, 15);

    //do the script of badminton
    ROS_WARN("badminton -- upper & lowerSTM");
}
void SCRIPT::rotateCCW(ros::NodeHandle& nh){
    double degrees[] = {0, PI/2, PI, -PI/2};
    double thetaToGo = degrees[(ODOM::faceTo + 1) % 4];
    ROS_WARN("---------------------------------");
    ROS_WARN("SCRIPT::rotateCCW, faceTo: %f -> %f",degrees[ODOM::faceTo],thetaToGo);
    auto amoungDeg = [&](double a, double b){
        if(b == PI)     return a < b - 0.2;
        if(b >= 0)  return a < b;
        return a < b || a > PI - 0.2;
    };

    ros::Rate rate(freq);
    while(nh.ok() && RESET::state && amoungDeg(ODOM::odometry.theta, thetaToGo)){
        ODOM::oriNow = orientation.data = 4;
        orientation_pub.publish(orientation);
        ros::spinOnce();
        rate.sleep();
    }
    // rotate_ed = 1;
    ODOM::odometry.theta = thetaToGo;
    // ODOM::oriNow = orientation.data = 0;
    ODOM::faceTo ++;    ODOM::faceTo %= 4;
    ROS_WARN("rotate_done!!");
}
void SCRIPT::overHurdles(ros::NodeHandle& nh, int& ori6State){
    ros::Rate rate(freq);
    if(ori6State == 0 && odometry.x >= 140 + 20 && MAP::nodeNow != -1){
        int k = 0, klast = -1;
        while(nh.ok() && RESET::state && ori6State++ < time_6*20){
            if(ori6State < 40){
                ODOM::oriNow = orientation.data = 6;
            }
            // else    ODOM::oriNow = orientation.data = 0;
            else    ODOM::oriNow = orientation.data = 6;
            orientation_pub.publish(orientation);
            k = ori6State/20;
            if(k != klast){
                ROS_WARN(" \"6\" /cmd_ori: %d, %d / %d (sec)",orientation.data ,ori6State/20, time_6);
                klast = k;
            }
            // ros::spinOnce();
            rate.sleep();
        }

        //左移右移 追到線
        ROS_WARN("---------------------------------");
        ROS_WARN("right shift & left shift to cont. stick on line");
        int after_6_shift_state = 0;
        while(1){
            ros::spinOnce();
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
                break;
            }

            if(stick){
                ROS_WARN("stick on the line!!");
                break;
            }

            orientation_pub.publish(orientation);
            cmd_vel_pub.publish(cmd_vel);
            rate.sleep();
        }

        ODOM::oriNow = orientation.data = 0;
        ODOM::odometry.y = MAP::node[MAP::nodeNow].second.second;
        ODOM::odometry.x = 300;
    }
}

void SCRIPT::testLine(ros::NodeHandle& nh){
    ros::Rate rate(freq);
    int oriParam_last = -1;
    
    while(nh.ok() && RESET::state){
        ros::spinOnce();
        int oriParam = 0;


        nh.getParam("/oriParam",oriParam);
        ODOM::oriNow = orientation.data = oriParam;
        if(oriParam != oriParam_last){
            if(oriParam == 0){
                MAP::nodeToGo = 7;
                MAP::nodeNow = 4;
            }else if(oriParam == 1){
                MAP::nodeToGo = 4;
                MAP::nodeNow = 5;
            }else if(oriParam == 2){
                MAP::nodeToGo = 4;
                MAP::nodeNow = 7;
            }else if(oriParam == 3){
                MAP::nodeToGo = 5;
                MAP::nodeNow = 4;
            }
            odometry.x = MAP::node[MAP::nodeNow].second.first;
            odometry.y = MAP::node[MAP::nodeNow].second.second;
        }

        if(MAP::nodeNow != -1){
            double ux = MAP::node[MAP::nodeToGo].second.first;
            double uy = MAP::node[MAP::nodeToGo].second.second;
            double vx = MAP::node[MAP::nodeNow].second.first;
            double vy = MAP::node[MAP::nodeNow].second.second;
            double x_diff = fabs(ux - vx);
            double y_diff = fabs(uy - vy);
            if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
                ODOM::oriNow = orientation.data = -2;
        }else{
            double ux = MAP::node[0].second.first;
            double uy = MAP::node[0].second.second;
            double vx = 60;
            double vy = 180;
            double x_diff = fabs(ux - vx);
            double y_diff = fabs(uy - vy);
            if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
                ODOM::oriNow = orientation.data = -2;
        }


        orientation_pub.publish(orientation);

        oriParam_last = oriParam;
        rate.sleep();
    }
}

void nh_Init(ros::NodeHandle& nh){
    orientation_pub = nh.advertise<std_msgs::Int8>("/cmd_ori", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    node_sub = nh.subscribe("/node_detect",1,nodeCallback);
    cam_pub = nh.advertise<std_msgs::Int32>("/mode", 1);
    number_sub = nh.subscribe("/numbers",1,numberCallback);
    // odom_sub = nh.subscribe("/cmd_vel",1,odomCallback);     //fake odom
    odom_sub = nh.subscribe("/realspeed",1,odomCallback);
    node_detect_pub = nh.advertise<std_msgs::Bool>("/node_detect", 1);
    laji_pub = nh.advertise<std_msgs::Int8>("/cmd_laji", 1);
    stickOnLine_sub = nh.subscribe("/stickOnLine",1,stickCallback);
    reset_sub = nh.subscribe("/reset",1,reset_callback);

    ONE.data = 1;

    nh.getParam("/freq",freq);
    nh.getParam("/tolerence",tolerence);
    nh.getParam("/decelerationZone",decelerationZone);
    nh.getParam("/nodeLoseConpDELAY",nodeLoseConpDELAY);
    nh.getParam("/secondCapt",secondCapt);
    nh.getParam("/thirdCapt",thirdCapt);
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
    ROS_INFO("{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d faceTo: %d",MAP::nodeNow,MAP::nodeToGo,odometry.x, odometry.y, odometry.theta, ODOM::oriNow, ODOM::faceTo);
}
void stickCallback(const std_msgs::Bool::ConstPtr& stick_){
    stick = stick_->data;
}
void reset_callback(const std_msgs::Int64::ConstPtr& reset_data){
    RESET::state = reset_data->data;
    cout << RESET::state << endl;
}