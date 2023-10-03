#include "map.h"
#include "cam.h"
#include "odom.h"
#include "reset.h"
using namespace std;

#define time_7 15

//global vars
int start_now = -1, start_togo = 0;
bool isNodeLast = false;
bool onNode = false;
double xNow, xLast = -1;
bool nodeLoseConp = 0;
int capt_ed_times = 0;
bool rotate_ed = 0;
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
//msgs
std_msgs::Int8 orientation;
geometry_msgs::Twist cmd_vel;
std_msgs::Int32 cam_mode;
std_msgs::Int8 cmd_laji;
geometry_msgs::Twist rotate_ang;

//scripts
namespace SCRIPT{
    void firstLevel(ros::NodeHandle& nh);
    void binBaiYa(ros::NodeHandle& nh);
    void dustBox(ros::NodeHandle& nh);
    
    void from_A_To_B(ros::NodeHandle& nh, int A, int B);

    void testLine(ros::NodeHandle& nh);
}

//initialization
void runInit(ros::NodeHandle& nh);

//Callback
void nodeCallback(const std_msgs::Bool::ConstPtr& is_node);
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers);
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel);

//main
int main(int argc, char **argv){
    ros::init(argc, argv, "main_func");
    ros::NodeHandle nh;

    runInit(nh);

    MAP::buildNode();
    MAP::initBuildEdge();

    // orientation.data = 0;   //front
    cam_mode.data = 1;
    ODOM::oriNow = orientation.data = MAP::startPointInit(start_now,start_togo);

    switch(RESET::state){
        case 0:
            while(nh.ok())  ros::spinOnce();
        case 1:
            // SCRIPT::firstLevel(nh);
            ROS_WARN("**************** pass 1st Level!! ****************");
        case 2:            
            SCRIPT::from_A_To_B(nh, -2, 13);
            // SCRIPT::binBaiYa(nh);
            ROS_WARN("**************** pass binBaiYa!! ****************");
        case 3:
            SCRIPT::from_A_To_B(nh, 13, 14);
            SCRIPT::dustBox(nh);
            ROS_WARN("**************** pass dustBox!! ****************");
        case 4:
            SCRIPT::dustBox(nh);
            ROS_WARN("**************** pass dustBox!! ****************");
        default:
            break;
    }
    return 0;
}

//scripts
void SCRIPT::firstLevel(ros::NodeHandle& nh){
    ROS_WARN("---------------------------------");
    ROS_WARN("\n\nFirstLevel:\tOn %d, -> %d ; go ahead: %d\n",start_now,start_togo,ODOM::oriNow);

    ros::Rate rate(20);
    int ori6State = 0;
    bool secRESET = false;
    // while(nh.ok() && MAP::nodeNow < 13){
    while(nh.ok() && !secRESET){
        // std::cout<<"secRESET:"<<secRESET<<" rotate_ed:"<<rotate_ed<<" ODOM::faceTo:"<<ODOM::faceTo<<endl;
        cam_pub.publish(cam_mode);
        ros::spinOnce();

        if(MAP::nodeNow == 12 && !rotate_ed){
            while(nh.ok() && odometry.theta < PI/2){
            //逆時針轉90度
                ros::spinOnce();
                ODOM::oriNow = orientation.data = 4;
                orientation_pub.publish(orientation);
                rate.sleep();
                ROS_WARN("rotating");
            }
            rotate_ed = 1;
            ODOM::oriNow = orientation.data = 0;
            ODOM::faceTo ++;
            ROS_WARN("rotate_done!!");
        }
        //第二重製區偏移
        // while(rotate_ed && ((odometry.y >= 330 && odometry.y < 370) || (odometry.y >= 330 && odometry.y < 370))){
        while(rotate_ed && (odometry.x < 710 || odometry.y < 370)){
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
                ROS_WARN("secRESET!!");
                break;
            }

            orientation_pub.publish(orientation);
            cmd_vel_pub.publish(cmd_vel);
            ROS_WARN("shifting");
            rate.sleep();
        }
        
        
        //在node上
        if(onNode){
            //檢查odom是否在node一定範圍內
            if(MAP::check_onNode(MAP::nodeToGo) == 0){
                ROS_WARN("Node misjudgment!!");
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
        if(odometry.x >= 140 + 20 && ori6State < 600){
            orientation.data = 6;
            ori6State++;
        }else if(ori6State == 600){
            // while(){
                //左移右移 追到線
            // }
            orientation.data = 0;
            ori6State++;
            odometry.x = 300;
        }

        //publish /cmd_ori
        orientation_pub.publish(orientation);

        //節點補償
        if(MAP::check_onNode(MAP::nodeToGo) == 2){
            if(MAP::nodeToGo - MAP::nodeNow == 3 && MAP::nodeToGo > 3 && MAP::nodeToGo < 7)
                ROS_WARN("no conp");
            else nodeLoseConp = 1;
            // ROS_WARN("nodeLoseConp");
        }else   nodeLoseConp = 0;
        if(nodeLoseConp){            
            std_msgs::Bool ONE;
            ONE.data = 1;
            node_detect_pub.publish(ONE);
        }

        //第二次辨識
        if(odometry.x >= secondCapt && capt_ed_times == 1){
            CAM::capture_n_detect(4, cam_pub, orientation_pub, nh);
            capt_ed_times++;
        }//第三次辨識
        else if(odometry.x >= thirdCapt && MAP::nodeNow > 3 && capt_ed_times == 2){
            CAM::capture_n_detect(7, cam_pub, orientation_pub, nh);
            capt_ed_times++;
            cam_mode.data = 2;
        }

        //20Hz
        rate.sleep();
    }
}
void SCRIPT::binBaiYa(ros::NodeHandle& nh){
    ros::Rate rate(20);
    ROS_WARN("---------------------------------");
    ROS_WARN("binBaiYa");
    MAP::nodeNow = 12;   MAP::nodeToGo = 13;
    ODOM::oriNow = orientation.data = 0;
    // while(nh.ok() && MAP::nodeToGo == 14){
    while(nh.ok()){
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
            //發布要去的node
            ODOM::oriNow = orientation.data = MAP::cmd_ori(MAP::nodeNow, MAP::nodeToGo);
            if(max != 16)   MAP::eraseEdge(MAP::nodeNow, MAP::nodeToGo);
            onNode = false;

            ROS_WARN("On %d, -> %d ; go ahead: %d",MAP::nodeNow,MAP::nodeToGo,orientation.data);

        }

        if(MAP::nodeNow == 16){
            int cmdori_7_times = 0;
            while(nh.ok() && cmdori_7_times < time_7*20){
                if(cmdori_7_times < 40){
                    orientation.data = 7;
                }
                else    orientation.data = -1;
                orientation_pub.publish(orientation);
            }
            onNode = true;
        }

        //靠近node時減速
        if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

        //跨坎劇本
        
        orientation_pub.publish(orientation);

        //節點補償
        if(MAP::check_onNode(MAP::nodeToGo) == 2){
            if(MAP::nodeToGo - MAP::nodeNow == 3 && MAP::nodeToGo > 3 && MAP::nodeToGo < 7)
                ROS_WARN("no conp");
            else nodeLoseConp = 1;
            // ROS_WARN("nodeLoseConp");
        }else   nodeLoseConp = 0;
        if(nodeLoseConp){            
            std_msgs::Bool ONE;
            ONE.data = 1;
            node_detect_pub.publish(ONE);
        }
        rate.sleep();
    }
}
void SCRIPT::from_A_To_B(ros::NodeHandle& nh, int A, int B){
    ROS_WARN("---------------------------------");
    ROS_WARN("from %d To %d",A,B);
    ODOM::oriNow = orientation.data = MAP::startPointInit(A,B);
    if(A == -2)     MAP::eraseEdge(12, 13);

    ros::Rate rate(20);
    while(nh.ok() && MAP::nodeNow < B){
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

        //靠近node時減速
        if(ODOM::slow(MAP::nodeToGo))     orientation.data = -2;

        //節點補償
        if(MAP::check_onNode(MAP::nodeToGo) == 2){
            if(MAP::nodeToGo - MAP::nodeNow == 3 && MAP::nodeToGo > 3 && MAP::nodeToGo < 7)
                ROS_WARN("no nodeLoseConp");
            else nodeLoseConp = 1;
        }else   nodeLoseConp = 0;
        if(nodeLoseConp){            
            std_msgs::Bool ONE;
            ONE.data = 1;
            node_detect_pub.publish(ONE);
        }
        
        //publish /cmd_ori
        orientation_pub.publish(orientation);

        //20Hz
        rate.sleep();
    }
}
void SCRIPT::dustBox(ros::NodeHandle& nh){
    ros::Rate rate(20);
    while(nh.ok()){
        cmd_laji.data = 1;
        laji_pub.publish(cmd_laji);
        rate.sleep();
    }
}
void SCRIPT::testLine(ros::NodeHandle& nh){
    ros::Rate rate(20);
    int oriParam_last = -1;
    
    while(nh.ok()){
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

    nh.getParam("/tolerence",tolerence);
    nh.getParam("/decelerationZone",decelerationZone);
    nh.getParam("/nodeLoseConpDELAY",nodeLoseConpDELAY);
    nh.getParam("/secondCapt",secondCapt);
    nh.getParam("/thirdCapt",thirdCapt );
    nh.getParam("/start_now",start_now );
    nh.getParam("/start_togo",start_togo );
}

void nodeCallback(const std_msgs::Bool::ConstPtr& is_node){
    bool isNode = is_node->data;
    if(isNode != isNodeLast && isNode){
        onNode = true;
        // ROS_WARN("(%.1lf, %.1lf, %.1lf) faceTo: %d",odometry.x, odometry.y, odometry.theta, orientation.data);
    }
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
    ROS_INFO("{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d faceTo: %d",MAP::nodeNow,MAP::nodeToGo,odometry.x, odometry.y, odometry.theta, ODOM::oriNow, ODOM::faceTo);
}