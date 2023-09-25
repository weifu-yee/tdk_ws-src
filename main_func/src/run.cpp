#include "map.h"
#include "cam.h"
#include "odom.h"
using namespace std;
using namespace MAP;

bool isNodeLast = false;
bool onNode = false;
int nodeToGo;
double xNow, xLast = -1;
bool nodeLoseConp = 0;
int capt_ed_times = 0;

ros::Publisher orientation_pub;
ros::Subscriber node_sub;
ros::Publisher cam_pub;
ros::Subscriber number_sub;
ros::Subscriber odom_sub;
ros::Publisher node_detect_pub;
ros::Publisher laji_pub;

std_msgs::Int8 orientation;
std_msgs::Int32 cam_mode;
std_msgs::Int8 cmd_laji;

namespace SCRIPT{
    void firstLevel(ros::NodeHandle& nh);
    void binBaiYa(ros::NodeHandle& nh);
    void dustBox(ros::NodeHandle& nh);
    void testLine(ros::NodeHandle& nh);
}
void nodeCallback(const std_msgs::Bool::ConstPtr& is_node){
    bool isNode = is_node->data;
    if(isNode != isNodeLast && isNode){
        onNode = true;
        // ROS_INFO("(%.1lf, %.1lf, %.1lf) faceTo: %d",odometry.x, odometry.y, odometry.theta, orientation.data);
    }
    isNodeLast = isNode;
}
void numberCallback(const std_msgs::Int32MultiArray::ConstPtr& the_numbers){
    for(auto i:the_numbers->data){
        CAM::numbers.insert(i);
    }
}
void odomCallback(const geometry_msgs::Twist::ConstPtr& ins_vel){
    odometry.update(ins_vel);
    // ROS_INFO("{%d -> %d}  (%.1lf, %.1lf, %.1lf) oriNow: %d",MAP::nodeNow,nodeToGo,odometry.x, odometry.y, odometry.theta, ODOM::oriNow);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main_func");
    ros::NodeHandle nh;
    orientation_pub = nh.advertise<std_msgs::Int8>("/cmd_ori", 1);
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
    MAP::buildNode();
    MAP::initBuildEdge();

    orientation.data = 0;   //front
    cam_mode.data = 1;

    // ROS_INFO("On %d, -> %d",nodeNow,nodeToGo);
    // ROS_INFO("(%lf, %lf, %lf)",odometry.x, odometry.y, odometry.theta);
    // ROS_INFO("go ahead: %d",orientation.data);

    // SCRIPT::testLine(nh);

    SCRIPT::firstLevel(nh);
    ROS_INFO("pass 1st Level!!");

    while(nh.ok()){
        orientation.data = 7;
        orientation_pub.publish(orientation);
        cam_mode.data = 2;
        cam_pub.publish(cam_mode);
    }
    return 0;
}

void SCRIPT::firstLevel(ros::NodeHandle& nh){
    ros::Rate rate(20);
    
    ROS_INFO("On -1, -> 0 ; go ahead: 0");
    while(nh.ok() && MAP::nodeNow < 12){
        cam_pub.publish(cam_mode);
        ros::spinOnce();

        if(onNode){
            // nodeLoseConp = 0;
            if(MAP::check_onNode(nodeToGo) == 0){
                ROS_INFO("Node misjudgment!!");
                onNode = false;
                continue;
            }
            
            if(nodeNow == -1){
                CAM::capture_n_detect(1, cam_pub, orientation_pub, nh);
                capt_ed_times ++;
            }
            nodeNow = nodeToGo;

            odometry.x = MAP::node[nodeNow].second.first;
            odometry.y = MAP::node[nodeNow].second.second;

            auto arr = MAP::adj_list[nodeNow];
            int max = -1;
            for(auto it = arr.begin(); it != arr.end(); ++it)   max = (max<*it)?*it:max;
            if(max == -1){
                ROS_INFO("NoWay!!");
                return;
            }
            nodeToGo = max;

            ODOM::oriNow = orientation.data = MAP::cmd_ori(nodeToGo, nodeNow);
            // if(nodeNow == 1 && nodeToGo == 4
            // || nodeNow == 2 && nodeToGo == 5
            // || nodeNow == 3 && nodeToGo == 6){
            //     orientation.data = 6;
            // }

            MAP::eraseEdge(nodeToGo, nodeNow);
            onNode = false;

            ROS_INFO("On %d, -> %d ; go ahead: %d",nodeNow,nodeToGo,orientation.data);

        }
        
        // if(MAP::disToOdom(nodeToGo) < decelerationZone)    orientation.data = -2;

        if(ODOM::slow(nodeToGo))     orientation.data = -2;
        // if(nodeNow != -1){
        //     double ux = MAP::node[nodeToGo].second.first;
        //     double uy = MAP::node[nodeToGo].second.second;
        //     double vx = MAP::node[MAP::nodeNow].second.first;
        //     double vy = MAP::node[MAP::nodeNow].second.second;
        //     double x_diff = fabs(ux - vx);
        //     double y_diff = fabs(uy - vy);
        //     if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
        //         orientation.data = -2;
        // }else{
        //     double ux = MAP::node[0].second.first;
        //     double uy = MAP::node[0].second.second;
        //     double vx = 60;
        //     double vy = 180;
        //     double x_diff = fabs(ux - vx);
        //     double y_diff = fabs(uy - vy);
        //     if(MAP::disToOdom(MAP::nodeNow) > (x_diff + y_diff - decelerationZone))
        //         orientation.data = -2;
        // }
        if(odometry.x >= 160 && odometry.x <= 210){
            orientation.data = 6;
            odometry.x == 160 + 50;
        }
        orientation_pub.publish(orientation);
        if(MAP::check_onNode(nodeToGo) == 2){
            nodeLoseConp = 1;
            // ROS_INFO("nodeLoseConp");
        }else   nodeLoseConp = 0;
        
        if(nodeLoseConp){            
            std_msgs::Bool ONE;
            ONE.data = 1;
            node_detect_pub.publish(ONE);
        }


        if(odometry.x >= secondCapt && capt_ed_times == 1){
            CAM::capture_n_detect(4, cam_pub, orientation_pub, nh);
            capt_ed_times++;
        }
        if(odometry.x >= thirdCapt && capt_ed_times == 2){
            CAM::capture_n_detect(7, cam_pub, orientation_pub, nh);
            capt_ed_times++;
            cam_mode.data = 0;
        }

        // ROS_INFO("On %d, -> %d ; go ahead: %d",nodeNow,nodeToGo,orientation.data);
        // ROS_INFO("go ahead: %d",orientation.data);

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
void SCRIPT::binBaiYa(ros::NodeHandle& nh){
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
                nodeToGo = 7;
                nodeNow = 4;
            }else if(oriParam == 1){
                nodeToGo = 4;
                nodeNow = 5;
            }else if(oriParam == 2){
                nodeToGo = 4;
                nodeNow = 7;
            }else if(oriParam == 3){
                nodeToGo = 5;
                nodeNow = 4;
            }
            odometry.x = MAP::node[nodeNow].second.first;
            odometry.y = MAP::node[nodeNow].second.second;
        }

        // if(MAP::disToOdom(nodeToGo) < decelerationZone){
        //     orientation.data = -2;
        //     ODOM::oriNow = -2;
        // }

        
        // double ux = MAP::node[nodeToGo].second.first;
        // double uy = MAP::node[nodeToGo].second.second;
        // double vx = MAP::node[MAP::nodeNow].second.first;
        // double vy = MAP::node[MAP::nodeNow].second.second;
        // double x_diff = fabs(ux - vx);
        // double y_diff = fabs(uy - vy);
        // if(MAP::disToOdom(MAP::nodeNow) < (x_diff + y_diff - decelerationZone))
        //     // ODOM::oriNow = orientation.data = -2;

        if(nodeNow != -1){
            double ux = MAP::node[nodeToGo].second.first;
            double uy = MAP::node[nodeToGo].second.second;
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