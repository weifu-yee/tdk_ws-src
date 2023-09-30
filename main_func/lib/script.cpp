#include "script.h"

ros::Publisher laji_pub = nh.advertise<std_msgs::Int8>("/laji", 1);


void SCRIPT::firstLevel(ros::NodeHandle& nh){
    ros::Rate rate(20);
    ROS_INFO("On -1, -> 0 ; go ahead: 0");
    int cmdori_6_times = 0;
    while(nh.ok() && MAP::nodeNow < 13){
        cam_pub.publish(cam_mode);
        ros::spinOnce();

        if(nodeNow == 12 && !rotate_ed){
            while(nh.ok() && odometry.theta < PI/2){
            //逆時針轉90度
                ros::spinOnce();
                ODOM::oriNow = orientation.data = 4;
                orientation_pub.publish(orientation);
                rate.sleep();
                ROS_INFO("rotating");
            }
            rotate_ed = 1;
            ODOM::oriNow = orientation.data = 0;
            ROS_INFO("rotate_ed");
        }

        if(onNode){
            //檢查odom是否在node一定範圍內
            if(MAP::check_onNode(nodeToGo) == 0){
                ROS_INFO("Node misjudgment!!");
                onNode = false;
                continue;
            }
            //第一次辨識
            if(nodeNow == -1){
                CAM::capture_n_detect(1, cam_pub, orientation_pub, nh);
                capt_ed_times ++;
            }
            //更新現在的node
            nodeNow = nodeToGo;
            odometry.x = MAP::node[nodeNow].second.first;
            odometry.y = MAP::node[nodeNow].second.second;
            //更新要去的node
            auto arr = MAP::adj_list[nodeNow];
            int max = -1;
            for(auto it = arr.begin(); it != arr.end(); ++it)   max = (max<*it)?*it:max;
            if(max == -1){
                ROS_INFO("NoWay!!");
                return;
            }
            nodeToGo = max;

            ODOM::oriNow = orientation.data = MAP::cmd_ori(nodeToGo, nodeNow);
            MAP::eraseEdge(nodeToGo, nodeNow);
            onNode = false;

            ROS_INFO("On %d, -> %d ; go ahead: %d",nodeNow,nodeToGo,orientation.data);

        }

        if(ODOM::slow(nodeToGo))     orientation.data = -2;
        if(odometry.x >= 140 + 20 && cmdori_6_times < 600){
            orientation.data = 6;
            cmdori_6_times++;
        }else if(cmdori_6_times == 600){
            // while(){
                //左移右移 追到線
            // }
            orientation.data = 0;
            cmdori_6_times++;
            odometry.x = 300;
        }

        orientation_pub.publish(orientation);
        if(MAP::check_onNode(nodeToGo) == 2){
            if(nodeToGo - nodeNow == 3 && nodeToGo > 3 && nodeToGo < 7)
                ROS_INFO("no conp");
            else nodeLoseConp = 1;
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
        else if(odometry.x >= thirdCapt && nodeNow > 3 && capt_ed_times == 2){
            CAM::capture_n_detect(7, cam_pub, orientation_pub, nh);
            capt_ed_times++;
            cam_mode.data = 2;
        }
        // else if(odometry.x >= 500 && capt_ed_times == 3){
        //     cam_mode.data = 2;
        //     capt_ed_times++;
        // }

        // ROS_INFO("On %d, -> %d ; go ahead: %d",nodeNow,nodeToGo,orientation.data);
        // ROS_INFO("go ahead: %d",orientation.data);

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