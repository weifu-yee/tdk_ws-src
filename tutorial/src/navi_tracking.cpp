#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#define pi 3.14159265359
#define width 48.0
#define length 41.5

// Sub : tracker_data, ori
// Pub : node_detect, vel

//base arguments
int black = 0,nonblack = 1;
size_t i;
double phy_maxMS = 0;
double slowMS = 0;

//main_function arguments --in ROS
int oriSub_ed = -1;
int ori=-1, prev_ori = -1;
std_msgs::Bool node_point;
std_msgs::Bool stick;

bool PID_mode, EX_mode;
bool _PID_mode;
double V = 0, vel_limit = 0;
double u_d = 0, u_theta = 0, u = 0;
geometry_msgs::Twist vel;
//tracker arguments --from Arduino
double Err_d,Err_theta;
int8_t std_tracker_data[20],temp[20];
// int8_t weight_array[20] = {5,2,0,-2,-5,-10,0,0,0,-10,-5,-2,0,2,5,10,0,0,0,10};
int8_t weight_array[20] = {2,1,0,-1,-2,-3,0,0,0,-3,-2,-1,0,1,2,3,0,0,0,3};


// 1. tracker data standardrize   robot coordinate convert to tracker coordinate
class Tracker {
public:
    Tracker(ros::NodeHandle& node_handle) : nh(node_handle) {
        sub_tracker = nh.subscribe<std_msgs::Int8MultiArray>("tracker_data", 10, &Tracker::tracker_callback, this);
        sub_ori = nh.subscribe<std_msgs::Int8>("cmd_ori", 10, &Tracker::ori_callback, this);
    }
    std::vector<int8_t> tracker_data;

    void tracker_callback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
        tracker_data = msg->data;
    } 
    void ori_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        oriSub_ed = msg->data;
    }
    void tracker_data_std(){
        
        for (i = 0; i < tracker_data.size(); ++i) {
            temp[i] = tracker_data[i];}
        // 放temp就修好了 '_'
        for (i = 0; i < 4; ++i){for(size_t j= 0; j < 5; ++j){
            if(i-ori < 0){
                std_tracker_data[(i-ori+4)%4*5+j]=temp[i*5+j];
            }
            else{
                std_tracker_data[(i-ori)%4*5+j]=temp[i*5+j];
            }
        }}
    }
    
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_tracker;  //tracker data
    ros::Subscriber sub_ori;   //map
};

// 2. node detect  pub : node_point
std_msgs::Bool node_detect(){
    std_msgs::Bool result;
    if(std_tracker_data[4] == black  && std_tracker_data[10] == black && (std_tracker_data[5] == black || std_tracker_data[9] == black)){
        result.data = false;
        return result;
    }
    if(std_tracker_data[0] == black  && std_tracker_data[14] == black && (std_tracker_data[19] == black || std_tracker_data[15] == black)){
        result.data = false;
        return result;
    }
    if(std_tracker_data[16] == black && std_tracker_data[17] == black && std_tracker_data[12] == black){
        result.data = true;
        return result;
    }
    if(std_tracker_data[6] == black && std_tracker_data[7] == black && std_tracker_data[12] == black){
        result.data = true;
        return result;
    }
    result.data = false;
    return result;
}

// 3. PID control --vel  
// 3-1 when on the black line
void error_cal(){
    int counter_F,counter_B,total_Err_F,total_Err_B;
    double Err_F,Err_B;
    size_t i;
    for(i = 0,PID_mode = false,  counter_F = 0,counter_B = 0,total_Err_F = 0,total_Err_B = 0; i < 20; ++i){
            if(std_tracker_data[i] == black){
                PID_mode = true;
                if(i <= 5 || i ==19){
                    total_Err_F += weight_array[i];
                    counter_F++;
                }
                if(i >= 9 && i <= 15){
                    total_Err_B += weight_array[i];
                    counter_B++;
        }}}
        if(counter_F == 0){
            Err_F = (double) total_Err_F;
        }
        else{
            Err_F = (double) total_Err_F / counter_F;
        }
        
        if(counter_B == 0){
            Err_B = (double) total_Err_B;
        }
        else{
            Err_B = (double) total_Err_B / counter_B;
        }
        Err_d = (Err_F + Err_B) / 2;
        Err_theta = atan((Err_F - Err_B) / 7);
}
double PID_control(double error, double kp, double ki, double kd){
    double u,differential;
    static double prev_error = 0, integral = 0;
    integral += error;
    if (ki * integral > 1) integral = 1/ki;
    else if (ki * integral < -1) integral = -1/ki;
    differential = error - prev_error;
    prev_error = error;

    u = kp * error + ki * integral + kd * differential;
    if (u > vel_limit) {u = vel_limit;}
    else if (u < -vel_limit) {u = -vel_limit;}

    return u;
}
// 3-2 when overshoot
double node_overshoot_logic(){
    //turn left
    if(prev_ori == (ori+3)%4){
        return slowMS;
    }
    //turn right
    else if(prev_ori == (ori+1)%4){
        return -slowMS;
    }
    return 0;
}

// tools
bool detectfallingEdge(bool current) {
    static bool previous = false;
    bool temp = !current && previous;
    previous = current;
    return temp;
}
bool detectRisingEdge(bool current) {
    static bool previous = false;
    bool temp = current && !previous;
    previous = current;
    return temp;
}
void PIDMODE(){
    for(i = 0,_PID_mode = false; i < 20; ++i){
        if(std_tracker_data[i] == black){
            _PID_mode = true;
        }
    }
}

//main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "navi_tracking");
    ros::NodeHandle nh;
    Tracker tracker(nh);
    ros::Publisher pub_node = nh.advertise<std_msgs::Bool>("node_detect", 10);;   //map
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("vel",10);
    ros::Publisher pub_stickOnLine = nh.advertise<std_msgs::Bool>("stickOnLine", 10);

    double span=0.05;
    
    nh.getParam("/span",span);
    nh.getParam("/black",black);
    nh.getParam("/nonblack",nonblack);
    nh.getParam("/phy_maxMS",phy_maxMS);
    nh.getParam("/slowMS",slowMS);

    double vkp = 0,vki = 0,vkd = 0,wkp = 0,wki = 0,wkd = 0;
    int last_ori = -1;
    int ori_old = 0;
    double _phy_maxMS;

    while (ros::ok()) {
        ros::spinOnce();
        ori = oriSub_ed;
        
        nh.getParam("vkp",vkp);
        nh.getParam("vki",vki);
        nh.getParam("vkd",vkd);

        nh.getParam("wkp",wkp);
        nh.getParam("wki",wki);
        nh.getParam("wkd",wkd);

        nh.getParam("/V",V);
        nh.getParam("/vel_limit",vel_limit);

        if(ori >= 0 && ori < 4 || ori == -2 || ori == 10){
            if(ori == -2){
                _phy_maxMS = slowMS;
                ori = last_ori;
            }else if(ori == -1){
                ori = last_ori;
            }else{
                _phy_maxMS = phy_maxMS;
            }


            
            tracker.tracker_data_std();
            node_point = node_detect();

            if(ori_old != ori){
                prev_ori = ori_old;
                ori_old = ori;
            }

            error_cal();
            u_d = -PID_control(Err_d,vkp,vki,vkd);
            u_theta = -PID_control(Err_theta,wkp,wki,wkd);

            // ROS_INFO("Err_d: %.1lf,Err_theta: %.5lf,u_d: %.1lf,u_theta: %.1lf",Err_d,Err_theta,u_d,u_theta);

            double VX = V*cos(((double)ori)*0.5*pi) - u_d*sin(((double)ori)*0.5*pi);
            double VY = V*sin(((double)ori)*0.5*pi) + u_d*cos(((double)ori)*0.5*pi);

//
            if(!PID_mode && prev_ori != -1){
                u_d = node_overshoot_logic();
                u_theta = 0;
                VX = -u_d*sin(((double)ori)*0.5*pi);
                VY = u_d*cos(((double)ori)*0.5*pi);
                // ROS_INFO("overshoot: prev_ori = %d , ori = %d; u_d: %.3lf, VX: %.3lf, VY: %.3lf",prev_ori,ori,u_d,VX,VY);
            }
//

            double W = u_theta;
            auto motorSpeed = [&](int i){
                if(i == 0)  return VX + VY + W*0.5*(width + length);
                if(i == 1)  return VX - VY - W*0.5*(width + length);
                if(i == 2)  return VX + VY - W*0.5*(width + length);
                return VX - VY + W*0.5*(width + length);
            };
            double maxMS = -1;
            for(int i = 0; i < 4; i++){
                double MS = fabs(motorSpeed(i));
                if(maxMS < MS)   maxMS = MS;
            }
            double ratioMS = 1;
            if(maxMS > _phy_maxMS)   ratioMS = _phy_maxMS / maxMS;
            vel.linear.x = VX * ratioMS;
            vel.linear.y = VY * ratioMS;
            vel.angular.z = W * ratioMS;

            pub_node.publish(node_point);
            pub_vel.publish(vel);

            if(ori == 10){                
                if(PID_mode)    stick.data = 1;
                else    stick.data = 0;
                pub_stickOnLine.publish(stick);
            }
        }

        // tracker_data_std();
        // PIDMODE();
        // if(_PID_mode)    stick.data = 1;
        // else    stick.data = 0;
        // pub_stickOnLine.publish(stick);
        // ROS_WARN("stick.data: %d",stick.data);

        last_ori = ori;
        ros::Duration(span).sleep();
    }

    return 0;
}