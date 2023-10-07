#ifndef _MAP_H_
#define _MAP_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <set>
#include <cmath>
#include <stdbool.h>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"

#include "cammera.h"
#include "odom.h"
// #define buildNodeFilePath "/home/ditrobotics/tdk_ws/src/main_func/params/buildNode.yaml"
// #define initBuildEdgeFilePath "/home/ditrobotics/tdk_ws/src/main_func/params/initBuildEdge.yaml"
#define buildNodeFilePath "/root/tdk_ws/src/main_func/params/buildNode.yaml"
#define initBuildEdgeFilePath "/root/tdk_ws/src/main_func/params/initBuildEdge.yaml"
#define num_of_nodes 18

using namespace std;

extern double freq;
extern double tolerence;
extern double decelerationZone;
extern double nodeLoseConpDELAY;

namespace MAP{
    extern int nodeNow, nodeToGo;
    extern vector<pair<int, pair<double, double>>> node;    //<index, x, y>
    extern vector<set<int>> adj_list;     //adjacency_list
    void buildNode();
    void initBuildEdge();
    int startPointInit(int now,int togo);
    void eraseEdge(int u, int v);
    int cmd_ori(int u, int v);
    int disToOdom(int u);
    int check_onNode(int u);
    bool nodeLoseConp();
}

using namespace MAP;

#endif