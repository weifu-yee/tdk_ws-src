#include "map.h"
#include "cam.h"
#include "odom.h"

double tolerence = 13;
double decelerationZone = 16;
double nodeLoseConpDELAY = 2;
int MAP::nodeNow = -1;
vector<pair<int, pair<double, double>>> MAP::node;     //<index, x, y>
vector<set<int>> MAP::adj_list(num_of_nodes);       //adjacency_list

void MAP::buildNode(){
    std::ifstream file(buildNodeFilePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    YAML::Node pathConfig = YAML::Load(file);
    for (auto nodeElement : pathConfig) {
        auto node = nodeElement["node"];
        int index = node[0].as<int>();
        double x = node[1].as<double>();
        double y = node[2].as<double>();
        if(index == -1 && 1){
            ODOM::odometry.x = x;
            ODOM::odometry.y = y;
            continue;
        }
        if(index == -2 && 0){
            ODOM::odometry.x = x;
            ODOM::odometry.y = y;
            continue;
        }
        MAP::node.push_back(make_pair(index, make_pair(x, y)));
    }
}
void MAP::initBuildEdge(){
    auto buildEdge = [&](int u, int v){
        MAP::adj_list[u].insert(v);
        MAP::adj_list[v].insert(u);
    };
    
    std::ifstream file(initBuildEdgeFilePath);
    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return;
    }
    YAML::Node pathConfig = YAML::Load(file);
    for (auto nodeElement : pathConfig) {
        auto edge = nodeElement["initEdge"];
        int u = edge[0].as<int>();
        int v = edge[1].as<int>();
        buildEdge(u,v);
    }
}
void MAP::eraseEdge(int u, int v){
    if(v == -1)  return;
    auto find_n_erase = [&](int x, int y){
        MAP::adj_list[x].erase(y);
    };
    find_n_erase(u,v);
    find_n_erase(v,u);
}
int MAP::cmd_ori(int u, int v){
    if(v == -1)  return 0;
    if(u == 13 && v == 12)  return 0;
    double ux = MAP::node[u].second.first;
    double uy = MAP::node[u].second.second;
    double vx = MAP::node[v].second.first;
    double vy = MAP::node[v].second.second;
    int ori;
    if(ux > vx)     ori = 0;
    else if(uy > vy)     ori = 1;
    else if(ux < vx)     ori = 2;
    else if(uy < vy)     ori = 3;
    else   return -1;
    return (ori + 4 - ODOM::faceTo) % 4;
}
int MAP::disToOdom(int u){
    double ux, uy;
    if(u == -1){
        ux = 60;
        uy = 180;        
    }else{
        ux = MAP::node[u].second.first;
        uy = MAP::node[u].second.second;
    }
    double x_diff = fabs(ux - ODOM::odometry.x);
    double y_diff = fabs(uy - ODOM::odometry.y);
    return (x_diff + y_diff);
}
int MAP::check_onNode(int u){
    double ux = MAP::node[u].second.first;
    double uy = MAP::node[u].second.second;
    double vx, vy;
    if(MAP::nodeNow == -1){
        vx = 60;
        vy = 180;
    }else{
        vx = MAP::node[MAP::nodeNow].second.first;
        vy = MAP::node[MAP::nodeNow].second.second;
    }
    double x_diff = fabs(ux - vx);
    double y_diff = fabs(uy - vy);
    double disNow = disToOdom(MAP::nodeNow);
    if(disNow < (x_diff + y_diff - tolerence))    return 0;
    if(disNow > (x_diff + y_diff + nodeLoseConpDELAY))    return 2;
    
    return 1;
}