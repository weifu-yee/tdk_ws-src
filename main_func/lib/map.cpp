#include "map.h"

double freq = 20;
double tolerence = 13;
double decelerationZone = 16;
double nodeLoseConpDELAY = 2;
int MAP::nodeNow = 31;
int MAP::nodeToGo = 0;

vector<pair<int, pair<double, double>>> MAP::node;     //<index, x, y>
vector<set<int>> MAP::adj_list(num_of_nodes);       //adjacency_list
stack<int>   MAP::des_baseball;

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
void MAP::initDesBaseball(){
    while(!MAP::des_baseball.empty())   MAP::des_baseball.pop();
    MAP::des_baseball.push(25);
    MAP::des_baseball.push(26);
    MAP::des_baseball.push(27);
    MAP::des_baseball.push(28);
}

double MAP::node_x(int u){
    return MAP::node[u].second.first;
}
double MAP::node_y(int u){
    return MAP::node[u].second.second;
}
int MAP::startPointInit(int now,int togo){
    MAP::nodeNow = now;
    MAP::nodeToGo = togo;
    ODOM::odometry.x = MAP::node_x(now);
    ODOM::odometry.y = MAP::node_y(now);
    
    if(now == 13 && togo == 16);
    else if(now == 14 && togo == 17);
    else    MAP::eraseEdge(now, togo);
    
    ODOM::faceTo = 0;
    if(togo >= 13 && togo <= 30){
        ODOM::faceTo = 1;
    }
    if(now >= 14 && now <= 30){
        ODOM::faceTo = 3;
    }

    ODOM::odometry.theta = PI/2*ODOM::faceTo;

    return MAP::cmd_ori(now, togo);
}
void MAP::eraseEdge(int u, int v){
    auto find_n_erase = [&](int x, int y){
        MAP::adj_list[x].erase(y);
    };
    find_n_erase(u,v);
    find_n_erase(v,u);
}
int MAP::cmd_ori(int u, int v){
    if(u == 12 && v == 13)  return 0;
    double ux = MAP::node[u].second.first;
    double uy = MAP::node[u].second.second;
    double vx = MAP::node[v].second.first;
    double vy = MAP::node[v].second.second;
    int ori;
    if(ux < vx)     ori = 0;
    else if(uy < vy)     ori = 1;
    else if(ux > vx)     ori = 2;
    else if(uy > vy)     ori = 3;
    else   return -1;
    return (ori + 4 - ODOM::faceTo) % 4;
}
int MAP::disToOdom(int u){
    double ux, uy;
    ux = MAP::node_x(u);
    uy = MAP::node_y(u);
    double x_diff = fabs(ux - ODOM::odometry.x);
    double y_diff = fabs(uy - ODOM::odometry.y);
    return (x_diff + y_diff);
}
int MAP::check_onNode(int u){
    double ux = MAP::node_x(u);
    double uy = MAP::node_y(u);
    double vx, vy;
    vx = MAP::node_x(MAP::nodeNow);
    vy = MAP::node_y(MAP::nodeNow);
    double x_diff = fabs(ux - vx);
    double y_diff = fabs(uy - vy);
    double disNow = disToOdom(MAP::nodeNow);
    if(disNow < (x_diff + y_diff - tolerence))    return 0;
    if(disNow > (x_diff + y_diff + nodeLoseConpDELAY)){
        if(MAP::nodeNow == 16 && u == 13 && ODOM::odometry.y > MAP::node_y(16)) ;
        else        return 2;
    }
    
    return 1;
}
bool MAP::nodeLoseConp(){
    if(MAP::check_onNode(MAP::nodeToGo) == 2)   return 1;
    return 0;
}