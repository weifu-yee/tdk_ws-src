#ifndef _CAM_H_
#define _CAM_H_

#include "map.h"

extern double secondCapt;
extern double thirdCapt;
extern int _pub4;

namespace CAM{
    extern set<int> numbers;
    void capture_n_detect(int op, ros::Publisher& cam_pub, 
        ros::Publisher& orientation_pub, ros::NodeHandle& nh);
    void what_to_erase(int a, int b);
}
using namespace CAM;

#endif