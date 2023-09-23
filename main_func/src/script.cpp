#include "script.h"

ros::Publisher laji_pub = nh.advertise<std_msgs::Int8>("/laji", 1);

void SCRIPT::dustBox(){
    whlie(nh.ok)
}