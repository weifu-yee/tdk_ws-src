#ifndef _CAMMERA_H_
#define _CAMMERA_H_

#include "map.h"

namespace CAM{
    extern std::set<int> numbers;
    extern bool cease;
    extern double capt_x[];
    extern std::vector<std::vector<int>> predict_numbers;
    void initPredictNumbers(void);
    void what_to_erase(int a, int b);
}
using namespace CAM;

#endif