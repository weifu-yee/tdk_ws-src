#ifndef _CAMMERA_H_
#define _CAMMERA_H_

#include "map.h"

extern int firstNum1;
extern int firstNum2;
extern int secondNum1;
extern int secondNum2;
extern int thirdNum1;
extern int thirdNum2;

namespace CAM{
    extern std::set<int> numbers;
    extern bool cease;
    extern double capt_x[];
    extern std::vector<std::vector<int>> predict_numbers;
    void initPredictNumbers(void);
    void what_to_insert(int t, int a, int b);
    void midway_reset_debug();
    void what_to_erase(int a, int b);
}
using namespace CAM;

#endif