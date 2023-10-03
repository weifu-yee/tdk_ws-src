#include "reset.h"

int RESET::state = 0;
int stateLast = 0;

bool RESET::resetRisingEdge(){
    bool flag = true;
    if(!RESET::state)   flag = false;
    if(stateLast)   flag = false;
    stateLast = RESET::state;
    return flag;
}