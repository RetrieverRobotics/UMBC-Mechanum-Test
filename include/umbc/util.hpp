/**
 * \file umbc/vcontroller.hpp
 *
 * Contains the prototype for the VController. The VController emulates
 * user controller input through reading a formatted binary file of
 * controller inputs.
 */

#ifndef _UMBC_UTIL_HPP_
#define _UMBC_UTIL_HPP_

#include "controller.hpp"
#include "controllerinput.hpp"
#include "api.h"

#include <cstdint>
#include <queue>
#include <map>
using namespace pros;
using namespace std;

namespace umbc {
class util{
    public:
    int32_t bound (int32_t b){
        if(b < -127){
            b = -127;
        }
        else if (b > 127) {
            b = 127;
        }
        return b;
    }


};
}

#endif
