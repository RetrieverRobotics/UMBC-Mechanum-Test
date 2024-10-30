/**
 * \file umbc/util.cpp
 *
 * Contains the implementaion of the VController. The VController
 * emulates user controller input through reading a formatted binary
 * file of controller inputs.
 */

#include "api.h"
#include "umbc.h"

#include <fstream>
#include <map>
#include <cstdint>
#include <string>

using namespace pros;
using namespace umbc;
using namespace std;

int32_t bound (int32_t b){
        if(b < -127){
            b = -127;
        }
        else if (b > 127) {
            b = 127;
        }
        return b;
    }