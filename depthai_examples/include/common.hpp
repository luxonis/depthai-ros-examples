#ifndef COMMON_HPP_
#define COMMON_HPP_

#ifndef IS_ROS2
#include <iostream>
#include <string>
#include "ros/ros.h"
template<typename T>
static void getParamWithWarning(ros::NodeHandle& pnh, const char* key, T val) {
    bool gotParam = pnh.getParam(key, val);
    if(!gotParam) {
        std::stringstream ss;
        ss << val;
        ROS_WARN("Could not find param '%s' on node '%s'. Defaulting to '%s'", key, pnh.getNamespace().c_str(), ss.str().c_str());
    }
}
#endif

#endif
