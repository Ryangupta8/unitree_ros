/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Author: Ryan Gupta

#ifndef GO1_HIGH_DRIVER_H
#define GO1_HIGH_DRIVER_H

// Unitree Modules
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

// ROS Modules
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

// using namespace UNITREE_LEGGED_SDK;

class Go1HighDriver{
public:
    // LCM Connection
    long motiontime;
    // UNITREE_LEGGED_SDK::TCmd SendHighLCM = {0};
    // UNITREE_LEGGED_SDK::TState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    // UNITREE_LEGGED_SDK::LCM roslcm

    pthread_t tid;

    Go1HighDriver(void*);

    void GetState(void*);

    void SendCmd();


};


#endif GO1_HIGH_DRIVER_H
