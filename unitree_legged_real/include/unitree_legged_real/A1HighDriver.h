/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Author: Ryan Gupta

#ifndef A1_HIGH_DRIVER_H
#define A1_HIGH_DRIVER_H

#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include "convert.h"

///ROS Modules
#include <ros/ros.h>
#include "math_utils/running_covariance.h"




class A1HighDriver{
public:
	// LCM Connection
	long motiontime;
	UNITREE_LEGGED_SDK::HighCmd SendHighLCM;
    UNITREE_LEGGED_SDK::HighState RecvHighLCM;
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    UNITREE_LEGGED_SDK::LCM roslcm;

    pthread_t tid;

    A1HighDriver();

    void GetState();

    void SendCmd();


};


#endif