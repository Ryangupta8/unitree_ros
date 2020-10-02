/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

void* update_loop(void* param)
{
    LCM *data = (LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "walk_ros_mode");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    // SetLevel(HIGHLEVEL);
    long motiontime = 0;
    HighCmd SendHighLCM = {0};
    HighState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    while (ros::ok()){
        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
        // printf("%f\n",  RecvHighROS.forwardSpeed);

        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.bodyHeight = 0.0f;

        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

       if(motiontime<3333){
            SendHighROS.mode = 1;
            SendHighROS.roll  = 0.0f;
            SendHighROS.pitch = -1.0f;
            SendHighROS.yaw = 0.0f; 
        }

        if(motiontime>=3333 && motiontime<6667){
            SendHighROS.mode = 1;
            SendHighROS.roll  = 0.0f;
            SendHighROS.pitch = -1.0f;
            SendHighROS.yaw = -0.8f;  
        }
        if(motiontime>=6667 && motiontime<8000){
            SendHighROS.mode = 1;
            SendHighROS.roll  = 0.0f;
            SendHighROS.pitch = -1.0f;
            SendHighROS.yaw = -0.8f;  
        }
        if(motiontime>=8000 && motiontime<12000){
            SendHighROS.mode = 1;
            SendHighROS.roll  = 0.0f;
            SendHighROS.pitch = -1.0f;
            SendHighROS.yaw = 0.8f;  
        }

        if(motiontime>10000){
            ros::shutdown();
        }

        SendHighLCM = ToLcm(SendHighROS);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

