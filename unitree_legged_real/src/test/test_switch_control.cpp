/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;


/* class Custom{
    Custom(){};
    void LCMRecv();

    LCM roslcm;
};

void Custom::LCMRecv()
{
    roslcm.Recv();
}
*/


double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}


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
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "test_switch_control");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    std::string robot_name;
    LeggedType rname;
    ros::param::get("/robot_name", robot_name);
    if(strcasecmp(robot_name.c_str(), "A1") == 0)
        rname = LeggedType::A1;
    else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
        rname = LeggedType::Aliengo;

    Control control(rname, LOWLEVEL);

    long motiontime=0;
    float torque = 0;
    LowCmd SendLowLCM = {0};
    LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    double pos = 0.3;

    while (ros::ok()){
        motiontime++;
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FL_1 position: %f\n",  RecvLowROS.motorState[FL_1].q);

        pos = pos*(-1);

        // // gravity compensation
        // SendLowROS.motorCmd[FR_0].tau = -0.65f;
        // SendLowROS.motorCmd[FL_0].tau = +0.65f;
        // SendLowROS.motorCmd[RR_0].tau = -0.65f;
        // SendLowROS.motorCmd[RL_0].tau = +0.65f;

        if( motiontime >= 500 && motiontime < 2000){
            torque = (0 - RecvLowROS.motorState[FL_2].q)*10.0f + (0 - RecvLowROS.motorState[FL_2].dq)*1.0f;
            if(torque > 5.0f) torque = 5.0f;
            if(torque < -5.0f) torque = -5.0f;

            SendLowROS.motorCmd[FL_2].tau = torque;
        }
        else if(motiontime >=2000 && motiontime <=3000){
            SendLowROS.motorCmd[FL_2].q =  pos;
            SendLowROS.motorCmd[FL_2].dq = 0;
            SendLowROS.motorCmd[FL_2].Kp = 5.0;
            SendLowROS.motorCmd[FL_2].Kd = 1.0;
            SendLowROS.motorCmd[FL_2].tau = 0.0f;
        }

        // Send command and spin ros loop
        SendLowLCM = ToLcm(SendLowROS);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
