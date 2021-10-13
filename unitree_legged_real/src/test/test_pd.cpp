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
#include "Eigen/Dense"

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
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "torque_ros_mode");
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
    }

    Eigen::VectorXd initial_jpos, kd, kp;
    initial_jpos = Eigen::VectorXd::Zero(12);
    kp = Eigen::VectorXd::Zero(12);
    kd = Eigen::VectorXd::Zero(12);
    initial_jpos << 0.68, 1.3, -2.9, -0.68, 1.3, -2.9, 0.75, 0.6, -2.75, -0.75, 0.6, -2.75;
    kp << 40, 40, 50, 40, 40, 50, 40, 40, 50, 40, 40, 50;
    kd << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;

    while (ros::ok()){
        motiontime++;
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        Eigen::VectorXd q, q_dot, q_ddot; 
        q = Eigen::VectorXd::Zero(12);
        q_ddot = Eigen::VectorXd::Zero(12);
        q_dot = Eigen::VectorXd::Zero(12);
        q[0] = RecvLowROS.motorState[FL_0].q;
        q[1] = RecvLowROS.motorState[FL_1].q;
        q[2] = RecvLowROS.motorState[FL_2].q;
        q[3] = RecvLowROS.motorState[FR_0].q;
        q[4] = RecvLowROS.motorState[FR_1].q;
        q[5] = RecvLowROS.motorState[FR_2].q;
        q[6] = RecvLowROS.motorState[RL_0].q;
        q[7] = RecvLowROS.motorState[RL_1].q;
        q[8] = RecvLowROS.motorState[RL_2].q;
        q[9] = RecvLowROS.motorState[RR_0].q;
        q[10] = RecvLowROS.motorState[RR_1].q;
        q[11] = RecvLowROS.motorState[RR_2].q;

        q_dot[0] = RecvLowROS.motorState[FL_0].dq_raw;
        q_dot[1] = RecvLowROS.motorState[FL_1].dq_raw;
        q_dot[2] = RecvLowROS.motorState[FL_2].dq_raw;
        q_dot[3] = RecvLowROS.motorState[FR_0].dq_raw;
        q_dot[4] = RecvLowROS.motorState[FR_1].dq_raw;
        q_dot[5] = RecvLowROS.motorState[FR_2].dq_raw;
        q_dot[6] = RecvLowROS.motorState[RL_0].dq_raw;
        q_dot[7] = RecvLowROS.motorState[RL_1].dq_raw;
        q_dot[8] = RecvLowROS.motorState[RL_2].dq_raw;
        q_dot[9] = RecvLowROS.motorState[RR_0].dq_raw;
        q_dot[10] = RecvLowROS.motorState[RR_1].dq_raw;
        q_dot[11] = RecvLowROS.motorState[RR_2].dq_raw;
 
        // gravity compensation
        // SendLowROS.motorCmd[FR_0].tau = -0.65f;
        // SendLowROS.motorCmd[FL_0].tau = +0.65f;
        // SendLowROS.motorCmd[RR_0].tau = -0.65f;
        // SendLowROS.motorCmd[RL_0].tau = +0.65f;

        Eigen::VectorXd trq_cmd_; trq_cmd_ = Eigen::VectorXd::Zero(12);
        
        if( motiontime >= 500){
            for(int i=0; i< q.size(); ++i){
              trq_cmd_[i] = kp[i] * (initial_jpos[i] - q[i]) + kd[i] * (0 - q_dot[i]);
            }
            /*torque = (0 - RecvLowROS.motorState[FL_1].q)*10.0f + (0 - RecvLowROS.motorState[FL_1].dq)*1.0f;
            if(torque > 5.0f) torque = 5.0f;
            if(torque < -5.0f) torque = -5.0f;

            SendLowROS.motorCmd[FL_1].q = PosStopF;
            SendLowROS.motorCmd[FL_1].dq = VelStopF;
            SendLowROS.motorCmd[FL_1].Kp = 0;
            SendLowROS.motorCmd[FL_1].Kd = 0;
            SendLowROS.motorCmd[FL_1].tau = torque;*/
            /*SendLowROS.motorCmd[FL_0].tau = trq_cmd_[0];
            SendLowROS.motorCmd[FL_1].tau = trq_cmd_[1];
            SendLowROS.motorCmd[FL_2].tau = trq_cmd_[2];
            SendLowROS.motorCmd[FR_0].tau = trq_cmd_[3];
            SendLowROS.motorCmd[FR_1].tau = trq_cmd_[4];
            SendLowROS.motorCmd[FR_2].tau = trq_cmd_[5];
            SendLowROS.motorCmd[RL_0].tau = trq_cmd_[6];
            SendLowROS.motorCmd[RL_1].tau = trq_cmd_[7];
            SendLowROS.motorCmd[RL_2].tau = trq_cmd_[8];
            SendLowROS.motorCmd[RR_0].tau = trq_cmd_[9];
            SendLowROS.motorCmd[RR_1].tau = trq_cmd_[10];
            SendLowROS.motorCmd[RR_2].tau = trq_cmd_[11];*/
        }
        /*std::cout << "q[FL_0] = " << q[0] << std::endl;
        std::cout << "initial_jpos[FL_0] = " << initial_jpos[0] << std::endl;
        std::cout << "q[FL_1] = " << q[1] << std::endl;
        std::cout << "initial_jpos[FL_1] = " << initial_jpos[1] << std::endl;
        std::cout << "q[FL_2] = " << q[2] << std::endl;
        std::cout << "initial_jpos[FL_2] = " << initial_jpos[2] << std::endl;
        std::cout << "q[FR_0] = " << q[3] << std::endl;
        std::cout << "initial_jpos[FR_0] = " << initial_jpos[3] << std::endl;
        std::cout << "q[FR_1] = " << q[4] << std::endl;
        std::cout << "initial_jpos[FR_1] = " << initial_jpos[4] << std::endl;*/
        std::cout << "q[FR_2] = " << q[5] << std::endl;
        /*std::cout << "initial_jpos[FR_2] = " << initial_jpos[5] << std::endl;
        std::cout << "q[RL_0] = " << q[6] << std::endl;
        std::cout << "initial_jpos[RL_0] = " << initial_jpos[6] << std::endl;
        std::cout << "q[RL_1] = " << q[7] << std::endl;
        std::cout << "initial_jpos[RL_1] = " << initial_jpos[7] << std::endl;
        std::cout << "q[RL_2] = " << q[8] << std::endl;
        std::cout << "initial_jpos[RL_2] = " << initial_jpos[8] << std::endl;
        std::cout << "q[RR_0] = " << q[9] << std::endl;
        std::cout << "initial_jpos[RR_0] = " << initial_jpos[9] << std::endl;
        std::cout << "q[RR_1] = " << q[10] << std::endl;
        std::cout << "initial_jpos[RR_1] = " << initial_jpos[10] << std::endl;
        std::cout << "q[RR_2] = " << q[11] << std::endl;
        std::cout << "initial_jpos[RR_2] = " << initial_jpos[11] << std::endl;*/

        SendLowLCM = ToLcm(SendLowROS);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
