/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Author: Ryan Gupta

#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

///ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>


#include "math_utils/running_covariance.h"

using namespace UNITREE_LEGGED_SDK;
using namespace std;


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

    ros::init(argc, argv, "test_a1_odom");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("commands", 1000);

    nav_msgs::Odometry odom_msg;
    geometry_msgs::Twist command_msg;

    tf2_ros::StaticTransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    // LCM Connection
    long motiontime = 0;
    HighCmd SendHighLCM = {0};
    HighState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    while (ros::ok()){
        ros::spinOnce();

        //cout << "ROBOT COMMANDS: " << endl;
        //cout << "forwardSpeed = " << SendHighROS.forwardSpeed << endl;
        //cout << "sideSpeed = " << SendHighROS.sideSpeed << endl;
        //cout << "rotateSpeed = " << SendHighROS.rotateSpeed << endl;

        motiontime = motiontime+2;

        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);

        current_time = ros::Time::now();

        ///////////////////
        //     ODOM     //
        /////////////////
        double dt = (current_time - last_time).toSec();
        double delta_x = (RecvHighROS.forwardSpeed * cos(th) - RecvHighROS.sideSpeed * sin(th)) * dt;
        double delta_y = (RecvHighROS.forwardSpeed * sin(th) + RecvHighROS.sideSpeed * cos(th)) * dt;
        double delta_th = RecvHighROS.rotateSpeed * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;
        while(th >2*3.141592)
        {
            th-=2*3.141592;
        
        }

        ///////////////////
        //   COMMANDS   //
        /////////////////

        // 3 Seconds of walking forward
        //if(motiontime < 3000){
            //SendHighROS.mode = 2;
            //SendHighROS.forwardSpeed = 0.5f;
            //SendHighROS.rotateSpeed = 0.0f;
        //}

         //5 Seconds of turning + forward 
        //if(motiontime >= 3000 && motiontime < 8000){
            //SendHighROS.mode = 2;
            //SendHighROS.forwardSpeed = 0.2f;
            //SendHighROS.rotateSpeed = 0.5f;
        //}

          //5 seconds of turning other way + forward
        //if(motiontime >= 8000 && motiontime < 13000){
            //SendHighROS.mode = 2;
            //SendHighROS.forwardSpeed = 0.2f;
            //SendHighROS.rotateSpeed = -0.5f;
        //}

          //5 seconds of turning other way
        //if(motiontime >= 13000 && motiontime < 18000){
            //SendHighROS.mode = 2;
            //SendHighROS.rotateSpeed = 0.5f;
        //}
        SendHighROS.mode = 2; 
        SendHighROS.rotateSpeed = 0.5f;

        cout << "ROBOT MEASUREMENTS: " << endl;
        //cout << "forwardSpeed = " << RecvHighROS.forwardSpeed << endl;
        //cout << "sideSpeed = " << RecvHighROS.sideSpeed << endl;
        cout << "rotateSpeed = " << RecvHighROS.rotateSpeed << endl;

        cout << "ODOMETRY: " << endl;
        cout << "x = " << x << endl;
        cout << "y = " << y << endl;
        cout << "theta = " << th << endl;

        cout << "---------------------------------------------------------" << endl;

        if(motiontime >= 18000){
            ros::shutdown();
        }

        SendHighLCM = ToLcm(SendHighROS);
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
