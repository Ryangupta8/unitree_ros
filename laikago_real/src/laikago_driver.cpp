/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

//Author: Nick Machak

//Links: https://gist.github.com/grassjelly/b06aaf5e73019868236eeb425ca60f76 (for some guidance)
// https://github.com/Ryangupta8/unitree_ros/blob/master/unitree_legged_msgs/msg/HighState.msg 

//Note: This is the script for unitree_legged_sdk/examples/example_walk.cpp

///ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//Laikago SDK Modules
#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "laikago_sdk/laikago_sdk.hpp"

// Command Laikago to drive, as per walk_mode.cpp

using namespace laikago;
using namespace std;

static long motiontime = 0;
HighCmd SendHighLCM = {0};
HighState RecvHighLCM = {0};
unitree_legged_msgs::HighCmd SendHighROS;
unitree_legged_msgs::HighState RecvHighROS;

Control control(HIGHLEVEL);
LCM roslcm;
boost::mutex mutex;

void* update_loop(void* data)
{
    while(ros::ok){
        boost::mutex::scoped_lock lock(mutex);
        roslcm.Recv();
        lock.unlock();
        usleep(2000);
    }
}

// Callback function for subscriber
void twist_callback(const geometry_msgs::Twist& vel_cmd)
{
	ROS_INFO("[v_x] I heard: [%s]", vel_cmd.linear.x);
	ROS_INFO("[v_y] I heard: [%s]", vel_cmd.linear.y);
	ROS_INFO("[v_z] I heard: [%s]", vel_cmd.angular.z);
	cout << "Twist Received" << endl;
}

int main( int argc, char* argv[] )
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

	ros::init(argc, argv, "laikago_ros_driver");
	
	ros::NodeHandle n;
	
	ros::Rate loop_rate(500);
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
	roslcm.SubscribeState();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, twist_callback);

	pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

    while (ros::ok()){

        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
        printf("%f\n",  RecvHighROS.forwardSpeed);

        ////////////////////////////
        // publish odometry here //
        //////////////////////////
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        //robot's position in x,y, and z
        odom.pose.pose.position.x = RecvHighROS.forwardPosition; // can I do this if nav_msgs/Odometry takes float64 but RecvHighROS uses float32 here?
        odom.pose.pose.position.y = RecvHighROS.sidePosition; // can I do this if nav_msgs/Odometry takes float64 but RecvHighROS uses float32 here?
        odom.pose.pose.position.z = 0.0;
        // robot's heading ??
        // odom.pose.pose.orientation = ??

        // odom.child_frame_id = ?? 
        // linear speed
        odom.twist.twist.linear.x = RecvHighROS.forwardSpeed;
        odom.twist.twist.linear.y = RecvHighROS.sideSpeed;
        odom.twist.twist.linear.z = 0.0;

        //angular speed
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = RecvHighROS.rotateSpeed;

        // include covariance matrix in odom topic??

        pub.publish(odom);

        //////////////////////////////
        // command Laikago to move //
        ////////////////////////////
        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.forwardSpeed = 0.0f;

        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        if(motiontime>1000 && motiontime<1500){
            SendHighROS.roll = 0.5f;
        }

        if(motiontime>1500 && motiontime<2000){
            SendHighROS.pitch = 0.3f;
        }

        if(motiontime>2000 && motiontime<2500){
            SendHighROS.yaw = 0.3f;
        }

        if(motiontime>2500 && motiontime<3000){
            SendHighROS.bodyHeight = -0.3f;
        }

        if(motiontime>3000 && motiontime<3500){
            SendHighROS.bodyHeight = 0.3f;
        }

        if(motiontime>3500 && motiontime<4000){
            SendHighROS.bodyHeight = 0.0f;
        }

        if(motiontime>4000 && motiontime<5000){
            SendHighROS.mode = 2;
        }

        if (motiontime>5000 && motiontime<21000){
        	SendHighROS.forwardSpeed = vel_cmd.linear.y;
        	SendHighROS.sideSpeed = vel_cmd.linear.x;
        	SendHighROS.rotateSpeed = vel_cmd.angular.z;
        }

        if(motiontime>20000 && motiontime<21000){
            SendHighROS.mode = 1;
        }

        memcpy(&SendHighLCM, &SendHighROS, sizeof(HighCmd));
        roslcm.Send(SendHighLCM);
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;            	    
}



