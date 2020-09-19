/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
// A script to gather robot state and publish it to ROS
// e.g. IMU data and velocity data
// Velocity data is used in a simple way for odometry
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

    ros::init(argc, argv, "a1_driver_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);

    // Vars for collecting raw IMU data
    double quat_[4];
    double gyro_[3];
    double acceler_[3];
    double rpy_[3];
    // ROS Msgs
    sensor_msgs::Imu imu_msg;
    nav_msgs::Odometry odom;

    long motiontime = 0;
    HighCmd SendHighLCM = {0};
    HighState RecvHighLCM = {0};
    unitree_legged_msgs::HighCmd SendHighROS;
    unitree_legged_msgs::HighState RecvHighROS;
    LCM roslcm;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    tf2_ros::StaticTransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    while (ros::ok()){
    	ros::spinOnce();

        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);

        current_time = ros::Time::now();

        ////////////////////////////
        //// Local Odom here //////      
        //////////////////////////

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (RecvHighROS.forwardSpeed * cos(th) - RecvHighROS.sideSpeed * sin(th)) * dt;
        double delta_y = (RecvHighROS.forwardSpeed * sin(th) + RecvHighROS.sideSpeed * cos(th)) * dt;
        double delta_th = RecvHighROS.rotateSpeed * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);   

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base";
        odom.twist.twist.linear.x = RecvHighROS.forwardSpeed;
        odom.twist.twist.linear.y = RecvHighROS.sideSpeed;
        odom.twist.twist.angular.z = RecvHighROS.rotateSpeed;

        //publish the message
        odom_pub.publish(odom);

        ////////////////////////////
        ////// IMU Here ///////////
        //////////////////////////
        // Get IMU data
        quat_[0] = RecvHighROS.imu.quaternion[0]; quat_[1] = RecvHighROS.imu.quaternion[1]; 
        quat_[2] = RecvHighROS.imu.quaternion[2]; quat_[3] = RecvHighROS.imu.quaternion[3];
        
        gyro_[0] = RecvHighROS.imu.gyroscope[0]; gyro_[1] = RecvHighROS.imu.gyroscope[1]; gyro_[2] = RecvHighROS.imu.gyroscope[2];

        acceler_[0] = RecvHighROS.imu.accelerometer[0]; acceler_[1] = RecvHighROS.imu.accelerometer[1]; 
        acceler_[2] = RecvHighROS.imu.accelerometer[2];

        rpy_[0] = RecvHighROS.imu.rpy[0]; rpy_[1] = RecvHighROS.imu.rpy[1]; rpy_[2] = RecvHighROS.imu.rpy[2];

        imu_msg.header.frame_id = "base";  
        // IMU Orientation                   
        imu_msg.orientation.x = quat_[0];
        imu_msg.orientation.y = quat_[1];
        imu_msg.orientation.z = quat_[2];
        imu_msg.orientation.w = quat_[3];

        // IMU Linear Acceleration
        imu_msg.linear_acceleration.x = acceler_[0];
        imu_msg.linear_acceleration.y = acceler_[1];
        imu_msg.linear_acceleration.z = acceler_[2];
        // IMU Angular Velocity
        imu_msg.angular_velocity.x = gyro_[0]; 
        imu_msg.angular_velocity.y = gyro_[1];
        imu_msg.angular_velocity.z = gyro_[2];

        imu_pub.publish(imu_msg);

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}