/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

// Author: Ryan Gupta
// Co-author: Nick Machak

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

class Listener{
public:
    double dx, dy, drz; 
    geometry_msgs::PoseWithCovariance pose;


// Callback function for /cmd_vel subscriber
    void twist_callback(const geometry_msgs::Twist::ConstPtr& vel_cmd)
    {
        dx = vel_cmd->linear.x;
        dy = vel_cmd->linear.y;
        drz = vel_cmd->angular.z;
        cout << "Twist Received" << endl;
    }

    void ekf_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& filtered_pose)
    {
        pose = filtered_pose->pose;
        cout << "Robot Pose EKF received" << endl;
    }
};

int main(int argc, char *argv[])
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init(argc, argv, "a1_driver_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    Listener listener;

    // Publisher Subscriber
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher odom_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
    ros::Publisher filtered_odom_pub = n.advertise<nav_msgs::Odometry>("filtered_odom", 1000);
    ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);
    ros::Subscriber ekf_pose_sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000, &Listener::ekf_pose_callback, &listener);
    // Covariance objects
    RunningCovariance cov_accel_, cov_rpy_, cov_gyro_, cov_pose_posit_, cov_pose_orient_, cov_twist_lin_, cov_twist_ang_;
    Eigen::Vector3d acceler_data_, rpy_data_, gyro_data_, pose_posit_data_, pose_orient_data_, twist_lin_data_, twist_ang_data_, acceler_cov_vec, rpy_cov_vec, gyro_cov_vec, pose_posit_cov_vec, pose_orient_cov_vec, twist_lin_cov_vec, twist_ang_cov_vec, acceler_var_vec, rpy_var_vec, gyro_var_vec, pose_posit_var_vec, pose_orient_var_vec, twist_lin_var_vec, twist_ang_var_vec;
    // Vars for collecting raw IMU data
    double quat_[4];
    double gyro_[3];
    double acceler_[3];
    double rpy_[3];
    // ROS Msgs
    sensor_msgs::Imu imu_msg;
    nav_msgs::Odometry odom;
    // Initialize covariance matrices
    for (int i = 0; i < 9; i++){
        imu_msg.linear_acceleration_covariance[i] = 0;
        imu_msg.orientation_covariance[i] = 0;
        odom.pose.covariance[i] = 0;
        odom.twist.covariance[i] = 0;
    }
    for (int i=9; i < 36; i++){
        odom.pose.covariance[i] = 0;
        odom.twist.covariance[i] = 0;
    }

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

        imu_msg.header.frame_id = "base_link";  
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

        // Convert raw IMU data to Eigen
        acceler_data_[0] = acceler_[0]; acceler_data_[1] = acceler_[1]; acceler_data_[2] = acceler_[2];
        rpy_data_[0] = rpy_[0]; rpy_data_[1] = rpy_[1]; rpy_data_[2] = rpy_[2];
        gyro_data_[0] = gyro_[0]; gyro_data_[0] = gyro_[1]; gyro_data_[0] = gyro_[2];  
        // Get covariance for Eigen data
        acceler_cov_vec = cov_accel_.Push(acceler_data_);
        rpy_cov_vec = cov_rpy_.Push(rpy_data_);
        gyro_cov_vec = cov_gyro_.Push(gyro_data_);

        // Get variance for Eigen data
        acceler_var_vec = cov_accel_.Variance();
        rpy_var_vec = cov_rpy_.Variance();
        gyro_var_vec = cov_gyro_.Variance();

        //// Collect covariance values for linear acceleration
        imu_msg.linear_acceleration_covariance[1] = imu_msg.linear_acceleration_covariance[3] = acceler_cov_vec[0];
        imu_msg.linear_acceleration_covariance[5] = imu_msg.linear_acceleration_covariance[7] = acceler_cov_vec[1];
        imu_msg.linear_acceleration_covariance[2] = imu_msg.linear_acceleration_covariance[6] = acceler_cov_vec[2];
        //// Collect variance values for linear acceleration
        imu_msg.linear_acceleration_covariance[0] = acceler_var_vec[0];
        imu_msg.linear_acceleration_covariance[4] = acceler_var_vec[1];
        imu_msg.linear_acceleration_covariance[8] = acceler_var_vec[2];
        //// Collect covariance values for orientation
        imu_msg.orientation_covariance[1] = imu_msg.orientation_covariance[3] = rpy_cov_vec[0];
        imu_msg.orientation_covariance[5] = imu_msg.orientation_covariance[7] = rpy_cov_vec[1];
        imu_msg.orientation_covariance[2] = imu_msg.orientation_covariance[6] = rpy_cov_vec[2];
        //// Collect variance values for orientation
        imu_msg.orientation_covariance[0] = rpy_var_vec[0];
        imu_msg.orientation_covariance[4] = rpy_var_vec[1];
        imu_msg.orientation_covariance[8] = rpy_var_vec[2];
        //// Collect covariance values for angular velocity
        imu_msg.angular_velocity_covariance[1] = imu_msg.angular_velocity_covariance[3] = gyro_cov_vec[0];
        imu_msg.angular_velocity_covariance[5] = imu_msg.angular_velocity_covariance[7] = gyro_cov_vec[1];
        imu_msg.angular_velocity_covariance[2] = imu_msg.angular_velocity_covariance[6] = gyro_cov_vec[2];
        //// Collect variance values for angular velocity
        imu_msg.angular_velocity_covariance[0] = gyro_var_vec[0];
        imu_msg.angular_velocity_covariance[4] = gyro_var_vec[1];
        imu_msg.angular_velocity_covariance[8] = gyro_var_vec[2];

        odom_pub.publish(imu_msg);

        ////////////////////////////
        //// Local Odom here //////      
        //////////////////////////       
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        // robot's position in x, y, z
        odom.pose.pose.position.x = RecvHighROS.forwardPosition;
        odom.pose.pose.position.y = RecvHighROS.sidePosition;
        odom.pose.pose.position.z = 0.0;
        // robot's heading
        odom.pose.pose.orientation.x = RecvHighROS.imu.quaternion[0]; 
        odom.pose.pose.orientation.y = RecvHighROS.imu.quaternion[1];
        odom.pose.pose.orientation.z = RecvHighROS.imu.quaternion[2];
        odom.pose.pose.orientation.w = RecvHighROS.imu.quaternion[3];

        // linear speed
        odom.twist.twist.linear.x = RecvHighROS.forwardSpeed;
        odom.twist.twist.linear.y = RecvHighROS.sideSpeed;
        odom.twist.twist.linear.z = 0.0;

        //angular speed
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = RecvHighROS.rotateSpeed;

        // convert raw data to Eigen
        //// position
        pose_posit_data_[0] = RecvHighROS.forwardPosition;
        pose_posit_data_[1] = RecvHighROS.sidePosition;
        pose_posit_data_[2] = 0.0;
        //// orientation same as rpy_
        pose_orient_data_[0] = rpy_[0];
        pose_orient_data_[1] = rpy_[1];
        pose_orient_data_[2] = rpy_[2];
        //// twist
        twist_lin_data_[0] = RecvHighROS.forwardSpeed;
        twist_lin_data_[1] = RecvHighROS.sideSpeed;
        twist_lin_data_[2] = 0.0;
        twist_ang_data_[0] = 0.0;
        twist_ang_data_[1] = 0.0;
        twist_ang_data_[2] = RecvHighROS.rotateSpeed;

        //Get covariance for Eigen data
        pose_posit_cov_vec = cov_pose_posit_.Push(pose_posit_data_);
        pose_orient_cov_vec = cov_pose_orient_.Push(pose_orient_data_);
        twist_lin_cov_vec = cov_twist_lin_.Push(twist_lin_data_);
        twist_ang_cov_vec = cov_twist_ang_.Push(twist_ang_data_);

        //Get variance for Eigen data
        pose_posit_var_vec = cov_pose_posit_.Variance();
        pose_orient_var_vec = cov_pose_orient_.Variance();
        twist_lin_var_vec = cov_twist_lin_.Variance();
        twist_ang_var_vec = cov_twist_ang_.Variance();

        // COLLECT COVARIANCE DATA FOR ODOM
        //// Covariance of Pose
        //////// Collect covariance values for position
        odom.pose.covariance[1] = odom.pose.covariance[3] = pose_posit_cov_vec[0]; // here
        odom.pose.covariance[5] = odom.pose.covariance[7] = pose_posit_cov_vec[1];
        odom.pose.covariance[2] = odom.pose.covariance[6] = pose_posit_cov_vec[2];
        odom.pose.covariance[0] = pose_posit_var_vec[0];
        odom.pose.covariance[4] = pose_posit_var_vec[1];
        odom.pose.covariance[8] = pose_posit_var_vec[2];

        // Leaving odom.covariance[9] through odom.covariance[27] equal to zero for now.

        //////// Collect covariance values for orientation
        odom.pose.covariance[28] = odom.pose.covariance[30] = pose_orient_cov_vec[0];
        odom.pose.covariance[32] = odom.pose.covariance[34] = pose_orient_cov_vec[1];
        odom.pose.covariance[29] = odom.pose.covariance[33] = pose_orient_cov_vec[2];
        //////// Collect variance values for orientation
        odom.pose.covariance[27] = pose_orient_var_vec[0];
        odom.pose.covariance[31] = pose_orient_var_vec[1];
        odom.pose.covariance[35] = pose_orient_var_vec[2];

        // COLLECT COVARIANCE DATA FOR TWIST
        //// Covariance of Twist
        //////// Collect covariance values for linear velocity
        odom.twist.covariance[1] = odom.twist.covariance[3] = twist_lin_cov_vec[0];       
        odom.twist.covariance[5] = odom.twist.covariance[7] = twist_lin_cov_vec[1];
        odom.twist.covariance[2] = odom.twist.covariance[6] = twist_lin_cov_vec[2];
        //////// Collect variance values for linear velocity
        odom.twist.covariance[0] = twist_lin_var_vec[0];
        odom.twist.covariance[4] = twist_lin_var_vec[1];
        odom.twist.covariance[8] = twist_lin_var_vec[2];

        // Leaving twist.covariance[9] through twist.covariance[27] equal to zero for now.

        //////// Collect covariance values for angular velocity
        odom.twist.covariance[28] = odom.twist.covariance[30] = twist_ang_cov_vec[0]; 
        odom.twist.covariance[32] = odom.twist.covariance[34] = twist_ang_cov_vec[1];
        odom.twist.covariance[29] = odom.twist.covariance[33] = twist_ang_cov_vec[2];
        //////// Collect variance values for orientation
        odom.twist.covariance[27] = twist_ang_var_vec[0]; 
        odom.twist.covariance[31] = twist_ang_var_vec[1];
        odom.twist.covariance[35] = twist_ang_var_vec[2];

        filtered_odom_pub.publish(odom);

        ///////////////////////////////
        // Publish robot_pose_ekf ////
        /////////////////////////////

        nav_msgs::Odometry filtered_odom;

        filtered_odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "filtered_odom";

        filtered_odom.pose = listener.pose;

        filtered_odom_pub.publish(filtered_odom);

        ////////////////////////
        // Unitre Legged SDK //
        //////////////////////

        SendHighROS.forwardSpeed = 0.0f;
        SendHighROS.sideSpeed = 0.0f;
        SendHighROS.rotateSpeed = 0.0f;
        SendHighROS.bodyHeight = 0.0f;

        SendHighROS.mode = 0;
        SendHighROS.roll  = 0;
        SendHighROS.pitch = 0;
        SendHighROS.yaw = 0;

        SendHighROS.mode = 2;
        SendHighROS.forwardSpeed = 0.8 ;
        SendHighROS.sideSpeed = listener.dy;
        SendHighROS.rotateSpeed = listener.drz;

        SendHighLCM = ToLcm(SendHighROS);
        roslcm.Send(SendHighLCM);
        

        // Kill program after 10000 = 10 seconds
        if(motiontime > 10000){
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep(); 
    }
    return 0;
}

