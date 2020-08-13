/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

//Author: Nick Machak
//Co-author: Ryan Gupta
//Co-author: Cem Karamanli

///ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include "math_utils/running_covariance.h"

//Laikago SDK Modules
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
unitree_legged_msgs::IMU RecvIMUo;

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

class Listener{
public:
	double dx, dy, drz;


// Callback function for subscriber
	void twist_callback(const geometry_msgs::Twist::ConstPtr& vel_cmd)
	{
		dx = vel_cmd->linear.x;
		dy = vel_cmd->linear.y;
		drz = vel_cmd->angular.z;
		ROS_INFO("[v_x] I heard: [%s]", vel_cmd->linear.x);
		ROS_INFO("[v_y] I heard: [%s]", vel_cmd->linear.y);
		ROS_INFO("[v_z] I heard: [%s]", vel_cmd->angular.z);
		cout << "Twist Received" << endl;
	}
};

int main( int argc, char* argv[] )
{
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

	ros::init(argc, argv, "laikago_ros_driver");
	
	ros::NodeHandle n;

	Listener listener;
	
	ros::Rate loop_rate(500);
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher odom_pub = n.advertise<sensor_msgs::Imu>("imu_data", 1000);
	roslcm.SubscribeState();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);

	pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);

    // Covariance objects
    RunningCovariance cov_accel_, cov_rpy_, cov_gyro_;
    Eigen::Vector3d acceler_data_, rpy_data_, gyro_data_, acceler_cov_vec, rpy_cov_vec, gyro_cov_vec, acceler_var_vec, rpy_var_vec, gyro_var_vec;
    // Vars for collecting raw IMU data
    double quat_[4];
    double gyro_[3];
    double acceler_[3];
    double rpy_[3];
    double linear_accel_cov[9]; 
    // STILL WAITING ON UNITREE TO CORRECTLY TELL US WHICH VALUE CORRESP.
    // TO ORIENTATION

    sensor_msgs::Imu imu_msg;
    // Begin Filler
    // There are some issue with the covariance calc that I am using
    // because covariance of a vector3d should be a 3x3 matrix. 
    // Note: the sensor_msgs/Imu expects a flattened version of this 3x3 matrix hence array[9]
    imu_msg.linear_acceleration_covariance[0] = 0; imu_msg.linear_acceleration_covariance[1] = 0; imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0; imu_msg.linear_acceleration_covariance[4] = 0; imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0; imu_msg.linear_acceleration_covariance[7] = 0; imu_msg.linear_acceleration_covariance[8] = 0;

    imu_msg.orientation_covariance[0] = 0; imu_msg.orientation_covariance[1] = 0; imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0; imu_msg.orientation_covariance[4] = 0; imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0; imu_msg.orientation_covariance[7] = 0; imu_msg.orientation_covariance[8] = 0;

    while (ros::ok()){

        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
        printf("%f\n",  RecvHighROS.forwardSpeed);

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

        // robot_localization assumes East North Up (ENU) 
        // frame for all IMU data
        // i.e. y = magnetic north, x = east, z = up
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

        // COLLECT COVARIANCE DATA FOR IMU_MSG
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

        // Todo in IMU:
        // 1) Get correct frames
        // 2) Get correct covariance
    	
        odom_pub.publish(imu_msg);
	
    	////////////////////////////
        //// Local Odom here //////      Odometry msg:i
        //////////////////////////       frame_id = where position and orientation are (map or odom)
    	// 				                 child_frame_id = twist data (base_link)
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        //robot's position in x,y, and z
        //nav_msgs/Odometry -> float64
        //RecvHighROS -> float32
        odom.pose.pose.position.x = RecvHighROS.forwardPosition;
        odom.pose.pose.position.y = RecvHighROS.sidePosition;
        odom.pose.pose.position.z = 0.0;
        // robot's heading
	// odom.pose.pose.orientation = RecvIMU.quaternion;
	// Did Ryan do this line above? Not good to push code that doesnt build


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
        	SendHighROS.forwardSpeed = listener.dx;
        	SendHighROS.sideSpeed = listener.dy;
        	SendHighROS.rotateSpeed = listener.drz;
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



