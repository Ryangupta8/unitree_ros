/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

//Author: Nick Machak
//Co-author: Ryan Gupta

///ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

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
	roslcm.SubscribeState();
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);

	pthread_t tid;
    pthread_create(&tid, NULL, update_loop, NULL);
	

    while (ros::ok()){

        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        memcpy(&RecvHighROS, &RecvHighLCM, sizeof(HighState));
        printf("%f\n",  RecvHighROS.forwardSpeed);

	// Get IMU reading
	double quat_ [4] = { RecvHighROS.imu.quaternion[0] , RecvHighROS.imu.quaternion[1] , RecvHighROS.imu.quaternion[2] , RecvHighROS.imu.quaternion[3] };
	double gyro_ [3] = { RecvHighROS.imu.gyroscope[0] , RecvHighROS.imu.gyroscope[1] , RecvHighROS.imu.gyroscope[2] };
	double acceler_ [3] = { RecvHighROS.imu.accelerometer[0] , RecvHighROS.imu.accelerometer[1] , RecvHighROS.imu.accelerometer[2] };
	double rpy_ [3] = { RecvHighROS.imu.rpy[0] , RecvHighROS.imu.rpy[1] , RecvHighROS.imu.rpy[2] };
	// Create sensor_msgs/imu			// robot_localization assumes East North Up (ENU) 
	// 						// frame for all IMU data
	// 						// i.e. x = magnetic north, z = center of earth
	sensor_msgs::Imu imu_msg;
	imu_msg.header.frame_id = "base_link";// Need Unitree Response			// robot_localization assumes East North Up (ENU) 
							// frame for all IMU data
							// // i.e. x = magnetic north, z = center of earth
	imu_msg.orientation.x = quat_[0];
	imu_msg.orientation.y = quat_[1];
	imu_msg.orientation.z = quat_[2];
	imu_msg.orientation.w = quat_[3];

	imu_msg.linear_acceleration.x = acceler_[0];
	imu_msg.linear_acceleration.y = acceler_[1];
	imu_msg.linear_acceleration.z = acceler_[2];
	
		
	
	////////////////////////////
        // publish odometry here //      Odometry msg:i
        //////////////////////////       frame_id = where position and orientation are (map or odom)
	// 				 child_frame_id = twist data (base_link)
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



