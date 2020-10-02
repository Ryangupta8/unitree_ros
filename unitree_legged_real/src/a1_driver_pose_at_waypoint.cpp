/*************************************************************************
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


class Listener{
public:
    double dx, dy, drz; 
    double goal_x, goal_y;

// Callback function for /cmd_vel subscriber
    void twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        dx = msg->linear.x;
        dy = msg->linear.y;
        drz = msg->angular.z;
        // cout << "Twist Received"  << endl;
    }

    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        goal_x = msg->pose.position.x;
        goal_y = msg->pose.position.y;
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

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher autonomy_pub = n.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);
    ros::Subscriber goal_sub= n.subscribe("/move_base_simple/goal", 1000, &Listener::goal_callback, &listener);

    nav_msgs::Odometry odom;
    std_msgs::Bool bool_msg; bool_msg.data = true;

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

    tf2_ros::StaticTransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    bool change_goal_bool = false;
    double prev_goal_x, prev_goal_y;
    prev_goal_x = 0; prev_goal_y = 0;
    int j = 0; 

    ros::Time current_time, last_time, pose_time_init, pose_time_final;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    autonomy_pub.publish(bool_msg);


    while (ros::ok()){
        ros::spinOnce();

        motiontime = motiontime+2;
        roslcm.Get(RecvHighLCM);
        RecvHighROS = ToRos(RecvHighLCM);
                       // check for incoming messages
        current_time = ros::Time::now();

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

        /////////////////////////
        // Unitree Legged SDK //
        ///////////////////////
        if((prev_goal_x != listener.goal_x && prev_goal_y != listener.goal_y)){
            change_goal_bool = true;
        }
        
        if(change_goal_bool){
            SendHighROS.mode = 1;
            cout << "j = " << j << endl;
            if(j < 2000){
                SendHighROS.roll  = 0.0f;
                SendHighROS.pitch = -1.0f;
                SendHighROS.yaw = 0.0f; 
                cout << "s1" << endl;
            }
            else if(j <= 6000){
                SendHighROS.mode = 1;
                SendHighROS.roll  = 0.0f;
                SendHighROS.pitch = -0.8f;
                SendHighROS.yaw = -0.8f;
                cout << "s2" << endl; 
            }
            else if(j <= 7000){
                SendHighROS.forwardSpeed = 0.0f;
                SendHighROS.sideSpeed = 0.0f;
                SendHighROS.rotateSpeed = 0.0f;
                SendHighROS.bodyHeight = 0.0f;

                SendHighROS.mode = 1;
                SendHighROS.roll  = 0;
                SendHighROS.pitch = 0;
                SendHighROS.yaw = 0;
                cout << "s3" << endl; 
            }
            else if(j <= 10000){
                SendHighROS.mode = 1;
                SendHighROS.roll  = 0.0f;
                SendHighROS.pitch = -0.8f;
                SendHighROS.yaw = 0.8f;
                cout << "s4" << endl;
            }
            else if(j <= 14000){
                SendHighROS.roll  = 0.0f;
                SendHighROS.pitch = 0.0f;
                SendHighROS.yaw = 0.0f; 
                cout << "s5" << endl;                 
            }
            else{
                SendHighROS.forwardSpeed = 0.0f;
                SendHighROS.sideSpeed = 0.0f;
                SendHighROS.rotateSpeed = 0.0f;
                SendHighROS.bodyHeight = 0.0f;

                SendHighROS.mode = 1;
                SendHighROS.roll  = 0;
                SendHighROS.pitch = 0;
                SendHighROS.yaw = 0;
                change_goal_bool = false;
 
            }
           
            
        }
        else{
            if(listener.dx == 0 && listener.dy == 0 && listener.drz == 0){
                SendHighROS.forwardSpeed = 0.0f;
                SendHighROS.sideSpeed = 0.0f;
                SendHighROS.rotateSpeed = 0.0f;
                SendHighROS.bodyHeight = 0.0f;

                SendHighROS.mode = 1;
                SendHighROS.roll  = 0;
                SendHighROS.pitch = 0;
                SendHighROS.yaw = 0;
            }
            else{
                SendHighROS.mode = 2;
                SendHighROS.forwardSpeed = listener.dx;
                SendHighROS.sideSpeed = listener.dy;
                SendHighROS.rotateSpeed = listener.drz;
                
            }
            j = 0;
        }
        
 
        j += 2;  

        prev_goal_x = listener.goal_x;
        prev_goal_y = listener.goal_y;

        SendHighLCM = ToLcm(SendHighROS);
        roslcm.Send(SendHighLCM);
        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;


}
