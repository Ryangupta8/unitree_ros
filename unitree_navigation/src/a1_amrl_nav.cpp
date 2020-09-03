#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkState.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <stdio.h>  
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>




class InputOutput{
public:
    Eigen::Quaternion<double> ori;
    Eigen::Vector3d pos, linear, angular; 

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
        linear << msg->linear.x, msg->linear.y, msg->linear.z;
        angular << msg->angular.x, msg->angular.y, msg->angular.z;
    }

    void curr_state_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
        pos << msg->pose[2].position.x, msg->pose[2].position.y, msg->pose[2].position.z;
        ori.x() = msg->pose[2].orientation.x;
        ori.y() = msg->pose[2].orientation.y;
        ori.z() = msg->pose[2].orientation.z;
        ori.w() = msg->pose[2].orientation.w;
    }

};


int main(int argc, char **argv)
{
    // Define variables
    double timestep = 0.01;
    bool first_time = true;
   	InputOutput listener;

    double odom_x, odom_y; odom_x = 0.; odom_y = 0.;

    std_msgs::Bool bool_msg; bool_msg.data = true;
    gazebo_msgs::ModelState output;
    nav_msgs::Odometry odom_msg;

    Eigen::Vector3d  angular;
    Eigen::AngleAxisd conversion;
    Eigen::Quaternion<double> dori;

    ros::init(argc, argv, "move_publisher", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    // Pubs & subs
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &InputOutput::cmd_vel_cb, &listener);
    ros::Subscriber curr_state_sub = nh.subscribe("/gazebo/model_states", 1, &InputOutput::curr_state_cb, &listener);

    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/correct_odom", 1000);

    ros::Rate loop_rate(100);

    // Odometry Vars
    double x = 8.0;
    double y = -2.0;
    double th = 0.0;

    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // gazebo_msgs/LinkStates is somehow not a vector of type gazebo_msgs/LinkState
    // So, the topic /gazebo/link_states does not seem to have a reference frame...

    // Goal: intake /cmd_vel and output dx, dy, dz and dorientation
    //       into Gazebo using gazebo_msgs into "base" reference frame
    while(ros::ok()){
        ros::spinOnce();

        bool_pub.publish(bool_msg);

        // Lift Laikago off the ground
        if(first_time){
            output.pose.position.z = 0.5;
            std::cout << "first time" << std::endl;
        }
        else{
            output.pose.position.z = 0.0;  
        } 

        /////////////////
        //// ODOM //////
        ///////////////
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (listener.linear[0] * cos(th) - listener.linear[1] * sin(th)) * dt;
        double delta_y = (listener.linear[0] * sin(th) + listener.linear[1] * cos(th)) * dt;
        double delta_th = listener.angular[2] * dt;

        // // Get AngleAxis for rotation
        // conversion.axis() = {0, 0, 1}; 
        // conversion.angle() = delta_th; //listener.angular[2];
  
        // // Convert result to Quaternion
        // double angle; angle = conversion.angle() * timestep;/////////
        // dori.x() = 0;
        // dori.y() = 0;
        // dori.z() = 1 * sin( angle/2. );
        // dori.w() = cos( angle/2. );

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

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        //set the position
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        //set the velocity
        odom_msg.child_frame_id = "base";
        odom_msg.twist.twist.linear.x = listener.linear[0];
        odom_msg.twist.twist.linear.y = listener.linear[1];
        odom_msg.twist.twist.angular.z = listener.angular[2];

        /////////////////
        //// NAVI //////
        ///////////////
        // output.pose.position.x = delta_x;
        // output.pose.position.y = delta_y;
        // output.pose.orientation = odom_quat;
        // output.pose.orientation.x = dori.x();
        // output.pose.orientation.y = dori.y();
        // output.pose.orientation.z = dori.z();
        // output.pose.orientation.w = dori.w();
        output.pose.position.x = x;
        output.pose.position.y = y;
        output.pose.orientation = odom_quat;
        output.reference_frame = "map"; 
        output.model_name = "a1_gazebo";

        last_time = current_time;

        move_publisher.publish(output);
        odom_pub.publish(odom_msg);
        first_time = false;

        loop_rate.sleep();
    }
    
    return 0;
}
