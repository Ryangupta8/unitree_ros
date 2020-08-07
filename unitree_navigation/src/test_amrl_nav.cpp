#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkState.h>
#include <string>
#include <stdio.h>  
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
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

    std_msgs::Bool bool_msg; bool_msg.data = true;
    gazebo_msgs::ModelState output;

    Eigen::Vector3d dxyz, angular;
    Eigen::AngleAxisd conversion;
    Eigen::Quaternion<double> dori;

    ros::init(argc, argv, "move_publisher", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    // Pubs & subs
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &InputOutput::cmd_vel_cb, &listener);
    ros::Subscriber curr_state_sub = nh.subscribe("/gazebo/model_states", 1, &InputOutput::curr_state_cb, &listener);

    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
    ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);

    ros::Rate loop_rate(100);


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
        
        // Get change of position
        dxyz = listener.linear * timestep;
        // Get AngleAxis for rotation
        conversion.axis() = {0, 0, 1};
        angular = listener.angular * timestep ; 
        conversion.angle() = angular[2]; //listener.angular[2];
  
        // Convert result to Quaternion
        double angle; angle = conversion.angle() * timestep;/////////
        dori.x() = 0;
        dori.y() = 0;
        dori.z() = 1 * sin( angle/2. );
        dori.w() = cos( angle/2. );

        output.pose.position.x = dxyz[0];
        output.pose.position.y = dxyz[1];
        output.pose.orientation.x = dori.x();
        output.pose.orientation.y = dori.y();
        output.pose.orientation.z = dori.z();
        output.pose.orientation.w = dori.w();
        output.reference_frame = "base"; 
        output.model_name = "laikago_gazebo";

        // std::cout << "angular[2] = " << angular[2] << std::endl;
        // std::cout << "dxyz[0] = " << dxyz[0] << std::endl;
        // std::cout << "dxyz[1] = " << dxyz[1] << std::endl;
        
        move_publisher.publish(output);
        first_time = false;

        loop_rate.sleep();
    }
    
    return 0;
}
