#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string>
#include <stdio.h>  
#include <tf/transform_datatypes.h>
// #include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <iostream>


class InputOutput{
public:
    geometry_msgs::Vector3 linear, angular;
	gazebo_msgs::ModelState output;
    // ros::NodeHandle nh;

    // ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
        linear = msg->linear;
        angular = msg->angular;
        // std::cout << "callback" << std::endl;

        // pub();
    }

    // void pub(){
        // output.twist.linear = linear;
        // output.twist.angular = angular;
        // output.model_name = "laikago_gazebo";
        // output.reference_frame = "map";

        // move_publisher.publish(output);
    // }

};


int main(int argc, char **argv)
{

    bool first_time = true;
   
   	InputOutput listener;

    ros::init(argc, argv, "move_publisher", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &InputOutput::cmd_vel_cb, &listener);

    ros::Publisher move_publisher = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);

    ros::Rate loop_rate(100);

    gazebo_msgs::ModelState output;

    
    while(ros::ok()){
        ros::spinOnce();

        if(first_time){
            output.pose.position.x = 0.0;
            output.pose.position.y = 0.0;
            output.pose.position.z = 1.5;
            output.twist.linear.x = 0;
            output.twist.linear.y = 0;
            output.twist.linear.z = 0;
            output.reference_frame = "world";
            std::cout << "first_time" << std::endl;
        }
        else{
            output.pose.position.x = 0.0;
            output.pose.position.y = 0.0;
            output.pose.position.z = 0.0;
            output.twist.linear = listener.linear;
            output.twist.angular = listener.angular;
            output.reference_frame = "base";
        }
        

        
        output.model_name = "laikago_gazebo";
        

        move_publisher.publish(output);
        first_time = false;
        // std::cout << "publish" << std::endl;

        loop_rate.sleep();
    }
    
    return 0;
}