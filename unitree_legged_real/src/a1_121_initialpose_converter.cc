// Author: Ryan Gupta
//
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <amrl_msgs/Localization2DMsg.h>
#include <math.h>

#include "ros/ros.h"

bool received = false;

class Listener{
  public:
    double x, y, yaw;

    void initialpose_cb(const amrl_msgs::Localization2DMsg::ConstPtr& msg)
    {
        x = msg->pose.x;
        y = msg->pose.y;
        yaw = msg->pose.theta;
        received =true;


        //geometry_msgs::PoseWithCovarianceStamped pose_msg;
    }
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "a1_121_initialpose_convert");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::Rate loop_rate(10);

    Listener listener;

    ros::Subscriber initposesub = n.subscribe("initialpose", 1000, &Listener::initialpose_cb, &listener);
    ros::Publisher initpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose_a1_121", 1000);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;

    ros::Time current_time;
    current_time = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();

        pose_msg.pose.pose.position.x = listener.x;
        pose_msg.pose.pose.position.y = listener.y;
        // Get quat from yaw
        double qw = cos(listener.yaw/2);
        double qz = sin(listener.yaw/2);
        pose_msg.pose.pose.orientation.w = qw;
        pose_msg.pose.pose.orientation.z = qz;

        if(received)
        {
            initpose_pub.publish(pose_msg);
            received=false;

        }
        loop_rate.sleep();
    }


}
