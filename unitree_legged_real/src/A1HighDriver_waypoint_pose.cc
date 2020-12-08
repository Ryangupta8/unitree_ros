
// Author Ryan Gupta

#include "unitree_legged_real/A1HighDriver.h"
// #include "convert.h"


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

using namespace UNITREE_LEGGED_SDK;
using namespace std;


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
    A1HighDriver driver;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher autonomy_pub = n.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);
    ros::Subscriber goal_sub= n.subscribe("/move_base_simple/goal", 1000, &Listener::goal_callback, &listener);

    nav_msgs::Odometry odom;
    std_msgs::Bool bool_msg; bool_msg.data = true;

    tf2_ros::StaticTransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    // Variables for waypoint posing
    bool change_goal_bool = false;
    double prev_goal_x, prev_goal_y;
    prev_goal_x = 0; prev_goal_y = 0;
    int j = 0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (ros::ok()){
        ros::spinOnce();

        autonomy_pub.publish(bool_msg);

        driver.motiontime += 2;
        driver.GetState();

        current_time = ros::Time::now();

        // Compute Odom in typical fashion from robot velocity
        double dt = (current_time - last_time).toSec();
        double delta_x = (driver.RecvHighROS.forwardSpeed * cos(th) - driver.RecvHighROS.sideSpeed * sin(th)) * dt;
        double delta_y = (driver.RecvHighROS.forwardSpeed * sin(th) + driver.RecvHighROS.sideSpeed * cos(th)) * dt;
        double delta_th = driver.RecvHighROS.rotateSpeed * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // Get quaternion for Odom
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        // Broadcast the whole transform using tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        // Now we fill odometry msg and publish 
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        // set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // set the velocity
        odom.child_frame_id = "base";
        odom.twist.twist.linear.x = driver.RecvHighROS.forwardSpeed;
        odom.twist.twist.linear.y = driver.RecvHighROS.sideSpeed;
        odom.twist.twist.angular.z = driver.RecvHighROS.rotateSpeed;

        // publish the message
        odom_pub.publish(odom);

        if((prev_goal_x != listener.goal_x && prev_goal_y != listener.goal_y)){
            change_goal_bool = true;
        }

        if(change_goal_bool){
            driver.SendHighROS.mode = 1;
            // cout << "j = " << j << endl;
            if(j < 2000){
                driver.SendHighROS.roll  = 0.0f;
                driver.SendHighROS.pitch = -1.0f;
                driver.SendHighROS.yaw = 0.0f; 
                // cout << "s1" << endl;
            }
            else if(j <= 6000){
                driver.SendHighROS.mode = 1;
                driver.SendHighROS.roll  = 0.0f;
                driver.SendHighROS.pitch = -0.8f;
                driver.SendHighROS.yaw = -0.8f;
                // cout << "s2" << endl; 
            }
            else if(j <= 7000){
                driver.SendHighROS.forwardSpeed = 0.0f;
                driver.SendHighROS.sideSpeed = 0.0f;
                driver.SendHighROS.rotateSpeed = 0.0f;
                driver.SendHighROS.bodyHeight = 0.0f;

                driver.SendHighROS.mode = 1;
                driver.SendHighROS.roll  = 0;
                driver.SendHighROS.pitch = 0;
                driver.SendHighROS.yaw = 0;
                // cout << "s3" << endl; 
            }
            else if(j <= 10000){
                driver.SendHighROS.mode = 1;
                driver.SendHighROS.roll  = 0.0f;
                driver.SendHighROS.pitch = -0.8f;
                driver.SendHighROS.yaw = 0.8f;
                // cout << "s4" << endl;
            }
            else if(j <= 14000){
                driver.SendHighROS.roll  = 0.0f;
                driver.SendHighROS.pitch = 0.0f;
                driver.SendHighROS.yaw = 0.0f; 
                // cout << "s5" << endl;                 
            }
            else{
                driver.SendHighROS.forwardSpeed = 0.0f;
                driver.SendHighROS.sideSpeed = 0.0f;
                driver.SendHighROS.rotateSpeed = 0.0f;
                driver.SendHighROS.bodyHeight = 0.0f;

                driver.SendHighROS.mode = 1;
                driver.SendHighROS.roll  = 0;
                driver.SendHighROS.pitch = 0;
                driver.SendHighROS.yaw = 0;
                change_goal_bool = false;
 
            }
           
            
        }
        else{
            if(listener.dx == 0 && listener.dy == 0 && listener.drz == 0){
                driver.SendHighROS.forwardSpeed = 0.0f;
                driver.SendHighROS.sideSpeed = 0.0f;
                driver.SendHighROS.rotateSpeed = 0.0f;
                driver.SendHighROS.bodyHeight = 0.0f;

                driver.SendHighROS.mode = 1;
                driver.SendHighROS.roll  = 0;
                driver.SendHighROS.pitch = 0;
                driver.SendHighROS.yaw = 0;
            }
            else{
                driver.SendHighROS.mode = 2;
                driver.SendHighROS.forwardSpeed = listener.dx;
                driver.SendHighROS.sideSpeed = listener.dy;
                driver.SendHighROS.rotateSpeed = listener.drz;
                
            }
            j = 0;
        }
        
 
        j += 2;  

        prev_goal_x = listener.goal_x;
        prev_goal_y = listener.goal_y;

        driver.SendCmd();

        last_time = current_time;
        loop_rate.sleep();

    }
    return 0;

}
