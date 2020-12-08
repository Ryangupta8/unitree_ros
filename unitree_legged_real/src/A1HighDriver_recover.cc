
// Author Ryan Gupta

#include "unitree_legged_real/A1HighDriver.h"
// #include "convert.h"


#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
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

    sensor_msgs::LaserScan scan;

    void twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        dx = msg->linear.x;
        dy = msg->linear.y;
        drz = msg->angular.z;
        // cout << "Twist Received"  << endl;
    }

    void laser_callback(const sensor_msgs::LaserScan msg){
        scan = msg;
    }
};

bool GetTurnDirection(sensor_msgs::LaserScan *scan){
    bool left_bool = true;
    double left_avg, right_avg;
    double sum = 0; double count = 0;
    // std::cout << "scan->range_min = " << scan->range_min << "   " << "scan->range_max = " << scan->range_max << std::endl;
    // std::cout << "scan->angle_min = " << scan->angle_min << "   " << "scan->angle_max = " << scan->angle_max << std::endl;
    // std::cout << "scan->angle_increment = " << scan->angle_increment << std::endl;
    
    // 639 data entries in /depth2scan output from Realsense
    // 320 is the middle. Let's look at the inside portion.
    for(int i=250; i<320; ++i){
        if( (!isnan(scan->ranges[i])) && (scan->ranges[i]<scan->range_max) && (scan->ranges[i]>scan->range_min) ) {
            ++count; sum += scan->ranges[i];
        }
    }
    left_avg = sum / count;
    count = 0; sum = 0;
    for(int i=320; i<390; ++i){
        if( (!isnan(scan->ranges[i])) && (scan->ranges[i] < scan->range_max) && (scan->ranges[i] > scan->range_min) ) 
        ++count; sum += scan->ranges[i];
    }
    right_avg = sum/count;
    if(isnan(right_avg)){
        right_avg = 0;
    }
    if(isnan(left_avg)){
        left_avg = 0;
    }
    if(right_avg > left_avg){
        left_bool = false;
    }
    std::cout << "left_avg = " << left_avg << std::endl;
    std::cout << "right_avg = " << right_avg << std::endl;

    return left_bool;
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

    Listener listener;
    A1HighDriver driver;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher autonomy_pub = n.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, &Listener::twist_callback, &listener);
    ros::Subscriber depth2scan_sub = n.subscribe("/depth2scan", 100, &Listener::laser_callback, &listener);

    nav_msgs::Odometry odom;
    std_msgs::Bool bool_msg; bool_msg.data = true;

    tf2_ros::StaticTransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    bool left = true; bool standing_still_bool = false; bool started = false;
    int j, k; j = 0; k = 0;

    ros::Time current_time, last_time, prev_recovery_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    prev_recovery_time = ros::Time::now();

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

        if(standing_still_bool){// Enter recovery
            k += 2;
            std::cout << "standing still was recognized as true" << endl;
            if(k<4000){
                driver.SendHighROS.mode = 2; // Continuous Walk
                if(left){
                    driver.SendHighROS.rotateSpeed = 0.196f; 
                }
                else{ 
                    driver.SendHighROS.rotateSpeed = -0.196f;
                } 
                std::cout << "turning loop" << std::endl;
            }
            else{
                driver.SendHighROS.forwardSpeed = 0.0f;
                driver.SendHighROS.sideSpeed = 0.0f;
                driver.SendHighROS.rotateSpeed = 0.0f;
                driver.SendHighROS.bodyHeight = 0.0f;

                driver.SendHighROS.mode = 1; // Forced Stand
                driver.SendHighROS.roll  = 0;
                driver.SendHighROS.pitch = 0;
                driver.SendHighROS.yaw = 0;
                prev_recovery_time = ros::Time::now();
                standing_still_bool = false;
                std::cout << "finished turning, setting still to false" << std::endl;
            }
            j = 0;
        }
        else{
            cout << "standing still recognized as false" << endl;
            // If cmd_vel is 0,0,0 then stand still
            if(listener.dx == 0 && listener.dy == 0 && listener.drz == 0){
                driver.SendHighROS.forwardSpeed = 0.0f;
                driver.SendHighROS.sideSpeed = 0.0f;
                driver.SendHighROS.rotateSpeed = 0.0f;
                driver.SendHighROS.bodyHeight = 0.0f;

                driver.SendHighROS.mode = 1; // Forced Stand
                driver.SendHighROS.roll  = 0;
                driver.SendHighROS.pitch = 0;
                driver.SendHighROS.yaw = 0;
                // If standing still for 2+ seconds, enter recovery
                j += 2;
                if((current_time - prev_recovery_time).toSec() > 10){
                    cout << "more than 10 sec since we last recovered" << endl;
                    if(started){
                        cout << "started is recognized as true" << endl;
                        if(j > 2000){
                            cout << "2 seconds of standing still" << endl;
                            cout << "calling GetTurnDirection" <<endl;
                            left = GetTurnDirection(&listener.scan);
                            cout << "returned GetTurnDirection" << endl;
                            standing_still_bool = true;
                            cout << "still set to true" << endl;
                        }
                    }
                }
            }
            // Else we want to walk as commanded
            else{
                driver.SendHighROS.mode = 2; // Continuous Walk
                driver.SendHighROS.forwardSpeed = listener.dx;
                driver.SendHighROS.sideSpeed = listener.dy;
                driver.SendHighROS.rotateSpeed = listener.drz;
                started = true; 
                j = 0;
                k = 0;
                cout << "graph_nav command. k= 0, j = 0" << endl;
            }
        }
        driver.SendCmd();

        last_time = current_time;
        loop_rate.sleep();

    }
    return 0;

}
