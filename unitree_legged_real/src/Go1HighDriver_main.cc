/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
// Author: Ryan Gupta

#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "convert.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

using namespace UNITREE_LEGGED_SDK;
using namespace std;

bool command_received;

class Listener{
public:
  unitree_legged_msgs::HighCmd command;

  void cmd_callback(const unitree_legged_msgs::HighCmd::ConstPtr& msg)
  {
    command.head[0] = msg->head[0];
    command.head[1] = msg->head[1];
    command.mode = msg->mode;
    command.gaitType = msg->gaitType;
    command.speedLevel = msg->speedLevel;
    command.footRaiseHeight = msg->footRaiseHeight;
    command.bodyHeight = msg->bodyHeight;
    command.position[0] = msg->position[0];
    command.position[1] = msg->position[1];
    command.euler[0] = msg->euler[0];
    command.euler[1] = msg->euler[1];
    command.euler[2] = msg->euler[2];
    command.velocity[0] = msg->velocity[0];
    command.velocity[1] = msg->velocity[1];
    command.yawSpeed = msg->yawSpeed;
    command.led[0].g = msg->led[0].g;
    command.led[1].g = msg->led[1].g;
    command.led[2].g = msg->led[2].g;
    command.led[3].g = msg->led[3].g;


    if(!command_received) ROS_INFO("Command Topic received");

    command_received = 1;
  }
};

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
  std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
            << "Make sure the robot is standing on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();

  ros::NodeHandle n;
  ros::Rate loop_rate(500);

  command_received = 0;

  Listener listener;

  // SetLevel(HIGHLEVEL);
  long motiontime = 0;
  TCmd SendHighLCM = {0};
  TState RecvHighLCM = {0};
  unitree_legged_msgs::HighCmd SendHighROS;
  unitree_legged_msgs::HighState RecvHighROS;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/go1_odom", 1000);
  ros::Publisher autonomy_pub = n.advertise<std_msgs::Bool>("/autonomy_arbiter/enabled", 1000);
  ros::Subscriber cmd_vel_sub = n.subscribe("/go1_cmd", 1000, &Listener::cmd_callback, &listener);

  nav_msgs::Odometry odom;
  std_msgs::Bool bool_msg; bool_msg.data = true;

  tf2_ros::StaticTransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  roslcm.SubscribeState();

  pthread_t tid;
  pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

  while (ros::ok()){
    motiontime = motiontime+2;

    autonomy_pub.publish(bool_msg);

    roslcm.Get(RecvHighLCM);
    RecvHighROS = ToRos(RecvHighLCM);

    double dt = (current_time - last_time).toSec();
    double delta_th = RecvHighROS.velocity[2] * dt;
    double delta_x = (RecvHighROS.velocity[0] * cos(th) - RecvHighROS.velocity[1] * sin(th)) * dt;
    double delta_y = (RecvHighROS.velocity[0] * sin(th) + RecvHighROS.velocity[1] * cos(th)) * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base";
    odom.twist.twist.linear.x = RecvHighROS.velocity[0];
    odom.twist.twist.linear.y = RecvHighROS.velocity[1];
    odom.twist.twist.angular.z = RecvHighROS.velocity[2];
    odom_pub.publish(odom);

    SendHighROS.head[0] = 0;
    SendHighROS.head[1] = 255;
    SendHighROS.mode = 0;
    SendHighROS.gaitType = 0; // listener.command.gaitType;
    SendHighROS.speedLevel = 0; // listener.command.speedLevel;
    SendHighROS.footRaiseHeight = 0; // listener.command.footRaiseHeight;
    SendHighROS.bodyHeight = -0.1; // listener.command.bodyHeight;
    SendHighROS.euler[0] = 0.2;
    SendHighROS.euler[1] = 0;
    SendHighROS.euler[2] = 0;
    SendHighROS.velocity[0] = 0.0f;
    SendHighROS.velocity[1] = 0.0f;
    SendHighROS.yawSpeed = 0.0f;
    SendHighROS.reserve = 0;
    SendHighROS.led[0].g = 255;
    SendHighROS.led[1].g = 255;
    SendHighROS.led[2].g = 255;
    SendHighROS.led[3].g = 255;

    // SendHighROS.head[0] = listener.command.head[0];
    // SendHighROS.head[1] = listener.command.head[1];
    // SendHighROS.mode = listener.command.mode;
    // SendHighROS.gaitType = listener.command.gaitType;
    // SendHighROS.speedLevel = listener.command.speedLevel;
    // SendHighROS.footRaiseHeight = listener.command.footRaiseHeight;
    // SendHighROS.bodyHeight = listener.command.bodyHeight;
    // SendHighROS.position[0] = listener.command.position[0];
    // SendHighROS.position[1] = listener.command.position[1];
    // SendHighROS.euler[0] = listener.command.euler[0];
    // SendHighROS.euler[1] = listener.command.euler[1];
    // SendHighROS.euler[2] = listener.command.euler[2];
    // SendHighROS.velocity[0] = listener.command.velocity[0];
    // SendHighROS.velocity[1] = listener.command.velocity[1];
    // SendHighROS.yawSpeed = listener.command.yawSpeed;
    // SendHighROS.led[0].g = listener.command.led[0].g;
    // SendHighROS.led[1].g = listener.command.led[1].g;
    // SendHighROS.led[2].g = listener.command.led[2].g;
    // SendHighROS.led[3].g = listener.command.led[3].g;


    SendHighLCM = ToLcm(SendHighROS, SendHighLCM);
    roslcm.Send(SendHighLCM);
    ros::spinOnce();
    loop_rate.sleep(); 
  }
  return 0;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "go1_driver_node");

  UNITREE_LEGGED_SDK::LCM roslcm(UNITREE_LEGGED_SDK::HIGHLEVEL);
  mainHelper<UNITREE_LEGGED_SDK::HighCmd, UNITREE_LEGGED_SDK::HighState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);

}
