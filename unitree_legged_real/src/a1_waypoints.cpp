// ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>

#include <cmath>


using namespace std;

class Listener{
public:
    geometry_msgs::Pose pose;

    void loc_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        pose = msg->pose;
    }
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "a1_waypoint_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    Listener listener;

    ros::Subscriber loc_sub = n.subscribe("/global_pose", 1000, &Listener::loc_cb, &listener);
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    ros::Publisher test_pub = n.advertise<std_msgs::String>("test", 1000);


    std_msgs::String test_msg; test_msg.data = "test";

    // To publish to move_base_simple/goal
    geometry_msgs::PoseStamped goal_msg;

    // Define the hard coded goal points
    double goal_x[12]; double goal_y[12]; double goal_th[12];
    goal_x[0] = 7.43; goal_y[0] = 55.13; goal_th[0] = -1.57; // center of smaller living room
    //goal_x[1] = 7.44; goal_y[1] = 50.13; goal_th[1] = -1.57;
    goal_x[1] = 7.43; goal_y[1] = 50.13; goal_th[1] = -1.57;// center of larger living room
    goal_x[2] = 7.5; goal_y[2] = 45.3; goal_th[2] = 0; // the foorway to back of apt
    goal_x[3] = 11.5; goal_y[3] = 45.75; goal_th[3] = 1.57; // back corner of apt closest to stairs
    goal_x[4] = 11.49; goal_y[4] = 55.5; goal_th[4] = 0.0; // Next to door to hall from kicthen
    goal_x[5] = 14.6; goal_y[5] = 55.9; goal_th[5] = -1.57;// in hallway facing towards drone cage
    goal_x[6] = 14.5; goal_y[6] = 37.5; goal_th[6] = -0.3; // Near drone cage in hall 
    goal_x[7] = 16.0; goal_y[7] = 30.25; goal_th[7] = -1.57; // outside drone cage door
    goal_x[8] = 12.88; goal_y[8] = 32.0; goal_th[8] = 1.57; // in front of vicon control pc
    goal_x[9] = 13.05; goal_y[9] = 39.7; goal_th[9] = 1.75; // Near the corner by stairs before turning
    goal_x[10] = 12.0; goal_y[10] = 41.0; goal_th[10] = 3.13; // Same but after the turn
    goal_x[11] = 4.5; goal_y[11] = 41.5; goal_th[11] = 1.57; // Back around to our desks
    // goal_x[5] = 11.5; goal_y[5] = 61.; goal_th[5] = -1.57;// by robocup @home setup
    //goal_x[6] = 7.4; goal_y[6] = 58.20; goal_th[6] = 3.13; // between table and wall
    //goal_x[7] = 5.2; goal_y[7] = 59.9; goal_th[7] = -1.8; // out in the passageway where person would be 
    // goal_x[2] = 7.43; goal_y[2] = 55.13; goal_th[2] = 1.57;
    //goal_x[3] = 5.70; goal_y[3] = 59.75; goal_th[3] = 3.14;
    // goal_x[3] = 5.40; goal_y[3] = 59.95; goal_th[3] = 3.13;
    // goal_x[4] = 5.0; goal_y[4] = 57.5; goal_th[4] = -1.57;

    geometry_msgs::Quaternion goal_quat, curr_quat; 
    int i = 0;

    double distance;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Get the quaternion of the goal for PoseStamped
    goal_quat = tf::createQuaternionMsgFromYaw(goal_th[i]);
    curr_quat = listener.pose.orientation;
    tf::Quaternion q(curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    cout << "yaw = " << yaw << endl;
    cout << "global_pose:" << endl;
    cout << curr_quat.x << "    " << curr_quat.y << "    " << curr_quat.z << "    " << curr_quat.w << endl;



    double angle_error;
    angle_error = goal_th[i] - yaw;
    cout << "angle error = " << angle_error << endl;

    // Fill the Goal msg
    goal_msg.header.stamp = current_time;
    goal_msg.pose.position.x = goal_x[i]; goal_msg.pose.position.y = goal_y[i]; goal_msg.pose.position.z = 0.;
    goal_msg.pose.orientation = goal_quat;

    bool first_time = true;

    while (ros::ok()){

        test_pub.publish(test_msg);
        
        current_time = ros::Time::now();

		distance = sqrt( pow((listener.pose.position.x - goal_x[i]), 2.0) + pow( (listener.pose.position.y - goal_y[i]), 2.0) );
        // cout << "distance = " << distance << endl;
    


		if(distance <= 0.35){
			++i;
			// Get the quaternion of the goal for PoseStamped
            goal_msg.header.stamp = current_time;
		    goal_quat = tf::createQuaternionMsgFromYaw(goal_th[i]);
            //if(i >= 5){ros::shutdown();}
		    // Fill the Goal msg
		    goal_msg.pose.position.x = goal_x[i]; goal_msg.pose.position.y = goal_y[i]; 
		    goal_msg.pose.orientation = goal_quat;
            //ros::Duration(2.5).sleep();
		    
		}
        goal_pub.publish(goal_msg);

		last_time = current_time;

        cout << "i = " << i << endl;
        if(i >= 12){
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

	
}
