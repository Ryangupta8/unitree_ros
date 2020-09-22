// ROS Modules
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
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
    double goal_x[4]; double goal_y[4]; double goal_th[4];
    goal_x[0] = 7.43; goal_y[0] = 55.13; goal_th[0] = -1.57;
    goal_x[1] = 7.43; goal_y[1] = 50.13; goal_th[1] = -1.57;
    goal_x[2] = 7.43; goal_y[2] = 55.13; goal_th[2] = 1.57;
    goal_x[3] = 5.70; goal_y[3] = 59.50; goal_th[3] = 3.14;

    geometry_msgs::Quaternion goal_quat; 
    int i = 0;

    double distance;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    // Get the quaternion of the goal for PoseStamped
    goal_quat = tf::createQuaternionMsgFromYaw(goal_th[i]);
    // Fill the Goal msg
    goal_msg.header.stamp = current_time;
    goal_msg.pose.position.x = goal_x[i]; goal_msg.pose.position.y = goal_y[i]; goal_msg.pose.position.z = 0.;
    goal_msg.pose.orientation = goal_quat;

    bool first_time = true;

    while (ros::ok()){

        test_pub.publish(test_msg);
        
        current_time = ros::Time::now();
        // if(first_time){
        //     goal_pub.publish(goal_msg);
        //     cout << "first time" << endl;
        //     first_time = false;
        // }
		
		distance = sqrt( pow((listener.pose.position.x - goal_x[i]), 2.0) + pow( (listener.pose.position.y - goal_y[i]), 2.0) );
        // cout << "distance = " << distance << endl;

		if(distance <= 0.15){
			++i;
			// Get the quaternion of the goal for PoseStamped
            goal_msg.header.stamp = current_time;
		    goal_quat = tf::createQuaternionMsgFromYaw(goal_th[i]);
            if(i >= 4){ros::shutdown();}
		    // Fill the Goal msg
		    goal_msg.pose.position.x = goal_x[i]; goal_msg.pose.position.y = goal_y[i]; 
		    goal_msg.pose.orientation = goal_quat;
		    
		}
        goal_pub.publish(goal_msg);

		last_time = current_time;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

	
}
