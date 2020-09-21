// /// ROS Modules

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>
// #include <tf/transform_broadcaster.h>
// #include <amrl_msgs/Localization2DMsg.h>

// using_namespace std;

// class Listener{
// public:
//     double x, y, th; 


// // Callback function for /cmd_vel subscriber
//     void loc_cb(const amrl_msgs::Localization2DMsg::ConstPtr& msg)
//     {
//         dx = msg->pose.x;
//         dy = msg->pose.y;
//         drz = msg->pose.theta;
//         // cout << "Twist Received"  << endl;
//     }
// };

int main(int argc, char *argv[])
{
	// ros::init(argc, argv, "a1_waypoint_node");
 //    ros::NodeHandle n;
 //    ros::Rate loop_rate(500);

 //    ros::Subscriber loc_sub = n.subscribe("/localization", 1000, &Listener::loc_cb, &listener);
 //    ros::Publisher goal_pub = advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);

 //    geometry_msgs::PoseStamped goal_msg;

 //    ros::Time current_time, last_time;
 //    current_time = ros::Time::now();
 //    last_time = ros::Time::now();

 //    while (ros::ok()){
 //        ros::spinOnce();
 //        current_time = ros::Time::now();
		

	// 	last_time = current_time;
 //        loop_rate.sleep();
 //    }

    return 0;

	// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
}