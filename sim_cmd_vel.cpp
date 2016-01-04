#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
//
// for testing purpose
//
int main(int argc,char **argv) {
	ros::init(argc,argv, "test_cmd_vel");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("sim/cmd_vel",1000);
	ros::Rate rate(2);
	printf("***SENDING velocity(Twist) TO sim/cmd_vel***");
	while(ros::ok) {
		geometry_msgs::Twist msg;
		msg.linear.z  = -5;
		pub.publish(msg);
		rate.sleep();
	}
	ros::shutdown();
}


