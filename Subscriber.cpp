#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>
#include <cstdio>
nav_msgs::Odometry currentState;
void callback(const nav_msgs::Odometry msg){
	//ROS_INFO_STREAM("Got something " << msg.pose.pose.position.x);
	printf("x:%lf y:%lf z:%lf\n",msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
}
void callback2(const nav_msgs::Odometry msg){
	//ROS_INFO_STREAM("Got something " << msg.pose.pose.position.x);
	printf("2nd node handle -> x:%lf y:%lf z:%lf\n",msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
}
int main(int argc,char **argv){
	ros::init(argc,argv,"subscribe_to_speed");
	ros::NodeHandle nh;
	ros::NodeHandle nh2;

	ros::Subscriber sub = nh.subscribe("auv/state", 1000, &callback);
	ros::Subscriber sub2 = nh2.subscribe("auv/state", 1000, &callback2);
	//ros::spin();
	ros::spinOnce();
	printf("Hello\n");
}