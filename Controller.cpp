#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include "PID.cpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Vector3.h>
#define abs(a) a<0?-a:a
#define epsilon 1.0e-7
#define	MIN_ERROR 1.0e-3
double cmdVelK[][3] = {{1,0,0},
					{1,0,0},
					{1,0,0},
					{0.3,0,0},
					{0.3,0,0},
					{0.4,0,0},
					}; //KP,KI,KD KP only cause order of acceleration
double fixPointK[][3] = {{2,0.1,0.2},
					{2,0.1,0.2},
					{4,0.1,0.2},
					{1,0.1,0.01},
					{1,0.1,0.01},
					{1,0.1,0.01},
					}; // KP,KI,KD
nav_msgs::Odometry previousState;
nav_msgs::Odometry currentState;
double cmd_vel[6]={1,0,0,0,0,0},position[6],vel[6];
double prevPosition[6],prevVel[6];
bool fixed[] = {true,true,true,true,true,true};
double fixedPosition[6] = {1,1,-1,0,0,0}; // x y z ? ? ?;
SPID pid[6];

void stateListenerCallBack(const nav_msgs::Odometry msg);
void cmd_velCallBack(const geometry_msgs::Twist msg);
void changeFixedState();
void setPreviousState();
bool equal(double a, double b);
bool isClose(double a, double b);
double* calculateError();
double angleError(double a,double b);
geometry_msgs::Twist calculatePID();


void init(){
	for(int i=0;i<6;i++){
		vel[i]=0;
		cmd_vel[i]=0;
		position[i]=0;
		if(fixed[i]){
			pid[i].setK(fixPointK[i][0],fixPointK[i][1],fixPointK[i][2]);
		}else{
			pid[i].setK(cmdVelK[i][0],cmdVelK[i][1],cmdVelK[i][2]);
		}
	}
}

int main(int argc,char **argv) {
	init();
	ros::init(argc,argv, "Controller");
	ros::NodeHandle nh;
	ros::Subscriber sub_state = nh.subscribe("auv/state", 1000, &stateListenerCallBack);
	ros::Subscriber sub_cmd_vel = nh.subscribe("sim/cmd_vel", 1000, &cmd_velCallBack);
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("zeabus/cmd_vel",1000);
	ros::Rate rate(100);
	while(ros::ok) {
		setPreviousState();
		ros::spinOnce();
		pub.publish(calculatePID());
		//printf("Current velo : %lf %lf %lf (%lf %lf %lf)\n",vel[0],vel[1],vel[2],cmd_vel[0],cmd_vel[1],cmd_vel[2]);
		printf(" x = %lf \n",position[3]);
		printf("x=%lf y=%lf z=%lf\n",position[0],position[1],position[2]);	
		printf("roll=%lf pitch=%lf yaw=%lf\n",position[3],position[4],position[5]);
		rate.sleep();
	}
	ros::shutdown();
}

void stateListenerCallBack(const nav_msgs::Odometry msg){
	currentState = msg;
	position[0] = msg.pose.pose.position.x;
	position[1] = msg.pose.pose.position.y;
	position[2] = msg.pose.pose.position.z;
	tfScalar roll,pitch,yaw;
	tf::Quaternion	quat(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
	position[3] = roll;
	position[4] = pitch;
	position[5] = yaw;
	//position[3]=msg.pose.pose.orientation.x;position[4]=msg.pose.pose.orientation.y;position[5]=msg.pose.pose.orientation.z;
	//printf("x=%lf y=%lf z=%lf\n",position[0],position[1],position[2]);	
	//printf("roll=%lf pitch=%lf yaw=%lf\n",position[3],position[4],position[5]);
	vel[0] = msg.twist.twist.linear.x;
	vel[1] = msg.twist.twist.linear.y;
	vel[2] = msg.twist.twist.linear.z;
	vel[3] = msg.twist.twist.angular.x;
	vel[4] = msg.twist.twist.angular.y;
	vel[5] = msg.twist.twist.angular.z;
}

void cmd_velCallBack(const geometry_msgs::Twist msg){
	cmd_vel[0] =msg.linear.x;
	cmd_vel[1] =msg.linear.y;
	cmd_vel[2] =msg.linear.z;
	cmd_vel[3] =msg.angular.x;
	cmd_vel[4] =msg.angular.y;
	cmd_vel[5] =msg.angular.z;
}

void setPreviousState(){
	for(int i=0;i<6;i++){
		prevPosition[i] = position[i];
		prevVel[i] = position[i];
	}
}
void changeFixedState(){
	for(int i = 0;i < 6;i++){
		if(equal(0.0,cmd_vel[i]) && !fixed[i] && isClose(vel[i],0.0)){
			pid[i].resetPID();
			pid[i].setK(fixPointK[i][0],fixPointK[i][1],fixPointK[i][2]);
			fixed[i] = true;
			fixedPosition[i] = position[i];
		}else if(!equal(0.0,cmd_vel[i]) && fixed[i]){
			pid[i].resetPID();
			pid[i].setK(cmdVelK[i][0],cmdVelK[i][1],cmdVelK[i][2]);
			fixed[i] = false;
		}
	}
}

double* calculateError(){
	double *error = (double*)calloc(6,sizeof(double));
	for(int i = 0;i < 6;i++){
		if(fixed[i]){
			if(i<3)
				error[i] = fixedPosition[i]-position[i];
			else
				error[i] = angleError(fixedPosition[i],position[i]);
		}else {
			error[i] = cmd_vel[i] - vel[i];
		}
		printf("error %lf",error[i]);
	}
	printf("\n");
	return error;
}

geometry_msgs::Twist calculatePID(){
	geometry_msgs::Twist t;
	changeFixedState();
	double *error = calculateError();
	t.linear.x = pid[0].pid(error[0]);
	t.linear.y = pid[1].pid(error[1]);
	t.linear.z = pid[2].pid(error[2]);
	t.angular.x = pid[3].pid(error[3]);
	t.angular.y = pid[4].pid(error[4]);
	t.angular.z = pid[5].pid(error[5]);
	printf("pid x = %lf ",t.angular.x);
	return t;
}
double revAng(double a){
	if(a>0)
		return -M_PI-(180-a);
	return M_PI+(a+M_PI);
}
/*double angleError(double a,double b){
	if(a > b){
		double temp = b;
		b = a;
		a = temp;
	}
	double angleDiff = abs(a-b);
	return M_PI-abs(a-b)<abs(a-b)? M_PI-abs(a-b):abs(a-b);
}*/
double angleError(double a,double b){
	if (abs(a-b)<abs(a-revAng(b)))
		return a-b;
	return a-revAng(b);
}
bool equal(double a, double b){ // for double error
    return fabs(a - b) < epsilon;
}

bool isClose(double a,double b){ // for sensor error
	return fabs(a - b) < MIN_ERROR;
}
