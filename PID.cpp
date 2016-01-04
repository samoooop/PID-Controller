#include <cstdio>
#include <ctime>
#include <ros/ros.h>
#define abs(a) a<0?-a:a
const static double minIntegral = 0.005;
class SPID{
  protected:
    double KP;
    double KI;
    double KD;
    ros::Time time;
    double sumOfError;
    double previousError;
    double diffError;
    bool fixed;
    double ttl;
  public:
    SPID(double KP,double KI,double KD);
    SPID();
    double pid(double error);
    void resetPID();
    void setK(double,double,double);
    ros::Time getTime();
};
SPID::SPID(){
  ros::Time::init();
  KP = 1;
  KI = 1;
  KD = 1;
  ttl=1000;
  this->time = ros::Time::now();
}
SPID::SPID(double KP,double KI,double KD){
  ros::Time::init();
  this->KP = KP;
  this->KI = KI;
  this->KD = KD;
  ttl=1000;
  this->time = ros::Time::now();
}

double SPID::pid(double error){
  ros::Time tmptime = this->time;
  this->time = ros::Time::now();
  double dt = time.toSec() - tmptime.toSec();
  printf("dt %lf \n",dt);
  diffError=error-previousError;
  sumOfError+=dt*(diffError);
  double result=error*KP+diffError*KD/dt+sumOfError*KI;
  previousError=error;
  ttl--;
  if(ttl==0){
    ttl=1000;
    sumOfError=0.00;
  }
  return result;
}
void SPID::resetPID(){
  this->time = ros::Time::now();
  this->sumOfError = 0;
  this->previousError = 0;
  this->diffError = 0;
}
void SPID::setK(double KP,double KI,double KD){
  this->KP = KP;
  this->KI = KI;
  this->KD = KD;
}ros::Time SPID::getTime(){
  return this->time;
}
