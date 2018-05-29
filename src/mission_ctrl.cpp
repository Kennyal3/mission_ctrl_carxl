#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "mission_ctrl/lidar_ctrl.h"

using namespace std;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void lidarCallback(const mission_ctrl::lidar_ctrl::ConstPtr& lidar_cmd);

//void laneDataCallback(const std_msgs::Float32::ConstPtr& laneData);

bool start = false;
bool reverse_button = false;
double steer = 0.0;
double vel = 1.0;

bool take_left = false;
bool take_right = false;
bool stop = false;
bool resume = false;
float steer_correction = 0.0;

int main(int argc, char *argv[]){
  double steer_auto = 0.0;
  double vel_auto = 0.0;

  ros::Subscriber joy_sub, lidar_ctrl_sub; //laneData;
  ros::Publisher twist_pub;
  geometry_msgs::Twist twist;
  ros::init(argc, argv, "mission_control");
  ros::NodeHandle nh;
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);
  lidar_ctrl_sub = nh.subscribe<mission_ctrl::lidar_ctrl>("Lidar_data", 10, &lidarCallback);
  //laneData = nh.subscribe<std_msgs::Float32>("laneData", 1000, &laneDataCallback);
  twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  cout << "Mission Control\n";
  ros::Rate loop_rate(10);
  while(ros::ok()){
    cout << "start: " << start << " steer_auto: " << steer_auto << " take_right: "
      << take_right << " take_left: " << take_left << " resume: " << resume
      << " stop: " << stop << endl;


    if(!start){
      twist.angular.z = steer;
      twist.linear.x = vel;
      twist.linear.z = reverse_button;
    }
    else if(start){
      if(take_right){
        vel_auto = -0.23;
        steer_auto = -1.0;
      }
      else if(take_left){
        vel_auto = -0.23;
        steer_auto = 1.0;
      }
      else if(resume){
        steer_auto = steer_correction*0.475;
        vel_auto = -0.23;
      }
      else if(stop){
        vel_auto = 1;
      }
      else{
        vel_auto = 0.0;
        steer_auto = 0.0;
      }
      twist.angular.z = steer_auto;
      twist.linear.x = vel_auto;
    }

    twist_pub.publish(twist);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  if(!start && joy->buttons[7])
    start = true;
  else if(start && joy->buttons[7])
    start = false;

  reverse_button = joy->buttons[0];
  steer = joy->axes[0];
  vel = joy->axes[5];
}

void lidarCallback(const mission_ctrl::lidar_ctrl::ConstPtr& lidar_cmd){
  take_left = lidar_cmd->is_left;
  take_right = lidar_cmd->is_right;
  stop = lidar_cmd->stops;
  resume = lidar_cmd->resume_image;
  steer_correction = lidar_cmd->lane_correct;
}
