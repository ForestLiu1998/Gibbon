#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <sensor_msgs/JointState.h>
#define PI 3.1415926

double joint_position;

double lds_ready;

bool sample_flag = false;

double reset_velocity=-0.05;



void ldsPositionCallback(const sensor_msgs::JointState::ConstPtr& joint_states) {
	joint_position = joint_states->position[0];
}

void ldsStartCallback(const std_msgs::Bool::ConstPtr& readysingle){
    lds_ready=readysingle->data;
}

int main(int argc, char **argv) {
  /* if (argc != 1){
      std::cout<<"no enough parameter"<<std::endl
               <<"please a number of rotating degree"<<std::endl
               <<"belonging to (0,1.570795)"<<std::endl;
      return -1;
  }*/
  //设置转速
  double velocity=0.05;
  std::cout<<"setting velocity is "<<velocity<<std::endl;
  
  //ros初始化
  // Publisher:     lds_velocity_pub    Float64      传输移动位置
  //                lds_stop_pub        Bool         发送结束请求
  // Subscriber:    lds_joint_state     JointState   接收角度信息
  //                lds_start_signal    Bool         接收开始请求
  ros::init(argc,argv,"lds_revolute");
  ros::NodeHandle lds_revolute;
  ros::Publisher lds_velocity_pub = lds_revolute.advertise<std_msgs::Float64>(
            "/lds_revolute/lds_revolute_velocity_controller/command",1);
  ros::Publisher lds_stop_pub = lds_revolute.advertise<std_msgs::Bool>(
            "/lds/stop",1);
  ros::Subscriber lds_joint_state = lds_revolute.subscribe<sensor_msgs::JointState>(
            "/lds_revolute/joint_states",1,ldsPositionCallback);
  ros::Subscriber lds_start_signal = lds_revolute.subscribe<std_msgs::Bool>(
            "/lds/start",1,ldsStartCallback);

  ros::Rate loop_rate(1);
  std_msgs::Float64 v;
  std_msgs::Bool sampleFlag;
  std::cout<<"system inited!"<<std::endl;
  double start_position;

  sampleFlag.data=false;

  while(ros::ok()){
      lds_stop_pub.publish(sampleFlag);
      std::cout<<"ready to start,waiting for order"<<std::endl;
      v.data=reset_velocity;
      lds_velocity_pub.publish(v);
      ros::spinOnce();
      while(joint_position>0){
        ros::spinOnce();
      }
      start_position=joint_position;
      ros::spinOnce();
      if(lds_ready){
          std::cout<<"pocessing*******************************************"<<std::endl;
          v.data=velocity;
          lds_velocity_pub.publish(v);
          sampleFlag.data=true;
          while((joint_position-start_position)< PI){
            ros::spinOnce();
          }
      lds_stop_pub.publish(sampleFlag);
      loop_rate.sleep();
      sampleFlag.data = false;
      }
      loop_rate.sleep();
  }

  return 0;

  
  
  
  
}

