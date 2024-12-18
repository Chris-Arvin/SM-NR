#ifndef SIMULATE_AUTOMATED_CAR_H
#define SIMULATE_AUTOMATED_CAR_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double wheelbase = 0.144;
float LIN_VEL_STEP_SIZE = 0.02;
float ANG_VEL_STEP_SIZE = 0.02;
double max_vel_x = 0.5; //最大x前向速度
double max_vel_theta = 1.5;  //最大转向角速度
double acc_lim_x = 0.2; //最大x向加速度
double acc_lim_theta = 0.3;  //最大角加速度

class SimulateAV {
 public:
  SimulateAV(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle);
  ~SimulateAV();
  void initializeSimulation();
  void publishRobotPosition();
  void runSimulation();
  void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist);

  
 private:
  ros::NodeHandle nh_, privatenh_;
  // publishers
  double g_updateRate_, g_simulationFactor_;
  std::string g_worldFrame_, g_robotFrame_;

  geometry_msgs::Twist g_currentTwist_;   //质心的速度v和w，来自topic的命令
  tf::Transform g_currentPose_; //质心的位置x，y，theta

  boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster_;
  boost::mutex mutex_;

  ros::Publisher pub_robot_position_;
  ros::Subscriber twist_subscriber_;

};

#endif
