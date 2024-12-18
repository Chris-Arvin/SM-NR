#ifndef SIMULATE_AUTOMATED_CAR_H
#define SIMULATE_AUTOMATED_CAR_H

#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vehicle_simulator/SocialVehicles.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double wheelbase = 0.144;
float LIN_VEL_STEP_SIZE = 0.02;
float ANG_VEL_STEP_SIZE = 0.02;
double max_vel_x = 0.5; //最大x前向速度
double max_vel_theta = 1.5;  //最大转向角速度
double max_steering_angle = 0.350; //最大转向角度
double acc_lim_x = 0.2; //最大x向加速度
double acc_lim_theta = 0.3;  //最大角加速度

class SimulateSV {
 public:
  SimulateSV(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle);
  ~SimulateSV();
  void initializeSimulation();
  void publishSVPositions();
  void publishSVFutures();
  void runSimulation();
  void onTwistReceived1(const geometry_msgs::Twist::ConstPtr& twist);
  void onTwistReceived2(const geometry_msgs::Twist::ConstPtr& twist);
  void onTwistReceived3(const geometry_msgs::Twist::ConstPtr& twist);

  
 private:
  ros::NodeHandle nh_, privatenh_;
  // publishers
  double g_updateRate_, g_simulationFactor_;
  std::string g_worldFrame_, g_robotFrame_;

  std::vector<geometry_msgs::Twist> g_currentTwist_;   //质心的速度v和w
  std::vector<geometry_msgs::Twist> g_currentCommand_;   //来自topic的命令
  std::vector<tf::Transform> g_currentPose_; //质心的位置x，y，theta
  std::vector<double> steering_angle_;   //前轮转向角
  boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster_;
  boost::mutex mutex1_, mutex2_, mutex3_;
  int all_vehicle_number_, dynamic_vehicle_number_;

  ros::Publisher pub_dynamic_sv_positions_;
  ros::Publisher pub_dynamic_sv_polys_;
  ros::Publisher pub_dynamic_sv_poses_;
  ros::Publisher pub_static_sv_positions_;
  ros::Publisher pub_static_sv_polys_;
  ros::Publisher pub_static_sv_poses_;
  ros::Subscriber twist_subscriber1_;
  ros::Subscriber twist_subscriber2_;
  ros::Subscriber twist_subscriber3_;

};

#endif
