#ifndef SIMULATE_ENVIRONMENT_H
#define SIMULATE_ENVIRONMENT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <vehicle_simulator/scenarioreader.h>

/// \class Simulator
/// \brief Simulation wrapper
class Simulator {
 public:
  explicit Simulator(const ros::NodeHandle& node);
  virtual ~Simulator();
  bool initializeSimulation();
  void runSimulation();


 private:
  void publishObstacleVisuals();
  void publishCenterlineVisuals();

 private:
  ros::NodeHandle nh_;
  // publishers
  ros::Publisher pub_obstacles_visuals_;
  ros::Publisher pub_centerline_visuals_;
  // frame ids
  std::string frame_id_;
  double updateRate_;
  ScenarioReader scenario_reader_;

};

#endif
