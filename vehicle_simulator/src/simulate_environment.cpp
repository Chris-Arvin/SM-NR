#include <vehicle_simulator/simulate_environment.h>

Simulator::Simulator(const ros::NodeHandle& node) : nh_(node) {
}

Simulator::~Simulator() {
  // shutdown  and publishers
  pub_obstacles_visuals_.shutdown();
}

bool Simulator::initializeSimulation() {
  // setup ros publishers
  pub_obstacles_visuals_ = nh_.advertise<visualization_msgs::Marker>("walls", 1, true);
  pub_centerline_visuals_ = nh_.advertise<visualization_msgs::Marker>("centerline", 1, true);
  // load additional parameters
  std::string scene_file_param;
  nh_.param<std::string>("scene_file", scene_file_param, "");
  if (scene_file_param == "") {
    ROS_ERROR_STREAM("Invalid scene file: " << scene_file_param);
    return false;
  }

  ROS_INFO_STREAM("Loading scene [" << scene_file_param << "] for simulation");

  const QString scenefile = QString::fromStdString(scene_file_param);
  if (scenario_reader_.readFromFile(scenefile) == false) {
    ROS_ERROR_STREAM(
        "Could not load the scene file, please check the paths and param "
        "names : "
        << scene_file_param);
    return false;
  }
  nh_.param<double>("update_rate", updateRate_, 25.0);
  nh_.param<std::string>("frame_id", frame_id_, "odom");
  return true;
}

void Simulator::runSimulation() {
  ros::Rate r(updateRate_);

  while (ros::ok()) {
    publishObstacleVisuals();
    publishCenterlineVisuals();
    ros::spinOnce();
    r.sleep();
  }
}

void Simulator::publishCenterlineVisuals(){
  visualization_msgs::Marker centerline_marker;
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  centerline_marker.header = msg_header;
  centerline_marker.color.r = 0.2;
  centerline_marker.color.g = 0.2;
  centerline_marker.color.b = 0.2;
  centerline_marker.color.a = 0.5;
  centerline_marker.scale.x = 0.02; 
  centerline_marker.pose.orientation.w = 1.0;
  centerline_marker.type = visualization_msgs::Marker::LINE_STRIP;

  float x_min=999, x_max=-999;
  for (auto obstacle : scenario_reader_.getObs()) {
    x_min = std::min(x_min, obstacle[0]);
    x_min = std::min(x_min, obstacle[2]);
    x_max = std::max(x_max, obstacle[0]);
    x_max = std::max(x_max, obstacle[2]);
  }

  geometry_msgs::Point p;
  p.y = 0;
  float step=0.3, length=0.2;
  int i=0;
  for (float pos=x_min; pos<x_max; pos+=step){
    centerline_marker.header.stamp = ros::Time::now();
    centerline_marker.id = i++;
    centerline_marker.points.clear();
    p.x = pos;
    centerline_marker.points.push_back(p);
    p.x = std::min(pos+length, x_max);
    centerline_marker.points.push_back(p);
    pub_centerline_visuals_.publish(centerline_marker); 
  }
}


void Simulator::publishObstacleVisuals() {
  visualization_msgs::Marker walls_marker;
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  walls_marker.header = msg_header;
  walls_marker.id = 10000;
  walls_marker.color.a = 1.0;
  walls_marker.color.r = 1.0;
  walls_marker.color.g = 1.0;
  walls_marker.color.b = 1.0;
  walls_marker.scale.x = 0.01; 
  walls_marker.scale.y = 0.01;
  walls_marker.scale.z = 0.04;
  walls_marker.pose.position.z = walls_marker.scale.z / 2.0;
  walls_marker.pose.orientation.w = 1.0;
  walls_marker.type = visualization_msgs::Marker::CUBE_LIST;

  for (auto obstacle : scenario_reader_.getObs()) {
    geometry_msgs::Point p;
    p.x = obstacle[0];
    p.y = obstacle[1];
    p.z = 0.0;
    walls_marker.points.push_back(p);
    p.x = obstacle[2];
    p.y = obstacle[3];
    p.z = 0.0;
    walls_marker.points.push_back(p);
  }

  pub_obstacles_visuals_.publish(walls_marker);
}

int main(int argc, char** argv) {
  // initialize resources
  ros::init(argc, argv, "simulate_environment");
  ros::NodeHandle node("~");
  Simulator sm(node);

  if (sm.initializeSimulation()) {
    ROS_INFO("node initialized, now running ");
    sm.runSimulation();
  } else {
    ROS_WARN("Could not initialize simulation, aborting");
    return EXIT_FAILURE;
  }

}