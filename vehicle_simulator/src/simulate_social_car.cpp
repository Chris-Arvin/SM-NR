/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
// 描述一个车的状态： (近似认为后轮无偏转，即 后轮速度等于车的速度、后轮朝向等于车身朝向)
// pose信息：车身的(x,y,theta)
// velocity信息：车身的的速度v和角速度omega。
#include<vehicle_simulator/simulate_social_car.h>

SimulateSV::SimulateSV(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle): nh_(publicHandle), privatenh_(privateHandle){}

SimulateSV::~SimulateSV() {
  // shutdown  and publishers
  pub_dynamic_sv_positions_.shutdown();
  pub_static_sv_positions_.shutdown();
  twist_subscriber1_.shutdown();
  twist_subscriber2_.shutdown();
  twist_subscriber3_.shutdown();
}

void SimulateSV::initializeSimulation(){
  // Process parameters
  nh_.param<std::string>("/simulate_environment/world_frame", g_worldFrame_, "odom");
  nh_.param<std::string>("/simulate_environment/robot_frame", g_robotFrame_,
                                   "base_footprint");
  nh_.param<double>("/simulate_environment/simulation_factor", g_simulationFactor_,
                              1.0);  // set to e.g. 2.0 for 2x speed
  nh_.param<double>("/simulate_environment/update_rate", g_updateRate_, 25.0);  // in Hz
  privatenh_.param<int>("all_vehicle_number", all_vehicle_number_, 6);
  privatenh_.param<int>("dynamic_vehicle_number", dynamic_vehicle_number_, 3);
  std::cout<<"** vehicle number: "<<all_vehicle_number_<<", "<<all_vehicle_number_<<std::endl;

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  geometry_msgs::Twist twist;
  twist.linear.x = -2;
  twist.angular.z = -2;
  for (int i=0; i<all_vehicle_number_; i++){
    privatenh_.param<double>("pose_initial_x"+std::to_string(i+1) , initialX, 0.0);
    privatenh_.param<double>("pose_initial_y"+std::to_string(i+1), initialY, 0.0);
    privatenh_.param<double>("pose_initial_theta"+std::to_string(i+1), initialTheta, 0.0);
    tf::Transform temp;
    temp.getOrigin().setX(initialX);
    temp.getOrigin().setY(initialY);
    temp.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));
    g_currentPose_.push_back(temp);
    g_currentTwist_.push_back(twist);
    g_currentCommand_.push_back(twist);
    steering_angle_.push_back(0);
  }
  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster_.reset(new tf::TransformBroadcaster());
  // 初始化 最多3个动态车
  twist_subscriber1_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel1", 3, boost::bind(&SimulateSV::onTwistReceived1, this ,_1));
  twist_subscriber2_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel2", 3, boost::bind(&SimulateSV::onTwistReceived2, this ,_1));
  twist_subscriber3_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel3", 3, boost::bind(&SimulateSV::onTwistReceived3, this ,_1));
  pub_dynamic_sv_positions_ = privatenh_.advertise<vehicle_simulator::SocialVehicles>("dynamic_sv_states", 3);
  pub_static_sv_positions_ = privatenh_.advertise<vehicle_simulator::SocialVehicles>("static_sv_states", 3);
  pub_dynamic_sv_polys_ = privatenh_.advertise<visualization_msgs::MarkerArray>("dynamic_sv_polys", 1000);
  pub_static_sv_polys_ = privatenh_.advertise<visualization_msgs::MarkerArray>("static_sv_polys", 1000);
  pub_dynamic_sv_poses_ = privatenh_.advertise<geometry_msgs::PoseArray>("dynamic_sv_poses", 100);
  pub_static_sv_poses_ = privatenh_.advertise<geometry_msgs::PoseArray>("static_sv_poses", 100);
}


void SimulateSV::publishSVFutures(){
  // The future poses of dynamic social vehicles
  geometry_msgs::PoseArray dynamic_veh_poses;
  dynamic_veh_poses.header.frame_id = g_worldFrame_;
  dynamic_veh_poses.header.stamp = ros::Time::now();
  double time_step = 0.2;
  for (int i=0; i<dynamic_vehicle_number_; i++){
    double x = g_currentPose_[i].getOrigin().getX();
    double y = g_currentPose_[i].getOrigin().getY();
    double theta = tf::getYaw(g_currentPose_[i].getRotation());
    double v = g_currentTwist_[i].linear.x;
    double omega = v/wheelbase*tan(steering_angle_[i]);
    for (int t=0; t<5; t++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = dynamic_veh_poses.header.frame_id;
      pose.header.stamp = dynamic_veh_poses.header.stamp;
      x += v*cos(theta)*time_step;
      y += v*sin(theta)*time_step;
      theta += omega*time_step;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = fabs(v)*t*time_step*0.1;
      auto ori = tf::createQuaternionFromRPY(0, 0, theta);
      pose.pose.orientation.x = ori.getX();
      pose.pose.orientation.y = ori.getY();
      pose.pose.orientation.z = ori.getZ();
      pose.pose.orientation.w = ori.getW();
      dynamic_veh_poses.poses.push_back(pose.pose);
    }
  }
  pub_dynamic_sv_poses_.publish(dynamic_veh_poses);

  // The rectangle-shaped outline of the future poses of dynamic social vehicles
  visualization_msgs::MarkerArray dynamic_veh_markerArray;
  std::vector<std::pair<double,double>> FOOTPRINT ={{-0.04, -0.093}, {-0.04, 0.093}, {0.22, 0.093}, {0.22, -0.093}, {-0.04, -0.093}};
  // std::vector<std::pair<float, float>> FOOTPRINT = {{-0.101, -0.093}, {-0.101, 0.093}, {0.139,0.093}, {0.139, -0.093}, {-0.101, -0.093}}; //accordance with robot_constrians.yaml
  for (int i=0; i<dynamic_veh_poses.poses.size(); i++){
    double center_x = dynamic_veh_poses.poses[i].position.x;
    double center_y = dynamic_veh_poses.poses[i].position.y;
    double center_z = dynamic_veh_poses.poses[i].position.z;
    double theta = tf::getYaw(dynamic_veh_poses.poses[i].orientation);

    visualization_msgs::Marker marker;
    marker.header.frame_id = g_worldFrame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w=1;
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(2.0);

    for (auto p:FOOTPRINT){
      geometry_msgs::Point point;
      point.x = cos(theta)*p.first + -sin(theta)*p.second + center_x;
      point.y = sin(theta)*p.first + cos(theta)*p.second + center_y;
      point.z = center_z;
      marker.points.push_back(point);
    }
    dynamic_veh_markerArray.markers.push_back(marker);
  }
  pub_dynamic_sv_polys_.publish(dynamic_veh_markerArray);


  // The poses of static social vehicles.
  geometry_msgs::PoseArray static_veh_poses;
  static_veh_poses.header.frame_id = g_worldFrame_;
  static_veh_poses.header.stamp = ros::Time::now();
  for (int i=dynamic_vehicle_number_; i<all_vehicle_number_; i++){
    double x = g_currentPose_[i].getOrigin().getX();
    double y = g_currentPose_[i].getOrigin().getY();
    double theta = tf::getYaw(g_currentPose_[i].getRotation());
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = static_veh_poses.header.frame_id;
    pose.header.stamp = static_veh_poses.header.stamp;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    auto ori = tf::createQuaternionFromRPY(0, 0, theta);
    pose.pose.orientation.x = ori.getX();
    pose.pose.orientation.y = ori.getY();
    pose.pose.orientation.z = ori.getZ();
    pose.pose.orientation.w = ori.getW();
    static_veh_poses.poses.push_back(pose.pose);
  }
  pub_static_sv_poses_.publish(static_veh_poses);

  // The rectangle-shaped outline of the poses of static social vehicles
  visualization_msgs::MarkerArray static_veh_markersArray;
  for (int i=0; i<static_veh_poses.poses.size(); i++){
    double center_x = static_veh_poses.poses[i].position.x;
    double center_y = static_veh_poses.poses[i].position.y;
    double theta = tf::getYaw(static_veh_poses.poses[i].orientation);

    visualization_msgs::Marker marker;
    marker.header.frame_id = g_worldFrame_;
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w=1;
    marker.id = i;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.01;
    
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(2.0);

    for (auto p:FOOTPRINT){
      geometry_msgs::Point point;
      point.x = cos(theta)*p.first + -sin(theta)*p.second + center_x;
      point.y = sin(theta)*p.first + cos(theta)*p.second + center_y;
      point.z = 0;
      marker.points.push_back(point);
    }
    static_veh_markersArray.markers.push_back(marker);
  }
  pub_static_sv_polys_.publish(static_veh_markersArray);
}


void SimulateSV::publishSVPositions(){
  vehicle_simulator::SocialVehicles dynamic_vehicles;
  dynamic_vehicles.header.frame_id = g_worldFrame_;
  dynamic_vehicles.header.stamp = ros::Time::now();


  
  for (int i=0; i<dynamic_vehicle_number_; i++){
    nav_msgs::Odometry temp;
    temp.header = dynamic_vehicles.header;
    temp.pose.pose.position.x = g_currentPose_[i].getOrigin().getX();
    temp.pose.pose.position.y = g_currentPose_[i].getOrigin().getY();
    temp.pose.pose.orientation.x = g_currentPose_[i].getRotation().getX();
    temp.pose.pose.orientation.y = g_currentPose_[i].getRotation().getY();
    temp.pose.pose.orientation.z = g_currentPose_[i].getRotation().getZ();
    temp.pose.pose.orientation.w = g_currentPose_[i].getRotation().getW();
    temp.twist.twist.linear.x = g_currentTwist_[i].linear.x;
    temp.twist.twist.angular.z = g_currentTwist_[i].angular.z;
    dynamic_vehicles.vehicles.push_back(temp);
  }

  vehicle_simulator::SocialVehicles static_vehicles;
  static_vehicles.header.frame_id = g_worldFrame_;
  static_vehicles.header.stamp = ros::Time::now();
  for (int i=dynamic_vehicle_number_; i<all_vehicle_number_; i++){
    nav_msgs::Odometry temp;
    temp.header = static_vehicles.header;
    temp.pose.pose.position.x = g_currentPose_[i].getOrigin().getX();
    temp.pose.pose.position.y = g_currentPose_[i].getOrigin().getY();
    temp.pose.pose.orientation.x = g_currentPose_[i].getRotation().getX();
    temp.pose.pose.orientation.y = g_currentPose_[i].getRotation().getY();
    temp.pose.pose.orientation.z = g_currentPose_[i].getRotation().getZ();
    temp.pose.pose.orientation.w = g_currentPose_[i].getRotation().getW();
    temp.twist.twist.linear.x = g_currentTwist_[i].linear.x;
    temp.twist.twist.angular.z = g_currentTwist_[i].angular.z;
    static_vehicles.vehicles.push_back(temp);
  }
  pub_dynamic_sv_positions_.publish(dynamic_vehicles);
  pub_static_sv_positions_.publish(static_vehicles);
}


void SimulateSV::runSimulation() {
  ros::Rate rate(g_updateRate_);
  const double dt = g_simulationFactor_/ g_updateRate_;
  while (ros::ok()) {
    for (int i=0; i<dynamic_vehicle_number_; i++){
      // 读取当前时刻的state
      double x = g_currentPose_[i].getOrigin().x();
      double y = g_currentPose_[i].getOrigin().y();
      double theta = tf::getYaw(g_currentPose_[i].getRotation());
      double v = g_currentTwist_[i].linear.x;
      double omega = g_currentTwist_[i].angular.z;

      // 从topic获得刹车和方向盘的command
      std::pair<double,double> action;
      {
        if (i==0) boost::mutex::scoped_lock lock(mutex1_);
        else if (i==1) boost::mutex::scoped_lock lock(mutex2_);
        else if (i==2) boost::mutex::scoped_lock lock(mutex3_);
        action.first = g_currentCommand_[i].linear.x;
        action.second = g_currentCommand_[i].angular.z;
      }

      //根据给出的command算 当前v 和 当前steering_angle
      if (action.first==-2 && action.second==-2){
        // x,y,theta不变，steering_angle不变，v和omega归零
        g_currentTwist_[i].linear.x=0;
        g_currentTwist_[i].angular.z=0;
        steering_angle_[i] = 0;
      }
      else{
        v += action.first*LIN_VEL_STEP_SIZE;
        v = std::max(v, -max_vel_x);
        v = std::min(v, max_vel_x);

        double temp_steering_angle = steering_angle_[i] + action.second*ANG_VEL_STEP_SIZE;
        temp_steering_angle = std::max(temp_steering_angle, -max_steering_angle);
        temp_steering_angle = std::min(temp_steering_angle, max_steering_angle);

        omega = v/wheelbase*tan(steering_angle_[i]);

        x += cos(theta) * v * dt;
        y += sin(theta) * v * dt;
        theta += omega * dt;

        // Update pose
        g_currentPose_[i].getOrigin().setX(x);
        g_currentPose_[i].getOrigin().setY(y);
        g_currentPose_[i].setRotation(tf::createQuaternionFromRPY(0, 0, theta));
        g_currentTwist_[i].linear.x = v;
        g_currentTwist_[i].angular.z = omega;
        steering_angle_[i] = temp_steering_angle;
      }

      // Broadcast transform
      g_transformBroadcaster_->sendTransform(tf::StampedTransform(
      g_currentPose_[i], ros::Time::now(), g_worldFrame_, "robot"+std::to_string(i+1)+"/"+g_robotFrame_));
    }

    for (int i=dynamic_vehicle_number_; i<all_vehicle_number_; i++){
      // Broadcast transform
      g_transformBroadcaster_->sendTransform(tf::StampedTransform(
      g_currentPose_[i], ros::Time::now(), g_worldFrame_, "robot"+std::to_string(i+1)+"/"+g_robotFrame_));
    }

    publishSVPositions();
    publishSVFutures();
    ros::spinOnce();
    rate.sleep();
  }
}

void SimulateSV::onTwistReceived1(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex1_);
  g_currentCommand_[0] = *twist;
}
void SimulateSV::onTwistReceived2(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex2_);
  g_currentCommand_[1] = *twist;
}
void SimulateSV::onTwistReceived3(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex3_);
  g_currentCommand_[2] = *twist;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_social_car");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");
  SimulateSV sm(nodeHandle, privateHandle);
  sm.initializeSimulation();
  // Run
  sm.runSimulation();
}
