#include <vehicle_simulator/simulate_automated_car.h>

SimulateAV::SimulateAV(const ros::NodeHandle& publicHandle, const ros::NodeHandle& privateHandle): nh_(publicHandle), privatenh_(privateHandle){}

SimulateAV::~SimulateAV() {
  // shutdown  and publishers
  pub_robot_position_.shutdown();
  twist_subscriber_.shutdown();
}
void SimulateAV::initializeSimulation(){
  // Process parameters
  nh_.param<std::string>("simulate_environment/world_frame", g_worldFrame_, "odom");
  nh_.param<std::string>("simulate_environment/robot_frame", g_robotFrame_,
                                   "base_footprint");
  nh_.param<double>("/simulate_environment/simulation_factor", g_simulationFactor_,
                              1.0);  // set to e.g. 2.0 for 2x speed
  nh_.param<double>("/simulate_environment/update_rate", g_updateRate_, 25.0);  // in Hz
  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  privatenh_.param<double>("pose_initial_x", initialX, 0.0);
  privatenh_.param<double>("pose_initial_y", initialY, 0.0);
  privatenh_.param<double>("pose_initial_theta", initialTheta, 0.0);

  g_currentPose_.getOrigin().setX(initialX);
  g_currentPose_.getOrigin().setY(initialY);
  g_currentPose_.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));
  g_currentTwist_.linear.x = 0;
  g_currentTwist_.angular.z = 0;

  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster_.reset(new tf::TransformBroadcaster());
  twist_subscriber_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 3, boost::bind(&SimulateAV::onTwistReceived, this ,_1));
  pub_robot_position_ = nh_.advertise<nav_msgs::Odometry>("/odom", 3);
}

/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
// 描述一个车的状态： (近似认为后轮无偏转，即 后轮速度等于车的速度、后轮朝向等于车身朝向)
// pose信息：车身的(x,y,theta)
// velocity信息：车身的的速度v和角速度omega。
void SimulateAV::publishRobotPosition() {
  nav_msgs::Odometry robot_location;
  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = g_robotFrame_;
  robot_location.header = msg_header;
  robot_location.pose.pose.position.x = g_currentPose_.getOrigin().getX();
  robot_location.pose.pose.position.y = g_currentPose_.getOrigin().getY();
  robot_location.pose.pose.orientation.x = g_currentPose_.getRotation().getX();
  robot_location.pose.pose.orientation.y = g_currentPose_.getRotation().getY();
  robot_location.pose.pose.orientation.z = g_currentPose_.getRotation().getZ();
  robot_location.pose.pose.orientation.w = g_currentPose_.getRotation().getW();
  robot_location.twist.twist.linear.x = g_currentTwist_.linear.x;
  robot_location.twist.twist.angular.z = g_currentTwist_.angular.z;
  pub_robot_position_.publish(robot_location);
}

void SimulateAV::runSimulation() {
  ros::Rate rate(g_updateRate_);
  const double dt = g_simulationFactor_ / g_updateRate_;
  while (ros::ok()) {
    // 读取当前时刻的state
    double x = g_currentPose_.getOrigin().x();
    double y = g_currentPose_.getOrigin().y();
    double theta = tf::getYaw(g_currentPose_.getRotation());
    double v = g_currentTwist_.linear.x;
    double omega = g_currentTwist_.angular.z;

    // 从topic获得刹车和方向盘的command
    {
      boost::mutex::scoped_lock lock(mutex_);
      v = g_currentTwist_.linear.x;
      omega = g_currentTwist_.angular.z;
    }


    v = std::max(v, -max_vel_x);
    v = std::min(v, max_vel_x);

    omega = std::max(omega, -max_vel_theta);
    omega = std::min(omega, max_vel_theta);

    x += cos(theta) * v * dt;
    y += sin(theta) * v * dt;
    theta += omega * dt;

    // Update pose
    g_currentPose_.getOrigin().setX(x);
    g_currentPose_.getOrigin().setY(y);
    g_currentPose_.setRotation(tf::createQuaternionFromRPY(0, 0, theta));
    g_currentTwist_.linear.x = v;
    g_currentTwist_.angular.z = omega;


    // Broadcast transform
    g_transformBroadcaster_->sendTransform(tf::StampedTransform(g_currentPose_, ros::Time::now(), g_worldFrame_, g_robotFrame_));

    publishRobotPosition();
    ros::spinOnce();
    rate.sleep();
  }
}

void SimulateAV::onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex_);
  g_currentTwist_ = *twist;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_automated_car");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");
  SimulateAV sm(nodeHandle, privateHandle);
  sm.initializeSimulation();
  // Run
  sm.runSimulation();
}
