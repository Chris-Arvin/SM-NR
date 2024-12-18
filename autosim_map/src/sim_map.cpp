#include <autosim_map/sim_map.h>

ObstacleProcess::ObstacleProcess(ros::NodeHandle nh){
  nh_ = nh;
  is_map_initialized = false;
  xMin=10000;
  xMax=-10000;
  yMin=10000;
  yMax=-10000;
  pub_map = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  sub_obstacles = nh_.subscribe<visualization_msgs::Marker>("/simulate_environment/walls", 1, boost::bind(&ObstacleProcess::ObstacleCallback, this ,_1));
}

void ObstacleProcess::YamlReader(){
  nh_.param<std::string>("simulate_environment/frame_id", frame_id, "odom");
  nh_.param<double>("simulate_environment/resolution", resolution, 0.1);
}

void ObstacleProcess::MatchMap(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
}


void ObstacleProcess::ObstacleCallback(const visualization_msgs::Marker::ConstPtr& walls) {
  if (!is_map_initialized){
    // find the origin and size of the map
    double px,py;  
    for (auto p:walls->points){
      px = p.x;
      py = p.y;
      if (px < xMin) xMin = px;
      if (px > xMax) xMax = px;
      if (py < yMin) yMin = py;
      if (py > yMax) yMax = py;
    }
    xMin -= resolution;
    xMax += resolution;
    yMin -= resolution;
    yMax += resolution;

    int width = floor((xMax-xMin)/resolution);
    int height = floor((yMax-yMin)/resolution);

    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.map_load_time = ros::Time::now();

    map.info.origin.position.x = xMin;
    map.info.origin.position.y = yMin;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 1;

    map.header.frame_id = frame_id;
    map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
              map.info.width,
              map.info.height,
              map.info.resolution);

    map.data.resize(map.info.width * map.info.height);
    for (unsigned int i=0; i<map.info.width * map.info.height; i++){
      map.data[i] = 1;
    }

    // project the obstacles into the map
    for (int x=0; x<width; x++){
      if (MAP_IDX(map.info.width,x,0) < map.data.size())
        map.data[MAP_IDX(map.info.width,x,0)] = 100;
      if (MAP_IDX(map.info.width,x,height-1) < map.data.size())
        map.data[MAP_IDX(map.info.width,x,height-1)] = 100;

    }
    is_map_initialized = true;
    pub_map.publish(map);
  }
  else{
    map.info.map_load_time = ros::Time::now();
    map.header.stamp = ros::Time::now();
    pub_map.publish(map);
  }
}