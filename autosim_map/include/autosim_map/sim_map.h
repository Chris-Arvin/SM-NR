/**
 *  \author Arvin <1120210190@mail.nankai.edu.cn>
*/
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class ObstacleProcess{
  public:
    ObstacleProcess(ros::NodeHandle nh);
    void YamlReader();
    void MatchMap();
    // callbacks
    void ObstacleCallback(const visualization_msgs::Marker::ConstPtr& walls);
    

  private:
    /// publishers
    ros::Publisher pub_map;
    /// subscribers
    ros::Subscriber sub_obstacles;
  
  private:
    /// message to be published
    nav_msgs::OccupancyGrid map;
    ros::NodeHandle nh_;
    std::string frame_id;
    double resolution;
    double person_diameter;
    bool is_map_initialized;
    double xMin, xMax, yMin, yMax;
};