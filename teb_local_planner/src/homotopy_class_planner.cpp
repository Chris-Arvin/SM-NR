/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/homotopy_class_planner.h>

#include <limits>

namespace teb_local_planner
{

HomotopyClassPlanner::HomotopyClassPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), robot_model_(new PointRobotFootprint()), initial_plan_(NULL), initialized_(false)
{
}

HomotopyClassPlanner::HomotopyClassPlanner(const TebConfig& cfg, TotgProcessor* totg_processor, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                           TebVisualizationPtr visual, const ViaPointContainer* via_points) : initial_plan_(NULL)
{
  initialize(cfg, totg_processor, obstacles, robot_model, visual, via_points);
}

HomotopyClassPlanner::~HomotopyClassPlanner()
{
}

void HomotopyClassPlanner::initialize(const TebConfig& cfg, TotgProcessor* totg_processor, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                      TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  cfg_ = &cfg;
  totg_processor_ = totg_processor;
  obstacles_ = obstacles;
  via_points_ = via_points;
  robot_model_ = robot_model;

  initialized_ = true;

  setVisualization(visual);
}

// 核心函数： optimize the path
bool HomotopyClassPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, std::string type, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  type_ = type;
  // if (type_=="stop")
  //   return true;

  path_ = TebOptimalPlanner(*cfg_, totg_processor_ ,obstacles_, robot_model_, visualization_, via_points_, type);
  path_.teb().initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  if (start_vel) path_.setVelocityStart(*start_vel);
  if (free_goal_vel) path_.setVelocityGoalFree();
  path_.optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true, cfg_->hcp.selection_obst_cost_scale, cfg_->hcp.selection_viapoint_cost_scale, cfg_->hcp.selection_alternative_time_cost); // compute cost as well inside optimizeTEB (last argument = true)
  return true;
}

void HomotopyClassPlanner::updateRobotModel(RobotFootprintModelPtr robot_model )
{
  robot_model_ = robot_model;
}

void HomotopyClassPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

bool HomotopyClassPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel){return false;}
bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel){return false;}


bool HomotopyClassPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{  
  if (path_.teb().poses().empty())
  {
    vx = 0; vy = 0; omega = 0;
    return false;
  }
  if (type_ == "stop"){
    vx = 0; vy = 0; omega = 0;
    return true;
  }
  path_.getVelocityCommand(vx, vy, omega, look_ahead_poses);
  double dis2up_wall = 999, dis2down_wall = 999;
  for (const ObstaclePtr& obst : *obstacles_){
    geometry_msgs::Polygon poly;
    obst->toPolygonMsg(poly);
    if(poly.points.size()==2 && poly.points.front().y>0)
      dis2up_wall = robot_model_->calculateDistance(path_.teb().Pose(0), obst.get());
    if(poly.points.size()==2 && poly.points.front().y<0)
      dis2down_wall = robot_model_->calculateDistance(path_.teb().Pose(0), obst.get());
  }
  // 离up_wall近 并且y轴上的速度>0
  if (dis2up_wall < 0.02 && vx*sin(path_.teb().Pose(0).theta())>0 ){
    vx *= dis2up_wall;
  }
  // 离down_wall近 并且y轴上的速度<0
  else if (dis2down_wall < 0.02 && vx*sin(path_.teb().Pose(0).theta())<0 ){
    vx *= dis2down_wall;
  }
  // std::cout<<"============= "<<path_.teb().Pose(0).theta()<<": "<<dis2up_wall<<", "<<dis2down_wall<<", "<<vx<<std::endl;

  return true;
}




void HomotopyClassPlanner::visualize()
{
  if (visualization_)
  {
    // Visualize best teb and feedback message if desired
    visualization_->publishLocalPlanAndPoses(path_.teb());

    if (path_.teb().sizePoses() > 0){ //TODO maybe store current pose (start) within plan method as class field.
      std::vector<PoseSE2> poses;
      for(int i=0; i<path_.teb().sizePoses(); i++)
        poses.push_back(path_.teb().Pose(i));
      visualization_->publishRobotFootprintModel(poses, *robot_model_);
    }

  }
  else ROS_DEBUG("Ignoring HomotopyClassPlanner::visualize() call, since no visualization class was instantiated before.");
}


bool HomotopyClassPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                                double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  return true;
}


void HomotopyClassPlanner::setPreferredTurningDir(RotType dir)
{
  path_.setPreferredTurningDir(dir);
}

bool HomotopyClassPlanner::hasDiverged() const
{
  return path_.hasDiverged();
}

void HomotopyClassPlanner::computeCurrentCost(std::vector<double>& cost, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  path_.computeCurrentCost(cost, obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
}

} // end namespace
