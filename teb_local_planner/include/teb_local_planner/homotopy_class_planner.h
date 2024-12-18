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

#ifndef HOMOTOPY_CLASS_PLANNER_H_
#define HOMOTOPY_CLASS_PLANNER_H_

#include <math.h>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>
#include <random>

#include <boost/shared_ptr.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <ros/console.h>
#include <ros/ros.h>

#include <teb_local_planner/planner_interface.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/equivalence_relations.h>
#include <teb_local_planner/totg_info_processor.h>

namespace teb_local_planner
{

//!< Inline function used for calculateHSignature() in combination with VertexPose pointers
inline std::complex<long double> getCplxFromVertexPosePtr(const VertexPose* pose)
{
  return std::complex<long double>(pose->x(), pose->y());
};


//!< Inline function used for calculateHSignature() in combination with geometry_msgs::PoseStamped
inline std::complex<long double> getCplxFromMsgPoseStamped(const geometry_msgs::PoseStamped& pose)
{
  return std::complex<long double>(pose.pose.position.x, pose.pose.position.y);
};

/**
 * @class HomotopyClassPlanner
 * @brief Local planner that explores alternative homotopy classes, create a plan for each alternative
 *	  and finally return the robot controls for the current best path (repeated in each sampling interval)
 *
 * Equivalence classes (e.g. homotopy) are explored using the help of a search-graph. \n
 * A couple of possible candidates are sampled / generated and filtered afterwards such that only a single candidate
 * per homotopy class remain. Filtering is applied using the H-Signature, a homotopy (resp. homology) invariant: \n
 *      - S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
 *      - C. Rösmann et al.: Planning of Multiple Robot Trajectories in Distinctive Topologies, ECMR, 2015.
 *
 * Followed by the homotopy class search, each candidate is used as an initialization for the underlying trajectory
 * optimization (in this case utilizing the TebOptimalPlanner class with the TimedElasticBand). \n
 * Depending on the config parameters, the optimization is performed in parallel. \n
 * After the optimization is completed, the best optimized candidate is selected w.r.t. to trajectory cost, since the
 * cost already contains important features like clearance from obstacles and transition time. \n
 *
 * Everyhting is performed by calling one of the overloaded plan() methods. \n
 * Afterwards the velocity command to control the robot is obtained from the "best" candidate
 * via getVelocityCommand(). \n
 *
 * All steps are repeated in the subsequent sampling interval with the exception, that already planned (optimized) trajectories
 * are preferred against new path initilizations in order to improve the hot-starting capability.
 */
class HomotopyClassPlanner : public PlannerInterface
{
public:

  using EquivalenceClassContainer = std::vector< std::pair<EquivalenceClassPtr, bool> >;

  /**
   * @brief Default constructor
   */
  HomotopyClassPlanner();

  /**
   * @brief Construct and initialize the HomotopyClassPlanner
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  HomotopyClassPlanner(const TebConfig& cfg, TotgProcessor* totg_processor, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                       TebVisualizationPtr visualization = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);

  /**
   * @brief Destruct the HomotopyClassPlanner.
   */
  virtual ~HomotopyClassPlanner();

  /**
   * @brief Initialize the HomotopyClassPlanner
   * @param cfg Const reference to the TebConfig class for internal parameters
   * @param obstacles Container storing all relevant obstacles (see Obstacle)
   * @param robot_model Shared pointer to the robot shape model used for optimization (optional)
   * @param visualization Shared pointer to the TebVisualization class (optional)
   * @param via_points Container storing via-points (optional)
   */
  void initialize(const TebConfig& cfg, TotgProcessor* totg_processor=NULL, ObstContainer* obstacles = NULL, RobotFootprintModelPtr robot_model = boost::make_shared<PointRobotFootprint>(),
                  TebVisualizationPtr visualization = TebVisualizationPtr(), const ViaPointContainer* via_points = NULL);

  void updateRobotModel(RobotFootprintModelPtr robot_model );

  /** @name Plan a trajectory */
  //@{

  /**
   * @brief Plan a trajectory based on an initial reference plan.
   *
   * Provide this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses).
   * @warning The current implementation extracts only the start and goal pose and calls the overloaded plan()
   * @param initial_plan vector of geometry_msgs::PoseStamped (must be valid until clearPlanner() is called!)
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x, linear.y (holonomic) and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *		      otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
  virtual bool plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, std::string type, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  virtual bool plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);
  virtual bool plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel = NULL, bool free_goal_vel=false);

  /**
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @param[out] vx translational velocity [m/s]
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots [m/s]
   * @param[out] omega rotational velocity [rad/s]
   * @param[in] look_ahead_poses index of the final pose used to compute the velocity command.
   * @return \c true if command is valid, \c false otherwise
   */
  virtual bool getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const;

  /**
   * @brief Check whether the planned trajectory is feasible or not.
   *
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint_spec The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with
   *         any obstacle in the costmap, \c false otherwise.
   */
  virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                    double inscribed_radius = 0.0, double circumscribed_radius=0.0, int look_ahead_idx=-1);

  //@}

  /** @name Visualization */
  //@{

  /**
   * @brief Register a TebVisualization class to enable visiualization routines (e.g. publish the local plan and pose sequence)
   * @param visualization shared pointer to a TebVisualization instance
   * @see visualizeTeb
   */
  void setVisualization(TebVisualizationPtr visualization);

   /**
    * @brief Publish the local plan, pose sequence and additional information via ros topics (e.g. subscribe with rviz).
    *
    * Make sure to register a TebVisualization instance before using setVisualization() or an overlaoded constructor.
    * @see setVisualization
    */
  virtual void visualize();


   /**
    * @brief Reset the planner.
    *
    * Clear all previously found H-signatures, paths, tebs and the hcgraph.
    */
  virtual void clearPlanner() {}


  /**
   * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
   *
   * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two
   * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
   * Initial means that the penalty is applied only to the first few poses of the trajectory.
   * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
   */
  virtual void setPreferredTurningDir(RotType dir);


  bool hasDiverged() const override;

  /**
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param viapoint_cost_scale Specify extra scaling for via points.
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
  virtual void computeCurrentCost(std::vector<double>& cost, double obst_cost_scale=1.0, double viapoint_cost_scale=1.0, bool alternative_time_cost=false);

  /**
   * @brief Access config (read-only)
   * @return const pointer to the config instance
   */
  const TebConfig* config() const {return cfg_;}

  /**
   * @brief Access current obstacle container (read-only)
   * @return const pointer to the obstacle container instance
   */
  const ObstContainer* obstacles() const {return obstacles_;}

  /**
   * @brief Returns true if the planner is initialized
   */
  bool isInitialized() const {return initialized_;}


protected:


  // external objects (store weak pointers)
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  ObstContainer* obstacles_; //!< Store obstacles that are relevant for planning
  TotgProcessor* totg_processor_;
  const ViaPointContainer* via_points_; //!< Store the current list of via-points

  // internal objects (memory management owned)
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  RobotFootprintModelPtr robot_model_; //!< Robot model shared instance

  const std::vector<geometry_msgs::PoseStamped>* initial_plan_; //!< Store the initial plan if available for a better trajectory initialization
  bool initialized_; //!< Keeps track about the correct initialization of this class

  TebOptimalPlanner path_;
  std::string type_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};

// //! Abbrev. for a shared pointer of a HomotopyClassPlanner instance.
// typedef boost::shared_ptr<HomotopyClassPlanner> HomotopyClassPlannerPtr;


} // namespace teb_local_planner

// include template implementations / definitions
// #include <teb_local_planner/homotopy_class_planner.hpp>

#endif /* HOMOTOPY_CLASS_PLANNER_H_ */
