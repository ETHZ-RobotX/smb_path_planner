/*
 * ompl_voxblox.h
 * @brief Header for implementation for ompl rrt using voxblox
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: Jan 23, 2019
 */

#pragma once

#include <ros/ros.h>
#include <smb_planner_common/planner_parameters.h>

#include "smb_global_planner/ompl/smb_ompl_setup.h"

namespace smb_global_planner {

class VoxbloxOmplRrt {
public:
  enum RrtPlannerType {
    kRrtConnect = 0,
    kRrtStar,
    kInformedRrtStar,
    kBitStar,
    kPrm
  };

  VoxbloxOmplRrt(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 const PlannerParameters &params);
  virtual ~VoxbloxOmplRrt() {}

  // Set / Get methods
  inline void setRobotRadius(double robot_radius) {
    params_.robot_radius = robot_radius;
  }

  void setBounds(const Eigen::Vector3d &lower_bound,
                 const Eigen::Vector3d &upper_bound);

  // Both are expected to be OWNED BY ANOTHER OBJECT that shouldn't go out of
  // scope while this object exists.
  void setTsdfLayer(voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer);
  void setEsdfLayer(voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer);

  inline void setOptimistic(bool optimistic) { optimistic_ = optimistic; }
  bool getOptimistic() const { return optimistic_; }

  inline void setTraversabilityCheck(bool check_traversability) {
    check_traversability_ = check_traversability;
  }

  bool getTraversabilityCheck() const { return check_traversability_; }

  double getNumSecondsToPlan() const {
    return params_.global_params.num_seconds_to_plan;
  }

  void setNumSecondsToPlan(double num_seconds) {
    params_.global_params.num_seconds_to_plan = num_seconds;
  }

  RrtPlannerType getPlanner() const { return planner_type_; }
  void setPlanner(RrtPlannerType planner) { planner_type_ = planner; }

  // Only call this once, only call this after setting all settings correctly.
  void setupProblem(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
  void setupTraversabilityProblem(const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &goal,
                                  const grid_map::GridMap &traversability_map);

  // Check for straight line planning
  bool validStraightLine(
          const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
          const int n_step, std::vector<Eigen::Vector3d> &path) const;

  bool validTraversableStraightLine(
          const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
          const int n_step, std::vector<Eigen::Vector3d> &path,
          const grid_map::GridMap &traversability_map) const;

  // Fixed start and end locations, returns list of waypoints between.
  bool getPathBetweenWaypoints(const Eigen::Vector3d &start,
                               const Eigen::Vector3d &goal,
                               std::vector<Eigen::Vector3d> &solution);

  void solutionPathToTrajectoryPoints(
      ompl::geometric::PathGeometric &path,
      std::vector<Eigen::Vector3d> &trajectory_points) const;

  // Even if planning fails, get the part of the tree that spans closest to
  // the original goal point. Returns true if it was actually successfully
  // able to plan to the original goal point, false otherwise.
  bool getBestPathTowardGoal(const Eigen::Vector3d &start,
                             const Eigen::Vector3d &goal,
                             std::vector<Eigen::Vector3d> &solution);

  void constructPrmRoadmap(double roadmap_construction_sec) {
    problem_setup_.setup();
    problem_setup_.constructPrmRoadmap(roadmap_construction_sec);
  }

protected:
  void setupFromStartAndGoal(const Eigen::Vector3d &start,
                             const Eigen::Vector3d &goal);

  double getDistanceEigenToState(const Eigen::Vector3d &eigen,
                                 const ompl::base::State *state_ptr);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Setup the problem in OMPL.
  ompl::smb::SmbSetupR problem_setup_;
  RrtPlannerType planner_type_;

  // Whether the planner is optimistic (true) or pessimistic (false) about
  // how unknown space is handled.
  // Optimistic uses the TSDF for collision checking, while pessimistic uses
  // the ESDF. Be sure to set the maps accordingly.
  bool optimistic_;
  bool check_traversability_;

  // Parameters
  PlannerParameters params_;

  // Planning bounds, if set.
  Eigen::Vector3d lower_bound_;
  Eigen::Vector3d upper_bound_;

  // NON-OWNED pointers to the relevant layers. TSDF only used if optimistic,
  // ESDF only used if pessimistic.
  voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer_;
  voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer_;

  double voxel_size_;
};

} // namespace smb_global_planner
