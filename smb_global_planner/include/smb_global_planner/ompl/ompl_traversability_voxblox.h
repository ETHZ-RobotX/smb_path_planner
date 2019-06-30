/**
  * @author Luca Bartolomei, V4RL
  * @brief  Extension of planning approaches based on OMPL and Voxblox to
  *         consider also the Traversability information
  * @date   13.06.2019
  */

#pragma once

#include <math.h>

#include <smb_planner_common/traversability_estimator.h>
#include <smb_planner_common/utility_mapping.h>

#include "smb_global_planner/ompl/ompl_voxblox.h"

using namespace smb_planner;

namespace ompl {

namespace smb {

template <typename VoxelType>
class TraversabilityVoxbloxValidityChecker : public base::StateValidityChecker {
public:
  TraversabilityVoxbloxValidityChecker(
      const base::SpaceInformationPtr &space_info, double robot_radius,
      double planning_height, double traversability_threshold,
      double maximum_difference_elevation, voxblox::Layer<VoxelType> *layer,
      const grid_map::GridMap &traversability_map)
      : base::StateValidityChecker(space_info), layer_(layer),
        traversability_map_(traversability_map), robot_radius_(robot_radius),
        planning_height_(planning_height),
        traversability_threshold_(traversability_threshold),
        maximum_difference_elevation_(maximum_difference_elevation) {
    CHECK_NOTNULL(layer);
    voxel_size_ = layer->voxel_size();
  }

  virtual bool isValid(const base::State *state) const {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    Eigen::Vector3d robot_position = omplRToEigen(state);
    Eigen::Vector3d projected_position;

    // Get traversability information
    TraversabilityStatus traversability_status =
        utility_mapping::getTrasversabilityInformation(
            traversability_map_, robot_position.head<2>(), planning_height_,
            traversability_threshold_, maximum_difference_elevation_,
            projected_position);
    // Check collision
    if (traversability_status == TraversabilityStatus::UNTRAVERSABLE) {
      return false;
    } else if (traversability_status == TraversabilityStatus::TRAVERSABLE) {
      return true;
    } else if (traversability_status == TraversabilityStatus::UNKNOWN) {
      projected_position = robot_position;
      projected_position(2) = planning_height_;
      return !checkCollisionWithRobot(projected_position);
    }

    // In this case the position is traversable. Consider it valid
    return true;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const = 0;

  // Returns whether the position is traversable
  virtual TraversabilityStatus
  checkTraversabilityPosition(const Eigen::Vector2d &position,
                              Eigen::Vector3d &projected_position) const = 0;

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const {
    return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
  }

  virtual bool checkTraversabilityMultiplePositions(
      const std::vector<Eigen::Vector2d> &positions,
      std::vector<Eigen::Vector3d> &projected_positions,
      std::vector<TraversabilityStatus> &traversabilities) const = 0;

  std::vector<Eigen::Vector2d> create2dBoundingBoxAroundPosition(
      const Eigen::Vector3d &robot_position) const {
    std::vector<Eigen::Vector2d> bb;

    // Upper level
    bb.push_back(Eigen::Vector2d(robot_position(0) + robot_radius_,
                                 robot_position(1) + robot_radius_));

    bb.push_back(Eigen::Vector2d(robot_position(0) - robot_radius_,
                                 robot_position(1) + robot_radius_));

    bb.push_back(Eigen::Vector2d(robot_position(0) + robot_radius_,
                                 robot_position(1) - robot_radius_));

    bb.push_back(Eigen::Vector2d(robot_position(0) - robot_radius_,
                                 robot_position(1) - robot_radius_));

    return bb;
  }

  float voxel_size() const { return voxel_size_; }

protected:
  voxblox::Layer<VoxelType> *layer_;
  grid_map::GridMap traversability_map_;

  float voxel_size_;
  double robot_radius_;
  double planning_height_;
  double traversability_threshold_;
  double maximum_difference_elevation_;
};

class TsdfTraversabilityVoxbloxValidityChecker
    : public TraversabilityVoxbloxValidityChecker<voxblox::TsdfVoxel> {
public:
  TsdfTraversabilityVoxbloxValidityChecker(
      const base::SpaceInformationPtr &space_info, double robot_radius,
      double planning_height, double traversability_threshold,
      double maximum_difference_elevation,
      voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer,
      const grid_map::GridMap &traversability_map)
      : TraversabilityVoxbloxValidityChecker(
            space_info, robot_radius, planning_height, traversability_threshold,
            maximum_difference_elevation, tsdf_layer, traversability_map),
        treat_unknown_as_occupied_(false) {}

  bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
  void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
    treat_unknown_as_occupied_ = treat_unknown_as_occupied;
  }
  void setValidTraversability(double traversability_threshold) {
    traversability_threshold_ = traversability_threshold;
  }

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

    voxblox::HierarchicalIndexMap block_voxel_list;
    utility_mapping::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
                                          &block_voxel_list);

    for (const std::pair<voxblox::BlockIndex, voxblox::VoxelIndexList> &kv :
         block_voxel_list) {
      // Get block -- only already existing blocks are in the list.
      voxblox::Block<voxblox::TsdfVoxel>::Ptr block_ptr =
          layer_->getBlockPtrByIndex(kv.first);

      if (!block_ptr) {
        continue;
      }

      for (const voxblox::VoxelIndex &voxel_index : kv.second) {
        if (!block_ptr->isValidVoxelIndex(voxel_index)) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        const voxblox::TsdfVoxel &tsdf_voxel =
            block_ptr->getVoxelByVoxelIndex(voxel_index);
        if (tsdf_voxel.weight < voxblox::kEpsilon) {
          if (treat_unknown_as_occupied_) {
            return true;
          }
          continue;
        }
        if (tsdf_voxel.distance <= 0.0f) {
          return true;
        }
      }
    }

    // No collision if nothing in the sphere had a negative or 0 distance.
    // Unknown space is unoccupied, since this is a very optimistic global
    // planner.
    return false;
  }

  virtual TraversabilityStatus
  checkTraversabilityPosition(const Eigen::Vector2d &position,
                              Eigen::Vector3d &projected_position) const {

    return utility_mapping::getTrasversabilityInformation(
        traversability_map_, position, planning_height_,
        traversability_threshold_, maximum_difference_elevation_,
        projected_position);
  }

  virtual bool checkTraversabilityMultiplePositions(
      const std::vector<Eigen::Vector2d> &positions,
      std::vector<Eigen::Vector3d> &projected_positions,
      std::vector<TraversabilityStatus> &traversabilities) const {

    if (!utility_mapping::checkTraversabilityMultiplePositions(
            traversability_map_, positions, planning_height_,
            traversability_threshold_, maximum_difference_elevation_,
            projected_positions, traversabilities)) {
      return false;
    }

    return true;
  }

protected:
  bool treat_unknown_as_occupied_;
};

template <typename VoxelType>
class TraversabilityVoxbloxMotionValidator : public base::MotionValidator {
public:
  TraversabilityVoxbloxMotionValidator(
      const base::SpaceInformationPtr &space_info,
      typename std::shared_ptr<TraversabilityVoxbloxValidityChecker<VoxelType>>
          validity_checker,
      double planning_height)
      : base::MotionValidator(space_info), validity_checker_(validity_checker),
        planning_height_(planning_height) {
    CHECK(validity_checker);
  }

  virtual bool checkMotion(const base::State *s1, const base::State *s2) const {
    std::pair<base::State *, double> unused;
    return checkMotion(s1, s2, unused);
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So opposite of checkCollision, but same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  virtual bool checkMotion(const base::State *s1, const base::State *s2,
                           std::pair<base::State *, double> &last_valid) const {
    Eigen::Vector3d start = omplRToEigen(s1);
    start(2) = planning_height_;

    Eigen::Vector3d goal = omplRToEigen(s2);
    goal(2) = planning_height_;

    // Project the start and the goal pose
    Eigen::Vector3d start_proj;
    TraversabilityStatus traversabilility_start =
        validity_checker_->checkTraversabilityPosition(start.head<2>(),
                                                       start_proj);
    if (traversabilility_start == TraversabilityStatus::UNTRAVERSABLE) {
      return false;
    } else {
      // Overwrite the starting position
      start = start_proj;
    }

    Eigen::Vector3d goal_proj;
    TraversabilityStatus traversabilility_goal =
        validity_checker_->checkTraversabilityPosition(goal.head<2>(),
                                                       goal_proj);
    if (traversabilility_goal == TraversabilityStatus::UNTRAVERSABLE) {
      return false;
    } else {
      // Overwrite the goal position
      goal = goal_proj;
    }

    double voxel_size = validity_checker_->voxel_size();

    voxblox::Point start_scaled, goal_scaled;
    voxblox::AlignedVector<voxblox::GlobalIndex> indices;

    // Convert the start and goal to global voxel coordinates.
    // Actually very simple -- just divide by voxel size.
    start_scaled = start.cast<voxblox::FloatingPoint>() / voxel_size;
    goal_scaled = goal.cast<voxblox::FloatingPoint>() / voxel_size;
    voxblox::castRay(start_scaled, goal_scaled, &indices);

    for (size_t i = 0; i < indices.size(); i++) {
      const voxblox::GlobalIndex &global_index = indices[i];

      Eigen::Vector3d pos = global_index.cast<double>() * voxel_size;
      Eigen::Vector3d projected_pos;
      TraversabilityStatus traversabilility_status =
          validity_checker_->checkTraversabilityPosition(pos.head<2>(),
                                                         projected_pos);

      if (traversabilility_status == TraversabilityStatus::UNTRAVERSABLE) {
        if (last_valid.first != nullptr) {
          ompl::base::ScopedState<ompl::smb::RStateSpace> last_valid_state(
              si_->getStateSpace());
          last_valid_state->values[0] = pos.x();
          last_valid_state->values[1] = pos.y();
          last_valid_state->values[2] = planning_height_;

          si_->copyState(last_valid.first, last_valid_state.get());
        }

        last_valid.second = static_cast<double>(i / indices.size());
        return false;
      } else if (traversabilility_status == TraversabilityStatus::UNKNOWN) {
        bool collision =
            validity_checker_->checkCollisionWithRobot(projected_pos);
        if (collision) {
          if (last_valid.first != nullptr) {
            ompl::base::ScopedState<ompl::smb::RStateSpace> last_valid_state(
                si_->getStateSpace());
            last_valid_state->values[0] = projected_pos.x();
            last_valid_state->values[1] = projected_pos.y();
            last_valid_state->values[2] = projected_pos.z();

            si_->copyState(last_valid.first, last_valid_state.get());
          }

          last_valid.second = static_cast<double>(i / indices.size());
          return false;
        }
      }
    }
    return true;
  }

protected:
  typename std::shared_ptr<TraversabilityVoxbloxValidityChecker<VoxelType>>
      validity_checker_;
  double planning_height_;
};

} // end namespace smb

} // end namespace ompl
