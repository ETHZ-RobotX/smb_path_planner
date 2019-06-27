/*
 * ompl_voxblox.h
 * @brief Header for useful ompl operations with voxblox
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: Jan 23, 2019
 */

#pragma once

#include <ompl/base/StateValidityChecker.h>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>

#include "smb_global_planner/ompl/ompl_types.h"

namespace ompl {

namespace smb {

/**
 * @brief This class implements a validity checker for OMPL using Voxblox. It
 *        is possible to decide the type of voxel (ESDF, TSDF)
 */
template <typename VoxelType>
class VoxbloxValidityChecker : public base::StateValidityChecker {
public:
  VoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                         double robot_radius, double planning_height,
                         voxblox::Layer<VoxelType> *layer)
      : base::StateValidityChecker(space_info), layer_(layer),
        robot_radius_(robot_radius), planning_height_(planning_height) {
    CHECK_NOTNULL(layer);
    voxel_size_ = layer->voxel_size();
  }

  virtual bool isValid(const base::State *state) const {
    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    Eigen::Vector3d robot_position = omplRToEigen(state);
    robot_position(2) = planning_height_;
    bool collision = checkCollisionWithRobot(robot_position);
    // We check the VALIDITY of the state, and the function above returns
    // whether the state was in COLLISION.
    return !collision;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const = 0;

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const {
    return checkCollisionWithRobot(global_index.cast<double>() * voxel_size_);
  }

  float voxel_size() const { return voxel_size_; }

protected:
  voxblox::Layer<VoxelType> *layer_;

  float voxel_size_;
  double robot_radius_;
  double planning_height_;
};

/**
 * @brief Class that implements a validity checker for OMPL with TSDF maps
 */
class TsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::TsdfVoxel> {
public:
  TsdfVoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                             double robot_radius, double planning_height,
                             voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, planning_height,
                               tsdf_layer),
        treat_unknown_as_occupied_(false) {}

  bool getTreatUnknownAsOccupied() const { return treat_unknown_as_occupied_; }
  void setTreatUnknownAsOccupied(bool treat_unknown_as_occupied) {
    treat_unknown_as_occupied_ = treat_unknown_as_occupied;
  }

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();
    voxblox::HierarchicalIndexMap block_voxel_list;

    voxblox::utils::getSphereAroundPoint(*layer_, robot_point, robot_radius_,
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

protected:
  bool treat_unknown_as_occupied_;
};

/**
 * @brief Class that implements a validity checker for OMPL with ESDF maps
 */
class EsdfVoxbloxValidityChecker
    : public VoxbloxValidityChecker<voxblox::EsdfVoxel> {
public:
  EsdfVoxbloxValidityChecker(const base::SpaceInformationPtr &space_info,
                             double robot_radius, double planning_height,
                             voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer)
      : VoxbloxValidityChecker(space_info, robot_radius, planning_height,
                               esdf_layer),
        interpolator_(esdf_layer) {}

  virtual bool
  checkCollisionWithRobot(const Eigen::Vector3d &robot_position) const {
    voxblox::Point robot_point = robot_position.cast<voxblox::FloatingPoint>();

    constexpr bool interpolate = false;
    voxblox::FloatingPoint distance;
    bool success = interpolator_.getDistance(
        robot_position.cast<voxblox::FloatingPoint>(), &distance, interpolate);
    if (!success) {
      return true;
    }
    return robot_radius_ >= distance;
  }

  virtual bool checkCollisionWithRobotAtVoxel(
      const voxblox::GlobalIndex &global_index) const {
    voxblox::EsdfVoxel *voxel = layer_->getVoxelPtrByGlobalIndex(global_index);
    if (voxel == nullptr) {
      return true;
    }
    return robot_radius_ >= voxel->distance;
  }

protected:
  // Interpolator for the layer.
  voxblox::Interpolator<voxblox::EsdfVoxel> interpolator_;
};

/**
 * @brief Motion validator that uses either of the validity checkers above to
 *        validate motions at voxel resolution.
 */
template <typename VoxelType>
class VoxbloxMotionValidator : public base::MotionValidator {
public:
  VoxbloxMotionValidator(
      const base::SpaceInformationPtr &space_info,
      typename std::shared_ptr<VoxbloxValidityChecker<VoxelType>>
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
      bool collision =
          validity_checker_->checkCollisionWithRobotAtVoxel(global_index);

      if (collision) {
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
      }
    }
    return true;
  }

protected:
  typename std::shared_ptr<VoxbloxValidityChecker<VoxelType>> validity_checker_;
  double planning_height_;
};

} // end namespace smb
} // end namespace ompl
