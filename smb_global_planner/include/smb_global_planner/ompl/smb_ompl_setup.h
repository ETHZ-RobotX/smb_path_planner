/*
 * smb_ompl_setup.h
 * @brief Header for useful ompl operations with voxblox for the robot
 * @author: Helen Oleynikova, ASL
 *          Modified by Luca Bartolomei, V4RL
 * Modified on: Jan 23, 2019
 */

#pragma once

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "smb_global_planner/ompl/ompl_traversability_voxblox.h"
#include "smb_global_planner/ompl/ompl_voxblox.h"

namespace ompl {
namespace smb {

// Setup class for a geometric planning problem with Real state space (R).
class SmbSetupR : public geometric::SimpleSetup {
public:
  SmbSetupR()
      : geometric::SimpleSetup(base::StateSpacePtr(new RStateSpace(2))) {}

  // Get some defaults.
  void setDefaultObjective() {
    getProblemDefinition()->setOptimizationObjective(
        ompl::base::OptimizationObjectivePtr(
            new ompl::base::PathLengthOptimizationObjective(
                getSpaceInformation())));
  }

  void setDefaultPlanner() { setRrtStar(); }

  void setRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTstar(getSpaceInformation())));
  }

  void setRrtConnect() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::RRTConnect(getSpaceInformation())));
  }

  void setInformedRrtStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::InformedRRTstar(getSpaceInformation())));
  }

  void setBitStar() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::BITstar(getSpaceInformation())));
  }

  void setPrm() {
    setPlanner(ompl::base::PlannerPtr(
        new ompl::geometric::PRM(getSpaceInformation())));
  }

  const base::StateSpacePtr &getGeometricComponentStateSpace() const {
    return getStateSpace();
  }

  void setStateValidityCheckingResolution(double resolution) {
    // This is a protected attribute, so need to wrap this function.
    si_->setStateValidityCheckingResolution(resolution);
  }

  void setLenghtOptimizationObjective(const double distance) {
    ompl::base::OptimizationObjectivePtr obj(
        new ompl::base::PathLengthOptimizationObjective(getSpaceInformation()));
    obj->setCostThreshold(ompl::base::Cost(distance));
    getProblemDefinition()->setOptimizationObjective(obj);
  }

  void setTsdfVoxbloxCollisionChecking(
      double robot_radius, double planning_height,
      voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer) {
    std::shared_ptr<TsdfVoxbloxValidityChecker> validity_checker(
        new TsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       planning_height, tsdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::TsdfVoxel>(
            getSpaceInformation(), validity_checker, planning_height)));
  }

  void setEsdfVoxbloxCollisionChecking(
      double robot_radius, double planning_height,
      voxblox::Layer<voxblox::EsdfVoxel> *esdf_layer) {
    std::shared_ptr<EsdfVoxbloxValidityChecker> validity_checker(
        new EsdfVoxbloxValidityChecker(getSpaceInformation(), robot_radius,
                                       planning_height, esdf_layer));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(
        base::MotionValidatorPtr(new VoxbloxMotionValidator<voxblox::EsdfVoxel>(
            getSpaceInformation(), validity_checker, planning_height)));
  }

  void setTsdfTraversabilityVoxbloxCollisionChecking(
      double robot_radius, double planning_height,
      double traversability_threshold, double maximum_difference_elevation,
      voxblox::Layer<voxblox::TsdfVoxel> *tsdf_layer,
      const grid_map::GridMap &traversability_map) {
    std::shared_ptr<TsdfTraversabilityVoxbloxValidityChecker> validity_checker(
        new TsdfTraversabilityVoxbloxValidityChecker(
            getSpaceInformation(), robot_radius, planning_height,
            traversability_threshold, maximum_difference_elevation, tsdf_layer,
            traversability_map));

    setStateValidityChecker(base::StateValidityCheckerPtr(validity_checker));
    si_->setMotionValidator(base::MotionValidatorPtr(
        new TraversabilityVoxbloxMotionValidator<voxblox::TsdfVoxel>(
            getSpaceInformation(), validity_checker, planning_height)));
  }

  void constructPrmRoadmap(double num_seconds_to_construct) {
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(num_seconds_to_construct);

    std::dynamic_pointer_cast<ompl::geometric::PRM>(getPlanner())
        ->constructRoadmap(ptc);
  }

  // Uses the path simplifier WITHOUT using B-spline smoothing which leads to
  // a lot of issues for us.
  void reduceVertices() {
    if (pdef_) {
      const base::PathPtr &p = pdef_->getSolutionPath();
      if (p) {
        time::point start = time::now();
        geometric::PathGeometric &path =
            static_cast<geometric::PathGeometric &>(*p);
        std::size_t num_states = path.getStateCount();

        reduceVerticesOfPath(path);
        // simplifyTime_ member of the parent class.
        simplifyTime_ = time::seconds(time::now() - start);
        OMPL_INFORM(
            "SmbSetupR: Vertex reduction took %f seconds and changed from %d to"
            " %d states",
            simplifyTime_, num_states, path.getStateCount());
        return;
      }
    }
    OMPL_WARN("No solution to simplify");
  }

  // Simplification of path without B-splines.
  void reduceVerticesOfPath(geometric::PathGeometric &path) {
    const double max_time = 0.1;
    base::PlannerTerminationCondition ptc =
        base::timedPlannerTerminationCondition(max_time);

    // Now just call near-vertex collapsing and reduceVertices.
    if (path.getStateCount() < 3) {
      return;
    }

    // try a randomized step of connecting vertices
    bool try_more = false;
    if (ptc == false) {
      try_more = psk_->reduceVertices(path);
    }

    // try to collapse close-by vertices
    if (ptc == false) {
      psk_->collapseCloseVertices(path);
    }

    // try to reduce verices some more, if there is any point in doing so
    int times = 0;
    while (try_more && ptc == false && ++times <= 5) {
      try_more = psk_->reduceVertices(path);
    }
  }
};

} // end namespace smb
} // end namespace ompl
