/**
  * @author Luca Bartolomei, V4RL
  * @brief  Main class that implements the CHOMP Planner. It is based on the
  *         following paper and related implementation:
  *
  *         "Local Path Optimizer for an Autonomous Truck in a Harbor Scenario",
  *          J. David, R. Valencia, R. Philippsen and K. Iagnemma, FSR 2017
  *
  *          Code: https://github.com/rafaelvalencia/path-adaptor
  *
  * @date   14.06.2019
  */

#pragma once

#include <ceres/ceres.h>
#include <smb_planner_common/planner_parameters.h>
#include <smb_planner_common/traversability_estimator.h>
#include <smb_planner_common/utility_mapping.h>
#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/integrator_utils.h>
#include <voxblox/utils/planning_utils.h>

using namespace smb_planner;

namespace smb_local_planner {

class ChompSolver {

public:
  /**
   * @brief Constructor
   * @param[in] esdf_map : ESDF map from voxblox used for planning
   * @param[in] params : parameters useful for planning
   */
  ChompSolver(voxblox::EsdfMap::Ptr esdf_map, const PlannerParameters &params);

  /**
   * @brief Destructor
   */
  ~ChompSolver();

  /**
   * @brief Main method that computes the local path between start and goal
   *        configurations
   */
  bool solve(const Eigen::VectorXd &start, const Eigen::VectorXd &goal);

  /**
   * @brief Main method that computes the local path between start and goal
   *        configurations using traversability information
   * @param start
   * @param goal
   * @param traversability_map
   * @return
   */
  bool solve(const Eigen::VectorXd &start, const Eigen::VectorXd &goal,
             const grid_map::GridMap &traversability_map);

  /**
   * @brief Method to extract the path from the solver
   */
  Eigen::VectorXd getPath() const { return xi_; }

  /**
   * @brief Method to extract the space dimension from the solver
   * @return
   */
  int getSpaceDimension() const { return c_dim_; }

private:
  /**
   * @brief Method that preallocates all the matrices for solver
   */
  void initChompSolver();

  /**
   * @brief Method that initialize the solution as a straight line
   * @param[in] start : start pose of the line connection
   * @param[in] goal : end pose of the line connection
   */
  void initStraightLinePath(const Eigen::VectorXd &start,
                            const Eigen::VectorXd &goal);
  /**
   * @brief Method that initialize the solution by stacking all the poses
   *        in one point
   * @param[in] start : start pose for the stacking
   * @param[in] goal : end pose of for the stacking
   */
  void initStackedPath(const Eigen::VectorXd &start,
                       const Eigen::VectorXd &goal);

  /**
   * @brief Main method to compute the cost and gradient during optimization
   *        by considering the smoothness of the path and the current map
   * @param[in] solution : current solution to evaluate (cost and gradient)
   * @param[out] cost : cost of the current solution
   * @param[out] gradient : gradient of the current solution for the cost funct
   */
  bool computeCostAndGradient(const Eigen::VectorXd &solution, double *cost,
                              double *gradient);

  /**
   * @brief Method that iterates over the yaws of the solution path and
   *        bounds the yaw values between (-pi, pi)
   */
  void boundTrajectoryAngles();

  /**
   * @brief Nested class to interface the solver with CERES
   */
  class NestedCeresFunction : public ceres::FirstOrderFunction {
  public:
    /**
     * @brief Constructor
     * @param[in] N : dimension of the configuration space
     * @param[in] parent : chomp solver to attach to
     */
    NestedCeresFunction(int N, ChompSolver *parent) : N_(N), parent_(parent) {}

    /**
     * @brief Main method to compute the cost and gradient during optimization
     *        by considering the smoothness of the path and the current map
     * @param[in] parameters : current solution to evaluate (cost and gradient)
     * @param[out] cost : cost of the current solution
     * @param[out] gradient : gradient of the current solution for the cost
     */
    virtual bool Evaluate(const double *parameters, double *cost,
                          double *gradient) const;

    /**
     * @brief Function that returns the number of parameters
     * @return the number of parameters
     */
    virtual int NumParameters() const;

  private:
    int N_;
    ChompSolver *parent_;
  };

protected:
  // Map
  voxblox::EsdfMap::Ptr esdf_map_;
  grid_map::GridMap traversability_map_;

  // Trajectory
  Eigen::VectorXd xi_; // the trajectory (q_1, q_2, ...q_n)
  Eigen::VectorXd qs_; // the start config a.k.a. q_0
  Eigen::VectorXd qe_; // the end config a.k.a. q_(n+1)

  // Parameters and auxiliaries
  PlannerParameters params_;
  int xi_dim_; // dimension of trajectory, xidim = nq * cdim

  // Matrices for gradient descent that can be preallocated
  Eigen::MatrixXd AA_;   // metric
  Eigen::VectorXd bb_;   // acceleration bias for start and end config
  Eigen::MatrixXd Ainv_; // inverse of AA

  // Auxiliaries
  bool initialized_;
  bool check_traversability_;
  const int c_dim_;
};

} // end namespace smb_local_planner
