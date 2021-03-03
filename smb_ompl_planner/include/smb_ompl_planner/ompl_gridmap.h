/*
 * Copyright (c) 2020, Vision for Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Vision for Robotics Lab, ETH Zurich nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * ompl_gridmap.h
 * @brief Header file for collision checking using gridmap
 * @author: Luca Bartolomei
 * Created on: March 11, 2020
 */

#pragma once

#include <Eigen/Dense>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{

typedef base::RealVectorStateSpace RStateSpace;

/**
 * @brief Class used to check the collision of the robot with the obstacles in
 *        the grid map
 */
class GridmapValidityChecker : public base::StateValidityChecker
{
public:
  GridmapValidityChecker(const base::SpaceInformationPtr& space_info,
                         const double interpolation_factor,
                         costmap_2d::Costmap2D* costmap)
      : base::StateValidityChecker(space_info),
        interpolation_factor_(interpolation_factor), costmap_(costmap)
  {
  }

  /**
   * @brief Method used by the OMPL planner to check if a sampled state is valid
   * @param[in] state: sample to be checked
   * @return True if the sample is valid, false otherwise
   */
  virtual bool isValid(const base::State* state) const
  {
    if (!si_->satisfiesBounds(state))
    {
      return false;
    }

    // Get the state in cell coordinates
    const RStateSpace::StateType* derived =
        static_cast<const RStateSpace::StateType*>(state);
    Eigen::Vector2d state_eigen(derived->values[0], derived->values[1]);

    // Get the cost
    unsigned char cost;
    if (!getCostAtState(state_eigen, cost))
    {
      // We are out of the map
      return false;
    }
    return checkCost(cost);
  }

  /**
   * @brief Method to check the cost given the costmap 2D
   * @param[in] cost : cost to check
   * @return True if there is no collision or if we are in unknown space,
   *         False otherwise
   */
  bool checkCost(const unsigned char& cost) const
  {
    // Check the cost - not all these checks are necessary, but we explicitly
    // add them for teaching purposes
    // Doc:https://wiki.ros.org/costmap_2d?action=AttachFile&do=get&target=costmapspec.png
    const unsigned char k_threshold = 50;
    if (cost == costmap_2d::LETHAL_OBSTACLE)
    {
      // In this case, we are in collision
      return false;
    }
    else if (cost <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             cost >= k_threshold)
    {
      // In this case, we are not completly in collision, but we may be very
      // close to an obstacle
      return false;
    }
    else if (cost <= k_threshold || cost == costmap_2d::NO_INFORMATION)
    {
      // In this case we are not in collision so we keep going with the checks
      // We plan also in unknown space, because it is a global planner
      return true;
    }

    return true;
  }

  /**
   * @brief Method to query the cost map to get the cost. Return true if we
   *        manage to extract the cost. The position to be check is also
   *        "inflated" by the robot radius.
   * @param[in] state: (x,y) position in Eigen format
   * @param[out] cost: result from costmap
   * @return True if we can get the cost, False otherwise
   */
  virtual bool getCostAtState(const Eigen::Vector2d& state,
                              unsigned char& cost) const
  {
    unsigned int state_x_i, state_y_i;
    Eigen::Vector2d inflated_state(state.x(), state.y());
    if (!costmap_->worldToMap(inflated_state.x(), inflated_state.y(), state_x_i,
                              state_y_i))
    {
      // We are out of the map
      return false;
    }

    // Get the cost
    cost = costmap_->getCost(state_x_i, state_y_i);
    return true;
  }

  double getInterpolationFactor() const { return interpolation_factor_; }

protected:
  double interpolation_factor_;
  costmap_2d::Costmap2D* costmap_;
};

class GridmapMotionValidator : public base::MotionValidator
{
public:
  GridmapMotionValidator(
      const base::SpaceInformationPtr& space_info,
      std::shared_ptr<GridmapValidityChecker> validity_checker)
      : base::MotionValidator(space_info), validity_checker_(validity_checker)
  {
    interpolation_factor_ = validity_checker_->getInterpolationFactor();
  }

  virtual bool checkMotion(const base::State* s1, const base::State* s2) const
  {
    std::pair<base::State*, double> unused;
    return checkMotion(s1, s2, unused);
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  virtual bool checkMotion(const base::State* s1, const base::State* s2,
                           std::pair<base::State*, double>& last_valid) const
  {
    // Get the states in eigen format
    const RStateSpace::StateType* s1_derived =
        static_cast<const RStateSpace::StateType*>(s1);
    Eigen::Vector2d s1_eigen(s1_derived->values[0], s1_derived->values[1]);

    const RStateSpace::StateType* s2_derived =
        static_cast<const RStateSpace::StateType*>(s2);
    Eigen::Vector2d s2_eigen(s2_derived->values[0], s2_derived->values[1]);

    // Discretize the connection s1-s2
    Eigen::Vector2d direction(s2_eigen - s1_eigen);
    int n_states = std::floor(direction.norm() / interpolation_factor_);

    // Check the particular case where n_states is 0. In this case, check just
    // the start and end states
    if (n_states == 0)
    {
      return validity_checker_->isValid(s1) && validity_checker_->isValid(s2);
    }

    // Iterate over the states along the connection
    bool valid = true;
    for (int i = 0; i <= n_states; ++i)
    {
      Eigen::Vector2d state_i =
          s1_eigen + double(i) / double(n_states) * direction;
      unsigned char cost;
      if (!validity_checker_->getCostAtState(state_i, cost))
      {
        valid = false;
      }
      else
      {
        valid = validity_checker_->checkCost(cost);
      }

      if (!valid)
      {
        if (last_valid.first != nullptr)
        {
          ompl::base::ScopedState<ompl::RStateSpace> last_valid_state(
              si_->getStateSpace());
          last_valid_state->values[0] = state_i.x();
          last_valid_state->values[1] = state_i.y();
          si_->copyState(last_valid.first, last_valid_state.get());
        }

        last_valid.second = static_cast<double>(i / n_states);
        break;
      }
    }

    return valid;
  }

protected:
  typename std::shared_ptr<GridmapValidityChecker> validity_checker_;
  double interpolation_factor_;
};

} // end namespace ompl

