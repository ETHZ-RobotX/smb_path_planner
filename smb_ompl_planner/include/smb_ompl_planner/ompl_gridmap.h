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
#include <ompl/base/MotionValidator.h>
#include <ompl/base/StateValidityChecker.h>

namespace ompl
{

/**
 * @brief Class used to check the collision of the robot with the obstacles in
 *        the grid map
 */
class GridmapValidityChecker : public base::StateValidityChecker
{
public:
  GridmapValidityChecker(const base::SpaceInformationPtr& space_info,
                         const double robot_radius)
      : base::StateValidityChecker(space_info), robot_radius_(robot_radius)
  {
  }

  virtual bool isValid(const base::State* state) const
  {
    if (!si_->satisfiesBounds(state))
    {
      return false;
    }

    std::cout << "Collision checker not implemented yet" << std::endl;
    return false;
  }

  // Returns whether there is a collision: true if yes, false if not.
  virtual bool
  checkCollisionWithRobot(const Eigen::Vector2d& robot_position) const
  {
    return false;
  }

protected:
  double robot_radius_;
};

class GridmapMotionValidator : public base::MotionValidator
{
public:
  GridmapMotionValidator(
      const base::SpaceInformationPtr& space_info,
      std::shared_ptr<GridmapValidityChecker> validity_checker)
      : base::MotionValidator(space_info), validity_checker_(validity_checker)
  {
  }

  virtual bool checkMotion(const base::State* s1, const base::State* s2) const
  {
    std::pair<base::State*, double> unused;
    return checkMotion(s1, s2, unused);
  }

  // Check motion returns *false* if invalid, *true* if valid.
  // So opposite of checkCollision, but same as isValid.
  // last_valid is the state and percentage along the trajectory that's
  // a valid state.
  virtual bool checkMotion(const base::State* s1, const base::State* s2,
                           std::pair<base::State*, double>& last_valid) const
  {
    return false;
  }

protected:
  typename std::shared_ptr<GridmapValidityChecker> validity_checker_;
  double robot_radius_;
};

} // end namespace ompl
