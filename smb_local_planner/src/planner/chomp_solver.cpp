#include "smb_local_planner/chomp_solver.h"

namespace smb_local_planner {

ChompSolver::ChompSolver(voxblox::EsdfMap::Ptr esdf_map,
                         const PlannerParameters &params)
    : esdf_map_(esdf_map), params_(params), check_traversability_(false),
      initialized_(false), c_dim_(3) {
  CHECK_NOTNULL(esdf_map_);

  // Initialize the number of size of the trajectory
  xi_dim_ = params_.local_params.nq * c_dim_;
}

ChompSolver::~ChompSolver() {}

bool ChompSolver::solve(const Eigen::VectorXd &start,
                        const Eigen::VectorXd &goal) {

  // Get the initial line path (excluding start and goal states)
  initStraightLinePath(start, goal);
  std::vector<double> parameters(xi_dim_);

  for (int i = 0; i < xi_dim_; i += c_dim_) {
    parameters[i] = xi_(i);
    parameters[i + 1] = xi_(i + 1);
    parameters[i + 2] = xi_(i + 2);
  }

  // Create an object.
  ceres::GradientProblem problem(new NestedCeresFunction(xi_dim_, this));

  ceres::GradientProblemSolver::Options options;
  options.line_search_direction_type =
      ceres::LineSearchDirectionType::NONLINEAR_CONJUGATE_GRADIENT;
  options.logging_type = ceres::SILENT;
  options.minimizer_progress_to_stdout = false;
  options.max_solver_time_in_seconds = params_.local_params.local_replan_dt;

  options.line_search_interpolation_type =
      ceres::LineSearchInterpolationType::BISECTION;
  ceres::GradientProblemSolver::Summary summary;

  // Fire up solver
  ceres::Solve(options, problem, parameters.data(), &summary);
  if (params_.verbose_planner) {
    std::cout << summary.FullReport() << std::endl;
  }

  if (summary.termination_type == ceres::TerminationType::NO_CONVERGENCE) {
    ROS_ERROR("[Smb Local Planner][CHOMP Solver] No covergence!");
    return false;
  }

  // Get the solution after optimization.
  for (int i = 0; i < parameters.size(); i += c_dim_) {
    xi_(i) = parameters[i];
    xi_(i + 1) = parameters[i + 1];
    xi_(i + 2) = parameters[i + 2];
  }
  return true;
}

bool ChompSolver::solve(const Eigen::VectorXd &start,
                        const Eigen::VectorXd &goal,
                        const grid_map::GridMap &traversability_map) {
  // Store traversability map and start planning
  traversability_map_ = traversability_map;
  check_traversability_ = true;

  return solve(start, goal);
}

void ChompSolver::initChompSolver() {
  // Initializes gradient descent vectors and matrices
  AA_ = Eigen::MatrixXd::Zero(xi_dim_, xi_dim_);

  for (size_t ii(0); ii < params_.local_params.nq; ++ii) {
    AA_.block(c_dim_ * ii, c_dim_ * ii, c_dim_, c_dim_) =
        2.0 * Eigen::MatrixXd::Identity(c_dim_, c_dim_);
    if (ii > 0) {
      AA_.block(c_dim_ * (ii - 1), c_dim_ * ii, c_dim_, c_dim_) =
          -1.0 * Eigen::MatrixXd::Identity(c_dim_, c_dim_);
      AA_.block(c_dim_ * ii, c_dim_ * (ii - 1), c_dim_, c_dim_) =
          -1.0 * Eigen::MatrixXd::Identity(c_dim_, c_dim_);
    }
  }

  AA_ /= params_.local_params.dt * params_.local_params.dt *
         (params_.local_params.nq + 1);

  bb_ = Eigen::VectorXd::Zero(xi_dim_);
  bb_.block(0, 0, c_dim_, 1) = qs_;
  bb_.block(xi_dim_ - c_dim_, 0, c_dim_, 1) = qe_;
  bb_ /= -params_.local_params.dt * params_.local_params.dt *
         (params_.local_params.nq + 1);

  Ainv_ = AA_.inverse();

  // Set flag to true
  initialized_ = true;
}

void ChompSolver::initStraightLinePath(const Eigen::VectorXd &start,
                                       const Eigen::VectorXd &goal) {
  // Store initial and final states
  qs_ = start;
  qe_ = goal;

  // Initialize trajectory
  xi_ = Eigen::VectorXd::Zero(xi_dim_);

  Eigen::VectorXd dxi(c_dim_);
  dxi << (qe_(0) - qs_(0)) / (params_.local_params.nq - 1),
      (qe_(1) - qs_(1)) / (params_.local_params.nq - 1),
      (qe_(2) - qs_(2)) / (params_.local_params.nq - 1);

  for (size_t ii(0); ii < params_.local_params.nq; ++ii) {
    xi_.block(c_dim_ * ii, 0, c_dim_, 1) = qs_ + ii * dxi;
  }

  // Sets gradient descent vectors and matrices with the initialized path
  initChompSolver();
}

void ChompSolver::initStackedPath(const Eigen::VectorXd &start,
                                  const Eigen::VectorXd &goal) {
  qs_ = start;
  qe_ = goal;

  xi_ = Eigen::VectorXd::Zero(xi_dim_);
  for (size_t ii(0); ii < params_.local_params.nq; ++ii) {
    xi_.block(c_dim_ * ii, 0, c_dim_, 1) = qs_;
  }

  // Sets gradient descent vectors and matrices with the initialized path
  initChompSolver();
}

bool ChompSolver::computeCostAndGradient(const Eigen::VectorXd &solution,
                                         double *cost, double *gradient) {

  Eigen::VectorXd nabla_smooth(AA_ * solution + bb_);
  Eigen::VectorXd const &xidd(nabla_smooth);
  xi_ = solution;

  double F_obs = 0.0;    // Obstacle functional value
  double F_smooth = 0.0; // Smoothness functional value

  /*** This part of the method has been modified from the planner available
   *   here: https://github.com/rafaelvalencia/path-adaptor
   */

  // Constrained CHOMP.
  // Impose nonholonmic (NH) restrictions with the rolling constraint.
  // Next we evaluate the constraint functional and its Jacobian b and C,
  // respectively, as it appears in CHOMP's IJRR paper
  Eigen::MatrixXd CC(Eigen::MatrixXd::Zero(xi_dim_, 1));
  double b = 0;

  Eigen::VectorXd c1(Eigen::VectorXd::Zero(3));
  Eigen::VectorXd c2(Eigen::VectorXd::Zero(3));
  Eigen::VectorXd cf1(Eigen::VectorXd::Zero(3));
  Eigen::VectorXd cf2(Eigen::VectorXd::Zero(3));

  for (size_t iq(0); iq < (params_.local_params.nq - 1); ++iq) {
    // Evaluate the constraint functional. It is defined by a sum of auxiliar
    // functions that depend on only two consecutive robot poses.
    Eigen::VectorXd const q1(xi_.block(iq * c_dim_, 0, c_dim_, 1));
    Eigen::VectorXd const q2(xi_.block((iq + 1) * c_dim_, 0, c_dim_, 1));

    // Nonholonomic constraint
    double nhc = (q2(0) - q1(0)) * sin(q1(2)) - (q2(1) - q1(1)) * cos(q1(2));
    // Forward motion constraint
    double fmc =
        cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)) +
        sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)),
                 2));

    // Square of the rolling constraint + forward motion constrain
    b += nhc * nhc + fmc;

    // Computation of the Jacobian of the NH constraint.
    // Jacobian of the auxiliar functions
    c1(0) = -2 * sin(q1(2)) *
            (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)));
    c1(1) = 2 * cos(q1(2)) *
            (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)));
    c1(2) = -2 * (cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1))) *
            (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)));
    c2(0) = 2 * sin(q1(2)) *
            (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)));
    c2(1) = -2 * cos(q1(2)) *
            (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)));
    c2(2) = 0;
    // end of NHC Jacobian

    // Computation of the Jacobian of the NH constraint.
    // Jacobian of the auxiliar functions
    cf1(0) = cos(q1(2)) +
             (cos(q1(2)) *
              (cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)))) /
                 sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) +
                              sin(q1(2)) * (q1(1) - q2(1)),
                          2));
    cf1(1) = sin(q1(2)) +
             (sin(q1(2)) *
              (cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)))) /
                 sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) +
                              sin(q1(2)) * (q1(1) - q2(1)),
                          2));
    cf1(2) = cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)) +
             ((cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1))) *
              (cos(q1(2)) * (q1(1) - q2(1)) - sin(q1(2)) * (q1(0) - q2(0)))) /
                 sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) +
                              sin(q1(2)) * (q1(1) - q2(1)),
                          2));
    cf2(0) = -cos(q1(2)) -
             (cos(q1(2)) *
              (cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)))) /
                 sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) +
                              sin(q1(2)) * (q1(1) - q2(1)),
                          2));
    cf2(1) = -sin(q1(2)) -
             (sin(q1(2)) *
              (cos(q1(2)) * (q1(0) - q2(0)) + sin(q1(2)) * (q1(1) - q2(1)))) /
                 sqrt(pow(cos(q1(2)) * (q1(0) - q2(0)) +
                              sin(q1(2)) * (q1(1) - q2(1)),
                          2));
    cf2(2) = 0;
    // end of FMC Jacobian

    if (iq == 0) {
      c2(0) += 2 * sin(qs_(2)) * (cos(qs_(2)) * (qs_(1) - q1(1)) -
                                  sin(qs_(2)) * (qs_(0) - q1(0)));
      c2(1) += -2 * cos(qs_(2)) * (cos(qs_(2)) * (qs_(1) - q1(1)) -
                                   sin(qs_(2)) * (qs_(0) - q1(0)));
      c2(2) += 0;

      cf2(0) += -cos(qs_(2)) -
                (cos(qs_(2)) * (cos(qs_(2)) * (qs_(0) - q1(0)) +
                                sin(qs_(2)) * (qs_(1) - q1(1)))) /
                    sqrt(pow(cos(qs_(2)) * (qs_(0) - q1(0)) +
                                 sin(qs_(2)) * (qs_(1) - q1(1)),
                             2));
      cf2(1) += -sin(qs_(2)) -
                (sin(qs_(2)) * (cos(qs_(2)) * (qs_(0) - q1(0)) +
                                sin(qs_(2)) * (qs_(1) - q1(1)))) /
                    sqrt(pow(cos(qs_(2)) * (qs_(0) - q1(0)) +
                                 sin(qs_(2)) * (qs_(1) - q1(1)),
                             2));
      cf2(2) += 0;

      double nhstart =
          (q1(0) - qs_(0)) * sin(qs_(2)) - (q1(1) - qs_(0)) * cos(qs_(2));
      double fmstart = cos(qs_(2)) * (qs_(0) - q1(0)) +
                       sin(qs_(2)) * (qs_(1) - q1(1)) +
                       sqrt(pow(cos(qs_(2)) * (qs_(0) - q1(0)) +
                                    sin(qs_(2)) * (qs_(1) - q1(1)),
                                2));    // forward motion constraint
      b += nhstart * nhstart + fmstart; // square of the rolling constraint
    }

    if (std::isnan(cf1(0))) {
      cf1(0) = 0.0;
    }
    if (std::isnan(cf1(1))) {
      cf1(1) = 0.0;
    }
    if (std::isnan(cf1(2))) {
      cf1(2) = 0.0;
    }
    if (std::isnan(cf2(0))) {
      cf2(0) = 0.0;
    }
    if (std::isnan(cf2(1))) {
      cf2(1) = 0.0;
    }
    if (std::isnan(cf2(2))) {
      cf2(2) = 0.0;
    }

    // Update Jacobian with the contributions from the Jacobian of the auxiliar
    // functions
    CC.block(iq * c_dim_, 0, c_dim_, 1) =
        CC.block(iq * c_dim_, 0, c_dim_, 1) + c1 + cf1;
    CC.block((iq + 1) * c_dim_, 0, c_dim_, 1) =
        CC.block((iq + 1) * c_dim_, 0, c_dim_, 1) + c2 + cf2;
  }

  CC /= params_.local_params.dt * params_.local_params.dt *
        (params_.local_params.nq + 1);
  b /= params_.local_params.dt * params_.local_params.dt *
       (params_.local_params.nq + 1);

  // Store the transpose of the inverse of the matrix
  Eigen::MatrixXd CCtrans = CC.transpose();
  Eigen::VectorXd nabla_obs(Eigen::VectorXd::Zero(xi_dim_));

  for (size_t iq(0); iq < params_.local_params.nq; ++iq) {
    Eigen::VectorXd const qq(xi_.block(iq * c_dim_, 0, c_dim_, 1));
    Eigen::VectorXd qd;
    if (0 == iq) {
      qd = 0.5 * (xi_.block((iq + 1) * c_dim_, 0, c_dim_, 1) - qs_);
    } else if (iq == params_.local_params.nq - 1) {
      qd = 0.5 * (qe_ - xi_.block((iq - 1) * c_dim_, 0, c_dim_, 1));
    } else {
      qd = 0.5 * (xi_.block((iq + 1) * c_dim_, 0, c_dim_, 1) -
                  xi_.block((iq - 1) * c_dim_, 0, c_dim_, 1));
    }

    Eigen::VectorXd const &xx(qq.block(0, 0, 2, 1));
    Eigen::VectorXd const &xd(qd.block(0, 0, 2, 1));

    // In this case, C and W are NOT the same
    Eigen::MatrixXd JJ(Eigen::MatrixXd::Zero(2, 3));
    JJ(0, 0) = 1;
    JJ(1, 1) = 1;

    double const vel(xd.norm());
    if (vel < 1.0e-3) {
      // Avoid div by zero further down
      continue;
    }
    Eigen::VectorXd const xdn(xd / vel);
    Eigen::VectorXd const xdd(JJ * xidd.block(iq * c_dim_, 0, c_dim_, 1));
    Eigen::MatrixXd const prj(Eigen::MatrixXd::Identity(2, 2) -
                              xdn * xdn.transpose()); // hardcoded planar case
    Eigen::VectorXd const kappa(prj * xdd / pow(vel, 2.0));

    // Obstacle cost function

    double distance;
    Eigen::Vector3d position(xx(0), xx(1), params_.planning_height);
    Eigen::Vector3d proj_position(position);
    Eigen::Vector3d gradient_esdf;
    TraversabilityStatus traversability_status =
        TraversabilityStatus::TRAVERSABLE;

    // Check the traversability
    if (check_traversability_) {
      // If place is untraversable, then cost will be inf and we can use
      // the same gradient we obtain from voxblox
      traversability_status = utility_mapping::getTrasversabilityInformation(
          traversability_map_, position.head<2>(), params_.planning_height,
          params_.traversability_threshold,
          params_.maximum_difference_elevation, proj_position);
    }
    esdf_map_->getDistanceAndGradientAtPosition(proj_position, &distance,
                                                &gradient_esdf);

    // Get the cost and the gradient values by processing the information
    // from the map
    double d = distance - params_.robot_radius;
    Eigen::Vector3d gradient_obstacles;

    if (d < 0) {
      gradient_obstacles = -gradient_esdf;
    } else if (d <= params_.local_params.eps) {
      gradient_obstacles =
          1.0 / params_.local_params.eps *
          (d * gradient_esdf - params_.local_params.eps * gradient_esdf);
    } else {
      gradient_obstacles = Eigen::Vector3d::Zero();
    }

    double cost_obst = 0.0;

    if (traversability_status == TraversabilityStatus::UNTRAVERSABLE) {
      cost_obst = std::numeric_limits<double>::infinity();
    } else if (d < 0) {
      cost_obst = params_.local_params.eps * 0.5 - d;
    } else if (d < params_.local_params.eps) {
      cost_obst = 1 / (2.0 * params_.local_params.eps) *
                  (d - params_.local_params.eps) *
                  (d - params_.local_params.eps);
    }

    // Increase the gradient for obstacles
    gradient_obstacles *= params_.local_params.cost_gain;
    nabla_obs.block(iq * c_dim_, 0, c_dim_, 1) +=
        JJ.transpose() * vel *
        (prj * gradient_obstacles.head(2) - cost_obst * kappa);

    // smoothness and obstacle costs
    F_smooth += pow(vel / params_.local_params.dt, 2.0);
    F_obs += cost_obst * (vel / params_.local_params.dt);
  }

  Eigen::VectorXd dxi(Ainv_ * (nabla_obs +
                               params_.local_params.lambda *
                                   nabla_smooth)); // unconstrained step
  Eigen::VectorXd gradient_eigen;
  if (b < 1.0e-10) {
    // Unconstrained update to initialize trajectory (it starts with b approx to
    // zero)
    xi_ -= dxi / params_.local_params.eta;
    Eigen::VectorXd dxi(
        Ainv_ * (nabla_obs + params_.local_params.lambda * nabla_smooth));
    gradient_eigen = dxi;
  } else {
    // Constrained optimization update
    Eigen::VectorXd CAC(CCtrans * Ainv_ * CC);
    Eigen::MatrixXd CACinv(CAC.inverse());

    // Auxiliary matrix for the update equation
    Eigen::VectorXd Proj(Ainv_ * CC * CACinv);
    Eigen::VectorXd cdxi(-dxi / params_.local_params.eta +
                         Proj * CCtrans * dxi / params_.local_params.eta -
                         Proj * b);
    // Update solution and the gradient
    xi_ += cdxi;
    gradient_eigen = -cdxi;
  }

  // Outputs:
  // Cost
  *cost = (F_obs + 0.5 * params_.local_params.lambda * F_smooth) /
          (params_.local_params.nq + 1);

  // Cost
  if (gradient != nullptr) {
    for (int i = 0; i < gradient_eigen.rows(); ++i) {
      gradient[i] = gradient_eigen(i);
    }
  }
  return true;
}

void ChompSolver::boundTrajectoryAngles() {
  for (size_t iq = 0; iq < params_.local_params.nq; ++iq) {
    // Makes the angles to be between Pi to -Pi
    Eigen::VectorXd q(xi_.block(iq * c_dim_, 0, c_dim_, 1));
    utility_mapping::pi2pi(q(2));
    xi_.block(iq * c_dim_, 0, c_dim_, 1) = q;
  }
}

/******************************************************************************/
bool ChompSolver::NestedCeresFunction::Evaluate(const double *parameters,
                                                double *cost,
                                                double *gradient) const {
  CHECK_NOTNULL(parent_);

  // Step 1: unpack the current solution
  Eigen::VectorXd solution(N_);
  for (int i = 0; i < N_; i += 3) {
    solution.block(i, 0, 3, 1) << parameters[i], parameters[i + 1],
        parameters[i + 2];
  }

  // Step 2: compute total cost and gradient
  if (!parent_->computeCostAndGradient(solution, cost, gradient))
    return false;

  return true;
}

int ChompSolver::NestedCeresFunction::NumParameters() const { return N_; }

} // end namespace smb_local_planner
