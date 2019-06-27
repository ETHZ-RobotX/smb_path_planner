/**
  * @author Luca Bartolomei, V4RL
  * @brief  Parameters for the planner
  * @date   13.06.2019
  */


#pragma once

#include <vector>
#include <string>

#include <ros/ros.h>
#include <Eigen/Dense>

namespace smb_planner {

    /**
     * @brief Different traversability status
     */
    enum TraversabilityStatus {TRAVERSABLE, UNTRAVERSABLE, UNKNOWN};

    /**
     * @brief Parameters used by global planner
     */
    struct GlobalPlannerParameters {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // OMPL optimization objective
        bool simplify_solution;
        bool trust_approx_solution;
        bool use_distance_threshold;
        double distance_threshold;

        int planner_type;
        double num_seconds_to_plan;
        double global_timer_dt;
        double global_interp_dt;

        // Planning behaviours
        bool optimistic_voxblox;

        // Bounds on the size of the map.
        Eigen::Vector3d lower_bound;
        Eigen::Vector3d upper_bound;
    };

    /**
     * @brief Parameters used by local planner
     */
    struct LocalPlannerParameters {
        // Thresholds for planning
        double local_goal_distance;
        double sec_ahead_planner;

        double local_replan_dt;
        double command_dt;
        double prediction_horizon_mpc;

        // Chomp parameters
        int nq;			    			// number of poses q in xi
        double dt;   	    		// time step
        double eta; 					// >= 1, regularization factor for grad. descent
        double lambda;      	// weight of smoothness objective
        double cost_gain;			// Gain inside cost function (usually 10)
        double eps; 					// Epsilon for the obstacle cost function
    };

    /**
     * @brief All Parameters used by both local and global planners
     */
    struct PlannerParameters {
        // Frame ID where we plan in
        std::string frame_id;
        
        // File input if we want to specify a voxblox map
        std::string input_filepath;

        // Utilities
        bool visualize;
        bool verbose_planner;
        double voxel_size; // Cache the size of the voxels used by the map.

        // Planning
        double bounding_box_inflation;
        double robot_radius;

        double planning_height;
        double max_initial_rotation;
        double threshold_goal_reached;

        // Traversability
        bool check_traversability;
        double traversability_threshold;
        double maximum_difference_elevation;

        // Velocities and accelerations
        double v_max;
        double a_max;
        double v_yaw_max;
        double a_yaw_max;
        double v_rot_on_spot;
        double sampling_dt;

        // Local and Global planners specific parameters
        LocalPlannerParameters local_params;
        GlobalPlannerParameters global_params;
    };

     /**
      * @brief Methods to read all the parameters from ROS server
      * @param[in] nh : Node handle for ROS communication
      * @param[out] params : parameters to be stored
      * @return True if process was successful, False otherwise
      */
    bool readPlannerParameters(
            const ros::NodeHandle& nh,
            PlannerParameters &params);

    /**
      * @brief Methods to read the parameters for global planner from ROS server
      * @param[in] nh : Node handle for ROS communication
      * @param[out] params : parameters to be stored
      * @return True if process was successful, False otherwise
      */
    bool readGlobalPlannerParameters(
            const ros::NodeHandle& nh,
            GlobalPlannerParameters &params);

    /**
      * @brief Methods to read the parameters for local planner from ROS server
      * @param[in] nh : Node handle for ROS communication
      * @param[out] params : parameters to be stored
      * @return True if process was successful, False otherwise
      */
    bool readLocalPlannerParameters(
            const ros::NodeHandle& nh,
            LocalPlannerParameters &params);

} // end namespace smb_planner
