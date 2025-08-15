# pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include "nullspace_mpc/param.hpp"
#include "nullspace_mpc/nullspace_mpc_core.hpp"
#include "mpc_eval_msgs/MPCEval.h"

namespace controller
{
    class MPC // MPC execution (dependent on ROS)
    {
        public:
            MPC();
            ~MPC();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            // publisher
            ros::Publisher pub_cmd_vel_;
            ros::Publisher pub_cmd_absvel_;
            ros::Publisher pub_cmd_vx_;
            ros::Publisher pub_cmd_vy_;
            ros::Publisher pub_cmd_omega_;
            ros::Publisher pub_mppi_calc_time_;
            ros::Publisher pub_mppi_optimal_traj_;
            ros::Publisher pub_mppi_sampled_traj_;
            ros::Publisher pub_mppi_overlay_text_;
            ros::Publisher pub_mppi_via_state_sequence_;
            ros::Publisher pub_mpc_eval_msg_;

            // subscriber
            //// odometry (global vehicle pose)
            ros::Subscriber sub_odom_;
            void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
            bool odom_received_;
            nav_msgs::Odometry latest_odom_;
            common_type::XYYaw observed_state_;
            common_type::VxVyOmega observed_velocity_;

            //// reference path
            ros::Subscriber sub_ref_path_;
            void refPathCallback(const nav_msgs::Path::ConstPtr& msg);
            bool ref_path_received_;
            nav_msgs::Path latest_ref_path_;
            common_type::XYYaw global_goal_state_;

            //// collision costmap
            ros::Subscriber sub_collision_costmap_;
            void collisionCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
            bool collision_costmap_received_;
            grid_map::GridMap collision_costmap_;

            //// distance error map
            ros::Subscriber sub_distance_error_map_;
            void distanceErrorMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
            bool distance_error_map_received_;
            grid_map::GridMap distance_error_map_;

            //// reference yaw map
            ros::Subscriber sub_ref_yaw_map_;
            void refYawMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
            bool ref_yaw_map_received_;
            grid_map::GridMap ref_yaw_map_;

            // method to calculate via state sequence
            std::vector<common_type::XYYaw> calcViaStateSequence(
                const common_type::XYYaw& observed_state,
                const nav_msgs::Path& ref_path,
                const double lookahead_distance,
                const std::vector<int>& idx_via_states
            );

            // timer
            ros::Timer timer_control_interval_;
            void calcControlCommand(const ros::TimerEvent& event);

            // rviz visualization
            void publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence);
            void publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences);
            void publishViaStateSequence(const std::vector<common_type::XYYaw>& via_state_sequence);
            void publishOverlayText(const std::string& text);

            // parameters
            double lookahead_distance_; // [m]
            std::vector<int> idx_via_states_; // index of via states

            // mppi core instance
            MPCCore* mpc_core_;
    };
} // namespace controller