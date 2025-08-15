#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>

namespace planning
{
    class ReferenceCostmapGenerator
    {
        public:
            ReferenceCostmapGenerator();
            ~ReferenceCostmapGenerator();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            // publisher
            //// goal pose marker
            ros::Publisher pub_goal_pose_marker_;
            void publishGoalPoseArrowMarker();
            void publishGoalPoseSphereMarker();

            //// distance error map
            ros::Publisher pub_distance_error_map_;
            void publishDistanceErrorMap();
            grid_map::GridMap distance_error_map_;

            //// reference yaw angle map
            ros::Publisher pub_ref_yaw_map_;
            void publishReferenceYawMap();
            grid_map::GridMap ref_yaw_map_;

            // subscriber
            //// reference path
            ros::Subscriber sub_ref_path_;
            void refPathCallback(const nav_msgs::Path::ConstPtr& msg);
            bool ref_path_received_;
            nav_msgs::Path latest_ref_path_;
            
            //// map
            ros::Subscriber sub_map_;
            void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
            bool map_received_;
            grid_map::GridMap latest_map_;

            // parameters
            double map_resolution_scale_;
    };
} // namespace planning
