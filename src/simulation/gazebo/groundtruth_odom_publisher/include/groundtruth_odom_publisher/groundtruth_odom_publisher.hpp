#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace gazebo
{
    class GroundTruthOdomPublisher
    {
        public:
            GroundTruthOdomPublisher();
            ~GroundTruthOdomPublisher();
            void groundTruthOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber sub_groundtruth_odom_;
            ros::Time prev_tf_timestamp_;
            tf2_ros::TransformBroadcaster tf_broadcaster_;
            geometry_msgs::TransformStamped odom_tf_;
            std::string base_frame_name, odom_frame_name;
    };
}