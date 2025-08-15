#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"

namespace gazebo
{

// constructor
GroundTruthOdomPublisher::GroundTruthOdomPublisher()
    : nh_(""), private_nh_("~")
{
    // load parameters 

    //// frame names
    private_nh_.param<std::string>("base_frame", base_frame_name, "base_link");
    private_nh_.param<std::string>("odom_frame", odom_frame_name, "odom");

    //// subscribing topic names
    std::string groundtruth_odom_topic;
    private_nh_.param<std::string>("groundtruth_odom_topic", groundtruth_odom_topic, "/groundtruth_odom");

    // initialize subscribers
    sub_groundtruth_odom_ = nh_.subscribe(groundtruth_odom_topic, 1, &GroundTruthOdomPublisher::groundTruthOdomCallback, this);
}

// destructor
GroundTruthOdomPublisher::~GroundTruthOdomPublisher()
{
    // No Contents
}

// /odom topic callback
void GroundTruthOdomPublisher::groundTruthOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // get current time
    ros::Time time_now_ = ros::Time::now();

    // publish tf (avoiding publishing tf with same timestamp repeatedly)
    if(prev_tf_timestamp_ != time_now_)
    {
        // update cache
        prev_tf_timestamp_ = time_now_;

        // publish odom tf
        odom_tf_.header.stamp = time_now_;
        odom_tf_.header.frame_id = odom_frame_name;
        odom_tf_.child_frame_id = base_frame_name;
        odom_tf_.transform.translation.x = msg->pose.pose.position.x;
        odom_tf_.transform.translation.y = msg->pose.pose.position.y;
        odom_tf_.transform.translation.z = msg->pose.pose.position.z;
        odom_tf_.transform.rotation.x = msg->pose.pose.orientation.x;
        odom_tf_.transform.rotation.y = msg->pose.pose.orientation.y;
        odom_tf_.transform.rotation.z = msg->pose.pose.orientation.z;
        odom_tf_.transform.rotation.w = msg->pose.pose.orientation.w;
        tf_broadcaster_.sendTransform(odom_tf_);
    }
}

} // namespace gazebo
