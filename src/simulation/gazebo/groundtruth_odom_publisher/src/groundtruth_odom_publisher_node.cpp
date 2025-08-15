#include "groundtruth_odom_publisher/groundtruth_odom_publisher.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "groundtruth_odom_publisher");
    gazebo::GroundTruthOdomPublisher groundtruth_odom_publisher;
    ros::spin();
    return 0;
};
