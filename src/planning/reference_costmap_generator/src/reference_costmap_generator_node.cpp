#include "reference_costmap_generator/reference_costmap_generator.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reference_costmap_generator");
    planning::ReferenceCostmapGenerator reference_costmap_generator;
    ros::spin();
    return 0;
};
