#include "world_handler/world_handler.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "world_handler");
    gazebo::WorldHandler world_handler;
    ros::spin();
    return 0;
};
