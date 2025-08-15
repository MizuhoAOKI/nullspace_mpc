#include "mppi_3d/mppi_3d.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_3d");
    controller::MPPI mppi;
    ros::spin();
    return 0;
};
