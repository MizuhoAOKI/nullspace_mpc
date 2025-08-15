#include "mppi_4d/mppi_4d.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_4d");
    controller::MPPI mppi;
    ros::spin();
    return 0;
};
