#include "mppi_h/mppi_h.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mppi_h");
    controller_mppi_h::MPPI mppi;
    ros::spin();
    return 0;
};
