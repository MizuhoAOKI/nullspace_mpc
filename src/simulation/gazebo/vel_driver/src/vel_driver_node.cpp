#include "vel_driver/vel_driver.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_driver");
    gazebo::VelDriver vel_driver;
    ros::spin();
    return 0;
};
