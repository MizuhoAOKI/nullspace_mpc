#include "joy_controller/joy_controller.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_controller");
    operation::JoyController joy_controller;
    ros::spin();
    return 0;
};
