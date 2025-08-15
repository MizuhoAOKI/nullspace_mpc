#include "nullspace_mpc/nullspace_mpc.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nullspace_mpc");
    controller::MPC mpc;
    ros::spin();
    return 0;
};
