#include "world_handler/world_handler.hpp"

namespace gazebo
{

// constructor
WorldHandler::WorldHandler()
    : nh_(""), private_nh_("~")
{
    // register SIGINT handler
    signal(SIGINT, WorldHandler::sigintHandler);
}

// destructor
WorldHandler::~WorldHandler()
{
    // No Contents
}

// add signal handler of Ctrl+C
void WorldHandler::sigintHandler(int sig)
{
    ROS_WARN("Ctrl+C is pressed. kill gazebo process.");
    int result_kill_gazebo = system("killall -9 gzserver gzclient&");
    ros::shutdown();
}

// timer callback
void WorldHandler::timerCallback(const ros::TimerEvent& event)
{
    // Not using this timer callback now...
    ROS_INFO("timer callback in world_handler node.");
}

} // namespace gazebo