#pragma once

#include <ros/ros.h>
#include <signal.h>

namespace gazebo
{
    class WorldHandler
    {
        public:
            WorldHandler();
            ~WorldHandler();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Timer timer_;
            static void sigintHandler(int sig);
            void timerCallback(const ros::TimerEvent& event);
    };
}