#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

namespace gazebo
{
    class VelDriver
    {
        public:
            VelDriver();
            ~VelDriver();
            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber sub_cmd_vel_;

            ros::Publisher pub_cmd_front_left_steer, pub_cmd_front_right_steer, pub_cmd_rear_left_steer, pub_cmd_rear_right_steer;
            ros::Publisher pub_cmd_front_left_rotor, pub_cmd_front_right_rotor, pub_cmd_rear_left_rotor, pub_cmd_rear_right_rotor;
            std_msgs::Float64 cmd_front_left_steer, cmd_front_right_steer, cmd_rear_left_steer, cmd_rear_right_steer;
            std_msgs::Float64 cmd_front_left_rotor, cmd_front_right_rotor, cmd_rear_left_rotor, cmd_rear_right_rotor;

            // vehicle parameters to be loaded from yaml file
            float l_f, l_r, d_l, d_r;
            float tire_radius;
    };
}