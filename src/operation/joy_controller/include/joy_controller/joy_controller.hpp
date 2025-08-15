#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

namespace operation
{
    class JoyController
    {
        public:
            JoyController();
            ~JoyController();
            void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;
            ros::Subscriber sub_joy_;
            ros::Publisher pub_cmd_vel_;
            geometry_msgs::Twist cmd_vel_;

            // joy stick axes to be loaded from yaml file
            int joy_top_left_button_idx;
            int joy_top_right_button_idx;
            int joy_left_stick_x_idx;
            int joy_left_stick_y_idx;
            int joy_right_stick_x_idx;
            int joy_right_stick_y_idx;

            // max command values to be loaded from yaml file
            float abs_max_linear_vel_x;
            float abs_max_linear_vel_y;
            float abs_max_angular_vel_z;
    };
}