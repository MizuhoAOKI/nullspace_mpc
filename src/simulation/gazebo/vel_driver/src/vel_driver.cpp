#include "vel_driver/vel_driver.hpp"

namespace gazebo
{

// constructor
VelDriver::VelDriver()
    : nh_(""), private_nh_("~")
{
    // load parameters 

    //// vehicle parameters
    private_nh_.param<float>("l_f", l_f, 0.5);
    private_nh_.param<float>("l_r", l_r, 0.5);
    private_nh_.param<float>("d_l", d_l, 0.5);
    private_nh_.param<float>("d_r", d_r, 0.5);
    private_nh_.param<float>("tire_radius", tire_radius, 0.2);

    //// subscribing topic names
    std::string control_cmd_vel_topic;
    private_nh_.param<std::string>("control_cmd_vel_topic", control_cmd_vel_topic, "/cmd_vel");

    //// publishing topic names
    std::string front_left_steer_cmd_topic, front_right_steer_cmd_topic, rear_left_steer_cmd_topic, rear_right_steer_cmd_topic;
    std::string front_left_rotor_cmd_topic, front_right_rotor_cmd_topic, rear_left_rotor_cmd_topic, rear_right_rotor_cmd_topic;
    private_nh_.param<std::string>("front_left_steer_cmd_topic", front_left_steer_cmd_topic, "/fwids/front_left_steer_rad/command");
    private_nh_.param<std::string>("front_right_steer_cmd_topic", front_right_steer_cmd_topic, "/fwids/front_right_steer_rad/command");
    private_nh_.param<std::string>("rear_left_steer_cmd_topic", rear_left_steer_cmd_topic, "/fwids/rear_left_steer_rad/command");
    private_nh_.param<std::string>("rear_right_steer_cmd_topic", rear_right_steer_cmd_topic, "/fwids/rear_right_steer_rad/command");
    private_nh_.param<std::string>("front_left_rotor_cmd_topic", front_left_rotor_cmd_topic, "/fwids/front_left_rotor_radpersec/command");
    private_nh_.param<std::string>("front_right_rotor_cmd_topic", front_right_rotor_cmd_topic, "/fwids/front_right_rotor_radpersec/command");
    private_nh_.param<std::string>("rear_left_rotor_cmd_topic", rear_left_rotor_cmd_topic, "/fwids/rear_left_rotor_radpersec/command");
    private_nh_.param<std::string>("rear_right_rotor_cmd_topic", rear_right_rotor_cmd_topic, "/fwids/rear_right_rotor_radpersec/command");

    // initialize subscribers
    sub_cmd_vel_ = nh_.subscribe(control_cmd_vel_topic, 1, &VelDriver::cmdVelCallback, this);

    // initialize publishers
    pub_cmd_front_left_steer = nh_.advertise<std_msgs::Float64>(front_left_steer_cmd_topic, 10);
    pub_cmd_front_right_steer = nh_.advertise<std_msgs::Float64>(front_right_steer_cmd_topic, 10);
    pub_cmd_rear_left_steer = nh_.advertise<std_msgs::Float64>(rear_left_steer_cmd_topic, 10);
    pub_cmd_rear_right_steer = nh_.advertise<std_msgs::Float64>(rear_right_steer_cmd_topic, 10);
    pub_cmd_front_left_rotor = nh_.advertise<std_msgs::Float64>(front_left_rotor_cmd_topic, 10);
    pub_cmd_front_right_rotor = nh_.advertise<std_msgs::Float64>(front_right_rotor_cmd_topic, 10);
    pub_cmd_rear_left_rotor = nh_.advertise<std_msgs::Float64>(rear_left_rotor_cmd_topic, 10);
    pub_cmd_rear_right_rotor = nh_.advertise<std_msgs::Float64>(rear_right_rotor_cmd_topic, 10);
}

// destructor
VelDriver::~VelDriver()
{
    // No Contents
}

// /cmd_vel topic callback
void VelDriver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // if msg contains NaN, return without publishing commands
    if (std::isnan(msg->linear.x) || std::isnan(msg->linear.y) || std::isnan(msg->angular.z))
    {
        ROS_WARN("Received NaN in Twist Command. Skip this message.");
    }

    // parse received twist message
    // Note : urdf model axis (forward : x, left : y) and vehicle local axis (right : x, forward : y) are different.
    float v_r    = -msg->linear.y;  // velocity in the vehicle right direction
    float v_f    =  msg->linear.x;  // velocity in the vehicle forward direction
    float omega =  msg->angular.z; // angular velocity around the z-axis, counter-clockwise is positive

    // [for debug] annouce received Twist message
    ROS_DEBUG("Received Twist Command: v_right = %+5.1f, v_forward = %+5.1f, yaw_rate = %+5.1f", v_r, v_f, omega);

    // convert from Vx, Vy, Omega to 8 DoF vehicle control commands
    float vx_fl = v_r - omega * l_f;
    float vx_fr = v_r - omega * l_f;
    float vx_rl = v_r + omega * l_r;
    float vx_rr = v_r + omega * l_r;

    float vy_fl = v_f - omega * d_l;
    float vy_fr = v_f + omega * d_r;
    float vy_rl = v_f - omega * d_l;
    float vy_rr = v_f + omega * d_r;

    cmd_front_left_steer.data  = atan2(-vx_fl, vy_fl);
    cmd_front_right_steer.data = atan2(-vx_fr, vy_fr);
    cmd_rear_left_steer.data   = atan2(-vx_rl, vy_rl);
    cmd_rear_right_steer.data  = atan2(-vx_rr, vy_rr);

    cmd_front_left_rotor.data  = sqrt(vx_fl*vx_fl + vy_fl*vy_fl) / tire_radius; // [rad/s]
    cmd_front_right_rotor.data = sqrt(vx_fr*vx_fr + vy_fr*vy_fr) / tire_radius; // [rad/s]
    cmd_rear_left_rotor.data   = sqrt(vx_rl*vx_rl + vy_rl*vy_rl) / tire_radius; // [rad/s]
    cmd_rear_right_rotor.data  = sqrt(vx_rr*vx_rr + vy_rr*vy_rr) / tire_radius; // [rad/s]

    // [for debug] annouce publishing Twist message
    ROS_DEBUG("FWIDS Steer Commands: FL_steer = %+5.1f, FR_steer = %+5.1f, RL_steer = %+5.1f, RR_steer = %+5.1f", cmd_front_left_steer.data, cmd_front_right_steer.data, cmd_rear_left_steer.data, cmd_rear_right_steer.data);
    ROS_DEBUG("FWIDS Rotor Commands: FL_rotor = %+5.1f, FR_rotor = %+5.1f, RL_rotor = %+5.1f, RR_rotor = %+5.1f", cmd_front_left_rotor.data, cmd_front_right_rotor.data, cmd_rear_left_rotor.data, cmd_rear_right_rotor.data);

    // publish 8 DoF vehicle commands to gazebo ros_control plugin
    pub_cmd_front_left_steer.publish(cmd_front_left_steer);
    pub_cmd_front_right_steer.publish(cmd_front_right_steer);
    pub_cmd_rear_left_steer.publish(cmd_rear_left_steer);
    pub_cmd_rear_right_steer.publish(cmd_rear_right_steer);

    pub_cmd_front_left_rotor.publish(cmd_front_left_rotor);
    pub_cmd_front_right_rotor.publish(cmd_front_right_rotor);
    pub_cmd_rear_left_rotor.publish(cmd_rear_left_rotor);
    pub_cmd_rear_right_rotor.publish(cmd_rear_right_rotor);
}

} // namespace gazebo
