#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>
#include "mppi_4d/common_type.hpp"
#include "mppi_4d/param.hpp"

namespace target_system
{

// define 3D state space
using StateSpace3D = common_type::XYYaw; // check definition of XYYaw in common_type.hpp
static constexpr int DIM_STATE_SPACE = 3;
//// conversion function from XYYaw to StateSpace3D
inline common_type::XYYaw convertXYYawToStateSpace3D(const StateSpace3D& state)
{
    return state; // in mppi_4d, StateSpace3D is equal to XYYaw
}
//// conversion function from StateSpace3D to XYYaw
inline StateSpace3D convertStateSpace3DToXYYaw(const common_type::XYYaw& state)
{
    return state; // in mppi_4d, StateSpace3D is equal to XYYaw
}

// define 4D control input space
static constexpr int DIM_CONTROL_SPACE = 4;
struct ControlSpace4D
{
    int dim = 4; // dimension
    double fl_steer; // front left steer angle [rad]
    double rr_steer; // rear right steer angle [rad]
    double fl_vel;   // front left wheel velocity [rad/s]
    double rr_vel;   // rear right wheel velocity [rad/s]

    // set max/min values
    double fl_steer_min = -1.57;
    double fl_steer_max = +1.57;
    double rr_steer_min = -1.57;
    double rr_steer_max = +1.57;
    double fl_vel_min = -2.0;
    double fl_vel_max = +2.0;
    double rr_vel_min = -2.0;
    double rr_vel_max = +2.0;

    // clamp control input
    void clamp()
    {
        fl_steer = std::max(fl_steer_min, std::min(fl_steer_max, fl_steer));
        rr_steer = std::max(rr_steer_min, std::min(rr_steer_max, rr_steer));
        fl_vel = std::max(fl_vel_min, std::min(fl_vel_max, fl_vel));
        rr_vel = std::max(fl_vel_min, std::min(fl_vel_max, rr_vel));
    }

    // return value as eigen matrix
    Eigen::Matrix<double, 4, 1> eigen()
    {
        Eigen::Matrix<double, 4, 1> vector;
        vector << fl_steer, rr_steer, fl_vel, rr_vel;
        return vector;
    }

    // update values with eigen matrix
    void update(const Eigen::Matrix<double, 4, 1>& vec)
    {
        fl_steer = vec(0, 0);
        rr_steer = vec(1, 0);
        fl_vel = vec(2, 0);
        rr_vel = vec(3, 0);
    }

    // initialize control input
    void setZero()
    {
        fl_steer = 0.0;
        rr_steer = 0.0;
        fl_vel = 0.0;
        rr_vel = 0.0;
    }

    // definition of iterator
    double& operator[](int idx)
    {
        switch (idx)
        {
        case 0:
            return fl_steer;
        case 1:
            return rr_steer;
        case 2:
            return fl_vel;
        case 3:
            return rr_vel;
        default:
            throw std::out_of_range("Index out of range");
        }
    }
};
//// conversion function from ControlSpace4D to VxVyOmega
inline common_type::VxVyOmega convertControlSpace4DToVxVyOmega(const ControlSpace4D& cmd4d, const param::Param& param)
{
    common_type::VxVyOmega cmd3d;
    double A = cmd4d.fl_vel * std::cos(cmd4d.fl_steer);
    double B = cmd4d.fl_vel * std::sin(cmd4d.fl_steer);
    double C = cmd4d.rr_vel * std::cos(cmd4d.rr_steer);
    double D = cmd4d.rr_vel * std::sin(cmd4d.rr_steer);
    cmd3d.vx = 0.5 * (A + C);
    cmd3d.vy = 0.5 * (B + D);
    double w_l = (A - cmd3d.vx) / (-param.target_system.d_l);
    double w_r = (C - cmd3d.vx) / (+param.target_system.d_r);
    cmd3d.omega = (w_l + w_r) / 2.0;
    return cmd3d;
}

// 8DoF vehicle command space
using ControlSpace8D = common_type::VehicleCommand8D; // check definition of VehicleCommand8D in common_type.hpp
static constexpr int DIM_VEHICLE_COMMAND_SPACE = 8;
//// conversion function from ControlSpace4D to ControlSpace8D
inline ControlSpace8D convertControlSpace4DToControlSpace8D(const ControlSpace4D& cmd4d, const param::Param& param)
{
    // convert cmd4d to cmd3d
    common_type::VxVyOmega cmd3d = convertControlSpace4DToVxVyOmega(cmd4d, param);

    // convert cmd3d to cmd8d
    ControlSpace8D cmd8d;
    cmd8d.steer_fl = std::atan2(cmd3d.vy + param.target_system.l_f * cmd3d.omega, cmd3d.vx - param.target_system.d_l * cmd3d.omega);
    cmd8d.steer_fr = std::atan2(cmd3d.vy + param.target_system.l_f * cmd3d.omega, cmd3d.vx + param.target_system.d_r * cmd3d.omega);
    cmd8d.steer_rl = std::atan2(cmd3d.vy - param.target_system.l_r * cmd3d.omega, cmd3d.vx - param.target_system.d_l * cmd3d.omega);
    cmd8d.steer_rr = std::atan2(cmd3d.vy - param.target_system.l_r * cmd3d.omega, cmd3d.vx + param.target_system.d_r * cmd3d.omega);
    cmd8d.rotor_fl = std::sqrt(std::pow(cmd3d.vx - param.target_system.d_l * cmd3d.omega, 2) + std::pow(cmd3d.vy + param.target_system.l_f * cmd3d.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_fr = std::sqrt(std::pow(cmd3d.vx + param.target_system.d_r * cmd3d.omega, 2) + std::pow(cmd3d.vy + param.target_system.l_f * cmd3d.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_rl = std::sqrt(std::pow(cmd3d.vx - param.target_system.d_l * cmd3d.omega, 2) + std::pow(cmd3d.vy - param.target_system.l_r * cmd3d.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_rr = std::sqrt(std::pow(cmd3d.vx + param.target_system.d_r * cmd3d.omega, 2) + std::pow(cmd3d.vy - param.target_system.l_r * cmd3d.omega, 2)) / param.target_system.tire_radius;
    return cmd8d;
}

// define state updating rule
inline StateSpace3D calcNextState(const StateSpace3D& current_state, const ControlSpace4D& cmd4d, const double dt, const param::Param& param)
{
    // clamp control input
    ControlSpace4D clamped_input4d = cmd4d;
    clamped_input4d.clamp();

    // convert ControlSpace4D to VxVyOmega
    common_type::VxVyOmega clamped_input3d = convertControlSpace4DToVxVyOmega(clamped_input4d, param);

    // calculate next state
    StateSpace3D next_state;
    next_state.x = current_state.x + clamped_input3d.vx * std::cos(current_state.yaw) * dt - clamped_input3d.vy * std::sin(current_state.yaw) * dt;
    next_state.y = current_state.y + clamped_input3d.vx * std::sin(current_state.yaw) * dt + clamped_input3d.vy * std::cos(current_state.yaw) * dt;
    next_state.yaw = current_state.yaw + clamped_input3d.omega * dt;
    next_state.unwrap(); // unwrap yaw angle

    // return next state
    return next_state;
}

} // namespace target_system

namespace controller
{

    // stage cost function
    inline double stage_cost(
        const target_system::StateSpace3D& state,
        target_system::ControlSpace4D& control_input4d,
        target_system::ControlSpace4D& prev_control_input4d,
        const grid_map::GridMap& collision_costmap,
        const grid_map::GridMap& distance_error_map,
        const grid_map::GridMap& ref_yaw_map,
        const target_system::StateSpace3D& goal_state,
        const param::Param& param
    )
    {
        // clamp control input
        control_input4d.clamp();
        prev_control_input4d.clamp();

        // convert ControlSpace4D to VxVyOmega
        common_type::VxVyOmega control_input3d = target_system::convertControlSpace4DToVxVyOmega(control_input4d, param);
        common_type::VxVyOmega prev_control_input3d = target_system::convertControlSpace4DToVxVyOmega(prev_control_input4d, param);

        // initialize stage cost
        double cost = 0.0;

        // only when the vehicle is not close to the goal
        if( std::sqrt( pow(goal_state.x - state.x, 2) + pow(goal_state.y - state.y, 2) ) > param.navigation.xy_goal_tolerance )
        {
            // track target velocity (considering only aligned component to the reference path)
            if (ref_yaw_map.isInside(grid_map::Position(state.x, state.y)))
            {
                double ref_yaw = ref_yaw_map.atPosition("ref_yaw", grid_map::Position(state.x, state.y), grid_map::InterpolationMethods::INTER_NEAREST);
                double diff_yaw = std::remainder(state.yaw - ref_yaw, 2 * M_PI); // diff_yaw is in [-pi, pi]
                Eigen::Matrix<double, 2, 1> ref_vel_direction;
                ref_vel_direction << std::cos(diff_yaw), std::sin(diff_yaw);
                Eigen::Matrix<double, 2, 1> current_vel;
                current_vel << control_input3d.vx, control_input3d.vy;
                double ref_aligned_vel = ref_vel_direction.dot(current_vel);
                cost += param.controller.weight_velocity_error * pow(ref_aligned_vel - param.controller.ref_velocity, 2);
            }

            // try to align with the reference path
            if (ref_yaw_map.isInside(grid_map::Position(state.x, state.y)))
            {
                double ref_yaw = ref_yaw_map.atPosition("ref_yaw", grid_map::Position(state.x, state.y), grid_map::InterpolationMethods::INTER_NEAREST);
                double diff_yaw = std::remainder(state.yaw - ref_yaw, 2 * M_PI); // diff_yaw is in [-pi, pi]
                cost += param.controller.weight_angular_error * diff_yaw * diff_yaw;
            }
        }

        // avoid collision
        if (collision_costmap.isInside(grid_map::Position(state.x, state.y)))
        {
            cost += param.controller.weight_collision_penalty * collision_costmap.atPosition("collision_cost", grid_map::Position(state.x, state.y));
        }

        // avoid large distance error from reference path
        if (distance_error_map.isInside(grid_map::Position(state.x, state.y)))
        {
            // change interpolation method more accurate
            double dist_error = distance_error_map.atPosition("distance_error", grid_map::Position(state.x, state.y), grid_map::InterpolationMethods::INTER_LINEAR);
            cost += param.controller.weight_distance_error_penalty * dist_error;
        }

        // penalize large control input change (vx, vy, omega)
        Eigen::Matrix<double, 1, 3> weight_command_change(param.controller.weight_cmd_change.data());
        cost += weight_command_change * (control_input3d.eigen() - prev_control_input3d.eigen()).cwiseAbs2();

        // penalize large vehicle command change (steer_fl, steer_fr, steer_rl, steer_rr, rotor_fl, rotor_fr, rotor_rl, rotor_rr)
        target_system::ControlSpace8D vehicle_command = target_system::convertControlSpace4DToControlSpace8D(control_input4d, param);
        target_system::ControlSpace8D prev_vehicle_command = target_system::convertControlSpace4DToControlSpace8D(prev_control_input4d, param);
        Eigen::Matrix<double, 1, target_system::DIM_VEHICLE_COMMAND_SPACE> weight_vehicle_command_change(param.controller.weight_vehicle_cmd_change.data());
        cost += weight_vehicle_command_change * (vehicle_command.eigen() - prev_vehicle_command.eigen()).cwiseAbs2();

        return cost;
    }

    // terminal cost function
    inline double terminal_cost(
        const target_system::StateSpace3D& state,
        const target_system::StateSpace3D& goal_state,
        const param::Param& param
    )
    {
        // initialize terminal cost
        double cost = 0.0;

        // only when the vehicle is not close to the goal
        // Note: this cost is needed to avoid the vehicle go far from the goal (i.e. avoid running in the opposite direction along the reference path)
        if( std::sqrt( pow(goal_state.x - state.x, 2) + pow(goal_state.y - state.y, 2) ) > param.navigation.xy_goal_tolerance )
        {
            // get closer to the goal
            cost = param.controller.weight_terminal_state_penalty * ( pow((state.x - goal_state.x), 2) + pow((state.y - goal_state.y), 2) );
        }

        return cost;
    }

} // namespace controller

