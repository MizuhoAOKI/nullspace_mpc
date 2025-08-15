#pragma once

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>
#include "mppi_3d/common_type.hpp"
#include "mppi_3d/param.hpp"

namespace target_system
{

// define 3D state space
using StateSpace3D = common_type::XYYaw; // check definition of XYYaw in common_type.hpp
static constexpr int DIM_STATE_SPACE = 3;
//// conversion function from XYYaw to StateSpace3D
inline common_type::XYYaw convertXYYawToStateSpace3D(const StateSpace3D& state)
{
    return state; // in mppi_3d, StateSpace3D is equal to XYYaw
}
//// conversion function from StateSpace3D to XYYaw
inline StateSpace3D convertStateSpace3DToXYYaw(const common_type::XYYaw& state)
{
    return state; // in mppi_3d, StateSpace3D is equal to XYYaw
}

// define 3D control input space
using ControlSpace3D = common_type::VxVyOmega; // check definition of VxVyOmega in common_type.hpp
static constexpr int DIM_CONTROL_SPACE = 3;
//// conversion function from ControlSpace3D to VxVyOmega
inline common_type::VxVyOmega convertControlSpace3DToVxVyOmega(const ControlSpace3D& cmd)
{
    return cmd; // in mppi_3d, ControlSpace3D is equal to VxVyOmega
}

// 8DoF vehicle command space
using ControlSpace8D = common_type::VehicleCommand8D; // check definition of VehicleCommand8D in common_type.hpp
static constexpr int DIM_VEHICLE_COMMAND_SPACE = 8;
//// conversion function from ControlSpace3D to ControlSpace8D
inline ControlSpace8D convertControlSpace3DToControlSpace8D(const ControlSpace3D& cmd, const param::Param& param)
{
    ControlSpace8D cmd8d;
    cmd8d.steer_fl = std::atan2(cmd.vy + param.target_system.l_f * cmd.omega, cmd.vx - param.target_system.d_l * cmd.omega);
    cmd8d.steer_fr = std::atan2(cmd.vy + param.target_system.l_f * cmd.omega, cmd.vx + param.target_system.d_r * cmd.omega);
    cmd8d.steer_rl = std::atan2(cmd.vy - param.target_system.l_r * cmd.omega, cmd.vx - param.target_system.d_l * cmd.omega);
    cmd8d.steer_rr = std::atan2(cmd.vy - param.target_system.l_r * cmd.omega, cmd.vx + param.target_system.d_r * cmd.omega);
    cmd8d.rotor_fl = std::sqrt(std::pow(cmd.vx - param.target_system.d_l * cmd.omega, 2) + std::pow(cmd.vy + param.target_system.l_f * cmd.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_fr = std::sqrt(std::pow(cmd.vx + param.target_system.d_r * cmd.omega, 2) + std::pow(cmd.vy + param.target_system.l_f * cmd.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_rl = std::sqrt(std::pow(cmd.vx - param.target_system.d_l * cmd.omega, 2) + std::pow(cmd.vy - param.target_system.l_r * cmd.omega, 2)) / param.target_system.tire_radius;
    cmd8d.rotor_rr = std::sqrt(std::pow(cmd.vx + param.target_system.d_r * cmd.omega, 2) + std::pow(cmd.vy - param.target_system.l_r * cmd.omega, 2)) / param.target_system.tire_radius;
    return cmd8d;
}

// define state updating rule
inline StateSpace3D calcNextState(const StateSpace3D& current_state, const ControlSpace3D& cmd, const double dt)
{
    // clamp control input
    ControlSpace3D clamped_cmd = cmd;
    clamped_cmd.clamp();

    // calculate next state
    StateSpace3D next_state;
    next_state.x = current_state.x + clamped_cmd.vx * std::cos(current_state.yaw) * dt - clamped_cmd.vy * std::sin(current_state.yaw) * dt;
    next_state.y = current_state.y + clamped_cmd.vx * std::sin(current_state.yaw) * dt + clamped_cmd.vy * std::cos(current_state.yaw) * dt;
    next_state.yaw = current_state.yaw + clamped_cmd.omega * dt;
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
        target_system::ControlSpace3D& control_input,
        target_system::ControlSpace3D& prev_control_input,
        const grid_map::GridMap& collision_costmap,
        const grid_map::GridMap& distance_error_map,
        const grid_map::GridMap& ref_yaw_map,
        const target_system::StateSpace3D& goal_state,
        const param::Param& param
    )
    {
        // clamp control input
        control_input.clamp();
        prev_control_input.clamp();

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
                current_vel << control_input.vx, control_input.vy;
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
        Eigen::Matrix<double, 1, target_system::DIM_CONTROL_SPACE> weight_command_change(param.controller.weight_cmd_change.data());
        cost += weight_command_change * (control_input.eigen() - prev_control_input.eigen()).cwiseAbs2();

        // penalize large vehicle command change (steer_fl, steer_fr, steer_rl, steer_rr, rotor_fl, rotor_fr, rotor_rl, rotor_rr)
        target_system::ControlSpace8D vehicle_command = target_system::convertControlSpace3DToControlSpace8D(control_input, param);
        target_system::ControlSpace8D prev_vehicle_command = target_system::convertControlSpace3DToControlSpace8D(prev_control_input, param);
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

