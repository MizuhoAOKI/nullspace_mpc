#pragma once

#include <string>
#include <vector>

namespace param
{

struct CommonParam
{
    struct Navigation
    {
        double xy_goal_tolerance;
        double yaw_goal_tolerance;
    };
    Navigation navigation;

    struct TargetSystem
    {
        double l_f;
        double l_r;
        double d_l;
        double d_r;
        double tire_radius;
    };
    TargetSystem target_system;

    struct Controller
    {
        double control_interval;
        int prediction_horizon;
        double step_len_sec;
    };
    Controller controller;

    struct ModeSelector
    {
        double yaw_error_threshold;
        double dist_error_threshold;
    };
    ModeSelector mode_selector;
};

} // namespace param
