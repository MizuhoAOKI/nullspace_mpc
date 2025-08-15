# pragma once

#include <omp.h>
#include <chrono>
#include <iostream>
#include <array>
#include <Eigen/Dense>
#include <random>
#include <grid_map_core/GridMap.hpp>
#include "common_type.hpp"
#include "common_param.hpp"
#include "mode1_mppi_3d/param.hpp"
#include "mode1_mppi_3d/mppi_3d_setting.hpp"

namespace controller_mppi_3d
{
// type definitions
using Samples = std::vector<double>;
using RankOfSamples = std::vector<int>;
using State = target_system_mppi_3d::StateSpace3D;
using StateSeq = std::vector<State>;
using StateSeqSamples = std::vector<StateSeq>;
using Control = target_system_mppi_3d::ControlSpace3D;
using ControlSeq = std::vector<Control>;
using ControlSeqSamples = std::vector<ControlSeq>;

// MPPI core class independent of ROS
class MPPI3DCore
{
    public:
        MPPI3DCore(param::CommonParam& param_common, param::MPPI3DParam& param);
        ~MPPI3DCore();
        common_type::VxVyOmega solveMPPI(
            const common_type::XYYaw& observed_state,
            const grid_map::GridMap& collision_costmap,
            const grid_map::GridMap& distance_error_map,
            const grid_map::GridMap& ref_yaw_map,
            const common_type::XYYaw& goal_state
        );
        // accessors
        float getCalcTime();
        double getStateCost();
        std::string getControllerName();
        bool isGoalReached();
        common_type::VehicleCommand8D getOptimalVehicleCommand();
        std::vector<common_type::XYYaw> getOptimalTrajectory();
        StateSeqSamples getFullSampledTrajectories();
        StateSeqSamples getEliteSampledTrajectories(int elite_sample_size);

        // for connection with MPPI-H
        param::MPPI3DParam getParam() { return param_; }
        std::vector<common_type::VxVyOmega> getOptimalVxVyOmegaSequence();
        void setOptimalVxVyOmegaSequence(std::vector<common_type::VxVyOmega>& u_opt_seq);

    private:
        // mppi params and functions
        param::MPPI3DParam param_;
        int K, T, XDIM, UDIM;
        float calc_time_; // mppi calculation time [ms]
        double state_cost_;
        bool is_goal_reached_;
        Samples costs_;
        RankOfSamples costs_rank_;
        Samples weights_;
        StateSeq x_opt_seq_;
        StateSeqSamples x_samples_;
        Samples calcWeightsOfSamples(const Samples& costs);
        Control u_opt_latest_;
        ControlSeq u_opt_seq_latest_;
        ControlSeqSamples u_samples_;
        ControlSeqSamples noises_;
        ControlSeq sigma_;
        ControlSeqSamples generateNoiseMatrix(ControlSeq& sigma);

        // for random number generation
        const int random_seed_ = 623;
        std::mt19937 psedo_random_engine_;

        // for savisky-golay filter
        int SG_FILTER_WINDOW_SIZE_, SG_FILTER_HALF_WINDOW_SIZE_, SG_FILTER_POLY_ORDER_;
        double SG_FILTER_DELTA_;
        Eigen::MatrixXd savisky_golay_coeffs_;
        ControlSeq u_log_seq_for_filter_; // (T-1) elements of the past u log; u_{-(T-1)}, u_{-(T-2)}, ..., u_{-1}
        void initSaviskyGolayFilter(
            const int half_window_size,
            const unsigned int poly_order,
            const double delta
        );
        Eigen::MatrixXd calcSaviskyGolayCoeffs(
            const int half_window_size, 
            const unsigned int poly_order, 
            const double delta
        );
        Control applySaviskyGolayFilter(ControlSeq& u_seq);
};
} // namespace controller_mppi_3d
