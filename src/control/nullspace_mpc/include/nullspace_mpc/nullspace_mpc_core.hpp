# pragma once

#include <omp.h>
#include <chrono>
#include <iostream>
#include <array>
#include <Eigen/Dense>
#include <random>
#include <grid_map_core/GridMap.hpp>
#include <qp_solver_collection/QpSolverCollection.h>
#include "nullspace_mpc/common_type.hpp"
#include "nullspace_mpc/param.hpp"
#include "nullspace_mpc/nullspace_mpc_setting.hpp"
#include "nullspace_mpc/hqp.hpp"
#include "nullspace_mpc/task_sse.hpp"
#include "nullspace_mpc/task_tts.hpp"
#include "nullspace_mpc/task_minvelacc.hpp"

namespace controller
{
// type definitions
using Samples = std::vector<double>;
using RankOfSamples = std::vector<int>;
using ViaState = target_system::StateSpace3D;
using ViaStateSeq = std::vector<ViaState>;
using ViaStateSeqSamples = std::vector<ViaStateSeq>;
using State = target_system::StateSpace3D;
using StateSeq = std::vector<State>;
using StateSeqSamples = std::vector<StateSeq>;
using Control = target_system::ControlSpace3D;
using ControlSeq = std::vector<Control>;
using ControlSeqSamples = std::vector<ControlSeq>;

// MPC core class independent of ROS
class MPCCore
{
    public:
        MPCCore(param::Param& param);
        ~MPCCore();
        common_type::VxVyOmega solveMPC(
            const common_type::XYYaw& observed_state,
            const common_type::VxVyOmega& observed_velocity,
            const grid_map::GridMap& collision_costmap,
            const grid_map::GridMap& distance_error_map,
            const grid_map::GridMap& ref_yaw_map,
            const std::vector<common_type::XYYaw>& via_state_sequence,
            const common_type::XYYaw& global_goal_state
        );
        double wrapAngle(double yaw, double center_yaw);

        // accessors
        float getCalcTime();
        double getStateCost();
        std::string getControllerName();
        bool isGoalReached();
        common_type::VehicleCommand8D getOptimalVehicleCommand();
        std::vector<common_type::XYYaw> getOptimalTrajectory();
        StateSeqSamples getFullSampledTrajectories();
        StateSeqSamples getEliteSampledTrajectories(int elite_sample_size);
        ViaStateSeq getViaStateSequence();

    private:
        // mppi params and functions
        param::Param param_;
        int K, T, VT, XDIM, UDIM, VXDIM;
        double STEP_LEN_SEC;
        std::vector<int> IDX_VIA_STATES;
        float calc_time_; // mppi calculation time [ms]
        double state_cost_;
        bool is_goal_reached_;
        Samples costs_;
        RankOfSamples costs_rank_;
        Samples weights_;
        ViaStateSeq via_state_opt_seq_;
        ViaStateSeqSamples via_state_samples_;
        StateSeq state_opt_seq_;
        StateSeqSamples state_samples_;
        Samples calcWeightsOfSamples(const Samples& costs);
        Control input_opt_latest_;
        ControlSeq input_opt_seq_latest_;
        ControlSeqSamples input_samples_;
        ViaStateSeqSamples noises_;
        ViaStateSeq sigma_;
        ViaStateSeqSamples generateNoiseMatrix(ViaStateSeq& sigma);
        Eigen::VectorXd solveHQP(
            const ViaStateSeq& via_state_seq,
            const common_type::VxVyOmega& global_vel,
            const common_type::XYYaw& observed_state
        );

        // for random number generation
        const int random_seed_ = 623;
        std::mt19937 psedo_random_engine_;

        // for savisky-golay filter
        int SG_FILTER_WINDOW_SIZE_, SG_FILTER_HALF_WINDOW_SIZE_, SG_FILTER_POLY_ORDER_;
        double SG_FILTER_DELTA_;
        Eigen::MatrixXd savisky_golay_coeffs_;
        ControlSeq input_log_seq_for_filter_; // (T-1) elements of the past control input log; u_{-(T-1)}, u_{-(T-2)}, ..., u_{-1}
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
        Control applySaviskyGolayFilter(ControlSeq& input_seq);
};
} // namespace controller
