# pragma once

#include <array>
#include <tuple>
#include <grid_map_core/GridMap.hpp>
#include "common_type.hpp"
#include "common_param.hpp"
#include "mode1_mppi_3d/param.hpp"
#include "mode2_mppi_4d/param.hpp"
#include "mode1_mppi_3d/mppi_3d_core.hpp"
#include "mode2_mppi_4d/mppi_4d_core.hpp"

namespace controller_mppi_h
{
// type definitions
using Samples = std::vector<double>;
using RankOfSamples = std::vector<int>;
using State = common_type::XYYaw;
using StateSeq = std::vector<State>;
using StateSeqSamples = std::vector<StateSeq>;
using Control = common_type::VxVyOmega;
using ControlSeq = std::vector<Control>;
using ControlSeqSamples = std::vector<ControlSeq>;

class MPPIHybridCore
{
    public:
        // get tuple of param_common, param_mode1, param_mode2
        MPPIHybridCore(std::tuple<param::CommonParam, param::MPPI3DParam, param::MPPI4DParam> param_tuple);
        ~MPPIHybridCore();
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

    private:
        // for mode selector
        int current_mode_idx_ = 1; // initial mode is MPPI_3D
        constexpr static int MODE_IDX_MPPI_3D = 1;
        constexpr static int MODE_IDX_MPPI_4D = 2;
        int selectMode(const common_type::XYYaw& current_state, const grid_map::GridMap& distance_error_map, const grid_map::GridMap& ref_yaw_map);

        // parameter storage
        std::tuple<param::CommonParam, param::MPPI3DParam, param::MPPI4DParam> param_tuple_;
        std::tuple<controller_mppi_3d::MPPI3DCore*, controller_mppi_4d::MPPI4DCore*> mppi_core_tuple_;
};
} // namespace controller_mppi_h