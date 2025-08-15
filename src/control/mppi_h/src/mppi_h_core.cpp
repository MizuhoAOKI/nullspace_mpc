#include "mppi_h/mppi_h_core.hpp"

namespace controller_mppi_h
{

// constructor
MPPIHybridCore::MPPIHybridCore(std::tuple<param::CommonParam, param::MPPI3DParam, param::MPPI4DParam> param_tuple)
{
    // store param_tuple
    param_tuple_ = param_tuple;

    // instantiate MPPI Core classes and store them in tuple
    mppi_core_tuple_ = std::make_tuple(
        new controller_mppi_3d::MPPI3DCore(
            std::get<param::CommonParam>(param_tuple_), // common param
            std::get<param::MPPI3DParam>(param_tuple_)  // mode 1 param
        ),
        new controller_mppi_4d::MPPI4DCore(
            std::get<param::CommonParam>(param_tuple_), // common param
            std::get<param::MPPI4DParam>(param_tuple_)  // mode 2 param
        )    
    );
}

// destructor
MPPIHybridCore::~MPPIHybridCore()
{
    // No Contents
}

// mode selector
int MPPIHybridCore::selectMode(const common_type::XYYaw& current_state, const grid_map::GridMap& distance_error_map, const grid_map::GridMap& ref_yaw_map)
{
    // get tracking error in distance
    double dist_error = distance_error_map.atPosition("distance_error", grid_map::Position(current_state.x, current_state.y), grid_map::InterpolationMethods::INTER_LINEAR);

    // get tracking error in yaw angle
    double ref_yaw = ref_yaw_map.atPosition("ref_yaw", grid_map::Position(current_state.x, current_state.y), grid_map::InterpolationMethods::INTER_NEAREST);
    double yaw_error = std::remainder(current_state.yaw - ref_yaw, 2 * M_PI); // yaw_error is in [-pi, pi]

    // select mode
    if
    (
        dist_error < std::get<param::CommonParam>(param_tuple_).mode_selector.dist_error_threshold &&
        std::abs(yaw_error) < std::get<param::CommonParam>(param_tuple_).mode_selector.yaw_error_threshold 
    )
    {
        // when tracking error is small, select mode 1 (MPPI_3D)
        current_mode_idx_ = MODE_IDX_MPPI_3D;
    }
    else
    {
        // when tracking error is large, select mode 2 (MPPI_4D)
        current_mode_idx_ = MODE_IDX_MPPI_4D;
    }
    return current_mode_idx_;
}

// mppi solver
common_type::VxVyOmega MPPIHybridCore::solveMPPI(
    const common_type::XYYaw& observed_state,
    const grid_map::GridMap& collision_costmap,
    const grid_map::GridMap& distance_error_map,
    const grid_map::GridMap& ref_yaw_map,
    const common_type::XYYaw& goal_state
)
{
    // select mode
    int mode = selectMode(observed_state, distance_error_map, ref_yaw_map);

    // calculate optimal control command
    if (mode == MODE_IDX_MPPI_3D)
    {
        // [mode 1] MPPI_3D
        common_type::VxVyOmega optimal_vxvyw_cmd_mppi_3d = 
            std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->solveMPPI(
                observed_state,
                collision_costmap,
                distance_error_map,
                ref_yaw_map,
                goal_state
        );

        // get optimal control sequence from MPPI_3D and give it to MPPI_4D.
        auto u_opt_seq_vxvyomega_ = std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getOptimalVxVyOmegaSequence();
        std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->setOptimalVxVyOmegaSequence(u_opt_seq_vxvyomega_);

        // return optimal control command
        return optimal_vxvyw_cmd_mppi_3d;
    }
    else if (mode == MODE_IDX_MPPI_4D)
    {
        // [mode 2] MPPI_4D
        common_type::VxVyOmega optimal_vxvyw_cmd_mppi_4d =
            std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->solveMPPI(
                observed_state,
                collision_costmap,
                distance_error_map,
                ref_yaw_map,
                goal_state
        );

        // get optimal control sequence from MPPI_4D and give it to MPPI_3D.
        auto u_opt_seq_vxvyomega_ = std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getOptimalVxVyOmegaSequence();
        std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->setOptimalVxVyOmegaSequence(u_opt_seq_vxvyomega_);

        // return optimal control command
        return optimal_vxvyw_cmd_mppi_4d;
    }
    else
    {
        // [invalid mode] announce error message
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << mode << std::endl;

        // return stop command
        common_type::VxVyOmega stop_vxvyw_cmd;
        stop_vxvyw_cmd.setZero();
        return stop_vxvyw_cmd;
    }
}

// get calc time [ms]
float MPPIHybridCore::getCalcTime()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getCalcTime();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getCalcTime();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return -99.99;
    }
}

// get state cost of the latest optimal trajectory
double MPPIHybridCore::getStateCost()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getStateCost();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getStateCost();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return 0.0;
    }
}

// get controller name
std::string MPPIHybridCore::getControllerName()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getControllerName();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getControllerName();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return "invalid mode";
    }
}

// check if the vehicle is reached to the goal
bool MPPIHybridCore::isGoalReached()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->isGoalReached();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->isGoalReached();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return false;
    }
}

// get optimal vehicle command (8DoF)
common_type::VehicleCommand8D MPPIHybridCore::getOptimalVehicleCommand()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getOptimalVehicleCommand();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getOptimalVehicleCommand();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return common_type::VehicleCommand8D();
    }
}

// get optimal trajectory
std::vector<common_type::XYYaw> MPPIHybridCore::getOptimalTrajectory()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getOptimalTrajectory();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getOptimalTrajectory();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return std::vector<common_type::XYYaw>();
    }
}

// get full sampled trajectories
StateSeqSamples MPPIHybridCore::getFullSampledTrajectories()
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getFullSampledTrajectories();
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getFullSampledTrajectories();
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return StateSeqSamples();
    }
}

// get elite sampled trajectories
StateSeqSamples MPPIHybridCore::getEliteSampledTrajectories(int elite_sample_size)
{
    if (current_mode_idx_ == MODE_IDX_MPPI_3D)
    {
        return std::get<controller_mppi_3d::MPPI3DCore*>(mppi_core_tuple_)->getEliteSampledTrajectories(elite_sample_size);
    }
    else if (current_mode_idx_ == MODE_IDX_MPPI_4D)
    {
        return std::get<controller_mppi_4d::MPPI4DCore*>(mppi_core_tuple_)->getEliteSampledTrajectories(elite_sample_size);
    }
    else
    {
        std::cerr << "[MPPIHybridCore] invalid mode selected: " << current_mode_idx_ << std::endl;
        return StateSeqSamples();
    }
}

} // namespace controller_mppi_h