#include "mode2_mppi_4d/mppi_4d_core.hpp"

namespace controller_mppi_4d
{

// constructor
MPPI4DCore::MPPI4DCore(param::CommonParam& param_common, param::MPPI4DParam& param)
{
    // save parameters
    //// save mode specific parameters
    param_ = param;
    //// save common parameters
    param_.navigation.xy_goal_tolerance = param_common.navigation.xy_goal_tolerance;
    param_.navigation.yaw_goal_tolerance = param_common.navigation.yaw_goal_tolerance;
    param_.target_system.l_f = param_common.target_system.l_f;
    param_.target_system.l_r = param_common.target_system.l_r;
    param_.target_system.d_l = param_common.target_system.d_l;
    param_.target_system.d_r = param_common.target_system.d_r;
    param_.target_system.tire_radius = param_common.target_system.tire_radius;
    param_.controller.control_interval = param_common.controller.control_interval;
    param_.controller.prediction_horizon = param_common.controller.prediction_horizon;
    param_.controller.step_len_sec = param_common.controller.step_len_sec;

    // load parameters
    K = param_.controller.num_samples;
    T = param_.controller.prediction_horizon;
    XDIM = target_system_mppi_4d::DIM_STATE_SPACE;
    UDIM = target_system_mppi_4d::DIM_CONTROL_SPACE;

    // initialize variables for mppi calculation
    costs_ = Samples(K, 0.0); // size is (K)
    costs_rank_ = RankOfSamples(K, 0); // size is (K)
    weights_ = Samples(K, 0.0); // size is (K)
    x_opt_seq_ = StateSeq(T, State()); // size is (T, XDIM)
    x_samples_ = StateSeqSamples(K, StateSeq(T, State())); // size is (K, T, XDIM)
    u_opt_latest_ = Control(); // size is (UDIM)
    u_opt_seq_latest_ = ControlSeq(T, Control()); // size is (T, UDIM)
    u_samples_ = ControlSeqSamples(K, ControlSeq(T, Control())); // size is (K, T, UDIM)
    noises_ = ControlSeqSamples(K, ControlSeq(T, Control())); // size is (K, T, UDIM)
    sigma_ = ControlSeq(T, Control()); // size is (T, UDIM)

    // initialize sigma_
    for (int t = 0; t < T; t++)
    {
        for (int u = 0; u < UDIM; u++)
        {
            sigma_[t][u] = param_.controller.sigma[u];
        }
    }

    // initialize pseudo random engine
    psedo_random_engine_.seed(random_seed_);

    // generate noise matrix
    noises_ = generateNoiseMatrix(sigma_);

    // initialize savisky-golay filter (i.e. calculate savisky-golay filter coefficients)
    if (param_.controller.use_sg_filter)
    {
        initSaviskyGolayFilter(
            param_.controller.sg_filter_half_window_size,
            param_.controller.sg_filter_poly_order,
            param_.controller.step_len_sec
        );
    }
}

// destructor
MPPI4DCore::~MPPI4DCore()
{
    // No Contents
}

// mppi solver
common_type::VxVyOmega MPPI4DCore::solveMPPI(
    const common_type::XYYaw& observed_state,
    const grid_map::GridMap& collision_costmap,
    const grid_map::GridMap& distance_error_map,
    const grid_map::GridMap& ref_yaw_map,
    const common_type::XYYaw& goal_state
)
{
    // check if the vehicle is close to the goal
    if( 
        std::sqrt( pow(goal_state.x - observed_state.x, 2) + pow(goal_state.y - observed_state.y, 2) ) < param_.navigation.xy_goal_tolerance &&
        std::abs(std::remainder(observed_state.yaw - goal_state.yaw, 2 * M_PI)) < param_.navigation.yaw_goal_tolerance
    )
    {
        // return zero velocity command
        common_type::VxVyOmega stop_vxvyw_cmd;
        stop_vxvyw_cmd.setZero();
        is_goal_reached_ = true;
        return stop_vxvyw_cmd;
    }
    else
    {
        is_goal_reached_ = false;
    }

    // initialization
    costs_ = Samples(K, 0.0); // initialize costs_ to 0.0

    // generate noise matrix, skipping this process if reduce_computation is true
    if (!param_.controller.reduce_computation)
    {
        noises_ = generateNoiseMatrix(sigma_);
    }

    // initialize timer to measure mppi calculation time [ms]
    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now(); // start timer

    // [CPU Acceleration with OpenMP]
    #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(1)
    for (int k = 0; k < K; k++)
    {
        // initialize state
        State x = target_system_mppi_4d::convertXYYawToStateSpace3D(observed_state); // in mppi_4d, StateSpace3D is equal to XYYaw
        x_samples_[k][0] = x; // save x_samples

        for (int t = 1; t < T+1; t++)
        {
            // sample control input sequence
            if (k < (1.0-param_.controller.param_exploration)*K){
                // sampling for exploitation
                u_samples_[k][t-1].update(u_opt_seq_latest_[t-1].eigen() + noises_[k][t-1].eigen());
            } else {
                // sampling for exploration
                u_samples_[k][t-1].update(noises_[k][t-1].eigen());
            }

            // update state
            x = target_system_mppi_4d::calcNextState(
                x, u_samples_[k][t-1], param_.controller.step_len_sec, param_
            );
            x_samples_[k][t-1] = x; // save x_samples

            // add stage cost
            Control prev_control_input = (t == 1) ? u_opt_latest_ : u_samples_[k][t-2];
            costs_[k] += controller_mppi_4d::stage_cost(
                    x,
                    u_samples_[k][t-1],
                    prev_control_input,
                    collision_costmap,
                    distance_error_map,
                    ref_yaw_map,
                    goal_state,
                    param_
            );
            costs_[k] += param_.controller.param_lambda * (1.0 - param_.controller.param_alpha) \
             * u_opt_seq_latest_[t-1].eigen().transpose() * (sigma_[t-1].eigen().asDiagonal().inverse()) * u_samples_[k][t-1].eigen();
        }
        // add terminal cost
        costs_[k] += controller_mppi_4d::terminal_cost(x, goal_state, param_);
    }

    // calculate weight for each sample
    weights_ = calcWeightsOfSamples(costs_);

    // calculate optimal control command
    ControlSeq u_opt_seq = u_opt_seq_latest_;
    for (int k = 0; k < K; k++)
    {
        for (int t = 0; t < T; t++)
        {
            u_opt_seq[t].update(u_opt_seq[t].eigen() + weights_[k] * noises_[k][t].eigen());
        }
    }

    // apply savisky-golay filter to get smoothed u_opt_seq[0]
    if (param_.controller.use_sg_filter)
    {
        u_opt_seq[0] = applySaviskyGolayFilter(u_opt_seq);
    }

    // clip control input between umin and umax
    for (int t = 0; t < T; t++)
    {
        u_opt_seq[t].clamp();
    }

    // get mppi calculation time [ms]
    end = std::chrono::system_clock::now();  // stop timer
    calc_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

    // calculate and save optimal state trajectory
    state_cost_ = 0.0;
    x_opt_seq_[0] = target_system_mppi_4d::convertXYYawToStateSpace3D(observed_state);
    for (int t = 1; t < T; t++)
    {
        x_opt_seq_[t] = target_system_mppi_4d::calcNextState(
            x_opt_seq_[t-1], u_opt_seq[t-1], param_.controller.step_len_sec, param_
        );

        // add stage cost
        Control prev_control_input = (t == 1) ? u_opt_latest_ : u_opt_seq_latest_[t-2];
        state_cost_ += controller_mppi_4d::stage_cost(
                x_opt_seq_[t],
                u_opt_seq[t-1],
                prev_control_input,
                collision_costmap,
                distance_error_map,
                ref_yaw_map,
                goal_state,
                param_
        );
        state_cost_ += param_.controller.param_lambda * (1.0 - param_.controller.param_alpha) \
            * u_opt_seq_latest_[t-1].eigen().transpose() * (sigma_[t-1].eigen().asDiagonal().inverse()) * u_opt_seq[t-1].eigen();
    }
    // add terminal cost
    state_cost_ += controller_mppi_4d::terminal_cost(x_opt_seq_[T-1], goal_state, param_);

    // convert optimal control command to VxVyOmega
    common_type::VxVyOmega optimal_vxvyw_cmd = target_system_mppi_4d::convertControlSpace4DToVxVyOmega(u_opt_seq[0], param_);
    u_opt_latest_ = u_opt_seq[0]; // update u_opt_latest_
    u_opt_seq_latest_ = u_opt_seq; // update u_opt_seq_latest_
    return optimal_vxvyw_cmd;
}

// generate noise matrix whose size is K x T x UDIM
//   K    : number of samples
//   T    : prediction horizon stepscost = c_terminal(x)
//   UDIM : control space dimension
//   sigma[t][u] : noise parameter (variance of normal distribution) at time t and control space dimension u
ControlSeqSamples MPPI4DCore::generateNoiseMatrix(ControlSeq& sigma)
{
    // declare noise matrix
    ControlSeqSamples noises = ControlSeqSamples(K, ControlSeq(T, Control()));

    // set random value to noises, which is normal distribution with mean 0.0 and variance sigma[t][u]
    // [CPU Acceleration with OpenMP]
    #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(3)
    for (int k = 0; k < K; k++)
    {
        for (int t = 0; t < T; t++)
        {
            for (int u = 0; u < UDIM; u++)
            {
                std::normal_distribution<double> normal_dist(0.0, sigma[t][u]);
                noises[k][t][u] = normal_dist(psedo_random_engine_);
            }
        }
    }

    return noises;
}

// calculate weight for each sample 
Samples MPPI4DCore::calcWeightsOfSamples(const Samples& costs)
{
    // initialize weights
    Samples weights = Samples(K, 0.0);

    // get the minimum cost of all samples
    double min_cost = *std::min_element(costs.begin(), costs.end());

    // calculate eta
    // [CPU Acceleration with OpenMP]
    double eta = 0.0;
    // #pragma omp parallel for num_threads(omp_get_max_threads())
    for (int k = 0; k < K; k++)
    {
        eta += std::exp( (-1.0/ param_.controller.param_lambda) * (costs[k] - min_cost) );
    }

    // calculate weight for each sample
    // #pragma omp parallel for num_threads(omp_get_max_threads())
    for (int k = 0; k < K; k++)
    {
        weights[k] = (1.0 / eta) * std::exp( (-1.0/ param_.controller.param_lambda) * (costs[k] - min_cost) );
    }

    // update ranking of costs // 1th: best (i.e. minimum cost), K: worst (i.e. maximum cost)
    std::iota(costs_rank_.begin(), costs_rank_.end(), 0); // initialize costs_rank_ with 0, 1, 2, ..., K-1
    std::sort(costs_rank_.begin(), costs_rank_.end(), [&](int i, int j) { return costs_[i] < costs_[j]; }); // sort costs_rank_ based on costs_ value
    // Note: best (minimum) cost is costs_[costs_rank_[0]], worst (maximum) cost is costs_[costs_rank_[K-1]]

    return weights;
}

// initialize savisky-golay filter
void MPPI4DCore::initSaviskyGolayFilter(
    const int half_window_size,
    const unsigned int poly_order,
    const double delta
)
{
    // load parameters and initialize variables for savisky-golay filter
    SG_FILTER_HALF_WINDOW_SIZE_ = half_window_size; // up to T-1
    //// raise error if SG_FILTER_HALF_WINDOW_SIZE_ is larger than T-1
    if (SG_FILTER_HALF_WINDOW_SIZE_ > T-1)
    {
        throw std::invalid_argument("SG_FILTER_HALF_WINDOW_SIZE_ must be less than or equal to (prediction_horizon)-1.");
    }
    SG_FILTER_WINDOW_SIZE_ = 2 * SG_FILTER_HALF_WINDOW_SIZE_ + 1; // HALF_WINDOW(past u log) | u_opt_seq[0] | HALF_WINDOW(u prediction)
    SG_FILTER_POLY_ORDER_ = poly_order;
    SG_FILTER_DELTA_ = delta;
    u_log_seq_for_filter_ = ControlSeq(SG_FILTER_HALF_WINDOW_SIZE_, Control()); // size is (SG_FILTER_HALF_WINDOW_SIZE_, UDIM)

    // calculate and save savisky-golay filter coefficients (you need to call this function only once)
    savisky_golay_coeffs_ = calcSaviskyGolayCoeffs(
        SG_FILTER_HALF_WINDOW_SIZE_,
        SG_FILTER_POLY_ORDER_,
        SG_FILTER_DELTA_
    );
}

// calculate savisky-golay filter coefficients
// reference: https://github.com/Izadori/cpp_eigen/blob/main/savgol/savgol.cpp
Eigen::MatrixXd MPPI4DCore::calcSaviskyGolayCoeffs(
    const int half_window_size, 
    const unsigned int poly_order, 
    const double delta
)
{
    // target data : y_{-n} ... y_{-1} | y_0 | y_1 ... y_n
    int n = half_window_size;
    int window_size = 2 * n + 1;

    // generate matrices
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(window_size, -n, n);
    Eigen::MatrixXd x = Eigen::MatrixXd::Ones(window_size, poly_order + 1);
    for(unsigned int i = 1; i <= poly_order; i++){
      x.col(i) = (x.col(i - 1).array() * v.array()).matrix();
    }

    // get (X^T * X)^-1 * X^T
    Eigen::MatrixXd coeff_mat = (x.transpose() * x).inverse() * x.transpose();

    // return a0 coefficients
    return coeff_mat.row(0).transpose();
}

// apply savisky-golay filter to get smoothed u_opt_seq[0]
Control MPPI4DCore::applySaviskyGolayFilter(ControlSeq& u_opt_seq)
{
    // initialize filtered control input
    Control u_opt_filtered;
    u_opt_filtered.setZero();

    // apply savisky-golay filter
    for (int i = 0; i < SG_FILTER_WINDOW_SIZE_; i++)
    {
        if (i < SG_FILTER_HALF_WINDOW_SIZE_)
        {
            u_opt_filtered.update(u_opt_filtered.eigen() + savisky_golay_coeffs_(i) * u_log_seq_for_filter_[i].eigen());
        }
        else
        {
            u_opt_filtered.update(u_opt_filtered.eigen() + savisky_golay_coeffs_(i) * u_opt_seq[i-SG_FILTER_HALF_WINDOW_SIZE_].eigen());
        }
    }

    // update u_log_seq_for_filter_ shifting index to the left
    for (int j = 0; j < SG_FILTER_HALF_WINDOW_SIZE_ - 1; j++)
    {
        u_log_seq_for_filter_[j] = u_log_seq_for_filter_[j+1];
    }
    u_log_seq_for_filter_[SG_FILTER_HALF_WINDOW_SIZE_ - 1] = u_opt_filtered; // update the latest element with u_opt_seq[0]

    // return smoothed control input
    return u_opt_filtered;
}

// get calc time [ms]
float MPPI4DCore::getCalcTime()
{
    return calc_time_;
}

// get state cost of the latest optimal trajectory
double MPPI4DCore::getStateCost()
{
    return state_cost_;
}

// get controller name
std::string MPPI4DCore::getControllerName()
{
    return param_.controller.name;
}

// check if the vehicle is reached to the goal
bool MPPI4DCore::isGoalReached()
{
    return is_goal_reached_;
}

// get optimal vehicle command (8DoF)
common_type::VehicleCommand8D MPPI4DCore::getOptimalVehicleCommand()
{
    return target_system_mppi_4d::convertControlSpace4DToControlSpace8D(u_opt_latest_, param_);
}

// return optimal state sequence
std::vector<common_type::XYYaw> MPPI4DCore::getOptimalTrajectory()
{
    std::vector<common_type::XYYaw> optimal_state_sequence;
    for (int t = 0; t < T; t++)
    {
        common_type::XYYaw state = target_system_mppi_4d::convertStateSpace3DToXYYaw(x_opt_seq_[t]);
        optimal_state_sequence.push_back(state);
    }
    return optimal_state_sequence;
}

// return full sampled state sequences
StateSeqSamples MPPI4DCore::getFullSampledTrajectories()
{
    std::vector<std::vector<common_type::XYYaw>> full_sampled_state_sequences;
    for (int k = 0; k < K; k++)
    {
        std::vector<common_type::XYYaw> state_sequence;
        for (int t = 0; t < T; t++)
        {
            common_type::XYYaw state = target_system_mppi_4d::convertStateSpace3DToXYYaw(x_samples_[k][t]);
            state_sequence.push_back(state);
        }
        full_sampled_state_sequences.push_back(state_sequence);
    }
    return full_sampled_state_sequences;
}

// return elite sampled state sequences
StateSeqSamples MPPI4DCore::getEliteSampledTrajectories(int elite_sample_size)
{
    std::vector<std::vector<common_type::XYYaw>> elite_sampled_state_sequences;
    for (int k = 0; k < elite_sample_size; k++)
    {
        std::vector<common_type::XYYaw> state_sequence;
        for (int t = 0; t < T; t++)
        {
            common_type::XYYaw state = target_system_mppi_4d::convertStateSpace3DToXYYaw(x_samples_[costs_rank_[k]][t]);
            state_sequence.push_back(state);
        }
        elite_sampled_state_sequences.push_back(state_sequence);
    }
    return elite_sampled_state_sequences;
}

// get optimal control input sequence
std::vector<common_type::VxVyOmega> MPPI4DCore::getOptimalVxVyOmegaSequence()
{
    std::vector<common_type::VxVyOmega> optimal_vxvyw_sequence;
    for (int t = 0; t < T; t++)
    {
        common_type::VxVyOmega vxvyw = target_system_mppi_4d::convertControlSpace4DToVxVyOmega(u_opt_seq_latest_[t], param_);
        optimal_vxvyw_sequence.push_back(vxvyw);
    }
    return optimal_vxvyw_sequence;
}

// set optimal control input sequence
void MPPI4DCore::setOptimalVxVyOmegaSequence(std::vector<common_type::VxVyOmega>& u_opt_seq)
{
    // save optimal control sequence
    for (int t = 0; t < T; t++)
    {
        u_opt_seq_latest_[t] = target_system_mppi_4d::convertVxVyOmegaToControlSpace4D(u_opt_seq[t], param_);
    }

    // apply savisky-golay filter to get smoothed u_opt_seq[0]
    if (param_.controller.use_sg_filter)
    {
        u_opt_seq_latest_[0] = applySaviskyGolayFilter(u_opt_seq_latest_);
    }

    // clip control input between umin and umax
    for (int t = 0; t < T; t++)
    {
        u_opt_seq_latest_[t].clamp();
    }

    // update u_opt_latest_
    u_opt_latest_ = u_opt_seq_latest_[0];
}

} // namespace controller_mppi_4d