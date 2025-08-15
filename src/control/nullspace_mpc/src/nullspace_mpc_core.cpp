#include "nullspace_mpc/nullspace_mpc_core.hpp"

namespace controller
{

// constructor
MPCCore::MPCCore(param::Param& param)
{
    // load parameters
    param_ = param;
    K = param.controller.num_samples;
    T = param.controller.prediction_horizon;
    VT = param.controller.idx_via_states.size();
    STEP_LEN_SEC = param.controller.step_len_sec;
    IDX_VIA_STATES = param.controller.idx_via_states;
    UDIM = target_system::DIM_CONTROL_SPACE;
    XDIM = target_system::DIM_STATE_SPACE;
    VXDIM = target_system::DIM_STATE_SPACE;

    // initialize variables for mppi calculation
    costs_ = Samples(K, 0.0); // size is (K)
    costs_rank_ = RankOfSamples(K, 0); // size is (K)
    weights_ = Samples(K, 0.0); // size is (K)
    via_state_opt_seq_ = StateSeq(VT, ViaState()); // size is (VT, VXDIM)
    via_state_samples_ = ViaStateSeqSamples(K, ViaStateSeq(VT, ViaState())); // size is (K, VT, VXDIM)
    state_opt_seq_ = StateSeq(T+1, State()); // size is (T+1, XDIM)
    state_samples_ = StateSeqSamples(K, StateSeq(T+1, State())); // size is (K, T+1, XDIM)
    input_opt_latest_ = Control(); // size is (UDIM)
    input_opt_seq_latest_ = ControlSeq(T, Control()); // size is (T, UDIM)
    input_samples_ = ControlSeqSamples(K, ControlSeq(T, Control())); // size is (K, T, UDIM)
    noises_ = ViaStateSeqSamples(K, ViaStateSeq(VT, ViaState())); // size is (K, T, UDIM)
    sigma_ = ViaStateSeq(VT, ViaState()); // size is (T, UDIM)

    // initialize via_state_opt_seq_
    for (int vt = 0; vt < VT; vt++)
    {
        via_state_opt_seq_[vt].update(Eigen::Matrix<double, 3, 1>(0.0, 0.0, 0.0));
    }

    // initialize sigma_
    for (int vt = 0; vt < VT; vt++)
    {
        for (int d = 0; d < VXDIM; d++)
        {
            sigma_[vt][d] = param.controller.sigma[vt*VXDIM + d];
        }
    }

    // initialize pseudo random engine
    psedo_random_engine_.seed(random_seed_);

    // generate noise matrix
    noises_ = generateNoiseMatrix(sigma_);

    // initialize savisky-golay filter (i.e. calculate savisky-golay filter coefficients)
    if (param.controller.use_sg_filter)
    {
        initSaviskyGolayFilter(
            param.controller.sg_filter_half_window_size,
            param.controller.sg_filter_poly_order,
            param.controller.step_len_sec
        );
    }
}

// destructor
MPCCore::~MPCCore()
{
    // No Contents
}

// mppi solver
common_type::VxVyOmega MPCCore::solveMPC(
    const common_type::XYYaw& observed_state,
    const common_type::VxVyOmega& observed_velocity,
    const grid_map::GridMap& collision_costmap,
    const grid_map::GridMap& distance_error_map,
    const grid_map::GridMap& ref_yaw_map,
    const std::vector<common_type::XYYaw>& via_state_sequence,
    const common_type::XYYaw& global_goal_state
)
{
    // initialize timer to measure mppi calculation time [ms]
    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now(); // start timer

    // if the vehicle is close to the goal, send zero velocity command.
    if( 
        std::sqrt( pow(global_goal_state.x - observed_state.x, 2) + pow(global_goal_state.y - observed_state.y, 2) ) < param_.navigation.xy_goal_tolerance &&
        std::abs(std::remainder(observed_state.yaw - global_goal_state.yaw, 2 * M_PI)) < param_.navigation.yaw_goal_tolerance
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

    // get the previous command velocity
    //// use the latest command velocity
    double global_vx = input_opt_latest_.vx * std::cos(observed_state.yaw) - input_opt_latest_.vy * std::sin(observed_state.yaw);
    double global_vy = input_opt_latest_.vx * std::sin(observed_state.yaw) + input_opt_latest_.vy * std::cos(observed_state.yaw);
    double global_yawrate = input_opt_latest_.omega;
    //// (alternative option) use the observed velocity
    // double global_vx = observed_velocity.vx * std::cos(observed_state.yaw) - observed_velocity.vy * std::sin(observed_state.yaw);
    // double global_vy = observed_velocity.vx * std::sin(observed_state.yaw) + observed_velocity.vy * std::cos(observed_state.yaw);
    // double global_yawrate = observed_velocity.omega;

    common_type::VxVyOmega global_vel;
    global_vel.vx = global_vx;
    global_vel.vy = global_vy;
    global_vel.omega = global_yawrate;

    // generate noise matrix, skipping this process if reduce_computation is true
    if (!param_.controller.reduce_computation)
    {
        noises_ = generateNoiseMatrix(sigma_);
    }

    // update the last element of via_state_opt_seq_ with the via_state_sequence
    for (int vt = 0; vt < VT; vt++)
    {
        via_state_opt_seq_[vt].update(via_state_sequence[vt].eigen());
    }

    // [CPU Acceleration with OpenMP]
    #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(1)
    for (int k = 0; k < K; k++)
    {
        // sample via state sequence
        for (int vt = 0; vt < VT; vt++)
        {
            // reference yaw angle of the target via state is via_state_sequence[vt].yaw
            // transform noises_ to the local frame of the target via state
            double yaw = via_state_sequence[vt].yaw;
            double frenet_x_noise = noises_[k][vt].x * std::cos(yaw) - noises_[k][vt].y * std::sin(yaw);
            double frenet_y_noise = noises_[k][vt].x * std::sin(yaw) + noises_[k][vt].y * std::cos(yaw);
            via_state_samples_[k][vt].update(
                Eigen::Matrix<double, 3, 1>(
                    via_state_opt_seq_[vt].x + frenet_x_noise,
                    via_state_opt_seq_[vt].y + frenet_y_noise,
                    wrapAngle(via_state_opt_seq_[vt].yaw + noises_[k][vt].yaw, observed_state.yaw)
                )
            );
        }

        // solve HQP to get state sequence passing via states
        Eigen::VectorXd x_star_ = solveHQP(
            via_state_samples_[k],
            global_vel,
            observed_state
        );
        // check if x_star_ is valid (i.e. not containing nan)
        if (x_star_.hasNaN())
        {
            costs_[k] = std::numeric_limits<double>::infinity(); // set cost to infinity to reject this sample
            continue; // skip the rest of the loop
        }

        // parse solution
        // save initial state at 0 step
        state_samples_[k][0].update(
            Eigen::Matrix<double, 3, 1>(
                x_star_.segment(0 * (XDIM + UDIM), XDIM)
            )
        );
        for (int t = 1; t < T; t++)
        {
            // save input at t step (input_t s.t. state_{t+1} <- f(state_t, input_t))
            input_samples_[k][t-1].update(
                Eigen::Matrix<double, 3, 1>(
                    x_star_.segment((t-1) * (XDIM + UDIM) + XDIM, UDIM)
                )
            );
            // save state at t step (state_t)
            state_samples_[k][t].update(
                Eigen::Matrix<double, 3, 1>(
                    x_star_.segment(t * (XDIM + UDIM), XDIM)
                )
            );
            // add stage cost
            Control prev_control_input = (t == 1) ? input_opt_latest_ : input_samples_[k][t-2];
            costs_[k] += controller::stage_cost(
                    state_samples_[k][t],
                    input_samples_[k][t-1],
                    prev_control_input,
                    collision_costmap,
                    distance_error_map,
                    ref_yaw_map,
                    global_goal_state,
                    param_
            );
        }
        // save state at T step (state_T)
        state_samples_[k][T].update(
            Eigen::Matrix<double, 3, 1>(
                x_star_.segment(T * (XDIM + UDIM), XDIM)
            )
        );
        // add terminal cost
        costs_[k] += controller::terminal_cost(state_samples_[k][T], global_goal_state, param_);

        // add cost for the KL distance between the base distribution and the sampled distribution
        for (int vt = 0; vt < VT; vt++)
        {
            costs_[k] += param_.controller.param_lambda * (1.0 - param_.controller.param_alpha) \
                * via_state_opt_seq_[vt].eigen().transpose() * (sigma_[vt].eigen().asDiagonal().inverse()) * via_state_samples_[k][vt].eigen();
        }
    }

    // calculate weight for each sample
    weights_ = calcWeightsOfSamples(costs_);

    // calculate optimal via state sequence
    for (int k = 0; k < K; k++)
    {
        for (int vt = 0; vt < VT; vt++)
        {
            via_state_opt_seq_[vt].update(via_state_opt_seq_[vt].eigen() + weights_[k] * noises_[k][vt].eigen());
        }
    }

    // get optimal state & input sequence
    Eigen::VectorXd x_opt_ = solveHQP(
        via_state_opt_seq_,
        global_vel,
        observed_state
    );

    // parse solution and calculate the optimal cost
    state_cost_ = 0.0;
    //// save initial state at 0 step
    state_opt_seq_[0].update(
        Eigen::Matrix<double, 3, 1>(
            x_opt_.segment(0 * (XDIM + UDIM), XDIM)
        )
    );
    for (int t = 1; t < T; t++)
    {
        // save input at t step (input_t s.t. state_{t+1} <- f(state_t, input_t))
        input_opt_seq_latest_[t-1].update(
            Eigen::Matrix<double, 3, 1>(
                x_opt_.segment((t-1) * (XDIM + UDIM) + XDIM, UDIM)
            )
        );
        // save state at t step (state_t)
        state_opt_seq_[t].update(
            Eigen::Matrix<double, 3, 1>(
                x_opt_.segment(t * (XDIM + UDIM), XDIM)
            )
        );
    }
    // save state at T step (state_T)
    state_opt_seq_[T].update(
        Eigen::Matrix<double, 3, 1>(
            x_opt_.segment(T * (XDIM + UDIM), XDIM)
        )
    );

    // apply savisky-golay filter to get smoothed input_opt_seq_latest_[0]
    if (param_.controller.use_sg_filter)
    {
        input_opt_seq_latest_[0] = applySaviskyGolayFilter(input_opt_seq_latest_);
    }

    // clip control input between umin and umax
    for (int t = 0; t < T; t++)
    {
        input_opt_seq_latest_[t].clamp();
    }

    // get mppi calculation time [ms]
    end = std::chrono::system_clock::now();  // stop timer
    calc_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

    // calculate and save optimal state trajectory
    state_cost_ = 0.0;
    for (int t = 1; t < T; t++)
    {
        // add stage cost
        Control prev_control_input = (t == 1) ? input_opt_latest_ : input_opt_seq_latest_[t-2];
        state_cost_ += controller::stage_cost(
                state_opt_seq_[t],
                input_opt_seq_latest_[t-1],
                prev_control_input,
                collision_costmap,
                distance_error_map,
                ref_yaw_map,
                global_goal_state,
                param_
        );
    }
    // add terminal cost
    state_cost_ += controller::terminal_cost(state_opt_seq_[T], global_goal_state, param_);
    // add cost for the KL distance between the base distribution and the sampled distribution
    for (int vt = 0; vt < VT; vt++)
    {
        state_cost_ += param_.controller.param_lambda * (1.0 - param_.controller.param_alpha) \
            * via_state_opt_seq_[vt].eigen().transpose() * (sigma_[vt].eigen().asDiagonal().inverse()) * via_state_opt_seq_[vt].eigen();
    }

    // convert optimal control command to VxVyOmega, transforming velocity command from global frame to local vehicle frame
    input_opt_latest_ = input_opt_seq_latest_[0]; // update input_opt_latest_
    common_type::VxVyOmega optimal_vxvyw_cmd;
    optimal_vxvyw_cmd.vx =  input_opt_latest_.vx * std::cos(observed_state.yaw) + input_opt_latest_.vy * std::sin(observed_state.yaw);
    optimal_vxvyw_cmd.vy = -input_opt_latest_.vx * std::sin(observed_state.yaw) + input_opt_latest_.vy * std::cos(observed_state.yaw);
    optimal_vxvyw_cmd.omega = input_opt_latest_.omega;
    return optimal_vxvyw_cmd;
}

Eigen::VectorXd MPCCore::solveHQP(
    const ViaStateSeq& via_state_seq,
    const common_type::VxVyOmega& global_vel,
    const common_type::XYYaw& observed_state
)
{
        // task to satisfy state equation
        hqp::SatisfyStateEquation task_sse;
        task_sse.setPredictionHorizon(T);
        task_sse.setTimeStepLength(STEP_LEN_SEC);
        task_sse.setInitialStates(observed_state.x, observed_state.y, observed_state.yaw);
        // TODO: make velocity limits configurable
        task_sse.setVelocityLimits(-2.0, 2.0, -2.0, 2.0, -1.57, 1.57);
        task_sse.constructTask();

        // task to track target states (via states + terminal state)
        hqp::TrackTargetState task_tts;
        task_tts.setPredictionHorizon(T);
        // add via state constraint
        for (int vt = 0; vt < VT; vt++)
        {
            task_tts.addTrackingPose(
                IDX_VIA_STATES[vt],
                via_state_seq[vt].x, via_state_seq[vt].y, wrapAngle(via_state_seq[vt].yaw, observed_state.yaw)
            );
        }
        task_tts.constructTask();

        // task to minimize velocity and acceleration (smoothing trajectory)
        hqp::MinimizeVelocityAndAcceleration task_minvelacc;
        task_minvelacc.setPredictionHorizon(T);
        task_minvelacc.setPreviousTwistCommand(global_vel.vx, global_vel.vy, global_vel.omega);
        task_minvelacc.constructTask();

        // formulate hierarchical quadratic programming (HQP) problem
        hqp::HQP hqp_solver;
        hqp_solver.addTask(&task_sse); // first priority
        hqp_solver.addTask(&task_tts); // second priority
        hqp_solver.addTask(&task_minvelacc); // third priority

        // solve HQP problem and return the result
        Eigen::VectorXd x_star_ = hqp_solver.solve();
        if (x_star_.rows() == 0)
        {
            std::cout << "[ERROR] HQP solver failed to find solution" << std::endl;
        }
        return x_star_;
}

// wrap yaw angle in the range of [center_yaw - pi, center_yaw + pi]
double MPCCore::wrapAngle(double yaw, double center_yaw)
{
    const double pi = M_PI;
    return yaw - 2.0 * pi * std::floor((yaw - center_yaw + pi) / (2.0 * pi));
}

// generate noise matrix whose size is K x T x UDIM
//   K    : number of samples
//   T    : prediction horizon stepscost = c_terminal(x)
//   UDIM : control space dimension
//   sigma[t][u] : noise parameter (variance of normal distribution) at time t and control space dimension u
ViaStateSeqSamples MPCCore::generateNoiseMatrix(ViaStateSeq& sigma)
{
    // declare noise matrix
    ViaStateSeqSamples noises = ViaStateSeqSamples(K, ViaStateSeq(T, ViaState()));

    // set random value to noises, which is normal distribution with mean 0.0 and variance sigma[t][u]
    // [CPU Acceleration with OpenMP]
    // #pragma omp parallel for num_threads(omp_get_max_threads()) collapse(3)
    for (int k = 0; k < K; k++)
    {
        for (int vt = 0; vt < VT; vt++)
        {
            for (int d = 0; d < VXDIM; d++)
            {
                std::normal_distribution<double> normal_dist(0.0, sigma[vt][d]);
                noises[k][vt][d] = normal_dist(psedo_random_engine_);
            }
        }
    }

    return noises;
}

// calculate weight for each sample 
Samples MPCCore::calcWeightsOfSamples(const Samples& costs)
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
void MPCCore::initSaviskyGolayFilter(
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
    SG_FILTER_WINDOW_SIZE_ = 2 * SG_FILTER_HALF_WINDOW_SIZE_ + 1; // HALF_WINDOW(past u log) | input_opt_seq[0] | HALF_WINDOW(u prediction)
    SG_FILTER_POLY_ORDER_ = poly_order;
    SG_FILTER_DELTA_ = delta;
    input_log_seq_for_filter_ = ControlSeq(SG_FILTER_HALF_WINDOW_SIZE_, Control()); // size is (SG_FILTER_HALF_WINDOW_SIZE_, UDIM)

    // calculate and save savisky-golay filter coefficients (you need to call this function only once)
    savisky_golay_coeffs_ = calcSaviskyGolayCoeffs(
        SG_FILTER_HALF_WINDOW_SIZE_,
        SG_FILTER_POLY_ORDER_,
        SG_FILTER_DELTA_
    );
}

// calculate savisky-golay filter coefficients
// reference: https://github.com/Izadori/cpp_eigen/blob/main/savgol/savgol.cpp
Eigen::MatrixXd MPCCore::calcSaviskyGolayCoeffs(
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

// apply savisky-golay filter to get smoothed input_opt_seq[0]
Control MPCCore::applySaviskyGolayFilter(ControlSeq& input_opt_seq)
{
    // initialize filtered control input
    Control input_opt_filtered;
    input_opt_filtered.setZero();

    // apply savisky-golay filter
    for (int i = 0; i < SG_FILTER_WINDOW_SIZE_; i++)
    {
        if (i < SG_FILTER_HALF_WINDOW_SIZE_)
        {
            input_opt_filtered.update(input_opt_filtered.eigen() + savisky_golay_coeffs_(i) * input_log_seq_for_filter_[i].eigen());
        }
        else
        {
            input_opt_filtered.update(input_opt_filtered.eigen() + savisky_golay_coeffs_(i) * input_opt_seq[i-SG_FILTER_HALF_WINDOW_SIZE_].eigen());
        }
    }

    // update input_log_seq_for_filter_ shifting index to the left
    for (int j = 0; j < SG_FILTER_HALF_WINDOW_SIZE_ - 1; j++)
    {
        input_log_seq_for_filter_[j] = input_log_seq_for_filter_[j+1];
    }
    input_log_seq_for_filter_[SG_FILTER_HALF_WINDOW_SIZE_ - 1] = input_opt_filtered; // update the latest element with input_opt_seq[0]

    // return smoothed control input
    return input_opt_filtered;
}

// get calc time [ms]
float MPCCore::getCalcTime()
{
    return calc_time_;
}

// get state cost of the latest optimal trajectory
double MPCCore::getStateCost()
{
    return state_cost_;
}

// get controller name
std::string MPCCore::getControllerName()
{
    return param_.controller.name;
}

// check if the vehicle is reached to the goal
bool MPCCore::isGoalReached()
{
    return is_goal_reached_;
}

// get optimal vehicle command (8DoF)
common_type::VehicleCommand8D MPCCore::getOptimalVehicleCommand()
{
    return target_system::convertControlSpace3DToControlSpace8D(input_opt_latest_, param_);
}

// return optimal state sequence
std::vector<common_type::XYYaw> MPCCore::getOptimalTrajectory()
{
    std::vector<common_type::XYYaw> optimal_state_sequence;
    for (int t = 0; t < T+1; t++)
    {
        common_type::XYYaw state = target_system::convertStateSpace3DToXYYaw(state_opt_seq_[t]);
        optimal_state_sequence.push_back(state);
    }
    return optimal_state_sequence;
}

// return full sampled state sequences
StateSeqSamples MPCCore::getFullSampledTrajectories()
{
    std::vector<std::vector<common_type::XYYaw>> full_sampled_state_sequences;
    for (int k = 0; k < K; k++)
    {
        std::vector<common_type::XYYaw> state_sequence;
        for (int t = 0; t < T+1; t++)
        {
            common_type::XYYaw state = target_system::convertStateSpace3DToXYYaw(state_samples_[k][t]);
            state_sequence.push_back(state);
        }
        full_sampled_state_sequences.push_back(state_sequence);
    }
    return full_sampled_state_sequences;
}

// return elite sampled state sequences
StateSeqSamples MPCCore::getEliteSampledTrajectories(int elite_sample_size)
{
    std::vector<std::vector<common_type::XYYaw>> elite_sampled_state_sequences;
    for (int k = 0; k < elite_sample_size; k++)
    {
        std::vector<common_type::XYYaw> state_sequence;
        for (int t = 0; t < T; t++)
        {
            common_type::XYYaw state = target_system::convertStateSpace3DToXYYaw(state_samples_[costs_rank_[k]][t]);
            state_sequence.push_back(state);
        }
        elite_sampled_state_sequences.push_back(state_sequence);
    }
    return elite_sampled_state_sequences;
}

// return via state sequence
ViaStateSeq MPCCore::getViaStateSequence()
{
    return via_state_opt_seq_;
}

} // namespace controller