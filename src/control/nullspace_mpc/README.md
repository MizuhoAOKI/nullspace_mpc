# nullspace_mpc
Nullspace MPC as a local planner for autonomous navigation of a 4wids vehicle.

## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /groundtruth_odom | nav_msgs/Odometry | The ground truth odometry data. |
| /move_base/NavfnROS/plan | nav_msgs/Path | The global path data. |
| /move_base/local_costmap/costmap | nav_msgs/OccupancyGrid | The local costmap data. |
| /distance_error_map | grid_map_msgs/GridMap | The distance error map data. |
| /ref_yaw_map | grid_map_msgs/GridMap | The reference yaw map data. |

## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /cmd_vel | geometry_msgs/Twist | The velocity command consisting of linear and angular velocities (vx, vy, omega). |
| /mpc/cmd/absvel | geometry_msgs/Twist | The absolute velocity command. i.e. absvel = sqrt(vx^2 + vy^2). |
| /mpc/cmd/vx | std_msgs/Float32 | The linear velocity command. The front direction of the vehicle is positive. |
| /mpc/cmd/vy | std_msgs/Float32 | The lateral velocity command. The left direction of the vehicle is positive. |
| /mpc/cmd/omega | std_msgs/Float32 | The angular velocity command. The counter-clockwise direction is positive. |
| /mpc/calc_time | std_msgs/Float32 | The calculation time of the controller. |
| /mpc/overlay_text | jsk_rviz_plugins::OverlayText | The overlay text to show controller name on rviz.|
| /mpc/optimal_traj | visualization_msgs/MarkerArray | The optimal trajectory for visualization. |
| /mpc/via_state_seq | visualization_msgs/MarkerArray | The via state sequence for visualization. |
| /mpc/sampled_traj | visualization_msgs/MarkerArray | The sampled trajectory for visualization. |
| /mpc/eval_info | mpc_eval_msgs/MPCEval | The evaluation information of the controller. |

## Node Parameters

| Parameter name | Type | Description |
| -------------- | ---- | ----------- |
| /nullspace_mpc/navigation/xy_goal_tolerance | double | The tolerance \[m] to determine that the goal position is reached. |
| /nullspace_mpc/navigation/yaw_goal_tolerance | double | The tolerance \[rad] to determine that the goal orientation is reached. |
| /nullspace_mpc/target_system/l_f | double | The distance \[m] from the vehicle's center to the front axle. |
| /nullspace_mpc/target_system/l_r | double | The distance \[m] from the vehicle's center to the rear axle. |
| /nullspace_mpc/target_system/d_l | double | The distance \[m] from the vehicle's center to the left wheel. |
| /nullspace_mpc/target_system/d_r | double | The distance \[m] from the vehicle's center to the right wheel. |
| /nullspace_mpc/target_system/tire_radius | double | The radius \[m] of the tire. |
| /nullspace_mpc/controller/name | string | The name of the controller. |
| /nullspace_mpc/controller/control_interval | double | The control interval \[s]. |
| /nullspace_mpc/controller/num_samples | int | The number of samples used in the optimization. |
| /nullspace_mpc/controller/prediction_horizon | int | The number of steps in the prediction horizon. |
| /nullspace_mpc/controller/step_len_sec | double | The time step length \[s] for each step in the prediction horizon. |
| /nullspace_mpc/controller/param_exploration | double | The exploration parameter for sampling. |
| /nullspace_mpc/controller/param_lambda | double | The λ parameter for the controller cost computation. |
| /nullspace_mpc/controller/param_alpha | double | The α parameter for cost smoothing. |
| /nullspace_mpc/controller/idx_via_states | int[] | Indices of via states in the prediction horizon (must be less than prediction_horizon). |
| /nullspace_mpc/controller/sigma | double[] | Standard deviations of noise for \[vx, vy, omega] at each via state. |
| /nullspace_mpc/controller/reduce_computation | bool | If true, noise sampling is performed only once and reused across all processes. |
| /nullspace_mpc/controller/weight_cmd_change | double[3] | Penalty weights for variation of \[vx, vy, omega]. |
| /nullspace_mpc/controller/weight_vehicle_cmd_change | double[8] | Penalty weights for variation of \[fl_steer, fr_steer, rl_steer, rr_steer, fl_vel, fr_vel, rl_vel, rr_vel]. |
| /nullspace_mpc/controller/ref_velocity | double | The reference velocity \[m/s]. |
| /nullspace_mpc/controller/weight_velocity_error | double | Penalty weight for velocity error. |
| /nullspace_mpc/controller/weight_angular_error | double | Penalty weight for angular error. |
| /nullspace_mpc/controller/weight_collision_penalty | double | Penalty weight for collisions. |
| /nullspace_mpc/controller/weight_distance_error_penalty | double | Penalty weight for distance error from the reference path. |
| /nullspace_mpc/controller/weight_terminal_state_penalty | double | Penalty weight for terminal state deviation. |
| /nullspace_mpc/controller/use_sg_filter | bool | If true, apply a Savitzky–Golay filter to smooth control commands. |
| /nullspace_mpc/controller/sg_filter_half_window_size | int | Half-window size for the Savitzky–Golay filter (1 to prediction_horizon - 1). |
| /nullspace_mpc/controller/sg_filter_poly_order | int | Polynomial order of the Savitzky–Golay filter. |

## Usage
To launch the node individually, run the following commands.

For nullspace_mpc node: (config/nullspace_mpc.yaml will be loaded.)
```
source devel/setup.bash
roslaunch nullspace_mpc nullspace_mpc.launch
```

## Note

`rostopic echo /mpc/eval_info` will show you the evaluation information of the controller in real-time.
An example output is shown below.

```
header: 
  seq: 126
  stamp: 
    secs: 13
    nsecs: 660000000
  frame_id: "nullspace_mpc_b"
state_cost: 1134.80859375
global_x: 6.007351875305176
global_y: 2.598604202270508
global_yaw: 0.27291983366012573
cmd_vx: 1.464385747909546
cmd_vy: -0.19296832382678986
cmd_yawrate: 0.28070420026779175
cmd_steer_fl: -0.03971844166517258
cmd_steer_fr: -0.03277630731463432
cmd_steer_rl: -0.24662145972251892
cmd_steer_rr: -0.20479810237884521
cmd_rotor_fl: 6.625393390655518
cmd_rotor_fr: 8.028000831604004
cmd_rotor_rl: 6.826725959777832
cmd_rotor_rr: 8.1949462890625
calc_time_ms: 3.0
goal_reached: False
```
