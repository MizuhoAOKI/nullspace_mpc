# mppi_3d
MPPI-3D local planner for autonomous navigation of a 4wids vehicle.

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
| /mpc/calc_time | std_msgs/Float32 | The calculation time of the MPPI controller. |
| /mpc/overlay_text | jsk_rviz_plugins::OverlayText | The overlay text to show mppi name on rviz.|
| /mpc/optimal_traj | visualization_msgs/MarkerArray | The optimal trajectory for visualization. |
| /mpc/sampled_traj | visualization_msgs/MarkerArray | The sampled trajectory for visualization. |
| /mpc/eval_info | mpc_eval_msgs/MPCEval | The evaluation information of the MPPI controller. |

## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| /mppi_3d/navigation/xy_goal_tolerance | double | The tolerance [m] to judge that the goal is reached. |
| /mppi_3d/navigation/yaw_goal_tolerance | double | The tolerance [rad] to judge that the goal is reached. |
| /mppi_3d/target_system/d_l | double | The distance [m] from the center of the vehicle to the left wheel. |
| /mppi_3d/target_system/d_r | double | The distance [m] from the center of the vehicle to the right wheel. |
| /mppi_3d/target_system/l_f | double | The distance [m] from the center of the vehicle to the front axle. |
| /mppi_3d/target_system/l_r | double | The distance [m] from the center of the vehicle to the rear axle. |
| /mppi_3d/target_system/tire_radius | double | The radius [m] of the tire. |
| /mppi_3d/controller/name | string | The name of the MPPI controller. |
| /mppi_3d/controller/control_interval | double | The control interval [s]. |
| /mppi_3d/controller/num_samples | int | The number of samples [samples]. |
| /mppi_3d/controller/prediction_horizon | int | The prediction horizon [steps]. |
| /mppi_3d/controller/step_len_sec | double | The step length [s] in the prediction horizon. |
| /mppi_3d/controller/param_exploration | double | The exploration parameter of MPPI. |
| /mppi_3d/controller/param_lambda | double | The lambda parameter of MPPI. |
| /mppi_3d/controller/param_alpha | double | The alpha parameter of MPPI. |
| /mppi_3d/controller/sigma | double[3] | The standard deviation of the noise for [vx, vy, omega]. |
| /mppi_3d/controller/reduce_computation | bool | If true, noise sampling is done only once and the same noise is used for all processes. |
| /mppi_3d/controller/weight_cmd_change | double[3] | The penalty weight for the variation of [vx, vy, omega]. |
| /mppi_3d/controller/weight_vehicle_cmd_change | double[8] | The penalty weight for the variation of [fl_steer, fr_steer, rl_steer, rr_steer, fl_vel, fr_vel, rl_vel, rr_vel]. |
| /mppi_3d/controller/reference_velocity | double | The reference velocity [m/s]. |
| /mppi_3d/controller/weight_velocity_error | double | The penalty weight for the velocity error. |
| /mppi_3d/controller/weight_angular_error | double | The penalty weight for the angular error. |
| /mppi_3d/controller/weight_collision_penalty | double | The penalty weight for the collision. |
| /mppi_3d/controller/weight_distance_error_penalty | double | The penalty weight for the distance error. |
| /mppi_3d/controller/weight_terminal_state_penalty | double | The penalty weight for the terminal state. |
| /mppi_3d/controller/use_sg_filter | bool | If true, Savitzky-Golay filter is used for smoothing the control input. |
| /mppi_3d/controller/sg_filter_half_window_size | int | The half window size of the Savitzky-Golay filter. |
| /mppi_3d/controller/sg_filter_poly_order | int | The polynomial order of the Savitzky-Golay filter. |

## Usage
To launch the node individually, run the following commands.

For mppi_3d_a node: (config/mppi_3d_a.yaml will be loaded.)
```
source devel/setup.bash
roslaunch mppi_3d mppi_3d_a.launch
```

For mppi_3d_b node: (config/mppi_3d_b.yaml will be loaded.)
```
source devel/setup.bash
roslaunch mppi_3d mppi_3d_b.launch
```

## Note

`rostopic echo /mpc/eval_info` will show you the evaluation information of the MPPI controller in real-time.
An example output is shown below.

```
header: 
  seq: 126
  stamp: 
    secs: 13
    nsecs: 660000000
  frame_id: "mppi_3d_b"
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
