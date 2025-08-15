# mpc_nav_evaluator
A node for evaluating MPC navigation performance based on MPCEval messages, publishing sequential goal poses from a scenario configuration, and saving evaluation results to a CSV file.

## Subscribing Topics

| Topic name | Type | Description |
| ---------- | ---- | ----------- |
| `~mpc_eval_topic` | `mpc_eval_msgs/MPCEval` | Evaluation information of the MPC controller, including state cost, vehicle pose, commands, and goal reach status. |

## Publishing Topics

| Topic name | Type | Description |
| ---------- | ---- | ----------- |
| `~goal_pose_topic` | `geometry_msgs/PoseStamped` | Goal pose to send to the navigation stack. Published sequentially according to the loaded scenario configuration. |

## Node Parameters

| Parameter name | Type | Description |
| -------------- | ---- | ----------- |
| `~initial_pose_x` | double | Initial x position [m] of the vehicle. Used to determine first goal and goal reach checks. Default: `0.0`. |
| `~initial_pose_y` | double | Initial y position [m] of the vehicle. Used to determine first goal and goal reach checks. Default: `0.0`. |
| `~mpc_eval_topic` | string | Topic name to subscribe for MPC evaluation data. |
| `~goal_pose_topic` | string | Topic name to publish goal poses. |
| `~scenario_config_path` | string | Path to YAML file containing scenario configuration (must contain a `goals` list). Default: `./default.yaml`. |
| `~eval_result_dir` | string | Directory path to save the evaluation result CSV file. Default: `./default`. |

## Scenario Configuration File
The scenario configuration YAML must contain a `goals` list, each with `goal_x` and `goal_y` fields.  
Example:
```yaml
goals:
- goal_x: 5.0
  goal_y: 0.0
- goal_x: -2.5
  goal_y: 2.5
```

## Behavior
1. **Initialization**  
   - Loads scenario configuration and parameters.
   - Creates a CSV file in `eval_result_dir` with a header row for evaluation results.
2. **Goal Publishing**  
   - Waits `launch_waiting_time` seconds after node start, then publishes the first goal.
   - Upon receiving `goal_reached=True` in `MPCEval` and detecting a new goal, publishes the next goal from the scenario.
3. **Data Logging**  
   - Appends each received `MPCEval` message to the CSV file with relevant evaluation data.
4. **Simulation Termination**  
   - When all goals are reached, sends a `make killall` command in the workspace to stop the simulation.

## CSV Output Format
The CSV file contains the following columns:
```
time_stamp,state_cost,global_x,global_y,global_yaw,cmd_vx,cmd_vy,cmd_yawrate,cmd_steer_fl,cmd_steer_fr,cmd_steer_rl,cmd_steer_rr,cmd_rotor_fl,cmd_rotor_fr,cmd_rotor_rl,cmd_rotor_rr,calc_time_ms,goal_reached_count
```

## Usage
To launch the node individually, run the following commands.
```bash
source devel/setup.bash
rosrun mpc_nav_evaluator mpc_nav_evaluator.py
```
