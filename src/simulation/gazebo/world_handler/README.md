# world_handler
This package makes gazebo world and spawn a 4WIDS vehicle.


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /fwids/front_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front left wheel. |
| /fwids/front_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front left wheel. |
| /fwids/front_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front right wheel. |
| /fwids/front_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front right wheel. |
| /fwids/rear_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear left wheel. |
| /fwids/rear_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear left wheel. |
| /fwids/rear_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear right wheel. |
| /fwids/rear_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear right wheel. |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /laser_link/scan | sensor_msgs/LaserScan | The laser scan data. (2D point cloud) |
| /groundtruth_odom | nav_msgs/Odometry | The ground truth odometry data. |
| /fwids/joint_states | sensor_msgs/JointState | The joint state data. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| /fwids/joint_state_controller/publish_rate | double | The rate [Hz] at which to publish the joint state. |


## Usage
To launch the gazebo world individually, run the following commands.
```
source devel/setup.bash
roslaunch world_handler launch_gazebo_world_with_fwids.launch
```

Press Ctrl+C to shut down the gazebo world.


## Note
You can ignore the following error message when you run the gazebo world.
It means that PID controllers are not used in the simulation, and it is true in this system.

In current settings, if you specify a target value of steering angle [rad] or rotor speed [rad/s] through ros_control, 
joints in gazebo will follow the value without any delay.
```
[ERROR] [xxxxx]: No p gain specified for pid. xxxxx
```

