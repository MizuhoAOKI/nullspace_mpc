# vel_driver
This package convert 3DoF twist message (vx, vy, yaw_rate) to 8DoF vehicle command message and publish it.  
Gazebo will subscribe the 8DoF vehicle command message and apply it to the 4WIDS vehicle model in the simulation.


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /cmd_vel        | geometry_msgs/Twist  | Target vx, vy, yaw_rate (on the Global Frame) to follow      |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /fwids/front_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front left wheel. |
| /fwids/front_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front left wheel. |
| /fwids/front_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the front right wheel. |
| /fwids/front_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the front right wheel. |
| /fwids/rear_left_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear left wheel. |
| /fwids/rear_left_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear left wheel. |
| /fwids/rear_right_steer_rad/command | std_msgs/Float64 | The target steering angle [rad] of the rear right wheel. |
| /fwids/rear_right_rotor_radpersec/command | std_msgs/Float64 | The target rotor speed [rad/s] of the rear right wheel. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| l_f                         | double | length from the front axle to the center of mass [m] |
| l_r                         | double | length from the rear axle to the center of mass [m] |
| d_l                         | double | length from the left wheel to the center of mass [m] |
| d_r                         | double | length from the right wheel to the center of mass [m] |
| tire_radius                 | double | tire radius of the target 4WIDS vehicle [m] |


## Usage
To launch the node individually, run the following commands.
```
source devel/setup.bash
roslaunch vel_driver vel_driver.launch
```


## Note
The conversion between 3DoF twist message and 8DoF vehicle command message is based on 
the assumption that there are no tire slip. See the following reference for more details.  
https://arxiv.org/abs/2409.08648
