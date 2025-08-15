# joy_controller
This node receives the joypad input and publishes twist message to operate the vehicle manually.


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /joy            | sensor_msgs/Joy      | The joystick message. |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /cmd_vel        | geometry_msgs/Twist | The velocity command. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
| dev | string | The device name of the joystick. default: /dev/input/js0 |
| joy_topic | string | The topic name of the joystick message. |
| control_cmd_vel_topic | string | The topic name of the velocity command. |
| joy_top_left_button_idx | int | The index of the top left button. |
| joy_top_right_button_idx | int | The index of the top right button. |
| joy_left_stick_x_idx | int | The index of the left stick x-axis. |
| joy_left_stick_y_idx | int | The index of the left stick y-axis. |
| joy_right_stick_x_idx | int | The index of the right stick x-axis. |
| joy_right_stick_y_idx | int | The index of the right stick y-axis. |
| abs_max_linear_vel_x | double | The maximum linear velocity [m/s] in the x-axis (vehicle forward is positive). |
| abs_max_linear_vel_y | double | The maximum linear velocity [m/s] in the y-axis (vehicle left is positive). |
| abs_max_angular_vel_z | double | The maximum angular velocity [rad/s] in the z-axis, counter-clockwise is positive. |


## Usage
To launch the node individually, run the following commands.
```
source devel/setup.bash
roslaunch joy_controller joy_controller.launch
```

