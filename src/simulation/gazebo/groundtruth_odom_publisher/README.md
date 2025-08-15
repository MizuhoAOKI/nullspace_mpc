# groundtruth_odom_publisher
This package subscribes /groundtruth_odom topic from the gazebo world and convert it to tf2 message.


## Subscribing Topics

| Topic name      | Type                 | Description                                                  |
| --------------- | -------------------- | ------------------------------------------------------------ |
| /groundtruth_odom | nav_msgs/Odometry | The ground truth odometry data. |


## Publishing Topics

| Topic name      | Type              | Description                                                  |
| --------------- | ----------------- | ------------------------------------------------------------ |
| /tf | tf2_msgs/TFMessage | The tf data. |


## Node Parameters
| Parameter name               | Type   | Description                                                  |
| ---------------------------- | ------ | ------------------------------------------------------------ |
|base_frame| string | The base frame of the tf data. |
|odom_frame| string | The odom frame of the tf data. |
|groundtruth_topic| string | The topic name of the ground truth odometry data. |


# Usage
To launch the node individually, run the following commands.
```
source devel/setup.bash
roslaunch groundtruth_odom_publisher groundtruth_odom_publisher.launch
```

