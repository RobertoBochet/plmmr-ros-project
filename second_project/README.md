# Second ROS project

## Files

The system is composed only by one custom package `robot_mapping`.

### robot_mapping

#### `launch` folder

In this folder there are the launch files for mapping and navigation. There are also the version with optitrack, only with visual odometry and with sensor fusion.

#### `node` folder

In this one there are the xml file will be included in launch file. Each file contains only one node type.

#### `cfg` folder

Here there are the configuration file of the `amcl` node in yaml format.

#### `rviz` folder

This contains the configuration of rviz will be launched by the launch file.

#### `maps` folder

The three maps computed from bag file are stored in this folder.

## TF

### With optitrack

When the position of the robot is gotten exploiting optitrack we use as fixed frame `world` and `Robot_1/base_link` as frame affixed to the robot. This one is linked to `os1_sensor` exploiting a static transformation.  
The link between `world` and `map` is always provided by `gmapping`.

### With visual odometry

When the visual odometry is used to calculate the robot position the fixed frame is `camera_odom_map`, the frame affixed to the robot is `camera_pose_frame`. This is connected with static transformation to `os1_sensor`. While the robot are mapping the link between `camera_odom_frame` and `map` is provided by `gmapping`, during navigation instead it is provided by `amcl` node.

### With visual odometry and imu

Unlike the TF with visual odometry, in this case the `os1_sensor` is staticaly connected to `base_link`, where this is the frame affixed to the robot. The connection between `camera_odom_frame` and `base_link` is provided by the `ekf_localization` node exploiting both imu and visual odometry. 

## How to start the the nodes

The launch files are contained in the `robot_mapping` package.
There are 5 launch files in this one:

- `mapping_optitrack` to mapping exploiting optitrack
- `mapping_visual` to mapping exploiting only the odometric vision
- `mapping_visual+imu` to mapping exploiting odometric vision and imu
- `navigation_visual` to test the navigation exploiting only the odometric vision
- `navigation_visual+imu` to test the navigation odometric vision and imu

## Bag files

To compute the map was used the `2020-05-14-16-09-36-traj1-os1-t265-pix.bag` file. The test of navigation were done on `2020-05-14-16-14-37-traj2-os1-t265-pix.bag` file.

## Info

The maps were created by both optitrack data, odometric and imu data, so it was generated 3 maps exploiting the 3 methods. The differences between maps generate with and without imu seem very small.

The laser scan was set between the heights of `-0.5m` and `-1.5m` (from the lidar) with the idea to detect also the low obstacles with the purpose of improving the behaviour of the global planner. 

For the navigation was been used the map generated with the sensor fusion.