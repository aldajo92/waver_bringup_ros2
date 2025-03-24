# Waver Bringup package for ROS2

This package contains the launch files and configuration files for the Waveshare Rover.

## Dependencies
This package depends on the following packages:
- [`waver_description`](https://github.com/aldajo92/waver_description_ros2)
- [`waver_cv`](https://github.com/aldajo92/waver_cv_ros2)
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox/tree/humble) (Install via apt)

## Launch Files
The following launch files are available in this package:
- `waver_bringup.launch.py`: Launches the Waveshare Rover Simulation with the minimun configuration files without navigation.
- `waver_bringup_nav.launch.py`: Launches the Waveshare Rover Simulation to perform the mapping operation.

## Mapping
The mapping operation is performed using the slam_toolbox package. The following launch files are available in this package:
- `waver_mapping.launch.py`: Launches the mapping operation using the slam_toolbox package.

To save the map, run the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```
