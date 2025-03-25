# Waver Bringup package for ROS2

This package contains the launch files and configuration files for the Waveshare Rover.

## Dependencies
This package depends on the following packages:
- [`waver_description`](https://github.com/aldajo92/waver_description_ros2)
- [`waver_cv`](https://github.com/aldajo92/waver_cv_ros2)
- [`slam_toolbox`](https://github.com/SteveMacenski/slam_toolbox/tree/humble) (Install via apt)

## Launch Files
The following launch files are available in this package:
- `mapping.launch.py`: Launches the mapping operation using the slam_toolbox package.
- `map_server.launch.py`: Launches the map server node to load a map.
- `localization.launch.py`: Launches the localization operation using the slam_toolbox package.

## Mapping
The mapping operation is performed using the slam_toolbox package:
```bash
ros2 launch waver_bringup mapping.launch.py
```

To save the map, run the following command:
```bash
ros2 run nav2_map_server map_saver_cli -f custom_map
```

Then move the custom_map.pgm and custom_map.yaml files to the [`maps`](./maps/) directory.