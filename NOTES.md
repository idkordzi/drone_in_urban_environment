### Dependencies

- ROS2 Jazzy -> [ROS2 Documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Gazebo Harmonic -> [Gazebo Docs](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- ArduPilot Gazebo models -> [ArduPilot GitHub](https://github.com/ArduPilot/ardupilot_gazebo) (install only project' dependencies, repository will be downloaded later automatically)

After installing all dependencies download git submodules:

> git submodule update --init

### Building project

> colcon build --symlink-install

### Setup and first run

> source setup.bash

Then

> ros2 launch drone_simulation launch.py
