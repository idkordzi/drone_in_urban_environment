
## Preinstallation

Good to have before starting:

```bash
sudo apt install build-essentials git cmake curl
```

## Dependencies installation

### ROS2 Jazzy

Follow installation guide for deb package from official [website](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) untill ROS installation. It is recommended to choose `ros-jazzy-desktop-full` package:

```bash
sudo apt install ros-jazzy-desktop-full
```

Remmember to source ros variables:

```bash
source /opt/ros/jazzy/setup.bash
```

It can be also added to `~/.bashrc` script to be visible each time new terminal is run.

### Gazebo Harmonic

Follow installation guide from official [website](https://gazebosim.org/docs/harmonic/install_ubuntu/).

### ArduPilot Gazebo models

Project uses [ArduPilot models](https://github.com/ArduPilot/ardupilot_gazebo) for simulation and some additional dependencies need to be installed:

```bash
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

And after Gazebo installation:

```bash
export GZ_VERSION=harmonic
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
sudo rosdep init
rosdep update
```

### CUDA & cuDNN (optional)

Target detection and parts of path planning algorithm can be accelerated using GPU. To do so, additional installation is required:

- CUDA 12.x -> [CUDA Toolkit download archive](https://developer.nvidia.com/cuda-12-9-1-download-archive)
- cuDNN 9.x -> [cuDNN downloads](https://developer.nvidia.com/cudnn-downloads)

## Repositiory preparation

After installing all dependencies download submodules:

```bash
cd drone_in_urban_environment
git checkout develop
git submodule update --init
```

## Building from source

To build project run:

```bash
colcon build --symlink-install
```

or:

```bash
./repo_build.sh
```

## Project setup

Source simulation variables:

```bash
source ./setup.bash
source ./install/setup.bash
```

## Launching simulation

```bash
ros2 launch drone_simulation launch.py
```

## Troubleshooting

### Executing build scripts

It may be necessary to modify access settings:

```bash
sudo chmod u+x ./repo_build.sh ./repo_clear.sh ./setup.bash
```

### CUDA path setup

If during build, cmake throws error in regard to CUDA C++ compiler being not set, it needs to be done manualy:

```bash
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
export PATH=$PATH:$CUDA_HOME/bin
```

It can be also added to `~/.bashrc` script to be visible each time new terminal is run.

### Double Gazebo server

In case when there are more then one instances of gazebo server, simulation will not be able to run smoothly.
To kill server, find process PID and terminate by hand:

```bash
ps aux | grep gz
pkill -9 <gz process id>
```

or from CUDA toolkit:

```bash
nvidia-smi | grep gazebo
kill -9 <gpu process id>
```
