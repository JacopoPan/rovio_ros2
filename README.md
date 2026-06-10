> [!NOTE]
> This is a 1-to-1 ROS2 translation of the original ROS1 repo: https://github.com/ethz-asl/rovio

# ROVIO ROS2

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework. The code is open-source (BSD License).
Video: https://youtu.be/ZMAISVy-6ao

## Installation

```sh
sudo apt update
sudo apt install -y --no-install-recommends freeglut3-dev libglew-dev valgrind

git clone https://github.com/ethz-asl/kindr.git && cd kindr
mkdir build && cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
sudo make install

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules -b feat/ros2 https://github.com/JacopoPan/rovio_ros2.git
cd ..
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rovio --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release

source /aas/ros2_ws/install/setup.bash && ros2 launch rovio rovio_node.launch.py
source /aas/ros2_ws/install/setup.bash && ros2 launch rovio valgrind_rovio.launch.py # Add option '-mno-avx512f' *after* '-march=native'
```
<!--
### Install with opengl scene ###
Additional dependencies: opengl, glut, glew (sudo apt-get install freeglut3-dev, sudo apt-get install libglew-dev)
```
#!command

catkin build rovio --cmake-args -DCMAKE_BUILD_TYPE=Release -DMAKE_SCENE=ON
```
-->

## Euroc Datasets
The rovio_node.launch file loads parameters such that ROVIO runs properly on the Euroc datasets.

The datasets are available at: [www.research-collection.ethz.ch](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f)

```sh
pip install rosbags
# Download and unzip, for example, "Vicon Room 2 Datasets (ZIP, 5734.81 MB)"
cd vicon_room2/V2_01_easy/
rosbags-convert --src V2_01_easy.bag --dst V2_01_easy_ros2 --dst-version 5 --dst-typestore ros2_humble
source /aas/ros2_ws/install/setup.bash && ros2 launch rovio rovio_rosbag_node.launch.py rosbag_path:=/absolute/path/to/V2_01_easy_ros2
```

## Further Notes

* Camera matrix and distortion parameters should be provided by a yaml file or loaded through rosparam
* The cfg/rovio.info provides most parameters for rovio. The camera extrinsics qCM (quaternion from IMU to camera frame, Hamilton-convention) and MrMC (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter doVECalibration to false. Please be carefull that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.

Papers:
* http://dx.doi.org/10.3929/ethz-a-010566547 (IROS 2015)
* http://dx.doi.org/10.1177/0278364917728574 (IJRR 2017)
