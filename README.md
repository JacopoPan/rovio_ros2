> [!NOTE]
> This is a 1-to-1 porting for ROS2 Humble of the original ROS1 repo: https://github.com/ethz-asl/rovio

# ROVIO ROS2

This repository contains the ROVIO (Robust Visual Inertial Odometry) framework:
[watch the video.](https://youtu.be/ZMAISVy-6ao)

The code is open-source (BSD License).

## Installation

```sh
sudo apt update
sudo apt install -y --no-install-recommends freeglut3-dev libglew-dev valgrind

git clone https://github.com/ethz-asl/kindr.git && cd kindr
mkdir build && cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
sudo make install

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules -b main https://github.com/JacopoPan/rovio_ros2.git
cd ..
source /opt/ros/humble/setup.bash    # Assumes ROS2 Humble was installed: https://docs.ros.org/en/humble/Installation.html
colcon build --packages-up-to rovio --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
                                                 -DMAKE_SCENE=ON -DCMAKE_BUILD_TYPE=Release

source /aas/ros2_ws/install/setup.bash
ros2 launch rovio rovio_node.launch.py
ros2 launch rovio valgrind_rovio.launch.py    # Re-build with option -DENABLE_VALGRIND_COMPATIBILITY=ON, if necessary
```

## EuRoC Datasets
The `rovio_node.launch.py` file loads parameters such that ROVIO runs properly on the EuRoC datasets.

The datasets are available at: [www.research-collection.ethz.ch](https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f)

```sh
# Download and unzip, for example, "Vicon Room 2 Datasets (ZIP, 5734.81 MB)"
cd vicon_room2/V2_01_easy/

pip install rosbags
rosbags-convert --src V2_01_easy.bag --dst V2_01_easy_ros2 --dst-version 5 --dst-typestore ros2_humble

source /aas/ros2_ws/install/setup.bash
ros2 launch rovio rovio_rosbag_node.launch.py rosbag_path:=/absolute/path/to/V2_01_easy_ros2

# Alternatively, in one terminal, run:
source /aas/ros2_ws/install/setup.bash && ros2 launch rovio rovio_node.launch.py
# In a second terminal, run:
ros2 bag play /absolute/path/to/V2_01_easy_ros2
```

## Further Notes

* Camera matrix and distortion parameters should be provided by a YAML file or loaded through ROS parameters
* The `cfg/rovio.info` provides most parameters for rovio. The camera extrinsics `qCM` (quaternion from IMU to camera frame, Hamilton-convention) and `MrMC` (Translation between IMU and Camera expressed in the IMU frame) should also be set there. They are being estimated during runtime so only a rough guess should be sufficient.
* Especially for application with little motion fixing the IMU-camera extrinsics can be beneficial. This can be done by setting the parameter `doVECalibration` to `false`. Please be careful that the overall robustness and accuracy can be very sensitive to bad extrinsic calibrations.

Papers:
* IROS 2015: http://dx.doi.org/10.3929/ethz-a-010566547
* IJRR 2017: http://dx.doi.org/10.1177/0278364917728574
