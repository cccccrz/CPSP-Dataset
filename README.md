# Dataset

- Jetpack 6.1

  check version `cat /etc/nv_tegra_release`

- Cuda 11.4

  check version `nvcc --version`

- ROS2 foxy on Ubuntu 20.04

  https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html

- ZED SDK 4.0.8

  https://www.stereolabs.com/en-it/developers/release/archives

- zed-ros2-wrapper humble-v4.0.8

  https://github.com/stereolabs/zed-ros2-wrapper/releases/tag/humble-v4.0.8



### ROS2 foxy 

Create package

```bash
ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

Build package

```bash
colcon build --packages-select my_package
colcon build --packages-select sync_saver
colcon build --symlink-install --packages-select sync_saver
```

Source package

```bash
source install/local_setup.bash
```

Use package

```bash
ros2 run my_package my_node
ros2 run sync_saver sync_saver
```



### zed-ros2-wrapper

version: humble-v4.0.8

##### install

```bash
mkdir -p ~/ros2_ws/src/
cd ~/ros2_ws/src/
git clone --branch humble-v4.0.8 --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
# dependencies
cd ~/ros2_ws/src/
git clone https://github.com/ros-drivers/nmea_msgs.git
# --recursive include zed-ros2-interfaces
# git clone --branch humble-v4.0.8 --recursive https://github.com/stereolabs/zed-ros2-interfaces.git
# check 
cd ..
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)

# add env
echo source $(pwd)/install/local_setup.bash >> ~/.bashrc # automatically source the installation in every new bash (optional)
source ~/.bashrc
```

##### run

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=<camera_model>
# zed mini
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm
```

```bash
# show topic
ros2 topic list
# echo topic context
ros2 topic echo /camera/left/image_rect_color --no-arr
```



### dependencies

ros2 foxy 

```bash
rosdep install -i --from-path node_ros2 --rosdistro foxy -y
```

ros2 foxy need OpenCV 4.2

conflict with OpenCV4.5：

```bash
# Uninstall cv_bridge that comes with ROS
sudo apt remove ros-foxy-cv-bridge

# Compile cv_bridge from source code (need to match your OpenCV 4.5)
mkdir -p ~/cv_bridge_ws/src
cd ~/cv_bridge_ws/src
git clone -b foxy https://github.com/ros-perception/vision_opencv.git
cd ..
colcon build --packages-select cv_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Recompile your package
cd ~/Project_Dataset
colcon build --packages-select sync_saver
```

