# Dataset

### Environment

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

##### ros2 foxy 

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

##### node sync_saver

```bash
sudo apt install libopencv-dev
sudo apt install libyaml-cpp-dev
```



### compile

```bash
# match OpenCV4.5
source /home/menna/cv_bridge_ws/install/setup.bash
# compile package
colcon build --packages-select sync_saver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



### Run

```bash
# run zed_wrapper node
# env have been added
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm config_file:=zed_camera.yaml

# run davis node
# TODO

# run sync node
# env have not been added
source install/local_setup.bash
ros2 run sync_saver sync_saver
```

##### zed_wrapper node

Topics to subscribe to:

- "/zed/zed_node/left/image_rect_color"
- "/zed/zed_node/right/image_rect_color"
- "/zed/zed_node/imu/data"

##### davis node #TODO see  [dev branch](https://github.com/cccccrz/CPSP-Dataset/tree/dev?tab=readme-ov-file)

##### sync_saver

Subscribe to the zed_wrapper node, synchronize messages and save them to form a data set.

The dataset structure is as follows:

![image-20250526160118298](https://cdn.jsdelivr.net/gh/cccccrz/CPSP-Dataset@main/dataset.png)



### PS.

##### Developing a ROS 2 C++ package

> ROS2 foxy  

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

# Car Building

## Original Source:

The original building design from RoboRacer (the home page also contains the complete bill of materials required): https://roboracer.ai/build  

>⚠️ **Please notice:** there are 3 custom pieces that cannot be directly purchased.
>- The antenna mount must be 3D printed, design files here: https://drive.google.com/drive/folders/1sy-XiJJ4hmhEKf5qQbUaPYY6Aw-L31Gk?usp=drive_link
>- The platform deck where all the computing elements are screwed on must be lasercut, design files here: https://drive.google.com/drive/folders/1NU4FZzvMEGKCOFzDBvnjyePnnSMvsZPG?usp=drive_link
>- The powerboard is not a single piece: the bill of materials lists all the components (resistors, capacitors, etc.) but you must solder them on the circuit board by yourself.

There are 3 useful YT videos showing the building procedure step-by-step:
- lower level chassis: https://www.youtube.com/watch?v=IoWHUGFfrRE
- setup of autonomy elements + upper level chassis: https://www.youtube.com/watch?v=L-V-0zzkl10
- putting all together: https://www.youtube.com/watch?v=vNVFCq688ck


## Design Variations:  
Several variations were made to the original project in order to obtain a design more suited for our purposes: analyze an environment and extrapolate a dataset from it.
Thanks to these adjustments, the overall project becomes much simpler and cheaper.

### Used Peaces:
- Traxxas car
- Lipo batteries
  - 3-headed cable (it gives power to the car's motor and the powerboard)
- Platform deck (lasercut)
- NVIDIA Jetson Orin Nano
  - WiFi module
  - SSD card
- Powerboard (assembled)
  - power cable (it gives power to the Jetson)
- Antennas
  - Antenna mount (3D print)
  - coaxial cables

### Unnecessary Peaces:
- VESC board
- Hokuyo Lidar

>⚠️ **Please notice:**
>- You have to craft yourself the 3-headed cable.
>- The coaxial cables come alongside the antennas if you buy them from the link provided in the bill of materials.
>- You must purchase yourself the WiFi module for the Jetson.

## Original design vs Modified design  
### Original  
The original building is designed to be completely autonomous and controlled remotely by a PC. ROS 2 is used exclusively for manouvering the vehicle and managing the synchronization of the different devices (Jetson, VESC, Lidar, Servo); it DOES NOT perform an elaboration of the surrounding environment (no cameras are involved).  
### Modified  
The modified building is designed to accomplish an elaboration of the surroundings while moving the car, in order to acquire a dataset. 

