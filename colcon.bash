#!/bin/bash
source /home/menna/cv_bridge_ws/install/setup.bash
colcon build --packages-select sync_saver --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
