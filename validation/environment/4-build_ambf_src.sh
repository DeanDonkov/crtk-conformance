#!/bin/bash
# Stage 3: crtk_msgs (catkin) + AMBF ambf-2.0 @ 16a81518 (cmake, finds catkin) + SRC python packages.
set -e
export LANG=C.UTF-8
source /opt/ros/noetic/setup.bash
mkdir -p /rc3/live/catkin_ws/src
[ -e /rc3/live/catkin_ws/src/crtk_msgs ] || cp -r /rc3/sources/crtk_msgs /rc3/live/catkin_ws/src/crtk_msgs
cd /rc3/live/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 2>&1 | tail -3
source /rc3/live/catkin_ws/devel/setup.bash
cd /rc3/sources/ambf-2.0 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 2>&1 | grep -E "FOUND|NOT FOUND|Error|error|Configuring done|Generating done" | head -20
make -j2 2>&1 | grep -E "error|Error|\[100%\]|Linking|warning: unused" | grep -v "warning" | tail -40
echo BUILD_AMBF_DONE
ls -la /rc3/sources/ambf-2.0/bin/lin-x86_64/ 2>/dev/null | head
