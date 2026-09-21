#!/bin/bash
# RC9: (1) add the RC3 package set (tf, tf2_geometry_msgs, message_filters, ...) to the from-source ROS core, same pins;
# (2) build AMBF branch ambf-2.0 @ 16a81518 (with submodules) against it, as RC3 step 4. No AMBF source is modified.
set -e
export LANG=C.UTF-8
cd /ws/ros_src
python3 ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic \
  --only-pkg-with-deps roscpp rospy roslib tf tf2_ros tf2_msgs tf2_py tf2_geometry_msgs roslaunch rostopic rosnode rosservice rosparam rosmsg rosbash \
     sensor_msgs geometry_msgs std_msgs diagnostic_msgs std_srvs message_generation message_runtime message_filters rosgraph_msgs \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -DSETUPTOOLS_DEB_LAYOUT=OFF \
  -j2 > /ws/ros_extra_build.log 2>&1 || { tail -40 /ws/ros_extra_build.log; exit 1; }
echo BUILD_ROS_EXTRA_DONE
source /opt/ros/noetic/setup.bash
cd /ws/src_live/ambf-2.0 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 > /ws/ambf_cmake.log 2>&1 || { tail -40 /ws/ambf_cmake.log; exit 1; }
make -j2 > /ws/ambf_make.log 2>&1 || { grep -n "error" /ws/ambf_make.log | head -30; exit 1; }
ls -la /ws/src_live/ambf-2.0/bin/lin-x86_64/ | head
echo BUILD_AMBF_DONE
