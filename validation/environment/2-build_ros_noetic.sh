#!/bin/bash
# Build the ROS 1 Noetic core from source (pinned commits in ros_src/manifest.txt) with catkin_make_isolated.
set -e
export LANG=C.UTF-8
cd /ws/ros_src
# roscpp/rosconsole need these packages from the workspace; build only what AMBF and the probes need.
python3 ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic \
  --only-pkg-with-deps roscpp rospy tf tf2_ros tf2_py tf2_geometry_msgs roslaunch rostopic rosnode rosservice rosparam rosmsg rosbash \
     sensor_msgs geometry_msgs std_msgs message_generation message_runtime message_filters rosgraph_msgs std_srvs \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -DSETUPTOOLS_DEB_LAYOUT=OFF \
  -j2 2>&1 | tail -20
echo BUILD_ROS_DONE
