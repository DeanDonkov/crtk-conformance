#!/bin/bash
# ROS 1 Noetic core from source (RC3-pinned commits, see ros_noetic_source_manifest.txt), subset needed by dVRK 2.4.0 + crtk-conformance.
set -e
export LANG=C.UTF-8
cd /ws/ros_src
python3 ./src/catkin/bin/catkin_make_isolated --install --install-space /opt/ros/noetic \
  --only-pkg-with-deps roscpp rospy roslib tf2_ros tf2_msgs tf2_py roslaunch rostopic rosnode rosservice rosparam rosmsg rosbash \
     sensor_msgs geometry_msgs std_msgs diagnostic_msgs std_srvs message_generation message_runtime rosgraph_msgs \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -DSETUPTOOLS_DEB_LAYOUT=OFF \
  -j2 > /ws/ros_build.log 2>&1 || { tail -40 /ws/ros_build.log; exit 1; }
tail -3 /ws/ros_build.log
echo BUILD_ROS_DONE
