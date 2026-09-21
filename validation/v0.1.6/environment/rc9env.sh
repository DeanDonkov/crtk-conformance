# sourced inside focal-rc9:live: ROS Noetic (from source, RC3 pins) + dVRK 2.4.0 workspace (crtk_msgs 1.3.0) + crtk-conformance
source /opt/ros/noetic/setup.bash
source /ws/dvrk_ws/devel/setup.bash
export PYTHONPATH=/repo/src:$PYTHONPATH
export LANG=C.UTF-8 ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
export DVRK_WS=/ws/dvrk_ws
