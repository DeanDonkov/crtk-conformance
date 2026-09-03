# Runtime environment inside focal-crtk:rc3 (source me): ROS Noetic (from source) + crtk_msgs + crtk-conformance (editable)
source /opt/ros/noetic/setup.bash
[ -f /rc3/live/catkin_ws/devel/setup.bash ] && source /rc3/live/catkin_ws/devel/setup.bash
export PYTHONPATH=/rc3/preprint/crtk-conformance/src:$PYTHONPATH
export LANG=C.UTF-8
export ROS_IP=127.0.0.1 ROS_HOSTNAME=localhost
