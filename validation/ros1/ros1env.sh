# Pure-Python ROS 1 (noetic) stack built from GitHub sources — no catkin.  Used for the v0.1.0 archive only;
# the 0.1.1 archives were produced in the container described in validation/environment/.
# Usage: R=/path/to/ros1 source validation/ros1/ros1env.sh   (R defaults to $HOME/ros1)
R=${R:-$HOME/ros1}
export PYTHONPATH=$R/genmsg/src:$R/genpy/src:$R/ros/core/roslib/src:$R/ros_comm/tools/rosgraph/src:$R/ros_comm/tools/rosmaster/src:$R/ros_comm/clients/rospy/src:$R/ros_comm/tools/rostopic/src:$R/ros_comm/tools/rosnode/src:$R/ros_comm/tools/rosparam/src:$R/site:$R/catkin/python:$PYTHONPATH
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_PACKAGE_PATH=$R
