# Running without a ROS installation: the pure-Python ROS 1 stack

The validation reported in the paper was executed in a container without ROS packages. rospy,
rosmaster and the message classes were built from the ROS GitHub sources (all pure Python):

```
git clone --depth 1 -b noetic-devel https://github.com/ros/ros_comm      # rospy, rosgraph, rosmaster, rostopic
git clone --depth 1 -b noetic-devel https://github.com/ros/ros           # roslib
git clone --depth 1 -b noetic-devel https://github.com/ros/genpy
git clone --depth 1 -b noetic-devel https://github.com/ros/genmsg
git clone --depth 1 -b noetic-devel https://github.com/ros/catkin        # python/catkin (find_in_workspaces)
git clone --depth 1 -b noetic-devel https://github.com/ros/std_msgs
git clone --depth 1 -b noetic-devel https://github.com/ros/common_msgs   # geometry_msgs, sensor_msgs
git clone --depth 1 -b noetic-devel https://github.com/ros/ros_comm_msgs # rosgraph_msgs
git clone --depth 1 -b noetic-devel https://github.com/ros/geometry2     # tf2_msgs
git clone --depth 1 https://github.com/collaborative-robotics/crtk_msgs
pip install rospkg catkin_pkg defusedxml
```

Message Python modules are generated with `genpy/scripts/genmsg_py.py` / `gensrv_py.py` (see
`validation/ros1/build_msgs.py` in the paper's supplementary material for the exact script) and
the source directories are placed on `PYTHONPATH`. A master is started with
`python -c "import rosmaster; rosmaster.rosmaster_main(['rosmaster','--core','-p','11311'])"`.

Verified with Python 3.11 on Ubuntu 24.04 (loopback TCPROS latency ≈ 0.5 ms mean at 200 Hz).
