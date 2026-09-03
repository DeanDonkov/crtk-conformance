# Running without a ROS installation: the pure-Python ROS 1 stack (used for the v0.1.0 archive)

The v0.1.0 validation archive was executed in an environment without ROS packages. rospy, rosmaster and
the message classes were built from the ROS GitHub sources (all pure Python).  The commits used were not
recorded at the time (`--depth 1 -b noetic-devel` clones, 2026-09-02); this is a known reproducibility gap
of the v0.1.0 archive.  The 0.1.1 archives were produced in the container described in
`validation/environment/`, whose ROS sources are pinned by commit.

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

Message Python modules are generated with `genpy/scripts/genmsg_py.py` / `gensrv_py.py`
(`python validation/ros1/build_msgs.py <dir with the clones>`) and the source directories are placed on
`PYTHONPATH` (`R=<dir> source validation/ros1/ros1env.sh`). A master is started with
`python -c "import rosmaster; rosmaster.rosmaster_main(['rosmaster','--core','-p','11311'])"`.

Verified with Python 3.11 on Ubuntu 24.04 (loopback TCPROS latency ≈ 0.5 ms mean at 200 Hz).
