#!/bin/bash
# dVRK 2.4.0 (jhu-saw/vcs ros1-dvrk-2.4.0.vcs) with catkin_tools on the from-source Noetic core.
# Hardware-, video-, RViz- and MATLAB-only packages are skipped; nothing in the dVRK sources is modified.
set -e
export LANG=C.UTF-8
source /opt/ros/noetic/setup.bash
cd /ws/dvrk_ws
catkin init > /dev/null
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DsawIntuitiveResearchKit_HAS_SUJ_Si=OFF > /dev/null
catkin config --skiplist dvrk_model dvrk_video dvrk_camera_registration dvrk_hrsv_widget dvrk_arms_from_ros \
  saw_intuitive_research_kit_example_bilateral_teleop saw_intuitive_research_kit_example_derived_teleop_psm \
  saw_intuitive_research_kit_example_psm_derived saw_intuitive_research_kit_tests saw_robot_io_1394_tests \
  saw_controllers_examples saw_text_to_speech_examples > /dev/null
catkin build -j2 -p1 --no-status --summary > /ws/dvrk_build.log 2>&1 || { grep -n -B5 -A20 "Errors\|error:" /ws/dvrk_build.log | tail -60; exit 1; }
tail -15 /ws/dvrk_build.log
echo BUILD_DVRK_DONE
