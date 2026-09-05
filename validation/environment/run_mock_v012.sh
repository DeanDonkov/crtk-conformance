#!/bin/bash
# v0.1.2 mock validation campaign inside focal-crtk:rc3 (mount /home/claude/rc3 at /rc3)
source /rc3/live/env.sh
git config --global --add safe.directory "*"
export RC3_CONTAINER_IMAGE="focal-crtk:rc3" RC3_CONTAINER_DIGEST="$(cat /rc3/live/image_digest.txt 2>/dev/null)" RC3_ROS_MANIFEST="validation/environment/ros_noetic_source_manifest.txt"
cd /rc3/preprint/crtk-conformance
python3 validation/run_validation_v012.py --out validation/v0.1.2/mock "$@" 2>&1 | tee validation/v0.1.2/mock/run_validation_v012.log
