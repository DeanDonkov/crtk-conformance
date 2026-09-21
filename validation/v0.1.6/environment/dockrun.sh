#!/bin/bash
# usage: dockrun.sh <name> "<command>"   (runs inside focal-rc9:live with the build dir at /ws and the repo at /repo)
NAME=$1; shift
docker rm -f $NAME >/dev/null 2>&1
docker run --name $NAME --rm --network host --shm-size 1g -v /home/claude/build:/ws -v /home/claude/src/crtk-conformance:/repo -w /repo focal-rc9:live \
  bash -c "source /ws/rc9env.sh; $*"
