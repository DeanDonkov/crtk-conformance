#!/bin/bash
# v0.1.3 archive: mock campaign, then live v1, then live v2 (sequential to avoid CPU contention)
cd /home/claude/rc3
mkdir -p preprint/crtk-conformance/validation/v0.1.3/mock
docker run --rm --name mock-v013 --network host -v /home/claude/rc3:/rc3 focal-crtk:rc3 bash /rc3/live/run_mock_v013.sh > /home/claude/rc3/live/mock-campaign-v013.out 2>&1
docker run --rm --name live-v1-v013 --network host -v /home/claude/rc3:/rc3 focal-crtk:rc3 bash /rc3/live/run_live.sh v1 /rc3/preprint/crtk-conformance/validation/v0.1.3/live-src-v1 > /home/claude/rc3/live/live-v1-v013.out 2>&1
docker run --rm --name live-v2-v013 --network host -v /home/claude/rc3:/rc3 focal-crtk:rc3 bash /rc3/live/run_live.sh v2 /rc3/preprint/crtk-conformance/validation/v0.1.3/live-src-v2 > /home/claude/rc3/live/live-v2-v013.out 2>&1
echo ALLDONE > /home/claude/rc3/live/run_all_rc5.done
