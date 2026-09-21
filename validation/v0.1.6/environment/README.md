# RC9 execution environment (21 September 2026)

The host was a cloud session with 2 vCPU, Docker Engine 29.4.3 and an egress policy that blocks Docker Hub and `packages.ros.org`. Neither block was routed around. Everything below was built from the Ubuntu archive (HTTP), GitHub sources and PyPI wheels downloaded on the host.

## Build chain

| Step | Result |
|---|---|
| `debootstrap --variant=minbase focal`, then `docker import` | `focal-minbase:dvrk` |
| `Dockerfile.1-toolchain`, with the host wheels in `wheels/` | `focal-dvrkbuild:rc9check` |
| `2-build_ros_noetic.sh`: ROS Noetic core from source at the RC3 pins (`ros_noetic_sources_as_built.txt`) | `focal-ros-noetic:rc9check` |
| `3-build_dvrk.sh`: dVRK 2.4.0 from `ros1-dvrk-2.4.0.vcs` (jhu-saw/vcs `7cc19fb`; `dvrk_sources_as_built.txt`) | workspace `dvrk_ws/` (20 packages built, 12 skiplisted) |
| `Dockerfile.4-campaign`, with the cp38 wheels in `wheels38rt/` | `focal-rc9:campaign` |
| `5-build_ros_extra_and_ambf.sh`: the RC3 ROS package set, then AMBF `ambf-2.0` @ `16a81518` (`src_live_sources_as_built.txt`). The result was committed as an image. | `focal-rc9:live` |

## Build details

- `3-build_dvrk.sh` runs with `--network host`, the session proxy and its CA mounted. This lets two CMake ExternalProjects (ReflexxesTypeII, gattlib) clone from GitHub.
- The dVRK is configured with `-DsawIntuitiveResearchKit_HAS_SUJ_Si=OFF`, a build option that changes no source (it drops the Bluetooth SUJ).
- No dVRK, SRC or AMBF source file was modified.

## Running the campaigns

Every campaign ran in `focal-rc9:live` through `dockrun.sh`, with the build directory mounted at `/ws` and this repository at `/repo`. `rc9env.sh` sources `/opt/ros/noetic` and the dVRK workspace.

`image_ids.txt` gives the image ids. `wheel_sha256.txt` lists the SHA-256 of every wheel. The wheels themselves are not archived, because they are public PyPI artifacts.
