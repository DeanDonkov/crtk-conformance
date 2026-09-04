# Execution environment of the v0.1.1 validation and live runs

Everything under `validation/v0.1.1/` was produced inside Docker images built from the files in this
directory, on a 2-vCPU x86-64 host (Linux 6.x, Docker 29.4.3). The build environment could not reach any
container registry, `packages.ros.org` or any ROS mirror (only `archive.ubuntu.com`, GitHub and PyPI were
reachable), so the ROS 1 Noetic stack was **built from source** on an Ubuntu 20.04 root filesystem
bootstrapped from the Ubuntu archive. The chain is reproducible from these files; the image ids and
content digests of the images actually used are in `image_ids.txt`.

| Step | File | Result |
|---|---|---|
| 0 | `debootstrap --variant=minbase --arch=amd64 focal ./focal http://archive.ubuntu.com/ubuntu`, then `tar -C focal -c . \| docker import - focal-minbase:rc3` | Ubuntu 20.04 base image |
| 1 | `Dockerfile.1-toolchain` (+ `wheel_manifest.txt` / `wheel_sha256.txt`: pure-Python build tools downloaded from PyPI on the host) | `focal-rosbuild:rc3` |
| 2 | `2-build_ros_noetic.sh` on the sources listed in `ros_noetic_repos.txt`, pinned to the commits in `ros_noetic_source_manifest.txt` (`catkin_make_isolated --install`, 60 packages) | `focal-ros-noetic:rc3` (committed container) |
| 3 | `Dockerfile.3-runtime` (cp38 wheels: numpy 1.24.4, scipy 1.10.1, jsonschema 4.17.3, matplotlib 3.7.5, pytest 7.4.4, PyYAML 6.0.1) | `focal-crtk:rc3` |
| 4 | `4-build_ambf_src.sh`: `crtk_msgs` @ 14b04fcf (catkin), AMBF branch `ambf-2.0` @ 16a8151816407dfedff6785748ae72323c280f67 with submodules (cmake, finds catkin), SRC scripts installed with `pip -e` | `ambf_simulator` binary, `ambf_client` Python package |
| 5 | `env.sh` (sourced in every run: ROS, `crtk_msgs`, `PYTHONPATH` → `src/`, `ROS_IP=127.0.0.1`) | — |
| 6 | `run_live.sh v1|v2 <outdir>` with `live_aux.py` and the two expectation files | `validation/v0.1.1/live-src-v1`, `live-src-v2` (earlier attempts kept as `live-src-v*-run*-{preliminary,home-pose,failed-startup,superseded}`) |
| 7 | `run_mock.sh` (`python3 validation/run_validation_v011.py --out validation/v0.1.1/mock` inside `focal-crtk:rc3`, then `analyze_v011.py` for `tables/` and the figures) | `validation/v0.1.1/mock` (superseded campaigns: `mock-run1-superseded`, `mock-run2-superseded`) |

Live targets: `surgical_robotics_challenge` v1.0.0 = 158b554e1a3e7cd618eadedf0505865891e25f27 and v2.0.0 =
03befbf1028d22b0a6495059af51e397646570cf, each on the AMBF `ambf-2.0` commit above (both READMEs require the
`ambf-2.0` branch without pinning a commit; the branch tip at the time of the run was used). The simulator ran
headless (`-g false`, under Xvfb) with the launch arguments of each release's own run script; the CRTK
interface script was started with `--two False --ecm False --scene False` (PSM1 only). No source file of
AMBF or of either SRC release was modified.

`meta_env.json` in each live directory and `meta.json` in the mock archive record the interpreter, package
versions (`pip freeze`), commits and commands of the run itself.
