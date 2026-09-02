"""Validation harness: starts a ROS master and a crtk-mock subprocess with a given configuration,
runs the probes from a separate node (this process), and returns the results.

Everything is reproducible from `python validation/run_validation.py` (see README).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from contextlib import contextmanager
from typing import Dict, Iterator, Optional

import rosgraph


def master_running() -> bool:
    try:
        return rosgraph.is_master_online()
    except Exception:
        return False


@contextmanager
def ros_master(port: int = 11411) -> Iterator[None]:
    """Start a private ROS master on `port` (so that no registrations from earlier runs survive)
    and point ROS_MASTER_URI at it for this process and all children."""
    os.environ["ROS_MASTER_URI"] = f"http://localhost:{port}"
    p = subprocess.Popen([sys.executable, "-c", f"import rosmaster; rosmaster.rosmaster_main(['rosmaster','--core','-p','{port}'])"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(50):
        if master_running():
            break
        time.sleep(0.1)
    try:
        yield
    finally:
        p.terminate()
        p.wait(timeout=5)


@contextmanager
def mock_node(preset: str = "reference", overrides: Optional[Dict] = None, namespace: str = "/PSM1", startup_s: float = 1.0) -> Iterator[subprocess.Popen]:
    """Run crtk-mock as a separate ROS node (separate process)."""
    cmd = [sys.executable, "-m", "crtk_mock.cli", "--preset", preset]
    for k, v in (overrides or {}).items():
        cmd += ["--set", f"{k}={json.dumps(v)}"]
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=os.environ.copy())
    # wait until the mock has registered at least one publisher in its namespace (bounded), then settle
    m = rosgraph.Master("/crtk_validation_harness")
    for _ in range(int(10.0 / 0.1)):
        try:
            pubs, _, _ = m.getSystemState()
            if any(t.startswith(namespace + "/") for t, _ in pubs):
                break
        except Exception:
            pass
        time.sleep(0.1)
    time.sleep(startup_s)
    try:
        yield p
    finally:
        p.terminate()
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
        # wait until the master no longer lists the mock's publications
        m = rosgraph.Master("/crtk_validation_harness")
        for _ in range(50):
            pubs, _, _ = m.getSystemState()
            if not any(t.startswith(namespace + "/") for t, _ in pubs):
                break
            time.sleep(0.1)
        time.sleep(0.2)
