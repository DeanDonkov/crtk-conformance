"""0.1.6 (RC9): ensure_enabled sends one state command at a time.  The released dVRK console's ROS 1 bridge keeps only
the latest state command (queue size 1), so the 0.1.5 back-to-back `enable` + `home` never re-enabled a disabled arm.
A fake adapter emulates that depth-one state channel and a homing motion that reports is_homed before it ends."""
import threading
import time

import pytest

pytest.importorskip("rospy")
from crtk_conformance.adapter import Buffer  # noqa: E402
from crtk_conformance.probes.common import ensure_enabled  # noqa: E402


class _State:
    def __init__(self, state, homed, busy):
        self.state, self.is_homed, self.is_busy = state, homed, busy


class DepthOneArm:
    """state_command is latched into a single slot and processed every 10 ms: a second command within one cycle
    replaces the first.  `home` is accepted only in ENABLED; homing takes `homing_s`, with is_homed true and is_busy
    true while it moves; `enable` is accepted from DISABLED only; each change publishes a state message."""

    def __init__(self, homing_s=0.3, state="DISABLED", homed=False):
        self.buf = Buffer(100)
        self.slot = None
        self.state, self.homed, self.busy = state, homed, False
        self.homing_end = None
        self.received, self.processed = [], []
        self._stop = False
        self.homing_s = homing_s
        self._pub()
        threading.Thread(target=self._loop, daemon=True).start()

    def _pub(self):
        self.buf.push(_State(self.state, self.homed, self.busy))

    def _loop(self):
        while not self._stop:
            time.sleep(0.01)
            c, self.slot = self.slot, None
            if c is not None:
                self.processed.append(c)
                if c == "enable" and self.state == "DISABLED":
                    self.state = "ENABLED"; self._pub()
                elif c == "home" and self.state == "ENABLED":
                    self.homed, self.busy, self.homing_end = True, True, time.monotonic() + self.homing_s; self._pub()
                elif c == "disable":
                    self.state = "DISABLED"; self._pub()
            if self.homing_end is not None and time.monotonic() >= self.homing_end:
                self.busy, self.homing_end = False, None; self._pub()

    # the adapter surface ensure_enabled uses
    def has(self, name):
        return name in ("operating_state", "state_command")

    def subscribe(self, name):
        return self.buf

    def wait_for(self, buf, timeout):
        t = time.monotonic()
        while time.monotonic() - t < timeout:
            new = buf.since(t)
            if new:
                return new[-1][1]
            time.sleep(0.005)
        return None

    def operating_state(self, timeout=1.0):
        d = self.buf.latest()
        return None if d is None else d[1]

    def state_command(self, cmd):
        self.received.append(cmd)
        self.slot = cmd


def test_back_to_back_commands_are_lost_on_a_depth_one_channel():
    arm = DepthOneArm()
    arm.state_command("enable"); arm.state_command("home")   # the 0.1.5 sequence
    time.sleep(0.2)
    assert arm.processed == ["home"] and arm.state == "DISABLED"
    arm._stop = True


def test_ensure_enabled_reenables_through_a_depth_one_channel_and_waits_for_homing():
    arm = DepthOneArm(homing_s=0.3)
    info = ensure_enabled(arm, timeout_s=3.0)
    assert info["commands_sent"] == ["enable", "home"] and arm.processed == ["enable", "home"]
    assert info["state_after"] == {"state": "ENABLED", "is_homed": True, "is_busy": False}
    assert info["enable_latency_s"] is not None and info["enable_latency_s"] >= 0.3
    arm.state_command("disable"); time.sleep(0.05)
    info = ensure_enabled(arm, timeout_s=3.0)       # the state-machine sub-probe's re-enable
    assert info["state_after"]["state"] == "ENABLED" and info["commands_sent"] == ["enable", "home"]
    arm._stop = True


def test_ensure_enabled_on_a_ready_arm_sends_nothing():
    arm = DepthOneArm(state="ENABLED", homed=True)
    info = ensure_enabled(arm, timeout_s=3.0)
    assert info["commands_sent"] == [] and info["state_after"]["state"] == "ENABLED" and info["enable_latency_s"] is not None
    time.sleep(0.05)
    assert arm.processed == []
    arm._stop = True
