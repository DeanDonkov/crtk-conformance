"""crtk_mock: a kinematic CRTK/ROS 1 mock with documented-behaviour presets.

MockConfig and the presets are ROS-free; MockCRTKNode (rospy) is imported lazily so that analysis code can read the
preset parameters without a ROS installation.
"""
from .config import MockConfig  # noqa: F401
from .presets import PRESETS, get  # noqa: F401


def __getattr__(name):
    if name == "MockCRTKNode":
        from .node import MockCRTKNode
        return MockCRTKNode
    raise AttributeError(name)
