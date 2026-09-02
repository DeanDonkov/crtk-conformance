"""`crtk-mock --preset emul-src-v1 [--set key=value ...]` — run the mock node until Ctrl-C."""
from __future__ import annotations

import argparse
import json
import sys
import time


def parse_value(v: str):
    try:
        return json.loads(v)
    except json.JSONDecodeError:
        return v


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="crtk-mock", description="Configurable CRTK-compatible mock node with injectable semantic divergences (ROS 1).")
    p.add_argument("--preset", default="reference")
    p.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="override a MockConfig field (JSON value)")
    p.add_argument("--list-presets", action="store_true")
    p.add_argument("--duration-s", type=float, default=0.0, help="exit after this many seconds (0 = run until interrupted)")
    args = p.parse_args(argv)
    from .presets import PRESETS, get
    from .node import MockCRTKNode

    if args.list_presets:
        for k, v in PRESETS.items():
            print(k, json.dumps(v.to_dict()))
        return 0
    cfg = get(args.preset)
    for kv in args.set:
        k, v = kv.split("=", 1)
        if not hasattr(cfg, k):
            print(f"unknown field {k}", file=sys.stderr)
            return 2
        setattr(cfg, k, parse_value(v))
    import signal
    import threading
    import rospy

    node = MockCRTKNode(cfg).start()
    print("crtk-mock running:", json.dumps(cfg.to_dict()), flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    t0 = time.time()
    while not stop.is_set():
        time.sleep(0.2)
        if args.duration_s and time.time() - t0 > args.duration_s:
            break
    node.shutdown()
    rospy.signal_shutdown("crtk-mock exit")  # unregisters this node from the master
    time.sleep(0.3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
