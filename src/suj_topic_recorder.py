#!/usr/bin/env python3
"""
One-shot dVRK measured_cp recorder:
grabs the first PoseStamped from each topic,
computes the average publish rate of /PSM1/measured_cp
with the same algorithm and window as `rostopic hz`,
dumps everything to JSON, then exits automatically.

Updated : 2025-07-16
"""

import json, rospy
from collections import deque
from pathlib import Path
from geometry_msgs.msg import PoseStamped


# ------------ rostopic-style Hz tracker (100-sample window) ------------
class HzTracker:
    def __init__(self, window: int = 5000):
        self._stamps = deque(maxlen=window + 1)   # 101 stamps → 100 intervals
        self._fallback_wall = False               # header.stamp == 0 ⇒ use wall time

    def add(self, msg: PoseStamped):
        if not self._fallback_wall and msg.header.stamp.to_sec() > 0.0:
            self._stamps.append(msg.header.stamp.to_sec())
        else:
            self._fallback_wall = True            # future adds use wall clock too
            self._stamps.append(rospy.get_time())

    def ready(self) -> bool:                      # same condition rostopic uses
        return len(self._stamps) >= self._stamps.maxlen

    def average(self):
        if not self.ready():
            return None
        dt = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / dt if dt > 0 else None
# -----------------------------------------------------------------------


class OneShotRecorder:
    MON_TOPIC = "/PSM1/measured_cp"               # rate to be measured

    def __init__(self, topics, outfile: Path):
        self._topics = topics
        self._out_path = outfile.expanduser().resolve()
        self._waiting = set(topics)               # topics still waiting first msg
        self._data = {tp: [] for tp in topics}
        self._subs = {}
        self._hz = HzTracker()
        self._done = False

        for tp in topics:
            self._subs[tp] = rospy.Subscriber(
                tp, PoseStamped, self._make_cb(tp), queue_size=1
            )

        rospy.loginfo("Awaiting first message on %d topics, "
                      "plus 101 stamps for Hz on %s …",
                      len(topics), self.MON_TOPIC)

    # ------------------------------------------------------------------
    def _make_cb(self, topic):
        def _cb(msg: PoseStamped):
            if self._done:
                return

            # 1) Feed Hz tracker
            if topic == self.MON_TOPIC:
                self._hz.add(msg)

            # 2) Capture the very first message for this topic
            if topic in self._waiting:
                self._data[topic].append({
                    "stamp": msg.header.stamp.to_sec(),
                    "position": {
                        "x": msg.pose.position.x,
                        "y": msg.pose.position.y,
                        "z": msg.pose.position.z,
                    },
                    "orientation": {
                        "x": msg.pose.orientation.x,
                        "y": msg.pose.orientation.y,
                        "z": msg.pose.orientation.z,
                        "w": msg.pose.orientation.w,
                    }
                })
                rospy.loginfo("Captured first sample from %s", topic)
                self._waiting.remove(topic)

                # after the first sample, we can stop listening *unless* it is
                # the monitoring topic (need more stamps for Hz)
                if topic != self.MON_TOPIC:
                    self._subs[topic].unregister()

            # 3) stop monitoring topic once enough stamps are collected
            if topic == self.MON_TOPIC and self._hz.ready():
                self._subs[topic].unregister()

            # 4) if all first-messages captured and Hz ready → save & quit
            if not self._waiting and self._hz.ready():
                self._save_and_shutdown()
        return _cb

    # ------------------------------------------------------------------
    def _save_and_shutdown(self):
        if self._done:
            return
        self._done = True

        hz_val = self._hz.average()
        self._data["average_hz"] = {self.MON_TOPIC: round(hz_val, 2) if hz_val else None}

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._out_path, "w") as f:
            json.dump(self._data, f, indent=2)
        rospy.loginfo("Saved one-shot data to %s  (Hz = %.2f)",
                      str(self._out_path), hz_val if hz_val else -1.0)

        rospy.signal_shutdown("Finished one-shot recording")
# -----------------------------------------------------------------------


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="One-shot recorder for dVRK measured_cp topics + Hz")
    parser.add_argument(
        "-o", "--output",
        default="SUJ_measured_cp_output.json",
        help="Output JSON filename"
    )
    args = parser.parse_args()

    rospy.init_node("measured_cp_oneshot_recorder", anonymous=True)

    topics = [
        "/SUJ/ECM/local/measured_cp",
        "/SUJ/PSM1/local/measured_cp",
        "/SUJ/PSM2/local/measured_cp",
        "/SUJ/PSM1/measured_cp",
        "/SUJ/PSM2/measured_cp",
        "/PSM1/measured_cp"          # ← new plain topic to monitor Hz
    ]

    OneShotRecorder(topics, Path(args.output))
    rospy.spin()


if __name__ == "__main__":
    main()
