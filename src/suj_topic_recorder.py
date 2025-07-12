#!/usr/bin/env python3
"""
One-shot dVRK measured_cp recorder:
grabs the very first PoseStamped from each topic,
dumps all five to JSON, then exits automatically.

Updated: 2025-07-12 
"""

import json
import rospy
from pathlib import Path
from geometry_msgs.msg import PoseStamped


class OneShotRecorder:
    def __init__(self, topics, out_path: Path):
        self._topics = topics
        self._out_path = out_path.expanduser().resolve()
        self._remaining = set(topics)        # topics still waiting
        self._data = {tp: [] for tp in topics}
        self._subs = {}                      # topic -> rospy.Subscriber
        self._done = False

        for tp in topics:
            self._subs[tp] = rospy.Subscriber(
                tp, PoseStamped, self._make_cb(tp), queue_size=1)
        rospy.loginfo("Waiting for first message on %d topics …", len(topics))

    def _make_cb(self, topic):
        def _cb(msg: PoseStamped):
            if self._done or topic not in self._remaining:
                return                       # already captured / shutting down

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
            self._remaining.remove(topic)


            self._subs[topic].unregister()

            if not self._remaining:          
                self._save_and_quit()
        return _cb

    def _save_and_quit(self):
        if self._done:     # guard
            return
        self._done = True

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._out_path, "w") as f:
            json.dump(self._data, f, indent=2)
        rospy.loginfo("Saved one-shot data to %s", str(self._out_path))
        rospy.signal_shutdown("Finished one-shot recording")


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="One-shot recorder for dVRK measured_cp topics")
    parser.add_argument("-o", "--output", default="SUJ_measured_cp_output.json",
                        help="Output JSON filename")
    args = parser.parse_args()

    rospy.init_node("measured_cp_oneshot_recorder", anonymous=True)

    topics = [
        "/SUJ/ECM/local/measured_cp",
        "/SUJ/PSM1/local/measured_cp",
        "/SUJ/PSM2/local/measured_cp",
        "/SUJ/PSM1/measured_cp",
        "/SUJ/PSM2/measured_cp",
    ]
    OneShotRecorder(topics, Path(args.output))
    rospy.spin()


if __name__ == "__main__":
    main()
