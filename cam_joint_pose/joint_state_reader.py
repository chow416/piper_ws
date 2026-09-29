#!/usr/bin/env python3
"""Joint angles and tool pose -- the other half of what a hand-eye calibration
needs at every pose.

Two questions have to be answered for each capture:

  1. Which joint angles was the arm holding, and had it actually stopped
     moving before the shutter opened?
  2. Where was the tool frame?  That is `T_base_link <- press_tool_link`.

(2) is answered through tf2 rather than by calling forward kinematics here.
`robot_state_publisher` already expands the URDF into a TF tree driven by
`/joint_states`, so one `lookup_transform` gives the answer, and it stays
correct if the URDF changes. The reference implementation that builds a
`moveit::core::RobotState` by hand is in
`reference/press_travel_monitor.cpp` if you ever need it outside ROS.

Read joint angles BY NAME, never by index
-----------------------------------------
`JointState.position` is only meaningful next to `JointState.name`, and the
order of `name` is not guaranteed to match any fixed list. `reference/go_home.py`
does it by name for exactly this reason, and so does everything below.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

#: Joints the press arm actually uses. joint6 exists in the URDF but is not
#: actuated in this configuration, so it is deliberately absent.
DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]

#: Frame the camera is rigidly attached to. This is the `tool` of hand-eye
#: calibration: T_press_tool_link <- camera. Note it is NOT `press_tcp` (the
#: rod tip) and NOT `gripper_base` (the camera is mounted under the tool).
DEFAULT_TOOL_FRAME = "press_tool_link"

DEFAULT_BASE_FRAME = "base_link"


def quaternion_to_matrix(x, y, z, w):
    """3x3 rotation matrix from a quaternion. No scipy dependency."""
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    s = 2.0 / norm
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


def transform_to_matrix(transform):
    """Turn a geometry_msgs Transform into a 4x4 homogeneous matrix."""
    t = transform.translation
    q = transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


class JointStateReader(Node):
    """Indexes `/joint_states` by joint name and can wait for the arm to stop.

    Spins itself with `spin_once` in a loop, so it works in a plain script
    without an executor thread. Do not also spin the node from elsewhere.
    """

    def __init__(self, topic="/joint_states", joint_names=None):
        super().__init__("cam_joint_pose_reader")
        self.joint_names = list(joint_names or DEFAULT_JOINTS)
        self._latest = None
        self.create_subscription(JointState, topic, self._on_state, 10)
        self.get_logger().info(f"listening on {topic} for {self.joint_names}")

    def _on_state(self, message):
        self._latest = message

    def _current(self):
        """The newest sample as {name: position}, or None if incomplete."""
        if self._latest is None:
            return None
        by_name = dict(zip(self._latest.name, self._latest.position))
        if not all(name in by_name for name in self.joint_names):
            return None
        stamp = self._latest.header.stamp
        seconds = stamp.sec + stamp.nanosec * 1e-9
        return ({name: by_name[name] for name in self.joint_names}, seconds)

    def wait_for_joints(self, timeout=10.0):
        """Block until a complete sample arrives. Raises on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            current = self._current()
            if current is not None:
                return current
        raise RuntimeError(
            f"no /joint_states containing {self.joint_names} within "
            f"{timeout:.1f} s. Is the hardware bringup running?")

    def wait_until_settled(self, tolerance_rad=0.002, timeout=8.0,
                           stable_samples=5, poll=0.05):
        """Block until the joints stop moving, then return the final sample.

        A capture is only valid if the arm is stationary: this path has no
        hardware capture timestamp, so any motion between "read the joints" and
        "open the shutter" turns directly into calibration error.

        Returns (joints, joint_stamp, seconds_waited). Raises on timeout, so a
        moving or unresponsive arm fails loudly instead of silently producing a
        bad pose.
        """
        start = time.monotonic()
        deadline = start + timeout
        previous = None
        stable = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=poll)
            current = self._current()
            if current is None:
                continue
            joints = current[0]
            if previous is not None:
                worst = max(abs(joints[n] - previous[n]) for n in self.joint_names)
                if worst <= tolerance_rad:
                    stable += 1
                    if stable >= stable_samples:
                        # Re-read once more so the returned sample is the one
                        # the settle check actually approved.
                        final = self._current() or current
                        return final[0], final[1], time.monotonic() - start
                else:
                    stable = 0
            previous = joints
        raise RuntimeError(
            f"joints did not settle within {timeout:.1f} s "
            f"(tolerance {tolerance_rad} rad). The arm may still be moving.")


class ToolPoseProvider:
    """Looks up `T_base_frame <- tool_frame` from the live TF tree."""

    def __init__(self, node, base_frame=DEFAULT_BASE_FRAME,
                 tool_frame=DEFAULT_TOOL_FRAME):
        self.node = node
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def lookup(self, stamp=None, timeout=1.0):
        """Return the 4x4 tool pose, or None if the transform is unavailable.

        `stamp` takes a `rclpy.time.Time`. Passing the image's timestamp is the
        correct thing to do; when the arm is stationary, `Time()` (latest) is
        equivalent and easier.
        """
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.tool_frame,
                stamp if stamp is not None else rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout))
        except Exception as exc:                           # noqa: BLE001
            self.node.get_logger().warning(
                f"TF {self.base_frame} <- {self.tool_frame} unavailable: {exc}")
            return None
        return transform_to_matrix(transform.transform)

    def describe_frames(self):
        return f"{self.base_frame} <- {self.tool_frame}"
