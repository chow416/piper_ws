#!/usr/bin/env python3
"""Aim the press rod at a point on the calibration board, and press it.

The board centre is the target. Everything between "the camera sees the board"
and "the task has a point to press" happens here, in one place, because that
chain has four links and only one of them is measured on the day:

    camera optics  ->  board pose in the camera frame   (marker_locator)
    board geometry ->  the centre is (0,0,0) in the board frame
    hand-eye       ->  camera frame to press_tool_link  (T_flange_cam.json)
    joint states   ->  press_tool_link to base_link     (tf2 / robot_state_publisher)

The hand-eye is applied **here**, on purpose, rather than being baked into the
URDF. Two reasons:

  - It is the number under test. Keeping it in one file outside the robot
    description means `ros2 run` picks up a new value without a rebuild, and it
    means a wrong press has exactly one suspect.

  - The URDF wants `T_flange<-camera_link` (mechanical), the file the colleague
    delivers is `T_flange<-camera_optical` (Rz(-90 degrees)). Converting on the
    way in is a 122 degree mistake waiting to happen; doing it once, here, in
    code that is checked, is better than doing it by hand in a xacro.

Once the numbers stop moving, move it into the URDF and delete the conversion.
Until then, "fine tune" means editing one JSON.

Why the board centre is still the default
-----------------------------------------
Because it is the only point on the board whose position in the board frame is
known without measuring anything: it is the origin. That is what makes it the
right target while the camera, the intrinsics and the hand-eye are the things
under test -- a taped-on button needs a tape measure, and that measurement
would be indistinguishable from the error we are trying to find.

Once the chain is trusted, the button is what you actually want to press, and
that is what `--button` is for. It reads the board-to-button transform from
`config/board.yaml` (`targets.<name>.tip_in_board` and `press_axis_in_board`)
instead of aiming at the origin, and it refuses to run on a target still marked
`taught: false` -- those coordinates are placeholders, not measurements.

The two modes share everything: the same hand-eye, the same jitter gate, the
same workspace box. Only the point in the board frame and the axis differ.

Getting the conventions right is the whole job
----------------------------------------------
Both of these are easy to invert and neither fails loudly:

  - The press direction is the board's **-Z**. Our board frame has +Z out of the
    face, toward the observer, so an outward surface normal is the negation of
    the direction we want.

  - The point is where the rod **tip** lands, not the flange. The task subtracts
    its own approach distance from the point it is given, so handing it an
    approach point presses short by exactly that much.

Usage
-----
    # what would be pressed, and where -- sends nothing
    python3 tools/press_board_centre.py --settle 6

    # aim 3 mm to the left in the board frame, and actually send it
    python3 tools/press_board_centre.py --offset-mm -3,0,0 --publish

    # press a button whose position was measured relative to the board
    python3 tools/press_board_centre.py --button red --publish
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

#: Where the arm is at the point the rod touches down. Same box the task node
#: enforces, so a target this accepts is not one it will silently drop.
WORKSPACE_MIN = (-0.30, -0.50, -0.02)
WORKSPACE_MAX = (0.65, 0.50, 0.80)

#: The rod tip in the press_tool_link frame. One number, from the xacro.
TCP_IN_FLANGE = np.array([0.0, 0.0, 0.1541])

DEFAULT_HAND_EYE = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "T_flange_cam.json")


def find_press_targets_dir():
    """Locate `press_targets.py`.

    This tool lives in the outer repo's `tools/`, the library lives in the
    `piper_marker_perception` package -- so this needs a path, not an import.
    Searching a couple of known layouts beats copying the parser here: the
    `taught` gate and the axis normalisation are exactly the parts that must
    not drift between the two entry points.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    for candidate in (
        os.path.join(root, "src/Piper_ros/src/piper_marker_perception/scripts"),
        "/ws/src/Piper_ros/src/piper_marker_perception/scripts",
        "/ws/install/piper_marker_perception/lib/piper_marker_perception",
        os.path.join(root,
                     "install/piper_marker_perception/lib/piper_marker_perception"),
    ):
        if os.path.isfile(os.path.join(candidate, "press_targets.py")):
            return candidate
    raise SystemExit(
        "找不到 press_targets.py —— 包编译过了吗？（找过 tools/../src 和 /ws）")


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    trace = np.trace(m)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2.0
    out = [0.0, 0.0, 0.0, 0.25 * s]
    out[i] = 0.25 * s
    out[j] = (m[j, i] + m[i, j]) / s
    out[k] = (m[k, i] + m[i, k]) / s
    return tuple(out)


def frame_from_axis(axis):
    """A right-handed frame whose third column is `axis`.

    The task reads only the +Z axis of the orientation it is handed -- the roll
    about the rod is free, because joint6 does not participate -- so any valid
    frame will do, and this one is chosen to be continuous in `axis`.
    """
    axis = axis / np.linalg.norm(axis)
    seed = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(axis, seed))) > 0.9:
        seed = np.array([0.0, 1.0, 0.0])
    first = np.cross(seed, axis)
    first /= np.linalg.norm(first)
    return np.column_stack((first, np.cross(axis, first), axis))


class Aim(Node):
    def __init__(self, args):
        super().__init__("press_board_centre")
        self.args = args
        self.board = None
        self.create_subscription(
            PoseStamped, args.board_topic, self._on_board,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE))
        self.publisher = self.create_publisher(PoseStamped, args.target_topic, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_board(self, msg):
        self.board = msg

    def spin_until(self, ready, timeout_s, what):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if ready():
                return
        raise SystemExit(f"等不到{what}（{timeout_s:.0f}s 超时）—— 上游节点起来了吗？")

    def flange_in_base(self):
        return self.tf_buffer.lookup_transform(
            self.args.frame, "press_tool_link", rclpy.time.Time())


def gather(aim, seconds):
    """Collect board poses for `seconds`, and report how much they moved."""
    positions, quat = [], None
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(aim, timeout_sec=0.05)
        if aim.board is None:
            continue
        p = aim.board.pose.position
        positions.append(np.array([p.x, p.y, p.z]))
        o = aim.board.pose.orientation
        quat = (o.x, o.y, o.z, o.w)
    if not positions:
        raise SystemExit("整段时间里一帧板子位姿都没收到")
    centre = np.median(positions, axis=0)
    jitter = max(float(np.linalg.norm(p - centre)) for p in positions) * 1000.0
    return centre, quat, jitter, len(positions)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--hand-eye", default=DEFAULT_HAND_EYE,
                        help="T_flange<-camera_optical 的 json（默认 %(default)s）")
    parser.add_argument("--board-topic", default="/perception/board_pose")
    parser.add_argument("--target-topic", default="/press/target_pose")
    parser.add_argument("--frame", default="base_link",
                        help="目标发布在哪个系（任务只收 world / base_link）")
    parser.add_argument("--settle", type=float, default=6.0,
                        help="发布前先看这么多秒，取中位数（默认 %(default)s）")
    parser.add_argument("--max-jitter-mm", type=float, default=2.0,
                        help="板子位姿抖动超过这个值就拒绝发布（默认 %(default)s）")
    parser.add_argument("--offset-mm", default="0,0,0",
                        help="在**板系**里把瞄准点挪开，格式 dx,dy,dz 毫米。"
                             "中心是 0,0,0；这个就是以后微调的旋钮。"
                             "--button 时作为额外的微调叠加上去")
    parser.add_argument("--button", default="",
                        help="按 config/board.yaml 里 targets.<name> 的位置，"
                             "而不是板心。该条目必须已经是 taught: true")
    parser.add_argument("--board-config", default="",
                        help="board.yaml 路径（默认从 piper_marker_perception "
                             "包的位置推出来）")
    parser.add_argument("--publish", action="store_true",
                        help="真的发到 /press/target_pose。不给就只算不发。")
    parser.add_argument("--repeats", type=int, default=3,
                        help="发布重复几次，防止 DDS 发现没完成时首条被丢")
    args = parser.parse_args()

    offset = np.array([float(v) / 1000.0 for v in args.offset_mm.split(",")])
    if offset.shape != (3,):
        raise SystemExit("--offset-mm 要三个数，例如 -3,0,0")

    # 默认 = 板心模式：目标点就是原点（加微调），按压轴就是板系 -Z。
    tip_in_board = offset
    press_axis = np.array([0.0, 0.0, -1.0])
    button_label = "板心（板系原点）"
    if args.button:
        targets_dir = find_press_targets_dir()
        sys.path.insert(0, targets_dir)
        import press_targets  # noqa: E402  （路径上一步才拼出来）

        config_path = args.board_config or os.path.normpath(
            os.path.join(targets_dir, os.pardir, "config", "board.yaml"))
        try:
            targets = press_targets.PressTargetSet.from_yaml(config_path)
        except press_targets.ConfigError as exc:
            raise SystemExit(f"{config_path}: {exc}")
        if args.button not in targets:
            raise SystemExit(
                f"{config_path} 里没有 targets.{args.button}"
                f"（有的是：{', '.join(targets.names())}）")
        chosen = targets[args.button]
        # ⚠️ 这道闸门不能省。未教示的条目里是占位坐标（0,0,0 之类），
        # 拿它去压 = 命令机械臂去一个没人知道的地方。
        if not chosen.taught:
            raise SystemExit(
                f"targets.{args.button} 还是 taught: false —— 那是占位值，不是量出来的。"
                f"\n量完把 {config_path} 里那个 taught 改成 true 再跑。"
                f"\n教示工具：ros2 run piper_marker_perception teach_press_offset.py"
                f" --name {args.button}")
        # 单位是米。--offset-mm 是额外微调，照旧叠上去。
        tip_in_board = np.asarray(chosen.tip_in_board, dtype=float) + offset
        press_axis = np.asarray(chosen.press_axis_in_board, dtype=float)
        # (0,0,0) 是个合法值 —— 它就是板心。所以"还没填数"和"有意让 marker
        # 中心对准按键"看起来一模一样，而两者只差一个会不会按到板子中间去。
        # 不拒绝（后者是真需求），但必须出声。
        if not np.any(tip_in_board):
            print("⚠️  targets.%s 的 tip_in_board 是 (0,0,0) —— 这等于"
                  "**按板心**，不是按按键。\n"
                  "    有意让 marker 中心对准按键就忽略这条；"
                  "否则是还没把量到的数填进去。" % args.button)
        button_label = (f"按键 {args.button}（板系 tip="
                        f"({tip_in_board[0]*1000:+.1f}, {tip_in_board[1]*1000:+.1f}, "
                        f"{tip_in_board[2]*1000:+.1f}) mm，轴="
                        f"({press_axis[0]:+.3f}, {press_axis[1]:+.3f}, "
                        f"{press_axis[2]:+.3f})）")

    hand_eye = np.array(json.load(open(args.hand_eye))["T_flange_cam_4x4"],
                        dtype=float)
    if hand_eye.shape != (4, 4):
        raise SystemExit(f"{args.hand_eye} 里的 T_flange_cam_4x4 不是 4x4")

    rclpy.init()
    aim = Aim(args)
    try:
        aim.spin_until(lambda: aim.board is not None, 40.0, "板子位姿")
        aim.spin_until(
            lambda: aim.tf_buffer.can_transform(args.frame, "press_tool_link",
                                                rclpy.time.Time()),
            20.0, f"TF {args.frame}<-press_tool_link")

        board_centre, quat, jitter, frames = gather(aim, args.settle)
        print(f"板中心(相机系)  = ({board_centre[0]*1000:+7.1f}, "
              f"{board_centre[1]*1000:+7.1f}, {board_centre[2]*1000:+7.1f}) mm"
              f"   共 {frames} 帧，抖动 {jitter:.2f} mm")
        if jitter > args.max_jitter_mm:
            raise SystemExit(
                f"★ 板子位姿抖动 {jitter:.1f} mm 超过 {args.max_jitter_mm:.1f} mm。\n"
                "  两种可能，先分清楚再决定动不动：\n"
                "   ① 码真的在动（刚贴上去还没稳、背后是软的、或有人在画面里）\n"
                "      → 等它稳住再来\n"
                "   ② 码很稳，但它是**单个小 marker** —— 噪声底本身就高。\n"
                "      实测：3×3 标定板 0.3~1.3 mm，单个 28.5 mm 码 3.3 mm。\n"
                "      这种情况下这是**精度真的差**，不是误报。\n"
                "      临时想让这一轮跑起来：--max-jitter-mm <更大的数>\n"
                "      治本：把码印大，或换回 3×3 板 —— 位姿精度基本正比于\n"
                "      角点铺开的范围（3×3 板是 36 个角点铺在 104 mm 上，\n"
                "      单码是 4 个角点挤在 24 px 里）。")

        transform = aim.flange_in_base().transform
        t = transform.translation
        flange = np.eye(4)
        flange[:3, :3] = quat_to_matrix((transform.rotation.x, transform.rotation.y,
                                         transform.rotation.z, transform.rotation.w))
        flange[:3, 3] = (t.x, t.y, t.z)

        base_camera = flange @ hand_eye
        board_camera = quat_to_matrix(quat)

        # 目标点在板系里是 tip_in_board，在相机系里 = 板位姿作用在它上面。
        # 板心模式它的值就是 offset（原点 + 微调）；按键模式它来自 board.yaml，
        # 是量出来的按键中心。
        point_camera = board_camera @ tip_in_board + board_centre
        target = base_camera[:3, :3] @ point_camera + base_camera[:3, 3]

        # 按压轴在板系里是 press_axis。板心模式恒为板系 -Z（+Z 是朝向观察者的
        # 外法线，按压要往里去）；按键模式可以是斜的 —— 按键行程轴和面板法线
        # 不一致时，斜着压会把它别住，所以那是个真输入，不是方便项。
        direction = base_camera[:3, :3] @ (board_camera @ press_axis)
        direction /= np.linalg.norm(direction)

        # ── 码贴正了吗 ────────────────────────────────────────────────────
        # 把码自己的 +Y（本应指向面板的"上坡"）投到 base 系，看它在**面板上**
        # 转了多少。这一项完全静默：码在面板上转 5°，画面几乎没区别、位姿自洽、
        # 重投影也小、连抖动都不变 —— 但 tip_in_board 会指错方向。
        # 偏移 97 mm 时，转 5° 就是 8.5 mm 的落点横偏。
        #
        # ⚠️⚠️ 判据必须是"与**面板上坡方向**的夹角"，**不能**跟 base 竖直比。
        # 板子靠在泡沫上时面板本身就斜（实测这块斜了 27°），码跟着面板一起斜是
        # **正确的** —— 偏移和按压轴都在码的坐标系里，本来就该跟着面板走。
        # 拿 base 竖直去比会把面板倾角也算成"码贴歪了"，让人去纠一个不存在的
        # 问题（我第一版就是这么写的，差点报出去）。
        axes_base = base_camera[:3, :3] @ board_camera
        marker_up = axes_base[:, 1]
        normal = axes_base[:, 2]
        panel_tilt = math.degrees(math.asin(
            max(-1.0, min(1.0, float(normal[2])))))
        projected_up = np.array([0.0, 0.0, 1.0]) - float(normal[2]) * normal
        print(f"面板倾角        = {panel_tilt:+.1f}°  （法线抬离水平面。靠在东西上时"
              f"很大，**这不是误差**）")
        if np.linalg.norm(projected_up) < 1e-6:
            print("码在面板上的转角 = 算不出（面板几乎水平，上坡方向没有定义）")
        else:
            projected_up /= np.linalg.norm(projected_up)
            in_plane = math.degrees(math.acos(max(-1.0, min(
                1.0, float(np.dot(marker_up, projected_up))))))
            offset_mm = float(np.linalg.norm(tip_in_board)) * 1000.0
            print(f"码在面板上的转角 = {in_plane:5.1f}°   （0° = 贴得很正）")
            if in_plane > 90.0:
                print("  ⚠️⚠️ 码在面板上上下颠倒了 —— 偏移方向整个反了，先贴正")
            elif in_plane > 3.0:
                print(f"  ⚠️ 码在面板上转了 {in_plane:.1f}° —— 以 {offset_mm:.0f} mm "
                      f"的偏移算，落点会横偏约 "
                      f"{offset_mm * math.sin(math.radians(in_plane)):.1f} mm。"
                      f"\n     最省事的修法：把码撕下来贴正（一条边平行于墙面的竖线）"
                      f"\n     或者照码自己的边重新量 tip_in_board（见 board.yaml）")

        tip = flange[:3, :3] @ TCP_IN_FLANGE + flange[:3, 3]
        print(f"瞄准点(base系)  = ({target[0]*1000:+7.1f}, {target[1]*1000:+7.1f}, "
              f"{target[2]*1000:+7.1f}) mm")
        print(f"按压对象        = {button_label}")
        print(f"按压方向        = ({direction[0]:+.4f}, {direction[1]:+.4f}, "
              f"{direction[2]:+.4f})")
        print(f"当前杆尖        = ({tip[0]*1000:+7.1f}, {tip[1]*1000:+7.1f}, "
              f"{tip[2]*1000:+7.1f}) mm")
        print(f"需要移动        = {np.linalg.norm(target - tip)*1000:.1f} mm")
        print(f"板系微调        = ({offset[0]*1000:+.1f}, {offset[1]*1000:+.1f}, "
              f"{offset[2]*1000:+.1f}) mm")

        if not (np.all(target >= np.array(WORKSPACE_MIN))
                and np.all(target <= np.array(WORKSPACE_MAX))):
            raise SystemExit("★ 目标在工作空间盒外，任务会拒收，不发")

        if not args.publish:
            print("\n（没给 --publish，只算不发）")
            return

        msg = PoseStamped()
        msg.header.frame_id = args.frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        qx, qy, qz, qw = matrix_to_quat(frame_from_axis(direction))
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw

        deadline = time.time() + 8.0
        while time.time() < deadline and aim.publisher.get_subscription_count() == 0:
            rclpy.spin_once(aim, timeout_sec=0.05)
        subscribers = aim.publisher.get_subscription_count()
        print(f"\n{args.target_topic} 订阅者 = {subscribers}")
        if subscribers == 0:
            raise SystemExit("★ 没有订阅者 —— 按压任务节点没在跑，发了也没人接")

        for _ in range(args.repeats):
            msg.header.stamp = aim.get_clock().now().to_msg()
            aim.publisher.publish(msg)
            rclpy.spin_once(aim, timeout_sec=0.05)
            time.sleep(0.15)
        print(f"已发布（重复 {args.repeats} 次）—— 机械臂马上就要动了")
    except KeyboardInterrupt:
        pass
    finally:
        aim.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
