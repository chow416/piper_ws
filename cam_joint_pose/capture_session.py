#!/usr/bin/env python3
"""Interactive capture session: one keypress per pose, for hand-eye calibration.

Everything here is glue. `camera_capture.Camera` already knows how to get a
correct frame out of the DaBai, and `joint_state_reader` already knows how to
read the joints by name and look the tool pose up in the live TF tree. This
script only orders them, which is the part that decides whether the data is
any good:

    Enter -> settle -> open the shutter -> re-check the joints -> TF -> save

That order is not a style choice. This path has no hardware capture timestamp,
so "the arm was stationary" cannot be measured, only *enforced*: the settle
wait happens before the frame, and the drift check after it catches the arm
that started moving again in between.

Output
------
    <out>/pose_000.jpg     raw frame (unrotated unless --rotate-180)
    <out>/pose_000.json    that pose's record
    <out>/poses.json       the whole session

`poses.json` is rewritten atomically after every capture, so Ctrl-C, a crash,
or a sagging arm never costs more than the pose in flight. Re-running the
script resumes the session; `--fresh` starts over.

Before running
--------------
`robot_state_publisher` must be alive (it comes from the hardware bringup) or
the TF lookup has nothing to answer with. And the camera is exclusive: stop
`tools/cam_view.py` first, or this script cannot open the device at all.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy

# Run from anywhere: the two sibling modules are imported by plain name.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_capture import Camera, DEFAULT_WARMUP_FRAMES      # noqa: E402
from joint_state_reader import (DEFAULT_BASE_FRAME,           # noqa: E402
                                DEFAULT_JOINTS, DEFAULT_TOOL_FRAME,
                                JointStateReader, ToolPoseProvider)

#: If a new pose's tool orientation differs from *every* existing pose by less
#: than this, it barely constrains AX = XB. The solve stays well conditioned
#: only when the capture set rotates about several different axes.
MIN_ROTATION_SPREAD_DEG = 15.0

#: How many times to redo settle+grab before giving up on a pose.
MAX_ATTEMPTS = 2

HELP_TEXT = """\
  回车        采一组（自动等停稳 → 抓图 → 复核 → 存盘）
  l          列出已采集的位姿
  d          删掉最后一组（存错了用这个）
  h / ?      显示这段帮助
  q          保存并退出

  采集前确认：
    · 标定板完整在画面里、占得够大
    · 这个位姿的旋转和之前几组明显不同（绕不同轴转）
    · 机械臂已经松手、不在动
"""


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="calib_data",
                        help="session directory (created if missing)")
    parser.add_argument("--target", type=int, default=20,
                        help="how many poses you are aiming for; display only")
    parser.add_argument("--fresh", action="store_true",
                        help="discard an existing session in --out and start over")

    group = parser.add_argument_group("camera")
    group.add_argument("--device", default="/dev/video2")
    group.add_argument("--width", type=int, default=1920)
    group.add_argument("--height", type=int, default=1080)
    group.add_argument("--fps", type=int, default=15)
    group.add_argument("--warmup", type=int, default=DEFAULT_WARMUP_FRAMES,
                       help="AE/AWB frames to throw away at startup")
    group.add_argument("--rotate-180", action="store_true",
                       help="rotate frames 180 deg. Pick one convention and "
                            "stay with it -- the intrinsics must be calibrated "
                            "on frames processed the same way (see README)")

    group = parser.add_argument_group("motion checks")
    group.add_argument("--settle-tolerance", type=float, default=0.002,
                       help="rad; a sample counts as still below this")
    group.add_argument("--settle-timeout", type=float, default=8.0,
                       help="seconds to wait for the arm to stop")
    group.add_argument("--drift-tolerance", type=float, default=0.01,
                       help="rad; max joint movement tolerated across the grab")
    group.add_argument("--no-diversity-check", action="store_true",
                       help="stop warning about poorly spread rotations")
    return parser.parse_args(argv)


# -- geometry ------------------------------------------------------------

def rotation_angle_deg(a, b):
    """Rotation angle, in degrees, between two 3x3 rotation matrices."""
    cosine = (np.trace(a @ b.T) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def closest_existing_deg(rotation, rotations):
    """Smallest angle between `rotation` and anything already captured."""
    if not rotations:
        return None
    return min(rotation_angle_deg(rotation, other) for other in rotations)


def matrix_of(sample):
    return np.asarray(sample["T_base_tool"], dtype=float)


# -- session file --------------------------------------------------------

def save_session(path, payload):
    """Write atomically -- a Ctrl-C mid-write must not corrupt the session."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def next_pose_index(samples):
    """Continue numbering after the highest pose already on disk."""
    highest = -1
    for sample in samples:
        try:
            highest = max(highest, int(str(sample["name"]).rsplit("_", 1)[-1]))
        except (KeyError, ValueError):
            continue
    return highest + 1


def print_summary(payload):
    samples = payload.get("samples", [])
    print(f"\n  已采集 {len(samples)} 组")
    if not samples:
        return

    matrices = [matrix_of(s) for s in samples]
    rotations = [m[:3, :3] for m in matrices]
    positions = np.array([m[:3, 3] for m in matrices])

    if len(samples) >= 2:
        angles = [rotation_angle_deg(rotations[i], rotations[j])
                  for i in range(len(rotations))
                  for j in range(i + 1, len(rotations))]
        angles.sort()
        middle = angles[len(angles) // 2]
        spread = max(float(np.linalg.norm(positions[i] - positions[j]))
                     for i in range(len(positions))
                     for j in range(i + 1, len(positions)))
        print(f"  两两旋转夹角   最小 {angles[0]:5.1f}°  中位 {middle:5.1f}°")
        print(f"  工具位置跨度        {spread * 1000:6.1f} mm")
        if angles[0] < MIN_ROTATION_SPREAD_DEG:
            print(f"  ⚠️  有两位姿旋转几乎一样（{angles[0]:.1f}°）—— "
                  f"补采时多绕别的轴转")
    for sample in samples:
        t = matrix_of(sample)[:3, 3]
        print(f"    {sample['name']}   "
              f"({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) m")


# -- the capture loop ----------------------------------------------------

def capture_loop(args, reader, poses, camera, payload, session_path):
    rotations = [matrix_of(s)[:3, :3] for s in payload["samples"]]
    out_dir = os.path.dirname(session_path)

    while True:
        taken = len(payload["samples"])
        shown = f"{taken + 1}/{args.target}" if taken < args.target else str(taken + 1)
        try:
            answer = input(f"\n[{shown}] 摆好位姿后回车采集 "
                           f"(l=列表 d=删最后一个 h=帮助 q=退出) > ")
        except EOFError:
            answer = "q"
        answer = answer.strip().lower()

        if answer in ("q", "quit", "exit"):
            return
        if answer in ("h", "?", "help"):
            print(HELP_TEXT)
            continue
        if answer in ("l", "list"):
            print_summary(payload)
            continue
        if answer == "d":
            if not payload["samples"]:
                print("  还没有可删的组")
                continue
            dropped = payload["samples"].pop()
            for suffix in (".jpg", ".json"):
                path = os.path.join(out_dir, dropped["name"] + suffix)
                if os.path.exists(path):
                    os.remove(path)
            rotations = [matrix_of(s)[:3, :3] for s in payload["samples"]]
            save_session(session_path, payload)
            print(f"  已删除 {dropped['name']}")
            continue
        if answer:
            print(f"  不认识的命令 {answer!r}；回车采集，h 看帮助")
            continue

        # ---- one capture ------------------------------------------------
        captured = None
        for attempt in range(1, MAX_ATTEMPTS + 1):
            # ① The arm must be standing still *before* the shutter opens.
            try:
                joints, joint_stamp, waited = reader.wait_until_settled(
                    tolerance_rad=args.settle_tolerance,
                    timeout=args.settle_timeout)
            except RuntimeError as exc:
                print(f"  ✗ {exc}")
                break

            # ② Now grab.
            result = camera.grab()
            if result is None:
                print("  ✗ 抓不到图：相机被拔了，或被别的进程占着。\n"
                      f"    用 `sudo fuser -v {camera.device}` 看是谁。")
                return
            jpeg, wall_time = result

            # ③ Prove it *stayed* still. Settling only says the arm was
            #    stationary a moment ago; this catches it having started to
            #    move again while the frame was being assembled.
            try:
                after, _ = reader.wait_for_joints(timeout=2.0)
                drift = max(abs(after[name] - joints[name])
                            for name in reader.joint_names)
            except RuntimeError:
                drift = float("nan")

            if math.isnan(drift):
                print("  ⚠️  抓图后读不到关节角，没法复核静止性 —— 这组不要")
                break
            if drift > args.drift_tolerance:
                print(f"  ⚠️  抓图前后关节动了 {drift:.4f} rad "
                      f"({math.degrees(drift):.2f}°)，超过 "
                      f"{args.drift_tolerance} —— 这组作废")
                if attempt < MAX_ATTEMPTS:
                    print("     松手等它停稳，我自动重采一次...")
                    continue
                break

            captured = (joints, joint_stamp, waited, jpeg, wall_time, drift)
            break

        if captured is None:
            print("  这次没采成，重新摆位姿再来")
            continue
        joints, joint_stamp, waited, jpeg, wall_time, drift = captured

        # ④ Tool pose. The arm has not moved, so "latest" is the right time.
        matrix = poses.lookup()
        if matrix is None:
            print("  ✗ 查不到 TF（robot_state_publisher 在跑吗？）—— 这组丢弃")
            continue

        # ⑤ Tell the operator when a pose adds little (see the constant).
        rotation = matrix[:3, :3]
        nearest = closest_existing_deg(rotation, rotations)
        if (nearest is not None and nearest < MIN_ROTATION_SPREAD_DEG
                and not args.no_diversity_check):
            print(f"  ⚠️  这个位姿和已有位姿只差 {nearest:.1f}°（< "
                  f"{MIN_ROTATION_SPREAD_DEG:.0f}°），对解手眼帮助很小。\n"
                  f"     建议取消：在这个提示下按 d 删掉，换个绕不同轴的姿态")

        # ⑥ Persist. Both files, every time.
        name = f"pose_{next_pose_index(payload['samples']):03d}"
        with open(os.path.join(out_dir, name + ".jpg"), "wb") as handle:
            handle.write(jpeg)
        sample = {
            "name": name,
            "image": name + ".jpg",
            "joints": {key: float(value) for key, value in joints.items()},
            "joint_stamp": float(joint_stamp),
            "wall_time": float(wall_time),
            "T_base_tool": matrix.tolist(),        # 4x4, row-major
            "settle_seconds": round(waited, 3),
            "post_grab_drift_rad": round(drift, 6),
            "closest_existing_deg": (None if nearest is None
                                     else round(nearest, 2)),
        }
        with open(os.path.join(out_dir, name + ".json"), "w",
                  encoding="utf-8") as handle:
            json.dump(sample, handle, indent=2, ensure_ascii=False)
        payload["samples"].append(sample)
        payload["updated"] = time.time()
        save_session(session_path, payload)
        rotations.append(rotation)

        t = matrix[:3, 3]
        line = (f"  ✓ {name}  停稳 {waited:.1f}s  漂移 {drift:.4f} rad  "
                f"位置 ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}) m")
        if nearest is not None:
            line += f"  距最近位姿 {nearest:.1f}°"
        print(line)
        if len(payload["samples"]) == args.target:
            print(f"\n  🎯 到 {args.target} 组了。够不够看旋转分散度：l 看一眼，"
                  f"然后 q 退出。")


# -- entry point ---------------------------------------------------------

def main(argv=None):
    args = parse_args(argv)
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)
    session_path = os.path.join(out_dir, "poses.json")

    if args.fresh:
        for entry in sorted(os.listdir(out_dir)):
            if entry == "poses.json" or (
                    entry.startswith("pose_")
                    and entry.endswith((".json", ".jpg"))):
                os.remove(os.path.join(out_dir, entry))
                print(f"  已清除 {entry}")

    payload = None
    if os.path.exists(session_path) and not args.fresh:
        try:
            with open(session_path, encoding="utf-8") as handle:
                payload = json.load(handle)
        except (OSError, ValueError) as exc:
            print(f"error: 读不了 {session_path}: {exc}\n"
                  f"       要重来就用 --fresh", file=sys.stderr)
            return 1
        if bool(payload.get("rotate_180")) != args.rotate_180:
            print("error: 这个 session 是用"
                  f"{' --rotate-180' if payload.get('rotate_180') else '未旋转'} "
                  "采的，和当前参数不一致。\n"
                  "       混用会让主点差一整个图幅 —— 要么加上/去掉"
                  " --rotate-180，要么 --fresh 重采。", file=sys.stderr)
            return 1
        print(f"  继续已有 session：{len(payload.get('samples', []))} 组")

    if payload is None:
        payload = {
            "created": time.time(),
            "tool_frame": DEFAULT_TOOL_FRAME,
            "base_frame": DEFAULT_BASE_FRAME,
            "rotate_180": args.rotate_180,
            "samples": [],
        }
    payload["updated"] = time.time()
    payload["joint_order"] = list(DEFAULT_JOINTS)

    rclpy.init()
    reader = None
    try:
        reader = JointStateReader()
        poses = ToolPoseProvider(reader)

        # Fail fast, before opening the exclusive device.
        try:
            reader.wait_for_joints(timeout=15.0)
        except RuntimeError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1

        payload["base_frame"] = poses.base_frame
        payload["tool_frame"] = poses.tool_frame
        payload["joint_order"] = list(reader.joint_names)
        payload.setdefault("samples", [])       # tolerate a hand-edited file

        try:
            camera = Camera(device=args.device, width=args.width,
                            height=args.height, fps=args.fps,
                            warmup_frames=args.warmup,
                            rotate_180=args.rotate_180)
        except RuntimeError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        payload["camera"] = {
            "device": camera.device,
            "width": args.width,
            "height": args.height,
            "fps": args.fps,
        }

        print("=" * 62)
        print("  手眼标定位姿采集")
        print("-" * 62)
        print(f"  输出目录   {out_dir}/")
        print(f"  目标组数   {args.target}"
              f"（已有 {len(payload['samples'])}）")
        print(f"  相机       {camera.device}  {args.width}x{args.height} "
              f"@{args.fps}  "
              f"{'已旋转 180°' if args.rotate_180 else 'raw（未旋转）'}")
        print(f"  工具系     {poses.describe_frames()}")
        print("=" * 62)

        try:
            capture_loop(args, reader, poses, camera, payload, session_path)
        except KeyboardInterrupt:
            print("\n\n  已中断")
        finally:
            camera.close()
            save_session(session_path, payload)
            print_summary(payload)
            print(f"\n  数据在 {out_dir}/")
            print(f"    poses.json     整个 session")
            print(f"    pose_NNN.jpg   原始图（务必和标定内参用的是同一批）")
            print(f"    pose_NNN.json  每组一份")
            print(f"\n  下一步见 README 的「下一步」一节（内参 → solvePnP → "
                  f"calibrateHandEye）")
        return 0
    finally:
        if reader is not None:
            reader.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
