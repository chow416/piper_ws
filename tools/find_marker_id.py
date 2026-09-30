#!/usr/bin/env python3
"""Print the ArUco IDs the camera can actually see, and how big they look.

Written for one specific confusion. `marker_locator` says

    not publishing: no board (need 1 markers)

which reads like "the camera sees nothing". The usual cause is that the camera
sees a marker whose ID is **not** in `board.yaml`: the detector filters by ID, so
a mismatch looks exactly like an empty view in that log line. A second, quieter
cause is a marker that is there but too small, too edge-on, or too blurry to
decode.

So this ignores the config entirely and reports what is in the image: every ID it
finds, across the dictionaries worth trying, with the marker's pixel size, how
square it looks, and where it sits in the frame.

    python3 tools/find_marker_id.py                    # grab a live frame
    python3 tools/find_marker_id.py --image snap.jpg   # or read a file

The camera is exclusive: stop marker_locator / cam_view first.

Reading the output
------------------
  * "一个都没检测到" -> the marker really is not visible to this camera.
  * an ID shows up here but not in the log -> that ID is what belongs in
    `board.yaml`'s `first_marker_id` / `accepted_ids`.
  * 边长 in px well under ~40 -> it will decode unreliably at this distance.
    Move closer or print it bigger.
  * 方度 far from 1.00 -> seen at a steep angle; the pose it gives will be poor
    even when the ID reads fine.
"""

import argparse
import os
import subprocess
import sys

import cv2
import numpy as np

#: Ordered by likelihood, not alphabetically.
DICTIONARIES = [
    ("DICT_4X4_50", cv2.aruco.DICT_4X4_50),
    ("DICT_5X5_100", cv2.aruco.DICT_5X5_100),
    ("DICT_6X6_250", cv2.aruco.DICT_6X6_250),
    ("DICT_4X4_100", cv2.aruco.DICT_4X4_100),
]

#: Below this many pixels on a side, small markers start dropping out.
SIDE_WARN_PX = 40.0


def grab_frames(device, width, height, count, timeout_s):
    """One raw MJPG stream, split into JPEGs.

    Same trick as cam_view.py: in MJPG every frame is already a complete JPEG,
    so FFD8/FFD9 *is* the container format and no decoder is needed to split.
    """
    result = subprocess.run(
        ["v4l2-ctl", "-d", device,
         f"--set-fmt-video=width={width},height={height},pixelformat=MJPG",
         "--stream-mmap", f"--stream-count={count}", "--stream-to=/dev/stdout"],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=timeout_s)
    blob = result.stdout
    frames = []
    cursor = 0
    while True:
        start = blob.find(b"\xff\xd8", cursor)
        if start < 0:
            break
        end = blob.find(b"\xff\xd9", start)
        if end < 0:
            break
        frames.append(blob[start:end + 2])
        cursor = end + 2
    return frames


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--image", default="",
                        help="读一张现成的图，而不是开相机")
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--warmup", type=int, default=30,
                        help="丢弃前这么多帧（AE/AWB 预热，默认 %(default)s）")
    args = parser.parse_args()

    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            raise SystemExit(f"读不到 {args.image}")
    else:
        if not os.path.exists(args.device):
            raise SystemExit(f"没有 {args.device}")
        print(f"抓帧 {args.device} {args.width}x{args.height}"
              f"（丢前 {args.warmup} 帧等 AE/AWB）…")
        jpegs = grab_frames(args.device, args.width, args.height,
                            args.warmup + 5, 30.0)
        if len(jpegs) <= args.warmup:
            raise SystemExit(f"只抓到 {len(jpegs)} 帧，不够预热"
                             "（相机被别的进程占着？）")
        image = cv2.imdecode(np.frombuffer(jpegs[-1], np.uint8),
                             cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit("最后一帧解不开")

    height, width = image.shape[:2]
    print(f"图像 {width}x{height}")

    params = cv2.aruco.DetectorParameters_create()
    # 不关：关掉精度差一个数量级（实测过）。
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    found = False
    near_edge = False
    for name, dictionary in DICTIONARIES:
        detector = cv2.aruco.Dictionary_get(dictionary)
        corners, ids, _ = cv2.aruco.detectMarkers(image, detector,
                                                  parameters=params)
        if ids is None or not len(ids):
            continue
        found = True
        print(f"\n{name}:")
        for quad, marker_id in zip(corners, ids.flatten()):
            points = quad.reshape(4, 2)
            sides = [float(np.linalg.norm(points[i] - points[(i + 1) % 4]))
                     for i in range(4)]
            centre = points.mean(axis=0)
            margin = min(points[:, 0].min(), points[:, 1].min(),
                         width - points[:, 0].max(),
                         height - points[:, 1].max())
            note = ""
            if max(sides) < SIDE_WARN_PX:
                note += "  ← 太小，解码会不稳"
            if min(sides) > 0 and max(sides) / min(sides) > 1.6:
                note += "  ← 斜得厉害，位姿会差"
            if margin < 8:
                near_edge = True
                note += "  ← 贴到画面边缘"
            print(f"  ID {int(marker_id):3d}  边长 {min(sides):5.1f}"
                  f"–{max(sides):5.1f} px  方度 {max(sides)/min(sides):.2f}"
                  f"  中心 ({centre[0]:5.1f}, {centre[1]:5.1f}){note}")

    if not found:
        print("\n✗ 一个都没检测到 —— 相机里确实看不到码。检查："
              "\n    ① 镜头有没有被挡住 / 对焦"
              "\n    ② 码是不是在画面里（可以先 tools/cam_view.py 看一眼）"
              "\n    ③ 码有没有印糊、或者贴在反光面上")
        return 1

    print("\n把上面出现的 ID 填进 board.yaml 的 first_marker_id 和 accepted_ids。")
    if near_edge:
        print("⚠️ 有码贴在画面边缘 —— 稍微移一下，边缘的角点容易解丢。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
