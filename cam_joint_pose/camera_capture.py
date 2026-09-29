#!/usr/bin/env python3
"""RGB capture for the Orbbec DaBai DCW2, for hand-eye calibration.

Why this does not use ROS
-------------------------
No ROS camera driver is installed in this workspace, and the depth sensor of
this camera is not a UVC device. The *RGB* sensor is plain UVC though, so
`v4l2-ctl` can read it directly. Keeping capture outside ROS also means this
script works before the hardware bringup is running.

Why this does not use OpenCV
----------------------------
The camera is configured for MJPG, in which every frame is already a complete
JPEG. Splitting the byte stream on the JPEG start/end markers recovers the
frames with no decoding and no re-encoding. OpenCV is only needed later, in the
calibration itself.

Timestamps
----------
This path gives no hardware capture time, so each frame is stamped when it is
assembled in userspace. That is only acceptable while the arm is *stationary*,
which is how hand-eye calibration has to be done anyway: move, wait for the
joints to settle, then capture. Never capture mid-motion.

The 180 degree sensor roll
--------------------------
The sensor is mounted upside down inside the housing, so raw frames come out
rotated by 180 degrees. Two self-consistent ways to deal with it:

  raw (default)  Do not rotate. Calibrate the camera intrinsics on raw frames
                 too. The 180 degrees is then absorbed into the hand-eye
                 result, which is perfectly fine as long as the perception node
                 uses that same result.

  --rotate-180   Rotate, so frames match the nominal optical frame in the URDF.
                 The intrinsics must be calibrated on the *rotated* frames:
                 the principal point becomes (W-1-cx, H-1-cy).

Pick one and stay consistent. Mixing rotated images with raw intrinsics (or the
datasheet values) shifts the principal point by a whole image dimension.
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

# The AE/AWB loop needs roughly two seconds of running frames before exposure
# and white balance stop drifting. Calibration must not see that transient.
DEFAULT_WARMUP_FRAMES = 30

JPEG_START = b"\xff\xd8"
JPEG_END = b"\xff\xd9"

# "Undefined" byte used by v4l2-ctl when it cannot open the device. Reported
# once instead of on every retry.
STREAM_ERROR = ""


def resolve_device(requested):
    """Return a usable device path, falling back to the stable by-id symlink.

    `/dev/videoN` is assigned in enumeration order, so it changes whenever the
    camera is replugged or the dock comes up in a different sequence. The by-id
    path always points at the same physical camera.
    """
    if requested and os.path.exists(requested):
        return requested
    candidates = sorted(glob.glob("/dev/v4l/by-id/*Orbbec*video-index0"))
    if candidates:
        print(f"note: {requested} not present; using {candidates[0]}",
              file=sys.stderr)
        return candidates[0]
    return None


class Camera:
    """Blocking RGB capture. Use as a context manager.

        with Camera() as cam:
            jpeg = cam.grab()
    """

    def __init__(self, device=None, width=1920, height=1080, fps=15,
                 pixel_format="MJPG", payload="mjpg",
                 warmup_frames=DEFAULT_WARMUP_FRAMES, rotate_180=False):
        self.width = width
        self.height = height
        self.rotate_180 = rotate_180
        self.warmup_frames = warmup_frames
        self.payload = payload
        self.device = resolve_device(device or "/dev/video2")
        if self.device is None:
            raise RuntimeError(
                "no camera found. Check `lsusb | grep -i orbbec` and "
                "`ls -l /dev/v4l/by-id/`.")

        cmd = [
            "v4l2-ctl", "-d", self.device,
            f"--set-fmt-video=width={width},height={height},"
            f"pixelformat={pixel_format}",
            f"--set-parm={fps}",
            "--stream-mmap", "--stream-to=-",
        ]
        self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                      stderr=subprocess.PIPE, bufsize=0)
        self._buf = bytearray()
        self._delivered = 0
        self._warm = False

    # -- framing ---------------------------------------------------------

    def _next_frame(self, timeout=5.0):
        """Block until one complete JPEG is available, or return None."""
        deadline = time.monotonic() + timeout
        while True:
            while True:
                start = self._buf.find(JPEG_START)
                if start < 0:
                    del self._buf[:-1]      # a marker may straddle the read
                    break
                end = self._buf.find(JPEG_END, start + 2)
                if end < 0:
                    if start:
                        del self._buf[:start]
                    break
                frame = bytes(self._buf[start:end + 2])
                del self._buf[:end + 2]
                return frame
            if timeout is not None and time.monotonic() >= deadline:
                return None
            chunk = self._proc.stdout.read(1 << 16)
            if not chunk:
                self._record_error()
                return None
            self._buf += chunk

    def _record_error(self):
        global STREAM_ERROR
        if self._proc.stderr is None:
            return
        text = self._proc.stderr.read().decode(errors="replace").strip()
        STREAM_ERROR = text.splitlines()[-1] if text else ""

    # -- public API ------------------------------------------------------

    def grab(self, timeout=5.0):
        """Return (jpeg_bytes, timestamp) with the warm-up already consumed.

        `jpeg_bytes` is a complete JPEG, ready to be written to disk. The
        timestamp is `time.time()`, i.e. when this process assembled the frame,
        not a hardware capture time -- see the module docstring.
        """
        while True:
            frame = self._next_frame(timeout)
            if frame is None:
                return None
            if not self._warm:
                self._delivered += 1
                if self._delivered <= self.warmup_frames:
                    continue
                self._warm = True
            if self.rotate_180:
                frame = _rotate180(frame)
            return frame, time.time()

    def close(self):
        if getattr(self, "_proc", None) is None:
            return
        self._proc.terminate()
        try:
            self._proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self._proc.kill()
        self._record_error()
        self._proc = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def _rotate180(jpeg):
    """Lossless 180 degree rotation -- a pure index flip, no resampling."""
    try:
        import io
        from PIL import Image
    except ImportError as exc:                             # pragma: no cover
        raise RuntimeError(
            "--rotate-180 needs Pillow (`pip install pillow`, or drop the "
            "flag and calibrate on raw frames instead)") from exc
    image = Image.open(io.BytesIO(jpeg)).rotate(180, expand=True)
    out = io.BytesIO()
    image.save(out, format="JPEG", quality=95)
    return out.getvalue()


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--pixel-format", default="MJPG")
    parser.add_argument("--out", default="capture",
                        help="output directory for the JPEG frames")
    parser.add_argument("--count", type=int, default=1,
                        help="how many frames to save")
    parser.add_argument("--interval", type=float, default=0.0,
                        help="seconds between saves (0 = as fast as possible)")
    parser.add_argument("--warmup", type=int, default=DEFAULT_WARMUP_FRAMES)
    parser.add_argument("--rotate-180", action="store_true",
                        help="rotate 180 deg (see the module docstring)")
    args = parser.parse_args()

    try:
        camera = Camera(device=args.device, width=args.width,
                        height=args.height, fps=args.fps,
                        pixel_format=args.pixel_format,
                        warmup_frames=args.warmup,
                        rotate_180=args.rotate_180)
    except RuntimeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    os.makedirs(args.out, exist_ok=True)
    manifest = []
    print(f"{camera.device}  {args.width}x{args.height} @{args.fps} "
          f"{args.pixel_format}"
          f"{'  rotated 180' if args.rotate_180 else '  raw'}")
    print(f"discarding {args.warmup} warm-up frames (AE/AWB)...")

    try:
        for index in range(args.count):
            result = camera.grab()
            if result is None:
                print(f"error: no frame received. {STREAM_ERROR}\n"
                      "    Is the camera unplugged, or already open in another\n"
                      f"    process? `fuser -v {camera.device}` names the user.",
                      file=sys.stderr)
                return 1
            jpeg, stamp = result
            name = f"frame_{index:04d}.jpg"
            path = os.path.join(args.out, name)
            with open(path, "wb") as handle:
                handle.write(jpeg)
            manifest.append({"file": name, "bytes": len(jpeg),
                             "wall_time": stamp})
            print(f"  {name}  {len(jpeg)} bytes")
            if args.interval and index + 1 < args.count:
                time.sleep(args.interval)
    finally:
        camera.close()

    meta = {
        "device": camera.device,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "pixel_format": args.pixel_format,
        "rotated_180": args.rotate_180,
        "warmup_frames": args.warmup,
        "frames": manifest,
    }
    with open(os.path.join(args.out, "capture.json"), "w") as handle:
        json.dump(meta, handle, indent=2)
    print(f"\n{len(manifest)} frame(s) -> {args.out}/  (see capture.json)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
