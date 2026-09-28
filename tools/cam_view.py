#!/usr/bin/env python3
"""Live view of the Orbbec RGB camera in any browser.

The camera speaks MJPG, which means every frame is already a complete JPEG.
That removes the need for OpenCV: we spawn `v4l2-ctl` and split its stdout on
the JPEG start/end markers, then hand the raw frames to the browser as a
`multipart/x-mixed-replace` stream. Firefox, Chrome and the VS Code embedded
browser all render that natively as video.

The sensor is mounted upside down, so the page rotates the image with a CSS
transform rather than re-encoding every frame. A 1080p rotate+save in Python
would cost about as much as the capture itself; the CSS version is free.

Usage:
    python3 tools/cam_view.py                 # 1920x1080 @ 15 fps on :8099
    python3 tools/cam_view.py --rotate 0      # if the camera gets re-mounted
    python3 tools/cam_view.py --width 640 --height 480 --fps 30
    python3 tools/cam_view.py --snapshot /tmp/shot.jpg   # one frame, no server
"""

import argparse
import glob
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WARMUP_FRAMES = 30


def resolve_device(requested):
    """Fall back to the stable by-id path when the video node has moved.

    `/dev/videoN` is assigned in enumeration order, so it changes whenever the
    camera is replugged or the dock comes up in a different sequence. The by-id
    symlink always points at the same physical camera, which is what a recorded
    command should depend on.
    """
    if os.path.exists(requested):
        return requested
    candidates = sorted(glob.glob("/dev/v4l/by-id/*Orbbec*video-index0"))
    if candidates:
        print(f"note: {requested} not present; using {candidates[0]}",
              file=sys.stderr)
        return candidates[0]
    print(f"error: neither {requested} nor any /dev/v4l/by-id/*Orbbec* exists.\n"
          f"       run `ls -l /dev/v4l/by-id/` and `lsusb | grep -i orbbec`.",
          file=sys.stderr)
    return None

PAGE = """<!doctype html>
<meta charset="utf-8">
<title>{device} {width}x{height} @{fps}</title>
<style>
  html,body {{ margin:0; height:100%; background:#111; overflow:hidden;
               font:13px/1.4 system-ui,sans-serif; color:#ddd; }}
  #wrap {{ display:flex; flex-direction:column; height:100%; }}
  #bar {{ padding:6px 10px; background:#1c1c20; display:flex; gap:14px;
          align-items:center; }}
  #bar b {{ color:#7fd67f; }}
  #view {{ flex:1; display:flex; align-items:center; justify-content:center;
           overflow:hidden; }}
  img {{ max-width:100%; max-height:100%;
         transform:rotate({rotate}deg); }}
</style>
<div id="wrap">
  <div id="bar">
    <b>{device}</b>
    <span>{width}&times;{height} MJPG @{fps} fps</span>
    <span>rotation {rotate}&deg;</span>
    <span id="st">connecting...</span>
  </div>
  <div id="view"><img id="im" src="/stream"></div>
</div>
<script>
  const im = document.getElementById('im'), st = document.getElementById('st');
  im.onload  = () => st.textContent = 'streaming';
  im.onerror = () => st.textContent = 'stream ended - restart the script';
</script>
"""


# Last line v4l2-ctl printed when it died. The retry loop reports it once
# instead of letting the tool print it on every attempt.
STREAM_ERROR = ""


def frame_source(device, width, height, fps, pixelformat):
    """Yield raw JPEG frames from v4l2-ctl, skipping the AE/AWB warm-up."""
    global STREAM_ERROR
    cmd = [
        "v4l2-ctl", "-d", device,
        f"--set-fmt-video=width={width},height={height},pixelformat={pixelformat}",
        f"--set-parm={fps}",
        "--stream-mmap", "--stream-to=-",
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, bufsize=0)
    buf = bytearray()
    seen = 0
    try:
        while True:
            chunk = proc.stdout.read(1 << 16)
            if not chunk:
                return
            buf += chunk
            while True:
                start = buf.find(b"\xff\xd8")
                if start < 0:
                    del buf[:-1]          # a marker may straddle the boundary
                    break
                end = buf.find(b"\xff\xd9", start + 2)
                if end < 0:
                    if start:
                        del buf[:start]
                    break
                frame = bytes(buf[start:end + 2])
                del buf[:end + 2]
                seen += 1
                if seen > WARMUP_FRAMES:
                    yield frame
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
        if proc.stderr is not None:
            text = proc.stderr.read().decode(errors="replace").strip()
            STREAM_ERROR = text.splitlines()[-1] if text else ""


class Broker:
    """Fan-out: one camera reader, many browser clients."""

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.frame = None
        self.cond = threading.Condition()
        self.count = 0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        """Keep the stream alive, retrying once a second when it drops.

        v4l2-ctl exits immediately when the device is gone or already open, so
        retrying without a delay spins and floods the console -- which is
        exactly what happened the first time the camera was unplugged
        mid-session and the log filled with `Cannot open device /dev/video2`.
        """
        failing = False
        while True:
            received = 0
            try:
                for frame in frame_source(**self.kwargs):
                    received += 1
                    if failing:
                        print(f"{self.kwargs['device']}: stream recovered",
                              file=sys.stderr)
                        failing = False
                    with self.cond:
                        self.frame = frame
                        self.count += 1
                        self.cond.notify_all()
                reason = STREAM_ERROR or f"stream ended after {received} frames"
            except Exception as exc:                       # noqa: BLE001
                reason = f"camera reader failed: {exc}"

            if not failing:
                print(f"{self.kwargs['device']}: {reason}\n"
                      "    Unplugged, or already open in another process?\n"
                      f"    fuser -v {self.kwargs['device']} names the other user.\n"
                      "    Retrying every second.", file=sys.stderr)
                failing = True
            time.sleep(1.0)

    def next(self, last_count, timeout=5.0):
        with self.cond:
            self.cond.wait_for(lambda: self.count != last_count, timeout)
            return self.frame, self.count


def make_handler(broker, page):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):                      # keep the console quiet
            pass

        def do_GET(self):
            if self.path.startswith("/stream"):
                self._stream()
            elif self.path.startswith("/snapshot"):
                if broker.frame:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(broker.frame)))
                    self.end_headers()
                    self.wfile.write(broker.frame)
                else:
                    self.send_error(503, "no frame yet")
            else:
                body = page.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            count = -1
            try:
                while True:
                    frame, count = broker.next(count)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/video2")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--pixelformat", default="MJPG")
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--rotate", type=int, default=180,
                    help="CSS rotation in degrees; the sensor is mounted upside down")
    ap.add_argument("--snapshot", metavar="FILE",
                    help="write one warmed-up frame and exit, no server")
    args = ap.parse_args()

    args.device = resolve_device(args.device)
    if args.device is None:
        return 1

    if args.snapshot:
        print(f"waiting for {WARMUP_FRAMES} warm-up frames...")
        for frame in frame_source(args.device, args.width, args.height,
                                  args.fps, args.pixelformat):
            with open(args.snapshot, "wb") as handle:
                handle.write(frame)
            print(f"{len(frame)} bytes -> {args.snapshot}"
                  f"  (rotate {args.rotate} deg to view)")
            return 0
        print("no frames received", file=sys.stderr)
        return 1

    broker = Broker(device=args.device, width=args.width, height=args.height,
                    fps=args.fps, pixelformat=args.pixelformat)
    page = PAGE.format(device=args.device, width=args.width, height=args.height,
                       fps=args.fps, rotate=args.rotate)
    handler = make_handler(broker, page)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)

    print(f"warming up ({WARMUP_FRAMES} frames, AE/AWB needs ~2-3 s)...")
    deadline = time.time() + 15
    while broker.frame is None and time.time() < deadline:
        time.sleep(0.2)
    if broker.frame is None:
        print("no frames after 15 s - is the camera busy?", file=sys.stderr)
        return 1

    print(f"\n  live view:  http://127.0.0.1:{args.port}/")
    print(f"  snapshot:   http://127.0.0.1:{args.port}/snapshot")
    print(f"  rotation:   {args.rotate} deg (CSS, no re-encoding)")
    print("\nCtrl-C to stop.\n")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
