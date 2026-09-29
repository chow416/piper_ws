#!/usr/bin/env python3
"""Watch the live picture during a press, without fighting over the camera.

`tools/cam_view.py` opens `/dev/video2` directly, and that device is exclusive:
start it while `marker_locator` is running and one of the two loses. During a
press the one that matters is `marker_locator`, because it is the thing feeding
the task its target -- so the camera cannot be shared, and this is the way round
it: `marker_locator` already publishes an annotated frame at
`/perception/debug_image/compressed`, and this relays those bytes to a browser.

It is also the better picture to watch. The camera device gives bare video,
while the debug image has the marker outlines, the board axes and the board
centre drawn on it, so a detection that is about to fail is visible before the
task stops getting targets. And because the compression already happened inside
`marker_locator`, this node never decodes anything -- it moves JPEG bytes from a
ROS subscription into an HTTP body, which is why it has no OpenCV dependency.

Run it inside the container, where ROS lives. The container shares the host
network, so the browser is on the host:

    python3 /ws/tools/debug_view.py            # then open http://127.0.0.1:8098/
    python3 /ws/tools/debug_view.py --port 8098 --topic /perception/debug_image/compressed
"""

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage

#: Matches what `marker_locator` publishes on, so nothing is dropped for QoS
#: reasons and the diagnosis of "nothing appears" stays about topics, not about
#: reliability settings.
CONTRACT_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST,
                          durability=DurabilityPolicy.VOLATILE)

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>marker_locator debug</title>
<style>
  body { margin:0; background:#111; color:#ddd;
         font:13px/1.5 ui-monospace, Menlo, Consolas, monospace; }
  img  { display:block; width:100%; height:auto; image-rendering:auto; }
  #bar { padding:8px 12px; display:flex; gap:24px; flex-wrap:wrap; }
  .k   { color:#888; } .v { color:#eee; }
  .ok  { color:#6c6; } .bad { color:#e66; } .warn { color:#eb3; }
</style></head><body>
<img src="/stream" alt="waiting for /perception/debug_image/compressed ...">
<div id="bar">
  <span><span class="k">帧</span> <span class="v" id="n">-</span></span>
  <span><span class="k">帧龄</span> <span class="v" id="age">-</span></span>
  <span><span class="k">帧率</span> <span class="v" id="hz">-</span></span>
  <span><span class="k">板子</span> <span class="v" id="board">-</span></span>
</div>
<script>
async function tick() {
  try {
    const s = await (await fetch('/status')).json();
    document.getElementById('n').textContent = s.frames;
    const age = document.getElementById('age');
    age.textContent = s.age === null ? '-' : s.age.toFixed(2) + ' s';
    age.className = s.age === null ? 'v' : (s.age < 0.8 ? 'ok' : 'bad');
    document.getElementById('hz').textContent =
        s.hz === null ? '-' : s.hz.toFixed(1) + ' Hz';
    const b = document.getElementById('board');
    if (s.board_z === null) { b.textContent = '没检出'; b.className = 'warn'; }
    else { b.textContent = s.board_z.toFixed(0) + ' mm'; b.className = 'ok'; }
  } catch (e) { /* 页面先于节点起来的瞬间，忽略 */ }
  setTimeout(tick, 400);
}
tick();
</script></body></html>
"""


class Frames:
    """The newest annotated JPEG, plus enough history to notice a stall."""

    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None
        self.at = None
        self.count = 0
        self.stamps = []
        self.board_z = None
        self.board_at = None

    def put(self, data, now):
        with self.lock:
            self.jpeg = data
            self.at = now
            self.count += 1
            self.stamps.append(now)
            self.stamps = self.stamps[-20:]

    def put_board(self, z, now):
        with self.lock:
            self.board_z = z
            self.board_at = now

    def wait_for_new(self, seen, timeout=5.0):
        """Block until the frame counter moves past `seen`."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if self.count > seen and self.jpeg is not None:
                    return seen + 1, self.jpeg, self.at
            time.sleep(0.02)
        return seen, None, None

    def status(self, now):
        with self.lock:
            age = None if self.at is None else now - self.at
            hz = None
            if len(self.stamps) >= 2:
                span = self.stamps[-1] - self.stamps[0]
                if span > 0:
                    hz = (len(self.stamps) - 1) / span
            # 板子超过 1 秒没更新就当没检出，别把陈旧的读数当现状
            board = self.board_z if (self.board_at is not None
                                     and now - self.board_at < 1.0) else None
            return {"frames": self.count, "age": age, "hz": hz, "board_z": board}


class Handler(BaseHTTPRequestHandler):
    frames = None

    def do_GET(self):
        if self.path.startswith("/stream"):
            self._stream()
        elif self.path.startswith("/status"):
            body = json.dumps(self.frames.status(time.monotonic())).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        seen = 0
        try:
            while True:
                seen, jpeg, _ = self.frames.wait_for_new(seen)
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass          # 刷新页面 / 关标签页，正常

    def log_message(self, *_args):
        pass              # 它是视频流，日志只会刷屏


class Relay(Node):
    def __init__(self, frames, args):
        super().__init__("debug_view")
        self.frames = frames
        self.create_subscription(CompressedImage, args.topic,
                                 self._on_image, CONTRACT_QOS)
        self.create_subscription(PoseStamped, args.board_topic,
                                 self._on_board, CONTRACT_QOS)

    def _on_image(self, msg):
        self.frames.put(bytes(msg.data), time.monotonic())

    def _on_board(self, msg):
        p = msg.pose.position
        self.frames.put_board((p.x * p.x + p.y * p.y + p.z * p.z) ** 0.5
                              * 1000.0, time.monotonic())


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/perception/debug_image/compressed")
    parser.add_argument("--board-topic", default="/perception/board_pose")
    parser.add_argument("--port", type=int, default=8098)
    args = parser.parse_args()

    frames = Frames()
    Handler.frames = frames
    rclpy.init()
    node = Relay(frames, args)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"调试图  {args.topic}")
    print(f"浏览器  http://127.0.0.1:{args.port}/")
    print("（这是 marker_locator 发的那张带标注的图，不占相机设备）")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
