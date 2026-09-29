# cam_joint_pose — 相机采图 + 关节/位姿获取

给手眼标定用的两个模块：**相机 RGB 采集** 和 **机械臂关节角 / 工具位姿获取**。

这两块是标定的输入端。标定本身（`solvePnP` + `calibrateHandEye`）不在这里。

---

## 文件清单

```
cam_joint_pose/
├── README.md                 ← 本文件
├── QUICKSTART.md             ← ★★ 一页命令速查（拿到包先看这个）
├── WORKFLOW_FOR_COLLEAGUE.md ← 完整交接流程（从收货到交付标定结果）
├── capture_session.py        ← ★ 采集就用这个：交互式，一组一回车
├── camera_capture.py         ← 相机 RGB 采集（可直接跑，也可当模块 import）
├── joint_state_reader.py     ← 关节角 + 工具位姿（可当模块 import）
└── reference/                ← 现有生产代码原文，供对照
    ├── go_home.py            ← 关节角读取的**标准写法**（注意第 112-124 行）
    └── send_joint_test.py    ← 另一个更短的关节角读取例子
```

`reference/` 里的是**从生产代码里直接拷来的原文**，不参与运行，只是让你看到
"实际在用"的写法长什么样。真正要用的是上面两个模块。

---

## 两个模块在干什么

### `camera_capture.py` —— 相机

```python
from camera_capture import Camera

with Camera() as cam:          # 自动找设备、自动丢弃预热帧
    jpeg, wall_time = cam.grab()
```

- **不依赖 ROS**：设备是标准 UVC，直接用 `v4l2-ctl` 读
- **不依赖 OpenCV**：配成 MJPG，每帧本身就是完整 JPEG，按 `FFD8`/`FFD9` 切出来即可
- **自动跳过预热**：默认丢前 30 帧（AE/AWB 要约 2 秒才收敛）
- **自动回退设备路径**：`/dev/video2` 不存在时自动找 `/dev/v4l/by-id/*Orbbec*`

命令行也能直接抓：

```bash
python3 camera_capture.py --out /tmp/shots --count 5
python3 camera_capture.py --help
```

### `joint_state_reader.py` —— 关节角 / 位姿

```python
from joint_state_reader import JointStateReader, ToolPoseProvider

reader = JointStateReader()
poses = ToolPoseProvider(reader)          # 复用同一个节点

joints, stamp, waited = reader.wait_until_settled()   # 等臂停稳
T = poses.lookup()                                    # 4×4: base_link ← press_tool_link
```

- `JointStateReader` **按关节名字索引**（绝不按下标）
- `wait_until_settled()` 等到连续若干次采样变化小于阈值才返回，**超时会报错**
- `ToolPoseProvider` 用 **tf2** 查工具位姿 —— 正运动学交给 `robot_state_publisher`
  和 URDF，不用自己算

---

## 采集：直接跑 `capture_session.py`

这是给操作者用的入口。它把上面两个模块按**能出正确数据**的顺序接起来：

```
回车 → 等臂停稳 → 抓图 → 复核没动 → 查 TF → 存盘
```

```bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
python3 /ws/cam_joint_pose/capture_session.py --out ~/calib_data
```

> `/ws` 是容器内路径。**原生 ROS 2 环境**请换成你的实际路径，或建软链
> `sudo ln -s ~/piper_ws /ws` —— 详见 `WORKFLOW_FOR_COLLEAGUE.md` 第 1.1 / 1.2 节。

| 键 | 作用 |
|---|---|
| **回车** | 采一组 |
| `l` | 列出已采集的位姿（含旋转分散度统计）|
| `d` | 删掉最后一组（存错了用）|
| `h` | 帮助 |
| `q` | 保存并退出 |

采集过程中它会挡住三类坏数据：

| 挡什么 | 怎么发现 |
|---|---|
| 臂还在动 | `wait_until_settled()`，超时直接拒绝 |
| **抓图前后动了** | 抓完再读一次关节角，变化 > `--drift-tolerance` 就作废，并自动重采一次 |
| **位姿旋转太单一** | 和已有位姿夹角 < 15° 时告警（残差会变大，甚至解不出来）|

`poses.json` **每采一组就原子重写一次**，所以 Ctrl-C / 崩溃最多损失正在采的那一组。
中断后重跑会**自动续采**（编号接着排）；要重来才用 `--fresh`。

常用参数：

```bash
python3 capture_session.py --target 25                 # 目标组数（只影响提示）
python3 capture_session.py --rotate-180                 # 见坑 1，选定后不能改
python3 capture_session.py --settle-tolerance 0.001     # 臂在抖时放宽/收紧
python3 capture_session.py --help
```

⚠️ 跑之前确认两件事：

1. **`robot_state_publisher` 活着** —— 来自硬件 bringup，否则查不到 TF
2. **`tools/cam_view.py` 已经关掉** —— 相机独占（坑 5）

---

## 下面这段骨架：`capture_session.py` 内部在做的事

要自己改采集逻辑的话，需要理解的顺序就是这个：

```python
import json
import os
import time

import numpy as np
import rclpy

from camera_capture import Camera
from joint_state_reader import JointStateReader, ToolPoseProvider

OUT = "calib_data"
os.makedirs(OUT, exist_ok=True)

rclpy.init()
reader = JointStateReader()
poses = ToolPoseProvider(reader)          # base_link <- press_tool_link

samples = []
with Camera(rotate_180=False) as cam:     # 见下方"180° 旋转"一节
    for index in range(20):
        input(f"[{index+1}/20] 把机械臂移到位姿，摆好标定板，然后回车...")

        # ① 等臂停稳 —— 这一步不能省，原因见下
        joints, joint_stamp, waited = reader.wait_until_settled()
        print(f"    停稳用了 {waited:.1f}s，关节角: "
              f"{ {k: round(v,4) for k,v in joints.items()} }")

        # ② 抓图（此时臂是静止的）
        result = cam.grab()
        if result is None:
            print("    抓图失败，重来"); continue
        jpeg, wall_time = result

        # ③ 查工具位姿（臂仍然静止，所以时间先后无所谓）
        T = poses.lookup()
        if T is None:
            print("    TF 拿不到，重来"); continue

        name = f"pose_{index:03d}"
        with open(os.path.join(OUT, name + ".jpg"), "wb") as f:
            f.write(jpeg)
        samples.append({
            "name": name,
            "joints": joints,
            "joint_stamp": joint_stamp,
            "wall_time": wall_time,
            "T_base_tool": T.tolist(),        # 4×4 行主序
        })
        print(f"    保存 {name}")

with open(os.path.join(OUT, "poses.json"), "w") as f:
    json.dump({"tool_frame": poses.tool_frame,
               "base_frame": poses.base_frame,
               "samples": samples}, f, indent=2)

print(f"\n{len(samples)} 组 -> {OUT}/poses.json")
rclpy.shutdown()
```

**运行前提**（缺一不可）：

```bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch piper_hardware piper_real_bringup.launch.py enable_motors:=true command_speed_ratio:=20
```

`robot_state_publisher` 必须活着，否则第 ③ 步查不到 TF。
（`capture_session.py` 会在打开相机**之前**就检查这一点，缺了会立刻报错退出。）

> 跑之前 `tools/cam_view.py` **必须关掉** —— 相机是独占设备。

---

## ⚠️ 六个坑（都是踩过的）

### 1. ⚠️ 图像有 180° 旋转

相机传感器在壳体内是倒装的，**原始图像是上下颠倒的**。

两个自洽的做法，**选一个并全程一致**：

| 做法 | 怎么用 | 内参怎么办 |
|---|---|---|
| **raw（默认）** | 不旋转，存原始图 | 内参也在**原始图**上标定。那 180° 会被**吸收进手眼标定结果**里，感知节点用同一份结果即可 |
| **`--rotate-180`** | 旋转后存，和 URDF 名义光学系一致 | 内参必须在**旋转后**的图上标定：主点变成 $(W-1-c_x,\ H-1-c_y)$ |

**混用会毁掉一切** —— 用原始图的内参去处理旋转后的图，主点差一整个图幅。

### 2. ⚠️ 工具坐标系是 `press_tool_link`，不是 `gripper_base`

press 配置下相机挂在 **`press_tool_link`** 下面（见 `piper_press_tool.xacro`）。
在普通配置下才是 `gripper_base`。**xacro 里 `parent_link` 是个参数**，两种都有。

另外注意：**不是 `press_tcp`**（那是压杆尖端）。

### 3. ⚠️ 相机端坐标系要用 `wrist_depth_camera_optical_frame`

`solvePnP` 输出的是**光学系**（Z 前 / X 右 / Y 下，REP-103）。

URDF 里有两个：
- `wrist_depth_camera_link` —— 机械系（X 前）
- `wrist_depth_camera_optical_frame` —— **光学系** ← 用这个

搞混 = 差一个 90° 旋转，结果完全错乱。

### 4. ⚠️ 当前的相机安装值是**设计值**，不是实测值

```xml
<joint name="wrist_depth_camera_joint" type="fixed">
  <origin xyz="-0.035 0 0.07" rpy="0 -1.57079632679 0" />   ← 待标定的量
  <parent link="press_tool_link" />
  <child link="wrist_depth_camera_link" />
</joint>
```

这就是手眼标定要求的东西。标定完成后用它覆盖这里的 `origin`。

### 5. ⚠️ 相机是**独占设备**

同一时刻只能有一个进程打开 `/dev/video2`。

```bash
sudo fuser -v /dev/video2        # 看谁占着
pkill -f camera_capture.py
```

### 6. ⚠️ 必须在**臂静止**时采集

这条路径拿不到硬件曝光时间戳，记录的只是"用户态组装完成"的时刻。
臂在动的话，这个延迟直接变成标定误差。

`wait_until_settled()` 就是干这个的，**不要绕过它**。

---

## 采集图像的建议参数

| 项 | 值 | 原因 |
|---|---|---|
| 分辨率 | 1920×1080 | 板子占画面越大，姿态精度越高 |
| 格式 | **MJPG** | YUYV 在 USB 2.0 上被卡到 1080p 只有 5 fps |
| 帧率 | 15 | 实测能跑满（14.85/14.925）。静态场景够了 |

---

## 标定板位置和位姿数（供参考）

- **位姿数 ≥ 20**，旋转要**多样化**（绕不同轴都转）——
  位姿太单一会让方程病态，解不稳定
- 每个位姿都要能**完整看到板子**（eye-in-hand 的主要约束）
- 板子**固定不动**，位置要让机械臂能从多个角度看到

---

## 下一步（标定本身，不在此包内）

拿到 `calib_data/` 之后：

```
① 相机内参标定        cv2.calibrateCamera（用同一块板子、同一批图的尺度）
② 每个位姿            cv2.solvePnP → T_cam←board
③ 每个位姿            poses.json 里的 T_base_tool
④ 解手眼              cv2.calibrateHandEye(R_gripper2base, t_gripper2base,
                                            R_target2cam,   t_target2cam)
⑤ 验证                重投影误差 + 用相机看压杆互检
```

容器里 OpenCV 4.5.4 **自带 `aruco` 和 `calibrateHandEye`**，零额外安装。

---

## 有疑问先看这里

| 现象 | 先查 |
|---|---|
| 抓图失败 `Cannot open device` | 相机是不是被别的进程占着（坑 5）|
| 找不到 `/dev/video2` | `lsusb \| grep -i orbbec`；脚本会自动回退 by-id |
| `TF ... unavailable` | `robot_state_publisher` 在跑吗？URDF 里有那个 frame 吗？|
| `joints did not settle` | 臂是不是还在动？或者 tolerance 太严 |
| 标定结果明显错乱 | 十有八九是坑 3（`link` vs `optical_frame`）|
