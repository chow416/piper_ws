# 快速上手 —— 一页命令

> 只有命令和必看的坑。原理、验收标准、完整背景见 `WORKFLOW_FOR_COLLEAGUE.md`。
> 下文假设代码包解开在 `~/piper_ws`，并已建好 `/ws` 软链。

---

## 一次性准备

```bash
# ── 解包 ────────────────────────────────────────────────────────────
mkdir -p ~/piper_ws
tar -xzf <包名>.tar.gz -C ~/piper_ws
sudo ln -s ~/piper_ws /ws          # 文档里所有 /ws/... 就走通了
```

```bash
# ── 依赖 ────────────────────────────────────────────────────────────
# 光有 ros-humble-desktop 不够，缺 MoveIt 和 ros2_control，编译直接挂
sudo apt install -y \
    ros-humble-moveit ros-humble-moveit-ros-perception \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-controller-manager ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-tf-transformations \
    ros-humble-joint-state-publisher-gui can-utils

python3 -m pip install python-can==4.6.1 transforms3d numpy scipy
```

```bash
# ── locale：中文系统必做 ────────────────────────────────────────────
locale | grep LC_NUMERIC           # 要看到 en_US.UTF-8
# 不是的话：
sudo locale-gen en_US.UTF-8
sudo update-locale LC_NUMERIC=en_US.UTF-8     # 改完重新登录
```

```bash
# ── 编译 ────────────────────────────────────────────────────────────
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只想跑标定、想省时间的话，可以跳过 20 MB 的 MTC：

```bash
colcon build --symlink-install --packages-up-to piper_press_moveit piper_hardware
```

---

## 每次开机

```bash
# ① 设备在不在：1d50:606f = CAN 适配器，2bc5 = 相机
lsusb | grep -E "1d50:606f|2bc5"

# ② ⚠️ 拉起 CAN —— 最容易漏，漏了机械臂完全不动
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0        # 要看到 state UP + can state ERROR-ACTIVE

# ③ 自检：清掉残留进程，要看到 === CLEAN ===
python3 /ws/src/Piper_ros/src/piper_elevator_manipulation/scripts/preflight_check.py
```

---

## 开三个终端

每个终端第一句都要先 source（否则 `ros2` 找不到这些包）：

```bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
```

### 终端 1 · 驱动机械臂（常驻，全程不关）

```bash
ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20
```

| 参数 | 说明 |
|---|---|
| `enable_motors:=true` | **默认 false**，不加这个机械臂**不会动** |
| `command_speed_ratio:=20` | 20% 速度。默认 100，手动调位姿时太快 |

就绪标志：

```
Arm feedback received
Motors enabled: the arm now holds its position under power.
```

### 终端 2 · MoveIt + RViz（在这里拖位姿）

```bash
ros2 launch piper_press_moveit piper_press_moveit.launch.py use_sim_time:=false
```

⚠️ **`use_sim_time:=false` 绝对不能省** —— 默认值是 `true`，实机上没有
`/clock`，节点时间会停在 0，**表现是 RViz 里机器人一动不动**。

### 终端 3 · 相机画面

```bash
python3 /ws/tools/cam_view.py     # 浏览器开 http://127.0.0.1:8099/
```

用来摆板子、挑位姿。**相机是独占设备 —— 采集前必须 Ctrl-C 关掉。**

---

## 采集（目标 ≥ 20 组）

先 Ctrl-C 关掉终端 3 的 `cam_view.py`，然后就在终端 3 里跑：

```bash
python3 /ws/cam_joint_pose/capture_session.py --out ~/calib_data
```

循环：

1. RViz 里拖一个新位姿（**旋转和上一个明显不同**）
2. 点 **Plan & Execute**
3. 回终端 3，**回车**
4. 重复到 ≥ 20 组

| 键 | 作用 |
|---|---|
| **回车** | 采一组 |
| `l` | 看已采位姿 + 旋转分散度统计 |
| `d` | 删掉最后一组 |
| `q` | 保存并退出 |

脚本会自动挡三类坏数据：没停稳 / **抓图前后动了**（自动重采一次）/
**位姿旋转太单一**（告警，建议 `d` 删掉重摆）。

`poses.json` 每采一组就原子重写，Ctrl-C 最多丢正在采的那组，重跑**自动续采**。

---

## 验证

```bash
python3 -c "
import json
d = json.load(open('$HOME/calib_data/poses.json'))
print('组数  :', len(d['samples']))
print('工具系:', d['base_frame'], '<-', d['tool_frame'])
for s in d['samples']:
    t = s['T_base_tool']
    print(f\"{s['name']}  ({t[0][3]:+.3f}, {t[1][3]:+.3f}, {t[2][3]:+.3f})  \"
          f\"距最近位姿 {s['closest_existing_deg']}°\")
"
```

`距最近位姿` 越大越好 —— 全都 30° 以上就很理想。

---

## 收工

| 操作 | 机械臂会怎样 |
|---|---|
| Ctrl-C 终端 2 / 3 | 纹丝不动 |
| **Ctrl-C 终端 1** | ⚠️ 失能，靠重力**缓慢下垂**（关节处有夹点，会夹手）|
| 关机械臂电源 | ⚠️ 直接掉 |

**先在臂下方垫泡沫/纸箱，再 Ctrl-C 终端 1。**
短时离开就**什么都别关** —— 臂使能着就稳住了。🚫 不要用手扶。

---

## 四个必看的坑

| 坑 | 现象 | 怎么办 |
|---|---|---|
| **can0 没拉起** | 机械臂完全不动，其他终端看起来都正常 | `sudo ip link set can0 up type can bitrate 1000000` |
| **忘了 `use_sim_time:=false`** | RViz 里机器人一动不动 | 补在 MoveIt launch 后面 |
| **`cam_view.py` 占着相机** | 采集报 `Cannot open device` | 先 Ctrl-C；`sudo fuser -v /dev/video2` 查占用 |
| **位姿旋转太单一** | 标定残差大，甚至解不出来 | 绕**不同轴**都要转，别只平移 |
