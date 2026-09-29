# 手眼标定工作流

> 这份文档写给拿到代码包的人。目标：**标定手腕相机与工具坐标系的关系**（eye-in-hand）。
>
> 产出：`T_press_tool_link ← wrist_depth_camera_optical_frame`，用于反解按键在
> `base_link` 下的位姿。
>
> 🚀 **只想快速跑起来？先看 `QUICKSTART.md`** —— 一页命令 + 四个必看的坑。
> 本文是它背后的完整背景、原理和验收标准。

---

## 0. 你会收到什么

```
piper_ws/
├── docker/                 ← 容器环境（Dockerfile + compose）
├── src/
│   ├── Piper_ros/          ← 全部 Piper ROS 包（git 仓库）
│   │   └── src/
│   │       ├── piper_hardware/              ← 实机 ros2_control 接口
│   │       ├── piper_description/           ← URDF（含相机挂载点）
│   │       ├── piper_press_moveit/          ← MoveIt 配置
│   │       └── piper_elevator_manipulation/ ← 按压任务 + 参考脚本
│   └── third_party/        ← ⚠️ MTC 等依赖
└── cam_joint_pose/         ← 采集工具 + 说明
```

### ⚠️ 第一件事：确认 `src/third_party/` 在不在

它**不在 git 仓库里**，是单独拷贝的。里面是编译依赖：

```
src/third_party/
├── moveit_task_constructor/
├── py_binding_tools/
└── piper_sdk/
```

**缺了就编译不过。** 先检查这个目录，比什么都重要。

> **`docker/` 要不要用？** 如果你有原生 Ubuntu 22.04 + ROS 2 Humble，**不用**。
> 但**别删** —— `docker/Dockerfile` 就是本项目的**权威依赖清单**，
> 第 1.1 节照着它列 apt 包。

---

## 1. 环境

### 1.1 两种走法

#### 走法 A —— 你有原生 22.04 + ROS 2 Humble（推荐）

**不需要 docker。** 但光有 `ros-humble-desktop` **不够** —— 这套要用 MoveIt 和 ros2_control：

```bash
sudo apt install -y \
    ros-humble-moveit ros-humble-moveit-ros-perception \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-controller-manager ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-tf-transformations \
    ros-humble-joint-state-publisher-gui can-utils

python3 -m pip install python-can==4.6.1 transforms3d numpy scipy
```

> 这份清单是从 `docker/Dockerfile` 里摘的 —— 那是**权威版本**，
> 缺东西而上面没列全时，直接去看它。

⚠️ **`LC_NUMERIC` 必须是 `en_US.UTF-8`** —— 中文 locale 下 **MoveIt / rcl 解析带小数点的
参数会失败**（踩过的坑，所以直接钉在镜像里）。先检查：

```bash
locale | grep LC_NUMERIC          # 要看到 en_US.UTF-8
```

不是就：

```bash
sudo locale-gen en_US.UTF-8
sudo update-locale LC_NUMERIC=en_US.UTF-8
# 改完重新登录，或临时 export LC_NUMERIC=en_US.UTF-8
export LC_NUMERIC=en_US.UTF-8
```

DDS 不用额外装：项目用 `rmw_fastrtps_cpp`，那是 Humble 的**默认值**。

#### 走法 B —— 用容器

当备选方案：想和主机系统隔离，或走法 A 卡住了。

```bash
cd piper_ws/docker
cp .env.example .env                # 改成自己机器的 UID/GID、DISPLAY
docker compose up -d
sg docker -c 'docker exec -ti piper_dev bash'   # 或者 docker compose exec piper bash
```

容器内已经是 ROS 2 Humble。**走法 B 时后续所有命令在容器内执行，`/ws` 就是工作区。**

### 1.2 ⚠️ 先解决 `/ws` 这个路径

**本文档里的 `/ws` 是容器内的路径。** 走法 A 时它不存在，两个办法二选一：

```bash
# ① 建软链（推荐 —— 下文所有命令原样可用）
sudo ln -s ~/piper_ws /ws

# ② 或者把下文所有 /ws/... 手动换成你的实际路径
```

同理，`PIPER_URDF_DIR` 环境变量默认也是 `/ws/...`，走法 A 时要自己设：

```bash
export PIPER_URDF_DIR=$HOME/piper_ws/src/Piper_ros/src/piper_description/urdf
```

### 1.3 编译

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

⚠️ **改完源码必须重新 `colcon build`**，否则跑的是旧二进制。

⏱️ 全量编译里 `moveit_task_constructor` 是大头。**只想先把标定跑起来的话**，
可以只编需要的三个包：

```bash
colcon build --symlink-install --packages-up-to piper_press_moveit piper_hardware
```

（`piper_elevator_manipulation` 依赖 MTC，**标定用不到它**；
以后要跑按压任务时再全量编。）

---

## 2. 硬件检查（每次开机都要做）

### 2.1 CAN 适配器

```bash
lsusb | grep 1d50:606f            # 应该看到 Geschwister Schneider CAN adapter
```

### 2.2 ⚠️ 拉起 can0（**最容易漏的一步**）

```bash
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

**必须看到 `state UP` 和 `can state ERROR-ACTIVE` 两项。**

> ⚠️ **USB-CAN 适配器每次重新插拔或重启都会回到 DOWN，位速率也会丢。**
> 没起来时：bringup 会**阻塞等反馈**，永远打不出 `Arm feedback received`，
> 表现为「机械臂完全不动」而其他终端看起来都正常。

### 2.3 相机

```bash
lsusb | grep 2bc5                 # Orbbec DaBai DCW2
ls /dev/v4l/by-id/                # 应该有 *Orbbec*video-index0
```

### 2.4 自检

```bash
ros2 run piper_elevator_manipulation preflight_check.py
```

看到 `=== CLEAN ===` 才能继续。它会检查有没有残留的 `ros2_control_node`/`move_group`
（重复实例会让机械臂收到矛盾指令，表现是「卡住不动」）。

---

## 3. 启动（三个终端）

### 终端 1 —— 硬件 + TF（常驻）

```bash
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash

ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20
```

**要看到这两行才算就绪：**

```
[PiperSystem]: Arm feedback received. Current joint angles (deg): ...
[PiperSystem]: Motors enabled: the arm now holds its position under power.
```

> 它是**唯一**的 `robot_state_publisher` 来源（TF 树的根）。后面查工具位姿全靠它。

### 终端 2 —— MoveIt + RViz（手动拖拽位姿）

```bash
ros2 launch piper_press_moveit piper_press_moveit.launch.py use_sim_time:=false
```

> ⚠️ **`use_sim_time:=false` 不能省。**
> 这个 launch 的默认值是 **`true`**（仿真用；参数说明原文就是
> *"Set false for the real arm"*）。开着它时 `move_group` 等的是 `/clock` 话题，
> 而实机上**没人发 `/clock`** → 节点时间停在 0 不动 →
> 机械臂模型不跟着 `/joint_states` 更新、TF 查询对不上时间。
> 表现是「RViz 里机器人一动不动」，很容易误判成硬件坏了。

起来后 RViz 里会有 **Motion Planning 面板** —— 拖末端的交互标记来指定任意姿态。

### 终端 3 —— 相机实时画面（摆板子、挑位姿时用）

```bash
python3 /ws/tools/cam_view.py          # 然后浏览器开 http://127.0.0.1:8099/
```

零依赖（只用标准库 + `v4l2-ctl`），已经在代码包里，不用另外装。

> ⚠️ **相机是独占设备 —— 采集前必须先 Ctrl-C 关掉它**，
> 否则 `capture_session.py` 打不开 `/dev/video2`。
> 流程是：开着它调板子位置 + 确认视角 → **关掉** → 再去采集。
>
> 忘了关的话，用 `sudo fuser -v /dev/video2` 看是谁占着。

**采集时必须能看到画面** —— 否则你不知道板子在不在画面里。

---

## 4. 摆标定板

- **固定在场景里，之后不能动。**
- 位置要让机械臂在**大部分位姿**下都能看到它
- 板子要够大 —— 建议在典型距离下占画面宽度 **≥ 30%**

---

## 5. 采集（目标 ≥ 20 组）

### ⚠️ 位姿的三条铁律

**① 旋转必须多样 —— 这是能不能标出来的决定性因素**

$AX=XB$ 的求解条件数取决于位姿的**旋转差异**。

| ❌ 不要 | ✅ 要 |
|---|---|
| 20 个位姿朝向都差不多 | **绕不同轴都要转** |
| 只平移，不动手腕 | 手腕姿态要有明显变化 |
| 位姿挤在一小片区域 | 分布散开 |

**② 每个位姿板子都要完整可见**（`optical_frame` 的四角都在画面内）

**③ 等机械臂停稳再采** —— 这条路径没有硬件曝光时间戳，臂在动会把延迟直接变成误差

### 采集循环

**用 `capture_session.py`，不要自己拼** —— 它把“等停稳 → 抓图 → 复核没动 → 查 TF → 存盘”这个顺序钉死了，并会阻挡坏数据。

```bash
# 关掉终端 3 的 cam_view.py 之后，就在终端 3 里跑（终端 1/2 要一直开着）
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
python3 /ws/cam_joint_pose/capture_session.py --out ~/calib_data
```

⚠️ **跑之前 `tools/cam_view.py` 必须关掉** —— 相机是独占设备，否则打不开。

```
① 在 RViz 里拖一个新的位姿（旋转和上一个明显不同）
② Plan & Execute
③ 回车，脚本自动采一组
④ 重复到 ≥ 20 组
```

| 键 | 作用 |
|---|---|
| **回车** | 采一组 |
| `l` | 列出已采的位姿 + 旋转分散度统计 |
| `d` | 删掉最后一组（存错了用）|
| `h` | 帮助 |
| `q` | 保存退出 |

**它会替你挡三类坏数据：**

| 挡什么 | 怎么发现 |
|---|---|
| 臂还在动 | 等停稳，超时直接拒绝 |
| **抓图前后动了** | 抓完再读一次关节角，变化过大就作废并**自动重采一次** |
| **位姿旋转太单一**（铁律①）| 和已有位姿夹角 < 15° 时告警，建议 `d` 删掉重摆 |

`poses.json` **每采一组就原子重写一次**，Ctrl-C / 崩溃最多损失正在采的那一组。
中断后重跑会**自动续采**；要重来才加 `--fresh`。

常见情况：

```bash
python3 capture_session.py --out ~/calib_data --target 25   # 目标改 25 组
python3 capture_session.py --out ~/calib_data --rotate-180  # 见附录 A，选定不能改
```

### 数据格式

```
calib_data/
├── pose_000.jpg        ← RGB 原始图（未旋转，见附录 A）
├── pose_001.jpg
├── ...
├── pose_000.json       ← 每组的单独记录
└── poses.json          ← 整个 session（标定读这个）
```

每组要存：

```json
{
  "name": "pose_000",
  "joints": {"joint1": 0.12, "joint2": 0.35, ...},
  "T_base_tool": [[...4x4...]],       // base_link ← press_tool_link
  "tool_frame": "press_tool_link",
  "base_frame": "base_link",
  "wall_time": 1790651569.08
}
```

**`T_base_tool` 用 4×4 齐次矩阵，行主序。**

---

## 6. 标定

### 6.1 相机内参（如果用同一批图）

```python
import cv2
rms, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points,
                                         image_size, None, None)
```

⚠️ **内参必须在和采集图完全相同的处理流程下标定。**
见附录 A（180° 旋转）。

### 6.2 每个位姿：板子在相机里的位姿

```python
ok, rvec, tvec = cv2.solvePnP(obj_points_3d, img_points_2d, K, dist)
R_target2cam, _ = cv2.Rodrigues(rvec)
# t_target2cam = tvec
```

**注意：这里解出来的是 `T_optical_frame ← board`**（solvePnP 用的是光学系）。

### 6.3 手眼求解

```python
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,   # ← FK 给的：T_base ← press_tool_link
    R_target2cam,   t_target2cam,     # ← solvePnP 给的：T_optical ← board
    method=cv2.CALIB_HAND_EYE_TSAI)
```

**OpenCV 里的 "gripper" = 我们的 `press_tool_link`**（工具安装点），
**不是 `link6`，也不是杆尖 `press_tcp`。**

解出来的是：

$$X = T_{press\_tool\_link \leftarrow optical\_frame}$$

### ⚠️ 6.4 别忘了这一步：光学系 → 机械系

xacro 里要填的是 **`T_press_tool_link ← wrist_depth_camera_link`**（机械系），
而上面解出来的是到**光学系**的。

两者差一个**已知的固定旋转**（`wrist_depth_camera.xacro` 里 `optical_joint` 的 `rpy`）：

$$T_{tool \leftarrow link} = T_{tool \leftarrow optical} \cdot T_{optical \leftarrow link}$$

**这一步忘了，结果会差一个 90° 旋转，看起来"差不多对"但实际全错。**

---

## 7. 验证

### 7.1 重投影误差

把解出来的 `X` 代回去，预测板子角点应该出现在图像的哪里，和实测比较。
**目标 < 1 px。**

### 7.2 用相机看压杆（整条链的端到端检查）

```
① 机械臂动到一个已知关节角
② 相机拍一张，找到压杆尖端在图像里的位置
③ 用整条链把它反推成 base_link 下的 3D 点
④ 和 FK 算出的 press_tcp 位置比较
```

| 差值 | 说明 |
|---|---|
| **< 1 mm** | ✅ 通了 |
| 几毫米，**恒定** | 平移分量有偏差 |
| 几毫米，**随位姿变** | 姿态分量有偏差 |
| **十几毫米** | 先怀疑 **TCP 长度**（`press_tcp` = 从 `press_tool_link` 沿 +Z 伸出 **154.1 mm**）|
| 差得离谱 | ⚠️ 十有八九是第 6.4 节那步漏了 |

---

## 8. 交付什么

```yaml
# 期望的交付格式
T_press_tool_link_link:
  translation: {x: ..., y: ..., z: ...}      # 米
  rotation:    {x: ..., y: ..., z: ..., w: ...}  # 单位四元数
  # 或者直接给 4x4 矩阵

相机内参:
  fx: ..., fy: ..., cx: ..., cy: ...
  dist: [k1, k2, p1, p2, k3]
  resolution: [1920, 1080]

报告:
  - 重投影误差（均值 / 最大）
  - 各 method 的对比（TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS）
  - 采集用了多少组、哪些组被剔除及原因
```

目标用途：把 `translation` + `rotation` 写进
`piper_description/urdf/wrist_depth_camera.xacro` 的 `origin`，
**覆盖当前的设计值**：

```xml
<joint name="wrist_depth_camera_joint" type="fixed">
  <origin xyz="-0.035 0 0.07" rpy="0 -1.57079632679 0" />   ← 换成实测值
  <parent link="press_tool_link" />
  <child link="wrist_depth_camera_link" />
</joint>
```

---

## 附录 A —— 图像有 180° 旋转

相机传感器在壳体内是**倒装**的，**原始图像上下颠倒**。

两个自洽的做法，**选一个并全程一致**：

| 做法 | 内参怎么办 | 那 180° 去哪了 |
|---|---|---|
| **存原始图（推荐）** | 在**原始图**上标定 | 被**手眼标定结果吸收**（因为 X 包含相机的所有安装角）|
| 旋转后存 | 必须在**旋转后**的图上标定，主点变成 $(W-1-c_x,\ H-1-c_y)$ | 体现在 URDF 里 |

**混用会毁掉一切** —— 用原始图的内参去处理旋转后的图，主点差一整个图幅。

> 推荐第一个：省掉一条容易忘的换算规则。标定和后续感知用同一份结果即可。

---

## 附录 B —— 排查表

| 现象 | 先查 |
|---|---|
| 机械臂完全不动 | **can0 起来了吗**（第 2.2 节）|
| bringup 打不出 `Arm feedback received` | 同上 |
| `preflight_check` 报 NOT CLEAN | 按它给的 PID `kill -9`，再重启终端 1 |
| 相机打不开 | 是不是被别的进程占着：`fuser -v /dev/video2` |
| 找不到 `/dev/video2` | `lsusb \| grep -i orbbec`；设备号会变 |
| 查不到 TF | `robot_state_publisher` 在跑吗？（终端 1）|
| `joints did not settle` | 臂还在动？或者容差太严 |
| 标定结果差一个 90° | 大概率是第 6.4 节漏了 |
| 标定结果不稳定 | **位姿旋转太单一**（第 5 节铁律①）|

---

## 附录 C —— 安全须知

**失能会让机械臂掉落。**

`PiperSystem::on_deactivate()` 会发 CAN `0x471` 失能帧，之后重力把 j2/j3 拽到硬限位，
末端大约**下垂 10 cm**（缓慢下垂，不是自由落体，**但关节处有夹点会夹伤手指**）。

| 操作 | 会失能吗 |
|---|---|
| Ctrl-C 终端 2/3 | ❌ |
| **Ctrl-C 终端 1** | ✅ **会掉** |
| `kill <pid>` 普通信号 | ✅ 会掉 |
| **`kill -9 <pid>`** | ❌ 保持使能、保持位姿 |

**收工顺序：先在臂下方垫泡沫/纸箱，再 Ctrl-C 终端 1。**
🚫 **不要用手扶**（夹手）；🚫 **不要用"关电源"退出**（急停除外）。
