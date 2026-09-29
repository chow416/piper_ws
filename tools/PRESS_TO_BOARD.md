# 按压标定板中心 —— 完整流程

从冷启动到压杆落在板中心、再归位。这是目前唯一一条**端到端**验证整条
视觉→手眼→运动链路的流程。

## 它到底在验证什么

四个环节串起来才能把杆尖送到板中心，任何一个错了都送不到：

| 环节 | 谁负责 | 错了会怎样 |
|---|---|---|
| 相机内参 | `camera_info_orbbec_640x480.yaml` | 板子位姿整体缩放 —— 所有距离同比例偏 |
| 板子尺寸 | `config/board.yaml` | 同上，而且**和内参退化**，单看一次测量分不开 |
| **手眼外参** | `T_flange_cam.json` | 落点整体偏；旋转错一点，落点随姿态漂几毫米到几厘米 |
| 关节 / TF | URDF + `/joint_states` | 落点偏，且和姿态耦合 |

**为什么用板中心**：板中心是整块板上**唯一不用量**的点 —— 它在板系里就是原点。
贴一个按钮上去就得用尺子量，而那个量出来的误差，和我们要找的误差**分不开**。

---

## 0. 前提检查

```bash
cd ~/piper_ws

# 容器在跑吗
sg docker -c 'docker ps --filter name=piper_dev --format "{{.Names}} {{.Status}}"'

# CAN 起来了吗（判据是 IFF_UP 位，不是 operstate —— CAN 没有载波概念）
ip -details link show can0 | head -1
python3 -c "print('can0 UP' if int(open('/sys/class/net/can0/flags').read().strip(),16)&1 else '★ can0 没起来')"
# 没起来就：
# sudo ip link set can0 up type can bitrate 1000000

# 相机在吗、被占用吗
v4l2-ctl --list-devices | grep -A2 -i orbbec
fuser -v /dev/video2 2>/dev/null || echo "相机空闲"

# ⚠️ 有没有残留的 ROS 进程（重复实例是老事故，见 docs）
ps -eo pid,comm --no-headers | grep -E "ros2_control_no|move_group|marker_loca" || echo "干净"
```

⚠️ **相机是独占的。** 如果你开着 `python3 tools/cam_view.py` 看画面，先关掉它：

```bash
pkill -f cam_view.py
```

⚠️ **改过 config 一定要重新 colcon build** —— `ros2 run` 读的是 `install/` 里的那份：

```bash
sg docker -c 'docker exec piper_dev bash -lc "
source /opt/ros/humble/setup.bash && cd /ws && colcon build --packages-select piper_marker_perception"'
```

---

## 1. 终端 A —— 使能电机并保持位姿

```bash
sg docker -c 'docker exec -it piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20"'
```

- **`command_speed_ratio:=20`** 是 20% 速度，调试期别开满。
- ⚠️ **这个终端决定机械臂的生死**：Ctrl-C 它 = 电机失能 = 臂靠重力下垂
  （j2/j3 会掉到机械硬限位，末端约下垂 10cm，**关节处有夹点会夹手**）。
  做实验期间**不要关这个终端**。要收工先垫泡沫再 Ctrl-C。
- 这个终端**必须留到最后**，中途 Ctrl-C 会让整条链路没数据。

---

## 2. 终端 B —— 起按压任务（先干跑！）

第一次对准一个新目标，**一定要先干跑**。干跑会完整地算 IK 和轨迹，但不动。

```bash
sg docker -c 'docker exec -it piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
    target_source:=topic max_presses:=1 \
    execute:=false hardware_enable:=false rviz:=false"'
```

看到这两行才算就绪：

```
Target source: topic '/press/target_pose', accepted frames world, base_link, ...
Waiting for a press target on '/press/target_pose' (waiting indefinitely)...
```

### 干跑 → 实机，怎么切换

干跑和实机**不能同时跑**（会抢 `/press/target_pose`，而且两个 `move_group` 是
老事故）。切换：

```bash
# 停掉当前这套
sg docker -c 'docker exec piper_dev bash -lc "pkill -f press_button_hardware.launch.py"'
sleep 3
# ⚠️ move_group 不会跟着 launch 一起退，必须单独清
sg docker -c 'docker exec piper_dev bash -lc "pkill -f move_group"'
ps -eo pid,comm --no-headers | grep move_group || echo "已清干净"
```

然后换成实机版（**唯一区别是最后两个参数**）：

```bash
sg docker -c 'docker exec -it piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
    target_source:=topic max_presses:=1 \
    execute:=true hardware_enable:=true rviz:=false"'
```

- **`max_presses:=1`** —— 压一次就退出。别用 `0`（无限次），那是给人盯着屏幕连续按用的。
- **`execute` 和 `hardware_enable` 是双闸门**，两个都得 true 才会动。

---

## 3. 终端 C —— 感知（找板子）

```bash
sg docker -c 'docker exec -it piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
ros2 run piper_marker_perception marker_locator.py --ros-args \
  --params-file /ws/install/piper_marker_perception/share/piper_marker_perception/config/marker_locator_orbbec.yaml"'
```

⚠️ **不要加 `target_frame:=base_link`。** 那会要求板子位姿走 TF 换算到 `base_link`，
也就要求手眼外参已经在 URDF 里（我们还没写进去，见文末）。**现在发在相机光学系**，
手眼由终端 D 的脚本显式施加 —— 这样只有一个变量。

启动日志应该看到：

```
intrinsics from .../camera_info_orbbec_640x480.yaml: CameraIntrinsics(fx=477.2, fy=477.7, ...)
BoardSpec(DICT_4X4_50, 3x3, L=28.5mm, S=9.5mm, 104.5x104.5mm)
lens 67.7 x 53.4 deg, fx=477.2 fy=477.7
board at (+0.0140, +0.1047, +0.4253) m in 'wrist_depth_camera_optical_frame'
```

⚠️ 相机独占 —— **这个终端和 `cam_view.py` 不能同时开**。

---

## 4. 终端 D —— 瞄准（先只算不发）

```bash
sg docker -c 'docker exec piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
cd /ws && python3 tools/press_board_centre.py --settle 6"'
```

不给 `--publish` 就**只算不发**，输出长这样：

```
板中心(相机系)  = (  +14.0,  +104.7,  +425.3) mm   共 191 帧，抖动 0.21 mm
瞄准点(base系)  = ( +497.3,   -18.3,  +203.2) mm
按压方向        = (+0.9993, -0.0315, +0.0209)
当前杆尖        = ( +169.7,    -0.0,  +264.1) mm
需要移动        = 333.7 mm
板系偏移        = (+0.0, +0.0, +0.0) mm
（没给 --publish，只算不发）
```

看到这几样再往下走：

| 该看什么 | 合理值 | 不对说明 |
|---|---|---|
| **抖动** | < 0.5 mm | > 2 mm 脚本会直接拒发 —— 板子在动，或有人在画面里 |
| **瞄准点** | 在 `[-0.30,-0.50,-0.02] ~ [0.65,0.50,0.80]` 内 | 出盒会被任务静默丢掉 |
| **需要移动** | 几十到几百 mm | 明显偏大 → 板子不在视野正前方 |
| **按压方向** | 板子竖着 → 约 ±X；平放 → 约 −Z | 方向不对 → 板子朝向不对 |

---

## 5. 真正按下去

```bash
sg docker -c 'docker exec piper_dev bash -lc "
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
cd /ws && python3 tools/press_board_centre.py --settle 6 --publish"'
```

**手放在电源开关附近。** 发出的瞬间机械臂就会开始动。

终端 B 会依次打印（全程约 11~13 秒）：

```
free_motion.path_source=prerecorded
Planning succeeded: approach, contact, press, retreat, return.
  planned total (motion only)           11.22 s
HARDWARE EXECUTION: no contact sensor. ...
Executing phase: via clearance (planned 1.70 s)
Executing phase: from route to computed pre-press (planned 3.24 s)
Executing phase: advance to button contact (planned 1.00 s)
Joint tracking at approach end:
  jointN  commanded ... measured ... lags by ... deg
Actual TCP at approach end: (0.5150, -0.0107, 0.2029), axial offset from target -0.0018 m
Executing fixed-travel press (planned travel 0.0030 m, safety cap 0.0060 m)
Executing recovery: retreat to pre-press waypoint
Executing recovery: return to captured start state
Press 1 finished with exit code 0.
Target source '/press/target_pose': 0 message(s) rejected in total.
```

**`Actual TCP at approach end` 那一行里的 `axial offset` 是控制器跟踪误差**，
不是标定误差 —— 它量的是"实际到了没有"，而"该去哪"是标定说了算。
实测约 **1.8 mm**。

---

## 6. 验收

**看杆尖在板上的落点。** 三个来源，从强到弱：

1. **压痕**：3mm 压入会在纸上留一个 12mm 直径的凹痕。这是最直接的证据。
2. **拍摄对比**：按前面 `/ws/log/board_before.png`，按后 `/ws/log/board_after.png`。
   ⚠️ 板子是**靠着的、不是固定的**，按压会推动它（实测按后板子位移 75 mm），
   所以两张图**不能直接做差** —— 要先用 `detect_board.py` 解出各自的板子位姿，
   把压痕换算到**板系**去比，那才与板子挪到哪儿无关。
3. **命令值**：终端 B 的 `Actual TCP` 和目标点的差 —— 只能说明控制器跟得上，
   不能说明标定对。

想省事就放一张白纸在板中心再按，凹痕在白纸上最清楚。

---

## 微调

全在这一个旋钮上 —— **板系**里的偏移，毫米：

```bash
# 往板系 -X 挪 3mm（"落点总是偏右就调这个"）
python3 tools/press_board_centre.py --offset-mm -3,0,0 --settle 6 --publish
```

先不加 `--publish` 看一眼，确认方向对了再发。

⚠️ **别用偏移去掩盖手眼外参的错误**。落点偏差如果是**随姿态变化**的
（在不同位置按，偏的方向不一样），那是**旋转**错了，调偏移救不回来 ——
应该重新标定手眼。偏移只能修**恒定**的偏差。

改手眼外参：直接编辑 `T_flange_cam.json`，**不用重新编译**，脚本每次读它。

---

## 坑（都是踩过的）

| 症状 | 真因 | 怎么办 |
|---|---|---|
| `marker_locator` 打不开相机 | `cam_view.py` 占着 | `pkill -f cam_view.py` |
| 改完 config 没生效 | `ros2 run` 读 `install/`，`colcon` 只复制不删除 | 重新 build；删过文件要 `rm -rf build/<pkg> install/<pkg>` |
| `move_group` 杀了还在 | 它不跟着 `ros2 launch` 的 PID 退 | `pkill -f move_group` 单独清 |
| 臂反应很慢 / 卡住不动 | **两个** `ros2_control_node` 或**两个** `move_group` 同时存在 | 见 `docs/`，用 `preflight_check.py` |
| IK 解不出来 | j2/j3 压在硬限位上，5 自由度塌缩成 3 | 先 `go_home.py` 把关节挪开限位 |
| RViz 里机器人不动 | `use_sim_time` 默认 true，实机没人发 `/clock` | 确认 `use_sim_time:=false` |
| 距离读数整体放大 2.86 倍 | 有人用了同事那份退化的内参 | 用 `check_intrinsics.py` 体检 |
| 落点在板子之外 | 手眼外参没套、或套错了坐标系 | 终端 D 的输出先过一遍 |

**动作之前，永远先跑终端 D 的"只算不发"。** 它 10 行输出里能看出上面大半的问题。

---

## 下一步（还没做的）

- **把手眼写进 URDF**：现在外参在脚本里显式施加。等数字稳定了，换算成
  `T_flange<-camera_link`（机械系！文件里那份是光学系，直接抄会差 122°）
  写进 `wrist_depth_camera.xacro` 的 `wrist_depth_camera_joint`，
  之后 `marker_locator` 就能直接发在 `base_link`，这条脚本也可以退休。
- **手眼外参的置信度**：同事给的结果**没有质量指标**（没有散布、没有姿态转动量、
  只报了一个方法）。要真正称得上"标定好了"，需要把采集会话拿过来重算一遍。
  现在能说的是：它通过了这次端到端按压，落点误差在几毫米。
