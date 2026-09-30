# 按压标定板中心 —— 指令集

按原来的五终端骨架排。**终端 0~2 原样不动**，终端 3 从「按压任务」扩成
「按压服务」（多带一个感知节点），终端 4 从「手输目标点」换成「瞄准板中心」。

| 终端 | 干什么 | 常驻 | 命令 |
|---|---|---|---|
| **0** | 自检 + can0 | 否 | `piper ros2 run piper_elevator_manipulation preflight_check.py` |
| **1** | 硬件使能 + 控制器 | **是** | `ros2 launch piper_hardware piper_real_bringup.launch.py …` |
| **2** | 归位 | 否 | `ros2 run piper_elevator_manipulation go_home.py` |
| **3** | 按压服务（感知 + 按压任务） | **是** | `tools/press_board.sh --serve [--live]` |
| **4** | 发目标点（瞄准板中心） | 否 | `tools/press_board.sh [--live]` |
---

## ⚠️ 先搞清楚你在哪一层

这套东西**横跨两层**，命令长得很像但只能在一边敲。提示符一眼能看出来：

| 提示符 | 在哪 | `sg docker -c 'docker exec …'` 能用吗 |
|---|---|---|
| `zlf@zlf-ZhaoYang-X5-14-IAL:~/piper_ws$` | **宿主机** | ✅ 这就是它的用法 |
| `dev@piper-dev:/ws$` | **容器里** | ❌ 会报 `sg: group 'docker' does not exist` |

容器里没有 `docker` 这条命令，更没有 docker 组 —— **报的错和真因差得很远**，
第一次碰到很难想到是层次错了。

⚠️ **`tools/press_board.sh` 本身两种层次都直接能用**（它自己判层，判据是工作区在
`~/piper_ws` 还是 `/ws`）—— **但它前面的 `cd` 和 `piper` 简写不能照抄**：

| | 宿主机（`zlf@…:~/piper_ws$`） | 容器里（`dev@piper-dev:/ws$`） |
|---|---|---|
| 进工作区 | `cd ~/piper_ws` | `cd /ws` |
| `piper` 简写 | 可用（先 `source ~/piper_ws/tools/piper.sh`） | **✗ 不存在**（实测），要写全 `ros2 run …` |
| 按压脚本 | `tools/press_board.sh …` | `tools/press_board.sh …`（一样） |

**光看「命令对不对」看不出来 —— 照抄另一层的 `cd` 只会得到
`No such file or directory`，而那看起来像脚本不存在。**

只有一件事必须知道怎么转换：

```bash
# 从宿主机开一个容器里的交互 shell —— 进去后的提示符变 dev@piper-dev:/ws$
sg docker -c 'docker exec -it piper_dev bash -l'

# 从宿主机只跑一条命令
sg docker -c 'docker exec -it piper_dev bash -lc "
    source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
    ros2 topic list"'
```

已经坐在容器里的话，**把外面两层剥掉，只留引号里的内容**：

```
宿主机:  sg docker -c 'docker exec -it piper_dev bash -lc "A && B"'
容器内:  A && B
```

这是唯一需要手改的转换，其余都一样。

```bash
# 一次性把 sg/docker/source 那串样板收成一个词（建议写进 ~/.bashrc）
# ⚠️ 只在**宿主机**有用 —— 容器里没有 piper 这个命令（实测）
source ~/piper_ws/tools/piper.sh
piper_check     # = 终端 0 的自检
piper_down      # 清理终端 3 起的服务
```

---

## 🚀 一键 demo（日常就用这个）

一个窗口跑完原来的「终端 3 + 终端 4」。**不需要另开窗口**。

```bash
# 容器里：cd /ws        宿主机：cd ~/piper_ws
cd /ws

tools/demo.sh                 # 干跑：起服务 + 瞄准一次，机械臂不动
tools/demo.sh --live          # 实机：起服务 + 真按一次
tools/demo.sh --live --offset-mm -3,0,0     # 瞄偏 3mm 再按
```

它做的事情，按顺序：

1. `--down` 清掉上一轮残留（**绝不动 bringup**）
2. 后台起服务，等就绪（就绪判据含 `move_group` 存活）
3. 跑一遍瞄准，打印板中心 / 瞄准点 / 按压方向 / 需要移动
4. `--live` 才发目标，并**只印这一轮**的日志（旧轮次不串场）
5. 停在交互循环里：

```
══ 回车 = 再按一次   a = 只瞄准   q = 退出并清理 ══
>
```

`q` 或 Ctrl-C 都会回收——服务是本脚本的子进程，不会留残壳。

**参数**

| 参数 | 默认 | 说明 |
|---|---|---|
| `--live` | 关 | 真按；不带就是干跑（照样算完整轨迹，但不动臂） |
| `--offset-mm a,b,c` | 0,0,0 | **板系**偏移，瞄准时现用 |
| `--settle` | 6 | 瞄准前稳定帧的时间 |

### 🎛 按压参数在一个文件里 —— 改它，下一次按压就生效

接近段 standoff、按压行程、接近起点 **不在命令行上**：

```
src/Piper_ros/src/piper_elevator_manipulation/config/press_params.yaml
```

**改完存盘即可。** 不用重起服务，不用重新编译 —— 节点每次按压前都重新读它。
判据是**终端上那三个数**变了。`demo.sh` / `press_board.sh --serve` 的横幅会把配置
文件里的值**算出来**印成毫米（它只读冒号后面那个数，行尾注释影响不到它），
节点日志也会打一行：

```
Press parameters (from .../press_params.yaml): approach <A> mm, contact standoff <B> mm, press depth <C> mm
```

（上面是**格式**，数字随配置文件变。）

⚠️ 横幅和日志印的都是**毫米**，不是文件里的米 —— 文件写 `0.015` ⇔ 终端说
`15.0 mm`，是同一件事。
⚠️ **别拿文件里的行尾注释去确认参数生效没有**：注释是手写的、会过期，
而且它曾经被原样打到屏幕上，看着像参数没生效。现在终端不印注释了。

"不用重新编译"也成立，因为 `install/` 里那份是**符号链接**指回源码
（`colcon --symlink-install`），编辑源码里的文件就是编辑运行时那个。
文件里每个值都写明了允许区间和实测依据 —— 见文件本身的注释。

⚠️ **`execute` / `hardware_enable` 故意不放进这个文件。** 那两道闸门必须每次在
命令行上现敲（`tools/demo.sh --live`）。一个能活过本次会话的 `execute: true`，
正是"只差一个马虎的 flag"那个隐患 —— 而这个 launch profile 的设计初衷就是避开它。
（再说它们要是放进去，`--live` 会被文件静默盖掉，比不放还糟。）

### 📐 三个点是怎么定的

```
接近起点 = 瞄准点 − approach_distance
按压起点 = 瞄准点 − contact_standoff       ← 接近段停在这里
按压终点 = 按压起点 + press_depth
```

（具体多少毫米看**终端横幅**，它每次都是从配置文件现算的。写在文档里就会过期。）

⇒ **调 standoff 会把整个按压段一起平移**，而 `press_depth` 始终是
"从停下的地方再压进去多少"，与标定误差无关。

⚠️ **早先不是这样，而且这个差别很关键。** 那时按压终点写成 `瞄准点 + press_depth`，
于是 standoff 只能移动按压**起点**、动不了终点 —— 无论 standoff 加多大，
杆尖都会推过瞄准点。实测过：standoff 从 10 mm 加到 25 mm，接近段确实停得更早了，
但按压段照样越过去，"戳进板子"这件事一点没改善。

现在的写法是 `pressed_position = contact_position + press_depth`，
连带把日志里那个误导人的 `planned travel` 也变正确了 —— 它本来印的是
`press_depth`，却标成"计划行程"，而当时的真实行程是 `standoff + depth`。

### 🎯 怎么找 standoff：从 0 开始加

模型杆尖比真实杆尖**靠后一个常数量**，所以瞄准点实际落在板面**里面**。
把 standoff 一点点加大，直到杆尖**刚好**停在板面前 ——
**那个数就是模型误差的直接测量**。

已测过的证据（同一个 standoff、三种板子姿态）：

| 板子 | 瞄准点 (mm) | 接近终点轴向偏差 |
|---|---|---|
| 直立 | (577.0, −53.1, 204.1) | −28.0 mm |
| 直立（板子自己挪了） | (582.6, −46.7, 204.2) | −27.8 mm |
| 斜过来 | (615.6, −55.2, 204.9) | −28.2 mm |

三组离散只有 **0.4 mm** —— **偏差不随板子姿态变化**。
这条很关键：如果偏差来自手眼**旋转**错，斜板子时这个数会明显变。
所以它是个恒定轴向量，也就是沿杆方向的固定误差 ⇒ 指向**压杆长度**，
而不是手眼。历史上就有过一次同类修正：135.8 → 154.1 mm（差 18.3 mm）。

🚧 **还没定案**：量出来的差值要不要拿去改 `piper_press_tool.xacro` 的
`tool_length`，得等那个数确定。在那之前，**不要拿这套参数去戳真按键** ——
现在能保证的只是"接近段停在板面前"，按压段真正进去多少仍取决于这个误差。

---

## 🏢 部署到电梯：按的是按键，不是板子

上面的流程打的是**板心**。板心是校准期的靶子 —— 它是板上唯一不用量就知道位置的点，
所以手眼、内参还没定的时候只能拿它当靶（量一个按键贴上去，那个量出来的误差和
我们要找的误差**分不开**）。

链条可信之后，要按的就是电梯按键，那需要**量一次**按键相对板子在哪。

### 几何关系在这个文件里

```
src/Piper_ros/src/piper_marker_perception/config/board.yaml
```

```yaml
targets:
  call:                      # 名字随便起，命令行用 --button <名字> 指定
    taught: true             # ← 量完改成 true，否则会被拦住
    tip_in_board: {x: 0.000, y: 0.080, z: 0.000}      # 杆尖该落到的点，**米**
    press_axis_in_board: {x: 0.0, y: 0.0, z: -1.0}    # 往哪个方向压，板系单位向量
```

板系约定（代码里钉死了，别改）：

```
原点 = 板心
+X  向右（面向板子看）
+Y  向上
+Z  垂直板面向外，指向你（观察者）
```

⇒ 所以"往板面里压" = 板系的 **−Z**，这就是默认轴 `(0, 0, -1)` 的来历，不是随便填的。
⇒ 板子贴反了会差 180°，而四元数看起来仍然"正常" —— 只能靠实测抓。

### 三个字段怎么理解

| 字段 | 是什么 | 怎么给值 |
|---|---|---|
| `tip_in_board` | 杆尖**应该落到的点**（按键中心）在板系里的位置 | 尺子：从板心量 dx / dy，dz 一般 0 |
| `press_axis_in_board` | 按压方向 | 按键行程轴和面板法线一致 → 留 `(0,0,-1)` |
| `taught` | 这条是不是量过的 | 量完把 `false` 改成 `true` |

⚠️ **只有位移、没有旋转时，`press_axis_in_board` 留默认就行。**
它存在的唯一理由是"按键行程轴和面板法线不一致"（凸起的按键、斜装的面板）——
那种情况斜着压会把按键别住，所以它是个真输入，不是方便项。

**绕按压轴的自转不在这里，也不需要**：手臂是 5 自由度的，joint6 不参与，
绕杆自转本来就是自由的，写进来也没人执行。

### 用起来

```bash
cd /ws && tools/demo.sh --button call            # 只算不发，先看数字
cd /ws && tools/demo.sh --button call --live     # 真按
```

它和板心模式共用**除了"板系里那个点 + 轴"以外的全部**：同一个手眼、同一个抖动门、
同一个工作空间盒、同一份 `press_params.yaml`。

⚠️ `taught: false` 时会被**拦住**，不会拿占位坐标去压：

```
targets.call 还是 taught: false —— 那是占位值，不是量出来的。
量完把 .../board.yaml 里那个 taught 改成 true 再跑。
教示工具：ros2 run piper_marker_perception teach_press_offset.py --name call
```

（`teach_press_offset.py` 可以用机械臂自己量这个关系 —— 把杆尖挪到按键上，
同时读板子位姿和 TF。但它要求板子位姿已经发在 `base_link`，也就是手眼得先写进
URDF（见文末「下一步」）。在那之前用手动量 + `--button` 这条路。）



---

## 它到底在验证什么

四个环节串起来才能把杆尖送到板中心，任何一个错了都送不到：

| 环节 | 谁负责 | 错了会怎样 |
|---|---|---|
| 相机内参 | `camera_info_orbbec_640x480.yaml` | 板子位姿整体缩放 —— 所有距离同比例偏 |
| 板子尺寸 | `config/board.yaml` | 同上，而且**和内参退化**，单看一次测量分不开 |
| **手眼外参** | `T_flange_cam.json` | 落点整体偏；旋转错一点，落点随姿态漂几毫米到几厘米 |
| 关节 / TF | URDF + `/joint_states` | 落点偏，且和姿态耦合 |

**为什么拿板中心当靶**：板中心是整块板上**唯一不用量**的点 —— 它在板系里就是
原点。贴一个按钮上去就得用尺子量，而那个量出来的误差，和我们要找的误差
**分不开**。

---

## 终端 0 —— 自检，开 can0

```bash
cd ~/piper_ws && source tools/piper.sh
piper ros2 run piper_elevator_manipulation preflight_check.py
sudo ip link set can0 up type can bitrate 1000000
ip -details link show can0
```

⚠️ **每次插拔 can0 都会 down。** 判据是 IFF_UP 位——CAN 没有载波概念，
看 `operstate` 会误判。

⚠️ `preflight_check.py` 查的是**残留进程**。多出来的 `ros2_control_node` 或
`move_group` 造成的症状是「臂反应很慢、卡在某个位姿不动」，看起来像控制器
或规划问题，实际不是。

也可以直接 `piper_check`（等价，少一次进容器）。

---

## 终端 1 —— 硬件使能 + 控制器（常驻，不要关）

```bash
# 容器里（piper 简写不存在，写全）
ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20

# 宿主机（先 source ~/piper_ws/tools/piper.sh）
piper ros2 launch piper_hardware piper_real_bringup.launch.py \
    enable_motors:=true command_speed_ratio:=20
```

等到 `Arm feedback received` + `Motors enabled` 两行。

- **`command_speed_ratio:=20`** 是 20% 速度，调试期别开满。
- ⚠️ **这个终端决定机械臂的生死**：Ctrl-C 它 = 电机失能 = 臂靠重力下垂
  （j2/j3 掉到机械硬限位，末端约下垂 10cm，**关节处有夹点会夹手**）。
  要收工先垫泡沫再 Ctrl-C。

---

## 终端 2 —— 归位

```bash
# 容器里
ros2 run piper_elevator_manipulation go_home.py

# 宿主机
piper ros2 run piper_elevator_manipulation go_home.py
```

⚠️ **这一步不能省。** `go_home.py` 自己的说明写得很清楚：

> Enabling the motors does not move the arm anywhere; it holds whatever pose the
> arm is already in, and **a parked arm rests with joint2 and joint3 pinned
> against their stops.** This script drives the arm to a deliberate,
> well-conditioned pose so the press task never has to plan from a degenerate
> configuration.

实测过：使能后 j2 = 0.0095 rad，几乎就压在硬限位 `[0, 3.14]` 的 0 上。
两个关节同时钉在限位上 → 5 自由度塌缩成 3 → 5D IK 解出来残差能有 4~11mm，
**而它不会报错**，只会让落点偏。

---

## 终端 3 —— 按压服务（常驻，来一条目标按一次）

```bash
# 容器里：cd /ws        宿主机：cd ~/piper_ws
cd /ws
tools/press_board.sh --serve          # 干跑：算 IK 和轨迹，但不动
tools/press_board.sh --serve --live   # 实机：真按

# ⚠️ 按压参数**不在命令行上**。--standoff / --depth 会被**直接拒绝**（退出码 2），
#    告诉你改哪个文件；想改行程就改那个文件：
#      src/Piper_ros/src/piper_elevator_manipulation/config/press_params.yaml
```

（这两个参数曾经是"接受但静默无效"—— 传了不报错，实际跑的是文件里那份，
只在日志里留一条 WARN。那比报错坏得多：敲了一个看似生效的参数，屏幕上
看不出任何异常。现在改成拒绝。）

这一条命令起了两个节点（原来要在两个终端里分别敲）：

- `marker_locator` —— 感知，找板子，发 `/perception/board_pose`
- `press_button_hardware.launch.py` —— 按压任务，`max_presses:=0` 常驻

就绪后打印：

```
════ 前提检查 ════
  容器    Up 4 hours      can0    UP
  相机    空闲             残留    干净
  bringup 在跑（电机使能中）

════ 起服务 ════  execute=false hardware_enable=false  max_presses=0
  常驻：来一条目标按一次。
  ✓ 任务已待命，感知在发板子位姿，move_group 在跑（5s）

════ 服务常驻中 ════   另开一个终端发目标：
     cd /home/zlf/piper_ws && tools/press_board.sh
     cd /home/zlf/piper_ws && tools/press_board.sh --live
  在这个终端里 Ctrl-C 会把服务连 move_group 一起清掉。
```

⚠️ **这个终端 Ctrl-C 会清干净**，包括那个不跟着 `ros2 launch` 退的
`move_group` —— 手动清过两次都漏了它。**绝不动 bringup。**

### 执行时看实时画面

`--serve` 会顺手把调试图转发到浏览器：

```
  实时画面（带 marker 标注）  http://127.0.0.1:8098/
```

⚠️ **服务会自己看着自己。** 它每 5 s 摸一次 `press_button_demo` 和 `move_group` 的
脉，连着两次不在就打印原因并退出，把服务锁放掉 —— 不会出现「终端 3 看着在跑、
其实栈早死了」那种情况。退出前确实已经没了的日志在 `/tmp/press.log` 和 `/tmp/ml.log`。

画面里是 `marker_locator` 发的那张带标注的图：九个 marker 的检测框、检测到的
角点、板中心的坐标系（红 +X / 绿 +Y / 蓝 +Z 指向板外），顶上一行实时写着
`markers 9  rms 0.46px  d 50.4cm`。下方还跟着帧龄、帧率和板距。

**为什么不用 `cam_view.py`**：`/dev/video2` 是独占的，感知占着它的时候
`cam_view.py` 打不开。而且调试图本来就是**更适合看的**那张 —— 它上面有检测结果，
快出错的时候能提前看出来，而裸视频什么也看不出来。转发器不解码（直接把 JPEG
字节搬进 HTTP 体），所以占不了多少 CPU。

```bash
tools/press_board.sh --serve --no-view        # 不要它
tools/press_board.sh --serve --view-port 9000 # 换端口
tools/press_board.sh --serve --live           # 实机，画面一样看
```

⚠️ 相机独占：**感知在跑的时候 `python3 tools/cam_view.py` 打不开。**
要看就看上面这个地址。

⚠️ **别加 `target_frame:=base_link`。** 那要求手眼外参先进 URDF（见文末）。
现在的板子位姿发在**相机光学系**，手眼由终端 4 的脚本显式施加 ——
这样落点错了只有一个嫌疑人。

---

## 终端 4 —— 发目标点（瞄准板中心）

```bash
# 容器里：cd /ws        宿主机：cd ~/piper_ws
cd /ws
tools/press_board.sh                  # 只算不发 —— 永远先跑这个
tools/press_board.sh --live           # 真发出去
tools/press_board.sh --live --offset-mm -3,0,0    # 瞄偏 3mm 再发
```

⚠️ **`move_group` 不在时会当场拦住**（实测）：发目标前先查规划器，不在就直接退出，
不会让你白等一轮：

```
★ move_group 不在跑 —— 现在发目标，任务会在 'capture live start state' 就失败：
     Failed to acquire current PlanningScene within timeout of 3s
   去终端 3 重起服务：tools/press_board.sh --serve [--live]
```

```
════ 瞄准 ════
  按压任务现在： execute=True  hardware_enable=True
板中心(相机系)  = (   -5.6,   +93.0,  +486.7) mm   共 194 帧，抖动 0.38 mm
瞄准点(base系)  = ( +560.3,    -0.3,  +203.2) mm
按压方向        = (+0.9975, -0.0681, +0.0162)
当前杆尖        = ( +169.7,    -0.0,  +264.1) mm
需要移动        = 395.3 mm
板系偏移        = (+0.0, +0.0, +0.0) mm
```

**它会自己去问任务当前是什么模式**（`ros2 param get execute`），所以不会出现
「你以为是实机、其实是干跑」这种状况。模式对不上会当场警告：

```
  ⚠️⚠️ 你给了 --live，但**任务自己是干跑模式**。
       目标会发出去，臂不会动。要真按，把服务那个终端 Ctrl-C 后
       用 tools/press_board.sh --serve --live 重起。
```

看到这几样再往下走：

| 该看什么 | 合理值 | 不对说明 |
|---|---|---|
| **抖动** | < 0.5 mm | > 2 mm 脚本直接拒发 —— 板子在动，或有人在画面里 |
| **瞄准点** | 在 `[-0.30,-0.50,-0.02] ~ [0.65,0.50,0.80]` 内 | 出盒会被任务静默丢掉 |
| **需要移动** | 几十到几百 mm | 明显偏大 → 板子不在视野正前方，或臂的起点太远 |
| **按压方向** | 板子竖着 → 约 ±X；平放 → 约 −Z | 方向不对 → 板子朝向不对 |
| **`execute`** | 要按就得是 `True` | 是 `False` → 服务是干跑起的 |

终端 3 那边会依次打印（全程约 11~13 秒）：

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

**`Actual TCP at approach end` 那行的 `axial offset` 是控制器跟踪误差**，
不是标定误差 —— 它量的是「实际到了没有」，而「该去哪」是标定说了算。
实测约 **1.8 mm**。

---

## 验收

**看杆尖在板上的落点。** 三个来源，从强到弱：

1. **压痕**：3mm 压入会在纸上留一个 12mm 直径的凹痕。最直接的证据。
2. **拍摄对比**：按前 `/ws/log/board_before.png`，按后 `/ws/log/board_after.png`。
   ⚠️ 板子是**靠着的、不是固定的**，按压会推动它（实测按后位移 **75 mm**），
   所以两张图**不能直接做差** —— 要先用 `detect_board.py` 解出各自的板子位姿，
   把压痕换算到**板系**去比，那才与板子挪到哪儿无关。
3. **命令值**：`Actual TCP` 和目标点的差 —— 只能说明控制器跟得上，不能说明标定对。

想省事就放一张白纸在板中心再按，凹痕在白纸上最清楚。

---

## 微调

一个旋钮：**板系**里的偏移，毫米。

```bash
tools/press_board.sh --offset-mm -3,0,0            # 先只算不发看一眼
tools/press_board.sh --offset-mm -3,0,0 --live
```

⚠️ **别用偏移掩盖手眼外参的错误**。落点偏差如果**随姿态变化**（换个位置按，
偏的方向就不一样），那是**旋转**错了，调偏移救不回来 —— 应该重新标定手眼。
偏移只能修**恒定**的偏差。

改手眼外参：直接编辑 `T_flange_cam.json`，**不用重新编译**，脚本每次读它。

---

## 随时可用

```bash
tools/press_board.sh --check      # 只做前提检查
tools/press_board.sh --down       # 清理终端 3 起的服务（含 move_group）
tools/press_board.sh --help
```

---

## 坑（都是踩过的）

| 症状 | 真因 | 怎么办 |
|---|---|---|
| `marker_locator` 打不开相机 | `cam_view.py` 占着 | `pkill -f cam_view.py` |
| 改完 config 没生效 | `ros2 run` 读 `install/`，`colcon` 只复制不删除 | 重新 build；删过文件要 `rm -rf build/<pkg> install/<pkg>` |
| `move_group` 杀了还在 | 它不跟着 `ros2 launch` 的 PID 退 | `tools/press_board.sh --down`（已自动） |
| 臂反应很慢 / 卡住不动 | **两个** `ros2_control_node` 或**两个** `move_group` | `tools/press_board.sh --check` |
| 终端 4 报 `Failed to acquire current PlanningScene`，臂一动不动 | `move_group` 被**残留实例**退出时按名字 `pkill` 误杀 | 已修（服务归属令牌）；重起终端 3 |
| 容器里 `piper`、`cd ~/piper_ws` 都报错 | 命令是从另一层抄的 | `cd /ws`；`piper` 写成全的 `ros2 run …` |
| IK 残差大、落点偏 | j2/j3 压在硬限位上，5 自由度塌缩成 3 | **终端 2 的 `go_home.py`** |
| RViz 里机器人不动 | `use_sim_time` 默认 true，实机没人发 `/clock` | 确认 `use_sim_time:=false` |
| 距离读数整体放大 2.86 倍 | 有人用了那份退化的内参 | `check_intrinsics.py` 体检 |
| 落点整体偏移 | 手眼外参没套、或套错了坐标系 | 终端 4 输出先过一遍 |
| 落点在板子之外 | 板子太远、超出臂展 | 看「需要移动」那一行 |

---

## 下一步（还没做的）

- **把手眼写进 URDF**：现在外参在脚本里显式施加。等数字稳定了，换算成
  `T_flange<-camera_link`（机械系！文件里那份是光学系，**直接抄会差 122°**）
  写进 `wrist_depth_camera.xacro` 的 `wrist_depth_camera_joint`。之后
  `marker_locator` 就能直接发在 `base_link`，终端 3 那个感知节点也能并进
  `press_button_hardware.launch.py`，回到四个终端。
- **手眼外参的置信度**：同事给的结果**没有质量指标**（没有散布、没有姿态转动量、
  只报了一个方法）。要真正称得上「标定好了」，得把采集会话拿过来重算一遍。
  现在能说的是：它通过了端到端按压，落点误差几毫米。
