# 接近位姿视觉二次修正 —— 设计

> **任务书 ④**：视觉伺服 —— 在接近位姿做二次观测并修正，限定迭代次数。
>
> - 日期：2026-10-12
> - 状态：**设计已定稿，未开工**
> - 分支：`feat/visual-refinement`（内层 `src/Piper_ros` + 外层 `piper_ws`，两者同步）
> - 基线提交：内层 `1ddf4bf` / 外层 `8876f1d`

---

## 1. 目标与不变量

### 1.1 要做什么

在**自由段之后、接近段之前**（即接近位姿 `pre_press`）停下来：

1. 用腕部相机对**按键旁边的第二块标定板**做一次观测；
2. 用这块板算出的按键位姿**修正**第一次（远处大板）给出的粗目标；
3. 用修正后的目标重算接近段与按压段，继续执行。

**限定迭代次数**：结构上支持多轮，本版本默认只做一轮。

### 1.2 不许破坏的不变量

来自 `.cursor/skills/piper-press-planning-control/`（架构契约）：

| 不变量 | 本设计如何保持 |
|---|---|
| 理想按压分**三段**：自由段 / 接近段 / 按压段 | 观测-修正**插在段与段之间**，不在任何段内部 |
| 按压段用**接触检测**停止 | 完全不动按压段 |
| **不得**用固定行程宣告按压成功 | 不变（`hardware_fixed_travel` 的现状不在本设计范围内） |
| 执行层**不决定策略** | 观测-修正循环写在编排层（`press_runner`），执行层只被拆成两个纯执行函数 |
| J1–J5 活动 / J6 固定 / TCP=`press_tcp` | 不动机器人模型 |
| 硬件默认不执行 | 不变；本设计不新增任何默认开启运动的东西 |

### 1.3 明确不做

- ❌ 按**中间**连续闭环（IBVS / 视觉伺服整段）—— 任务书限定在"接近位姿"
- ❌ 修正在按压段内的停止条件
- ❌ 行为树（BT 是契约定下的独立目标，不和本设计捆绑）
- ❌ 动 press 段的速度/接触语义

---

## 2. 改动前的基线（现状）

### 2.1 现在的按压链路

```
press_button_demo.cpp (main)
  └─ runSinglePress                       (press_runner.cpp)
       ├─ createTask → plan               (press_task_builder.cpp)
       └─ executePressSequence            (press_executor.cpp)
            └─ 一个 for 循环，执行到 "press button" 之前的所有阶段
               （自由段 + 接近段一起跑掉），然后按压 + 恢复
```

### 2.2 现有的阶段名（`collectMotions` 抽出来的顺序）

```
via clearance                          ─┐
from route to computed pre-press        │ 自由段
advance to button contact               ← 接近段（起点 = pre_press）
press button                            ← 按压段
retreat to pre-press waypoint           ─┐
return to captured start state          ─┘ 恢复（实际由 createRecoveryTask 重建）
```

### 2.3 现有的目标通道

`TargetSource`（`target_source.{hpp,cpp}`）已经处理：

- `frame_id` 白名单、`stamp` 非零与新鲜度、工作空间盒、四元数模长
- 拒收只记数打日志，**不动机械臂**
- `discardPendingTarget()` —— 丢弃运动期间到达的旧目标

**本设计复用它，不新增校验逻辑。**

---

## 3. 设计

### 3.1 阶段与数据流

```mermaid
flowchart LR
    A["① 粗目标<br/>A 板 ID 0-8"] --> B["自由段"]
    B --> C["pre_press<br/>（接近位姿，已外移）"]
    C -.->|"🆕 停稳 → 观测精目标<br/>B 板 ID 9-17"| D["融合 → 重建任务<br/>（采样规划，不用预录路点）"]
    D --> E["接近段"] --> F["按压段"] --> G["恢复"]
```

```
粗目标（A 板）
   │
   ├─ createTask(use_prerecorded_route = true) → plan
   ├─ runToPrePress()                    ← 跑到接近位姿就停
   │
   ├─ 停稳（waitForSettledJoints）
   ├─ for round in 0 .. max_refinement_rounds-1:
   │     等 /press/refine_pose（超时 refine_wait_timeout_s）
   │     合格？ ── 否 → 降级，跳出循环
   │     融合 → 新目标
   │
   ├─ createTask(新目标, use_prerecorded_route = false) → plan
   └─ executeFromPrePress()              ← 接近段 + 按压段 + 恢复
```

### 3.2 感知侧：一个节点、两块板、两个话题

`marker_locator` 从"一份板配置"扩展成"一份配置里两块板"：

| 板 | `first_marker_id` / `accepted_ids` | 话题 | 可见时机 |
|---|---|---|---|
| **A（粗）** | `0` / `[0..8]` | `/press/target_pose`（**已存在**） | 自由段之前，远处 |
| **B（精）** | `9` / `[9..17]` | **`/press/refine_pose`**（新增） | 到 `pre_press` 之后，按键旁 |

**按 marker ID 分流检测**，两块板同框也不会被拼成一次 `solvePnP`。

打印图已经生成（设计值 marker 30 mm / 间隙 10 mm，板宽 110 mm）：

- `aruco_board_A4.png`（ID 0–8）
- `aruco_board_A4_ID9.png`（ID 9–17）

**⚠️ `board.yaml` 里的 `marker_length` / `marker_separation` 填的是打印后的实测值（当前 `0.0285` / `0.0095`），不是设计值。** 换板子/重印后必须重新量。

**任务侧零新校验代码**：建**两个 `TargetSource` 实例**，一个订阅 `/press/target_pose`，一个订阅 `/press/refine_pose`。

### 3.3 任务侧：断点（落点 1）

`executePressSequence` 拆成两个**纯执行**函数：

```cpp
// 跑到接近位姿就停：执行 "advance to button contact" 之前的所有阶段
int runToPrePress(const rclcpp::Node::SharedPtr& node,
                  const PlanningTask& planning_task,
                  JointStateCache& joint_states);

// 从接近位姿起：接近段 + 按压段 + 恢复
int executeFromPrePress(const rclcpp::Node::SharedPtr& node,
                        const PressTaskParameters& parameters,
                        const PlanningTask& planning_task,
                        JointStateCache& joint_states,
                        bool require_contact);
```

**断点判据**从 `stage_name == "press button"` 改成
`stage_name == "advance to button contact" || stage_name == "press button"` ——
因为接近位姿在**接近段之前**。

`approach_completed` / `approach_motion` 的记账归到 `executeFromPrePress`。

**为什么是落点 1 而不是回调或调两次**：
- 回调会把"什么时候停下来观测"这个**策略**塞进执行层，违反契约 §3；
- 调两次执行层零改动，但第二次会把自由段再跑一遍，还要处理预录路点重放陷阱；
- 拆分意味着接近段这个**阶段边界被显式化** —— 而本设计本来就要在它上面插环节。

### 3.4 修正后怎么重建任务

`createTask` 加一个开关：

```cpp
PlanningTask createTask(const rclcpp::Node::SharedPtr& node,
                        const PressTaskParameters& parameters,
                        const std::map<std::string, double>& start_positions,
                        bool use_prerecorded_route = true);
```

**第二次调用传 `false`**：

- 自由段退化成"用采样规划器从**当前位置**（旧 `pre_press`）走到**新** `pre_press`"
- 修正量是毫米级 ⇒ 这一段只有几毫米，OMPL 秒解
- **避开了预录路点重放陷阱**：不关掉的话，臂会被带回路点起点再走一遍（几米的来回），
  而且中途可能碰到东西
- 接近段 + 按压段的 IK 用新目标重解

实现上就是在 `createTask` 内部把 `loadFreeMotionRoute(...)` 的结果传成
`std::nullopt`，让 `makeFreeMotionBranches` 走采样回退分支。

### 3.5 修正量的融合

> **前提**：B 板也是完整 3×3（36 个角点）。**如果实测发现 B 板只能看到部分 marker，
> 法向融合要降级为"法向只用第一次"** —— 见 §7。

**为什么可以融合**：两次观测的是**同一个物理量**（按键位姿），只是误差来源不同。

**位置与法向分别处理，都做融合（方案 C）**：

```cpp
// 位置：线性加权
p = (1 - w) * p1 + w * p2;

// 法向：先查夹角，再插值归一化
double theta = acos(clamp(n1.dot(n2), -1.0, 1.0));
if (theta > refine_max_angle_deg) {
    用 n1;                     // 跳飞 → 拒绝第二次的法向
} else {
    n = ((1 - w) * n1 + w * n2).normalized();   // 小角度下等价于 slerp
}
```

**⚠️ 法向必须做夹角检查。** 线性插值只在夹角小时有意义；夹角大时（比如 B 板跳飞解出
反向法向）插值结果会指向荒唐方向，**而归一化之后看起来完全正常**。

**合理性上限**：

```cpp
if ((p2 - p1).norm() > refine_max_correction_m) {
    拒绝第二次观测，用粗目标继续;      // 日志报原因
}
```

**权重从哪来**：现在**没有数据支撑任何具体值**。任务书的前提是"第二次更准" ⇒ `w` 偏大；
但共模误差（相机内参、手眼外参）融合**消不掉**，收益有上限。
⇒ 先给参数默认 `0.8`，**每次打印 Δp / Δθ / 实际修正量**，跑几次后用实测方差反推。

**误差分解（写在这里以免以后误解）**：

| 误差源 | 第一次 | 第二次 | 融合能否消除 |
|---|---|---|---|
| 角点像素噪声 | 独立 | 独立 | ✅ |
| 板子几何标定 | A 板的 | B 板的 | ✅ |
| 板→按键偏移 `o_tip` | `o_tip1` | `o_tip2` | ✅（各自量） |
| 相机内参 | 共用 | 共用 | ❌ **共模** |
| 手眼外参 | 共用 | 共用 | ❌ **共模** |

### 3.6 失败降级（三层）

**核心原则：视觉是增益项，不是依赖项。任何一步出问题，都要退回"能按"的那条路。**

| 情况 | 行为 | 日志 |
|---|---|---|
| 超时没收到精目标 | 用粗目标继续 | `WARN` |
| 精目标校验不过（frame / stamp / 工作空间） | 同上，`TargetSource` 已报原因 | `WARN` |
| 修正后 IK / 规划失败 | **回退用粗目标重建一次**；再失败才报错 | `ERROR` |

### 3.7 新增参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_refinement_rounds` | **1** | 修正轮数上限。**设 0 = 完全关闭本功能** |
| `refine_target_topic` | `/press/refine_pose` | 精目标话题 |
| `refine_wait_timeout_s` | **2.0** | 等精目标的超时；**必须非零**（视觉坏了任务不能挂死） |
| `refine_fusion_weight` | **0.8** | 精目标权重 `w`（0=只用粗，1=只用精） |
| `refine_max_correction_m` | **0.020** | `\|p2-p1\|` 超过此值 → 拒绝第二次 |
| `refine_max_angle_deg` | **3.0** | 法向夹角超过此值 → 法向用第一次 |

### 3.8 接近位姿要外移（`approach_distance` 上限放开）

**问题背景**：腕部相机**横向偏置约 105 mm**（手眼外参平移 X），光轴平行于工具轴。
⇒ 它**看不到工具轴上的东西**（这解释了早先实测的"压杆不在画面里"）。
板子必须贴在相机看得见的地方，而按键本身在视野外。

**实测视野（640×480）**：横向 67.7°、纵向 53.4°。

**按当前 `approach_distance = 40 mm` 估算**（相机比 TCP 靠后 ≈89.5 mm）：

| 方向 | 相机距面板 ≈129.5 mm 处的视野 | 110 mm 板子 |
|---|---|---|
| 横向 | 173.7 mm | ✅ 放得下 |
| 纵向 | **130.0 mm** | ⚠️ 只剩 **20 mm** 余量 |

⇒ **把接近位姿外移，让相机离板子更远，就能看全。**

**要改的代码**（`press_parameters.cpp` 的 `validatePressParameters`）：

```cpp
if (parameters.approach_distance < 0.01 ||
    parameters.approach_distance > 0.15) {        // ← 上限要放开
```

**具体放到多少，必须实测**（§7）。计算依据：板高 110 mm，需要
`d · tan(53.4°/2) > 55 mm` ⇒ 相机距面板 `d > 109 mm`，留余量取 **≥150 mm**；
换算回 TCP 距面板（相机比 TCP 靠后 89.5 mm）≈ **≥60 mm**。
**这些数字依赖手眼外参，而它可靠性存疑 —— 以实测为准。**

#### ⚠️⚠️ 反向硬下限：接近位姿不能无限外移（2026-10-12 实机 dry-run 实测）

**外移接近位姿有一个反向的硬下限 —— 位姿离基座太近会直接解不出 IK。**

| 瞄点高度 z | 瞄点 x 最小可行 | 对应 `pre_press` x |
|---|---|---|
| 0.300 | ≈ 0.21 ~ 0.22 | 0.17 ~ 0.18 |
| 0.240 | ≈ 0.24 ~ 0.26 | 0.20 ~ 0.22 |

原因：**home 位姿是为相机视野选的**，`joint2 = 0.011 rad` 离下限 0 只剩 0.6°
（`home_pose.yaml` 自己就警告过）。当前杆尖 `x = 0.173` 基本已是「压杆水平」时的
可达边界 —— **再往回退 13 mm 就无解**。

失败签名（很好认）：`best direction error = 0.040 deg`（方向几乎完美）而
`best position error = 0.0129 m`（位置差一个量级）⇒ 是**可达域**问题，不是求解器问题
（IK 搜索力度足够：250 迭代 × 34 个起点）。

> ⚠️ **不能靠抬 `joint2` 绕开** —— `home_pose.yaml` 的出路写着「抬 j2 15°，
> **同时**把板子挪到新视野里」。而本设计的立身之本就是相机视野，抬 j2 会把它一起丢掉。

**⇒ 约束不是「`approach_distance` 最大能多大」，而是这个组合式：**

$$x_{button} - d_{approach} \;\ge\; 0.18\ \text{m}$$

| 按键 x | `approach_distance` 上限 |
|---|---|
| 0.24 | **0.06 m（刚好够 §3.8 要的 60 mm）** |
| 0.26 | 0.08 m |
| 0.30 | 0.12 m（宽松） |

**⇒ 只要电梯按键在 `x ≳ 0.26`，`≥60 mm` 的接近段就完全没问题。
真正的危险是按键本身太靠里。**

> 精确边界在 0.16 ~ 0.18 之间（0.18 通过、0.16 失败）；上表取了保守值。
> 该数字**随目标和姿态变化**，落地前要用真实按键位姿复测一次。

**实现影响**：校验不能只看 `approach_distance` 自己，**要把
`button_x - approach_distance` 一起验**，否则一个合法的 `approach_distance`
配一个靠里的按键，症状是运行到接近位姿才发现 IK 失败 —— 而那时臂已经动了。

**代价（要知情）**：接近段变长。接近段速度缩放是 `cartesian.approach_speed_scaling`，
原来 25 mm 的直线约 1.6 s，拉长后按比例增加。这一段位移省不掉（相机需要那个距离）。

---

## 4. 关键决定与理由（备查）

| 决定 | 选择 | 理由 |
|---|---|---|
| 观测时机 | 接近位姿（段与段之间） | 任务书原文"在接近位姿"；且此时板子最近 |
| 断点 | **落点 1**：拆 `executePressSequence` | 见 §3.3 |
| 感知组织 | 一个节点 + 两块板 + 两个话题 | 相机只开一次；按 ID 分流，同框不误拼 |
| 修正次数 | 参数 `max_refinement_rounds` 默认 1 | 字面满足"限定迭代次数"，将来加迭代只改参数 |
| 融合范围 | **位置 + 法向都融合** | B 板是完整 3×3，法向有 36 点约束，可信 |
| 权重来源 | 参数 + 日志数据 | 没有实测方差之前不假装知道 |
| 失败处理 | 降级到粗目标 | 视觉不能被做成硬依赖 |

---

## 5. 验收标准

| 级别 | 判据 |
|---|---|
| **单元/离线** | 融合函数：夹角 0° / 小角度 / 超限 / 反向 四种输入的正确输出；降级路径 |
| **干跑**（Gazebo `execute:=false`） | 日志出现 `Refine round 1/1: ...`（或 fallback WARN）；六个阶段全部规划成功；修正后 `advance to button contact` 的时长合理 |
| **降级** | 不发 `/press/refine_pose` 时出现 fallback WARN，按压照常完成 |
| **仿真执行** | Gazebo 里六个阶段跑完，`SUCCESS` |
| **回归** | `max_refinement_rounds = 0` 时行为与改动前**完全一致** |
| **实机** | 由用户执行；本设计的责任是"不劣化现有链路" |

---

## 6. 文档影响（按契约要求同步更新）

| 文件 | 改什么 |
|---|---|
| `.cursor/skills/piper-press-planning-control/ARCHITECTURE.md` §1 | "Button recognition is outside the current scope" → 更新 |
| 同文件 §3 Planning layer | 增加"接近位姿观测-修正"这一环节 |
| 同文件 §4 Press target | 增加 `/press/refine_pose` 的语义 |
| `.cursor/skills/piper-press-planning-control/SKILL.md` | "Perception is paused" → 更新为"双码已接入" |
| `piper_elevator_manipulation/docs/perception_interface.md` | 增加 `/press/refine_pose` 的完整契约 |
| `piper_elevator_manipulation/docs/todo.md` | 新增一条（任务书 ④），带现状证据与验收 |

---

## 7. 未决 / 需要实测的数据

> **上一节的 `x_button − d_approach ≥ 0.18` 是这一节的瓶颈项** —— 它同时用到
> 下面第 1 行的 `x_button` 和第 2 行的 `d_approach`。先量 `x_button` 才能定 `d_approach`。

| # | 项 | 谁量 | 影响 |
|---|---|---|---|
| 1 | **真实按键在 base_link 下的 `x`** | 用户（尺量 / 示教） | 与 `d_approach` 组成 §3.8 的组合约束；**决定下面第 2 行的上限** |
| 2 | **`approach_distance` 的新值** | 用户（`tools/cam_view.py`，看全板子的最小距离） | 决定接近段长度与上限放开到多少；**必须同时满足 ≤ `x_button − 0.18`** |
| 3 | **B 板从 `pre_press` 能看到几个 marker** | 用户（同上，不动臂） | 决定法向能不能融合；< 6 个则方案要先调 |
| 4 | **`o_tip2`：B 板板系原点 → 按键的固定偏移** | 用户（尺量） | 不填则第二次观测自带固定偏差 |
| 5 | **`fusion_weight` 的实测依据** | 跑几次看 Δp 散布 | 从“拍的值”换成逆方差 |

> 第 1/2/3 项**可以在同一次不动臂的相机查看里一次性量完**（把臂摆到
> `pre_press`、把 B 板贴到预期位置、开 `tools/cam_view.py`）。
> 这也是实现计划的**第 0 步**。

---

## 8. 实施顺序

留给 `writing-plans` 技能展开。粗粒度顺序（依赖关系）：

1. `press_parameters`：放开 `approach_distance` 上限、**新增组合校验**
   （`x_button − d_approach ≥ 0.18`，见 §3.8）、新增 §3.7 的参数
2. 感知侧：`board.yaml` 双板配置 + `marker_locator` 按 ID 分流发两个话题
3. `createTask` 加 `use_prerecorded_route` 开关
4. 拆 `executePressSequence` → `runToPrePress` / `executeFromPrePress`
5. 编排层：`runSinglePress` 里的有界修正循环 + 融合 + 降级
6. 文档同步（§6）
