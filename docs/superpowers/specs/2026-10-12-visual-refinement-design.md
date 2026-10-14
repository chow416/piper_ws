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

1. 用腕部相机对**按键旁边的单个 ArUco 码（ID 9）**做一次观测；
2. 用它算出的按键位姿**修正**第一次（远处 3×3 大板）给出的粗目标 ——
   **只修正位置**，按压方向仍用第一次的（见 §3.5）；
3. 用修正后的目标重算接近段与按压段，继续执行。

**限定迭代次数**：结构上支持多轮，本版本默认只做一轮。

### 1.2 不许破坏的不变量

来自 `.cursor/skills/piper-press-planning-control/`（架构契约）：

| 不变量 | 本设计如何保持 |
|---|---|
| 理想按压分**三段**：自由段 / 接近段 / 按压段 | 观测-修正**插在段与段之间**，不在任何段内部 |
| 按压段的停止条件 | **完全不动**（见下方说明） |
| 执行层**不决定策略** | 观测-修正循环写在编排层（`press_runner`），执行层只被拆成两个纯执行函数 |
| J1–J5 活动 / J6 固定 / TCP=`press_tcp` | 不动机器人模型 |
| 硬件默认不执行 | 不变；本设计不新增任何默认开启运动的东西 |

#### ⚠️ 关于「按压段怎么判定成功」—— 本硬件没有接触检测（事实陈述）

**实机上只有一种手段：固定行程。** 这不是本设计的取舍，是硬件现实：

- **没有任何接触传感器** —— `/press/contact` 在实机上**没有发布者**
- `executePressSequence(..., require_contact)` 的两种取值，`press_executor.hpp:13-19`
  已经写明：`true` = 仿真（虚拟接触）；`false` = **hardware without a contact sensor**
- 实机路径传 `false`（`press_runner.cpp:154-161`），按压**在轨迹走完时结束**，
  实测 TCP 行程只作**安全上限**，不是判据。同一段代码自己打着：
  > `HARDWARE EXECUTION: no contact sensor. The press travel comes from the
  > planned trajectory; the measured TCP travel is only a safety cap ... above it.`

⇒ 契约里「按压段用接触检测停止 / 不得用固定行程宣告按压成功」这两条，**在现有硬件上
做不到**。这是**先于本设计就存在的差距**，本设计：

- ❌ 不引入接触检测
- ❌ 不改变按压段的停止条件
- ❌ **不把它当作待办** —— 不在本任务范围内

### 1.3 明确不做

- ❌ 按**中间**连续闭环（IBVS / 视觉伺服整段）—— 任务书限定在"接近位姿"
- ❌ 修正在按压段内的停止条件
- ❌ **接触检测** —— **本硬件根本没有接触检测手段**，按压只能按固定行程判定（详见 §1.2）。
      契约里那两条要求在实机上做不到，那是**先于本设计存在**的差距，**不在本任务范围内**
- ❌ 动 press 段的速度 / 接触语义
- ❌ 行为树（BT 是契约定下的独立目标，不和本设计捆绑）

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
    C -.->|"🆕 停稳 → 观测精目标<br/>B 码 ID 9"| D["融合 → 重建任务<br/>（采样规划，不用预录路点）"]
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

**⚠️ 实际的感知链路是【两个节点】，不是本节早先写的一个** —— 2026-10-14 核代码纠正：

```
marker_locator  ──[板子位姿]──▶  press_target_bridge  ──[按键位姿]──▶  press_button_demo
                 board_topic                      target_topic
                 (默认 /perception/board_pose)    (默认 /press/target_pose)
```

⇒ **双板改动两个节点都要动**：前者“每块板各解一次”，后者“每块板各施一次 `tip_in_board`”。

| 通道 | 形态 | `accepted_ids` | 板子位姿话题 | 按键目标话题 |
|---|---|---|---|---|
| **粗（A）** | **3×3 整板** | `[0..8]` | `board_topic`（**已存在**） | `/press/target_pose`（**已存在**） |
| **精（B）** | **单个 marker** | `[9]` | **新增一个同名参数** | **`/press/refine_pose`**（新增） |

> **2026-10-14 决定：B 用单个 marker，不用 3×3。**
> 收益见 §3.8（把"接近位姿要外移"整块要求解掉了），代价见下方。
> `board.yaml` 本身就支持 1×1 形态 —— **代码零改动**，只改
> `markers_x` / `markers_y` / `marker_separation` / `accepted_ids` 四个字段。

#### ⚠️ 单码的代价：**没有冗余**

这是用单码唯一真正的损失，而且你们**已经实测过它的失效方式**（`board.yaml` 原文）：

> ⚠️ **单个 marker 没有冗余**：3×3 靠"解出了几个 marker"能发现坏情况
> （实测贴反时 9 个只剩 **1 个**能解码，数量就是那个信号），
> 单个只剩"检出 / 没检出"一个信号。
>
> 而那 1 个码足够让 `solvePnP` 拟合出一个**朝前、指向正确、重投影 RMS 只有
> 0.14 px** 的位姿 —— **而且是错的**。

⇒ **解出来的错位姿，和解出来的对位姿长得一模一样。** 3×3 抓它靠的是**码的数量**，
不是精度。单码把这个信号丢掉了。

**本设计怎么兜**：

| 措施 | 作用 |
|---|---|
| `refine_max_correction_m`（默认 20 mm） | 精目标离粗目标太远 ⇒ 拒绝。**贴反 / 解飞主要靠这一道** |
| `refine_max_angle_deg`（默认 3°） | 法向跳飞 ⇒ 法向退回第一次 |
| **法向只取第一次**（§3.5） | 单码不参与法向，从源头去掉平面位姿歧义 |
| 每次都打印 Δp / Δθ | 跑几次就能看出散布是否异常 |

> ⚠️ **它挡不住"稳定地错"。** 码要是贴反了，每一次都会稳定地解出一个镜像位姿，
> 而 `refine_max_correction_m` 只在它**偏离粗目标足够远**时才拦得住。
> ⇒ **贴完码必须做一次人工核对**（§7）。

#### 同框不会混 —— 但保证来自哪里必须说清

**对象点表本身就是白名单**，不是事后过滤（`aruco_board.py:346-352`）：

```python
by_id = spec.object_points_by_id()   # 只有 accepted_ids 里的 ID 才有对象点
for marker_corners, marker_id in zip(corners, ids.flatten()):
    if int(marker_id) not in by_id:
        continue                      # not part of this board; ignore it
```

一个 marker 对 `solvePnP` 来说只是一堆像素点，**它的物理含义完全来自"查表查到的对象点"**——
查不到 ⇒ 它就不是这块板的一部分。这是**查找表层面**的隔离，比事后过滤彻底。

| 场景 | 结果 |
|---|---|
| 现在：那个单码（ID 9）进入画面 | A 板配置 `accepted_ids: [0..8]` ⇒ `continue` 掉，**完全无感** |
| 将来：两组 ID 不相交（`[0..8]` / `[9]`） | 各解各的，**不可能拼成一次 `solvePnP`** |

#### ⚠️⚠️ 四条必须守住的东西

**① 话题路由必须显式隔离（最大的坑，而且失败是静默的）**

PnP 层的隔离是白送的；**"哪块板的结果发到哪个话题"必须显式写**。

```python
# ✅ 正确：两块板各解一次、各发各的
coarse = detect(frame, ..., spec_a, min_markers=min_a)
fine   = detect(frame, ..., spec_b, min_markers=min_b)
if coarse: publish(coarse, "/press/target_pose")
if fine:   publish(fine,   "/press/refine_pose")

# ❌ 错误：谁先解出就发谁
result = coarse or fine
publish(result, "<某个话题>")
```

`pre_press` 那个位置**两块板可能同框**。写成后者 ⇒ 把 A 板的位置发到精修正话题上 ⇒
**融合出一个错的修正量，而所有日志、RMS、marker 数全部正常**。

**② `accepted_ids` 必须互斥，且要硬校验**

上面的保证**完全建立在"ID 不重叠"上**。共用 ID（比如都从 0 开始）会让同一个码出现在两个
`by_id` 里，两块板的 PnP 各自都会收下它 —— 保证**立刻失效**。

⇒ 配置加载时做**硬检查**：两块板的 `accepted_ids` 交集非空 ⇒ **拒绝启动**。不能靠人记。

**③ 每块板各存一份几何**

`board.yaml` 现在 `marker_length` / `marker_separation` **全文件只有一份**，而它自己写着：

> ⚠️ **不要填打印图上的设计值。** 图按 30.0 mm 出，A4 打出来实测是 28.5 mm。

两块板共用一份几何 ⇒ 哪天只重印了其中一块、换了一台打印机，那块板的**所有距离会静默地
按比例偏**。⇒ **两块板各存一份几何。**

**④ `min_markers` 是主判据，RMS 不是**

`aruco_board.py` 的文档记着实测结论：

> 贴反时 9 个码只剩 **1 个**能解码，但**那 1 个足够让 `solvePnP` 拟合出一个朝前、指向正确、
> 重投影 RMS 只有 0.14 px 的位姿** —— 而且是错的。

**PnP 不会因为"只用了 1 个码"而报警。** 可信信号是 `detection.marker_ids` 的**长度**。

**门槛自动跟随板子大小**（`BoardSpec.auto_min_markers`，约为总数的 3/5）：

| | 门槛 | 含义 |
|---|---|---|
| **A 板**（3×3） | **9 个里的 6 个** | 仍有冗余 —— 贴反 / 遮挡会被**数量**抓到 |
| **B 码**（1×1） | **1 个** | **检出即通过**，冗余为零 |

> ⚠️ **这就是第 ④ 条对 B 码失效的原因。** 对 A 板，“marker 数量”仍是最可信的信号；
> 对 B 码，这个信号退化成“检出 / 没检出”，而**镜像位姿是会被检出的**。
> 所以 §3.2 开头那段警告不是理论问题。

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

> **`require_contact` 是既有参数，原样透传。** 实机恒为 `false`（**硬件无接触传感器**，
> 见 §1.2）。本设计**不新增任何接触语义**，只是把它跟着搬过去 ——
> 签名保持一致，`press_executor.cpp` 里那条 `require_contact` 分支一行不动。

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

> **2026-10-14：B 改用单个 marker（ID 9），所以「法向只用第一次」是默认行为，
> 不再是"实测发现才降级"。** 原因见 §3.2 的冗余警告 —— 单码只有 4 个共面角点，
> 法向受**平面位姿歧义**影响，不值得让它参与。

**为什么位置可以融合**：两次观测的是**同一个物理量**（按键位置），只是误差来源不同。

**位置融合，法向不融合**：

```cpp
// 位置：线性加权
p = (1 - w) * p1 + w * p2;

// 法向：只用第一次观测（粗目标那次）
n = n1;
```

**⚠️ 但夹角检查留着** —— 它不再是"融合前的门"，而是用来**发现精目标解飞了**：

```cpp
double theta = acos(clamp(n1.dot(n2), -1.0, 1.0));
if (theta > refine_max_angle_deg) {
    拒绝第二次观测，用粗目标继续;      // 单码解飞时 n2 会乱指
}
```

> 线性插值只在夹角小时才有意义，而单码的 `n2` 本来就不稳 ⇒ **干脆不插值**，
> 只把 `n2` 当**健康度信号**用。这一条把"插值出荒唐方向、归一化之后看不出来"
> 那个坑从源头去掉了。

**合理性上限**：

```cpp
if ((p2 - p1).norm() > refine_max_correction_m) {
    拒绝第二次观测，用粗目标继续;      // 日志报原因
}
```

**权重从哪来（换成单码之后更要紧）**：

任务书的前提是"第二次更准"，但**换成单码之后这个前提没被证实过**：

- 单码**角点少、基线短**，姿态抖动比整板大。历史实测（`board.yaml` 的 1×1 说明里
  记着）：整板 `0.3 ~ 1.3 mm`，单个 28.5 mm 码 `2.3 ~ 3.3 mm` ——
  ⚠️ **但那次没记录测距**，不能直接拿来比（第二次是在近得多的距离上量的）。
- 位置误差大致**随距离线性下降** ⇒ 近距离的单码未必输。
- 共模误差（相机内参、手眼外参）融合**消不掉**，收益本来就有上限。

⇒ **默认 `w` 定为 `0.5`**：在"第二次更准"被证实之前，不该把 80% 的权重压在
一个**没有冗余**的观测上。**每次打印 Δp / Δθ / 实际修正量，跑几次用实测方差反推**，
再把它改对。

**误差分解（写在这里以免以后误解）**：

| 误差源 | 第一次 | 第二次 | 融合能否消除 |
|---|---|---|---|
| 角点像素噪声 | 独立 | 独立 | ✅ |
| 目标几何标定 | A 板（3×3）的 | B 码（单码）的 | ✅ |
| 目标→按键偏移 `o_tip` | `o_tip1` | `o_tip2` | ✅（各自量） |
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
| `refine_target_topic` | `/press/refine_pose` | 精目标话题（**单码 ID 9**，见 §3.2） |
| `refine_wait_timeout_s` | **2.0** | 等精目标的超时；**必须非零**（视觉坏了任务不能挂死） |
| `refine_fusion_weight` | **0.5** | 精目标权重 `w`（0=只用粗，1=只用精）。**必须实测反推**，见 §3.5 |
| `refine_max_correction_m` | **0.020** | `\|p2-p1\|` 超过此值 → 拒绝第二次。**单码方案下这是主要的安全闸** |
| `refine_max_angle_deg` | **10.0** | 两次法向夹角超过此值 → 判定精目标解飞，整个第二次作废。见下方 |

> **`refine_max_angle_deg` 为什么从 3.0 放宽到 10.0**
> 法向不再参与融合（§3.5），所以这道门**不再是"融合前的门槛"，而是"解飞了吗"的粗判**。
> 单码的 `n2` 本来就比整板抖，3° 会频繁误触发 —— 而每次误触发的后果是
> **退回粗目标**，功能看着在跑、其实永远不生效。**设置过紧 = 静默失效。**
> 真正的主力是 `refine_max_correction_m`（20 mm，位置层面）。
> 阈值从日志里的 Δθ 分布反推（会被打印出来）。

**感知侧要跟着改的配置**：`marker_locator` 的 B 板改成单码形态 —
`markers_x: 1`、`markers_y: 1`、`marker_separation: 0.0`、`accepted_ids: [9]`，
`min_markers` 保持自动跟随（单码时算出来就是 **1**）。

> ⚠️ **这意味着"检出即通过"** —— 唯一的门只剩 `rms` 上限，
> 而 §3.2 已经说明 rms **抓不到**镜像位姿（错的那种 rms 反而更小）。
> 这是单码方案的根本代价，别忘。

### 3.8 接近位姿**不再需要外移**（单码把这块约束解掉了）

> **2026-10-14：第二次观测改用单个 marker，这一节原本要解决的整个问题消失了。**
> 推导保留着 —— 它同时是**贴码位置的指导**。

**原来的问题**：腕部相机**横向偏置约 105 mm**（手眼外参平移 X），光轴平行于工具轴
⇒ **看不到工具轴上的东西**（这解释了早先实测的"压杆不在画面里"）。
按键在视野外，所以必须贴个东西到相机看得到的地方。

**而要拍的东西有多大，决定了相机得退多远** —— 这是全部症结所在：

| 要进画面的东西 | 需要占的面积 | 相机距面板下限 |
|---|---|---|
| 整张 3×3 板 | 110 × 110 mm | ≈ **109 mm**（纵向视场撑不下） |
| **单个 28.5 mm 码** | 28.5 × 28.5 mm | **基本没有下限** |

**实测视野（640×480）**：横向 67.7°、纵向 53.4°。

**码在画面里有多大**（`fx ≈ 477 px`；相机比 TCP 靠后 ≈89.5 mm）：

| `approach_distance` | 相机距面板 | 码的像素宽 |
|---|---|---|
| 20 mm | 109.5 mm | ≈ 124 px |
| **40 mm（现状）** | **129.5 mm** | **≈ 105 px** |
| 60 mm | 149.5 mm | ≈ 91 px |

⇒ **`approach_distance` 保持现状即可，`press_parameters.cpp` 一行代码都不用改。**

> 对照一下原来的要求：外移到 ≥60 mm ⇒ 接近起点再往回推 20 mm ——
> 而下面那个**反向硬下限**正好卡在那里。**单码把这对矛盾一起解掉了。**

#### 📌 贴码位置的指导（这才是本节对你实际操作有用的部分）

画面中心落在**按键侧方约 105 mm** 处（相机光轴就偏了这么多）。相机距面板 129.5 mm
时，视场半宽 ≈87 mm、半高 ≈65 mm ⇒ **码可以贴在距按键横向 18 ~ 192 mm、
垂直 ±65 mm 的范围里。**

| 约束 | 说明 |
|---|---|
| **别挡住按压路径** | 杆从 `pre_press` 沿工具轴推到按键，码要在这个走廊的侧面 |
| **别斜贴** | 尽量正对；单码本来就吃姿态 |
| 离按键近一点更好 | `o_tip2` 是**放大倍数**，越短，板子转角误差放大得越少 |

> ⚠️ 那个"105 mm"来自手眼外参，而它**可靠性存疑**（见 §3.5 的误差表）。
> **以实机看画面为准** —— 这正是 §7 第 2/3 项要测的东西。
> **先看画面，再贴码。**

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

**⇒ 组合约束仍然值得写成校验，只是它现在属于"锦上添花"：**

$$x_{button} - d_{approach} \;\ge\; 0.18\ \text{m}$$

| 按键 x | `approach_distance` 上限 |
|---|---|
| 0.24 | 0.06 m |
| 0.30 | 0.12 m（宽松） |

> 精确边界在 0.16 ~ 0.18 之间（0.18 通过、0.16 失败）；上表取了保守值。
> 该数字**随目标和姿态变化**，落地前要用真实按键位姿复测一次。

**实现影响（可选）**：**本设计不再强制这一条** —— 因为 `approach_distance`
不需要动了。但它值得作为一道**启动校验**留着：一个合法的 `approach_distance`
配一个靠里的按键，症状是**运行到接近位姿才发现 IK 失败，而那时臂已经动了**。
⇒ 保留在 §8，但标记为**可选**。

**~~代价（接近段变长）~~ —— 这条作废了。** 单码方案下接近段长度不变，
接近段的耗时也不变。

---

## 4. 关键决定与理由（备查）

| 决定 | 选择 | 理由 |
|---|---|---|
| 观测时机 | 接近位姿（段与段之间） | 任务书原文“在接近位姿”；且此时离按键最近 |
| 断点 | **落点 1**：拆 `executePressSequence` | 见 §3.3 |
| 感知组织 | 一个节点 + 两块板 + 两个话题 | 相机只开一次；按 ID 分流，同框不误拼 |
| **精目标用单码** | **单个 marker ID 9，不用 3×3**（2026-10-14 定） | 把 §3.8 的“接近位姿要外移”**整块解掉**（要进画面的从 110 mm 板变成 28.5 mm 码）；代价是**没有冗余**（§3.2） |
| 修正次数 | 参数 `max_refinement_rounds` 默认 1 | 字面满足“限定迭代次数”，将来加迭代只改参数 |
| 融合范围 | **只融合位置；法向用第一次** | 单码只有 4 个共面角点，撑不起法向，且平面位姿歧义会翻（§3.5） |
| 权重来源 | 参数 + 日志数据 | 没有实测方差之前不假装知道；**单码后默认降到 0.5** |
| 失败处理 | 降级到粗目标 | 视觉不能被做成硬依赖 |

---

## 5. 验收标准

| 级别 | 判据 |
|---|---|
| **单元/离线（融合）** | 夹角正常 / 超限 两种输入的正确输出；超限时**整个第二次作废**（不是只丢法向）；降级路径 |
| **单元/离线（配置校验）** | 两块板 `accepted_ids` 有交集 ⇒ **拒绝启动**并指明是哪两个 ID；无交集 ⇒ 正常 |
| **单元/离线（同框不串）** | 合成一张**两板同框**的图 ⇒ `/press/target_pose` 收到 **A 板**的位姿、`/press/refine_pose` 收到 **ID 9 单码**的位姿，**两者都不串**（可扩展现有的 `selftest_offline.py` 合成图基础设施） |
| **干跑**（Gazebo `execute:=false`） | 日志出现 `Refine round 1/1: ...`（或 fallback WARN）；六个阶段全部规划成功 |
| **⚠️ 功能真的生效了** | 干跑时**必须至少看到一次成功的修正**，不能一路 fallback WARN —— 单码方案最大的风险是阈值过紧导致**静默失效**（看着在跑、其实永远退回粗目标）。日志里要有非零的 Δp 和实际修正量 |
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

> **换成单码之后这一节反而变短了**：原来第 2 项（重定 `approach_distance`）整个不需要了
> —— §3.8 已经算过，单码在 40 mm 的现状下就有 ≈105 px，够用。

| # | 项 | 怎么做 | 影响 |
|---|---|---|---|
| 1 | **码贴哪儿** | 把臂摆到 `pre_press`，开 `tools/cam_view.py` 看画面 → 在画面里挑位置贴（**不动臂**） | 决定 `o_tip2`；**先看画面再贴，别先贴** |
| 2 | **单码解出来的位姿准不准** | **同一帧图像**分别用 `accepted_ids: [9]` 和 `[9..17]` 各解一次，比 Δp / Δθ | 决定 `fusion_weight` 能不能往上提；也是验证"第二次更准"这个前提的唯一办法 |
| 3 | **⚠️ 人工核对没有镜像** | 贴完码让 `marker_locator` 解一次，把画了坐标轴的 debug 图看一遍 | 单码**没有数量冗余**，而且镜像位姿的 rms 反而更小 —— 只能靠人眼 |
| 4 | **`o_tip2`：码中心 → 按键的固定偏移** | 尺量，板系（`+X 右 / +Y 上`），单位米 | 不填则第二次观测自带固定偏差 |
| 5 | **`fusion_weight`** | 跑几次看 Δp 散布，逆方差反推 | 把默认的 0.5 换成实测值 |
| 6 | **`refine_max_angle_deg` 的分布** | 看日志里 Δθ 的实际散布 | 默认 10° 是拍的；**设太紧 = 单码静默失效** |

> 第 1/3 项可以**在同一次不动臂的相机查看里做完**（把臂摆到 `pre_press`、
> 开 `tools/cam_view.py`）。这是实现计划的**第 0 步**。
>
> 第 2 项**不需要额外采集** —— 同一帧图像跑两次 `detect()` 就是了。
>
> ⚠️ 第 2 项和第 6 项是同一个风险的两面：**单码方案的失效方式是“静默”的**
> （阈值太紧 → 永远退回粗目标；镜像 → 稳定地错）。这两项就是防线。

---

## 8. 实施顺序

留给 `writing-plans` 技能展开。粗粒度顺序（依赖关系）：

0. **先量数据**（§7 第 1/3 项）：把臂摆到 `pre_press`、开 `cam_view` 看画面，
   定下**码贴在哪里** —— **这一步不做，`o_tip2` 就没法填**
1. `press_parameters`：新增 §3.7 的参数。
   ~~放开 `approach_distance` 上限~~ —— **不需要了**（§3.8）。
   组合校验（`x_button − d_approach ≥ 0.18`）**可选**，但值得做：
   防的是“臂都动了才发现 IK 失敗”
2. 感知侧（**两个节点都要改**，见 §3.2 的拓扑纠正）：
   - `board.yaml` 拆成**两块板各自一份几何 + 互斥 ID**（加载时硬校验），
     **B 板填成单码形态**（`markers_x/y: 1`、`marker_separation: 0.0`、`accepted_ids: [9]`）
   - `marker_locator` **每块板各跑一次 `detect()`、各发一个板子位姿话题**；
     `press_target_bridge` **两个板子位姿话题各订一个、各发一个按键目标话题** ——
     禁止“谁先解出就发谁”（见 §3.2 四条）
3. `createTask` 加 `use_prerecorded_route` 开关
4. 拆 `executePressSequence` → `runToPrePress` / `executeFromPrePress`
5. 编排层：`runSinglePress` 里的有界修正循环 + **只融合位置** + 降级
6. 文档同步（§6）
