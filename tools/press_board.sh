#!/usr/bin/env bash
#
# 按压标定板中心。按原来的五终端骨架排：
#
#   终端 3（常驻）  tools/press_board.sh --serve [--live] [--no-view]
#                   = 感知 marker_locator + 按压任务，来一条目标按一次
#                     顺手把调试图转发到 http://127.0.0.1:8098/ 供观察
#   终端 4          tools/press_board.sh [--live] [--offset-mm a,b,c]
#                   = 瞄准板中心先看一眼；加 --live 才真发
#   任意            tools/press_board.sh --check     # 只做前提检查
#                   tools/press_board.sh --down      # 清理（含 move_group）
#
# 为什么要有这个脚本：原骨架里终端 3 和终端 4 之间还夹着一个感知节点，
# 而且手动停服务时 **move_group 不跟着 ros2 launch 的 PID 退** —— 今天为此
# 手动补了两刀。多出来的 move_group 造成的症状是「臂反应很慢、卡住不动」，
# 看起来像控制器或规划问题，实际不是。--serve 退出走 trap，一定能清掉。
#
# ⚠️ 不动 bringup。那是操作者的急停手柄，中断它电机会失能、臂会垂下来。

set -uo pipefail

LIVE=0
SERVE=0
DOWN=0
CHECK=0
SETTLE=6
OFFSET="0,0,0"
# 按压参数，单位**米**，和 launch 参数同名。默认值和 launch 默认值一致，
# 不在这里偷偷不一样。安全 demo 用 --standoff 0.010 --depth 0.0015。
STANDOFF=0.005
DEPTH=0.003
VIEW=1            # --serve 时顺手把调试图转发到浏览器，--no-view 关掉
VIEW_PORT=8098
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

while [ $# -gt 0 ]; do
  case "$1" in
    --live)      LIVE=1 ;;
    --serve)     SERVE=1 ;;
    --down)      DOWN=1 ;;
    --check)     CHECK=1 ;;
    --no-view)   VIEW=0 ;;
    --view-port) VIEW_PORT="$2"; shift ;;
    --standoff)  STANDOFF="$2"; shift ;;
    --depth)     DEPTH="$2"; shift ;;
    --settle)    SETTLE="$2"; shift ;;
    --offset-mm) OFFSET="$2"; shift ;;
    -h|--help)   sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "不认识的参数：$1"; exit 2 ;;
  esac
  shift
done

# ---------------------------------------------------------------- 我们在哪一层
# 这个脚本两种敲法都要能跑：
#   宿主机   ~/piper_ws/tools/press_board.sh   → 进容器跑
#   容器里   /ws/tools/press_board.sh          → 就在本地跑
#
# 为什么非判不可：宿主机上要 `sg docker -c 'docker exec …'`，而容器里
# **没有 docker 这条命令**，更没有 docker 组，敲下去只会得到
# `sg: group 'docker' does not exist` —— 报的错和真因差得很远。
# 判据是**工作区自己在哪**：宿主是 ~/piper_ws，容器里是 /ws。
# ⚠️ 不要用 /opt/ros/humble 判断 —— 宿主上也装了 ROS（实测），会两边都判成容器。
case "$WS_ROOT" in
  /ws) IN_CONTAINER=1 ;;
  *)   IN_CONTAINER=0 ;;
esac

PRELUDE='source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && cd /ws'

if [ "$IN_CONTAINER" = "1" ]; then
  # ⚠️ **必须先 set +u 再 source。** ROS 的 /opt/ros/humble/setup.bash 会直接引用
  # $AMENT_TRACE_SETUP_FILES 这类可能未定义的变量，而本脚本开了 set -u
  # （nounset），一 source 就是：
  #     /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
  # 宿主分支没这个问题，因为那里是 docker exec 起的一个**全新 bash**，没有 set -u ——
  # 所以这个 bug 只在容器里直接跑时才现形。
  dc()    { ( set +u; eval "$PRELUDE && $*" ); }
  dc_bg() { ( set +u; eval "$PRELUDE && $*" ) & }
else
  # 命令走 stdin 而不是 -c，因为传给它的命令里经常带单引号，
  # 而这里外面已经套了 sg 的双引号和 bash 的单引号 —— 三层引号叠起来必炸一次。
  dc()    { sg docker -c "docker exec -i piper_dev bash" <<<"$PRELUDE && $*"; }
  dc_bg() { sg docker -c "docker exec -d piper_dev bash -lc \"$PRELUDE && $*\""; }
fi

# ---------------------------------------------------------------- 服务归属令牌
# ⚠️ 这一段是整份脚本里唯一防止**自伤**的东西，要删先读完。
#
# 真实事故：后台残留了 9 个 --serve 实例，其中某一个退出时执行 cleanup 里的
#   pkill -f '[m]ove_group'
# 这条 pkill **按名字杀，不认主人**，于是把**后起**的那个 move_group 一起带走了。
# 现场长这样（/tmp/press.log）：
#   [move_group-1] You can start planning now!      ← 前 1.5 s 刚报就绪
#   [move_group-1] [INFO] [rclcpp]: signal_handler(SIGINT/SIGTERM)   ← 收到信号
#   moveit_cpp::MoveItCpp::~MoveItCpp() → Segmentation fault（exit -11）
# 后果：任务端永远等不到规划场景，日志里只有
#   Failed to acquire current PlanningScene within timeout of 3s
# 臂一动不动，而报错把锅甩给 MoveIt 的析构段错误 —— 非常难查。
#
# 令牌文件放在**工作区里**而不是 /tmp：宿主看到 ~/piper_ws/.press_serve.owner，
# 容器看到 /ws/.press_serve.owner —— bind mount，是同一个文件，两边才说得通。
SERVE_OWNER_FILE="$WS_ROOT/.press_serve.owner"
# ⚠️ 锁文件必须在 cleanup **之前**定义：--down 的 cleanup 也要删它（见下面那段），
# 而 --down 不经过 --serve 分支 —— 写在那里的话这里就是 unbound variable（set -u）。
SERVE_LOCK_FILE="$WS_ROOT/.press_serve.lock"
SERVE_TOKEN="$$-$(date +%s)"
FORCE=0

cleanup() {
  # 只有**当前令牌的持有者**有资格动现场。令牌被别人拿走了，说明后来者已接管，
  # 我们这种残留实例退出时必须闭嘴 —— 当初 move_group 就是这么被误杀的。
  # --down 是操作者显式要求清场，用 FORCE=1 绕过令牌。
  if [ "$FORCE" != "1" ] && [ "$(cat "$SERVE_OWNER_FILE" 2>/dev/null)" != "$SERVE_TOKEN" ]; then
    echo
    echo "—— 本实例（pid $$）已不是服务主人，退出时不做任何清理，避免误杀后起的服务。"
    return 0
  fi
  # ⚠️ 模式写成 '[p]ress…' 而不是 'press…'：pgrep/pkill -f 比的是**整条命令行**，
  # 而执行这条命令的 shell 自己的命令行里就有这个字符串。不加方括号的话，
  # 在容器里 `bash -lc "pgrep -af ros2_control_node"` 会匹配到那个 bash 自己。
  # 方括号让正则匹配别人、不匹配字面量 '['，一个字符就解决了。
  dc "pkill -f '[p]ress_button_hardware.launch.py'; pkill -f '[p]ress_button_demo'; pkill -f '[m]arker_locator.py'; pkill -f '[t]ools/debug_view.py'" >/dev/null 2>&1
  sleep 2
  dc "pkill -f '[m]ove_group'" >/dev/null 2>&1   # ⚠️ 不跟着 launch 退，必须单独来一下
  # ⚠️ 残留的 --serve **外壳**也要收：它不是容器里那些节点，上面几个 pkill 都看不见
  # 它，而它拿着服务锁。不收的话下一轮 --serve 会被自己的锁挡在门外，报
  # 「已经有 --serve 在跑」，可现场其实早就空了 —— 看着像 bug，其实是残壳。
  # 两种层次各有一份外壳，所以容器里一下、宿主上也一下：
  #   容器里跑的（docker exec 进去的那种）宿主 pgrep 看不见；
  #   宿主上跑的（sg docker -c '...' 那种）容器里也看不见。
  dc "pkill -f '[t]ools/press_board.sh --serve'" >/dev/null 2>&1
  pkill -f "[t]ools/press_board.sh --serve" >/dev/null 2>&1
  # 锁文件本身也删掉：万一外壳在另一个命名空间里收不到，删了文件下一轮才能建新的
  # （flock 认 inode，删掉即换新 inode）。令牌才是防误杀的那道，见上面的说明。
  rm -f "$SERVE_LOCK_FILE"
  echo
  echo "—— 已清理：press_button_hardware / press_button_demo / marker_locator / debug_view / move_group"
  # 以前这里无条件写「bringup 仍在跑」。bringup 不在的时候那是句假话，
  # 而这句话是操作者判断「臂还使能着吗」的依据，说错比不说更糟。
  if dc "pgrep -f '[r]os2_control_node'" >/dev/null 2>&1; then
    echo "—— bringup 仍在跑，机械臂使能中、保持位姿"
  else
    echo "—— ⚠️ bringup 没在跑 —— 机械臂现在是失能的（终端 1 起了吗？）"
  fi
  rm -f "$SERVE_OWNER_FILE"
}

# 栈是不是齐的：两个关键节点都得在。
stack_alive() {
  dc "pgrep -f '[p]ress_button_demo'" >/dev/null 2>&1 || return 1
  dc "pgrep -f '[m]ove_group'"        >/dev/null 2>&1 || return 1
  return 0
}

# --down 是操作者显式要求清场：绕过令牌，无条件清。
if [ "$DOWN" = "1" ]; then FORCE=1; cleanup; exit 0; fi

# ---------------------------------------------------------------- 起服务前的前提
# 起服务时要**没有**残留；瞄准时正好相反，服务必须**在**。两件事分开写。
serve_preflight() {
  local bad=0
  echo "════ 前提检查 ════"
if [ "$IN_CONTAINER" = "1" ]; then
  echo "  位置    容器内（「容器」那项跳过）"
else
  status=$(sg docker -c 'docker ps --filter name=piper_dev --format "{{.Status}}"' 2>/dev/null)
  if [ -n "$status" ]; then echo "  容器    $status"; else echo "  容器    ✗ 没在跑"; bad=1; fi
fi

if python3 -c "import sys; sys.exit(0 if int(open('/sys/class/net/can0/flags').read().strip(),16)&1 else 1)" 2>/dev/null; then
  echo "  can0    UP"
else
  echo "  can0    ✗ 没起来 —— sudo ip link set can0 up type can bitrate 1000000"; bad=1
fi

if fuser /dev/video2 >/dev/null 2>&1; then
  echo "  相机    ✗ 被占用 —— 先 pkill -f cam_view.py"; bad=1
else
  echo "  相机    空闲"
fi

left=$(dc "pgrep -a -f '[m]ove_group|[p]ress_button|[m]arker_locator'" 2>/dev/null)
if [ -n "$left" ]; then
  echo "  残留    ✗ 上一轮没清干净："; echo "$left" | sed 's/^/            /'
  echo "            —— 跑 tools/press_board.sh --down 清掉"; bad=1
else
  echo "  残留    干净"
fi

if ! pgrep -f "[r]os2_control_node" >/dev/null 2>&1 && \
   ! dc "pgrep -f '[r]os2_control_node'" >/dev/null 2>&1; then
  echo "  bringup ✗ 没在跑 —— 先在另一个终端起 piper_real_bringup（见 PRESS_TO_BOARD.md 第 1 节）"
  bad=1
else
  echo "  bringup 在跑（电机使能中）"
fi

  return $bad
}

# ---------------------------------------------------------------- 路由
if [ "$SERVE" = "1" ] || [ "$CHECK" = "1" ]; then
  serve_preflight || { echo; echo "★ 前提不满足，没起任何东西。"; exit 1; }
  if [ "$CHECK" = "1" ]; then echo; echo "—— 前提都满足。"; exit 0; fi
fi

# ================================================================ 终端 3：服务
if [ "$SERVE" = "1" ]; then
# 同一个服务起两份只会互抢相机、互踩任务节点，而且**后停的那份会把前一份的
# move_group 杀掉**（见上面令牌说明）。所以这里先拦一道，别让它叠起来。
#
# 用 flock 而不是 pgrep 认亲兄弟。为什么：命令常常是包在包装 shell 里跑的
# （run_in_terminal、某些终端会 `bash -lc "…"`），那个包装 shell 的命令行里
# 同样含 'press_board.sh --serve'，只排除 $$ 会把自己判成「别人」——
# 实测就这么误报过一次，把自己挡在门外。flock 认的是文件锁，不认命令行：
# 进程一死锁自动放；宿主实例和容器实例共用同一个 inode，两边也能互斥。
exec 9>"$SERVE_LOCK_FILE"
if ! flock -n 9; then
  echo "★ 已经有 --serve 在跑（锁 $SERVE_LOCK_FILE 被占），不再叠一份。"
  echo "   用那个终端就好；真要推倒重来：tools/press_board.sh --down"
  exit 1
fi

trap cleanup EXIT INT TERM
echo "$SERVE_TOKEN" > "$SERVE_OWNER_FILE"

if [ "$LIVE" = "1" ]; then EXEC=true; ENABLE=true; else EXEC=false; ENABLE=false; fi
echo
echo "════ 起服务 ════  execute=$EXEC hardware_enable=$ENABLE  max_presses=0"
echo "  contact_standoff_m=$STANDOFF   （接近段停在瞄准点之前这么远）"
echo "  press_depth=$DEPTH   （按压段越过瞄准点的行程）"
echo "  常驻：来一条目标按一次。"
dc_bg "ros2 run piper_marker_perception marker_locator.py --ros-args \
  --params-file /ws/install/piper_marker_perception/share/piper_marker_perception/config/marker_locator_orbbec.yaml \
  > /tmp/ml.log 2>&1"
dc_bg "ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
  target_source:=topic max_presses:=0 execute:=$EXEC hardware_enable:=$ENABLE rviz:=false \
  contact_standoff_m:=$STANDOFF press_depth:=$DEPTH \
  > /tmp/press.log 2>&1"

# 调试图转发 —— 不占相机设备，所以能和 marker_locator 同时开。
if [ "$VIEW" = "1" ]; then
  dc_bg "python3 tools/debug_view.py --port $VIEW_PORT > /tmp/view.log 2>&1"
fi

echo "  等节点起来（最多 60 s）..."
for i in $(seq 1 60); do
  # ⚠️ 这里用 sed 不用 awk：命令要穿过 dc 的三层展开，写 awk '{print $3}' 的话
  # 调用处转义成 \$3，到了接收端 shell **又会被展开一次**（无位置参数 → 空），
  # awk 实际收到的是 `print `（打印整行），比较 = "1" 就永远不成立 ——
  # 表现为“服务明明起来了但一直说没就绪”。sed 不需要 $，没有这个问题。
  subs=$(dc "timeout 4 ros2 topic info /press/target_pose 2>/dev/null | sed -n 's/^Subscription count: *//p'" 2>/dev/null)
  board=$(dc "timeout 4 ros2 topic info /perception/board_pose 2>/dev/null | sed -n 's/^Publisher count: *//p'" 2>/dev/null)
  # move_group 必须**活着**：任务在 'capture live start state' 那一段要问它拿
  # PlanningScene，拿不到就报 Failed to acquire current PlanningScene within
  # timeout of 3s —— 那句话写在任务日志里，看着像任务的问题，其实是规划器不在。
  # 只查订阅数/发布数是看不出这件事的，以前就是这么漏过去的。
  mg=$(dc "timeout 4 pgrep -c -f '[m]ove_group'" 2>/dev/null)
  if [ "${subs:-0}" = "1" ] && [ "${board:-0}" != "0" ] && [ "${mg:-0}" != "0" ]; then
    echo "  ✓ 任务已待命，感知在发板子位姿，move_group 在跑（${i}s）"; break
  fi
  # move_group 起来又死了的话别白等 60 s —— 日志里有现成的尸检报告。
  if [ "${mg:-0}" = "0" ] && dc "grep -qE 'Segmentation fault|process has died' /tmp/press.log" 2>/dev/null; then
    echo "  ✗ move_group 起过又死了 —— 规划器不在，任务会在 'capture live start state' 失败。"
    echo "     日志里是这样："
    dc "grep -E 'signal_handler|Segmentation fault|process has died' /tmp/press.log | tail -4 | cut -c1-140 | sed 's/^/       /'" 2>/dev/null
    echo "     最常见原因：另一个 press_board.sh 实例退出时按名字 pkill 误杀。"
    exit 1
  fi
  sleep 1
  if [ "$i" = "60" ]; then
    echo "  ✗ 等了 60 s 还没就绪（任务订阅者=${subs:-0}，板子发布者=${board:-0}，move_group 进程=${mg:-0}）。看看："
    dc "grep -viE 'corrupt|Select' /tmp/press.log | tail -5" 2>/dev/null
    dc "grep -viE 'corrupt|Select' /tmp/ml.log | tail -5" 2>/dev/null
    exit 1
  fi
done

  echo
  echo "════ 服务常驻中 ════"
  if [ "$VIEW" = "1" ]; then
    echo "  实时画面（带 marker 标注）  http://127.0.0.1:$VIEW_PORT/"
    echo "  （这是 marker_locator 发的调试图，不占相机设备。相机是独占的，"
    echo "    所以感知在跑的时候 cam_view.py 打不开 —— 要看就看这个。）"
    echo
  fi
  echo "  另开一个终端发目标："
  echo "     cd $WS_ROOT && tools/press_board.sh"
  echo "     cd $WS_ROOT && tools/press_board.sh --live"
  echo "  在这个终端里 Ctrl-C 会把服务连 move_group 一起清掉。"
  # 常驻循环，但**不是**傻睡。本来就出过一次「终端 3 看着在运行、其实栈早死了」：
  # move_group 被误杀后 ros2 launch 没重生它，press_button_demo 却还活着，
  # 于是终端 4 每发一次都在 capture live start state 失败，而人以为服务好着。
  # 每 5 s 摸一次脉，栈死了就主动退、把锁放掉，让操作者能干净重起。
  # ⚠️ 两个都要摸：当初那条事故路径正是「move_group 死了、press_button_demo 还活着」，
  # 只看任务节点根本发现不了 —— 终端 4 会一直在 capture live start state 失败，
  # 而终端 3 这边岁月静好。
  # ⚠️ 判死要**连着两次**：press_button_demo 比 move_group 晚约 7 s 才起（实测
  #    t≈3 s / t≈10 s），单次采样撞上这个窗口、或者 dc 那一层偶发抽风，都会假报。
  #    实测假报过一次，代价是好好的栈被自己清掉。
  while true; do
    sleep 5
    if ! stack_alive; then
      sleep 3
      if ! stack_alive; then
        echo
        echo "⚠️ 服务栈不完整 —— press_button_demo 或 move_group 不在了。"
        echo "   日志：/tmp/press.log 和 /tmp/ml.log。"
        echo "   本终端即将退出并释放服务锁，请重起：tools/press_board.sh --serve [--live]"
        exit 1
      fi
    fi
  done
fi

# ================================================================ 终端 4：发目标
subs=$(dc "timeout 4 ros2 topic info /press/target_pose 2>/dev/null | sed -n 's/^Subscription count: *//p'" 2>/dev/null)
board=$(dc "timeout 4 ros2 topic info /perception/board_pose 2>/dev/null | sed -n 's/^Publisher count: *//p'" 2>/dev/null)
if [ "${subs:-0}" != "1" ] || [ "${board:-0}" = "0" ]; then
  echo "★ 服务没在跑。先在另一个终端起：tools/press_board.sh --serve [--live]"
  echo "   （按压任务订阅者=${subs:-0}，板子位姿发布者=${board:-0}）"
  exit 1
fi

# 规划器在不在 —— 不在的话现在就得拦住。否则目标发出去、臂一动不动，
# 只在 /tmp/press.log 里留一句 Failed to acquire current PlanningScene。
mg=$(dc "timeout 4 pgrep -c -f '[m]ove_group'" 2>/dev/null)
if [ "${mg:-0}" = "0" ]; then
  echo "★ move_group 不在跑 —— 现在发目标，任务会在 'capture live start state' 就失败："
  echo "     Failed to acquire current PlanningScene within timeout of 3s"
  echo "   去终端 3 重起服务：tools/press_board.sh --serve [--live]"
  exit 1
fi

# 任务会不会真动 —— 直接问它，不靠人记住
task_exec=$(dc "timeout 6 ros2 param get /piper_press_button_demo execute 2>/dev/null | sed -n 's/.*[[:space:]]//p'" 2>/dev/null)
task_hw=$(dc "timeout 6 ros2 param get /piper_press_button_demo hardware_enable 2>/dev/null | sed -n 's/.*[[:space:]]//p'" 2>/dev/null)
echo "════ 瞄准 ════"
echo "  按压任务现在： execute=$task_exec  hardware_enable=$task_hw"
if [ "$LIVE" = "1" ] && [ "$task_exec" != "True" ]; then
  echo
  echo "  ⚠️⚠️ 你给了 --live，但**任务自己是干跑模式**。"
  echo "       目标会发出去，臂不会动。要真按，把服务那个终端 Ctrl-C 后"
  echo "       用 tools/press_board.sh --serve --live 重起。"
elif [ "$LIVE" != "1" ] && [ "$task_exec" = "True" ]; then
  echo "  （任务是实机模式，但这里没加 --live —— 所以只算不发）"
fi

# ⚠️ 先数一遍「Press N finished」有几条。等结果时**必须**比这个数更大才算这一轮
# 跑完 —— 光 `grep -q` 的话，第二次按压之后那个循环会立刻命中**上一轮**留下的旧日志，
# 于是「执行中」瞬间结束、打印出来的是上一轮的结果（实测踩到，很容易看漏）。
pressed_before=$(dc "grep -c 'Press .* finished' /tmp/press.log" 2>/dev/null)
pressed_before=${pressed_before:-0}

if [ "$LIVE" = "1" ]; then PUB="--publish"; else PUB=""; fi
dc "python3 tools/press_board_centre.py --settle $SETTLE --offset-mm $OFFSET $PUB"
aim_rc=$?

if [ "$aim_rc" != "0" ]; then
  echo; echo "★ 瞄准脚本退出码 $aim_rc —— 没有发目标。"; exit $aim_rc
fi

if [ "$LIVE" != "1" ]; then
  echo
  echo "════ 只算不发 ════ 什么都没动。加 --live 才发。"
  exit 0
fi

# ---------------------------------------------------------------- 等结果
if [ "$LIVE" = "1" ]; then
  echo
  echo "════ 执行中 ════"
  done_now=0
  for i in $(seq 1 60); do
    done_now=$(dc "grep -c 'Press .* finished' /tmp/press.log" 2>/dev/null)
    if [ "${done_now:-0}" -gt "$pressed_before" ] 2>/dev/null; then break; fi
    sleep 1
  done
  if [ "${done_now:-0}" -le "$pressed_before" ] 2>/dev/null; then
    echo "  ⚠️ 等了 60 s 还没看到这一轮结束，日志尾巴："
  fi
  echo
  # 只印**这一轮**：从最后一条 'Accepted target' 开始，不带旧轮次。
  n=$(dc "grep -n 'Accepted target' /tmp/press.log | tail -1 | cut -d: -f1" 2>/dev/null)
  dc "tail -n +${n:-1} /tmp/press.log | grep -viE 'corrupt|Select|ompl|moveit_ros|planner configuration|was not set|was set to|adapter|Loaded robot|No root/virtual' | cut -c1-165"
  echo
  dc "grep -E 'Actual TCP at approach end' /tmp/press.log | tail -1" 2>/dev/null \
    | sed 's/^/  → /'
  echo
  echo "  ⚠️ 这行是**控制器跟踪误差**，不是标定误差。落点准不准要看板上的压痕。"
fi
