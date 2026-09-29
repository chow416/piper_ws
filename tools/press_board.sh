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

cleanup() {
  # ⚠️ 模式写成 '[p]ress…' 而不是 'press…'：pgrep/pkill -f 比的是**整条命令行**，
  # 而执行这条命令的 shell 自己的命令行里就有这个字符串。不加方括号的话，
  # 在容器里 `bash -lc "pgrep -af ros2_control_node"` 会匹配到那个 bash 自己。
  # 方括号让正则匹配别人、不匹配字面量 '['，一个字符就解决了。
  dc "pkill -f '[p]ress_button_hardware.launch.py'; pkill -f '[p]ress_button_demo'; pkill -f '[m]arker_locator.py'; pkill -f '[t]ools/debug_view.py'" >/dev/null 2>&1
  sleep 2
  dc "pkill -f '[m]ove_group'" >/dev/null 2>&1   # ⚠️ 不跟着 launch 退，必须单独来一下
  echo
  echo "—— 已清理：press_button_hardware / press_button_demo / marker_locator / debug_view / move_group"
  # 以前这里无条件写「bringup 仍在跑」。bringup 不在的时候那是句假话，
  # 而这句话是操作者判断「臂还使能着吗」的依据，说错比不说更糟。
  if dc "pgrep -f '[r]os2_control_node'" >/dev/null 2>&1; then
    echo "—— bringup 仍在跑，机械臂使能中、保持位姿"
  else
    echo "—— ⚠️ bringup 没在跑 —— 机械臂现在是失能的（终端 1 起了吗？）"
  fi
}

if [ "$DOWN" = "1" ]; then cleanup; exit 0; fi

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
trap cleanup EXIT INT TERM

if [ "$LIVE" = "1" ]; then EXEC=true; ENABLE=true; else EXEC=false; ENABLE=false; fi
echo
echo "════ 起服务 ════  execute=$EXEC hardware_enable=$ENABLE  max_presses=0"
echo "  常驻：来一条目标按一次。"
dc_bg "ros2 run piper_marker_perception marker_locator.py --ros-args \
  --params-file /ws/install/piper_marker_perception/share/piper_marker_perception/config/marker_locator_orbbec.yaml \
  > /tmp/ml.log 2>&1"
dc_bg "ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
  target_source:=topic max_presses:=0 execute:=$EXEC hardware_enable:=$ENABLE rviz:=false \
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
  if [ "${subs:-0}" = "1" ] && [ "${board:-0}" != "0" ]; then
    echo "  ✓ 任务已待命，感知在发板子位姿（${i}s）"; break
  fi
  sleep 1
  if [ "$i" = "60" ]; then
    echo "  ✗ 等了 60 s 还没就绪。看看："
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
  while true; do sleep 3600; done
fi

# ================================================================ 终端 4：发目标
subs=$(dc "timeout 4 ros2 topic info /press/target_pose 2>/dev/null | sed -n 's/^Subscription count: *//p'" 2>/dev/null)
board=$(dc "timeout 4 ros2 topic info /perception/board_pose 2>/dev/null | sed -n 's/^Publisher count: *//p'" 2>/dev/null)
if [ "${subs:-0}" != "1" ] || [ "${board:-0}" = "0" ]; then
  echo "★ 服务没在跑。先在另一个终端起：tools/press_board.sh --serve [--live]"
  echo "   （按压任务订阅者=${subs:-0}，板子位姿发布者=${board:-0}）"
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
  for i in $(seq 1 40); do
    if dc "grep -q 'Press .* finished' /tmp/press.log" 2>/dev/null; then break; fi
    sleep 1
  done
  echo
  dc "grep -viE 'corrupt|Select|ompl|moveit_ros|planner configuration' /tmp/press.log | tail -18"
  echo
  dc "grep -E 'Actual TCP at approach end' /tmp/press.log" 2>/dev/null \
    | sed 's/^/  → /'
  echo
  echo "  ⚠️ 这行是**控制器跟踪误差**，不是标定误差。落点准不准要看板上的压痕。"
fi
