#!/usr/bin/env bash
#
# 一次完整的按压周期：起任务 → 起感知 → 瞄准 →（可选）按下去。
#
#   tools/press_board.sh                  # 干跑：完整规划，不动
#   tools/press_board.sh --live           # 真机按下去
#   tools/press_board.sh --live --offset-mm -3,0,0
#   tools/press_board.sh --down           # 只清理
#
# 只做了一件事：把原本要在三个终端里分别敲的启动命令收进一个脚本，并且
# **保证退出时清理干净**。清理这件事之前手动做过两次，两次都漏了 move_group
# （它不跟着 ros2 launch 的 PID 退），于是下一个进程看到一个多余的规划器，
# 症状是「臂反应很慢」—— 而真因要查很久才找得到。
#
# ⚠️ 这个脚本**不动 bringup**。机械臂的使能开关（piper_real_bringup）在另一个
# 终端里，那是操作者的急停手柄；脚本退出时臂仍然使能、仍然保持位姿，这是对的。

set -uo pipefail

DRY=1
LIVE=0
DOWN=0
KEEP=0
SETTLE=6
OFFSET="0,0,0"
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

while [ $# -gt 0 ]; do
  case "$1" in
    --live)        LIVE=1; DRY=0 ;;
    --dry-run)     DRY=1; LIVE=0 ;;
    --down)        DOWN=1 ;;
    --keep)        KEEP=1 ;;
    --settle)      SETTLE="$2"; shift ;;
    --offset-mm)   OFFSET="$2"; shift ;;
    -h|--help)     sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "不认识的参数：$1"; exit 2 ;;
  esac
  shift
done

PRELUDE='source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && cd /ws'

# 命令走 stdin 而不是 -c，因为传给它的命令里经常带单引号，
# 而这里外面已经套了 sg 的双引号和 bash 的单引号 —— 三层引号叠起来必炸一次。
dc()    { sg docker -c "docker exec -i piper_dev bash" <<<"$PRELUDE && $*"; }
dc_bg() { sg docker -c "docker exec -d piper_dev bash -lc \"$PRELUDE && $*\""; }

cleanup() {
  dc "pkill -f press_button_hardware.launch.py; pkill -f press_button_demo; pkill -f marker_locator.py" >/dev/null 2>&1
  sleep 2
  dc "pkill -f move_group" >/dev/null 2>&1   # ⚠️ 不跟着 launch 退，必须单独来一下
  echo
  echo "—— 已清理：press_button_hardware / press_button_demo / marker_locator / move_group"
  echo "—— bringup 仍在跑，机械臂使能中、保持位姿"
}

if [ "$DOWN" = "1" ]; then cleanup; exit 0; fi

# ---------------------------------------------------------------- 前提
echo "════ 前提检查 ════"
bad=0
status=$(sg docker -c 'docker ps --filter name=piper_dev --format "{{.Status}}"' 2>/dev/null)
if [ -n "$status" ]; then echo "  容器    $status"; else echo "  容器    ✗ 没在跑"; bad=1; fi

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

left=$(dc "pgrep -a -f 'move_group|press_button|marker_locator'" 2>/dev/null)
if [ -n "$left" ]; then
  echo "  残留    ✗ 上一轮没清干净："; echo "$left" | sed 's/^/            /'
  echo "            —— 跑 tools/press_board.sh --down 清掉"; bad=1
else
  echo "  残留    干净"
fi

if ! pgrep -f "ros2_control_node" >/dev/null 2>&1 && \
   ! dc "pgrep -f ros2_control_node" >/dev/null 2>&1; then
  echo "  bringup ✗ 没在跑 —— 先在另一个终端起 piper_real_bringup（见 PRESS_TO_BOARD.md 第 1 节）"
  bad=1
else
  echo "  bringup 在跑（电机使能中）"
fi

if [ "$bad" = "1" ]; then echo; echo "★ 前提不满足，没起任何东西。"; exit 1; fi

trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- 起节点
if [ "$LIVE" = "1" ]; then EXEC=true; ENABLE=true; else EXEC=false; ENABLE=false; fi
echo
echo "════ 启动 ════  execute=$EXEC hardware_enable=$ENABLE"
dc_bg "ros2 run piper_marker_perception marker_locator.py --ros-args \
  --params-file /ws/install/piper_marker_perception/share/piper_marker_perception/config/marker_locator_orbbec.yaml \
  > /tmp/ml.log 2>&1"
dc_bg "ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
  target_source:=topic max_presses:=1 execute:=$EXEC hardware_enable:=$ENABLE rviz:=false \
  > /tmp/press.log 2>&1"

echo "  等节点起来（最多 60 s）..."
for i in $(seq 1 60); do
  subs=$(dc "timeout 4 ros2 topic info /press/target_pose 2>/dev/null | awk '/Subscription count/{print \$3}'" 2>/dev/null)
  board=$(dc "timeout 4 ros2 topic info /perception/board_pose 2>/dev/null | awk '/Publisher count/{print \$3}'" 2>/dev/null)
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

# ---------------------------------------------------------------- 瞄准
echo
echo "════ 瞄准 ════"
if [ "$LIVE" = "1" ]; then PUB="--publish"; else PUB=""; fi
dc "python3 tools/press_board_centre.py --settle $SETTLE --offset-mm $OFFSET $PUB"
aim_rc=$?

if [ "$aim_rc" != "0" ]; then
  echo; echo "★ 瞄准脚本退出码 $aim_rc —— 没有发目标。"; exit $aim_rc
fi

# ---------------------------------------------------------------- 等结果
if [ "$LIVE" = "1" ]; then
  echo
  echo "════ 执行中 ════"
  for i in $(seq 1 40); do
    if dc "grep -q 'Press 1 finished' /tmp/press.log" 2>/dev/null; then break; fi
    sleep 1
  done
  echo
  dc "grep -viE 'corrupt|Select|ompl|moveit_ros|planner configuration' /tmp/press.log | tail -18"
  echo
  dc "grep -E 'Actual TCP at approach end' /tmp/press.log" 2>/dev/null \
    | sed 's/^/  → /'
  echo
  echo "  ⚠️ 上面这行是**控制器跟踪误差**，不是标定误差。落点准不准要看板上的压痕。"
else
  echo
  echo "════ 干跑完成 ════ 什么都没动。"
  echo "  加 --live 才会真按。"
fi

# ---------------------------------------------------------------- 留或不留
if [ "$KEEP" = "1" ]; then
  echo
  echo "—— --keep：节点都留着。再瞄一次（另开终端）："
  echo "     cd $WS_ROOT && source tools/piper.sh && piper python3 tools/press_board_centre.py --settle $SETTLE"
  echo "   Ctrl-C 结束并清理。"
  while true; do sleep 3600; done
fi
