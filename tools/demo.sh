#!/usr/bin/env bash
#
# 一键 demo —— 一个窗口跑完原来的「终端 3（服务）+ 终端 4（发目标）」。
#
#   tools/demo.sh                 干跑：起服务 + 瞄准一次，机械臂不动
#   tools/demo.sh --live          实机：起服务 + 真按一次
#   tools/demo.sh --live --button call   按 board.yaml 里量好的那个按键，而不是板心
#   tools/demo.sh --offset-mm -3,0,0
#
# ⚠️ 按压参数（接近段 standoff、按压行程、干跑/实机）**不在命令行上**，在：
#
#      src/Piper_ros/src/piper_elevator_manipulation/config/press_params.yaml
#
#    改那个文件，**下一次按压就生效** —— 不用重起服务、不用重新编译。
#    节点每次按压前都会重新读它。
#
# ⚠️ --button 的几何（按键相对板子在哪、往哪个方向压）在另一个文件：
#
#      src/Piper_ros/src/piper_marker_perception/config/board.yaml
#
#    量完把对应的 taught 改成 true，否则会被拦住（那是占位值，不是量出来的）。
#
# 按完**不退出**，停在交互循环里（那时服务还在，回车就能再按一次）：
#
#     回车 = 再按一次        a = 只瞄准不按        q = 退出并清理
#
# Ctrl-C 一样会清理。绝不动 bringup。
#
# 为什么合并成一个脚本：终端 3 只是「把节点起起来然后常驻」，终端 4 只是「发一条
# 目标」，中间没有任何需要人盯着的地方。分开的唯一后果是多一个窗口、多一处忘记
# 回收 —— 历史上那个把**后起**的 move_group 按名字误杀的残留实例，就是这么来的。
#
# ⚠️ --offset-mm 不用重起，它每次瞄准时现用。

set -uo pipefail

LIVE=0
OFFSET="0,0,0"
SETTLE=6
BUTTON_ARG=()
JITTER_ARG=()

while [ $# -gt 0 ]; do
  case "$1" in
    --live)      LIVE=1 ;;
    --offset-mm) OFFSET="$2"; shift ;;
    --settle)    SETTLE="$2"; shift ;;
    --button)    BUTTON_ARG=(--button "$2"); shift ;;
    # 稳定性闸门，毫米。默认 2.0 是按 3×3 标定板的噪声底定的；
    # 单个小 marker 的噪声底高得多（28.5 mm 的单码实测 3.3 mm），
    # 会被误报成"板子在动"。
    --max-jitter-mm) JITTER_ARG=(--max-jitter-mm "$2"); shift ;;
    -h|--help)   sed -n '2,20p' "$0"; exit 0 ;;
    --standoff|--depth)
      echo "★ --standoff / --depth 已经搬到配置文件里了，不再从命令行传："
      echo
      echo "     src/Piper_ros/src/piper_elevator_manipulation/config/press_params.yaml"
      echo
      echo "   改那个文件，下一次按压就生效（不用重起服务、不用重新编译）。"
      exit 2 ;;
    *) echo "不认识的参数：$1"; exit 2 ;;
  esac
  shift
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PB="$HERE/press_board.sh"
PARAMS_FILE="$(cd "$HERE/.." && pwd)/src/Piper_ros/src/piper_elevator_manipulation/config/press_params.yaml"
SERVE_LOG=/tmp/demo_serve.log
SERVE_PID=""

# 回收一定发生：服务是本脚本的子进程，关窗口 / Ctrl-C / q 都走这里。
# ⚠️ 先 kill 子进程再 --down：--serve 自己的 trap 也会清一遍，但那是它自己的
#    令牌还成立的时候；这里再 --down 一次是 FORCE，保证没有漏网。
teardown() {
  trap - EXIT INT TERM
  echo
  echo "════ 收尾 ════"
  if [ -n "$SERVE_PID" ] && kill -0 "$SERVE_PID" 2>/dev/null; then
    kill "$SERVE_PID" 2>/dev/null
    sleep 1
  fi
  "$PB" --down
}
trap teardown EXIT INT TERM

press_once() {
  local flag=""
  [ "$LIVE" = "1" ] && flag="--live"
  # shellcheck disable=SC2086
  "$PB" $flag --settle "$SETTLE" --offset-mm "$OFFSET" "${BUTTON_ARG[@]}" "${JITTER_ARG[@]}"
}

# ---------------------------------------------------------------- 起服务
echo "════ 清掉上一轮残留 ════"
"$PB" --down 2>&1 | grep -E '已清理|bringup' | sed 's/^/  /'

echo
echo "════ 起服务（原终端 3）════"
serve_flag=""
[ "$LIVE" = "1" ] && serve_flag="--live"
# shellcheck disable=SC2086
# 按压参数不在这里传：它们在 config/press_params.yaml 里，节点每次按压前现读。
# 传 --standoff/--depth 只会被那个文件盖掉，还多一条 WARN，所以干脆不传。
# 干跑/实机闸门**仍然只从命令行来**（--live），刻意不放进那个文件。
"$PB" --serve $serve_flag > "$SERVE_LOG" 2>&1 &
SERVE_PID=$!

ready=0
for i in $(seq 1 90); do
  if grep -q "服务常驻中" "$SERVE_LOG" 2>/dev/null; then
    grep -E '位置|can0|相机|残留|bringup|standoff|press_depth|✓' "$SERVE_LOG" | sed 's/^/  /'
    ready=1
    break
  fi
  if ! kill -0 "$SERVE_PID" 2>/dev/null; then
    echo "  ✗ 服务没起来。它自己的输出："
    sed 's/^/     /' "$SERVE_LOG"
    exit 1
  fi
  sleep 1
done
if [ "$ready" != "1" ]; then
  echo "  ✗ 等了 90 s 还没就绪。日志（$SERVE_LOG）尾巴："
  tail -20 "$SERVE_LOG" | sed 's/^/     /'
  exit 1
fi

echo
if [ "${#BUTTON_ARG[@]}" -gt 0 ]; then
  echo "  ★ 按压对象：按键 ${BUTTON_ARG[1]}（来自 board.yaml），不是板心"
fi
# 把配置文件里的值原样摆出来 —— 操作者要的就是"现在到底会怎么按"。
if [ -f "$PARAMS_FILE" ]; then
  echo "════ 按压参数（来自配置文件）════"
  grep -E '^[a-z_]+:' "$PARAMS_FILE" | sed 's/^/  /'
  echo "  文件：$PARAMS_FILE"
  echo "  ★ 改这个文件，下一次按压就生效，不用重起。"
else
  echo "  ⚠️ 配置文件不在：$PARAMS_FILE"
  echo "     节点会退回 launch 参数（默认 standoff 5 mm / depth 4 mm）。"
fi

echo
echo "  实时画面（带 marker 标注和板中心坐标系）：http://127.0.0.1:8098/"
echo "  （相机是独占的，所以感知在跑的时候 cam_view.py 打不开 —— 要看就看这个。）"
echo "  服务日志：$SERVE_LOG"
if [ "$LIVE" != "1" ]; then
  echo
  echo "  ⚠️ 现在是**干跑**（任务不动臂）。要真按：加 --live 重跑本脚本。"
fi

# ---------------------------------------------------------------- 第一轮
echo
press_once
first_rc=$?

# ---------------------------------------------------------------- 交互循环
if [ "$LIVE" != "1" ]; then
  echo
  echo "════ 只算不发 ════ 什么都没动。"
fi

while true; do
  echo
  printf "══ 回车 = 再按一次   a = 只瞄准   q = 退出并清理 ══\n> "
  if ! read -r ans; then
    echo
    break
  fi
  case "$ans" in
    q|Q|quit|exit) break ;;
    a|A)
      saved="$LIVE"; LIVE=0; press_once; LIVE="$saved"
      ;;
    *) press_once ;;
  esac
done
