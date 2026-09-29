#!/usr/bin/env bash
# 在开发容器里跑一条命令 —— 把那一长串 sg/docker/source 样板收成一个词。
#
#   source ~/piper_ws/tools/piper.sh      # 或写进 ~/.bashrc
#   piper ros2 topic list
#   piper python3 tools/press_board_centre.py --settle 6
#
# 为什么要它：容器里的工作区是 /ws，ROS 在 /opt/ros/humble，还需要 source
# install/setup.bash。这三件事每次都得写全，写漏 source 的表现是「命令找不到」
# 或者更糟 —— 找到的是宿主上那份，报的错完全对不上。
#
# ⚠️ **两种层次都要能用**（2026-09-29 用户踩过）：
#     宿主机提示符  zlf@zlf-ZhaoYang-X5-14-IAL:~/piper_ws$
#     容器内提示符  dev@piper-dev:/ws$
# 在**容器里**敲 `sg docker -c 'docker exec …'` 只会得到
# `sg: group 'docker' does not exist` —— 因为容器里没有 docker 这条命令，
# 更没有 docker 组。所以这里先判断层次，容器里就当普通函数用。
#
# 判据是**工具目录自己在哪**，不是 /opt/ros/humble —— 宿主上也装了 ROS，
# 拿它当判据会两边都判成容器（实测踩过）。
PIPER_TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "$PIPER_TOOLS" in
  /ws/tools) PIPER_IN_CONTAINER=1 ;;
  *)         PIPER_IN_CONTAINER=0 ;;
esac

piper() {
  if [ "$PIPER_IN_CONTAINER" = "1" ]; then
    if [ $# -eq 0 ]; then exec bash -l; fi
    # set +u 两次：调用者的 shell 可能开了 nounset，而 ROS 的 setup.bash
    # 会引用未定义变量（例如 $AMENT_TRACE_SETUP_FILES），一 source 就退出。
    ( set +u; source /opt/ros/humble/setup.bash
      source /ws/install/setup.bash && cd /ws && eval "$*" )
    return $?
  fi
  if [ $# -eq 0 ]; then
    sg docker -c "docker exec -it piper_dev bash -l"
    return
  fi
  # -i 保证 stdin 能进容器（Ctrl-C 能传下去）；-t 只在真的有终端时加，
  # 否则从脚本里调用会报 "the input device is not a TTY"。
  local flags="-i"
  [ -t 0 ] && flags="-it"
  sg docker -c "docker exec $flags piper_dev bash -lc \
'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && cd /ws && $*'"
}

# 停掉所有会挡路的进程，但**不动 bringup** —— 那是机械臂的使能，停了臂会垂下来。
# 转调 press_board.sh，它自己会判断在宿主机还是容器里，两处不用各维护一份。
piper_down() {
  "$PIPER_TOOLS/press_board.sh" --down
}

# 前提检查。任何一项不过就别往下走。
piper_check() {
  "$PIPER_TOOLS/press_board.sh" --check
}
