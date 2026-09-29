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

piper() {
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
piper_down() {
  sg docker -c "docker exec piper_dev bash -lc '
    pkill -f press_button_hardware.launch.py
    pkill -f press_button_demo
    pkill -f marker_locator.py
    sleep 1
    pkill -f move_group
    sleep 1
  '"
  echo "已停：press_button_hardware / press_button_demo / marker_locator / move_group"
  echo "⚠️ bringup 仍然在跑 —— 机械臂还使能着，能动是正常的"
}

# 前提检查。任何一项不过就别往下走。
piper_check() {
  local bad=0
  echo "容器   $(sg docker -c 'docker ps --filter name=piper_dev --format "{{.Status}}"')"
  if [ "$(cat /sys/class/net/can0/flags 2>/dev/null | python3 -c 'import sys;print(int(sys.stdin.read().strip(),16)&1)' 2>/dev/null)" = "1" ]; then
    echo "can0    UP"
  else
    echo "can0    ✗ 没起来 —— sudo ip link set can0 up type can bitrate 1000000"
    bad=1
  fi
  if fuser /dev/video2 >/dev/null 2>&1; then
    echo "相机    ✗ 被占用 —— 先 pkill -f cam_view.py"
    bad=1
  else
    echo "相机    空闲"
  fi
  local left
  left=$(sg docker -c "docker exec piper_dev bash -lc 'pgrep -a -f \"move_group|press_button|marker_locator\"'" 2>/dev/null)
  if [ -n "$left" ]; then
    echo "残留    ✗ 上一轮没清干净："
    echo "$left" | sed 's/^/          /'
    bad=1
  else
    echo "残留    干净"
  fi
  return $bad
}
