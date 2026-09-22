#!/usr/bin/env bash
# =============================================================================
#  容器入口脚本
#   · 自动 source ROS 2 环境
#   · 若 /ws 已 colcon build 过，自动 source install/setup.bash
#   · 启动时做一次环境自检（图形、CAN、工作区状态）
# =============================================================================
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"

# --- ROS 2 基础环境 ---------------------------------------------------------
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "[warn] 未找到 /opt/ros/${ROS_DISTRO}/setup.bash" >&2
fi

# --- 工作区 overlay ---------------------------------------------------------
for ws in /ws "${HOME}/piper_ws" /opt/piper_ws; do
    if [ -f "${ws}/install/setup.bash" ]; then
        # shellcheck disable=SC1090
        source "${ws}/install/setup.bash"
        echo "[info] 已加载工作区: ${ws}/install/setup.bash"
        break
    fi
done

# --- 额外环境文件（挂载进来即可生效，不用重建镜像） -------------------------
if [ -f /etc/piper_env.sh ]; then
    # shellcheck disable=SC1091
    source /etc/piper_env.sh
fi

# --- 首次进入时的自检 -------------------------------------------------------
if [ -t 0 ] && [ "${PIPER_QUIET:-0}" != "1" ]; then
    if [ -z "${PIPER_BANNER_SHOWN:-}" ]; then
        export PIPER_BANNER_SHOWN=1
        echo "───────────────────────────────────────────────────────────────"
        echo " PiPER 开发容器   ROS 2 ${ROS_DISTRO}   Ubuntu $(. /etc/os-release && echo "$VERSION_ID")"
        echo " 工作区 /ws   →   $(ls -d /ws/src/* 2>/dev/null | wc -l) 个源码目录"
        printf ' Mesa: %s\n' "$(dpkg-query -W -f='${Version}' libgl1-mesa-dri 2>/dev/null || echo '未知')"

        if [ -S /tmp/.X11-unix/X"${DISPLAY#*:}" ] 2>/dev/null && [ -n "${DISPLAY}" ]; then
            if command -v glxinfo >/dev/null 2>&1; then
                rend=$(glxinfo -B 2>/dev/null | awk -F': ' '/^[[:space:]]*(OpenGL )?renderer string/{print $2; exit}')
                printf ' 图形: DISPLAY=%s  渲染器=%s\n' "${DISPLAY}" "${rend:-未知}"
            else
                printf ' 图形: DISPLAY=%s（未安装 glxinfo）\n' "${DISPLAY}"
            fi
        else
            printf ' 图形: DISPLAY=%s（X11 socket 未挂载或不可用，RViz/Gazebo 无法显示）\n' "${DISPLAY:-未设置}"
        fi

        if ip link show can0 >/dev/null 2>&1; then
            echo " CAN: can0 已存在（$(ip -br link show can0 | awk '{print $2}')）"
        elif ls /dev/ttyACM* /dev/ttyUSB* >/dev/null 2>&1; then
            echo " CAN: 检测到串口设备 $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
        else
            echo " CAN: 未检测到 can0 / USB 串口设备（未透传或未插）"
        fi

        cat <<'TIP'
 ───────────────────────────────────────────────────────────────
  常用命令：
    colcon build --symlink-install            # 编译工作区
    ros2 launch piper_description display.launch.py
    ros2 launch piper_moveit_config demo.launch.py
    ros2 launch piper start_single_piper.launch.py can_port:=can0
  环境自检：  check-env
 ───────────────────────────────────────────────────────────────
TIP
    fi
fi

exec "$@"
