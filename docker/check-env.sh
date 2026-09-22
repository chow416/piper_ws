#!/usr/bin/env bash
# =============================================================================
#  容器环境自检 —— 进容器后直接敲 `check-env`
#  逐项回答："这个容器到底能不能跑 RViz / Gazebo / 实体机械臂？"
# =============================================================================
ROS_DISTRO="${ROS_DISTRO:-humble}"

c_ok()   { printf "  \033[1;32m✓\033[0m %-26s %s\n" "$1" "$2"; }
c_warn() { printf "  \033[1;33m!\033[0m %-26s %s\n" "$1" "$2"; }
c_bad()  { printf "  \033[1;31m✗\033[0m %-26s %s\n" "$1" "$2"; }
hdr()    { printf "\n\033[1;36m── %s\033[0m\n" "$1"; }

hdr "系统"
c_ok "发行版" "$(. /etc/os-release; echo "$PRETTY_NAME")"
c_ok "Python" "$(python3 -V 2>&1 | awk '{print $2}')"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    c_ok "ROS 2 ${ROS_DISTRO}" "$(dpkg-query -W -f='${Version}' ros-${ROS_DISTRO}-desktop 2>/dev/null || echo '已安装')"
else
    c_bad "ROS 2 ${ROS_DISTRO}" "未安装"
fi
c_ok "locale" "LANG=${LANG}  LC_NUMERIC=${LC_NUMERIC:-未设置}"
if [ "${LC_NUMERIC}" = "en_US.UTF-8" ] || [ "${LC_ALL}" = "en_US.UTF-8" ]; then :; else
    c_warn "locale" "LC_NUMERIC 非 en_US.UTF-8，MoveIt 参数解析可能报错"
fi

hdr "关键组件"
for p in \
    "moveit :ros-${ROS_DISTRO}-moveit" \
    "ros2_control :ros-${ROS_DISTRO}-ros2-control" \
    "controller_manager :ros-${ROS_DISTRO}-controller-manager" \
    "Gazebo Classic :ros-${ROS_DISTRO}-gazebo-ros-pkgs" \
    "Nav2 :ros-${ROS_DISTRO}-nav2-bringup" \
    "slam_toolbox :ros-${ROS_DISTRO}-slam-toolbox" ; do
    name=${p%%:*}; pkg=${p##*:}
    v=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null || true)
    [ -n "$v" ] && c_ok "$name" "$v" || c_warn "$name" "未安装（$pkg）"
done

hdr "Python 库"
python3 - <<'PY'
import importlib, sys
for mod, name in [("can", "python-can"), ("piper_sdk", "piper_sdk"),
                  ("transforms3d", "transforms3d"), ("numpy", "numpy"),
                  ("scipy", "scipy")]:
    try:
        m = importlib.import_module(mod)
        v = getattr(m, "__version__", "?")
        print(f"  \033[1;32m✓\033[0m {name:<26} {v}")
    except Exception as e:
        print(f"  \033[1;31m✗\033[0m {name:<26} 导入失败: {e}")
PY

hdr "图形（RViz / Gazebo 显示）"
echo "  DISPLAY=${DISPLAY:-未设置}"
if [ -z "${DISPLAY}" ]; then
    c_bad "X11" "DISPLAY 未设置 —— RViz/Gazebo 无法显示"
    echo "      宿主上执行一次:  xhost +local:"
    echo "      然后确认 compose 挂载了 /tmp/.X11-unix 并透传了 DISPLAY"
elif [ ! -S "/tmp/.X11-unix/X${DISPLAY#*:}" ]; then
    c_bad "X11 socket" "/tmp/.X11-unix/X${DISPLAY#*:} 不存在 —— 宿主上执行: xhost +local: 后重开容器"
else
    c_ok "X11 socket" "/tmp/.X11-unix/X${DISPLAY#*:}"
fi

mesa=$(dpkg-query -W -f='${Version}' libgl1-mesa-dri 2>/dev/null || echo 未知)
c_ok "Mesa 版本" "${mesa}"
if command -v glxinfo >/dev/null 2>&1; then
    rend=$(glxinfo -B 2>/dev/null | awk -F': ' '/^[[:space:]]*(OpenGL )?renderer string/{print $2; exit}')
    accel=$(glxinfo -B 2>/dev/null | awk -F': ' '/Accelerated/{print $2; exit}')
    if echo "${rend}" | grep -qiE "llvmpipe|softpipe|swrast"; then
        c_warn "OpenGL 渲染器" "${rend}  （软件渲染：功能正常，帧率较低）"
        echo "      要启用硬件加速：--build-arg ENABLE_MESA_PPA=1 重建，并挂载 /dev/dri"
    else
        c_ok "OpenGL 渲染器" "${rend}  (加速=${accel:-?})"
    fi
else
    c_warn "glxinfo" "未安装，无法判断渲染器"
fi

if [ -e /dev/dri/renderD128 ]; then
    c_ok "/dev/dri/renderD128" "已透传 ($(stat -c '%G %a' /dev/dri/renderD128))"
    echo "${groups}" | grep -qE '\brender\b' && c_ok "render 组" "已在组内" || \
        c_warn "render 组" "当前用户不在 render 组（$(id -nG)）"
else
    c_warn "/dev/dri" "未透传 —— 只能用软件渲染"
fi

hdr "实体机械臂（CAN）"
if ip link show can0 >/dev/null 2>&1; then
    st=$(ip -br link show can0 | awk '{print $2}')
    if [ "${st}" = "UP" ]; then c_ok "can0" "已 UP  ${st}"; else
        c_warn "can0" "存在但未 UP —— sudo ip link set can0 up type can bitrate 1000000"; fi
elif command -v ip >/dev/null 2>&1 && ip link show type can 2>/dev/null | grep -q .; then
    c_warn "CAN 接口" "$(ip -br link show type can | awk '{print $1}' | tr '\n' ' ')（未命名为 can0）"
else
    c_warn "CAN 接口" "无 can0。USB-CAN 需 --device=/dev/ttyACM0；SocketCAN 需 USB-CAN 内核驱动 + privileged"
fi
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | sed 's/^/     串口: /' || true

hdr "工作区"
if [ -d /ws ]; then
    c_ok "/ws" "$(ls -d /ws/src/*/ 2>/dev/null | wc -l) 个源码包"
    [ -f /ws/install/setup.bash ] && c_ok "已编译" "/ws/install 存在" || \
        c_warn "未编译" "在 /ws 下执行: colcon build --symlink-install"
    ws_owner=$(stat -c '%U' /ws)
    [ "${ws_owner}" = "$(id -un)" ] && c_ok "属主" "${ws_owner}" || \
        c_warn "属主" "属主是 ${ws_owner}，当前是 $(id -un)（可能写不进去）"
else
    c_bad "/ws" "未挂载工作区"
fi

echo
