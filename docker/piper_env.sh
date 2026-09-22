# =============================================================================
#  容器内附加环境变量 —— 被 entrypoint 自动 source
#  改完只需要重启容器，不用重建镜像：
#     docker compose restart
# =============================================================================

# --- ROS 2 -------------------------------------------------------------------
# 同一网络里跑多组实验时，给每组分配不同 DOMAIN_ID，避免互相串数据
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# DDS 实现：rmw_fastrtps_cpp（默认）| rmw_cyclonedds_cpp
# 局域网多机联调、或组播不稳时，CycloneDDS 往往更顺
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# --- PiPER 实体机械臂 ---------------------------------------------------------
# CAN 接口名（SocketCAN）或串口（USB-CAN 适配器）
export PIPER_CAN_PORT="${PIPER_CAN_PORT:-can0}"

# piper_sdk / Piper_ros 里有些示例依赖这个变量定位 URDF 与 mesh
export PIPER_URDF_DIR="${PIPER_URDF_DIR:-/ws/src/Piper_ros/src/piper_description/urdf}"

# --- 图形 ---------------------------------------------------------------------
# RViz 里用软件渲染画大点云时，可关掉抗锯齿省点性能
export QT_X11_NO_MITSHM=1

# --- 下面按需放开 -------------------------------------------------------------

# 有多台机器时，把 ROS 2 通信限制在某个网卡上（多网卡机器上很有用）
# export ROS_LOCALHOST_ONLY=1
# export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
