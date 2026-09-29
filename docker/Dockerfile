# syntax=docker/dockerfile:1
# =============================================================================
#  PiPER 机械臂开发环境容器
#  基座：Ubuntu 22.04 LTS  +  ROS 2 Humble Hawksbill
#
#  ── 为什么是 22.04 + Humble ──────────────────────────────────────────────
#   · 与松灵 Piper_ros 的 humble 分支完全对齐，URDF / MoveIt 配置 / 消息定义
#     零改动即可编译
#   · 22.04 仓库仍含 Gazebo Classic 11，原版 piper_gazebo 仿真链路可用
#   · 自带 Python 3.10，piper_sdk / python-can 直接 pip 装
#
#  ── 为什么用容器 ─────────────────────────────────────────────────────────
#   · 环境即镜像：同事只需装 Docker，无需关心自己的 Ubuntu / ROS / Python 版本
#   · 宿主 24.04 的图形栈保持不动，不会再触发 Arrow Lake-P 核显的黑屏问题
#
#  ── 构建 ─────────────────────────────────────────────────────────────────
#   默认走清华镜像（国内速度正常）：
#     docker build -t piper-dev:humble .
#
#   宿主是新款 Intel 核显（Arrow Lake / Lunar Lake 等），想让容器内也拿到
#   硬件加速（而不是退回 CPU 软件渲染），加 PPA 把容器内 Mesa 升到 25.x：
#     docker build --build-arg ENABLE_MESA_PPA=1 -t piper-dev:humble .
#
#   海外协作者改用官方源：
#     docker build \
#       --build-arg UBUNTU_MIRROR=archive.ubuntu.com \
#       --build-arg ROS2_MIRROR=packages.ros.org/ros2/ubuntu \
#       --build-arg ROSDISTRO_MIRROR=raw.githubusercontent.com/ros/rosdistro/master \
#       --build-arg PIP_INDEX=https://pypi.org/simple \
#       -t piper-dev:humble .
# =============================================================================

ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

# ---------------------------------------------------------------------------
# 可配置项
# ---------------------------------------------------------------------------
ARG UBUNTU_MIRROR=mirrors.tuna.tsinghua.edu.cn
ARG ROS2_MIRROR=mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu
ARG ROSDISTRO_MIRROR=mirrors.tuna.tsinghua.edu.cn/rosdistro
ARG PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple

# 1 = 加入 kisak/kisak-mesa PPA，把容器内 Mesa 从 22.0.1 升到 25.x
#     （仅当宿主是新款 Intel 核显、需要容器内硬件加速时才开）
ARG ENABLE_MESA_PPA=0

ARG ROS_DISTRO=humble
ARG USER_NAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Asia/Shanghai

# ---------------------------------------------------------------------------
# 1) APT 换源 + 基础工具链
# ---------------------------------------------------------------------------
RUN set -eux; \
    # ubuntu:22.04 的最小基础镜像未必包含 CA 证书库。必须先通过它
    # 自带的 HTTP 源安装 ca-certificates，之后才能安全切到 HTTPS 镜像。
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates; \
    if [ "${UBUNTU_MIRROR}" != "archive.ubuntu.com" ]; then \
        sed -i \
            -e "s|http://archive.ubuntu.com/ubuntu|https://${UBUNTU_MIRROR}/ubuntu|g" \
            -e "s|http://security.ubuntu.com/ubuntu|https://${UBUNTU_MIRROR}/ubuntu|g" \
            -e "s|http://ports.ubuntu.com|https://${UBUNTU_MIRROR}/ubuntu-ports|g" \
            /etc/apt/sources.list; \
    fi; \
    printf 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\n' > /etc/apt/apt.conf.d/80-piper-retries; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates curl wget gnupg lsb-release \
        apt-utils locales tzdata sudo \
        git vim nano less tmux htop tree unzip bash-completion \
        build-essential cmake ninja-build pkg-config ccache \
        python3-pip python3-dev python3-venv python3-setuptools \
        iproute2 iputils-ping net-tools usbutils udev kmod \
        can-utils v4l-utils \
        x11-utils x11-apps mesa-utils \
        libgl1 libglu1-mesa libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libgbm1 \
        dbus-x11 \
    ; \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2) 语言环境
#    注意 LC_NUMERIC 显式设为 en_US.UTF-8 —— 某些中文 locale 下 MoveIt / rcl
#    解析带小数的参数会失败，这是踩过的坑，直接在镜像里钉死。
# ---------------------------------------------------------------------------
RUN set -eux; \
    sed -i 's/^# *\(en_US.UTF-8\)/\1/' /etc/locale.gen; \
    locale-gen; \
    update-locale LANG=en_US.UTF-8 LC_NUMERIC=en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8 \
    LC_NUMERIC=en_US.UTF-8

# ---------------------------------------------------------------------------
# 3) ROS 2 apt 源
#    密钥走 rosdistro 镜像（清华 /rosdistro/ros.key，实测 HTTP 200）。
#    rosdep 索引同理：rosdep 默认去 raw.githubusercontent.com 取 index-v4.yaml，
#    国内不通，用 ROSDISTRO_INDEX_URL 指到镜像。
# ---------------------------------------------------------------------------
ENV ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml

RUN set -eux; \
    curl -fsSL "https://${ROSDISTRO_MIRROR}/ros.key" \
        -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    . /etc/os-release; \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://${ROS2_MIRROR} ${VERSION_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list; \
    apt-get update

# ---------------------------------------------------------------------------
# 4) ROS 2 Humble Desktop + 开发工具
# ---------------------------------------------------------------------------
RUN set -eux; \
    apt-get install -y \
        "ros-${ROS_DISTRO}-desktop" \
        ros-dev-tools \
    ; \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 5) 运动规划 / 控制 / 仿真相关（PiPER 与课题都需要）
# ---------------------------------------------------------------------------
RUN set -eux; \
    apt-get update; \
    apt-get install -y \
        "ros-${ROS_DISTRO}-moveit" \
        "ros-${ROS_DISTRO}-moveit-ros-perception" \
        "ros-${ROS_DISTRO}-moveit-setup-assistant" \
        "ros-${ROS_DISTRO}-pilz-industrial-motion-planner" \
        "ros-${ROS_DISTRO}-ros2-control" \
        "ros-${ROS_DISTRO}-ros2-controllers" \
        "ros-${ROS_DISTRO}-controller-manager" \
        "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
        "ros-${ROS_DISTRO}-xacro" \
        "ros-${ROS_DISTRO}-robot-state-publisher" \
        "ros-${ROS_DISTRO}-tf-transformations" \
        "ros-${ROS_DISTRO}-plotjuggler-ros" \
        "ros-${ROS_DISTRO}-ros2bag" \
        "ros-${ROS_DISTRO}-diagnostic-updater" \
    ; \
    apt-get install -y \
        "ros-${ROS_DISTRO}-gazebo-ros-pkgs" \
        "ros-${ROS_DISTRO}-gazebo-plugins" \
        "ros-${ROS_DISTRO}-gazebo-ros2-control" \
    ; \
    apt-get install -y \
        "ros-${ROS_DISTRO}-nav2-bringup" \
        "ros-${ROS_DISTRO}-slam-toolbox" \
        "ros-${ROS_DISTRO}-robot-localization" \
    ; \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 6) 可选：容器内升级 Mesa
#    22.04 出厂 Mesa 22.0.1 不认识 Arrow Lake-P（8086:7d51），容器内会静默
#    退回 llvmpipe 软件渲染（可用，但帧率低）。开这个开关即可拿到 Mesa 25.x。
# ---------------------------------------------------------------------------
RUN set -eux; \
    if [ "${ENABLE_MESA_PPA}" = "1" ]; then \
        apt-get update; \
        apt-get install -y --no-install-recommends software-properties-common; \
        add-apt-repository -y ppa:kisak/kisak-mesa; \
        apt-get update; \
        apt-get install -y \
            libgl1-mesa-dri libglx-mesa0 libegl-mesa0 libgbm1 \
            libgl1 libglu1-mesa mesa-vulkan-drivers mesa-utils; \
        rm -rf /var/lib/apt/lists/*; \
        echo "--- 容器内 Mesa 版本 ---"; \
        dpkg-query -W -f='${Package} ${Version}\n' libgl1-mesa-dri || true; \
    else \
        echo "[info] 未启用 Mesa PPA，容器内将使用 22.04 自带的 Mesa 22.0.1"; \
        echo "[info] 传入 --build-arg ENABLE_MESA_PPA=1 可升级到 Mesa 25.x"; \
    fi

# ---------------------------------------------------------------------------
# 7) Python 生态
# ---------------------------------------------------------------------------
RUN set -eux; \
    if [ "${PIP_INDEX}" != "https://pypi.org/simple" ]; then \
        mkdir -p /etc; \
        printf '[global]\nindex-url = %s\ntrusted-host = %s\ntimeout = 60\n' \
            "${PIP_INDEX}" \
            "$(printf '%s' "${PIP_INDEX}" | sed -E 's#^https?://([^/:]+).*#\1#')" \
            > /etc/pip.conf; \
    fi; \
    python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel; \
    python3 -m pip install --no-cache-dir \
        python-can==4.6.1 \
        canopen \
        pyserial \
        transforms3d \
        numpy scipy matplotlib \
        pyyaml \
        piper_sdk \
    ; \
    python3 -c "import can, piper_sdk; print('[ok] python-can', can.__version__, '| piper_sdk 已安装')"

# ---------------------------------------------------------------------------
# 8) rosdep（索引与 source list 全部指向镜像，避开 raw.githubusercontent.com）
# ---------------------------------------------------------------------------
RUN set -eux; \
    mkdir -p /etc/ros/rosdep/sources.list.d; \
    { \
        echo "# 由 docker/Dockerfile 生成 —— 全部指向清华 rosdistro 镜像"; \
        echo "yaml https://${ROSDISTRO_MIRROR}/rosdep/osx-homebrew.yaml osx"; \
        echo "yaml https://${ROSDISTRO_MIRROR}/rosdep/base.yaml"; \
        echo "yaml https://${ROSDISTRO_MIRROR}/rosdep/python.yaml"; \
        echo "yaml https://${ROSDISTRO_MIRROR}/rosdep/ruby.yaml"; \
        echo "gbpdistro https://${ROSDISTRO_MIRROR}/releases/fuerte.yaml fuerte"; \
    } > /etc/ros/rosdep/sources.list.d/20-default.list

# ---------------------------------------------------------------------------
# 9) 非 root 用户
#    ubuntu 基础镜像自带 uid=1000 的 ubuntu 用户，先删掉再建同名 uid 的 dev，
#    这样容器内写出的文件在宿主上归你自己所有（避免一堆 root:root 的 colcon 产物）。
# ---------------------------------------------------------------------------
RUN set -eux; \
    userdel -r ubuntu 2>/dev/null || true; \
    groupdel ubuntu 2>/dev/null || true; \
    groupadd -g "${USER_GID}" "${USER_NAME}"; \
    useradd -m -u "${USER_UID}" -g "${USER_GID}" -s /bin/bash "${USER_NAME}"; \
    usermod -aG sudo,video,render,plugdev,dialout "${USER_NAME}"; \
    printf '%s ALL=(ALL) NOPASSWD:ALL\n' "${USER_NAME}" > "/etc/sudoers.d/90-${USER_NAME}"; \
    chmod 0440 "/etc/sudoers.d/90-${USER_NAME}"

# ---------------------------------------------------------------------------
# 10) Bash 环境
# docker exec / compose exec 启动的是新进程，不会经过 entrypoint；把 ROS 与
# 工作区 overlay 写入 Bash 初始化文件，保证进入容器后能直接使用 ros2。
# ---------------------------------------------------------------------------
RUN set -eux; \
    printf '%s\n' \
        '#!/usr/bin/env bash' \
        'if [ -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]; then' \
        '  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"' \
        'fi' \
        'if [ -f /ws/install/setup.bash ]; then' \
        '  source /ws/install/setup.bash' \
        'fi' \
        'if [ -f /etc/piper_env.sh ]; then' \
        '  source /etc/piper_env.sh' \
        'fi' \
        > /usr/local/share/piper-bash-env.sh; \
    chmod 0644 /usr/local/share/piper-bash-env.sh; \
    printf '\n# PiPER ROS 环境（由 Dockerfile 管理）\nsource /usr/local/share/piper-bash-env.sh\n' \
        >> "/home/${USER_NAME}/.bashrc"; \
    chown "${USER_NAME}:${USER_GID}" "/home/${USER_NAME}/.bashrc"

# ---------------------------------------------------------------------------
# 11) entrypoint + 使用说明
# ---------------------------------------------------------------------------
COPY entrypoint.sh  /usr/local/bin/entrypoint.sh
COPY check-env.sh   /usr/local/bin/check-env
COPY README.md      /opt/PIPER_CONTAINER_README.md
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/check-env; \
    printf '%s\n' '#!/bin/sh' 'exec /usr/local/bin/check-env "$@"' > /usr/local/bin/ros2-check-env; \
    chmod +x /usr/local/bin/ros2-check-env

# rosdep 以普通用户身份初始化（缓存写到 ~/.ros）
RUN set -eux; \
    su - "${USER_NAME}" -c "rosdep update --rosdistro ${ROS_DISTRO}" \
    || echo "[warn] rosdep update 未成功，可在容器内手动执行：rosdep update"

# ---------------------------------------------------------------------------
# 运行期环境
# ---------------------------------------------------------------------------
ENV ROS_DISTRO=${ROS_DISTRO} \
    ROS_DOMAIN_ID=0 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    QT_X11_NO_MITSHM=1 \
    QT_QPA_PLATFORM=xcb \
    BASH_ENV=/usr/local/share/piper-bash-env.sh \
    RCUTILS_COLORIZED_OUTPUT=1

WORKDIR /ws
USER ${USER_NAME}

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
