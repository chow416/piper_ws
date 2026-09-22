# PiPER 开发容器

Ubuntu 22.04 + ROS 2 Humble + MoveIt2 + Gazebo Classic，整套环境打包成一个镜像。

**目标是"环境即镜像"**：把项目目录连同这个 `docker/` 一起发给同事，对方只要装了 Docker，
`docker compose up` 就能跑起来 —— 不需要关心自己机器上是 Ubuntu 22.04 还是 24.04、
ROS 是 Humble 还是 Jazzy、Python 是 3.10 还是 3.12。

---

## 1 为什么是 22.04 + Humble

| 原因 | 说明 |
|---|---|
| **与同事、机器人主控一致** | 同事的开发机与机器人主控都是 Ubuntu 22.04。容器在这个版本上开发，环境与目标完全一致，交付不产生任何漂移 |
| **与松灵官方对齐** | `Piper_ros` 的 `humble` 分支是官方维护的分支，URDF / MoveIt 配置 / 消息定义零改动即可编译 |
| **Gazebo Classic 还在** | 22.04 仓库里仍有 `gazebo 11.10.2`，原版 `piper_gazebo` 仿真链路可用（24.04 已经没有这个包了） |
| **Python 3.10** | `piper_sdk` / `python-can` 直接 `pip install`，不用折腾 |

而宿主继续用 Ubuntu 24.04 —— 因为本机是 Intel Arrow Lake-P 核显（PCI `8086:7d51`），
22.04 出厂的 Mesa 22.0.1 驱动不了它，装 22.04 会直接开机黑屏。
**所以容器在这里不是"额外套一层"，而是让本机回到 22.04 这条主线的唯一途径。**
容器不负责驱动显示器，所以这个问题不会带进容器里（详见第 5 节）。

---

## 2 首次使用（宿主侧）

### 2.1 装 Docker

```bash
# 方式一：用 Ubuntu 仓库里的 docker.io（简单，够用）
sudo apt-get update && sudo apt-get install -y docker.io docker-compose-v2
sudo usermod -aG docker "$USER"      # 之后重新登录，才能免 sudo 用 docker

# 方式二：Docker 官方脚本（版本更新，支持更好）
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker "$USER"
```

如果团队另有Docker安装脚本，也可以在宿主机执行该脚本完成相同步骤。

### 2.2 准备 .env

```bash
cd ~/piper_ws/docker
cp .env.example .env
# 确认 UID/GID 与自己一致
id -u; id -g
```

### 2.3 构建镜像

```bash
cd ~/piper_ws/docker

# 普通机器（容器内软件渲染）
docker compose build

# 想启用容器内硬件加速（新款 Intel 核显，比如本机）
docker compose -f docker-compose.yml -f docker-compose.gpu.yml build
```

首次构建约 15～25 分钟，下载 3～4 GB。中途全部走清华镜像，不需要科学上网。

### 2.4 放行 X11（让容器里的 RViz 能显示）

Wayland 会话下也走 XWayland，一样适用：

```bash
xhost +local:
```

每次重启宿主后需要重新执行一次。想更严格的话，用 `xhost +si:localuser:$USER` 代替。

---

## 3 日常使用

```bash
cd ~/piper_ws/docker

# 起容器（进交互式 shell）
docker compose up -d
docker compose exec piper bash

# 装 GPU 那套的起法
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d

# 停 / 删
docker compose stop
docker compose down
```

### 容器内：编译与自检

```bash
check-env                                  # 环境自检，先跑这个
cd /ws
rm -rf build install log                   # 首次进容器建议清一次：这些是旧 Humble 产物，绝对路径对不上
colcon build --symlink-install
source install/setup.bash
```

### 容器内：跑 PiPER

```bash
# 只看模型 + MoveIt 规划演示
ros2 launch piper_description display.launch.py
ros2 launch piper_with_gripper_moveit demo.launch.py

# Gazebo Classic 仿真
ros2 launch piper_gazebo piper_gazebo.launch.py

# 接实体机械臂（需要先把 CAN 透传进来，见下节）
ros2 launch piper start_single_piper.launch.py can_port:=can0
```

---

## 4 接实体机械臂：CAN 透传

容器默认看不到 CAN 设备，按你的适配器类型二选一。

### 4.1 USB-CAN 适配器（CANable / 广成 USBCAN 等，走串口）

```bash
ls /dev/ttyACM* /dev/ttyUSB*          # 宿主上先确认设备名
```

在 `docker-compose.yml` 里放开对应注释，或临时用命令行：

```bash
docker compose run --rm --device=/dev/ttyACM0 piper bash
```

### 4.2 SocketCAN（`can0`，需要宿主先把内核模块与网卡配好）

宿主侧先建好接口：

```bash
sudo modprobe gs_usb                              # USB-CAN 用 gs_usb；PCIe 卡换成对应驱动
sudo ip link set can0 up type can bitrate 1000000 # PiPER 用 1 Mbps
ip -br link show can0
```

然后让容器共享宿主网络后就能直接 `ip link show can0` 看到它。
如果容器内看不到，改用特权模式（在 compose 里放开 `privileged: true`，或用 `docker compose run --rm --privileged piper bash`）。

> `privileged: true` 会放开全部设备权限，仅调试用；正式跑建议改成
> `cap_add: [NET_ADMIN]` + `network_mode: host`。

### 4.3 验证

```bash
# 容器内
check-env                                  # CAN 那一栏应该是绿的
ros2 run piper piper_single_ctrl           # 或 piper_sdk 自带的示例
```

---

## 5 图形与 GPU

容器共享宿主内核，但**用户态是独立的** —— 容器里用哪个 Mesa 由镜像决定，跟宿主无关。

| | 宿主（24.04） | 容器（22.04 镜像） |
|---|---|---|
| Mesa | 25.2.8，认识 `8086:7d51` | **22.0.1，不认识** |
| 后果 | 会话初始化失败 → 开机黑屏 | 退回 `llvmpipe` 软件渲染 → **功能正常，帧率低** |

所以方案 A（装 22.04）被否决的理由不能直接套到容器上：
宿主黑屏是致命的，容器里只是"慢"。**规划、求解、CAN 通信这些都不受影响，
只有 RViz / Gazebo 的画面帧率会低一些。**

三种处理方式：

| 做法 | 效果 | 什么时候用 |
|---|---|---|
| **什么都不做** | 软件渲染，RViz 流畅可用，Gazebo 偏慢 | 默认，最省事，适合算法/规划开发 |
| **加 GPU overlay** | 容器内 Mesa 升到 25.x + 透传 `/dev/dri`，恢复硬件加速 | 要看 Gazebo 仿真、或嫌 RViz 卡 |
| 挂宿主 `dri/` 目录 | ❌ 不行 | 宿主 Mesa 25.2.8 是针对 glibc 2.39 编的，jammy 只有 2.35，会报 `GLIBC_2.38 not found` |

用 overlay 构建：

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d --build
docker compose exec piper check-env          # 看 "OpenGL 渲染器" 那行
```

`check-env` 里显示 `llvmpipe` 就是软件渲染，显示 `Mesa Intel(R) Graphics (...)` 就是硬件加速生效了。

---

## 6 多机联调 / 与宿主互通

基础 compose 用了 `network_mode: host`，所以：

- 容器里的 ROS 2 节点 ↔ 宿主上的 ROS 2 节点，直接互相可见
- 同一局域网内的另一台电脑（同样 `ROS_DOMAIN_ID`）也能看到

如果网络里同时跑好几组实验互相串数据，改 `ROS_DOMAIN_ID` 隔开：

```bash
ROS_DOMAIN_ID=42 docker compose up -d
```

组播不稳（VPN、公司网络）时换 DDS：

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp docker compose up -d
```

---

## 7 交给同事

两种方式，按对方网络条件选。

**方式一：给源码目录（推荐，体积小）**

把 `~/piper_ws` 整个打包发过去（含 `src/`、`docker/`、`examples/`），对方：

```bash
tar xzf piper_ws.tar.gz -C ~
cd ~/piper_ws/docker
cp .env.example .env
docker compose build            # 首次约 15~25 分钟
xhost +local:
docker compose up -d
docker compose exec piper bash -lc 'cd /ws && colcon build --symlink-install && check-env'
```

**方式二：给镜像 tar（对方网络差时）**

```bash
# 你的机器上导出
docker save piper-dev:humble -o piper-dev-humble.tar
# 压缩后大约 1.5~2 GB
zstd -19 piper-dev-humble.tar -o piper-dev-humble.tar.zst

# 对方机器上导入
docker load -i piper-dev-humble.tar
cd ~/piper_ws/docker && docker compose up -d     # 已有镜像，不会再 build
```

> 注意：镜像里的 UID 是构建时定的（默认 1000）。对方 UID 不是 1000 的话，
> 要么改 `.env` 里的 `USER_UID` 重建，要么接受 `/ws` 里产出文件属主不一致
> （后果是宿主上编辑需要 sudo）。

---

## 8 文件说明

| 文件 | 作用 |
|---|---|
| `Dockerfile` | 镜像定义：换源、装 ROS 2 Humble Desktop + MoveIt2 + Gazebo Classic + Python 库 |
| `docker-compose.yml` | 基础运行配置：X11 转发、工作区挂载、网络模式 |
| `docker-compose.gpu.yml` | GPU overlay：透传 `/dev/dri` + 容器内 Mesa 升到 25.x |
| `entrypoint.sh` | 容器入口：source ROS 与工作区，打印自检横幅 |
| `check-env.sh` | `check-env` 命令的实现：逐项报告系统/组件/图形/CAN/工作区状态 |
| `piper_env.sh` | 附加环境变量，挂进容器的 `/etc/piper_env.sh`，改完 `docker compose restart` 生效 |
| `.env.example` | 构建参数模板 |
| `.dockerignore` | 构建上下文过滤 |

---

## 9 常见问题

**`docker: permission denied while trying to connect to the Docker daemon socket`**
没重新登录。`sudo usermod -aG docker $USER` 之后必须**重新登出登入**（或 `newgrp docker`）。

**构建时 apt 卡住或超时**
`.env` 里源配错了。确认 `UBUNTU_MIRROR` / `ROS2_MIRROR` 没带 `https://` 前缀（脚本会自己拼）。

**容器里 `ros2 launch ... demo.launch.py` 报 `cannot open display`**
宿主上执行 `xhost +local:`，并确认 `echo $DISPLAY` 是 `:0` 或 `:1`，与 `.env` 一致。

**RViz 能开但特别卡**
正常，软件渲染。要流畅就上 GPU overlay（第 5 节）。

**编译报 `Target dependency '/ws/build/...idl' does not exist`**
工作区路径里有非 ASCII 字符。保持 `/ws` 或纯英文路径 —— 这是踩过的坑。

**`ros2` 命令报 locale / 参数解析错误**
镜像里已经钉死 `LC_NUMERIC=en_US.UTF-8`。如果自己改过，改回去。

**改了代码但容器里跑的还是旧的**
容器内没重新编译。`cd /ws && colcon build --symlink-install`。
用了 `--symlink-install` 时，只改 Python 源码不用重编，但要 `source install/setup.bash`。
