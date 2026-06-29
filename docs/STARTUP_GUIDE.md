# ROS2 抓取系统启动说明

这份说明按当前工作区 `/home/ustc/ros2_ws` 的实际代码整理。容器内路径是 `/root/host_home/ros2_ws`，它映射到主机的 `/home/ustc/ros2_ws`。

## 1. 当前系统结构

主要链路是：

```text
socket_client
  -> socket_node/socket_server, TCP 9090
  -> grasp_py/grasp_client, ROS2 service: /trigger_grasp_pipeline
  -> grounded_sam_py/grounded_sam_server, ROS2 service: /grounded_sam_service
  -> graspnet_py/graspnet_server, ROS2 service: /graspnet_service
  -> 返回 grasp pose
```

当前固定主流程是：Grounded-SAM 根据文字提示生成 `workspace_mask.png`，GraspNet 用 `color.png/depth.png/workspace_mask.png/camera.json` 输出抓取位姿。

## 2. Docker 环境

根目录有 `Dockerfile`、`docker-compose.yml` 和 `Makefile`。当前 compose 服务名是 `rm_robot_dev`，容器名是 `rm_robot`，镜像名是 `rm_robot`。

在主机终端执行：

```bash
cd /home/ustc/ros2_ws
make run
make shell
```

如果修改过 `Dockerfile`，先重建：

```bash
cd /home/ustc/ros2_ws
make build
make run
make shell
```

如果第一次配置依赖，进容器后执行：

```bash
cd /root/host_home/ros2_ws
./setup_project.sh
```

我检查到镜像里已有这些虚拟环境：

```text
/opt/venv_grasp
/opt/venv_graspnet
/opt/venv_groundedsam
/opt/venv_vggt
```

注意：当前终端尝试启动 compose 容器时遇到过 `could not select device driver "nvidia" with capabilities: [[gpu]]`。如果你也遇到这个错误，说明 Docker 还不能拿到 NVIDIA runtime，需要先修好 NVIDIA Container Toolkit 或 Docker GPU 配置。

## 3. 每个 ROS2 终端都先 source

下面所有 ROS2 命令都建议在容器里跑。每开一个新终端，先执行：

```bash
cd /root/host_home/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 4. 启动视觉抓取 pipeline

先进入环境：

```bash
cd /root/host_home/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

然后一条命令启动整条链路：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py
```

常用可选参数：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py \
  visualize_sampled:=false \
  socket_port:=9090 \
  image_save_path:=/root/host_home/ros2_ws/data/images
```

`visualize_sampled:=false` 建议保留。远端 SSH 下 Open3D 可视化窗口容易阻塞服务。

Socket server 会把收到的 RGB 和 Depth 保存为：

```text
/root/host_home/ros2_ws/data/images/color.png
/root/host_home/ros2_ws/data/images/depth.png
```

GraspNet 还需要同一目录下有：

```text
/root/host_home/ros2_ws/data/images/camera.json
```

当前工作区里这个文件已经存在。Depth 图建议保持 16-bit PNG，否则 GraspNet 的点云尺度容易出错。

## 5. 验证 ROS2 service

服务启动后，开新容器终端：

```bash
cd /root/host_home/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service list | grep -E 'grounded_sam|graspnet|trigger_grasp'
```

应该能看到：

```text
/grounded_sam_service
/graspnet_service
/trigger_grasp_pipeline
```

也可以绕过 socket，直接调 pipeline：

```bash
ros2 service call /trigger_grasp_pipeline grasp_srv_interface/srv/TriggerGrasp \
"{input_path: '/root/host_home/ros2_ws/data/images', text_prompt: 'toy'}"
```

## 6. Socket client 用法

我修了一下 `src/socket_node/socket_node/socket_client.py`，现在它可以作为轻量测试客户端用。

当前我已经重新编译了 `socket_node`，可以直接这样跑：

```bash
cd /root/host_home/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run socket_node socket_client \
  --server_host 127.0.0.1 \
  --server_port 9090 \
  --rgb_path /root/host_home/ros2_ws/data/images/color.png \
  --depth_path /root/host_home/ros2_ws/data/images/depth.png \
  --text_prompt toy
```

不想重新编译时，也可以直接运行脚本：

```bash
cd /root/host_home/ros2_ws
python3 src/socket_node/socket_node/socket_client.py \
  --server_host 127.0.0.1 \
  --server_port 9090 \
  --rgb_path data/images/color.png \
  --depth_path data/images/depth.png \
  --text_prompt toy
```

如果从主机或另一台机器连容器，因为 compose 使用 `network_mode: host`，`server_host` 填运行服务那台机器的 IP，端口默认 `9090`。

## 7. 修改代码后的重编译建议

这些 ROS2 Python 可执行文件会绑定到当前激活的 conda Python：

```text
graspnet_server
grounded_sam_server
grasp_client
socket_server
```

如果改了 Python 包后入口没更新，重新构建即可：

```bash
cd /root/host_home/ros2_ws
source /opt/ros/humble/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate /data/jiaxuanLin/conda_envs/grasp_ws_py312
colcon build --symlink-install --packages-select grasp_srv_interface grasp_py socket_node graspnet_py grounded_sam_py
source install/setup.bash
```

## 8. 常见问题

`could not select device driver "nvidia"`：

Docker GPU runtime 没配置好。GraspNet 和 Grounded-SAM 都建议用 GPU，先确认 `nvidia-smi` 和 Docker 的 GPU 支持。

`GraspNet 找不到 camera.json`：

确认 socket server 的 `image_save_path` 目录下有 `camera.json`。默认目录是 `/root/host_home/ros2_ws/data/images`。

抓取结果为空：

优先检查 depth 是否是 16-bit PNG、`camera.json` 内参是否匹配、`text_prompt` 是否能分割出目标、`workspace_mask.png` 是否正常。

Open3D 窗口卡住：

启动 launch 时加 `visualize_sampled:=false`。

端口冲突：

启动 launch 时改端口：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py socket_port:=9091
```

客户端也对应改 `--server_port 9091`。
