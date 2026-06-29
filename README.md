# grasp_ws 5090 使用说明

这份说明对应 5090 机器上的原生迁移版工作区：

```bash
ssh -p 10022 lab809@114.214.211.251
cd /data/jiaxuanLin/grasp_ws
```

当前路线是 **Ubuntu 24.04 + ROS2 Jazzy + conda + CUDA 12.8**，不走 Docker。Python 依赖都在 conda 环境中，系统 Python 不需要再装这些视觉/深度学习包。

## 1. 迁移结果

路径：

```text
工作区:      /data/jiaxuanLin/grasp_ws
conda 环境: /data/jiaxuanLin/conda_envs/grasp_ws_py312
CUDA:       /usr/local/cuda-12.8
ROS2:       /opt/ros/jazzy
```

已迁移并验证：

```text
src/graspnet_py
src/grounded_sam_py
src/grasp_py
src/socket_node
src/grasp_srv_interface
graspnet-baseline
Grounded-SAM-2
data/images
```

已剔除：

```text
vggt_py ROS 包未迁移
Vggt.srv 已删除
抓取 pipeline 不再暴露 mode 参数
```

当前主流程固定为：

```text
Grounded-SAM 生成 workspace_mask.png -> GraspNet 输出抓取位姿
```

## 2. 每个终端先进入环境

每打开一个 SSH 终端，先执行：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
```

这个脚本会做这些事：

```bash
export GRASP_WS=/data/jiaxuanLin/grasp_ws
export CUDA_HOME=/usr/local/cuda-12.8
source /opt/ros/jazzy/setup.bash
conda activate /data/jiaxuanLin/conda_envs/grasp_ws_py312
source /data/jiaxuanLin/grasp_ws/install/setup.bash
```

检查当前 Python 必须是 conda：

```bash
which python
# /data/jiaxuanLin/conda_envs/grasp_ws_py312/bin/python
```

## 3. 重新编译工作区

正常使用不需要天天编译。改了 ROS2 包代码后执行：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
colcon build --symlink-install --packages-select \
  grasp_srv_interface graspnet_py grounded_sam_py grasp_py socket_node
source install/setup.bash
```

注意：远端 conda 环境里已经安装了 conda 版 `colcon`。不要直接用 `/usr/bin/colcon`，否则 Python 节点入口可能指到系统 Python。

确认入口使用 conda Python：

```bash
head -1 install/graspnet_py/lib/graspnet_py/graspnet_server
head -1 install/grounded_sam_py/lib/grounded_sam_py/grounded_sam_server
```

应该看到：

```text
#!/data/jiaxuanLin/conda_envs/grasp_ws_py312/bin/python
```

## 4. 启动视觉抓取 pipeline

先执行一次：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
```

然后一条命令启动整条链路：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py
```

常用可选参数：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py \
  visualize_sampled:=false \
  max_depth_raw:=2000 \
  socket_port:=9090 \
  image_save_path:=/data/jiaxuanLin/grasp_ws/data/images
```

`visualize_sampled:=false` 建议保留。远端 SSH 下 Open3D 可视化窗口容易阻塞服务。
`max_depth_raw:=2000` 会保留 2m 内的深度点；顶部相机距离目标超过 1m 时不要使用 800mm 之类的近距离阈值。

服务链路：

```text
socket client
  -> socket_node/socket_server, TCP 9090
  -> /trigger_grasp_pipeline
  -> /grounded_sam_service
  -> /graspnet_service
  -> 返回 grasp pose
```

失败时 socket client 会收到 `success=false`、`error_code` 和 `message`。错误码说明见：

```text
/data/jiaxuanLin/grasp_ws/docs/ERROR_CODES.md
```

## 5. 验证 ROS services

另开一个终端，仍然先：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
ros2 service list | grep -E 'grounded_sam|graspnet|trigger_grasp'
```

应该能看到：

```text
/grounded_sam_service
/graspnet_service
/trigger_grasp_pipeline
```

绕过 socket，直接调用 pipeline：

```bash
ros2 service call /trigger_grasp_pipeline grasp_srv_interface/srv/TriggerGrasp \
"{input_path: '/data/jiaxuanLin/grasp_ws/data/images', text_prompt: 'toy'}"
```

## 6. 输入数据要求

默认输入目录：

```text
/data/jiaxuanLin/grasp_ws/data/images
```

GraspNet 需要同一目录下有：

```text
color.png
depth.png
workspace_mask.png
camera.json
```

Socket server 收到请求后会写入：

```text
color.png
depth.png
```

Grounded-SAM 会生成：

```text
workspace_mask.png
```

`camera.json` 不会由 socket 自动生成。如果换相机或内参，记得更新这个文件。

深度图必须尽量保持 **16-bit PNG**。如果 depth 被保存成 8-bit，点云尺度会错，GraspNet 可能没有抓取结果。

## 7. Socket client 测试

在服务端同一台机器上测试：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
ros2 run socket_node socket_client \
  --server_host 127.0.0.1 \
  --server_port 9090 \
  --rgb_path /data/jiaxuanLin/grasp_ws/data/images/color.png \
  --depth_path /data/jiaxuanLin/grasp_ws/data/images/depth.png \
  --text_prompt toy
```

从另一台机器连 5090：

```bash
ros2 run socket_node socket_client \
  --server_host 114.214.211.251 \
  --server_port 9090 \
  --rgb_path /path/to/color.png \
  --depth_path /path/to/depth.png \
  --text_prompt toy
```

如果服务器在校园网/NAT 后面，`server_host` 用实际可达的内网或转发地址。

## 8. 已做过的验证

已在 5090 上验证：

```text
torch 2.7.1+cu128 可用
CUDA 可用，GPU 为 NVIDIA GeForce RTX 5090
GraspNet CUDA 扩展 pointnet2/knn 已编译
GroundingDINO CUDA 扩展已编译
Grounded-SAM 模型可加载
GraspNet checkpoint 可加载
ROS2 Jazzy workspace 可编译
四个服务节点可同时启动
Socket server 可监听 0.0.0.0:9090
```

没有在这一步执行真实机械臂抓取动作。

## 9. 常见问题

如果 Grounded-SAM 报 `get_head_mask` 相关错误，检查 `transformers` 版本：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
python -c "import transformers; print(transformers.__version__)"
```

当前已固定为：

```text
transformers==4.44.2
```

如果 GraspNet 运行时提示 `grasp_nms` 未安装，这是可接受的。当前服务会跳过 NMS，继续按 score 排序取最佳抓取。

如果 `colcon build` 找不到 `lark`，说明没有进入 conda 环境：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
python -c "import lark; print('lark ok')"
```

如果 socket 端口被占用：

```bash
ss -ltnp | grep 9090
```

如果需要清理残留测试节点：

```bash
pkill -INT -f '[g]rounded_sam_server|[g]raspnet_server|[g]rasp_client|[s]ocket_server'
```

如果 `apt update` 看到 librealsense 的 GPG key warning，可以先忽略；当前迁移所需依赖已经安装完成。

## 10. 依赖重装参考

只有环境坏掉时才需要参考这里。

GraspNet 关键点：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
export TORCH_CUDA_ARCH_LIST=12.0
export FORCE_CUDA=1
export MAX_JOBS=8

python -m pip install --no-deps -e graspnet-baseline/graspnetAPI
python -m pip install transforms3d trimesh

cd graspnet-baseline/pointnet2 && python setup.py install
cd ../knn && python setup.py install
```

Grounded-SAM 关键点：

```bash
source /data/jiaxuanLin/grasp_ws/env.sh
export TORCH_CUDA_ARCH_LIST=12.0
export FORCE_CUDA=1
export MAX_JOBS=8
export SAM2_BUILD_CUDA=1
export SAM2_BUILD_ALLOW_ERRORS=1

python -m pip install "transformers==4.44.2"
cd Grounded-SAM-2
python -m pip install --no-build-isolation --no-deps -e .
cd grounding_dino
python setup.py build_ext --inplace
cd ..
python -m pip install --no-build-isolation --no-deps -e grounding_dino
```
