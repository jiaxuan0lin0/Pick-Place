# Pick-Place

Pick-Place 是一个基于 ROS2 的视觉抓取系统，用于从 RGB-D 图像和文字提示中生成可执行的抓取位姿。系统将 Grounded-SAM 的开放词汇分割能力与 GraspNet 的抓取位姿估计能力串联起来，并通过 ROS2 service 与 TCP socket 暴露给外部机器人或上位机调用。

## 功能特性

- 基于文字提示定位目标物体，并生成 `workspace_mask.png`
- 使用 RGB-D 图像、相机内参和 workspace mask 估计抓取位姿
- 通过 ROS2 service 编排 Grounded-SAM、GraspNet 和 pipeline trigger
- 提供 TCP socket server/client，支持外部进程发送 RGB-D 图像并获取抓取结果
- 支持结构化错误码，便于机器人端做重试、换视角或人工介入
- 默认保留源码与示例数据，大模型权重和编译产物通过 `.gitignore` 排除

## 系统架构

```mermaid
flowchart LR
    A[Socket Client] -->|JSON + RGB + Depth| B[socket_node / socket_server]
    B -->|TriggerGrasp| C[grasp_py / grasp_client]
    C -->|GroundedSam| D[grounded_sam_py / grounded_sam_server]
    D -->|workspace_mask.png| C
    C -->|Graspnet| E[graspnet_py / graspnet_server]
    E -->|geometry_msgs/Pose[]| C
    C -->|success, error_code, poses| B
    B -->|JSON response| A

    F[data/images/color.png] --> D
    G[data/images/depth.png] --> E
    H[data/images/camera.json] --> E
    I[data/images/workspace_mask.png] --> E
```

核心流程：

```text
text_prompt + color.png
  -> Grounded-SAM
  -> workspace_mask.png
  -> GraspNet(color.png, depth.png, workspace_mask.png, camera.json)
  -> grasp pose
```

## 目录结构

```text
.
├── src/
│   ├── grasp_py/              # 抓取 pipeline 编排节点和 launch 文件
│   ├── grounded_sam_py/       # Grounded-SAM ROS2 service 节点
│   ├── graspnet_py/           # GraspNet ROS2 service 节点
│   ├── socket_node/           # TCP socket server/client
│   └── grasp_srv_interface/   # ROS2 service definitions
├── Grounded-SAM-2/            # Grounded-SAM / SAM2 相关代码
├── graspnet-baseline/         # GraspNet baseline 相关代码
├── data/images/               # 默认 RGB-D 输入与中间结果目录
├── docs/                      # 启动说明、错误码等补充文档
├── env.sh                     # 本地环境入口脚本
├── setup_project.sh           # 项目配置脚本
└── README.md
```

## 环境要求

推荐环境：

- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12 conda environment
- CUDA 12.8
- NVIDIA GPU

依赖说明：

- `Grounded-SAM-2` 和 `graspnet-baseline` 依赖深度学习模型与 CUDA 扩展。
- 本仓库不提交大模型权重、checkpoint、编译目录或日志目录。
- 如果你的 ROS2、CUDA 或 conda 路径与当前环境不同，请先按本机环境调整 `env.sh`。

## 环境初始化

进入仓库根目录后执行：

```bash
source env.sh
```

如果不使用 `env.sh`，至少需要确保当前终端已经完成：

```bash
export GRASP_WS="$PWD"
source install/setup.bash
```

检查 Python 环境：

```bash
which python
python --version
```

## 编译

修改 ROS2 包后重新编译：

```bash
colcon build --symlink-install --packages-select \
  grasp_srv_interface graspnet_py grounded_sam_py grasp_py socket_node
source install/setup.bash
```

## 启动 Pipeline

启动完整抓取链路：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py grasp_ws:="$PWD"
```

常用参数：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py \
  grasp_ws:="$PWD" \
  visualize_sampled:=false \
  max_depth_raw:=2000 \
  socket_port:=9090 \
  image_save_path:=data/images
```

参数说明：

| 参数 | 默认用途 |
| --- | --- |
| `grasp_ws` | 工作区根目录，建议传入当前仓库路径 |
| `image_save_path` | socket server 保存 `color.png` 和 `depth.png` 的目录 |
| `socket_host` | socket server 监听地址 |
| `socket_port` | socket server 监听端口 |
| `visualize_sampled` | 是否打开 GraspNet/Open3D 可视化窗口 |
| `max_depth_raw` | GraspNet 使用的最大原始深度值，设为 `0` 可关闭上限 |

远程 SSH 环境下建议保留：

```bash
visualize_sampled:=false
```

## 验证 ROS2 Service

启动后，另开终端并进入同一环境：

```bash
source env.sh
ros2 service list | grep -E 'grounded_sam|graspnet|trigger_grasp'
```

应能看到：

```text
/grounded_sam_service
/graspnet_service
/trigger_grasp_pipeline
```

绕过 socket，直接调用 pipeline：

```bash
ros2 service call /trigger_grasp_pipeline grasp_srv_interface/srv/TriggerGrasp \
"{input_path: 'data/images', text_prompt: 'toy'}"
```

## Socket Client 调用

在服务端本机测试：

```bash
ros2 run socket_node socket_client \
  --server_host 127.0.0.1 \
  --server_port 9090 \
  --rgb_path data/images/color.png \
  --depth_path data/images/depth.png \
  --text_prompt toy
```

从其他机器调用时，将 `--server_host` 改为运行 pipeline 的机器地址，并确认网络能访问 `socket_port`。

Socket 请求分三段发送：

1. JSON bytes，内容包含 `text_prompt`
2. RGB 图像 PNG bytes
3. Depth 图像 PNG bytes

每段消息前都有 8 字节 big-endian unsigned length header。

## 输入数据

默认输入目录：

```text
data/images
```

GraspNet 需要同一目录下存在：

```text
color.png
depth.png
workspace_mask.png
camera.json
```

说明：

- `color.png`：RGB 图像
- `depth.png`：深度图，建议保持 16-bit PNG
- `workspace_mask.png`：Grounded-SAM 生成的目标区域 mask
- `camera.json`：相机内参，需要与当前 RGB-D 数据匹配

Socket server 收到请求后会写入：

```text
data/images/color.png
data/images/depth.png
```

`workspace_mask.png` 通常由 Grounded-SAM 自动生成。`camera.json` 不会由 socket server 自动生成，换相机或内参后需要手动更新。

## 输出格式

Socket 成功响应示例：

```json
{
  "success": true,
  "error_code": "OK",
  "message": "成功检测到 1 个最佳抓取姿态.",
  "grasp_poses": {
    "position": [0.0, 0.0, 0.0],
    "orientation": [0.0, 0.0, 0.0, 1.0]
  }
}
```

失败响应示例：

```json
{
  "success": false,
  "error_code": "NO_DETECTION",
  "message": "GroundedSAM failed to detect the target object."
}
```

机器人端建议先判断 `success`。当 `success=false` 时，再根据 `error_code` 决定重试、换视角、修改 prompt 或上报人工处理。

更多错误码说明见：

```text
docs/ERROR_CODES.md
```

## Service 接口

### TriggerGrasp

Service:

```text
/trigger_grasp_pipeline
```

Type:

```text
grasp_srv_interface/srv/TriggerGrasp
```

Request:

| 字段 | 说明 |
| --- | --- |
| `input_path` | 包含 RGB-D 输入和相机内参的目录 |
| `text_prompt` | Grounded-SAM 使用的目标文字提示 |

Response:

| 字段 | 说明 |
| --- | --- |
| `success` | pipeline 是否成功 |
| `error_code` | 结构化错误码，成功时为 `OK` |
| `message` | 面向调试的人类可读信息 |
| `grasp_poses` | GraspNet 输出的抓取位姿数组 |

## 模型权重和大文件

本仓库不应提交以下内容：

- `build/`
- `install/`
- `log/`
- `*.pth`
- `*.pt`
- `*.ckpt`
- `*.onnx`
- `*.tar`
- `*.zip`
- 本地模型缓存和数据集缓存

常见模型文件放置位置可参考：

```text
Grounded-SAM-2/checkpoints
Grounded-SAM-2/gdino_checkpoints
graspnet-baseline/logs
```

具体模型来源、版本和校验方式请以对应上游项目说明为准。TODO: 补充本项目实际使用的模型版本、下载方式和 checksum。

## Troubleshooting

### 看不到 ROS2 service

确认每个终端都已经进入同一 ROS2 环境：

```bash
source env.sh
ros2 service list
```

如果仍然看不到 service，重新编译并 source：

```bash
colcon build --symlink-install --packages-select \
  grasp_srv_interface graspnet_py grounded_sam_py grasp_py socket_node
source install/setup.bash
```

### Grounded-SAM 没有检测到目标

常见原因：

- `text_prompt` 与画面目标不匹配
- 目标遮挡严重或不在画面中
- RGB 图像质量不足

建议处理：

- 换更明确的 prompt
- 换视角或靠近目标
- 保存当前 `color.png` 和 `workspace_mask.png` 做复现分析

### GraspNet 没有输出抓取姿态

常见原因：

- `depth.png` 不是 16-bit PNG
- `camera.json` 与当前相机不匹配
- `workspace_mask.png` 区域内有效深度点过少
- 碰撞检测后没有可用抓取

建议处理：

- 检查深度图格式和尺度
- 更新相机内参
- 换视角重新采集
- 查看 GraspNet 节点日志

### Open3D 可视化窗口阻塞

远程或无显示环境下启动时关闭可视化：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py \
  grasp_ws:="$PWD" \
  visualize_sampled:=false
```

### Socket 端口冲突

启动时改端口：

```bash
ros2 launch grasp_py grasp_pipeline.launch.py \
  grasp_ws:="$PWD" \
  socket_port:=9091
```

客户端对应修改：

```bash
ros2 run socket_node socket_client \
  --server_host 127.0.0.1 \
  --server_port 9091 \
  --rgb_path data/images/color.png \
  --depth_path data/images/depth.png \
  --text_prompt toy
```

### 权重或 checkpoint 缺失

确认所需权重已经按对应上游项目要求放置在仓库内的相对目录中，并且没有被误提交到 Git。若模型版本不确定，请先补充权重清单和下载说明。

## 开发建议

- 修改 ROS2 service definition 后，重新编译 `grasp_srv_interface` 以及依赖它的包
- 修改 Python 节点后，使用 `--symlink-install` 可减少重复安装成本
- 真实机器人调用时，优先处理 `success` 和 `error_code`
- 提交前检查是否误加入大文件：

```bash
git status --short
git ls-files -z | xargs -0 -r du -h | sort -h | tail
```

## License

TODO: 补充本仓库代码、`Grounded-SAM-2`、`graspnet-baseline` 及其依赖的许可证说明。
