# Grasp Pipeline Error Codes

本文档说明 `/data/jiaxuanLin/grasp_ws` 中抓取视觉 pipeline 的结构化错误码。

socket client 最终会收到 JSON 响应。成功时：

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

失败时：

```json
{
  "success": false,
  "error_code": "NO_DETECTION",
  "message": "GroundedSAM失败: 未检测到目标框，prompt='cup.'"
}
```

机器人端建议优先判断 `success`，失败时再根据 `error_code` 决定重试、换视角、换 prompt 或上报人工处理。

## GroundedSAM Codes

这些错误码来自 `grounded_sam_service`，会经由 `trigger_grasp_pipeline` 和 `socket_server` 原样返回给 socket client。

| error_code | 含义 | 常见原因 | 建议处理 |
| --- | --- | --- | --- |
| `EMPTY_PROMPT` | `text_prompt` 为空 | socket 请求没有传目标词，或目标词被清洗后为空 | 机器人端补充目标词后重试 |
| `IMAGE_NOT_FOUND` | `color.png` 不存在 | socket server 没成功保存 RGB 图，或输入目录错误 | 重新发送 RGB/Depth 图像，检查 `image_save_path` |
| `IMAGE_LOAD_FAILED` | 图像读取失败 | `color.png` 损坏、格式异常或权限问题 | 重新采集图像，确认图片能被 OpenCV/PIL 读取 |
| `NO_DETECTION` | GroundingDINO 没检测到目标框 | 目标不在画面内、prompt 不匹配、阈值过高、遮挡严重 | 换视角、换 prompt、靠近目标或降低阈值 |
| `SAM_NO_MASK` | SAM2 没生成有效 mask | 检测框存在但分割失败，或 mask 为空 | 换视角、改善光照/遮挡，必要时重新检测 |
| `SAM_FAILED` | SAM2 推理异常 | SAM2 输入框异常、模型运行报错、显存/环境问题 | 查看服务日志，必要时重启 pipeline |
| `INTERNAL_ERROR` | 其他未知异常 | 未覆盖的运行时错误，例如保存 mask 失败 | 查看服务日志，保留输入图用于复现 |

## GraspNet Codes

这些错误码来自 `graspnet_service`。只有 GroundedSAM 成功生成 `workspace_mask.png` 后才会进入 GraspNet。

| error_code | 含义 | 常见原因 | 建议处理 |
| --- | --- | --- | --- |
| `GRASPNET_NO_GRASP` | GraspNet 没输出抓取结果 | 深度图无效、点云质量差、mask 区域有效点太少 | 检查 16 位深度图和相机内参，重新采集 |
| `GRASPNET_COLLISION_EMPTY` | 碰撞检测后没有可用抓取 | `collision_thresh` 太严格，或点云/物体姿态导致全部碰撞 | 放宽碰撞阈值、换视角或重新采集 |
| `GRASPNET_FAILED` | GraspNet 推理异常 | 深度图格式、camera.json、模型推理或点云处理异常 | 查看日志中的 `推理失败` 详情 |
| `GRASPNET_SERVICE_CALL_FAILED` | grasp client 调用 GraspNet 服务失败 | GraspNet 节点未启动或服务通信异常 | 检查 `/graspnet_service` 是否存在 |

## Pipeline And Socket Codes

这些错误码来自 `grasp_client` 或 `socket_server`，用于表示服务链路或 socket 输入阶段的问题。

| error_code | 含义 | 常见原因 | 建议处理 |
| --- | --- | --- | --- |
| `OK` | 流程成功 | 已返回抓取位姿 | 机器人端执行后续抓取动作 |
| `GSAM_SERVICE_CALL_FAILED` | grasp client 调用 GroundedSAM 服务失败 | GroundedSAM 节点未启动或服务通信异常 | 检查 `/grounded_sam_service` 是否存在 |
| `TRIGGER_SERVICE_CALL_FAILED` | socket server 调用 TriggerGrasp 服务失败 | `grasp_client` 节点未启动或服务通信异常 | 检查 `/trigger_grasp_pipeline` 是否存在 |
| `SOCKET_SAVE_PATH_FAILED` | socket server 创建/写入保存目录失败 | `image_save_path` 权限或磁盘问题 | 检查目录权限和磁盘空间 |
| `SOCKET_RECV_FAILED` | socket server 没收到完整数据 | socket client 断开、消息长度不完整、网络中断 | 重新发送请求 |
| `INVALID_JSON` | 请求 JSON 解析失败 | socket client 发送的第一段不是合法 JSON | 检查 socket client 请求格式 |
| `RGB_IMAGE_INVALID` | RGB 图像解码或保存失败 | RGB 数据不是有效图片 | 重新采集并发送 RGB 图 |
| `DEPTH_IMAGE_INVALID` | Depth 图像解码或保存失败 | Depth 数据不是有效图片，或深度格式异常 | 重新采集并发送 16 位深度图 |

## Return Path

失败信息的回传路径：

```text
grounded_sam_server / graspnet_server
  -> grasp_client TriggerGrasp response
  -> socket_server JSON response
  -> robot socket client
```

核心约定：

| 字段 | 说明 |
| --- | --- |
| `success` | 是否成功。机器人端先判断这个字段。 |
| `error_code` | 结构化错误码。成功为 `OK`。 |
| `message` | 给人看的说明，可能包含 prompt、路径或异常细节。 |
| `grasp_poses` | 仅成功时返回，包含最佳抓取姿态。 |

