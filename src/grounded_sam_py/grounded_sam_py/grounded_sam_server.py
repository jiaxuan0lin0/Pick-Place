import rclpy
from rclpy.node import Node
import torch
from grasp_srv_interface.srv import GroundedSam
import sys
import os
import cv2
import numpy as np
import supervision as sv
from contextlib import nullcontext
from pathlib import Path
from torchvision.ops import box_convert
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate
from omegaconf import OmegaConf
import logging


# 设置groundedsam地址
WORKSPACE_DIR = os.environ.get('GRASP_WS', '/root/host_home/ros2_ws')
GSamDIR = os.path.join(WORKSPACE_DIR, 'Grounded-SAM-2')
sys.path.append(os.path.join(GSamDIR))

from sam2.sam2_image_predictor import SAM2ImagePredictor
from grounding_dino.groundingdino.util.inference import load_model, load_image, predict

#注册参数值
SAM2_CHECKPOINT = os.path.join(GSamDIR, "checkpoints/sam2.1_hiera_large.pt")
SAM2_MODEL_CONFIG = "sam2.1/sam2.1_hiera_l"
SAM2_CONFIG_DIR = os.path.join(GSamDIR, "sam2/configs")
GROUNDING_DINO_CONFIG = os.path.join(GSamDIR, "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py")
GROUNDING_DINO_CHECKPOINT = os.path.join(GSamDIR, "gdino_checkpoints/groundingdino_swint_ogc.pth")
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# create output directory
OUTPUT_DIR = Path(WORKSPACE_DIR) / "docs/data_gsamed"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

class GroundedSamServer(Node):
    def __init__(self):
        super().__init__('grounded_sam_server')
        self.get_logger().info('Grounded_Sam 服务节点启动中')
        #创建srv服务
        self.srv = self.create_service(
            GroundedSam,
            'grounded_sam_service',
            self.handle_gsam_request
        )
        self.get_logger().info('Grounded_Sam 服务已启动，等待请求...')
        #加载计算设备
        self.get_logger().info(f'计算设备: {DEVICE}')
        #groundedsam模型加载
        self.get_logger().info('Grounded_Sam 模型加载中...')
        self.grounded_sam_model_load()
        self.get_logger().info('Grounded_Sam 模型加载完成.')

    #groundedsam模型加载函数
    def grounded_sam_model_load(self):
        # build SAM2 image predictor
        sam2_checkpoint = SAM2_CHECKPOINT
        if torch.cuda.is_available() and torch.cuda.get_device_properties(0).major >= 8:
        # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
        self.get_logger().info(f"正在从{SAM2_MODEL_CONFIG}加载sam2配置文件")
        sam2_model = build_sam2(SAM2_MODEL_CONFIG, sam2_checkpoint, device=DEVICE, config_dir=SAM2_CONFIG_DIR)
        self.sam2_predictor = SAM2ImagePredictor(sam2_model)
        # build grounding dino model
        self.grounding_model = load_model(
            model_config_path=GROUNDING_DINO_CONFIG, 
            model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
            device=DEVICE
        )

    def get_all_images_from_dir(self, dir_path):
        extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.tiff']
        dir_path = Path(dir_path)
        image_paths = [str(p) for p in dir_path.iterdir() if p.is_file() and p.suffix.lower() in extensions]
        image_paths.sort()
        return image_paths

    def process_single_image(self,image_path,text_prompt):
        """
        对单张图片进行分割，返回 GraspNet 使用的黑白掩码。
        Args:
            image_path:string 输入图片路径
            text_prompt:string 分割提示词
        """
        self.get_logger().info(f"正在分割图片{image_path}")
        text_prompt = text_prompt.strip()
        if not text_prompt:
            message = "GroundedSAM失败: text_prompt 为空，无法执行分割。"
            self.get_logger().error(message)
            return None, "EMPTY_PROMPT", message
        text_prompt = text_prompt + '.'

        try:
            image_source, image = load_image(image_path)
            self.sam2_predictor.set_image(image_source)
        except Exception as e:
            message = f"GroundedSAM失败: 加载输入图像失败: {e}"
            self.get_logger().error(message)
            return None, "IMAGE_LOAD_FAILED", message

        try:
            boxes, confidences, labels = predict(
                model=self.grounding_model,
                image=image,
                caption=text_prompt,
                box_threshold=BOX_THRESHOLD,
                text_threshold=TEXT_THRESHOLD,
                device=DEVICE
            )
        except Exception as e:
            message = f"GroundedSAM失败: GroundingDINO 推理失败: {e}"
            self.get_logger().error(message)
            return None, "INTERNAL_ERROR", message

        if boxes is None or boxes.numel() == 0:
            message = f"GroundedSAM失败: 未检测到目标框，prompt='{text_prompt}'"
            self.get_logger().warn(message)
            return None, "NO_DETECTION", message

        # process the box prompt for SAM 2
        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").cpu().numpy()
        if input_boxes.shape[0] == 0:
            message = "GroundedSAM失败: 转换后的检测框为空，跳过 SAM2 分割。"
            self.get_logger().warn(message)
            return None, "NO_DETECTION", message

        # 仅在 CUDA 上启用 autocast；CPU 下走空上下文。
        amp_ctx = torch.autocast(device_type='cuda', dtype=torch.float16) if DEVICE == "cuda" else nullcontext()
        try:
            with amp_ctx:
                masks, scores, logits = self.sam2_predictor.predict(
                    point_coords=None,
                    point_labels=None,
                    box=input_boxes,
                    multimask_output=False,
                )
        except Exception as e:
            message = f"GroundedSAM失败: SAM2 分割失败: {e}"
            self.get_logger().error(message)
            return None, "SAM_FAILED", message
        """
        Post-process the output of the model to get the masks, scores, and logits for visualization
        """
        # convert the shape to (n, H, W)
        if masks is None or masks.size == 0:
            message = "GroundedSAM失败: SAM2 未输出有效 mask。"
            self.get_logger().warn(message)
            return None, "SAM_NO_MASK", message
        if masks.ndim == 4:
            masks = masks.squeeze(1)

        # 将掩码数据存入 Detections 对象，方便处理
        detections = sv.Detections(xyxy=input_boxes,mask=masks.astype(bool))

        # 合并所有掩码
        if detections.mask is not None and len(detections.mask) > 0:
            combined_mask = np.any(detections.mask, axis=0)
            return (combined_mask * 255).astype(np.uint8), "OK", None
        message = "GroundedSAM失败: 未生成有效掩码。"
        self.get_logger().warn(message)
        return None, "SAM_NO_MASK", message


    def handle_gsam_request(self,request,response):
        self.get_logger().info('收到Grounded_Sam请求，正在处理...')
        # setup the input image and text prompt for SAM 2 and Grounding DINO
        # VERY important: text queries need to be lowercased + end with a dot(已在process_single_image自动实现无需额外增加)
        text_prompt = request.text_prompt.lower().strip()
        input_path = request.input_path
        target_filename = "color.png"
        image_path = os.path.join(input_path,target_filename)
        if not os.path.exists(image_path):
            response.success = False
            response.output_path = str(input_path)
            response.error_code = "IMAGE_NOT_FOUND"
            response.message = f"GroundedSAM失败: 输入图像不存在: {image_path}"
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f"成功读取图像: {image_path}")
        try:
            mask_image, error_code, error_message = self.process_single_image(image_path,text_prompt)
        except Exception as e:
            mask_image = None
            error_code = "INTERNAL_ERROR"
            error_message = f"GroundedSAM失败: 未处理异常: {e}"
            self.get_logger().error(error_message)
        if mask_image is None:
            response.success = False
            response.output_path = str(input_path)
            response.error_code = error_code or "INTERNAL_ERROR"
            response.message = error_message or "GroundedSAM失败: 未生成有效掩码，请检查 text_prompt 或输入图片。"
            self.get_logger().error(response.message)
            return response
        mask_filename = "workspace_mask.png"
        mask_path = os.path.join(input_path,mask_filename)
        try:
            if not cv2.imwrite(str(mask_path), mask_image):
                raise RuntimeError("cv2.imwrite 返回 False")
        except Exception as e:
            response.success = False
            response.output_path = str(input_path)
            response.error_code = "INTERNAL_ERROR"
            response.message = f"GroundedSAM失败: 保存 mask 失败: {e}"
            self.get_logger().error(response.message)
            return response
        self.get_logger().info(f"已保存结果至{mask_path}")
        response.success = True
        response.output_path = str(input_path)
        response.error_code = "OK"
        response.message = f"GroundedSAM成功: 已生成掩码 {mask_path}"
        return response
            

def build_sam2(
    config_file,
    ckpt_path=None,
    device="cuda",
    mode="eval",
    hydra_overrides_extra=[],
    apply_postprocessing=True,
    config_dir=None,
    **kwargs,
):
    if config_dir is None:
        raise ValueError("config_dir must be provided(absolute path to configs directory)")
    if apply_postprocessing:
        hydra_overrides_extra = hydra_overrides_extra.copy()
        hydra_overrides_extra += [
            # dynamically fall back to multi-mask if the single mask is not stable
            "++model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability=true",
            "++model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta=0.05",
            "++model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh=0.98",
        ]
    from hydra.core.global_hydra import GlobalHydra
    if GlobalHydra.instance().is_initialized():
        GlobalHydra.instance().clear()
    with initialize_config_dir(config_dir = config_dir, version_base=None):
        cfg = compose(config_name=config_file, overrides=hydra_overrides_extra)
        OmegaConf.resolve(cfg)
    # Read config and init model
        model = instantiate(cfg.model, _recursive_=True)
    _load_checkpoint(model, ckpt_path)
    model = model.to(device)
    if mode == "eval":
        model.eval()
    return model

def _load_checkpoint(model, ckpt_path):
    if ckpt_path is not None:
        sd = torch.load(ckpt_path, map_location="cpu", weights_only=True)["model"]
        missing_keys, unexpected_keys = model.load_state_dict(sd)
        if missing_keys:
            logging.error(missing_keys)
            raise RuntimeError()
        if unexpected_keys:
            logging.error(unexpected_keys)
            raise RuntimeError()
        logging.info("Loaded checkpoint sucessfully")
        
def main():
    rclpy.init()
    grounded_sam_server = GroundedSamServer()
    try:
        rclpy.spin(grounded_sam_server)
    except KeyboardInterrupt:
        grounded_sam_server.get_logger().info('服务器节点被用户手动关闭...')
    finally:
        grounded_sam_server.get_logger().info('服务器节点正在关闭...') 
        grounded_sam_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('服务器节点已关闭.')
