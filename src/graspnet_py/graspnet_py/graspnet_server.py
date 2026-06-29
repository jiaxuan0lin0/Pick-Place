# python
import rclpy
from rclpy.node import Node
import torch
import os
import sys
import numpy as np
from PIL import Image
import open3d as o3d
import glob
import traceback
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion
from grasp_srv_interface.srv import Graspnet
from pathlib import Path
import json
import cv2  # 用于深度图 unchanged 读取兜底

WORKSPACE_DIR = os.environ.get('GRASP_WS', '/root/host_home/ros2_ws')
GraspNetDIR = os.path.join(WORKSPACE_DIR, 'graspnet-baseline')
sys.path.append(os.path.join(GraspNetDIR, 'models'))
sys.path.append(os.path.join(GraspNetDIR, 'dataset'))
sys.path.append(os.path.join(GraspNetDIR, 'utils'))

from graspnetAPI import GraspGroup
from graspnet import GraspNet, pred_decode
from graspnet_dataset import GraspNetDataset
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

def _abs(path):
    return os.path.abspath(path)

def _ensure_bool_mask(mask):
    if mask.ndim == 3:
        mask = mask[..., 0]
    return mask > 0

def read_color_rgb(path):
    img = Image.open(path).convert('RGB')
    return np.array(img, dtype=np.float32) / 255.0

def read_mask_single_channel(path):
    img = Image.open(path)
    # 保留为单通道 8 位即可
    if img.mode != 'L':
        img = img.convert('L')
    return np.array(img)

def read_depth_unchanged(path):
    """
    尝试保持 16 位深度读取。
    优先用 PIL 原样读取；若得到 8 位，再用 OpenCV 的 IMREAD_UNCHANGED 兜底。
    返回 numpy.uint16 数组。
    """
    img = Image.open(path)
    print(f"depth PIL mode: {img.mode}")
    arr = np.array(img)
    # 如果 PIL 给的是 16 位（'I;16' 或 'I' 等），arr.dtype 通常是 uint16 或 int32
    if arr.dtype == np.uint16:
        return arr
    # 若是 8 位，则用 OpenCV 再读一遍（不改变位深）
    arr_cv = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if arr_cv is None:
        raise RuntimeError(f"OpenCV 无法读取深度图: {path}")
    print(f"depth OpenCV dtype: {arr_cv.dtype}, shape: {arr_cv.shape}")
    if arr_cv.dtype == np.uint16:
        return arr_cv
    # 如果仍不是 16 位，说明源文件就是 8 位，无法恢复真实深度
    # 强行提升到 uint16 只会把 0～255 的数搬到 0～255（信息已丢失）
    raise RuntimeError("深度图文件不是 16 位（很可能以 8 位保存）。请检查保存流程，确保以 uint16 PNG 写出。")

class GraspnetServer(Node):
    def __init__(self):
        super().__init__('graspnet_server')
        self.get_logger().info('Graspnet 服务节点启动中')

        # 参数
        self.declare_parameter('checkpoint_path', os.path.join(WORKSPACE_DIR, 'graspnet-baseline/logs/log_rs/checkpoint.tar'))
        self.declare_parameter('num_point', 20000)
        self.declare_parameter('num_view', 300)
        self.declare_parameter('collision_thresh', 0.01)
        self.declare_parameter('voxel_size', 0.01)
        self.declare_parameter('use_workspace_mask', True)
        self.declare_parameter('max_depth_raw', 2000)
        self.declare_parameter('visualize_sampled', True)
        self.declare_parameter('debug', True)

        # 获取参数
        self.checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        self.num_point = self.get_parameter('num_point').get_parameter_value().integer_value
        self.num_view = self.get_parameter('num_view').get_parameter_value().integer_value
        self.collision_thresh = self.get_parameter('collision_thresh').get_parameter_value().double_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.use_workspace_mask = self.get_parameter('use_workspace_mask').get_parameter_value().bool_value
        self.max_depth_raw = self.get_parameter('max_depth_raw').get_parameter_value().integer_value
        self.visualize_sampled = self.get_parameter('visualize_sampled').get_parameter_value().bool_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        # 服务
        self.srv = self.create_service(Graspnet, 'graspnet_service', self.handle_graspnet_request)
        self.get_logger().info('Graspnet 服务已启动，等待请求...')

        # 设备
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"计算设备: {self.device}")
        try:
            self.get_logger().info(f"CUDA 版本: {torch.version.cuda}")
        except Exception:
            pass

        # 模型
        self.get_logger().info("正在加载 GraspNet 模型...")
        self.graspnet_net = self.graspnet_get_net()
        self.get_logger().info("GraspNet 模型加载完成.")

    def graspnet_get_net(self):
        net = GraspNet(input_feature_dim=0, num_view=self.num_view, num_angle=12, num_depth=4,
                       cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        net.to(self.device)
        checkpoint = torch.load(self.checkpoint_path)
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint.get('epoch', -1)
        print(f"-> loaded checkpoint {self.checkpoint_path} (epoch: {start_epoch})")
        net.eval()
        return net

    # def get_and_process_data(self, data_dir):
    #     # 路径
    #     color_path = os.path.join(data_dir, 'color.png')
    #     depth_path = os.path.join(data_dir, 'depth.png')
    #     mask_path = os.path.join(data_dir, 'workspace_mask.png')
    #     cam_path = os.path.join(data_dir, 'camera.json')
    #     print("color 路径:", _abs(color_path))
    #     print("depth 路径:", _abs(depth_path))
    #     print("mask 路径:", _abs(mask_path))
    #     print("camera 路径:", _abs(cam_path))

    #     # 读取
    #     color = read_color_rgb(color_path)
    #     depth = read_depth_unchanged(depth_path)  # 关键修复：保持 16 位
    #     workspace_mask = read_mask_single_channel(mask_path)

    #     print(f"color 形状: {color.shape}, dtype: {color.dtype}, 范围: [{color.min():.3f}, {color.max():.3f}]")
    #     nonzero = (depth > 0).sum()
    #     print(f"depth 形状: {depth.shape}, dtype: {depth.dtype}, 非零: {nonzero}")
    #     if nonzero > 0:
    #         print(f"depth min(>0): {depth[depth>0].min()}, max: {depth.max()}")
    #     else:
    #         print("depth 全为 0，请检查深度文件")
    #     print(f"mask 形状: {workspace_mask.shape}, dtype: {workspace_mask.dtype}, 非零: {(workspace_mask>0).sum()}")

    #     # 相机参数
    #     with open(cam_path, 'r') as f:
    #         params = json.load(f)
    #     intrinsic = np.array(params['camera_matrix'], dtype=np.float32)
    #     width = int(params.get('width', 1280))
    #     height = int(params.get('height', 720))
    #     factor_depth = float(params.get('factor_depth', 1000.0))
    #     print(f"intrinsic:\n{intrinsic}")
    #     print(f"width: {width}, height: {height}, factor_depth: {factor_depth}")

    #     # 点云
    #     camera = CameraInfo(width, height,
    #                         intrinsic[0][0], intrinsic[1][1],
    #                         intrinsic[0][2], intrinsic[1][2],
    #                         factor_depth)
    #     cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    #     z = cloud[..., 2]
    #     x = cloud[..., 0]
    #     y = cloud[..., 1]
    #     valid_z = z[z > 0]
    #     print(f"点云 Z 范围: min={valid_z.min() if valid_z.size>0 else None}, max={valid_z.max() if valid_z.size>0 else None}")
    #     print(f"点云 X/Y 范围: X [{x.min()}, {x.max()}], Y [{y.min()}, {y.max()}]")

    #     # 掩膜
    #     workspace_mask_bool = _ensure_bool_mask(workspace_mask)
    #     depth_valid = (depth > 0) & (depth < self.max_depth_raw)
    #     mask = depth_valid if not self.use_workspace_mask else (workspace_mask_bool & depth_valid)
    #     mask = mask.astype(bool)
    #     print(f"mask 总像素: {mask.size}, 有效像素: {mask.sum()} (use_workspace_mask={self.use_workspace_mask}, max_depth_raw={self.max_depth_raw})")

    #     cloud_masked = cloud[mask]
    #     color_masked = color[mask]
    #     print(f"有效点数量: {len(cloud_masked)}")
    #     if len(cloud_masked) < 1000:
    #         print("警告: 掩膜后有效点过少，尝试仅用深度掩膜回退...")
    #         mask = depth_valid.astype(bool)
    #         cloud_masked = cloud[mask]
    #         color_masked = color[mask]
    #         print(f"回退后有效点数量: {len(cloud_masked)}")
    #         if len(cloud_masked) < 1000:
    #             raise RuntimeError("有效点仍过少，请检查 workspace_mask 与深度数据。")

    #     # 采样
    #     n_valid = len(cloud_masked)
    #     if n_valid >= self.num_point:
    #         idxs = np.random.choice(n_valid, self.num_point, replace=False)
    #     else:
    #         idxs1 = np.arange(n_valid)
    #         idxs2 = np.random.choice(n_valid, self.num_point - n_valid, replace=True)
    #         idxs = np.concatenate([idxs1, idxs2], axis=0)
    #     cloud_sampled = cloud_masked[idxs]
    #     color_sampled = color_masked[idxs]
    #     z_s = cloud_sampled[:, 2]
    #     print(f"采样点数量: {len(cloud_sampled)}，Z[min, max] = [{z_s.min() if z_s.size>0 else None}, {z_s.max() if z_s.size>0 else None}]")

    #     # 构造输入
    #     end_points = dict()
    #     cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    #     device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    #     cloud_sampled = cloud_sampled.to(device)
    #     end_points['point_clouds'] = cloud_sampled
    #     end_points['cloud_colors'] = color_sampled

    #     cloud_o3d = o3d.geometry.PointCloud()
    #     cloud_o3d.points = o3d.utility.Vector3dVector(cloud_sampled.astype(np.float32))
    #     cloud_o3d.colors = o3d.utility.Vector3dVector(color_sampled.astype(np.float32))

    #     return end_points, cloud_o3d


     
    def get_and_process_data(self, data_dir):
        color_path = os.path.join(data_dir, 'color.png')
        depth_path = os.path.join(data_dir, 'depth.png')
        mask_path = os.path.join(data_dir, 'workspace_mask.png')
        cam_path = os.path.join(data_dir, 'camera.json')

        color = read_color_rgb(color_path)
        depth = read_depth_unchanged(depth_path)
        workspace_mask = read_mask_single_channel(mask_path)
        with open(cam_path, 'r') as f:
            params = json.load(f)

        if color.shape[:2] != depth.shape[:2]:
            raise RuntimeError(f"color/depth 尺寸不一致: color={color.shape[:2]}, depth={depth.shape[:2]}")
        if workspace_mask.shape[:2] != depth.shape[:2]:
            raise RuntimeError(f"workspace_mask/depth 尺寸不一致: mask={workspace_mask.shape[:2]}, depth={depth.shape[:2]}")

        intrinsic = np.array(params['camera_matrix'], dtype=np.float32)
        height, width = depth.shape[:2]
        width = int(params.get('width', width))
        height = int(params.get('height', height))
        factor_depth = float(params.get('factor_depth', 1000.0))
        if depth.shape[:2] != (height, width):
            raise RuntimeError(f"camera.json 尺寸与深度图不一致: camera=({height}, {width}), depth={depth.shape[:2]}")

        self.get_logger().info(
            f"GraspNet 输入: color={color.shape}, depth={depth.shape}/{depth.dtype}, "
            f"mask={workspace_mask.shape}, factor_depth={factor_depth}, max_depth_raw={self.max_depth_raw}"
        )

        nonzero_depth = depth[depth > 0]
        if nonzero_depth.size == 0:
            raise RuntimeError("深度图全为 0")
        self.get_logger().info(
            "深度统计 raw: "
            f"min={int(nonzero_depth.min())}, median={float(np.median(nonzero_depth)):.1f}, "
            f"max={int(nonzero_depth.max())}, valid_ratio={nonzero_depth.size / depth.size:.3f}"
        )

        camera = CameraInfo(
            width, height,
            intrinsic[0][0], intrinsic[1][1],
            intrinsic[0][2], intrinsic[1][2],
            factor_depth,
        )
        cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

        depth_valid = depth > 0
        if self.max_depth_raw > 0:
            depth_valid = depth_valid & (depth <= self.max_depth_raw)
        workspace_mask_bool = _ensure_bool_mask(workspace_mask)
        mask = depth_valid if not self.use_workspace_mask else (workspace_mask_bool & depth_valid)
        mask = mask.astype(bool)

        workspace_depth = depth[workspace_mask_bool & (depth > 0)]
        if workspace_depth.size > 0:
            lt_800 = np.count_nonzero(workspace_depth < 800) / workspace_depth.size
            lt_1500 = np.count_nonzero(workspace_depth < 1500) / workspace_depth.size
            self.get_logger().info(
                "workspace 深度统计 raw: "
                f"min={int(workspace_depth.min())}, median={float(np.median(workspace_depth)):.1f}, "
                f"max={int(workspace_depth.max())}, <800={lt_800:.3f}, <1500={lt_1500:.3f}"
            )

        cloud_masked = cloud[mask]
        color_masked = color[mask]
        valid_count = len(cloud_masked)
        self.get_logger().info(
            f"有效点数量: {valid_count}/{mask.size} "
            f"(use_workspace_mask={self.use_workspace_mask}, max_depth_raw={self.max_depth_raw})"
        )
        if valid_count == 0:
            raise RuntimeError(
                "深度过滤后没有有效点。请检查 workspace_mask、深度单位，以及 max_depth_raw 是否过小。"
            )
        if valid_count < 1000:
            self.get_logger().warn(f"有效点过少: {valid_count}，GraspNet 结果可能不稳定")

        if valid_count >= self.num_point:
            idxs = np.random.choice(valid_count, self.num_point, replace=False)
        else:
            idxs1 = np.arange(valid_count)
            idxs2 = np.random.choice(valid_count, self.num_point - valid_count, replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]

        z_s = cloud_sampled[:, 2]
        self.get_logger().info(
            f"采样点数量: {len(cloud_sampled)}, Z[m] min={float(z_s.min()):.3f}, "
            f"median={float(np.median(z_s)):.3f}, max={float(z_s.max()):.3f}"
        )

        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
        cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))

        end_points = dict()
        cloud_sampled_torch = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(self.device)
        end_points['point_clouds'] = cloud_sampled_torch
        end_points['cloud_colors'] = color_sampled

        return end_points, cloud_o3d
    

    def get_grasps(self, net, end_points):
        with torch.no_grad():
            try:
                # print(f"end_p:{end_points['point_clouds']}")
                end_points_out = net(end_points)
                # print(f"out_end_p:{end_points_out}")
            except RuntimeError as e:
                if 'out of memory' in str(e).lower():
                    self.get_logger().warn('CUDA OOM，回退到 CPU 并降采样重试')
                    torch.cuda.empty_cache()
                    net.to(torch.device('cpu'))
                    end_points_cpu = {
                        'point_clouds': end_points['point_clouds'].to(torch.device('cpu')),
                        'cloud_colors': end_points['cloud_colors']
                    }
                    end_points_out = net(end_points_cpu)
                else:
                    raise
            grasp_preds = pred_decode(end_points_out)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        print(f"pred_decode 后抓取数量: {gg_array.shape[0]}")
        gg = GraspGroup(gg_array)
        return gg

    def collision_detection(self, gg, cloud_points_np):
        mfcdetector = ModelFreeCollisionDetector(cloud_points_np, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        gg = gg[~collision_mask]
        return gg

    def vis_grasps(self, gg, cloud):
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.001, origin=[0, 0, 0])
        grippers = gg.to_open3d_geometry_list()
        o3d.visualization.draw_geometries([cloud, *grippers, coordinate_frame])

    def handle_graspnet_request(self, request, response):
        self.get_logger().info(f'收到请求，input_path={request.input_path}')
        try:
            input_path = request.input_path
            end_points, cloud = self.get_and_process_data(input_path)
            # o3d.visualization.draw_geometries([cloud])

            gg = self.get_grasps(self.graspnet_net, end_points)
            self.get_logger().info(f"原始抓取数量: {len(gg)}")

            if len(gg) == 0:
                response.grasp_poses = []
                response.success = False
                response.error_code = "GRASPNET_NO_GRASP"
                response.message = "pred_decode 后没有抓取结果。请检查深度是否为 16 位、点云 Z 范围是否合理（已打印）。"
                return response

            if self.collision_thresh > 0:
                gg = self.collision_detection(gg, np.array(cloud.points))
                self.get_logger().info(f"碰撞检测后抓取数量: {len(gg)}")
                if len(gg) == 0:
                    response.grasp_poses = []
                    response.success = False
                    response.error_code = "GRASPNET_COLLISION_EMPTY"
                    response.message = "碰撞检测后没有抓取结果，请放宽 collision_thresh 或检查点云。"
                    return response

            try:
                nms_gg = gg.nms()
                if nms_gg is not None:
                    gg = nms_gg
            except ModuleNotFoundError as e:
                if e.name == 'grasp_nms':
                    self.get_logger().warn('grasp_nms 未安装，跳过 NMS；继续按 score 排序。')
                else:
                    raise
            gg.sort_by_score()
            best_grasp = gg[0]
            print("最佳抓取:", best_grasp)

            # R_align = np.array([[0, 0, 1],
            #                     [0, 1, 0],
            #                     [-1, 0, 0]], dtype=float)
            # best_grasp.rotation_matrix = best_grasp.rotation_matrix @ R_align

            pose_msg = Pose()
            pose_msg.position = Point(x=float(best_grasp.translation[0]),
                                      y=float(best_grasp.translation[1]),
                                      z=float(best_grasp.translation[2]))
            quat = R.from_matrix(best_grasp.rotation_matrix).as_quat()
            pose_msg.orientation = Quaternion(x=float(quat[0]),
                                              y=float(quat[1]),
                                              z=float(quat[2]),
                                              w=float(quat[3]))

            if self.visualize_sampled:
                self.vis_grasps(gg[:1], cloud)

            response.grasp_poses = [pose_msg]
            response.success = True
            response.error_code = "OK"
            response.message = "成功检测到 1 个最佳抓取姿态."
            return response

        except Exception as e:
            detail = f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
            self.get_logger().error(f'处理请求失败: {detail}')
            if self.debug:
                self.get_logger().error(traceback.format_exc())
            response.grasp_poses = []
            response.success = False
            response.error_code = "GRASPNET_FAILED"
            response.message = f'推理失败: {detail}'
            return response

    def find_ply_file(self, input_path):
        ply_file = glob.glob(os.path.join(input_path, '*.ply'))
        if len(ply_file) == 0:
            self.get_logger().error(f"在目录 {input_path} 中未找到 .ply 文件.")
            return None
        return ply_file[0]

    def load_ply_as_np(self, ply_file):
        self.get_logger().info(f"正在加载点云文件: {ply_file}")
        pcd = o3d.io.read_point_cloud(ply_file)
        points = np.asarray(pcd.points)
        return points

    def process_point_cloud(self, points):
        self.get_logger().info("开始处理输入点云数据...")
        n = len(points)
        print(f"输入点云数量: {n}")
        if n == 0:
            raise RuntimeError("输入 .ply 点云为空")
        if n >= self.num_point:
            idxs = np.random.choice(n, self.num_point, replace=False)
        else:
            idxs1 = np.arange(n)
            idxs2 = np.random.choice(n, self.num_point - n, replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)
        cloud_sampled = points[idxs]
        end_points = dict()
        cloud_sampled_torch = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(self.device)
        end_points['point_clouds'] = cloud_sampled_torch
        end_points['cloud_colors'] = None
        cloud_o3d = o3d.geometry.PointCloud()
        cloud_o3d.points = o3d.utility.Vector3dVector(cloud_sampled.astype(np.float32))
        self.get_logger().info("点云数据处理完成.")
        return end_points, cloud_o3d

def main(args=None):
    rclpy.init(args=args)
    graspnet_server = GraspnetServer()
    try:
        rclpy.spin(graspnet_server)
    except KeyboardInterrupt:
        graspnet_server.get_logger().info('服务器节点被用户手动关闭...')
    finally:
        graspnet_server.get_logger().info('服务器节点已关闭.')
        graspnet_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
