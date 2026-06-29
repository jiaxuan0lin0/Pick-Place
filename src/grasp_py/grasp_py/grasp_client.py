from grasp_srv_interface.srv import Graspnet, GroundedSam, TriggerGrasp
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

class GraspServerNode(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self.get_logger().info('Grasp 客户端节点启动中')

        self.reentrant_group = ReentrantCallbackGroup()

        ############################################################
        # --- 内部服务客户端，负责调用graspnet、groundedsam服务 ---
        ############################################################

        #创建groundedsam_cli服务
        self.gsam_cli = self.create_client(GroundedSam, 'grounded_sam_service', 
                                           callback_group=self.reentrant_group)
        while not self.gsam_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待...')
        self.get_logger().info('GroundedSam 客户端已启动.')

        #创建graphnet_cli服务
        self.gspnet_cli = self.create_client(Graspnet, 'graspnet_service', 
                                             callback_group=self.reentrant_group)
        while not self.gspnet_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务不可用，正在等待...')
        self.get_logger().info('Graspnet 客户端已启动.')

        # #创建 move_cli服务
        # self.move_cli = self.create_client(Move, 'move')
        # while not self.move_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Move 服务不可用，正在等待...')
        # self.get_logger().info('Move 客户端已启动.')

        #############################################
        # --- 外部服务服务端，负责接收指令开始grasp流程 ---
        #############################################
        self.trigger_service = self.create_service(TriggerGrasp, 
                                                   'trigger_grasp_pipeline', 
                                                   self.trigger_grasp_callback,
                                                   callback_group=self.reentrant_group)
        self.get_logger().info('TriggerGrasp 服务已启动，等待请求...')

    def call_grounded_sam(self,input_path=None, text_prompt=None):
        self.get_logger().info('准备调用 GroundedSam 服务...')
        #创建请求
        gsam_req = GroundedSam.Request()
        #设置请求参数
        self.get_logger().info('开始设置请求参数')
        gsam_req.input_path = input_path
        gsam_req.text_prompt = text_prompt
        self.get_logger().info(f'请求参数已设置: input_path={gsam_req.input_path}, text_prompt={gsam_req.text_prompt}')
        self.get_logger().info('正在发送请求...')
        try:
            response = self.gsam_cli.call(gsam_req)
            self.get_logger().info('收到GroundedSam响应')
            return response
        except Exception as e:
            message = f'调用 GroundedSam 服务时出错: {e}'
            self.get_logger().error(message)
            response = GroundedSam.Response()
            response.success = False
            response.output_path = input_path or ''
            response.error_code = "GSAM_SERVICE_CALL_FAILED"
            response.message = message
            return response
    
    def call_graspnet(self, input_path=None):
        self.get_logger().info('准备调用 Graspnet 服务...')
        #创建请求
        gspnet_req = Graspnet.Request()
        #设置请求参数
        self.get_logger().info('开始设置请求参数')
        gspnet_req.input_path = input_path
        self.get_logger().info(f'请求参数已设置: input_path={gspnet_req.input_path}')
        self.get_logger().info('正在发送请求...')
        try:
            response = self.gspnet_cli.call(gspnet_req)
            self.get_logger().info('收到Graspnet响应')
            return response
        except Exception as e:
            message = f'调用 Graspnet 服务时出错: {e}'
            self.get_logger().error(message)
            response = Graspnet.Response()
            response.success = False
            response.error_code = "GRASPNET_SERVICE_CALL_FAILED"
            response.message = message
            response.grasp_poses = []
            return response
    def send_request(self,input_path=None, text_prompt=None):
        print("------")
        self.get_logger().info(f'准备调用 GroundedSam 服务 with input_path={input_path}, text_prompt={text_prompt}')
        gsam_response = self.call_grounded_sam(input_path=input_path, text_prompt=text_prompt)
        if gsam_response is None:
            self.get_logger().error('GroundedSam 服务调用失败，未收到响应.')
            return None
        if not gsam_response.success:
            self.get_logger().error(f'GroundedSam 服务执行失败: {gsam_response.message}')
            return gsam_response
        self.get_logger().info('GroundedSam 服务调用成功')

        self.get_logger().info(f'准备调用 Graspnet 服务 with input_path={gsam_response.output_path}')
        gspnet_response = self.call_graspnet(gsam_response.output_path)
        if gspnet_response is None:
            self.get_logger().error('Graspnet 服务调用失败，未收到响应.')
            return None
        if not gspnet_response.success:
            self.get_logger().error(f'Graspnet 服务执行失败: {gspnet_response.message}')
            return gspnet_response
        self.get_logger().info('Graspnet 服务调用成功.')
        return gspnet_response
        
    def trigger_grasp_callback(self, request, response):
        """
        当'trigger_grasp_pipeline'服务被调用时触发此回调函数
        
        Args:
            request (grasp_srv_interface.srv.TriggerGrasp.Request): 
                服务请求，包含触发抓取流程的参数
                - input_path (str): 输入图像路径
                - text_prompt (str): 文本提示
            

            response (grasp_srv_interface.srv.TriggerGrasp.Response): 
                服务响应，包含抓取结果
                - success (bool): 抓取是否成功
                - message (str): 抓取结果信息
                - grasp_poses (list of Pose): 抓取位姿列表

        Returns:
            grasp_srv_interface.srv.TriggerGrasp.Response:
            - success (bool): 抓取是否成功
            - message (str): 抓取结果信息
            - grasp_poses (list of Pose): 抓取位姿列表
        """
        self.get_logger().info(f'收到 TriggerGrasp 服务请求: path={request.input_path}, '
                               f'prompt={request.text_prompt}, '
                               '\n开始执行抓取流程...')
        # 有待完善，现在暂时使用外部请求地址，后面继承move capture后应当调整为使用capture的图像路径
        response_req = self.send_request(input_path=request.input_path,
                                     text_prompt=request.text_prompt)
        # 填充并返回响应
        # print(response)
        if response_req and response_req.success:
            self.get_logger().info('视觉处理执行成功,正在返回位姿...')
            response.success = True
            response.error_code = response_req.error_code or "OK"
            response.message = response_req.message or "视觉Pipeline执行成功"
            response.grasp_poses = response_req.grasp_poses
            # print(response)
        else:
            message = response_req.message if response_req and response_req.message else "视觉Pipeline执行失败,未有抓取位姿"
            error_code = response_req.error_code if response_req and response_req.error_code else "INTERNAL_ERROR"
            self.get_logger().error(f'视觉处理执行失败: {message}')
            response.success = False
            response.error_code = error_code
            response.message = message
            response.grasp_poses = []
        print("返回response")
        return response
        
def main(args=None):
    rclpy.init(args=args)
    grasp_server_node = GraspServerNode()
    #创建多线程执行器
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(grasp_server_node)

    try:
        grasp_server_node.get_logger().info('GraspSergverNode 正在运行，等待服务请求...')
        executor.spin()
    except KeyboardInterrupt:
        grasp_server_node.get_logger().info('GraspServerNode 被用户手动关闭...')
    except Exception as e:
        grasp_server_node.get_logger().error(f'GraspServerNode 运行时发生错误: {e}')
    finally:
        grasp_server_node.get_logger().info('GraspServerNode 正在关闭...')
        executor.shutdown()
        grasp_server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
