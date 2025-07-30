import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped,Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import TransformBroadcaster
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from visualization_msgs.msg import MarkerArray
import math

class PlanManageNode(Node):
    def __init__(self):
        super().__init__('plan_manage')

        self.x = 0.0  # 初始值无意义，等待topic
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.2

        self.v = 0.0
        self.omega = 0.0

        self.global_path = None
        self.path_index = 0

        self.goal_pose = None  # 新增：保存外部goal
        self.initial_pose_received = False  # 新增：标记是否收到初始位姿
        self._wait_initial_pose_logged = False  # 新增：只打印一次等待提示

        self.compute_global_path_ok = False
        self.send_global_path_to_follow_path_ok = False
        self.global_costmap_ok = False

        self.send_base_link = TransformBroadcaster(self)

        self.compute_global_path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.send_global_path_to_follow_path_client = ActionClient(self, FollowPath, '/follow_path')

        self.create_subscription(Path, '/global_path', self.global_path_callback, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 1)
        self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)  # 新增订阅goal
        self.create_subscription(PoseWithCovarianceStamped, '/initial_pose', self.initial_pose_callback, 10)  # 新增订阅初始位姿

        self.timer = self.create_timer(self.t, self.timer_callback)
    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.omega = msg.angular.z
    def global_costmap_callback(self, msg: OccupancyGrid):
        if not self.global_costmap_ok:
            self.global_costmap_ok = True
            self.get_logger().info('Global CostMap ok')

    def global_path_callback(self, msg: Path):
        if not msg:
            return
        self.global_path = msg
        self.path_index = 0
        self.get_logger().info(f'已接收全局路径，共 {len(self.global_path.poses)} 个点')
        self.send_global_path_to_follow_path_ok = False
    def timer_callback(self):
        if not self.initial_pose_received:
            if not self._wait_initial_pose_logged:
                self.get_logger().info('等待初始位姿...')
                self._wait_initial_pose_logged = True
            return
        else:
            self._wait_initial_pose_logged = False
        if self.global_costmap_ok and not self.compute_global_path_ok and self.goal_pose:
            self.compute_global_path()

        if self.global_path and not self.send_global_path_to_follow_path_ok:
            self.send_global_path_to_follow_path()

        dx = self.v * math.cos(self.theta) * self.t
        dy = self.v * math.sin(self.theta) * self.t
        dtheta = self.omega * self.t
        self.x += dx
        self.y += dy
        self.theta += dtheta        

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.send_base_link.sendTransform(t)

    def send_global_path_to_follow_path(self):
        if not self.send_global_path_to_follow_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('无法连接到 FollowPath 服务')
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.global_path

        self.get_logger().info('发送 Global Path 目标给 FollowPath …')
        send_goal = self.send_global_path_to_follow_path_client.send_goal_async(goal_msg)
        send_goal.add_done_callback(self.follow_path_response)
    def compute_global_path(self):
        if not self.goal_pose:
            self.get_logger().warn('未收到目标点，无法规划路径')
            return
        if not self.compute_global_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('全局路径规划服务不可用，稍后重试…')
            self.compute_global_path_ok = False
            return

        goal = ComputePathToPose.Goal()
        goal.goal = self.goal_pose  # 用收到的goal
        self.get_logger().info('发送全局路径规划请求…')
        send_goal = self.compute_global_path_client.send_goal_async(goal)
        send_goal.add_done_callback(self.global_plan_response)

    def global_plan_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('全局路径规划请求被拒绝')
            self.compute_global_path_ok = False
            return
        self.get_logger().info('全局路径规划请求已被接受')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.global_plan_result_response)

    def global_plan_result_response(self, future):
        self.compute_global_path_ok = True
        result = future.result().result
        if hasattr(result, 'path') and hasattr(result.path, 'poses'):
            num_poses = len(result.path.poses)
            self.get_logger().info(f'全局路径规划成功，路径点数: {num_poses}')
            if num_poses == 0:
                self.get_logger().warn('全局路径为空，可能目标不可达或地图有障碍')
        else:
            self.get_logger().warn('全局路径规划结果异常')
    
    def follow_path_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('发送 Global Path 目标给 FollowPath 被拒绝')
            self.send_global_path_to_follow_path_ok = False
            return
        self.send_global_path_to_follow_path_ok = True
        self.get_logger().info('FollowPath 已接收 Global Path')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_path_result_response)

    def follow_path_result_response(self, future):
        self.get_logger().info(f'FollowPath 完成')

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.compute_global_path_ok = False  # 每次收到新goal都重新规划
        self.get_logger().info(f'收到新的目标点: ({msg.pose.position.x}, {msg.pose.position.y})')

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        self.initial_pose_received = True
        self.get_logger().info(f'收到初始位姿: x={self.x}, y={self.y}, theta={self.theta}')

def main(args=None):
    rclpy.init(args=args)
    node = PlanManageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C,准备退出...')
    finally:
        node.destroy_node()