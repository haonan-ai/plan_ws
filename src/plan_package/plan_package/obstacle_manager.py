import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math

class ObstacleManagerNode(Node):
    def __init__(self):
        super().__init__('obstacle_manager')
        
        # 配置QoS以匹配/global_map话题的Transient Local持久性
        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅鼠标点击和地图
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(OccupancyGrid, '/global_map', self.map_callback, map_qos)
        
        # 发布修改后的地图
        self.map_publisher = self.create_publisher(OccupancyGrid, '/global_map', map_qos)
        
        # 存储当前地图
        self.current_map = None
        self.obstacle_radius = 0.2  # 障碍物半径（米）
        self.obstacle_value = 100   # 障碍物值（100表示占用）
        
        self.get_logger().info('障碍物管理器已启动，点击rviz2中的点来设置临时障碍物')
    
    def map_callback(self, msg: OccupancyGrid):
        """接收并存储当前地图"""
        self.current_map = msg
        self.get_logger().debug('收到地图更新')
    
    def clicked_point_callback(self, msg: PointStamped):
        """处理鼠标点击事件"""
        if self.current_map is None:
            self.get_logger().warn('地图尚未加载，无法设置障碍物')
            return
        
        # 获取点击的世界坐标
        world_x = msg.point.x
        world_y = msg.point.y
        
        self.get_logger().info(f'收到点击点: ({world_x:.2f}, {world_y:.2f})')
        
        # 将世界坐标转换为地图坐标
        map_x, map_y = self.world_to_map_coords(world_x, world_y)
        
        if map_x is None or map_y is None:
            self.get_logger().warn('点击点超出地图范围')
            return
        
        # 在地图上设置障碍物
        self.add_obstacle_to_map(map_x, map_y)
        
        # 发布修改后的地图
        self.map_publisher.publish(self.current_map)
        self.get_logger().info(f'已在地图坐标 ({map_x}, {map_y}) 设置障碍物')
    
    def world_to_map_coords(self, world_x, world_y):
        """将世界坐标转换为地图坐标"""
        if self.current_map is None:
            return None, None
        
        # 获取地图原点
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        
        # 计算相对于地图原点的偏移
        relative_x = world_x - origin_x
        relative_y = world_y - origin_y
        
        # 转换为地图像素坐标
        map_x = int(relative_x / self.current_map.info.resolution)
        map_y = int(relative_y / self.current_map.info.resolution)
        
        # 检查是否在地图范围内
        if (0 <= map_x < self.current_map.info.width and 
            0 <= map_y < self.current_map.info.height):
            return map_x, map_y
        else:
            return None, None
    
    def add_obstacle_to_map(self, center_x, center_y):
        """在地图上添加圆形障碍物"""
        if self.current_map is None:
            return
        
        # 计算障碍物半径在地图上的像素数
        radius_pixels = int(self.obstacle_radius / self.current_map.info.resolution)
        
        # 获取地图数据
        map_data = list(self.current_map.data)
        width = self.current_map.info.width
        height = self.current_map.info.height
        
        # 在圆形区域内设置障碍物
        for dx in range(-radius_pixels, radius_pixels + 1):
            for dy in range(-radius_pixels, radius_pixels + 1):
                # 检查是否在圆形范围内
                if dx*dx + dy*dy <= radius_pixels*radius_pixels:
                    x = center_x + dx
                    y = center_y + dy
                    
                    # 检查边界
                    if 0 <= x < width and 0 <= y < height:
                        index = y * width + x
                        map_data[index] = self.obstacle_value
        
        # 更新地图数据
        self.current_map.data = tuple(map_data)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C,准备退出...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 