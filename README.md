# 环境要求

- 操作系统：Ubuntu 22.04  
- ROS 版本：ROS 2 Humble

# 安装步骤

```bash
# 克隆仓库
git clone https://github.com/haonan-ai/plan_ws.git
cd plan_ws

# 安装依赖
rosdep install -r -y --from-paths ./src --ignore-src

# 编译工作空间
colcon build --symlink-install

# 启动系统
ros2 launch plan_launch plan_launch.py
```

# 使用说明
启动后，在 RViz 中使用以下工具：

- 2D Pose Estimate：设置机器人初始位置

- 2D Goal Pose：设置目标位置，触发路径规划

- Publish Point：点击地图放置障碍物
