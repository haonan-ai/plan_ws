# 环境要求

- 操作系统：Ubuntu 22.04  
- ROS 版本：ROS 2 Humble

# 安装步骤

```bash
# 克隆仓库
git clone https://github.com/haonan-ai/plan_ws.git
或
git clone git@github.com:haonan-ai/plan_ws.git

# 进入工作空间
cd plan_ws

# 安装依赖
rosdep install -r -y --from-paths ./src --ignore-src

# 编译工作空间
colcon build --symlink-install
source install/setup.zsh

# 运行
ros2 launch plan_manage test_node.launch.py
ros2 launch plan_manage plan.launch.py
```
# launch文件

## test_node.launch.py
- 启动Rviz2
- 读取map.pgm，发布topic：/map/global_map
- 收到/clicked_point后发布/task_manager/PoseArray
- 收到/plan/cmd_vel后，更新车辆位姿，发布/tf
## plan.launch.py
- 收到/task_manager/pose_array，发布/plan/global_paths
- 收到/task_manager/pose,发布/plan/global_path
- 订阅/plan/global_path，输出控制指令：/plan/cmd_vel

# 使用说明

启动后，在 RViz 中使用以下工具：
- Publish Point：模拟发布多途径点，得到全局路径
- pose goal:发布pose，开始前往目标点
