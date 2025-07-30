from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d','/home/jack/plan_ws/config/rviz_config.rviz'],)

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0'],
        output='screen')

    declare_map = DeclareLaunchArgument(
        'map_params',
        default_value='/home/jack/plan_ws/map/map.yaml',
        description='Absolute path to map .yaml')
    declare_global_plan_params = DeclareLaunchArgument(
        'global_plan_params',
        default_value='/home/jack/plan_ws/config/global_plan.yaml',
        description='global_plan_parameters .yaml')
    declare_local_plan_params = DeclareLaunchArgument(
        'local_plan_params',
        default_value='/home/jack/plan_ws/config/local_plan.yaml',
        description='Full path to the MPPI controller param file.')
    
    map_yaml = LaunchConfiguration('map_params')
    global_plan_yaml = LaunchConfiguration('global_plan_params')
    local_plan_yaml = LaunchConfiguration('local_plan_params')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'topic_name': 'global_map'}])

    global_plan_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='global_plan_server',
        output='screen',
        parameters=[global_plan_yaml])
    
    local_plan_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='local_plan_server',
        output='screen',
        parameters=[local_plan_yaml])

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server',
                'global_plan_server',
                'local_plan_server',
                ]}])

    plan_manage = Node(
        package='plan_package',
        executable='plan_manage',
        name='plan_manage',
        output='screen')

    obstacle_manager = Node(
        package='plan_package',
        executable='obstacle_manager',
        name='obstacle_manager',
        output='screen')

    return LaunchDescription([
        rviz_node,
        static_map_to_odom,
        plan_manage,
        obstacle_manager,

        declare_map,
        declare_global_plan_params,
        declare_local_plan_params,
        map_server,
        global_plan_server,
        local_plan_server,

        lifecycle_mgr,
        ])
