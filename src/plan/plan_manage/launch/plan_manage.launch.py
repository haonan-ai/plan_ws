from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    plan_manage = Node(
        package='plan_manage',
        executable='plan_manage',
        name='plan_manage',
        namespace='plan',
        output='screen')

    declare_global_plan_params = DeclareLaunchArgument(
        'global_plan_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('plan_manage'),
            'config',
            'global_plan.yaml'
        ]),
        description='global_plan_parameters .yaml')

    declare_global_costmap_params = DeclareLaunchArgument(
        'global_costmap_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('plan_manage'),
            'config',
            'global_costmap.yaml'
        ]),
        description='global_costmap_parameters .yaml')

    global_plan_yaml = LaunchConfiguration('global_plan_params')
    global_costmap_yaml = LaunchConfiguration('global_costmap_params')

    global_plan_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='global_plan_server',
        namespace='plan',
        output='screen',
        parameters=[global_plan_yaml, global_costmap_yaml],
        )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        namespace='plan',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'global_plan_server'
                ]}])

    return LaunchDescription([
        plan_manage,

        declare_global_plan_params,
        declare_global_costmap_params,
        global_plan_server,

        lifecycle_mgr,
        ])