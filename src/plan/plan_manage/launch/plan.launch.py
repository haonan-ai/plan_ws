from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    global_path_node = Node(
        package='plan_manage',
        executable='global_path_node',
        name='global_path_node',
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

    global_plan_yaml = LaunchConfiguration('global_plan_params')

    global_plan_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='global_plan_server',
        namespace='plan',
        output='screen',
        parameters=[global_plan_yaml],
        )
    
    plan_manage_node = Node(
        package='plan_manage',
        executable='plan_manage_node',
        name='plan_manage_node',
        namespace='plan',
        output='screen')

    declare_local_plan_params = DeclareLaunchArgument(
        'local_plan_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('plan_manage'),
            'config',
            'local_plan.yaml'
        ]),
        description='Full path to the MPPI controller param file.')

    local_plan_yaml = LaunchConfiguration('local_plan_params')

    local_plan_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='local_plan_server',
        namespace='plan',
        output='screen',
        parameters=[local_plan_yaml],
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
                'global_plan_server',
                'local_plan_server',
                ]}])

    return LaunchDescription([
        global_path_node,

        declare_global_plan_params,
        global_plan_server,
        
        plan_manage_node,

        declare_local_plan_params,
        local_plan_server,

        lifecycle_mgr,
        ])