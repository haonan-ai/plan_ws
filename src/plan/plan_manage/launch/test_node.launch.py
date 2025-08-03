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
        namespace='test',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('plan_manage'),
            'config',
            'rviz_config.rviz'
        ])])

    test_node = Node(
        package='plan_manage',
        executable='test_node',
        name='test_node',
        namespace='test',
        output='screen')

    declare_map = DeclareLaunchArgument(
        'map_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav2_map_server'),
            'map',
            'map.yaml'
        ]),
        description='Absolute path to map .yaml')
    
    map_yaml = LaunchConfiguration('map_params')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='map',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'topic_name': 'global_map'}])

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        namespace='map',
        parameters=[{
            'autostart': True,
            'node_names': [
                'map_server'
                ]}])
    return LaunchDescription([
        rviz_node,
        test_node,
        declare_map,
        map_server,
        lifecycle_mgr
        ])