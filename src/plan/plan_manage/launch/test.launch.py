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
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('plan_manage'),
            'config',
            'rviz_config.rviz'
        ])])

    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        namespace='localization',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0'],
        output='screen')
    
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base',
        namespace='localization',
        arguments=[
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
            '--x', '4.7', '--y', '4.7', '--z', '0.0',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0'],
        output='screen')
    
    declare_map = DeclareLaunchArgument(
        'map_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('plan_manage'),
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

    test_node = Node(
        package='plan_manage',
        executable='test_node',
        name='test_node',
        namespace='map',
        output='screen')

    return LaunchDescription([
        rviz_node,
        static_map_to_odom,
        static_odom_to_base,

        declare_map,
        map_server,

        lifecycle_mgr,
        test_node
        ])