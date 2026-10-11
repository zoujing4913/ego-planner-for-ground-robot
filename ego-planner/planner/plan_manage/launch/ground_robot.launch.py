from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('ego_planner')
    config = LaunchConfiguration('config')
    frame = LaunchConfiguration('planning_frame')
    scan = LaunchConfiguration('scan_topic')
    odom = LaunchConfiguration('odom_topic')
    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    declarations = [
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([share, 'config', 'ground_robot.yaml'])),
        DeclareLaunchArgument('planning_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('goal_topic', default_value='goal_pose'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    planner = Node(
        package='ego_planner', executable='ego_planner_node', name='ego_planner_node', output='screen',
        parameters=[config, {'use_sim_time': sim_time, 'grid_map.frame_id': frame, 'grid_map.scan_topic': scan}],
        remappings=[('/odom_map', odom), ('/grid_map/odom', odom), ('/way_point', LaunchConfiguration('goal_topic'))],
    )
    controller = Node(
        package='ego_planner', executable='traj_server', name='traj_server', output='screen',
        parameters=[config, {'use_sim_time': sim_time, 'world_frame': frame,
                            'base_frame': LaunchConfiguration('base_frame'),
                            'cmd_topic': LaunchConfiguration('cmd_vel_topic')}],
        remappings=[('/state_estimation', odom), ('scan', scan)],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': sim_time}],
        arguments=['-d', PathJoinSubstitution([share, 'config', 'ground_robot.rviz']), '-f', frame],
        remappings=[('/goal_pose', LaunchConfiguration('goal_topic')), ('/scan', scan)],
    )
    return LaunchDescription(declarations + [planner, controller, rviz])
