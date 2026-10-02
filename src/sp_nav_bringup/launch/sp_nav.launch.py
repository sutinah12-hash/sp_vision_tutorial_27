import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    share = get_package_share_directory('sp_nav_bringup')
    params = LaunchConfiguration('params_file').perform(context)
    controller = LaunchConfiguration('controller').perform(context)
    if controller not in ('mpc', 'pid'):
        raise ValueError('controller must be mpc or pid')
    smooth = LaunchConfiguration('smoothing').perform(context).lower() == 'true'
    # pluginlib looks up the XML class name, not its C++ type string.
    selected = ('MpcController', 'MpcController') if controller == 'mpc' else (
        'PidController', 'PidController')
    result = []
    for package, executable, name, overrides in [
        ('sp_map_server', 'esdf_map_publisher', 'esdf_map_publisher', {}),
        ('sp_global_planner', 'planner_server', 'planner_server', {'AStar.smoothing_enabled': smooth}),
        ('sp_controller_server', 'controller_node', 'controller_server',
         {'plugin_name': selected[0], 'plugin_type': selected[1]}),
        ('sp_decision', 'sp_decision_node', 'sp_decision', {}),
        ('sp_nav_bt', 'nav_interface_node', 'nav_interface_node', {}),
    ]:
        result.append(Node(package=package, executable=executable, name=name,
                           output='screen', parameters=[params, overrides]))
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        result.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                           arguments=['-d', os.path.join(share, 'rviz', 'rviz.rviz')]))
    return result


def generate_launch_description():
    share = get_package_share_directory('sp_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav_params.yaml')),
        DeclareLaunchArgument('controller', default_value='mpc', choices=['mpc', 'pid']),
        DeclareLaunchArgument('smoothing', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=nodes),
    ])
