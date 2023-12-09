import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_ros2_nodes(context, *args):
    file = LaunchConfiguration('file')
    file_name = file.perform(context)
    print(file_name)

    general_package_dir = get_package_share_directory('roboticpark_config')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    config_path = os.path.join(general_package_dir, 'resources', file_name)

    # ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M"), ], output='screen'),
    # rviz_config_path = os.path.join(general_package_dir, 'rviz', 'Demo_Crazyflie_formation.rviz')
    node_list = []

    

    '''
    node_list.append(Node(
        package='turtlesim',
        namespace=file,
        executable='turtlesim_node',
        name='sim'
    ))
    '''

    if 0:
        node_list.append(Node(
                package='rqt_gui',
                executable='rqt_gui',
                name='interface',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
            )
        )
    if 0:
        node_list.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', rviz_config_path],
            )
        )
    if 0:
        node_list.append(Node(
                package='measure_process_ros2_pkg',
                executable='measure_process',
                name='benchmark',
                output='screen',
                parameters=[{
                    'process_name' : 'webots-bin, driver, ros2, swarm_driver, rviz2, kheperaIV_clien, centralized_for',
                    'process_period' : 0.5},
                ],
            )
        )
    if 0:
        node_list.append(Node(
                package='uned_crazyflie_driver',
                executable='swarm_driver',
                name='swarm',
                output='screen',
                parameters=[
                    {'enviroment': 'mrs'},
                    {'config': config_path},
                    {'robots': 'dron01, dron02, dron03, dron04, dron05'}
                ]
            )
        )
    if 0:
        node_list.append(Node(
                package='uned_swarm_task',
                executable='centralized_formation_controller',
                name='formation_controller',
                output='screen',
                parameters=[
                    {'config_file': config_path},
                    {'use_sim_time': use_sim_time},
                ]
            )
        )
    '''
    node_list.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            name='dron01',
            arguments=['--frame-id', 'dron01/base_link', '--child-frame-id', 'map'],
        )
    )
    '''

    return node_list

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'file',
            default_value='pwd',
            description='path config file'
        ),
        launch.actions.OpaqueFunction(function=get_ros2_nodes),
        # ExecuteProcess(cmd=[file], output='screen'),
    ])