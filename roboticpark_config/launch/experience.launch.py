import os
import pathlib
import launch
import yaml
import datetime
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
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    distributed_architecture = False

    # ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M"), ], output='screen'),
    # rviz_config_path = os.path.join(general_package_dir, 'rviz', 'Demo_Crazyflie_formation.rviz')
    node_list = []

    #-------------------#
    #     Load File     #
    #-------------------#
    file = LaunchConfiguration('file')
    file_name = file.perform(context)

    general_package_dir = get_package_share_directory('roboticpark_config')
    config_path = os.path.join(general_package_dir, 'resources', file_name)
    with open(config_path, 'r') as file:
            documents = yaml.safe_load(file)

    #----------------------#
    #     Architecture     #
    #----------------------#
    if documents['Architecture']['mode'] == 'centralized':
        node_list.append(Node(
            package=documents['Architecture']['node']['pkg'],
            executable=documents['Architecture']['node']['executable'],
            name=documents['Architecture']['node']['name'],
            output='screen',
            parameters=[
                {'config_file': config_path},
                {'use_sim_time': use_sim_time},
            ]
        ))
    elif documents['Architecture']['mode'] == 'distributed_ros2':
        distributed_architecture = True
    
    #------------------------#
    #     CPU Monitoring     #
    #------------------------#
    if documents['CPU_Monitoring']['enable']:
        node_list.append(Node(
            package=documents['CPU_Monitoring']['node']['pkg'],
            executable=documents['CPU_Monitoring']['node']['executable'],
            name=documents['CPU_Monitoring']['node']['name'],
            output='screen',
            parameters=[{
                'process_name' : documents['CPU_Monitoring']['processes'],
                'process_period' : 0.5},
            ],
        ))
    
    #--------------------#
    #     Interfaces     #
    #--------------------#
    if documents['Interface']['enable']:
        if documents['Interface']['rqt']['enable']:
            node_list.append(Node(
                package=documents['Interface']['rqt']['node']['pkg'],
                executable=documents['Interface']['rqt']['node']['executable'],
                name=documents['Interface']['rqt']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
            ))
        if documents['Interface']['rviz2']['enable']:
            node_list.append(Node(
                package=documents['Interface']['rviz2']['node']['pkg'],
                executable=documents['Interface']['rviz2']['node']['executable'],
                name=documents['Interface']['rviz2']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', documents['Interface']['rviz2']['file']],
            ))
        if documents['Interface']['own']['enable']:
            node_list.append(Node(
                package=documents['Interface']['own']['node']['pkg'],
                executable=documents['Interface']['own']['node']['executable'],
                name=documents['Interface']['own']['node']['name'],
                parameters=[
                    {'use_sim_time': use_sim_time},
                ],
                arguments=['-d', documents['Interface']['own']['file']],
            ))
    
    #----------------------#
    #     Data Logging     #
    #----------------------#
    if documents['Data_Logging']['enable']:
        e = datetime.datetime.now()
        if documents['Data_Logging']['all']:
            if documents['Data_Logging']['name'] == 'date':
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', e.strftime("%Y-%m-%d-%H-%M")], output='screen', shell=True
                ))
            else:
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-a', '-o', documents['Data_Logging']['name']], output='screen', shell=True
                ))
        else:
            if documents['Data_Logging']['name'] == 'date':
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', e.strftime("%Y-%m-%d-%H-%M"), documents['Data_Logging']['topics']], output='screen', shell=True
                ))
            else:
                node_list.append(ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', documents['Data_Logging']['name'], documents['Data_Logging']['topics']], output='screen', shell=True
                ))
    
    #------------------------#
    #     Operation mode     #
    #------------------------#
    if documents['Operation']['mode'] == 'virtual' or documents['Operation']['mode'] == 'hybrid':
        if documents['Operation']['tool'] == 'Webots':
            webots = WebotsLauncher(
                world=PathJoinSubstitution([general_package_dir, 'worlds', documents['Operation']['world']]),
                mode='realtime',
                ros2_supervisor=True
            )
            reset_handler = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots._supervisor,
                    on_exit=get_ros2_nodes,
                )
            )
            kill_ros2 = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
            node_list.append(webots)
            node_list.append(webots._supervisor)
            node_list.append(reset_handler)
            node_list.append(kill_ros2)
        elif documents['Operation']['tool'] == 'Gazebo':
            world_path = os.path.join(general_package_dir, 'worlds', documents['Operation']['world'])
            gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args',
                ], output='screen'
            )

            kill_ros2 = launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=gazebo,
                    on_exit=[
                        launch.actions.EmitEvent(event=launch.events.Shutdown())
                    ],
                )
            )
            node_list.append(gazebo)
            node_list.append(kill_ros2)

    
    #----------------#
    #     Robots     #
    #----------------#
    print('TO-DO: Robots')
    for x in documents['Robots']:
        print(x)

    #--------------------#
    #     Supervisor     #
    #--------------------#
    print('TO-DO: Supervisor')

    #---------------#
    #     Other     #
    #---------------#    
    print('TO-DO: Physical nodes: Positioning System')

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