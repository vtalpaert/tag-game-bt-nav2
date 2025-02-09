import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                          IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    tag_bt_nav2_dir = get_package_share_directory('tag_bt_nav2')

    # Launch configuration variables
    params_file = os.path.join(tag_bt_nav2_dir, 'config', 'nav2_params.yaml')
    bt_xml_file = os.path.join(tag_bt_nav2_dir, 'config', 'navigate_to_pose_w_replanning_and_recovery.xml')
    autostart = LaunchConfiguration('autostart')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': bt_xml_file
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    # Define robot poses
    robots = {
        'robot1': {'x': -1.7, 'y': -0.5, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        'robot2': {'x': 0.6, 'y': 1.7, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': -2.0}
    }

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name, init_pose in robots.items():
        group = GroupAction([
            LogInfo(msg=['Launching ', robot_name]),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                         'launch',
                                                         'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot_name,
                                'use_namespace': 'True',
                                'use_sim_time': 'True',
                                'params_file': configured_params,
                                'autostart': autostart,
                                'use_rviz': 'True',
                                'use_simulator': 'False',
                                'headless': 'True',
                                'use_robot_state_pub': use_robot_state_pub,
                                'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                'z_pose': TextSubstitution(text=str(init_pose['z'])),
                                'roll': TextSubstitution(text=str(init_pose['roll'])),
                                'pitch': TextSubstitution(text=str(init_pose['pitch'])),
                                'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                'robot_name': TextSubstitution(text=robot_name)}.items())
        ])
        bringup_cmd_group.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Log information
    ld.add_action(LogInfo(msg=['Starting 2 robots simulation']))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                         msg=['params yaml: ', params_file]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                         msg=['using robot state pub: ', use_robot_state_pub]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings),
                         msg=['autostart: ', autostart]))

    # Add the actions to launch all nodes
    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
