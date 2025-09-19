import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, LogInfo, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration  # <-- correct import at top

def generate_launch_description():
    # Declare a launch argument for config file
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            os.environ['HOME'],
            'ros2_ws/src/auto_nav_pipeline/config/auto_nav_config.yaml'
        ),
        description='Path to auto_nav_pipeline config YAML file'
    )

    # Function to load YAML and setup commands
    def setup_commands(context, *args, **kwargs):
        file_path = context.launch_configurations['config_file']
        with open(file_path, 'r') as f:
            cfg = yaml.safe_load(f)

        params = cfg['auto_nav_pipeline_node']['ros__parameters']
        spawn_pose = params['spawn_pose']
        goal_pose = params['goal_pose']
        map_file = params['map']
        world_launch_file = params['world_launch_file']

        gazebo_cmd = [
            'ros2', 'launch', 'turtlebot3_gazebo', world_launch_file,
            f"x_pose:={spawn_pose['x']}",
            f"y_pose:={spawn_pose['y']}",
            f"Z_pose:={spawn_pose['z']}"
        ]

        nav2_cmd = [
            'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
            'use_sim_time:=True',
            f"map:={map_file}"
        ]
        
        auto_localize_cmd = [
            'ros2', 'run', 'simple_navigation', 'auto_localize_and_nav'
        ]

        return [
            LogInfo(msg="[AUTO_NAV_PIPELINE] Starting pipeline..."),

            # Step 1: Launch Gazebo (logs only)
            LogInfo(msg=f"[AUTO_NAV_PIPELINE] Launching Gazebo..."),
            ExecuteProcess(cmd=gazebo_cmd, output='screen', shell=True),

            # Step 2: Wait 10s then launch Navigation2 (logs only)
            TimerAction(
                period=10.0,
                actions=[
                    LogInfo(msg=f"[AUTO_NAV_PIPELINE] Launching Nav2..."),
                    ExecuteProcess(cmd=nav2_cmd, output='screen', shell=True)
                ]
            ),

            # Step 3: Wait another 10s then run auto_localize_and_nav (logs only)
            TimerAction(
                period=20.0,
                actions=[
                    LogInfo(msg=f"[AUTO_NAV_PIPELINE] Launching auto_localize_and_nav..."),
                    ExecuteProcess(cmd=auto_localize_cmd, output='screen', shell=True)
                ]
            ),

            # Step 4: Start auto_nav_pipeline C++ node (keep on screen for focus)
            LogInfo(msg="[AUTO_NAV_PIPELINE] Starting auto_nav_pipeline C++ node..."),
            Node(
                package='auto_nav_pipeline',
                executable='auto_nav_pipeline',
                name='auto_nav_pipeline_node',
                output='screen',
                shell=True,
                parameters=[file_path],
                emulate_tty=True
            ),
        ]

    return LaunchDescription([
        config_arg,
        OpaqueFunction(function=setup_commands)
    ])
