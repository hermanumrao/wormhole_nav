import os
import yaml
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # Load config file
    home = os.environ['HOME']
    config_file = os.path.join(
        home,
        'ros2_ws/src/auto_nav_pipeline/config/auto_nav_config.yaml'  # relative to $HOME
    )

    with open(config_file, 'r') as f:
        cfg = yaml.safe_load(f)

    # Drill down into the new format
    params = cfg['auto_nav_pipeline_node']['ros__parameters']
    spawn_pose = params['spawn_pose']
    goal_pose = params['goal_pose']
    map_file = params['map']
    world_launch_file = params['world_launch_file']

    # Build commands
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
    
    print("[DEBUG] nav2_cmd:", nav2_cmd)

    auto_localize_cmd = [
        'ros2', 'run', 'simple_navigation', 'auto_localize_and_nav'
    ]

    return LaunchDescription([
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
                ExecuteProcess(cmd=auto_localize_cmd, output='log', shell=True)
            ]
        ),

        # Step 4: Start auto_nav_pipeline C++ node (keep on screen for focus)
        LogInfo(msg="[AUTO_NAV_PIPELINE] Starting auto_nav_pipeline C++ node..."),
        Node(
            package='auto_nav_pipeline',
            executable='auto_nav_pipeline',
            name='auto_nav_pipeline_node',
            output='screen',  # <-- only this stays on terminal
            parameters=[config_file],  # still pass whole file to node
            emulate_tty=True
        ),
    ])
