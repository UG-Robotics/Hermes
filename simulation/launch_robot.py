import random
import xacro
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # ── WRO Randomised Start ──────────────────────────────────────────────────
    starts = [
        {'x':  0.0,  'y':  1.0,  'yaw':  0.00, 'dir': -1},   # Heads & Heads
        {'x':  0.0,  'y': -1.0,  'yaw':  3.14, 'dir':  1},  # Tails & Tails
        {'x': -1.0,  'y':  0.0,  'yaw':  1.57, 'dir': -1},   # Tails & Heads
        {'x':  1.0,  'y':  0.0,  'yaw': -1.57, 'dir':  1},  # Heads & Tails
    ]
    start = random.choice(starts)

    # ── Automatically Rebuild Xacro to URDF String ────────────────────────────
    xacro_file = '/root/wro_hermes_simul/src/robot.xacro'
    robot_description_xml = xacro.process_file(xacro_file).toxml()

    # ── 1. Spawn robot ────────────────────────────────────────────────────────
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_xml,  # Pass the compiled XML directly
            '-name', 'hermes_bot',
            '-x', str(start['x']),
            '-y', str(start['y']),
            '-z', '0.001',         # Keeps wheels exactly on the ground
            '-Y', str(start['yaw']),
        ],
        output='screen',
    )

    # ── 2. Gazebo ↔ ROS 2 bridge ──────────────────────────────────────────────
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tof_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tof_left@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tof_right@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    # ── 3. Autonomous controller ──────────────────────────────────────────────
    controller_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                executable='python3',
                arguments=[
                    '/root/wro_hermes_simul/src/autonomous_drive.py',
                    '--ros-args', 
                    '-p', f'turn_direction:={start["dir"]}'
                ],
                name='wro_controller',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        spawn_node,
        bridge_node,
        controller_node,
    ])
