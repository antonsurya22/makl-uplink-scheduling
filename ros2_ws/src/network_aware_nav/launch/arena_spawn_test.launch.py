import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gui = LaunchConfiguration('gui')

    pkg_naw = get_package_share_directory('network_aware_nav')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_tb3_desc = get_package_share_directory('turtlebot3_description')

    world_file = os.path.join(pkg_naw, 'worlds', 'arena_lab.world')

    burger_sdf = os.path.join(
        pkg_naw,
        'worlds',
        'turtlebot3_burger_contact.sdf'
    )

    burger_urdf = os.path.join(
        pkg_tb3_desc,
        'urdf',
        'turtlebot3_burger.urdf'
    )

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui)
    )

    # Fix TurtleBot3 URDF: remove unresolved ${namespace}
    make_fixed_urdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f"sed 's/${{namespace}}//g' {burger_urdf} > /tmp/tb3_burger_fixed.urdf"
        ],
        output='screen'
    )

    robot_state_publisher = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                    '/tmp/tb3_burger_fixed.urdf',
                    '--ros-args', '-p', 'use_sim_time:=true'
                ],
                output='screen'
            )
        ]
    )

    spawn_burger = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'burger',
                    '-file', burger_sdf,
                    '-x', '-1.05',
                    '-y', '-0.92',
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        gzserver,
        gzclient,
        make_fixed_urdf,
        robot_state_publisher,
        spawn_burger,
    ])
