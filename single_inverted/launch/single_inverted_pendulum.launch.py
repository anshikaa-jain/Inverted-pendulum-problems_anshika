from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    package_dir = os.path.dirname(os.path.abspath(__file__))
    rviz_config_file = os.path.join(package_dir, '..', 'config', 'config.rviz')
    
    visualizer = Node(
        package = 'rviz2',
        executable = 'rviz2',
        # name='rviz2',
        arguments = ['-d', rviz_config_file]

    )

    dynamics = Node(
        package = 'single_inverted',
        executable = 'dynamics_sim'
    )

    interfacer = Node(
        package = 'single_inverted',
        executable = 'interface.py'
    )

    balancer = Node(
        package = 'single_inverted',
        executable = 'balance.py'
    )

    swing_up = Node(
        package = 'single_inverted',
        executable = 'swing_up.py'
    )

    return LaunchDescription([
        visualizer,
        dynamics,
        #interfacer,
        #balancer,
        swing_up
    ])
