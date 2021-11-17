import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    joystick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cabin_teleop'), 'launch'), '/joystick_start.py'])
    )

    thruster_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cabin_controllers'), 'launch'), '/thruster_controller_launch.py'])
    )

    pwm_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('cabin_controllers'), 'launch'), '/pwm_controller_launch.py'])
    )

    return LaunchDescription([
        joystick_node,
        thruster_controller_node,
        pwm_controller_node
    ])

