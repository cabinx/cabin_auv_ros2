from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    property_file = os.path.join(get_package_share_directory('cabin_controllers'), 'cfg', 'bluerov_heavy_properties.yaml')
    pwm_file = get_package_share_directory('cabin_controllers') + "/cfg/thruster_force_vs_pwm.yaml"

    return LaunchDescription([
        Node(
            package="cabin_controllers",
            namespace="cabin_auv",
            executable="pwm_controller",
            name="pwm_controller_",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"properties_file": property_file},
                {"pwm_file" : pwm_file}
            ]
        )
    ])