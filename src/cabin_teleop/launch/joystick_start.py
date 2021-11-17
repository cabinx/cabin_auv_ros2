from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            namespace="cabin_auv",
            executable="joy_node",
            name="custom_joy_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"autorepeat_rate": 0.0}
            ]
        ),
        
        Node(
            package="cabin_teleop",
            namespace="cabin_auv",
            executable="t4_controller",
            name="t4_controller_node",
            output="screen",
            emulate_tty=True,
            parameters=[]
        )
    ])