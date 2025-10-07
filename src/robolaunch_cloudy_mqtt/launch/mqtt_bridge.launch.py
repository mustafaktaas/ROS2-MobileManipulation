from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robolaunch_cloudy_mqtt',
            executable='mqtt_bridge',
            name='mqtt_bridge',
            output='screen'
        )
    ])