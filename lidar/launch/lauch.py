import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar',
            executable='my_node',
            name='my_node',
            output='screen',
            emulate_tty=True,
            parameters=[],
            remappings=[
                ('/scan', '/your/lidar/topic'),  # Hier Pfad zum Lidar-Topic anpassen
            ],
        ),
    ])