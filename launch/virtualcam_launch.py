from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cam_width', default_value='324'),
        DeclareLaunchArgument('cam_height', default_value='244'),
        DeclareLaunchArgument('cam_fps', default_value='20'),
        DeclareLaunchArgument('switch_rb', default_value='True'),
        Node(
            package='ros2_virtualcam',
            executable='virtualcam_node',
            name='virtualcam_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'cam_width': LaunchConfiguration('cam_width'),
                'cam_height': LaunchConfiguration('cam_height'),
                'cam_fps': LaunchConfiguration('cam_fps'),
                'switch_rb': LaunchConfiguration('switch_rb'),
                }
            ]
        )
    ])