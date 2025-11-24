from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic'
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera_info',
        description='Camera info topic'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mono',
        description='VSLAM mode: mono, stereo, or rgbd'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transforms'
    )

    world_frame_arg = DeclareLaunchArgument(
        'world_frame',
        default_value='world',
        description='World/map frame name'
    )

    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera',
        description='Camera frame name'
    )

    # VSLAM node
    vslam_node = Node(
        package='pycuvslam_ros2',
        executable='vslam_node',
        name='pycuvslam_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'mode': LaunchConfiguration('mode'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'world_frame': LaunchConfiguration('world_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
        }]
    )

    return LaunchDescription([
        camera_topic_arg,
        camera_info_topic_arg,
        mode_arg,
        publish_tf_arg,
        world_frame_arg,
        camera_frame_arg,
        vslam_node,
    ])
