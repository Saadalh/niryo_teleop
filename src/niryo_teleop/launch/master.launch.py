"""Launch the master joint-state publisher (Chunk 3).

Subscribes the Ned2 driver's /master_raw/joint_states and publishes measured
state on /master/joint_states. No hardware SDK in this node; input_topic is
the driver's namespaced topic via config/master_params.yaml
(currently /master_raw/joint_states).
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'master_params.yaml']
    )
    return LaunchDescription([
        Node(
            package='niryo_teleop',
            executable='master_joint_state_publisher',
            name='master_joint_state_publisher',
            parameters=[config],
            output='screen',
        ),
    ])
