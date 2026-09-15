"""Launch master publisher + teleoperation controller (Chunk 4 chain).

master_joint_state_publisher republished /master/joint_states feeds
teleoperation_controller (/master/joint_states -> /actor/joint_cmd).
Each node loads its own config file.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    master_config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'master_params.yaml']
    )
    teleop_config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'teleop_params.yaml']
    )
    return LaunchDescription([
        Node(
            package='niryo_teleop',
            executable='master_joint_state_publisher',
            name='master_joint_state_publisher',
            parameters=[master_config],
            output='screen',
        ),
        Node(
            package='niryo_teleop',
            executable='teleoperation_controller',
            name='teleoperation_controller',
            parameters=[teleop_config],
            output='screen',
        ),
    ])
