"""Runtime nodes for dataset collection.

Starts the relays and the actor controller, but keeps the actor controller
gated (``auto_start: false``) so calibration and bench preparation cannot move
the arm. The collector enables it over ``/actor_controller/enable``.

The official ``niryo_ned_ros2_driver`` is a separate process:

    ros2 launch niryo_ned_ros2_driver driver.launch.py \
        drivers_list_file:=<dual_robot_config>
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    master_config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'master_params.yaml']
    )
    actor_config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'actor_params.yaml']
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
            executable='actor_joint_state_publisher',
            name='actor_joint_state_publisher',
            parameters=[actor_config],
            output='screen',
        ),
        Node(
            package='niryo_teleop',
            executable='actor_controller',
            name='actor_controller',
            parameters=[actor_config, {'auto_start': False}],
            output='screen',
        ),
    ])
