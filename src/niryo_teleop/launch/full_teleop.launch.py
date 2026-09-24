"""Launch the complete two-robot teleoperation chain.

Nodes (each loads its own config section):
    master_joint_state_publisher  /master_raw/joint_states -> /master/joint_states
    teleoperation_controller      /master/joint_states -> /actor/joint_cmd
    actor_controller              /actor/joint_cmd -> Niryo FollowJointTrajectory
    actor_joint_state_publisher   /actor_raw/joint_states -> /actor/joint_states

The official niryo_ned_ros2_driver stays a separate dependency::

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
    teleop_config = PathJoinSubstitution(
        [FindPackageShare('niryo_teleop'), 'config', 'teleop_params.yaml']
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
            executable='teleoperation_controller',
            name='teleoperation_controller',
            parameters=[teleop_config],
            output='screen',
        ),
        Node(
            package='niryo_teleop',
            executable='actor_controller',
            name='actor_controller',
            parameters=[actor_config],
            output='screen',
        ),
        Node(
            package='niryo_teleop',
            executable='actor_joint_state_publisher',
            name='actor_joint_state_publisher',
            parameters=[actor_config],
            output='screen',
        ),
    ])
