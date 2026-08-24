import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur_moveit_config").to_moveit_configs()

    # Servo configuratieparameters
    servo_params = {
        "moveit_servo": {
            "use_gazebo": False,
            "status_topic": "~/status",
            "publish_joint_positions": True,
            "publish_joint_velocities": False,
            "publish_joint_accelerations": False,
            "command_out_type": "trajectory_msgs/JointTrajectory",
            "command_out_topic": "/joint_trajectory_controller/joint_trajectory",
            "incoming_command_topic": "/servo_node/delta_twist_cmds",
            "robot_link_command_frame": "base_link",
            "planning_frame": "base_link",
            "ee_frame_name": "tool0",
            "cartesian_command_in_topic": "~/delta_twist_cmds",
            "joint_command_in_topic": "~/delta_joint_cmds",
            "joint_topic": "/joint_states",
            "linear_scale": 1.0,
            "rotational_scale": 1.0,
            "joint_scale": 1.0,
            "check_collisions": True,
            "collision_check_rate": 10.0,
            "collision_distance_safety_factor": 1.2,
            "min_units_per_step": 0.001,
            "max_units_per_step": 0.05,
            "scale": {
                "linear": 0.4,
                "rotational": 0.8,
                "joint": 0.5
            }
        }
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
