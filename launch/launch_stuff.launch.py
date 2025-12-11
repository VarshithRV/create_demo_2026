from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os


def launch_setup(context, *args, **kwargs):
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    left_ur_type = LaunchConfiguration("left_ur_type")
    left_safety_limits = LaunchConfiguration("left_safety_limits")
    left_safety_pos_margin = LaunchConfiguration("left_safety_pos_margin")
    left_safety_k_position = LaunchConfiguration("left_safety_k_position")
    left_kinematics_params_file = LaunchConfiguration("left_kinematics_params_file")
    
    left_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "joint_limits.yaml"])
    left_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "physical_parameters.yaml"])
    left_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", left_ur_type, "visual_parameters.yaml"])

    right_ur_type = LaunchConfiguration("right_ur_type")
    right_safety_limits = LaunchConfiguration("right_safety_limits")
    right_safety_pos_margin = LaunchConfiguration("right_safety_pos_margin")
    right_safety_k_position = LaunchConfiguration("right_safety_k_position")
    right_kinematics_params_file = LaunchConfiguration("right_kinematics_params_file")
    
    right_joint_limit_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "joint_limits.yaml"])
    right_physical_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "physical_parameters.yaml"])
    right_visual_params = PathJoinSubstitution([FindPackageShare(description_package), "config", right_ur_type, "visual_parameters.yaml"])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "name:=dual_arm_workcell",
            " ",
            "left_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "left_tf_prefix:=left_",
            " ",
            "left_joint_limit_params:=",
            left_joint_limit_params,
            " ",
            "left_kinematics_params:=",
            left_kinematics_params_file,
            " ",
            "left_physical_params:=",
            left_physical_params,
            " ",
            "left_visual_params:=",
            left_visual_params,
            " ",
            "left_safety_limits:=",
            left_safety_limits,
            " ",
            "left_safety_pos_margin:=",
            left_safety_pos_margin,
            " ",
            "left_safety_k_position:=",
            left_safety_k_position,
            " ",
            "left_name:=",
            "left_ur16e",
            " ",
            "left_script_filename:=",
            "ros_control.urscript",
            " ",
            "left_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "left_output_recipe_filename:=",
            "rtde_output_recipe.txt",
            " ",
            "right_robot_ip:=xxx.yyy.zzz.www",
            " ",
            "right_tf_prefix:=right_",
            " ",
            "right_joint_limit_params:=",
            right_joint_limit_params,
            " ",
            "right_kinematics_params:=",
            right_kinematics_params_file,
            " ",
            "right_physical_params:=",
            right_physical_params,
            " ",
            "right_visual_params:=",
            right_visual_params,
            " ",
            "right_safety_limits:=",
            right_safety_limits,
            " ",
            "right_safety_pos_margin:=",
            right_safety_pos_margin,
            " ",
            "right_safety_k_position:=",
            right_safety_k_position,
            " ",
            "right_name:=",
            "right_ur16e",
            " ",
            "right_script_filename:=",
            "ros_control.urscript",
            " ",
            "right_input_recipe_filename:=",
            "rtde_input_recipe.txt",
            " ",
            "right_output_recipe_filename:=",
            "rtde_output_recipe.txt",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # robot_description and param moveit param initialization is finished, now moveit config preparation needs to be done
    # moveit config stuff
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "config", moveit_config_file]
            )
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(value=robot_description_semantic_content,value_type=str)
    }

    robot_description_kinematics = {
    "robot_description_kinematics": {
        "left_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        },
        "right_ur16e": {
            "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            "kinematics_solver_attempts": 3,
            "kinematics_solver_search_resolution": 0.005,
            "kinematics_solver_timeout": 0.005,
        }
    }}

    #### nodes

    left_pose_tracking_node = Node(
        package="motion_planning_abstractions",
        executable="pose_tracker",
        name="left_pose_tracker",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": use_sim_time},
            {
                "planning_group": "left_ur16e",
                "endeffector_link": "left_tool0",
                "servo_controller": "left_forward_position_controller",
                "non_servo_controller": "left_scaled_joint_trajectory_controller",
                "servo_node_namespace": "left_servo_node_main",
                "P_GAIN": 1.0,
                "I_GAIN": 0.0,
                "D_GAIN": 0.0,
                "K_GAIN": 1.0,
                "terminate":False, # should the tracking terminate?
                "linear_stop_threshold": 0.01, #m
                "angular_stop_threshold": 0.01, #rad
                "planning_frame":"world",
                "linear_iir_alpha":0.9, # range [0.0,1.0], more implies filter more
                "angular_iir_alpha":0.5, # range [0.0,1.0], more implies filter more
            },
        ]
    )

    left_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="left_preaction_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "left_ur16e",
                "shoulder_pan": -0.2518866697894495,
                "shoulder_lift": -1.2164602738669892,
                "elbow": -2.014770746231079,
                "wrist_1": -3.0570813618102015,
                "wrist_2": -2.2569201628314417,
                "wrist_3": 0.0768733024597168,
                "side": "left",
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    right_preaction_server = Node(
        package="motion_planning_abstractions",
        executable="predefined_state_server",
        name="right_preaction_server",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                "planning_group": "right_ur16e",
                "shoulder_pan": -3.2785912195788782,
                "shoulder_lift": -1.6929518185057582,
                "elbow": 2.1487663427936,
                "wrist_1": -2.3819247684874476,
                "wrist_2": -2.7175918261157435,
                "wrist_3": 1.8448426723480225,
                "side": "right",
            },
            {"use_sim_time": use_sim_time},
        ],
    )


    apriltag_grid_detector = Node(
        package="apriltag_grid_detector",
        executable="apriltag_grid_detector",
        name="apriltag_grid_detector",
        output="screen",
        parameters=[
            {
                "alpha": 0.25,
                "marker_separation": 4.0,  # mm
                "marker_size": 40.0,        # mm
                "object.name": "object0",
                # 2 rows x 2 cols, flattened [row0, row1, ...]
                "grid.rows": 2,
                "grid.cols": 2,
                "grid.ids": [9, 10, 11, 12],
                "color_image_topic": "/camera/right_camera/color/image_raw",
                "camera_info_topic": "/camera/right_camera/color/camera_info",
                "depth_image_topic": "/camera/right_camera/depth/image_rect_raw",
                "detection_rate": 30.0,
            }
        ],
    )

    nodes_to_start = [
        left_pose_tracking_node,
        left_preaction_server,
        right_preaction_server,
        apriltag_grid_detector,
    ]
    
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []

    # general arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="dual_arm_workcell_moveit_config",
            description="dual_arm_workcell_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="dual_arm_workcell.srdf",
            description="MoveIt SRDF file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="dual_arm_workcell_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="dual_arm_workcell.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    # left robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("left_ur_type"),
                    "left_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual left robot used.",
        )
    )

    # right robot arguments (used in launch_setup)
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
            default_value="ur16e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_kinematics_params_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "config",
                    LaunchConfiguration("right_ur_type"),
                    "right_default_kinematics.yaml",
                ]
            ),
            description="The calibration configuration of the actual right robot used.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])