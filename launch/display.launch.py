#!/usr/bin/env python3

"""
Project: Lab Powder Manipulation
File: display.launch.py
Authors: Lang Yun, Liang Yan

Unified online acquisition + visualization launch:

    Nucleo -> encoder bridge -> /joint_angles
                              -> /joint_states
                                   |-> event-driven KDL FK -> /FK_pose
                                   `-> robot_state_publisher -> TF -> RViz

    usb_cam -> /webcam/image_raw
            -> /webcam/image_raw/compressed

    /FK_pose
    /joint_angles
    /webcam/image_raw/compressed
            -> sensor_sync -> /synced_msg

Before recording:
    A lightweight AprilTag count monitor starts immediately and keeps running
    until the launch is shut down.

Recording:
    rosbag and text recorders begin after the same 15-second setup interval.
    AprilTag pose estimation and hand-eye calibration remain offline.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from datetime import datetime
import os


def generate_launch_description():
    """Start the full robot + camera acquisition and visualization pipeline."""

    # --------------------------------------------------------------------------
    # User-adjustable launch arguments
    # --------------------------------------------------------------------------
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    video_device = LaunchConfiguration("video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    pixel_format = LaunchConfiguration("pixel_format")

    recording_directory = os.getcwd()

    # Give the user time to position the camera and confirm at least one tag.
    recording_delay_sec = 15.0

    run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_output_name = f"arm_dataset_{run_timestamp}"

    # --------------------------------------------------------------------------
    # Robot description and RViz config
    # --------------------------------------------------------------------------
    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("arm_v2_description"),
            "urdf",
            "Arm_V2_edited_2.urdf",
        ]
    )

    robot_description = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str,
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("arm_v2_description"),
            "rviz",
            "arm_v2_display.rviz",
        ]
    )

    # --------------------------------------------------------------------------
    # Camera
    # usb_cam owns the physical camera and publishes under /webcam.
    # --------------------------------------------------------------------------
    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="webcam",
        name="usb_cam",
        output="screen",
        parameters=[
            {
                "video_device": video_device,
                "io_method": "mmap",
                "pixel_format": pixel_format,
                "image_width": ParameterValue(
                    image_width,
                    value_type=int,
                ),
                "image_height": ParameterValue(
                    image_height,
                    value_type=int,
                ),
                "framerate": ParameterValue(
                    framerate,
                    value_type=float,
                ),
                "camera_name": "webcam",
                "frame_id": "webcam_frame",
            }
        ],
    )

    # --------------------------------------------------------------------------
    # Robot-side online nodes
    # --------------------------------------------------------------------------
    encoder_bridge = Node(
        package="arm_v2_description",
        executable="encoder_joint_state_bridge",
        name="encoder_joint_state_bridge",
        parameters=[
            {
                "serial_port": serial_port,
                "baud_rate": ParameterValue(
                    baud_rate,
                    value_type=int,
                ),
                # Nucleo line:
                # 1 value | 2 value | 3 value | ... | 6 value
                "angle_extraction_identifiers": [
                    "1 ",
                    "| 2 ",
                    "| 3 ",
                    "| 4 ",
                    "| 5 ",
                    "| 6 ",
                ],
            }
        ],
        output="screen",
    )

    # Every /joint_states callback directly runs KDL FK and publishes one
    # /FK_pose with the exact source JointState timestamp.
    fk_publisher = Node(
        package="arm_v2_description",
        executable="record_tf_path",
        name="joint_state_fk_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "base_frame": "base_link",
                "tip_frame": "tip_link",
                "joint_states_topic": "/joint_states",
                "fk_pose_topic": "/FK_pose",
            }
        ],
        output="screen",
    )

    # robot_state_publisher is used for TF and RViz.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Sensor synchronization
    # Each unique compressed image is packaged with the latest FK pose and
    # optional joint angles. AprilTag pose estimation remains offline.
    # --------------------------------------------------------------------------
    sensor_sync = Node(
        package="arm_v2_description",
        executable="sensor_sync",
        name="sensor_sync",
        output="screen",
        parameters=[
            {
                "frame": "base_link",
                "fk_topic": "/FK_pose",
                "joint_angles_topic": "/joint_angles",
                "image_topic": "/webcam/image_raw/compressed",
                "synced_topic": "/synced_msg",
            }
        ],
    )
    # --------------------------------------------------------------------------
    # Lightweight AprilTag count monitor; runs continuously until shutdown.
    # --------------------------------------------------------------------------

    apriltag_count_monitor = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "arm_v2_description",
            "apriltag_count_monitor",
            "--ros-args",
            "-p",
            "image_topic:=/webcam/image_raw",
            "-p",
            "tag_family:=tag25h9",
        ],
        output="screen",
    )
    # --------------------------------------------------------------------------
    # Recorders
    # --------------------------------------------------------------------------
    bag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_output_name,
            "/synced_msg",
            "/FK_pose",
            "/joint_angles",
            "/webcam/image_raw/compressed",
        ],
        cwd=recording_directory,
        output="screen",
    )

    synced_msg_text_recorder = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            until ros2 topic type /synced_msg >/dev/null 2>&1; do
                sleep 0.5
            done

            exec ros2 topic echo /synced_msg > synced_msg.txt
            """,
        ],
        cwd=recording_directory,
        output="screen",
    )

    fk_pose_text_recorder = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            until ros2 topic type /FK_pose >/dev/null 2>&1; do
                sleep 0.5
            done

            exec ros2 topic echo /FK_pose > FK_pose.txt
            """,
        ],
        cwd=recording_directory,
        output="screen",
    )

    joint_angles_text_recorder = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            until ros2 topic type /joint_angles >/dev/null 2>&1; do
                sleep 0.5
            done

            exec ros2 topic echo /joint_angles > joint_angles.txt
            """,
        ],
        cwd=recording_directory,
        output="screen",
    )

    image_raw_text_recorder = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            """
            until ros2 topic type /webcam/image_raw/compressed >/dev/null 2>&1; do
                sleep 0.5
            done

            exec ros2 topic echo /webcam/image_raw/compressed > image_raw.txt
            """,
        ],
        cwd=recording_directory,
        output="screen",
    )

    # Start all recording outputs after exactly the same 15-second setup period
    # so the rosbag and text files cover the same acquisition window.
    delayed_recorders = TimerAction(
        period=recording_delay_sec,
        actions=[
            LogInfo(
                msg=(
                    "15-second AprilTag setup period complete. "
                    "Starting rosbag and text recording now."
                )
            ),
            bag_recorder,
            synced_msg_text_recorder,
            fk_pose_text_recorder,
            joint_angles_text_recorder,
            image_raw_text_recorder,
        ],
    )

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Nucleo virtual serial device",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="460800",
                description="Must match Serial.begin() in the Nucleo firmware",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="Linux V4L2 camera device",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="1280",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="720",
            ),
            DeclareLaunchArgument(
                "framerate",
                default_value="150.0",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="mjpeg2rgb",
                description=(
                    "Must be supported by both the camera and usb_cam. "
                    "Run usb_cam with pixel_format:=test to list driver formats."
                ),
            ),

            LogInfo(
                msg=(
                    "Camera and robot pipeline starting now. "
                    "Rosbag recording will begin in 15 seconds. "
                    "Use the AprilTag count in this terminal to position the camera."
                )
            ),

            # Start the online sources immediately.
            camera,
            encoder_bridge,
            fk_publisher,
            robot_state_publisher,
            rviz,
            sensor_sync,

            # Continuously report the number of visible AprilTags.
            apriltag_count_monitor,

            # Start all recording after the setup interval.
            delayed_recorders,
        ]
    )