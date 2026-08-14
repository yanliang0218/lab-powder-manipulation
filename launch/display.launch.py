"""
Project: Lab Powder Manipulation
File: display.launch.py
"""


from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# ROS2 launch looks for this function automatically
# automatically called when running ros2 launch arm_v2_description display.launch.py

def generate_launch_description():
    #builds the urdf file path
    urdf_file = PathJoinSubstitution([
        FindPackageShare('arm_v2_description'),
        'urdf',
        'Arm_V2_edited_2.urdf'
    ])
    
    rviz_config = PathJoinSubstitution([
    FindPackageShare('arm_v2_description'),
    'rviz',
    'arm_v2_display.rviz'
])

    #create a variable called robot_description, this will contain the text content of the urdf file
    robot_description = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )
    
    #launching these by default
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[
      		  {'rate': 400}
    	    ],
            remappings=[
            	('/joint_states','/joint_states_gui')
            ],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        Node(
	    package='arm_v2_description',
	    executable='record_tf_path',
	    name='tf_path_recorder',
	    output='screen'
	    ),
      Node(
	    package='arm_v2_description',
	    executable='encoder_joint_state_bridge',
	    name='encoder_joint_state_bridge',
	    output='screen'
	    )
    ])
