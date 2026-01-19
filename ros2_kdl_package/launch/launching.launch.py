import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Percorso del file dei parametri
    config_params = PathJoinSubstitution([
        FindPackageShare('ros2_kdl_package'),
        'config',
        'kdl_params.yaml'
    ])

    # --- ARGOMENTI ---
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface', 
        default_value='velocity',
        description='Select controller: position, velocity or effort'
    )
    
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', 
        default_value='velocity_ctrl_null',
        description='Select velocity controller: velocity_ctrl, velocity_ctrl_null or vision'
    )

    # --- NODO ARUCO DETECTION ---
    # I topic della camera sono già bridgati da gazebo_fra2mo.launch.py
    # Dobbiamo solo rimappare correttamente
    aruco_node = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,         # Dimensione del tag in metri (5cm)
            'marker_id': 1,              # ID del tag ArUco
            'reference_frame': 'iiwa2_camera_link', 
            'camera_frame': 'iiwa2_camera_link',
            'marker_frame': 'aruco_marker_frame',
            'use_sim_time': True
        }],
        remappings=[
            ('/image', '/iiwa2_camera'),
            ('/camera_info', '/camera_info')
        ],
        output='screen'
    )

    # --- NODO KDL (IL TUO CODICE C++) ---
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa2',
        output='screen',
        parameters=[
            config_params,
            {'use_sim_time': True, 'cmd_interface': 'velocity', 'ctrl': 'vision'}
        ],
        remappings=[
            ('joint_states', '/iiwa2/joint_states'),
            ('velocity_controller/commands', '/iiwa2/velocity_controller/commands'),
            ('/aruco_single/pose', '/aruco_single/pose')
        ]
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        aruco_node,
        ros2_kdl_node
    ])