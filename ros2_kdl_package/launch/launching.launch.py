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
        default_value='vision',
        description='Select velocity controller: velocity_ctrl, velocity_ctrl_null or vision'
    )

    # --- NODO ARUCO MARKER PUBLISHER (Rileva TUTTI i tag) ---
    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,         # Dimensione 5cm
            'reference_frame': 'iiwa2_camera_link', 
            'camera_frame': 'iiwa2_camera_link',
            'use_sim_time': True ,
            'dictionary': 10,  # AGGIUNGI: 0=DICT_4X4_50, 10=DICT_ARUCO_ORIGINAL, 16=DICT_6X6_250
        }],
        remappings=[
            ('/image', '/iiwa2_camera'),
            ('/camera_info', '/camera_info')
        ],
        output='screen'
    )
    
    # --- NODO KDL ---
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
            ('velocity_controller/commands', '/iiwa2/velocity_controller/commands')
            # Nota: il remapping della posa non serve più perché useremo le TF
        ]
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        aruco_marker_publisher,

        ros2_kdl_node
    ])