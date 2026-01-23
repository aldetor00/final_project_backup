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

    # --- NODO ARUCO MARKER PUBLISHER (Rileva TUTTI i tag) ---
    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,
            'reference_frame': 'iiwa2_camera_link',
            'camera_frame': 'iiwa2_camera_link',
            'use_sim_time': True,
            'dictionary': 0,  # DICT_4X4_50
        }],
        remappings=[
            ('/image', '/iiwa2_camera'),
            ('/camera_info', '/camera_info')
        ],
        output='screen'
    )
    
    # --- NODO KDL SERVER ---
    ros2_kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa2',
        output='screen',
        parameters=[
            config_params,
            {
                'use_sim_time': True,
                'traj_duration': 4.0,
                'Kp': 10.0
            }
        ],
        remappings=[
            ('joint_states', '/iiwa2/joint_states'),
        ]
    )

    # --- STATIC TF PER CAMERA (se necessario) ---
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', '0.07',
            '--y', '0.0',
            '--z', '-0.03',
            '--roll', '3.14',
            '--pitch', '-1.57',
            '--yaw', '0.0',
            '--frame-id', 'iiwa2_tool0',
            '--child-frame-id', 'iiwa2/iiwa2_link_7/iiwa2_camera'
        ],
        output='screen'
    )

    return LaunchDescription([
        aruco_marker_publisher,
        ros2_kdl_node,
        static_tf_node
    ])