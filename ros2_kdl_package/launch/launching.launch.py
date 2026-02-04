import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    config_params = PathJoinSubstitution([
        FindPackageShare('ros2_kdl_package'),
        'config',
        'kdl_params.yaml'
    ])

    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface', default_value='velocity',
        description='Select controller: position, velocity or effort'
    )
    ctrl_arg = DeclareLaunchArgument(
        'ctrl', default_value='vision',
        description='Select velocity controller'
    )

    # --- NODO ARUCO ---
    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_marker_publisher',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,
            'reference_frame': 'iiwa2_camera_link', 
            'camera_frame': 'iiwa2_camera_link',
            'use_sim_time': True ,
            'dictionary': 10,
        }],
        remappings=[
            ('/image', '/iiwa2_camera'),
            ('/camera_info', '/camera_info')
        ],
        output='screen'
    )
    
    # --- NODO KDL per IIWA 1 ---
    # Namespace: "iiwa"
    # Topic risultanti: /iiwa/joint_states, /iiwa/iiwa_arm_controller/joint_trajectory
    ros2_kdl_node_iiwa1 = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa',
        output='screen',
        parameters=[
            config_params,
            {
                'use_sim_time': True, 
                'cmd_interface': 'velocity', 
                'ctrl': 'vision',
                'robot_base_link': 'iiwa_link_0', 
                'robot_ee_link': 'iiwa_tool0'
            }
        ]
        # Nota: i remapping non servono più perché il codice C++ ora usa nomi relativi!
    )

    # --- NODO KDL per IIWA 2 ---
    # Namespace: "iiwa2"
    # Topic risultanti: /iiwa2/joint_states, /iiwa2/iiwa_arm_controller/joint_trajectory
    ros2_kdl_node_iiwa2 = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        namespace='iiwa2',
        output='screen',
        parameters=[
            config_params,
            {
                'use_sim_time': True, 
                'cmd_interface': 'velocity', 
                'ctrl': 'vision',
                'robot_base_link': 'iiwa2_link_0', 
                'robot_ee_link': 'iiwa2_tool0'
            }
        ]
    )

    return LaunchDescription([
        cmd_interface_arg,
        ctrl_arg,
        aruco_marker_publisher,
        ros2_kdl_node_iiwa1,
        ros2_kdl_node_iiwa2
    ])