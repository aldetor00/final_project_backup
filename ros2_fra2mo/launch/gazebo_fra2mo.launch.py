import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import  PathJoinSubstitution
from launch.actions import  ExecuteProcess, TimerAction
from launch.event_handlers import  OnProcessStart
# ... il resto dei tuoi import

def generate_launch_description():
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_iiwa = get_package_share_directory('iiwa_description')
    
    xacro_fra2mo = os.path.join(pkg_fra2mo, "urdf", "fra2mo.urdf.xacro")
    xacro_iiwa1 = os.path.join(pkg_iiwa, "urdf", "iiwa1.urdf.xacro")
    xacro_iiwa2 = os.path.join(pkg_iiwa, "urdf", "iiwa2.urdf.xacro")
    world_file = os.path.join(pkg_fra2mo, "worlds", "project.sdf")

    # --- Funzione per estrarre le posizioni iniziali ---
    def load_yaml_initial_pos(file_name, prefix):
        path = os.path.join(pkg_iiwa, 'config', file_name)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        pos_dict = data['initial_positions']
        names = [f"{prefix}{k}" for k in pos_dict.keys()]
        values = [str(v) for v in pos_dict.values()]
        return names, values

    iiwa1_names, iiwa1_values = load_yaml_initial_pos('iiwa1_initial_positions.yaml', 'iiwa_')
    iiwa2_names, iiwa2_values = load_yaml_initial_pos('iiwa2_initial_positions.yaml', 'iiwa2_')

    # --- Robot State Publishers ---
    rsp_fra2mo = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='fra2mo_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_fra2mo]), value_type=str), 
            'use_sim_time': True
        }]
    )

    rsp_iiwa1 = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        namespace='iiwa',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_iiwa1]), value_type=str), 
            'use_sim_time': True
        }]
    )

    rsp_iiwa2 = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher', 
        namespace='iiwa2',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_iiwa2]), value_type=str), 
            'use_sim_time': True
        }]
    )

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 
                'launch', 
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r']
        }.items()
    )

    # --- Spawn Robots ---
    spawn_fra2mo = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'fra2mo', 
            '-z', '0.1'
        ]
    )

    spawn_iiwa1 = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', '/iiwa/robot_description', 
            '-name', 'iiwa1', 
            '-x', '-4.6', '-y', '0.25', '-z', '0.1',
            '-joint-names'] + iiwa1_names + [
            '-joint-positions'] + iiwa1_values,
        output='screen'
    )

    spawn_iiwa2 = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', '/iiwa2/robot_description', 
            '-name', 'iiwa2', 
            '-x', '9', '-y', '3.1', '-z', '0.1',
            '-joint-names'] + iiwa2_names + [
            '-joint-positions'] + iiwa2_values,
        output='screen'
    )

    # --- Controller Spawners ---
    jsb_iiwa1 = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "-c", "/iiwa/controller_manager"]
    )
    arm_iiwa1 = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["iiwa_arm_controller", "-c", "/iiwa/controller_manager"]
    )
    
    jsb_iiwa2 = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "-c", "/iiwa2/controller_manager"]
    )
    
    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='log',
        arguments=['velocity_controller', '-c', '/iiwa2/controller_manager'],
    )
    # --- Event Handlers ---
    load_iiwa1_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa1, 
            on_exit=[jsb_iiwa1, arm_iiwa1]
        )
    )

    load_iiwa2_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_iiwa2, 
            on_exit=[jsb_iiwa2, #arm_iiwa2,
                     velocity_controller]
        )
    )
    

    # --- Bridge ROS-Gazebo ---
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/fra2mo/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/iiwa2_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/iiwa/gripper/attach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/gripper/detach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/gripper/state_package@std_msgs/msg/Bool@ignition.msgs.Boolean', # <--- AGGIUNTA VIRGOLA
            '/iiwa/gripper/attach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/gripper/detach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa/gripper/state_package2@std_msgs/msg/Bool@ignition.msgs.Boolean', # <--- AGGIUNTA VIRGOLA
            '/iiwa2/gripper/attach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa2/gripper/detach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa2/gripper/state_package@std_msgs/msg/Bool@ignition.msgs.Boolean', # <--- AGGIUNTA VIRGOLA
            '/iiwa2/gripper/attach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa2/gripper/detach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
            '/iiwa2/gripper/state_package2@std_msgs/msg/Bool@ignition.msgs.Boolean'
        ],
        output='screen'
    )

    # --- ArUco Detector ---
    aruco_detector = Node(
        package='aruco_ros',
        executable='marker_publisher',
        name='aruco_detector',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.05,  # 5cm - MODIFICA IN BASE AI TUOI MARKER
            'reference_frame': 'iiwa2_camera_link',
            'camera_frame': 'iiwa2_camera_link',
            'use_sim_time': True,
        }],
        remappings=[
            ('/camera/image_raw', '/iiwa2_camera/image'),
            ('/camera/camera_info', '/iiwa2_camera/camera_info'),
        ],
        output='screen'
    )
    
    detach_package1 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/iiwa/gripper/detach_package', 'std_msgs/msg/Empty', '{}'],
        output='screen'
    )
    detach_package2 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/iiwa/gripper/detach_package2', 'std_msgs/msg/Empty', '{}'],
        output='screen'
    )
    detach_package3 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/iiwa2/gripper/detach_package', 'std_msgs/msg/Empty', '{}'],
        output='screen'
    )
    detach_package4 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/iiwa2/gripper/detach_package2', 'std_msgs/msg/Empty', '{}'],
        output='screen'
    )
    # Handler: Quando il bridge parte, aspetta 5 secondi e poi invia il detach
    detach_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=bridge,
            on_start=[
                TimerAction(
                    period=10.0,  # Tempo per garantire che il robot sia spawnato
                    actions=[detach_package1, detach_package2, detach_package3, detach_package4]
                )
            ]
        )
    )
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH", 
            value=os.path.join(pkg_fra2mo, 'models') + ':' + 
                  pkg_iiwa + ':' + 
                  os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        rsp_fra2mo, 
        rsp_iiwa1, 
        rsp_iiwa2,
        gazebo,
        spawn_fra2mo,
        spawn_iiwa1,
        spawn_iiwa2,
        load_iiwa1_controllers,
        load_iiwa2_controllers,
        #velocity_controller,
        bridge,
        detach_handler,
        #aruco_detector  # ArUco detector integrato
    ])