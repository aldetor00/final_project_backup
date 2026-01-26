# final_project_backup
''' ros2 topic pub --once /iiwa/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,0.5 ,0.5, 1.0]}" '''
ros2 topic pub --once /iiwa2/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,1.5 ,0.5, 1.0]}" 
ros2 topic pub --once /iiwa2/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,0.5 ,1.5, 1.0]}" 
ros2 topic pub --once /iiwa/gripper/detach_package2 std_msgs/msg/Empty {}
ros2 launch ros2_kdl_package launching.launch.py ctrl:=vision
ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [0.5, 0.5, 0.0, -1.0, 0.0, 0.5, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
ros2 topic pub --once /iiwa2/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa2_joint_a1', 'iiwa2_joint_a2', 'iiwa2_joint_a3', 
    'iiwa2_joint_a4', 'iiwa2_joint_a5', 'iiwa2_joint_a6', 'iiwa2_joint_a7'
  ],
  points: [
    {
      positions: [-3.0, 1.0, 0.0, -1.0, 0.0, 0.5, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
 [1.57, -0.45, 0.0, 1.5, 0.0, -0.90, 0.0]
       positions: [-3.0, 1.0, 0.0, -1.0, 0.0, 0.5, 0.0],
      positions: [-3.0, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0],
      positions: [1.5, 0.0, 0.0, -1.5, 0.0, 0.5, 0.0],
      positions: [1.5, 1.5, 0.0, -1.0, 0.0, 0.5, 0.0],
ros2 launch ros2_kdl_package launching.launch.py
ros2 run ros2_kdl_package dual_marker_client

pick pacco 1 iiwa 1: ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [-2.8, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
pick pacco 2 iiwa 1: ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [2.8, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
place package su fra2mo iiwa: ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [0.0, 0.5, 0.0, -1.45, 0.0, 1.0, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"
ros2 topic pub --once /iiwa/iiwa_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'iiwa_joint_a1', 'iiwa_joint_a2', 'iiwa_joint_a3', 
    'iiwa_joint_a4', 'iiwa_joint_a5', 'iiwa_joint_a6', 'iiwa_joint_a7'
  ],
  points: [
    {
      positions: [0.0, 0.5, 0.0, -1.55, 0.0, 1.0, 0.0],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}"


