# final_project_backup
''' ros2 topic pub --once /iiwa/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,0.5 ,0.5, 1.0]}" '''
ros2 topic pub --once /iiwa2/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,1.5 ,0.5, 1.0]}" 
ros2 topic pub --once /iiwa2/iiwa_arm_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,-0.5, 2.3 ,-1.8,0.5 ,1.5, 1.0]}" 
ros2 topic pub --once /iiwa/gripper/detach_package2 std_msgs/msg/Empty {}
ros2 launch ros2_kdl_package launching.launch.py ctrl:=vision

