#!/usr/bin/env python3
# Waypoint follower for fra2mo robot - Modified version with repeated cycle
# Full coordinates with position (x,y,z) and orientation (x,y,z,w) quaternions

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time

# ============================================
# WAYPOINT DEFINITIONS
# ============================================
# Modify these values to change waypoints
# Orientation is in quaternion format (x, y, z, w)

waypoints_forward = [
    {
        "position": {"x": -3.32, "y": 0.12, "z": 0.10},
        "orientation": {"x": 0.0, "y": 0.0, "z": -0.0948, "w": 0.9955}
    },
    {
        "position": {"x": 3.2, "y": 2.3, "z": 0.10},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
        "position": {"x": 6.8, "y": 2.3, "z": 0.10},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
        "position": {"x": 6.8, "y": 3.0, "z": 0.10},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    },
    {
        "position": {"x": 7.10, "y": 2.85,  "z": 0.10},
        "orientation": {"x": 0.0, "y": 0.0, "z": 1.0,  "w":0.0}
    }
]

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    def create_pose(transform):
        """Create a PoseStamped from position and orientation dictionary"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose
    
    def navigate_waypoints(waypoints_list, description):
        """Navigate through a list of waypoints using followWaypoints"""
        print(f'\n{"="*60}')
        print(f'{description}')
        print(f'{"="*60}')
        print(f'Total waypoints: {len(waypoints_list)}\n')
        
        # Print all waypoints
        for i, wp in enumerate(waypoints_list):
            pos = wp["position"]
            print(f'  [{i+1}] x={pos["x"]:6.2f}, y={pos["y"]:6.2f}, z={pos["z"]:6.2f}')
        print()
        
        # Convert waypoint dictionaries to PoseStamped messages
        goal_poses = [create_pose(wp) for wp in waypoints_list]
        
        # Start navigation
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)
        
        # Monitor progress
        i = 0
        last_waypoint = -1
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            
            if feedback:
                current_wp = feedback.current_waypoint
                if current_wp != last_waypoint:
                    pos = waypoints_list[current_wp]["position"]
                    print(f'\n→ Executing waypoint {current_wp + 1}/{len(goal_poses)}:')
                    print(f'  Position: x={pos["x"]:.2f}, y={pos["y"]:.2f}')
                    last_waypoint = current_wp
            
            # Safety timeout (10 minutes)
            now = navigator.get_clock().now()
            if now - nav_start > Duration(seconds=600):
                print('\n⚠ Navigation timeout! Canceling task...')
                navigator.cancelTask()
                return False
            
            time.sleep(0.2)
        
        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('\n✓ All waypoints reached successfully!\n')
            return True
        elif result == TaskResult.CANCELED:
            print('\n✗ Navigation was canceled!\n')
            return False
        elif result == TaskResult.FAILED:
            print('\n✗ Navigation failed!\n')
            return False
        else:
            print('\n✗ Navigation has invalid status!\n')
            return False
    
    def navigate_single_waypoint(waypoint_dict, description):
        """Navigate to a single waypoint"""
        pos = waypoint_dict["position"]
        orient = waypoint_dict["orientation"]
        
        print(f'\n{"="*60}')
        print(f'{description}')
        print(f'{"="*60}')
        print(f'Position: x={pos["x"]:.2f}, y={pos["y"]:.2f}, z={pos["z"]:.2f}')
        print(f'Orientation: x={orient["x"]:.4f}, y={orient["y"]:.4f}, z={orient["z"]:.4f}, w={orient["w"]:.4f}\n')
        
        goal_pose = create_pose(waypoint_dict)
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f'Distance remaining: {feedback.distance_remaining:.2f} m', end='\r')
            time.sleep(0.5)
        
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'\n✓ Waypoint reached successfully!\n')
            return True
        else:
            print(f'\n✗ Failed to reach waypoint!\n')
            return False
    
    def wait_for_iiwa(seconds, station_name):
        """Wait for iiwa manipulator to complete its actions"""
        print(f'{"="*60}')
        print(f'⏳ Waiting {seconds} seconds for {station_name} actions')
        print(f'{"="*60}')
        for i in range(seconds, 0, -1):
            print(f'   Time remaining: {i:2d} seconds', end='\r')
            time.sleep(1)
        print(f'   ✓ Wait complete! {" "*20}\n')
    
    # ============================================
    # INITIALIZATION
    # ============================================
    print('\n' + '='*60)
    print(' '*15 + 'FRA2MO WAYPOINT MISSION - EXTENDED')
    print('='*60)
    print('\nMission Overview:')
    print(f'  • First forward path: {len(waypoints_forward)} waypoints')
    print(f'  • Reverse path: {len(waypoints_forward)-1} waypoints')
    print(f'  • Second forward path: {len(waypoints_forward)-1} waypoints')
    print(f'  • Total waypoints: {len(waypoints_forward) + (len(waypoints_forward)-1) + (len(waypoints_forward)-1)}')
    print(f'  • iiwa1 operations: 2')
    print(f'  • iiwa2 operations: 2')
    print('\nWaiting for Nav2 to activate...')
    
    navigator.waitUntilNav2Active(localizer='smoother_server')
    print('✓ Nav2 is active! Starting mission...\n')
    
    # ============================================
    # FIRST FORWARD PATH (iiwa1 → iiwa2)
    # ============================================
    
    # Step 1: Go to iiwa1 station (waypoint 1)
    if not navigate_single_waypoint(waypoints_forward[0], 'WAYPOINT 1: iiwa1 Station (First Visit)'):
        print('❌ Mission failed at waypoint 1.')
        exit(1)
    wait_for_iiwa(30, 'iiwa1')
    
    # Step 2: Navigate waypoints 2-5 (batch navigation)
    if not navigate_waypoints(waypoints_forward[1:], 'FIRST FORWARD PATH: Waypoints 2-5'):
        print('❌ Mission failed during first forward path.')
        exit(1)
    
    # Step 3: Wait at iiwa2 station (waypoint 5)
    wait_for_iiwa(30, 'iiwa2 (First Visit)')
    
    # ============================================
    # REVERSE PATH (iiwa2 → iiwa1)
    # ============================================
    
    # Create reversed waypoints (from waypoint 5 back to waypoint 1)
    waypoints_reverse = list(reversed(waypoints_forward[:-1]))
    
    if not navigate_waypoints(waypoints_reverse, 'REVERSE PATH: Waypoints 4→1'):
        print('❌ Mission failed during reverse path.')
        exit(1)
    
    # Wait at iiwa1 station (second visit)
    wait_for_iiwa(30, 'iiwa1 (Second Visit)')
    
    # ============================================
    # SECOND FORWARD PATH (iiwa1 → iiwa2)
    # ============================================
    
    # Navigate from iiwa1 back to iiwa2 (waypoints 2-5)
    if not navigate_waypoints(waypoints_forward[1:], 'SECOND FORWARD PATH: Waypoints 2-5'):
        print('❌ Mission failed during second forward path.')
        exit(1)
    
    # Final wait at iiwa2 station (second visit)
    wait_for_iiwa(30, 'iiwa2 (Second Visit)')
    
    # ============================================
    # MISSION COMPLETE
    # ============================================
    print('\n' + '='*60)
    print(' '*15 + '🎉 MISSION COMPLETED 🎉')
    print('='*60)
    print('\nMission Summary:')
    print(f'  ✓ First forward path: {len(waypoints_forward)} waypoints (iiwa1 → iiwa2)')
    print(f'  ✓ Reverse path: {len(waypoints_reverse)} waypoints (iiwa2 → iiwa1)')
    print(f'  ✓ Second forward path: {len(waypoints_forward)-1} waypoints (iiwa1 → iiwa2)')
    print(f'  ✓ Total distance traveled: [calculated by nav2]')
    print(f'  ✓ iiwa1 operations: 2 completed')
    print(f'  ✓ iiwa2 operations: 2 completed')
    print(f'  ✓ All objectives achieved!')
    print('='*60 + '\n')
    
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()