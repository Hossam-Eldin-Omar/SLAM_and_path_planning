#!/usr/bin/python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamp(navigator, pos_x, pos_y, orien_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orien_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose
def main():
    rclpy.init()
    nav = BasicNavigator()

    #--set_initial pose
    initial_pose = create_pose_stamp(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    
    # -- wait for Nav2
    nav.waitUntilNav2Active()

    # -- set pose stamps
    goal_pose1 = create_pose_stamp(nav, 4.0, 1.0, 1.57)
    goal_pose2 = create_pose_stamp(nav, 2.5, 2.0, 1.57)

    way_points = [goal_pose1, goal_pose2]

    nav.followWaypoints(way_points)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
    
    print(nav.getResult())


    rclpy.shutdown()

if __name__ == '__main__':
    main()