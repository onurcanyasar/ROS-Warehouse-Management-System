#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped
import tf

def send_goal():
    rospy.init_node('send_goal_node')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Give some time for the publisher to connect
    goal = PoseStamped()
    goal.header.frame_id = "map"  # Ensure this is correctly set to your fixed frame
    goal.header.stamp = rospy.Time.now()

    # Set the goal position and orientation
    goal.pose.position.x = 2.1
    goal.pose.position.y = 0.5
    goal.pose.position.z = 0.0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)  # Adjust as needed for the desired orientation
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    rospy.loginfo("Sending goal")
    pub.publish(goal)
    rospy.loginfo("Goal sent")
if __name__ == '__main__':
 
    send_goal() 
