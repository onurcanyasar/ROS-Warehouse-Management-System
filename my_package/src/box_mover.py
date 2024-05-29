#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from my_package.msg import MoveBoxAToB, CarryCommand 
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
class BoxMover:
    def __init__(self):
        rospy.init_node('box_mover', anonymous=True)
        self.box_pose = None
        self.target_pose = None
        self.current_box_name = ""
        self.state = "IDLE"

        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('/box_carrier/move_box_a_to_b', MoveBoxAToB, self.move_box_a_to_b_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        self.state_pub = rospy.Publisher('/box_carrier/state', String, queue_size=10)
        self.carry_pub = rospy.Publisher('/box_carrier/carry_box', MoveBoxAToB, queue_size=10)
        self.release_pub = rospy.Publisher('/box_carrier/release_box', CarryCommand, queue_size=10)

    def move_box_a_to_b_callback(self, msg):
        self.box_pose = msg.box_pose
        self.target_pose = msg.target_pose
        self.current_box_name = msg.box_name

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = self.box_pose

        rospy.loginfo("Moving to the box...")
        self.state_pub.publish("MOVING_TO_BOX")
        self.state = "MOVING_TO_BOX"
        self.move_base_goal_pub.publish(goal)

    def move_base_result_callback(self, msg):
        if msg.status.status == 3:  # Goal reached
            if self.state == "MOVING_TO_BOX":
                rospy.loginfo("Reached the box. Starting to carry the box.")
                self.state_pub.publish("REACHED_TO_BOX")
                self.initiate_carry_action()
            elif self.state == "MOVING_TO_TARGET":
                rospy.loginfo("Reached the target point. Releasing the box.")
                self.state_pub.publish("REACHED_TO_TARGET")
                self.release_box()
                self.state = "IDLE"
        elif msg.status.status in [4, 5, 9]:  # Goal aborted, rejected, or lost
            rospy.loginfo("Failed to reach the target point.")
            self.state_pub.publish("FAIL")
            self.state = "IDLE"


    def initiate_carry_action(self):
        rospy.loginfo("Initiating the carry action...")
        carry_msg = MoveBoxAToB()
        carry_msg.box_name = self.current_box_name
        carry_msg.box_pose = self.box_pose
        carry_msg.target_pose = self.target_pose
        self.carry_pub.publish(carry_msg)
        rospy.sleep(1)  # Wait a moment to ensure the carry action has started

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose = self.target_pose
        rospy.loginfo("Moving to the target point...")
        self.state_pub.publish("Moving to the target point...")
        self.state = "MOVING_TO_TARGET"
        self.move_base_goal_pub.publish(goal)

    def release_box(self):
        release_msg = CarryCommand()
        release_msg.box_name = self.current_box_name
        release_msg.carry = False
        self.release_pub.publish(release_msg)
        rospy.sleep(1)  # Wait a moment to ensure the release action has completed

if __name__ == '__main__':
    try:
        box_mover = BoxMover()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
