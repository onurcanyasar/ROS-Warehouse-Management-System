#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String
from my_package.msg import MoveBoxAToB, CarryCommand


class BoxCarrier:
    def __init__(self, node_name, robot_name, distance_in_front):
        self.node_name = node_name
        self.robot_name = robot_name
        rospy.init_node(node_name, anonymous=True)
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')

        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.rate = rospy.Rate(30)  # 30 Hz

        self.is_carrying = False
        self.distance_in_front = distance_in_front
        self.current_box_name = ""
        self.state = "IDLE"

        self.state_pub = rospy.Publisher('/box_carrier/state', String, queue_size=10)
        rospy.Subscriber('/box_carrier/carry_box', MoveBoxAToB, self.carry_box_callback)
        rospy.Subscriber('/box_carrier/release_box', CarryCommand, self.stop_carrying_callback)

    def carry_box_callback(self, msg):
        self.current_box_name = msg.box_name
        self.state_pub.publish("CARRYING " + self.current_box_name)
        self.start_carrying()

    def start_carrying(self):
        self.is_carrying = True
        while not rospy.is_shutdown() and self.is_carrying:
            try:
                # Get the robot's current state
                robot_state = self.get_model_state(self.robot_name, "")

                # Extract robot's position and orientation
                robot_pose = robot_state.pose
                robot_position = robot_pose.position
                robot_orientation = robot_pose.orientation

                # Convert robot orientation from quaternion to Euler angles
                orientation_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

                # Calculate the position in front of the robot
                box_position = ModelState()
                box_position.model_name = self.current_box_name
                box_position.pose.position.x = robot_position.x + self.distance_in_front * math.cos(yaw)
                box_position.pose.position.y = robot_position.y + self.distance_in_front * math.sin(yaw)
                box_position.pose.position.z = 0.15
                box_position.pose.orientation = robot_orientation  # Maintain the same orientation as the robot

                # Set the box state
                self.set_model_state(box_position)

            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

    def stop_carrying_callback(self, msg):
        self.is_carrying = False
        rospy.loginfo("Stopped carrying the box.")
        self.state_pub.publish("RELEASED")


if __name__ == '__main__':
    try:
        node_name = "box_carrier_node"
        robot_name = "turtlebot3"
        distance_in_front = 0.3  # Adjust the distance as needed

        carrier = BoxCarrier(node_name, robot_name, distance_in_front)
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
