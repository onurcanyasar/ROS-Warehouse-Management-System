#!/usr/bin/env python

from box_carrier import BoxCarrier
import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState, LinkState
import math
from tf.transformations import euler_from_quaternion
def set_model_position(model_name, x, y, z):
    rospy.init_node('set_model_position', anonymous=True)
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        resp = set_model_state(model_state)
        if resp.success:
            rospy.loginfo("Successfully set model position.")
        else:
            rospy.logerr("Failed to set model position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
def update_box_position(robot_name, box_name, distance_in_front):
    rospy.init_node('update_box_position', anonymous=True)
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')

    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    rate = rospy.Rate(30)  # 30 Hz

    while not rospy.is_shutdown():
        try:
            # Get the robot's current state
            robot_state = get_model_state(robot_name, "")

            # Extract robot's position and orientation
            robot_pose = robot_state.pose
            robot_position = robot_pose.position
            robot_orientation = robot_pose.orientation

            # Convert robot orientation from quaternion to Euler angles
            orientation_list = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

            # Calculate the position in front of the robot
            box_position = ModelState()
            box_position.model_name = box_name
            box_position.pose.position.x = robot_position.x + distance_in_front * math.cos(yaw)
            box_position.pose.position.y = robot_position.y + distance_in_front * math.sin(yaw)
            box_position.pose.position.z = robot_position.z
            box_position.pose.orientation = robot_orientation  # Maintain the same orientation as the robot

            # Set the box state
            set_model_state(box_position)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        rate.sleep()
