#!/usr/bin/env python3
import math

import tf
from geometry_msgs.msg import Pose


def trivial_position_to_pose(x, y, rad_angle):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    pose.orientation = tf.transformations.quaternion_from_euler(0, 0, rad_angle)
    return pose


shelf_locations = [(trivial_position_to_pose(2.1, 0.5, 0)),
                   (trivial_position_to_pose(2.1, -1.2, 0)),
                   (trivial_position_to_pose(2.1, -2.9, 0)),
                   (trivial_position_to_pose(2.1, -4.6, 0))
                   ]

box_locations = [(trivial_position_to_pose(0.3, 4.4, math.pi)),
                 (trivial_position_to_pose(0.3, 5.0, math.pi)),
                 (trivial_position_to_pose(0.3, 5.6, math.pi)),
                 (trivial_position_to_pose(0.3, 6.3, math.pi)),
                 ]


def main():
    pass


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import math
import rospy
import tf
from geometry_msgs.msg import Pose
from my_package.msg import MoveBoxAToB  # Replace 'my_package' with the actual package name


def trivial_position_to_pose(x, y, rad_angle):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0
    quaternion = tf.transformations.quaternion_from_euler(0, 0, rad_angle)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


shelf_locations = [trivial_position_to_pose(2.1, 0.5, 0),
                   trivial_position_to_pose(2.1, -1.2, 0),
                   trivial_position_to_pose(2.1, -2.9, 0),
                   trivial_position_to_pose(2.1, -4.6, 0)
                   ]

box_locations = [trivial_position_to_pose(0.3, 4.4, math.pi),
                 trivial_position_to_pose(0.3, 5.0, math.pi),
                 trivial_position_to_pose(0.3, 5.6, math.pi),
                 trivial_position_to_pose(0.3, 6.3, math.pi),
                 ]


def main():
    rospy.init_node('orchestrator', anonymous=True)
    pub = rospy.Publisher('/box_carrier/move_box_a_to_b', MoveBoxAToB, queue_size=10)
    rospy.sleep(1)  # Wait a moment to ensure the publisher is registered

    move_box_msg = MoveBoxAToB()
    move_box_msg.box_pose = box_locations[1]  # box_2
    move_box_msg.target_pose = shelf_locations[3]  # shelf_1
    move_box_msg.box_name = "box_1"

    rospy.loginfo("Publishing move box message...")
    pub.publish(move_box_msg)
    rospy.loginfo("Message published.")



if __name__ == '__main__':
    main()
