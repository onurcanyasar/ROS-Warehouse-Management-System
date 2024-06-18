#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker

def display_box(data, name, text_name, id):
    try:
        model_index = data.name.index(name)  # Replace 'your_model_name' with your Gazebo model name
        pose = data.pose[model_index]

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = id
        marker.type = Marker.CUBE  # Choose the appropriate shape
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.3  # Set the appropriate scale
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Create the text marker
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "text"
        text_marker.id = id 
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position = pose.position
        text_marker.pose.position.z += 2.5  # Adjust the height as needed
        text_marker.scale.z = 0.5  # Text height
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = text_name 

        pub.publish(marker)
        pub.publish(text_marker)
    except ValueError:
        pass
def callback(data):
    
    display_box(data, "box_0", "b0", 0)
    display_box(data, "box_1", "b1", 1)
    display_box(data, "box_2", "b2", 2)
    display_box(data, "box_3", "b3", 3)
    display_box(data, "aws_robomaker_warehouse_ShelfE_01_001", "s0", 4)
    display_box(data, "aws_robomaker_warehouse_ShelfD_01_001", "s1", 5)
    display_box(data, "aws_robomaker_warehouse_ShelfD_01_002", "s2", 6)
    display_box(data, "aws_robomaker_warehouse_ShelfE_01_002", "s3", 7)
    

    
    # display_box(data, "box_1", 1)
    # display_box(data, "box_2", 2)
    # display_box(data, "box_3", 3)
    # display_box("aws_robomaker_warehouse_ShelfE_01_001", 4)
    # display_box("aws_robomaker_warehouse_ShelfE_01_002", 5)
    # display_box("aws_robomaker_warehouse_ShelfE_01_003", 6)
    
if __name__ == '__main__':
    rospy.init_node('gazebo_to_rviz')
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.spin()
    