#!/usr/bin/env python3
import math
import threading

import rospy
import tf
from geometry_msgs.msg import Pose
from my_package.msg import MoveBoxAToB  # Replace 'my_package' with the actual package name
from tkinter import ttk
import tkinter as tk
from ttkthemes import ThemedTk

from nav_msgs.msg import Odometry
from std_msgs.msg import String


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

shelf_map = {}
box_map = {}
for i in range(4):
    shelf_map[f"shelf_{i}"] = shelf_locations[i]
    box_map[f"box_{i}"] = box_locations[i]


def send_goal(box_name, shelf_name):
    global pub
    move_box_msg = MoveBoxAToB()
    move_box_msg.box_pose = box_map[box_name]
    move_box_msg.target_pose = shelf_map[shelf_name]
    move_box_msg.box_name = box_name

    rospy.loginfo("Publishing move box message...")
    pub.publish(move_box_msg)
    rospy.loginfo("Message published.")


def on_button_click(entry1, entry2):
    text1 = entry1.get()
    text2 = entry2.get()
    print(f'Box Name: {text1}, Shelf Name: {text2}')
    send_goal(text1, text2)


def stop():
    move_box_msg = MoveBoxAToB()
    move_box_msg.box_name = "STOP"
    rospy.loginfo("Publishing stop message...")
    pub.publish(move_box_msg)
    rospy.loginfo("Message published.")

    
def tkinter_loop():
    # Create the main window
    root = ThemedTk(theme="breeze")
    root.title("ROS Orchestrator")

    # Create a frame to hold the text fields
    frame = tk.Frame(root)
    frame.pack(pady=10)

    label1 = tk.Label(frame, text="Box Name")
    label2 = tk.Label(frame, text="Shelf Name")
    options1 = [*box_map]
    options2 = [*shelf_map]

    combobox1 = ttk.Combobox(frame, values=options1, state="readonly")
    combobox2 = ttk.Combobox(frame, values=options2, state="readonly")
    combobox1.current(0)
    combobox2.current(0)

    # Pack the text fields horizontally
    label1.grid(row=0, column=0, padx=5, pady=(0, 5))
    label2.grid(row=0, column=1, padx=5, pady=(0, 5))
    combobox1.grid(row=1, column=0, padx=5)
    combobox2.grid(row=1, column=1, padx=5)
    # Create a button
    button = tk.Button(root, text="SEND", command=lambda: on_button_click(combobox1, combobox2))
    button.pack(pady=10)

    #add a stop button tkinter
    stop_button = tk.Button(root, text="STOP", command=stop)
    stop_button.pack(pady=10)
    global state_label
    # State label at the bottom
    state_label = tk.Label(root, text="State: N/A")
    state_label.pack(pady=10)

    # Schedule the state label update function to run periodically
    # Start the Tkinter event loop
    root.mainloop()


carrier_state = None
carried_box_name = None
pub = None
state_label = None


def odom_callback(msg):
    global x, y, yaw, carrier_state, carried_box_name, box_map
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    yaw = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])[2]
    if carrier_state is not None and carrier_state == "CARRYING" and carried_box_name is not None:
        box_map[carried_box_name] = trivial_position_to_pose(x, y, yaw)
    if carrier_state is not None and carrier_state == "MOVING_TO_TARGET":
        box_map[carried_box_name] = trivial_position_to_pose(x, y, yaw)


def state_callback(msg):
    global carrier_state, carried_box_name
    state = msg.data
    if "CARRYING" in state:
        carried_box_name = msg.data.split()[-1]
        carrier_state = "CARRYING"
    else:
        carrier_state = msg.data

    state_label.config(text=f"State: {carrier_state}")

def ros_loop():
    global pub
    rospy.init_node('orchestrator', anonymous=True)
    pub = rospy.Publisher('/box_carrier/move_box_a_to_b', MoveBoxAToB, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/box_carrier/state', String, state_callback)


def main():
    ros_loop()
    tkinter_thread = threading.Thread(target=tkinter_loop)
    tkinter_thread.start()

    tkinter_thread.join()


if __name__ == '__main__':
    main()
