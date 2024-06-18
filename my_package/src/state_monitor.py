#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import threading

class StateMonitor:
    def __init__(self):
        rospy.init_node('state_monitor', anonymous=True)
        self.state_sub = rospy.Subscriber('/box_carrier/state', String, self.state_callback)
        self.move_base_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.state_pub = rospy.Publisher('/box_carrier/state', String, queue_size=10)
        self.current_state = ""
        self.released_time = None
        self.monitor_state()

    def state_callback(self, msg):
        self.current_state = msg.data
        if self.current_state == "RELEASED":
            if self.released_time is None:
                self.released_time = rospy.Time.now()
        else:
            self.released_time = None

    def monitor_state(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.released_time:
                elapsed_time = rospy.Time.now() - self.released_time
                if elapsed_time.to_sec() >= 3:
                    self.send_goal_to_origin()
                    self.released_time = None
                    self.state_pub.publish("MOVING_TO_ORIGIN")
                        
            rate.sleep()

    def send_goal_to_origin(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        rospy.loginfo("Sending goal to (0, 0, 0)")
        self.move_base_goal_pub.publish(goal)

if __name__ == '__main__':
    try:
        state_monitor = StateMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
