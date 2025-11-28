#!/usr/bin/env python
"""Simple ROS node that simulates lead/stop logic for the dog.
- Subscribes to a 'mode' topic to switch behavior (idle, health, boss, return)
- Publishes velocity commands on /cmd_vel when leading
- Publishes simple state on /dog_state
This is a simulation stub for development without real hardware.
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class DogController:
    def __init__(self):
        rospy.init_node('dog_controller', anonymous=True)
        self.mode = 'idle'
        self.state_pub = rospy.Publisher('/dog_state', String, queue_size=10)
        self.mode_sub = rospy.Subscriber('/dog_mode', String, self.mode_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def mode_cb(self, msg):
        rospy.loginfo('Mode set to: %s' % msg.data)
        self.mode = msg.data

    def publish_state(self):
        s = String()
        s.data = self.mode
        self.state_pub.publish(s)

    def lead_behavior(self):
        # When leading, publish a slow forward velocity within safe limits
        t = Twist()
        t.linear.x = 0.3  # m/s safe max per PRD
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def stop_behavior(self):
        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def run(self):
        rospy.loginfo('Dog controller started')
        while not rospy.is_shutdown():
            if self.mode in ['health', 'boss']:
                self.lead_behavior()
            else:
                self.stop_behavior()
            self.publish_state()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DogController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
