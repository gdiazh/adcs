#!/usr/bin/python

__author__ = 'gdiaz'

# ROS INTERFACE

"""Provides a high level interface over ROS.
"""

import rospy, time

from threading import Thread
from std_msgs.msg import String
from std_msgs.msg import Float32

class SpeedTest:
    def __init__(self, test_name):
        # Only argument stuff
        self.running = False
        self.mode = "manual"
        self.speed = 1400
        self.step = 50
        self.max_speed = 2000
        self.setting_time = 55

    def initialize(self):
        # Get params and allocate msgs
        self.state_update_rate = rospy.get_param('/rate', 1000)

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        #subscribers
        self.test_command_sub = rospy.Subscriber('/test_command_speed', String, self.process_command)
        #publishers
        self.speedX_pub = rospy.Publisher('/speed_x', Float32, queue_size=70)
        self.controller_command_pub = rospy.Publisher('/controller_command', String, queue_size=70)
        Thread(target=self.update_test).start()

    def stop(self):
        self.running = False
        self.test_command_sub.unregister()
        self.speedX_pub.unregister()
        self.controller_command_pub.unregister()

    def process_command(self, msg):
        print "test state?(ON/END): "+msg.data
        if (msg.data == "E"):
            self.running = False
            return

    def update_test(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Update speed command
            if (self.speed>=self.max_speed):
                self.running = False
                # rospy.signal_shutdown("TEST END")
            	break
            self.speedX_pub.publish(self.speed)
            self.controller_command_pub.publish("set-speed")
            time.sleep(self.setting_time)
            self.speed = self.speed + self.step
            rate.sleep()
        return

if __name__ == '__main__':
    test_name = raw_input("start test? (Y/N): ")
    rospy.init_node('speed_test')
    speed_test = SpeedTest(test_name)
    speed_test.initialize()
    speed_test.start()
    rospy.spin()
    speed_test.stop()