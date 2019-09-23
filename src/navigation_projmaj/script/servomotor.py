#!/usr/bin/env python

import rospy
import time
import sys
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


class Servomotor:
    # if mode = False, the box is closed
    # else, open
    if sys.argv[1] == 'close':
        mode = False
    elif sys.argv[1] == 'open':
        mode = True
    else:
        mode = False
    angle_motor_1 = 0.0

    def __init__(self):
        rospy.init_node('servomotor', anonymous=True)
        self.pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size=10)
        rospy.Subscriber('/joint1_controller/state', JointState, self.callback)

    def open_box(self):
        self.angle_motor_1 = 1.5
        rospy.loginfo("Open Box")
        self.pub.publish(self.angle_motor_1)

    def close_box(self):
        self.angle_motor_1 = 0.0
        rospy.loginfo("Close Box")
        self.pub.publish(self.angle_motor_1)

    def callback(self, data):
        if not self.mode:
            self.close_box()
            time.sleep(2)
        if self.mode:
            self.open_box()
            time.sleep(15)
            self.mode = False

        rospy.loginfo(self.angle_motor_1)

    def listener(self):
        rospy.Subscriber('/joint1_controller/state', JointState, self.callback)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        s = Servomotor()
        s.listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
