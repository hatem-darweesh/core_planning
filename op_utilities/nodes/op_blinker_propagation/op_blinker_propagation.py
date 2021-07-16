#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from autoware_msgs.msg import LampCmd
from autoware_msgs.msg import Waypoint


class OpBlinkerPropagation:
    def __init__(self):
        self.cmd = LampCmd()
        self.cmd.l = 0
        self.cmd.r = 0
        self.prev_ind = 3
        self.pub = rospy.Publisher('lamp_cmd', LampCmd, queue_size=10)
        self.sub = rospy.Subscriber('op_current_behavior', Waypoint, self.callback)
        rospy.init_node('op_blinker_propagation', anonymous=True)

    def callback(self, msg):
        ind = msg.direction
        if self.prev_ind != ind:
            if ind == 3:
                self.cmd.l = 0
                self.cmd.r = 0
            elif ind == 1:
                self.cmd.l = 0
                self.cmd.r = 1
            elif ind == 0:
                self.cmd.l = 1
                self.cmd.r = 0
            elif ind == 2:
                self.cmd.l = 1
                self.cmd.r = 1
            else:
                self.cmd.r = 0
                self.cmd.l = 0
            self.pub.publish(self.cmd)
        self.prev_ind = ind

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = OpBlinkerPropagation()
        node.run()
    except rospy.ROSInterruptException:
        pass
