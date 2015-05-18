#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import String

from reflex_sf_msgs.msg import SFPose
import sf_state


# BUTTON_DICT =  {'1': 'button_open',
#                 '2': 'button_close',
#                 '3': 'button_dof',
#                 '4': 'button_preshape',
#                 '5': 'button_zero_fingers'}
#                 Do we want to be able to tighten or loosen?


class SFWrapper:
    def __init__(self):
        rospy.init_node('sf_wrapper')
        rospy.Subscriber("/buttons", String, self.button_callback)
        self.hand = sf_state.SFState()

    def button_callback(self, msg):
        if msg.data[0] == 's':
            self.command_finger_pose_from_sliders(msg.data)
        else:
            if msg.data == '1':
                rospy.loginfo('1) Opening hand')
                self.hand.open()
            if msg.data == '2':
                rospy.loginfo('2) Closing hand')
                self.hand.close()
            if msg.data == '3':
                rospy.loginfo('3) Tightening')
                self.hand.tighten()
            if msg.data == '4':
                rospy.loginfo('4) Cycling preshape')
                self.hand.cycle_preshape()
            if msg.data == '5':
                rospy.loginfo('5) Running DOF')
                self.hand.dof()

    def command_finger_pose_from_sliders(self, data_string):
        values = data_string.split(':')[1:]
        values = [3.5 * float(val) for val in values]
        cmd = SFPose(f1=values[0], f2=values[1], f3=values[2])
        self.hand.command_fingers_preserve_preshape(cmd)


if __name__ == '__main__':
        wrapper = SFWrapper()
        rospy.spin()
