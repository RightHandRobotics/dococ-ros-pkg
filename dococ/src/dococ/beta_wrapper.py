#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

from reflex_msgs.srv import CommandHand, CommandHandRequest

# BUTTON_DICT =  {'1': 'button_open',
#                 '2': 'button_close',
#                 '3': 'button_dof',
#                 '4': 'button_preshape',
#                 '5': 'button_zero_fingers'}
#                 Do we want to be able to tighten or loosen?


class BetaWrapper:
    def __init__(self):
        rospy.init_node('beta_wrapper')
        rospy.Subscriber("/buttons", String, self.button_callback)
        self.zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
        self.command_smarts = rospy.ServiceProxy('/reflex/command_smarts', CommandHand)
        self.preshape_state = 'cylinder'
        self.preshape_cycle = {'cylinder':'spherical',
                               'spherical':'pinch',
                               'pinch':'cylinder'}

    def button_callback(self, msg):
        if msg.data[0] == 's':
            self.command_finger_pose_from_sliders(msg.data)
        else:
            if msg.data == '1':
                rospy.loginfo('1) Opening hand')
                self.command_smarts(CommandHandRequest('open'))
            if msg.data == '2':
                rospy.loginfo('2) Guarded close')
                self.zero_tactile()
                rospy.sleep(1.0)
                self.command_smarts(CommandHandRequest('guarded_move'))
            if msg.data == '3':
                rospy.loginfo('3) Preshape cycle')
                self.preshape_state = self.preshape_cycle[self.preshape_state]
                self.command_smarts(CommandHandRequest(self.preshape_state))

if __name__ == '__main__':
        wrapper = BetaWrapper()
        rospy.spin()
