from copy import deepcopy

import rospy

from reflex_sf_msgs.msg import SFPose


class SFState():
    def __init__(self):
        self.publisher = rospy.Publisher('/reflex_sf/command', SFPose,
                                         queue_size=1)
        self.preshape_points = {'ROT_CYL': 0.0, 'ROT_SPH': 0.9,
                                'ROT_PINCH': 2.25}  # position in radians
        self.preshape_cycle = {self.preshape_points['ROT_CYL']:
                               self.preshape_points['ROT_SPH'],
                               self.preshape_points['ROT_SPH']:
                               self.preshape_points['ROT_PINCH'],
                               self.preshape_points['ROT_PINCH']:
                               self.preshape_points['ROT_CYL']}
        self.small_finger_delta = 0.1  # position in almost radians
        self.inner_finger_limit = 0.0  # position in almost radians
        self.outer_finger_limit = 4.2  # position in almost radians
        self.last_sent_command = SFPose()

    def publish_state(self, sfpose):
        self.publisher.publish(sfpose)
        self.last_sent_command = deepcopy(sfpose)

    def small_delta_within_bounds(self, finger_pos, direction):
        '''
        Takes a finger and returns a new position in the direction (+1/-1)
        without going outside of the hand bounds
        '''
        if direction > 0:
            if finger_pos + self.small_finger_delta > self.outer_finger_limit:
                return finger_pos
            else:
                return (finger_pos + self.small_finger_delta)
        else:
            if finger_pos - self.small_finger_delta < self.inner_finger_limit:
                return finger_pos
            else:
                return (finger_pos - self.small_finger_delta)

    def open(self):
        cmd = SFPose(preshape=self.last_sent_command.preshape)
        self.command_fingers_preserve_preshape(cmd)

    def close(self):
        cmd = deepcopy(self.last_sent_command)
        cmd.f1 = self.outer_finger_limit
        cmd.f2 = self.outer_finger_limit
        cmd.f3 = self.outer_finger_limit
        self.publish_state(cmd)

    def tighten(self):
        cmd = deepcopy(self.last_sent_command)
        cmd.f1 = self.small_delta_within_bounds(cmd.f1, 1)
        cmd.f2 = self.small_delta_within_bounds(cmd.f2, 1)
        cmd.f3 = self.small_delta_within_bounds(cmd.f3, 1)
        self.publish_state(cmd)

    def loosen(self):
        cmd = deepcopy(self.last_sent_command)
        cmd.f1 = self.small_delta_within_bounds(cmd.f1, -1)
        cmd.f2 = self.small_delta_within_bounds(cmd.f2, -1)
        cmd.f3 = self.small_delta_within_bounds(cmd.f3, -1)
        self.publish_state(cmd)

    def command_fingers_preserve_preshape(self, sfpose):
        cmd = deepcopy(self.last_sent_command)
        cmd.f1 = sfpose.f1
        cmd.f2 = sfpose.f2
        cmd.f3 = sfpose.f3
        self.publish_state(cmd)

    def cycle_preshape(self):
        cmd = deepcopy(self.last_sent_command)
        cmd.preshape = self.preshape_cycle[cmd.preshape]
        self.publish_state(cmd)

    def dof(self, wait_time=2.0):
        cmd = SFPose(f1=self.outer_finger_limit)
        self.publish_state(cmd)
        rospy.sleep(wait_time)

        cmd.f2 = self.outer_finger_limit
        self.publish_state(cmd)
        rospy.sleep(wait_time)

        cmd.f3 = self.outer_finger_limit
        self.publish_state(cmd)
        rospy.sleep(wait_time)

        self.publish_state(SFPose())
        rospy.sleep(wait_time)

        self.cycle_preshape()
        rospy.sleep(wait_time)

        self.cycle_preshape()
        rospy.sleep(wait_time)

        self.cycle_preshape()
        rospy.sleep(wait_time)
