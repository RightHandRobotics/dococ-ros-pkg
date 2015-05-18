#!/usr/bin/env python

import unittest

import mock

import sf_state
from reflex_sf_msgs.msg import SFPose


class TestSFState(unittest.TestCase):
    @mock.patch('rospy.Publisher')
    def setUp(self, publisher_mock):
        self.SFS = sf_state.SFState()

    def test_publish_state(self):
        cmd = 0.1
        self.SFS.publish_state(cmd)
        self.SFS.publisher.publish.assert_called_once_with(cmd)
        self.assertAlmostEqual(cmd, self.SFS.last_sent_command)

    def test_small_delta_within_bounds(self):
        given_up = [1.2, 4.1, 4.2, -1.1, -0.1, 5.0]
        given_down = [1.4, 4.3, 4.3, -1.0, 0.0, 5.1]
        expected = [1.3, 4.2, 4.2, -1.0, 0.0, 5.0]

        for i in range(len(expected)):
            self.assertAlmostEqual(expected[i],
                self.SFS.small_delta_within_bounds(given_up[i], 1))
            self.assertAlmostEqual(expected[i],
                self.SFS.small_delta_within_bounds(given_down[i], -1))

    def test_open(self):
        self.SFS.publish_state = mock.MagicMock()

        self.SFS.last_sent_command.preshape = 0.0
        expected = SFPose()
        self.SFS.open()
        self.SFS.publish_state.assert_called_once_with(expected)

        self.SFS.publish_state.reset_mock()
        self.SFS.last_sent_command.preshape = 1.5
        expected.preshape = 1.5
        self.SFS.open()
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_close(self):
        self.SFS.publish_state = mock.MagicMock()

        self.SFS.last_sent_command.preshape = 0.0
        expected = SFPose()
        expected.f1 = 4.2
        expected.f2 = 4.2
        expected.f3 = 4.2
        self.SFS.close()
        self.SFS.publish_state.assert_called_once_with(expected)

        self.SFS.publish_state.reset_mock()
        self.SFS.last_sent_command.preshape = 1.5
        expected.preshape = 1.5
        self.SFS.close()
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_tighten(self):
        self.SFS.publish_state = mock.MagicMock()
        self.SFS.small_delta_within_bounds = mock.MagicMock()
        self.SFS.small_delta_within_bounds.side_effect = [1, 2, 3]
        self.SFS.last_sent_command = SFPose()
        self.SFS.last_sent_command.preshape = 2.6
        self.SFS.tighten()

        expected = SFPose()
        expected.f1 = 1
        expected.f2 = 2
        expected.f3 = 3
        expected.preshape = 2.6
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_loosen(self):
        self.SFS.publish_state = mock.MagicMock()
        self.SFS.small_delta_within_bounds = mock.MagicMock()
        self.SFS.small_delta_within_bounds.side_effect = [1, 2, 3]
        self.SFS.last_sent_command = SFPose()
        self.SFS.last_sent_command.preshape = 2.6
        self.SFS.loosen()

        expected = SFPose()
        expected.f1 = 1
        expected.f2 = 2
        expected.f3 = 3
        expected.preshape = 2.6
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_command_fingers_preserve_preshape(self):
        self.SFS.publish_state = mock.MagicMock()
        self.SFS.last_sent_command.preshape = 2.1
        cmd = SFPose()
        cmd.f1 = 2.9
        cmd.f2 = 1.6
        cmd.f3 = 0.3
        self.SFS.command_fingers_preserve_preshape(cmd)

        expected = cmd
        expected.preshape = 2.1
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_cycle_preshape(self):
        self.SFS.publish_state = mock.MagicMock()
        self.SFS.last_sent_command = SFPose()
        self.SFS.cycle_preshape()
        expected = SFPose()
        expected.preshape = 0.9
        self.SFS.publish_state.assert_called_once_with(expected)

        self.SFS.publish_state.reset_mock()
        self.SFS.last_sent_command.preshape = 0.9
        self.SFS.cycle_preshape()
        expected.preshape = 2.25
        self.SFS.publish_state.assert_called_once_with(expected)

        self.SFS.publish_state.reset_mock()
        self.SFS.last_sent_command.preshape = 2.25
        self.SFS.cycle_preshape()
        expected = SFPose()
        self.SFS.publish_state.assert_called_once_with(expected)

    def test_dof(self):
        # This function seemed both simple to debug and annoying to test, so
        # it won't be tested
        pass
