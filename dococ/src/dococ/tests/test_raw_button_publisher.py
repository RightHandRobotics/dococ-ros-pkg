#!/usr/bin/env python

import unittest

import mock
import rospy

import raw_button_publisher


class TestRawButtonPublisher(unittest.TestCase):
    @mock.patch('rospy.loginfo')
    @mock.patch('rospy.init_node')
    def setUp(self, init_node_mock, log_mock):
        self.RPB = raw_button_publisher.RawButtonPublisher()

    def test_is_known_simple_message(self):
        good_strings = ['1', '2', '3', '4', '5']
        bad_strings = ['', '-1', '6', 'r', 'sadfasd', '10', '0f', '\t', '\\']

        [self.assertTrue(self.RPB.is_known_simple_message(good))
            for good in good_strings]
        [self.assertFalse(self.RPB.is_known_simple_message(bad))
            for bad in bad_strings]

    def test_is_known_complex_message(self):
        good_strings = ['s:0.12:0.62:0.23', 's:0.00:0.02:0.77']
        bad_strings = ['a:36:12:7', '0', 's:4:4:4:5', '']

        [self.assertTrue(self.RPB.is_known_complex_message(good))
            for good in good_strings]
        [self.assertFalse(self.RPB.is_known_complex_message(bad))
            for bad in bad_strings]

    def test_check_and_publish_data(self):
        self.RPB.button_pub = mock.MagicMock()
        time_since_last_call = rospy.Duration.from_sec(0.05)
        data = '1'
        self.RPB.check_and_publish_data(time_since_last_call, data)
        assert not self.RPB.button_pub.publish.called, \
            'data published when it should not have been'

        self.RPB.button_pub.reset_mock()
        time_since_last_call = rospy.Duration.from_sec(0.15)
        self.RPB.check_and_publish_data(time_since_last_call, data)
        self.RPB.button_pub.publish.assert_called_once_with(data)

        self.RPB.button_pub.reset_mock()
        time_since_last_call = rospy.Duration.from_sec(0.05)
        data = 's:10:50:100'
        self.RPB.check_and_publish_data(time_since_last_call, data)
        self.RPB.button_pub.publish.assert_called_once_with(data)

        self.RPB.button_pub.reset_mock()
        time_since_last_call = rospy.Duration.from_sec(0.15)
        data = 's:10:50'
        self.RPB.check_and_publish_data(time_since_last_call, data)
        assert not self.RPB.button_pub.publish.called, \
            'data published when it should not have been'

    def test_open_dococ_serial(self):
        pass  # Don't really know how to test this function

    def test_read_publish_serial(self):
        pass
