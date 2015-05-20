#!/usr/bin/env python

import serial

import rospy
from std_msgs.msg import String


class RawButtonPublisher():
    def __init__(self):
        rospy.init_node('raw_arduino_button_publisher')
        self.button_pub = rospy.Publisher('/buttons', String, queue_size=1)
        self.debounce = rospy.Duration.from_sec(0.1)
        self.known_simple_messages = ['1', '2', '3', '4', '5']
        self.arduino = None

    def is_known_simple_message(self, data):
        '''
        Checks whether given piece of data matches known simple messages
        '''
        data = data.strip()
        for message in self.known_simple_messages:
            if data == message:
                return True
        return False

    def is_known_complex_message(self, data):
        '''
        Checks whether a given piece of data matches the slider data from the
        Arduino (s:0.99:0.64:0.15 for some random integer assortment)
        '''
        data = data.strip().split(':')
        if len(data) != 4 or data[0] != 's':
            return False
        for floating in data[1:]:
            try:
                float(floating)
            except ValueError:
                return False
        return True

    def open_dococ_serial(self):
        try:
            self.arduino = serial.Serial('/dev/dococ_arduino', 115200)
            rospy.loginfo('Connection successful')
        except serial.SerialException:
            rospy.loginfo('Have you copied dococ/rules/99-dococ-devices.rules')
            rospy.loginfo('/into /etc/udev/rules.d/ ? Or maybe you are using')
            rospy.loginfo('an Arduino that is not an Uno? Your computer will')
            rospy.loginfo('not correctly symlink the device in either case')

    def check_and_publish_data(self, time_since_last_call, data):
        if time_since_last_call > self.debounce or data[0] == 's':
            data = data.strip()
            if self.is_known_simple_message(data) or \
               self.is_known_complex_message(data):
                self.button_pub.publish(data)

    def main(self):
        self.open_dococ_serial()
        if self.arduino is not None:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                data = self.arduino.readline()
                self.check_and_publish_data(rospy.Time.now() - now, data)
        rospy.loginfo('Connector to Arduino failed, halting...')


if __name__ == '__main__':
    try:
        RBP = RawButtonPublisher()
        RBP.main()
    except rospy.ROSInterruptException:
        rospy.loginfo('Raw button publisher failed, halting...')
