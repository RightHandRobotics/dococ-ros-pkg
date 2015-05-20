#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String
from std_srvs.srv import Empty
from reflex_msgs.srv import CommandHand
from reflex_msgs.msg import RadianServoPositions
import numpy as np

BUTTON_DICT =  {'0': 'button_guarded',
				'1': 'button_grasp_harder',
				'2': 'button_dof',
				'3': 'button_set',
				'4': 'button_preshape'}

# hand state variables
OPEN = 1
CLOSED = 2
BUSY = 3
ABORT = 0

ROT_CYL = 0.0		# radians rotation 
ROT_SPH = 0.8		# radians rotation
ROT_PINCH = 1.57 	# radians rotation

class Primitive_Wrapper:
	def __init__(self):
		rospy.Subscriber("/buttons", String, self.button_callback)
		self.hand_state = OPEN
		self.hand_shape = ROT_CYL
		self.command_smarts = rospy.ServiceProxy('/reflex/command_smarts', CommandHand)
		self.set_reflex_hand = rospy.Publisher('set_reflex_hand', RadianServoPositions)
		self.zero_tactile = rospy.ServiceProxy('/zero_tactile', Empty)
		self.zero_fingers = rospy.ServiceProxy('/zero_fingers', Empty)
		self.finger_pos = np.array([-1.,-1.,-1.])
	
	def open_hand(self):
		self.command_smarts('open')
		self.hand_state = OPEN

	def button_callback(self,data):
		#print "got data"
		temp = data.data
		if temp[0] != 's':
			getattr(self, BUTTON_DICT[temp])()
		else:
			vals = temp.split(':')[1:4]
			vals = [3.5 * float(val) for val in vals]
			if np.max(abs(np.array(vals) - self.finger_pos)) > 0.2:
				self.finger_pos = vals
				self.set_reflex_hand.publish(vals + [self.hand_shape]);
	
	def button_preshape(self):
		print "executing preshape"
		print self.hand_state
		print self.hand_shape
		if self.hand_state == OPEN:
			if self.hand_shape == ROT_CYL:
				self.hand_shape = ROT_SPH
				self.command_smarts('spherical')
			elif self.hand_shape == ROT_SPH:
				self.hand_shape = ROT_PINCH
				self.command_smarts('pinch')
			elif self.hand_shape == ROT_PINCH:
				self.hand_shape = ROT_CYL
				self.command_smarts('cylinder')
		elif self.hand_state == CLOSED:
			print "opening hand"
			self.open_hand()
	
	def button_set(self):
		pass
	
	def button_openloop(self):
		if self.hand_state == OPEN:	
			print "executing openloop"
			if self.hand_shape == ROT_PINCH:
				self.reflex.grasp_openloop(CLOSED_PINCH)
			else:
				self.reflex.grasp_openloop(CLOSED_PWR)
			self.hand_state = CLOSED

		elif self.hand_state == CLOSED:
			print "opening hand"
			self.open_hand()

	def button_zero(self):
		if not self.hand_state == OPEN:
			self.open_hand()
		self.zero_tactile()
		self.zero_fingers()

	def button_guarded(self):
		if self.hand_state == OPEN:
			print "executing guarded"
			#os.system('rosservice call /zero_tactile')
			self.zero_tactile()
			if self.hand_shape == ROT_PINCH:
				self.command_smarts('f1 guarded_move f2 guarded_move')
			else:
				self.command_smarts('guarded_move')
			self.hand_state = CLOSED


		elif self.hand_state == CLOSED:
			print "opening hand"
			self.open_hand()

	def button_grasp_harder(self):
		print "executing harder"
		#self.logger.log_txt('ACTION:HARDER')	
		self.command_smarts('tighten')
		self.hand_state = CLOSED

		
	def button_dof(self):
		print "executing guarded"
		self.command_smarts('dof')

if __name__ == '__main__':
		rospy.init_node('button_decipher')
		handle = Primitive_Wrapper()
		rospy.spin()

