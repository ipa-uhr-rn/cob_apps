#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *
import time


class RobertsTest(script):

	def Initialize(self):
		
		# assign listener
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))

		#add milk box to environment
		self.sss.detach_object("milk_box")
		self.sss.add_object("milk_box")
		self.sss.add_object("table_full")

		# initialize components (not needed for simulation)
		self.sss.init("torso")
		self.sss.init("tray")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("head")
		self.sss.init("base")

		# move to initial positions
		handle_torso = self.sss.move("torso", "back", False)
		handle_tray = self.sss.move("tray", "down", False)
		handle_sdh = self.sss.move("sdh", "cylclosed", False)
		handle_head = self.sss.move("head", "back", False)

		# wait for initial movements to finish
		handle_torso.wait()
		handle_tray.wait()
		handle_sdh.wait()
		handle_head.wait()

		handle_arm = self.sss.move("arm", "folded", True, "planned")
		handle_arm.wait()

		# localize the robot with rviz
#		if not self.sss.parse:
#			print "Please localize the robot with rviz"
#		self.sss.wait_for_input()

		# move base to initial position
#		self.handle_base = self.sss.move("base", "kitchen")


	def Run(self):

		#define if planned or unplanned motion
		mode = "planned"
		#mode = "None"

		# move arm to pregrasp position
		handle_arm = self.sss.move("arm","pregrasp", False, mode)
		handle_arm.wait()
		handle_arm = self.sss.move("arm", "pre_milk", False, mode)
		handle_arm.wait()
		self.sss.move("sdh", "cylopen", True)

		#grab milk but first remove milkbox from collision space
		self.sss.remove_object("milk_box")
		self.sss.move("arm", "grab_milk")
		self.sss.move("sdh", "box_grasp", True)

#cart_rel[[z,x,y],[oritation]]

		# Prepare to move milk box
		handle_torso = self.sss.move("torso", "home", False)
		handle_tray = self.sss.move("tray", "up", False)
		self.sss.move_cart_rel("arm",[[0.2,0.0,0.0],[0.0,0.0,0.0]])
		handle_tray.wait()

		#re-add milkbox and attach to the robot
		self.sss.add_object("milk_box")
		self.sss.attach_object("milk_box")

		#move milk box
		self.sss.move("arm", "overtray", True, mode)

		# put milk box onto tray
		self.sss.move("arm","tray")
	#	self.sss.move_cart_rel("arm", [[0.0, 0.0, -0.05], [0, 0, 0]])
		self.sss.move("sdh", "cylopen")
		self.sss.move_cart_rel("arm",[[0.0,0.0,-0.1],[0,0,0]])

		#detach milk box from robot (add is here to ensure no discrepencies occure with placement)
#		self.sss.detach_object("milk_box")
#		self.sss.add_object("milk_box")

		# move arm to folded position
#		self.sss.move_cart_rel("arm", [[0.1, 0.0, -0.1], [0, 0, 0]])
#		self.sss.move("sdh", "cylclosed",False)
#		self.sss.move("arm","folded",True,mode)

		#move to tray
#		self.sss.move("sdh","cylopen",False)
#		self.sss.move("arm","overtray",True,mode)		
		
		#pickup milk and remove from collision space
#		self.sss.remove_object("milk_box")
#		self.sss.move("arm","overtray",True,mode)
#		self.sss.move_cart_rel("arm",[[0.0,0.00,0.02],[0,0,0]])
#		self.sss.move("sdh","cylclosed")
	
		#attach milk box in colision space
#		self.sss.move_cart_rel("arm",[[0.1,0.0,-0.1],[0,0,0]])
#		self.sss.add_object("milk_box")
#		self.sss.attach_object("milk_box")

		#move milk to floor
#		self.sss.move("arm","pre_drop", True, mode)
#		self.sss.move("arm","drop",True,mode)
		
		#drop milk....detach from robot too
#		self.sss.move("sdh","cylopen")
#		self.sss.detach_object("milk_box")
#		self.sss.add_object("milk_box")
#		self.sss.move("arm","pre_drop", True, mode)

		#return to folded position
#		self.sss.move("sdh","cylclosed",False)
#		self.sss.move("arm","folded", True, mode)

		#pickup milk from floor
#		self.sss.move("sdh","cylopen",False)
#		self.sss.move("arm","pre_drop", True, mode)
#		self.sss.remove_object("milk_box")
#		self.sss.move("arm","drop", True, mode)
#		self.sss.move("sdh","cylclosed")
#		self.sss.add_object("milk_box")
#		self.sss.attach_object("milk_box")

		#place milk on table
#		self.sss.move("arm","pre_drop",True,mode)
#		self.sss.move("arm","grab_milk", True, mode)
#		self.sss.move("sdh","cylopen")
#		self.sss.detach_object("milk_box")
#		self.sss.add_object("milk_box")
	
		#return to folded position
#		self.sss.move_cart_rel("arm",[[0.1,0.0,0.1],[0,0,0]])
#		self.sss.move("sdh","cylclosed",False)
#		self.sss.move("arm","folded", True, mode)


if __name__ == "__main__":
	SCRIPT = RobertsTest()
	SCRIPT.Start()
