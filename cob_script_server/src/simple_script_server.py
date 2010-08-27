#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_apps
# \note
#   ROS package name: cob_script_server
#
# \author
#   Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# \author
#   Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2010
#
# \brief
#   Implementation of ROS node for script_server.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import time
import os
import sys
import types

import roslib
roslib.load_manifest('cob_script_server')
import rospy
import actionlib
from cob_msgs.msg import *
from cob_srvs.srv import *
#from cob_actions.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from pr2_controllers_msgs.msg import *
from move_base_msgs.msg import *
from tf.transformations import *
from sound_play.libsoundplay import SoundClient
import pygraphviz as pgv

graph=""
graph_wait_list=[]

## Script class from which all script inherit.
#
# Implements basic functionalities for all scripts.
class script():
	## Dummy function for initialization
	def Initialize(self):
		pass

	## Dummy function for main run function
	def Run(self):
		pass

	## Function to start the script
	#
	# Creates a ROS node and calls Initialize() and Run().
	#
	# \param name Name of the ROS node.
	def Start(self, name):
		self.sss = simple_script_server()
		rospy.init_node(name)
		self.Initialize()
		self.Run()
		
	def Name(self):
		#return os.path.dirname(__file__)
		filename = os.path.basename(sys.argv[0])
		basename, extension = os.path.splitext(filename)
		print "filename = ", filename
		print "basename = ", basename
		print "extension = ", extension
	
	## Function to generate graph view of script.
	#
	# Starts the script in simulation mode and calls Initialize() and Run().
	#
	# \param level Level for graph generation
	def Parse(self,level):
		global graph
		self.sss = simple_script_server(level, simulate=True)
		self.Initialize()
		self.Run()
		self.graph = graph

## Simple script server class.
#
# Implements the python interface for the script server.
class simple_script_server:
	# Decides wether do use the ROS sound_play package play sound and speech or to start the services
	#	directly via command line. The command line version has the great advantage that it works!
	use_ROS_sound_play = False

	## Initializes simple_script_server class
	#
	# \param level Level for graph generation
	# \param simulate Defines wether to run script in simulation for graph generation or not
	def __init__(self, level=0, simulate=False):
		global graph
		self.ns_global_prefix = "/script_server"
		self.simulate = simulate
		self.level = level
		graph = pgv.AGraph()
		graph.node_attr['shape']='box'
#		self.represent.node_attr['fixedsize']='true'
		self.last_node = "Start"
		self.function_counter = 0
		#self.ns_global_prefix = ""
		self.soundhandle = SoundClient()
		time.sleep(1)

	def AppendGraph(self, function_name, component_name, parameter_name, blocking=True):
		global graph
		global graph_wait_list
		if type(parameter_name) is types.StringType:
			graphstring = str(self.function_counter)+"_"+function_name+"_"+component_name+"_"+parameter_name
		else:
			graphstring = str(self.function_counter)+"_"+function_name+"_"+component_name
			
		graph.add_edge(self.last_node, graphstring)
		for waiter in graph_wait_list:
			graph.add_edge(waiter, graphstring)
		graph_wait_list=[]
		ah = action_handle(simulation=True)
		if blocking:
			self.last_node = graphstring
		else:
			ah.parent_node = graphstring

		self.function_counter += 1
		return ah

#------------------- Init section -------------------#
	## Initializes different components.
	#
	# Based on the component, the corresponding init service will be called.
	#
	# \param component_name Name of the component.
	def init(self,component_name):
		self.trigger(component_name,"init")

	## Stops different components.
	#
	# Based on the component, the corresponding stop service will be called.
	#
	# \param component_name Name of the component.
	def stop(self,component_name):
		self.trigger(component_name,"stop")

	## Recovers different components.
	#
	# Based on the component, the corresponding recover service will be called.
	#
	# \param component_name Name of the component.
	def recover(self,component_name):
		self.trigger(component_name,"recover")

	## Deals with all kind of trigger services for different components.
	#
	# Based on the component and service name, the corresponding trigger service will be called.
	#
	# \param component_name Name of the component.
	# \param service_name Name of the trigger service.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def trigger(self,component_name,service_name,blocking=True):
		if(self.simulate):
			if (int(self.level) >= 1):
				return self.AppendGraph(service_name, component_name, "")
			return
		rospy.loginfo("<<%s>> <<%s>>", service_name, component_name)
		rospy.loginfo("Wait for <<%s>> to <<%s>>...", component_name, service_name)
		service_full_name = "/" + component_name + "_controller/" + service_name
		try:
			rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			return False
		try:
			init = rospy.ServiceProxy(service_full_name,Trigger)
			#print init()
			init()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return False
		rospy.loginfo("...<<%s>> is <<%s>>", component_name, service_name)
		return True

#------------------- Move section -------------------#
	## Deals with all kind of movements for different components.
	#
	# Based on the component, the corresponding move functions will be called.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move(self,component_name,parameter_name,blocking=True):
		if(self.simulate):
			return self.AppendGraph("move", component_name, parameter_name, blocking)
		rospy.loginfo("Move <<%s>> to <<%s>>",component_name,parameter_name)
		if component_name == "base":
			return self.move_base(component_name,parameter_name,blocking)
		else:
			return self.move_traj(component_name,parameter_name,blocking)

	## Deals with movements of the base.
	#
	# A target will be sent to the actionlib interface of the move_base node.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_base(self,component_name,parameter_name,blocking):
		ah = action_handle()
		ah.component_name = component_name
		ah.parameter_name = parameter_name
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.error_code = 2
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.error_code = 3
				return ah
		else:
			#print i,"type1 = ", type(i)
			DOF = 3
			if not len(param) == DOF: # check dimension
				rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,DOF,len(param))
				print "parameter is:",param
				ah.error_code = 3
				return ah
			else:
				for i in param:
					#print i,"type2 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						print type(i)
						rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						rospy.logdebug("accepted parameter %f for %s",i,component_name)

		# convert to pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "/map"
		pose.pose.position.x = param[0]
		pose.pose.position.y = param[1]
		pose.pose.position.z = 0.0
		q = quaternion_from_euler(0, 0, param[2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		
		# call action server
		action_server_name = "/move_base"
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.error_code = 4
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		self.check_pause()
		client_goal = MoveBaseGoal()
		client_goal.target_pose = pose
		#print client_goal
		self.client.send_goal(client_goal)
		ah.error_code = 0 # full success
		ah.client = self.client

		if blocking:
			rospy.logdebug("actionlib client waiting for result...")
			ah.wait()
		else:
			rospy.logdebug("actionlib client not waiting for result, continuing...")
		
		return ah

	## Deals with all kind of trajectory movements for different components.
	#
	# A trajectory will be sent to the actionlib interface of the corresponding component.
	#
	# \param component_name Name of the component.
	# \param parameter_name Name of the parameter on the ROS parameter server.
	# \param blocking Bool value to specify blocking behaviour.
	def move_traj(self,component_name,parameter_name,blocking):
		ah = action_handle()
		ah.component_name = component_name
		ah.parameter_name = parameter_name
		
		# selecting component
		if component_name == "tray":
			joint_names = ["torso_tray_joint"]
		elif component_name == "torso":
			joint_names = ["torso_lower_neck_pan_joint","torso_lower_neck_tilt_joint","torso_upper_neck_pan_joint","torso_upper_neck_tilt_joint"]
		elif component_name == "arm":
			joint_names = ["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]
		elif component_name == "sdh":
			joint_names = ["sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_11_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_21_joint", "sdh_finger_22_joint", "sdh_finger_23_joint"]
		else:
			rospy.logerr("component %s not kown to script_server",component_name)
			ah.error_code = 1
			return ah
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
				ah.error_code = 2
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check trajectory parameters
		if not type(param) is list: # check outer list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.error_code = 3
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is list: # check inner list
					rospy.logerr("no valid parameter for %s: not a list of lists, aborting...",component_name)
					print "parameter is:",param
					ah.error_code = 3
					return ah
				else:
					if not len(i) == len(joint_names): # check dimension
						rospy.logerr("no valid parameter for %s: dimension should be %d and is %d, aborting...",component_name,len(joint_names),len(i))
						print "parameter is:",param
						ah.error_code = 3
						return ah
					else:
						for j in i:
							#print j,"type2 = ", type(j)
							if not ((type(j) is float) or (type(j) is int)): # check type
								print type(j)
								rospy.logerr("no valid parameter for %s: not a list of float or int, aborting...",component_name)
								print "parameter is:",param
								ah.error_code = 3
								return ah
							else:
								rospy.logdebug("accepted parameter %f for %s",j,component_name)
		
		# convert to trajectory message
		traj = JointTrajectory()
		traj.header.stamp = rospy.Time.now()+rospy.Duration(2)
		traj.joint_names = joint_names
		point_nr = 0
		for i in param:
			point_nr = point_nr + 1
			point = JointTrajectoryPoint()
			point.positions = i
			point.time_from_start=rospy.Duration(5*point_nr) # this value is set to 5 sec per point. TODO: read from parameter
			traj.points.append(point)
		
		# call action server
		operation_mode_name = "/" + component_name + '_controller/OperationMode'
		action_server_name = "/" + component_name + '_controller/joint_trajectory_action'
		rospy.set_param(operation_mode_name, "position")
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, JointTrajectoryAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.error_code = 4
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		self.check_pause()
		client_goal = JointTrajectoryGoal()
		client_goal.trajectory = traj
		#print client_goal
		self.client.send_goal(client_goal)
		ah.error_code = 0 # full success
		ah.client = self.client

		if blocking:
			rospy.logdebug("actionlib client waiting for result...")
			ah.wait()
		else:
			rospy.logdebug("actionlib client not waiting for result, continuing...")
		
		return ah

	def move_cart_rel(self,component_name,position=[0.0, 0.0, 0.0],orientation=[0.0, 0.0, 0.0]):

		# convert to Pose message
		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now() 	
		pose.pose.position.x = position[0]
		pose.pose.position.y = position[1]
		pose.pose.position.z = position[2]
		q = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]
		print pose
		
		# call action server
		action_server_name = "/" + component_name + '_controller/move_cart_rel'
		rospy.logdebug("calling %s action server",action_server_name)
		self.client = actionlib.SimpleActionClient(action_server_name, MoveCartAction)
		# trying to connect to server
		rospy.logdebug("waiting for %s action server to start",action_server_name)
		if not self.client.wait_for_server(rospy.Duration(5)):
			# error: server did not respond
			rospy.logerr("%s action server not ready within timeout, aborting...", action_server_name)
			ah.error_code = 4
			return ah
		else:
			rospy.logdebug("%s action server ready",action_server_name)

		# sending goal
		self.check_pause()
		client_goal = MoveCartGoal()
		client_goal.trajectory = traj
		#print client_goal
		self.client.send_goal(client_goal)
		ah.error_code = 0 # full success
		ah.client = self.client

		if blocking:
			rospy.logdebug("actionlib client waiting for result...")
			ah.wait()
		else:
			rospy.logdebug("actionlib client not waiting for result, continuing...")
		
		return ah
	
	## Set the operation mode for different components.
	#
	# Based on the component, the corresponding set_operation_mode service will be called.
	#
	# \param component_name Name of the component.
	# \param mode Name of the operation mode to set.
	# \param blocking Service calls are always blocking. The parameter is only provided for compatibility with other functions.
	def set_operation_mode(self,component_name,mode,blocking=False):
		rospy.loginfo("setting <<%s>> to operation mode <<%s>>",component_name, mode)
		rospy.set_param("/" + component_name + "_controller/OperationMode",mode)
			
#------------------- LED section -------------------#
	## Set the color of the cob_light component.
	#
	# The color is given by a parameter on the parameter server.
	#
	# \param parameter_name Name of the parameter on the parameter server which holds the rgb values.
	def set_light(self,parameter_name):
		if(self.simulate):
			return self.AppendGraph("LED", "", parameter_name)
		rospy.loginfo("Set light to %s",parameter_name)
		pub = rospy.Publisher('light_controller/command', Light)
		time.sleep(0.5) # we have to wait here until publisher is ready, don't ask why
		
		# get joint values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/light/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/light/" + parameter_name)
				return 2
			param = rospy.get_param(self.ns_global_prefix + "/light/" + parameter_name)
		else:
			param = parameter_name
			
		# check color parameters
		if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for light: not a list, aborting...")
			print "parameter is:",param
			return 3
		else:
			if not len(param) == 3: # check dimension
				rospy.logerr("no valid parameter for light: dimension should be 3 (r,g,b) and is %d, aborting...",len(param))
				print "parameter is:",param
				return 3
			else:
				for i in param:
					#print i,"type1 = ", type(i)
					if not ((type(i) is float) or (type(i) is int)): # check type
						print type(i)
						rospy.logerr("no valid parameter for light: not a list of float or int, aborting...")
						print "parameter is:",param
						return 3
					else:
						rospy.logdebug("accepted parameter %f for light",i)
		
		# convert to light message
		color = Light()
		color.header.stamp = rospy.Time.now()
		if type(parameter_name) is str:
			color.name.data = parameter_name
		else:
			color.name.data = "unspecified"
		color.r = param[0]
		color.g = param[1]
		color.b = param[2]

		# publish color		
		pub.publish(color)
		
		return 0 # full success

#-------------------- Sound section --------------------#
	## Say some text.
	#
	# The text to say may be given by a list of strings or a single string which points to a parameter on the ROS parameter server.
	#
	# \param parameter_name Name of the parameter
	# \param language Language to use for the TTS system
	def say(self,parameter_name,language="en"):
		component_name = "sound"
		ah = action_handle()
		ah.component_name = component_name
		ah.parameter_name = parameter_name
		text = ""
		
		rospy.loginfo("Saying <<%s>>",parameter_name)
		
		# get values from parameter server
		if type(parameter_name) is str:
			if not rospy.has_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
				ah.error_code = 2
				return ah
			param = rospy.get_param(self.ns_global_prefix + "/" + component_name + "/" + language + "/" + parameter_name)
		else:
			param = parameter_name
		
		# check parameters
		if not type(param) is list: # check list
				rospy.logerr("no valid parameter for %s: not a list, aborting...",component_name)
				print "parameter is:",param
				ah.error_code = 3
				return ah
		else:
			for i in param:
				#print i,"type1 = ", type(i)
				if not type(i) is str:
					rospy.logerr("no valid parameter for %s: not a list of strings, aborting...",component_name)
					print "parameter is:",param
					ah.error_code = 3
					return ah
				else:
					text = text + i + " "
					rospy.logdebug("accepted parameter <<%s>> for <<%s>>",i,component_name)
		#print text
		self.soundhandle.say(text)

	def Speak(self,parameter_name,mode="DEFAULT"):
		if(self.simulate):
			return self.AppendGraph("Speak", "", parameter_name)

		""" Speak sound specified by 'parameter_name' either via TTS or by playing a WAV-File
		Possible modes are:
		DEFAULT - use mode set by a global parameter (default)
		WAV_DE	- play wav-Files with German Text
		WAV_EN	- play wav-FIles with English Text
		FEST_EN	- use Text-to-speech with the English Festival voice
		CEPS_EN	- use Text-to-speech with the English Cepstral voice David
		CEPS_DE	- use Text-to-speech with the German Cepstral voice Matthias
		MUTE	- play no sound at all
		"""
		rospy.logdebug("Speak <<%s>> in mode <<%s>>",parameter_name,mode)
		ah = action_handle()
		ah.parameter_name = parameter_name
		
		# get mode from global parameter if necessary
		if mode == "DEFAULT":
			if not rospy.has_param(self.ns_global_prefix + "/sound/speech_mode"):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/speech_mode")
				ah.error_code = 2
				return ah
			mode = rospy.get_param(self.ns_global_prefix + "/sound/speech_mode")
		
		# play sound depending on the mode that was chosen
		if mode == "WAV_DE":
			rospy.loginfo("Playing German WAV file %s",parameter_name)
			
			# get path for German WAV files
			if not rospy.has_param(self.ns_global_prefix + "/sound/wav_de_path"):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/wav_de_path")
				ah.error_code = 2
				return ah
			wav_path = rospy.get_param(self.ns_global_prefix + "/sound/wav_de_path")
			
			# play sound
			rospy.loginfo("Playing file %s",wav_path + parameter_name + ".wav")
			if self.use_ROS_sound_play:
				self.soundhandle.playWave(wav_path + parameter_name + ".wav")
				ah.error_code = 0
			else:
				retVal = os.system("aplay -q " + wav_path + parameter_name + ".wav")
				if retVal == 127:
					rospy.logerr("Calling audio player 'aplay' caused a failure. Check if it is installed and works properly!")
					ah.error_code = 4
				elif retVal == 1:
					rospy.logerr("Calling audio player 'aplay' caused a failure. Check if wave file is existing and the path is valid!")
					ah.error_code = 3
				else:
					ah.error_code = 0
			return ah 
			
		elif mode == "WAV_EN":
			rospy.loginfo("Playing English WAV file %s",parameter_name)
			
			# get path for English WAV files
			if not rospy.has_param(self.ns_global_prefix + "/sound/wav_en_path"):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/wav_en_path")
				ah.error_code = 2
				return ah
			wav_path = rospy.get_param(self.ns_global_prefix + "/sound/wav_en_path")
			
			# play sound
			rospy.loginfo("Playing file %s",wav_path + parameter_name + ".wav")
			if self.use_ROS_sound_play:
				self.soundhandle.playWave(wav_path + parameter_name + ".wav")
				ah.error_code = 0
			else:
				retVal = os.system("aplay -q " + wav_path + parameter_name + ".wav")
				if retVal == 127:
					rospy.logerr("Calling audio player 'aplay' returned a failure. Check if it is installed and works properly!")
					ah.error_code = 4
				elif retVal == 1:
					rospy.logerr("Calling audio player 'aplay' returned a failure. Check if wave file is existing and the path is valid!")
					ah.error_code = 3
				else:
					ah.error_code = 0
			return ah 
			
		elif mode == "FEST_EN":
			# get the text string to speak
			if not rospy.has_param(self.ns_global_prefix + "/sound/speech_en/"+parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/speech_en/"+parameter_name)
				ah.error_code = 2
				return ah 
			text_string = rospy.get_param(self.ns_global_prefix + "/sound/speech_en/"+parameter_name)
			
			# send text string to TTS system
			ah.error_code = self.SpeakStr(text_string,mode)
			return ah
	
		elif mode == "CEPS_EN":
			# get the text string to speak
			if not rospy.has_param(self.ns_global_prefix + "/sound/speech_en/"+parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/speech_en/"+parameter_name)
				ah.error_code = 2
				return ah 
			text_string = rospy.get_param(self.ns_global_prefix + "/sound/speech_en/"+parameter_name)
			
			# send text string to TTS system
			ah.error_code = self.SpeakStr(text_string,mode)
			return ah

		elif mode == "CEPS_DE":
			# get the text string to speak
			if not rospy.has_param(self.ns_global_prefix + "/sound/speech_de/"+parameter_name):
				rospy.logerr("parameter %s does not exist on ROS Parameter Server, aborting...",self.ns_global_prefix + "/sound/speech_de/"+parameter_name)
				ah.error_code = 2
				return ah 
			text_string = rospy.get_param(self.ns_global_prefix + "/sound/speech_de/"+parameter_name)
			
			# send text string to TTS system
			ah.error_code = self.SpeakStr(text_string,mode)
			return ah

		elif mode == "MUTE":
			rospy.loginfo("Playing sound %s (muted)",parameter_name)
			ah.error_code = 0
			return ah
		
		else:
			rospy.logerr("ROS has no sound mode %s!",mode)
			ah.error_code = 2
			return ah

	def SpeakStr(self,text,mode):
		if(self.simulate):
			return self.AppendGraph("SpeakStr", text, mode)
	
		""" Speak the string 'text' via the TTS system specified by mode
		Possible modes are:
		FEST_EN	- use Text-to-speech with the English Festival voice
		CEPS_EN	- use Text-to-speech with the English Cepstral voice David
		CEPS_DE	- use Text-to-speech with the German Cepstral voice Matthias
		MUTE	- play no sound at all
		"""
		# verify that argument 'text' is a string
		if not type(text) == str:
			rospy.logerr("no valid parameter for text-to-speech system: Not a string, aborting...")
			return 3
		
		# get parameter for temporary wav file
		param_name = self.ns_global_prefix +"/sound/temp_wav_file"
		if not rospy.has_param(param_name):
			rospy.logerr("parameter <<%s>> does not exist on ROS Parameter Server, aborting...",param_name)
			return 2
		temp_wav_file = rospy.get_param(self.ns_global_prefix +"/sound/temp_wav_file")

		# play sound depending on the mode that was chosen
		if mode == "FEST_EN":
			rospy.loginfo("Using English Festival Voice for speaking '%s'",text)
			
			# send text string to TTS system
			if self.use_ROS_sound_play:
				self.soundhandle.say(text)
				return 0
			else:
				retVal = os.system("echo "+text+" | text2wave | aplay -q")
				if retVal != 0:
					rospy.logerr("calling Festival TTS system returned failure. Check if it is installed and works properly!")
					return 4
				else:
					return 0	

		elif mode == "CEPS_EN":
			rospy.loginfo("Using English Cepstral Voice David for speaking '%s'",text)
			
			# send text string to TTS system
			retVal = os.system("swift -n \"David\" -e \"utf-8\" \"" + text + "\" -o " + temp_wav_file)
			if retVal != 0:
				rospy.logerr("Calling Cepstral TTS system returned failure. Check if Cepstral voice \"David\" is set up properly!")
				return 4
			retVal = os.system("aplay -q " + temp_wav_file)
			if retVal == 127:
				rospy.logerr("Calling audio player 'aplay' returned a failure. Check if it is installed and works properly!")
				return 4
			elif retVal == 1:
				rospy.logerr("Calling audio player 'aplay' returned a failure. Check the directory for temporary file is existing and has write access!")
				return 3
			else:
				return 0

		elif mode == "CEPS_DE":
			rospy.loginfo("Using German Cepstral Voice Matthias for speaking '%s'",text)
			
			# send text string to TTS system
			retVal = os.system("swift -n \"Matthias\" -e \"utf-8\" \"" + text + "\" -o " + temp_wav_file)
			if retVal != 0:
				rospy.logerr("Calling Cepstral TTS system returned failure. Check if Cepstral voice \"Matthias\" is set up properly!")
				return 4
			retVal = os.system("aplay -q " + temp_wav_file)
			if retVal == 127:
				rospy.logerr("Calling audio player 'aplay' returned a failure. Check if it is installed and works properly!")
				return 4
			elif retVal == 1:
				rospy.logerr("Calling audio player 'aplay' returned a failure. Check the directory for temporary file is existing and has write access!")
				return 3
			else:
				return 0
			return 0

		elif mode == "MUTE":
			rospy.loginfo("Playing sound %s (muted)",text)
			return 0

		else:
			rospy.logerr("ROS has no sound mode %s!",mode)
			return 2

#------------------- General section -------------------#
	## Sleep for a certain time.
	#
	# \param duration Duration in seconds to sleep.
	#
	def sleep(self,duration):
		if(self.simulate):
			if (int(self.level) >= 2):
				return self.AppendGraph("sleep", "", 0)
			return
		rospy.loginfo("Wait for %f sec",duration)
		time.sleep(duration)

	## Waits for user input.
	#
	# Waits either for a user input or until timeout is reached.
	#
	# \param duration Duration in seconds for timeout.
	# 
	# \todo implement waiting for timeout
	def wait_for_input(self,duration=0):
		if(self.simulate):
			if (int(self.level) >= 2):
				return self.AppendGraph("Wait_for_input", "", 0)
			return
		rospy.loginfo("Wait for user input...")
		retVal = sys.stdin.readline()
		rospy.loginfo("Got string >%s<",retVal)
		return retVal
		#key = input()
		#return key

	## Checks if script is in pause mode
	#
	# Check if pause is globally set. If yes, enter a wait loop until the parameter is reset.
	# 
	# \todo check if pause is working
	def check_pause(self):
		pause_was_active = False

		while rospy.get_param("/script_server/pause"):
			if not pause_was_active:
				rospy.loginfo("ActionServer set to pause mode. Waiting for resume...")
				pause_was_active = True
			time.sleep(1)

		if pause_was_active:
			rospy.loginfo("Resuming...")
			return 1
		else:
			return 0

#------------------- action_handle section -------------------#	
## Action handle class.
#
# The action handle is used to implement asynchronous behaviour within the script.
class action_handle:
	## Initializes the action handle.
	def __init__(self, simulation=False):
		self.error_code = -1
		self.component_name = None
		self.parameter_name = None
		self.simulation = simulation
		self.parent_node = ""

	## Waits for the action to be finished.
	#
	# If duration is specified, waits until action is finished or timeout is reached.
	#
	# \param duration Duration for timeout.
	def wait(self,duration=None):
		global graph_wait_list
		if(self.simulation):
			if(self.parent_node != ""):
				graph_wait_list.append(self.parent_node)
			return
		if self.error_code == 0:			
			if duration is None:
				rospy.loginfo("Wait for <<%s>> reaching <<%s>>...",self.component_name, self.parameter_name)
				self.client.wait_for_result()
			else:
				rospy.loginfo("Wait for <<%s>> reached <<%s>> (max %f secs)...",self.component_name, self.parameter_name,duration)
				if not self.client.wait_for_result(rospy.Duration(duration)):
					rospy.logerr("Timeout while waiting for <<%s>> to reach <<%s>>. Continuing...",self.component_name, self.parameter_name)
					error_code = 10
					return
			rospy.loginfo("...<<%s>> reached <<%s>>",self.component_name, self.parameter_name)
		else:
			rospy.logwarn("Execution of action was aborted, wait not possible. Continuing...")
		return self.error_code

	## Gets the error code for a action execution.
	def get_error_code(self):
		return self.error_code
