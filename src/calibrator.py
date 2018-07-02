#!/usr/bin/env python
# coding=utf-8

# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy 

import time, threading, sys

from robotnik_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from std_srvs.srv import Trigger
from imu_calibrator.msg import CalibratorStatus
from robotnik_msgs.srv import enable_disable
from mavros_msgs.msg import State as MavrosState

DEFAULT_FREQ = 100.0
DEFAULT_BIAS = 0.01
# Time than the robot has to be stopped to starting the calibration process
DEFAULT_STOPPEDTIME = 10
# Time than the gyroscope is checking his bias
DEFAULT_GYROTIME = 0.5
# Time between calibrations
DEFAULT_STANDBYTIME = 300
MAX_FREQ = 500.0

# Estados añadidos
CHECKMOVEMENT_STATE = 800
CHECKGYRO_STATE = 900


class Calibrator:
	
	def __init__(self, args):
		
		self.node_name = rospy.get_name() #.replace('/','')
		self.desired_freq = args['desired_freq']
		self.desired_bias = args['desired_bias']
		self.desired_stoppedtime = args['desired_stoppedtime']
		self.desired_gyrotime = args['desired_gyrotime']
		self.desired_standbytime = args['desired_standbytime']
		self.enable = enable_disable()
		self.enable.value = True
		self.disable = enable_disable()
		self.disable.value = False
		self.calibracionTerminada = True
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
		if self.desired_bias <= 0.0:
			rospy.loginfo('%s::init: Desired bias (%f) is not possible. Setting desired_bias to %f'%(self.node_name,self.desired_bias, DEFAULT_BIAS))
			self.desired_bias = DEFAULT_BIAS
		if self.desired_stoppedtime <= 0.0:
			rospy.loginfo('%s::init: Desired stopped time (%f) is not possible. Setting desired_stoppedtime to %f'%(self.node_name,self.desired_stoppedtime, DEFAULT_STOPPEDTIME))
			self.desired_stoppedtime = DEFAULT_STOPPEDTIME
		if self.desired_gyrotime <= 0.0:
			rospy.loginfo('%s::init: Desired gyro time (%f) is not possible. Setting desired_gyrotime to %f'%(self.node_name,self.desired_gyrotime, DEFAULT_GYROTIME))
			self.desired_gyrotime = DEFAULT_GYROTIME
		if self.desired_standbytime <= 0.0:
			rospy.loginfo('%s::init: Desired standby time (%f) is not possible. Setting desired_standbytime to %f'%(self.node_name,self.desired_standbytime, DEFAULT_STANDBYTIME))
			self.desired_standbytime = DEFAULT_STANDBYTIME
	
		self.status = CalibratorStatus()
		self.status.state = "idle"
		self.status.remaining_time = self.desired_stoppedtime

		self.real_freq = 0.0
		
		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1
		
		# The robot is changing his position
		self.mov = True
		# Position of the robot (linear.x, angular.z)
		self.pos = [0,0]
		# The robot needs calibration
		self.calibrate = False
		# Last inclination of the robot
		self.incMin = [100,100,100]
		self.incMax = [-100,-100,-100]
		# True if the robot has been stopped the required time
		self.checkedMovement = False
		# Time when the search has started
		self.t1 = 0.0
		
		# self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		
			
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True
		
		return 0

	
	def movimiento(self,data):
		aux = self.pos

		x = round(data.twist.twist.linear.x, 5)
		z = round(data.twist.twist.angular.z, 5)
		self.pos = [x, z]

		if self.pos != aux:
			self.mov = True


	def giroscopio(self,data):
		x = data.vector.x
		y = data.vector.y
		z = data.vector.z

		self.incMin = [min(x,self.incMin[0]),min(y,self.incMin[1]),min(z,self.incMin[2])]
		self.incMax = [max(x,self.incMax[0]),max(y,self.incMax[1]),max(z,self.incMax[2])]

		if abs(self.incMin[0]-self.incMax[0])>=self.desired_bias or abs(self.incMin[1]-self.incMax[1])>=self.desired_bias or abs(self.incMin[2]-self.incMax[2])>=self.desired_bias:
			self.calibrate = True

	def calibrado(self,data):
		if data.system_status == 0:
			self.calibracionTerminada = False
		else:
			self.calibracionTerminada = True

	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
		
		# Publishers
		self.state_publisher = rospy.Publisher('calibrator/status', CalibratorStatus, queue_size=10)
		# Subscribers
		self.odom = rospy.Subscriber('robotnik_base_control/odom', Odometry, self.movimiento)
		self.gyro = rospy.Subscriber('imu/rpy/filtered', Vector3Stamped, self.giroscopio)
		self.calibrated = rospy.Subscriber('mavros/state', MavrosState, self.calibrado)
		# Service Servers
		rospy.wait_for_service('calibrate_imu_gyro')
		rospy.wait_for_service('robotnik_base_control/enable')
		self.calibrateService = rospy.ServiceProxy('calibrate_imu_gyro', Trigger)
		self.enableController = rospy.ServiceProxy('robotnik_base_control/enable', enable_disable)
		# self.service_server = rospy.Service('~service', Empty, self.serviceCb)
		# Service Clients
		self.enable_controller_service_client = rospy.ServiceProxy('enable_controller', enable_disable)
		# ret = self.service_client.call(ServiceMsg)
		
		self.ros_initialized = True
		
		# self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)
		
		# Cancels current timers
		
		self.state_publisher.unregister()
		
		self.initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		# Performs ROS topics & services shutdown
		self.state_publisher.unregister()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()

			elif self.state == CHECKMOVEMENT_STATE:
				self.checkMovementState()

			elif self.state == CHECKGYRO_STATE:
				self.checkGyroState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			
			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)
		
		return 0
		
		
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
					
		return 0
		
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		if not self.initialized:
			self.setup()
		else:
			if self.checkedMovement == True:
				detenido = self.enableController(self.disable.value)
				if detenido == False:
					self.checkedMovement = False
				else:
					self.calibrate = False
					self.incMin = [100,100,100]
					self.incMax = [-100,-100,-100]
			else:
				self.enableController(self.enable.value)
				self.mov = False

			self.t1 = time.time()
			self.switchToState(State.READY_STATE)

		return
	
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.enableController(self.enable.value)
		for i in reversed(range(self.desired_standbytime)):
				self.status.remaining_time = i
				self.state_publisher.publish(self.status)
				rospy.sleep(1)

		self.status.state = "idle"
		self.state_publisher.publish(self.status)
		self.status.remaining_time = self.desired_stoppedtime
		self.switchToState(State.INIT_STATE)
		
		return
	
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		if self.checkedMovement:
			self.switchToState(CHECKGYRO_STATE)
		else:
			self.switchToState(CHECKMOVEMENT_STATE)
		return
		
	
	def checkMovementState(self):

		tdiff = time.time() - self.t1


		if self.mov:
			self.switchToState(State.INIT_STATE)
		else:
			if tdiff >= self.desired_stoppedtime:
				self.checkedMovement = True
				self.status.state = "checking"
				self.status.remaining_time = self.desired_gyrotime
				self.state_publisher.publish(self.status)
				self.switchToState(State.INIT_STATE)
			else:
				self.status.remaining_time = round(self.desired_stoppedtime-tdiff,4)
				self.status.state = "idle"
				self.state_publisher.publish(self.status)
				self.switchToState(State.READY_STATE)

		return


	def checkGyroState(self):

		tdiff = time.time() - self.t1

		if self.mov == True:
			self.checkedMovement = False
			self.status.state = "idle"
			self.status.remaining_time = self.desired_stoppedtime
			self.state_publisher.publish(self.status)
			self.switchToState(State.INIT_STATE)

		if self.calibrate:
			self.calibracionTerminada = False
			self.status.state = "calibrating"
			rospy.loginfo("Calibrating, this can take a while. Please, don't move the robot during this process.")
			self.calibrateService()
			for i in reversed(range(60)):
				self.status.remaining_time = i
				self.state_publisher.publish(self.status)
				rospy.sleep(1)
				if self.calibracionTerminada:
					break
			self.switchToState(State.INIT_STATE)
		else:
			if tdiff >= self.desired_gyrotime:
				self.checkedMovement = False
				self.status.state = "standby"
				self.status.remaining_time = self.desired_standbytime
				self.state_publisher.publish(self.status)
				self.switchToState(State.STANDBY_STATE)
			else:
				self.status.remaining_time = round(self.desired_gyrotime-tdiff,2)
				self.status.state = "checking"
				self.state_publisher.publish(self.status)
				self.switchToState(State.READY_STATE)
		
		return

	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		
			
		return
	
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			#rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))
		
		return
	
		
	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'

		elif state == CHECKMOVEMENT_STATE:
			return 'CHECKMOVEMENT_STATE'

		elif state == CHECKGYRO_STATE:
			return 'CHECKGYRO_STATE'

		else:
			return 'UNKNOWN_STATE'
	
		
	# def publishROSstate(self):
	# 	'''
	# 		Publish the State of the component at the desired frequency
	# 	'''
	# 	self.msg_state.state = self.state
	# 	self.msg_state.state_description = self.stateToString(self.state)
	# 	self.msg_state.desired_freq = self.desired_freq
	# 	self.msg_state.real_freq = self.real_freq
	# 	self._state_publisher.publish(self.msg_state)
		
	# 	self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
	# 	self.t_publish_state.start()
	
	"""
	def topicCb(self, msg):
		'''
			Callback for inelfe_video_manager state
			@param msg: received message
			@type msg: std_msgs/Int32
		'''
		# DEMO
		rospy.loginfo('Calibrator:topicCb')
	
	def serviceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: std_srv/Empty
		'''
		# DEMO
		rospy.loginfo('Calibrator:serviceCb')
	"""	
		
def main():

	rospy.init_node("calibrator")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'desired_bias' : DEFAULT_BIAS,
	  'desired_stoppedtime' : DEFAULT_STOPPEDTIME,
	  'desired_gyrotime' : DEFAULT_GYROTIME,
	  'desired_standbytime' : DEFAULT_STANDBYTIME
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the param has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))
	
	rc_node = Calibrator(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
