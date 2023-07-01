#!/usr/bin/python3

#Code initiated on 20 November 2020
#Comments added on 27 January 2021
#SwarmAPI restructured on 1 October 2021
#SwarmAPI reinstalled on Drone Demo PC on 1 July 2023

#Saved under https://github.com/ThomasCarstens/cfScripts/blob/master/ros_action_server.py
#Tutorial: https://github.com/ThomasCarstens/txa-dvic-projects-tutos/tree/main/Behaviour%20Planning%20with%20ROS

#DEBUGGING: use rospy.loginfo() and rospy.logfatal

import numpy as np
import rospy
from rospy import Duration

import tf
from geometry_msgs.msg import TransformStamped, Point, Pose
import time
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import sys
import signal
import os
from pycrazyswarm import *
import uav_trajectory
import actionlib
import actionlib_tutorials.msg
#import actionlib_tutorials.srv
from math import pow, atan2, sqrt


import pyttsx3
global engine
engine = pyttsx3.init()

"""CTRL-C PREEMPTION"""
def signal_handler(signal, frame):
	sys.exit(0)

"""AUDIO INTEGRATION"""
def speak(engine, text):
    engine.say(text)
    engine.runAndWait()

"""INITIATED BY MAIN"""
class PerimeterMonitor(object):

    def __init__(self, name):
        print("name is", name)
        self.uniqueid= time.time()
        time.sleep(0.1)
	
        #######################
	#Exclusive use of Python API (exclusive=cannot be shared between processes) (txa#007)
        self.swarm = Crazyswarm()
        self.allcfs = self.swarm.allcfs
        self.timeHelper = self.swarm.timeHelper
        speak (engine, "Action server activated.")

	#ROS Point message structure for ROS pose updates.
        self.cf1_pose = Point()
        self.cf2_pose = Point()
        self.cf3_pose = Point()
        self.cf4_pose = Point()
        self.cf5_pose = Point()
        self.cf6_pose = Point()
        self.desired_position = Point()
        self._feedback_cf2 = actionlib_tutorials.msg.DoTrajFeedback()

        #Enable ("arm") the active drones that you wish to fly (txa#001)
        self.arm = True
        self.enable_cf1 = True
        self.enable_cf2 = True
        self.enable_cf3 = True
        self.enable_cf4 = True
        self.enable_cf5 = True
        self.enable_cf6 = True
        # Note: currently only using ARM for all drones

        # Class variables common to all drones (txa#002)
        self.success = False
        self.collision_tolerance = 0.2
        self.sleeptime = 4.0
        self.justenoughsleeptime = 0.005
        self.counter = 0
        #self.target = np.array()

        # Get these values from parameter server (txa#003)
        heli_param = rospy.get_param("~helicoidale", None)
        fig8_param = rospy.get_param("~figure8_smooth", None)
        
	# Upload as variables, called according to action parameters (txa#003)
        self.heli = uav_trajectory.Trajectory()
        self.heli.loadcsv(heli_param)

        self.fig8 = uav_trajectory.Trajectory()
        self.fig8.loadcsv(fig8_param)


################################################################################
		# Action server initialisation. 

#	ACTION: PREDEFINED TRAJECTORY
		#DRONE [goal.id] executes Traj [goal.shape]
        #trajectory_action ACTION SERVER [name='trajectory_action', drone: self.allcfs.crazyflies[0], variable:'_as2', callback:'traj_callback']"""
        self._feedback2 = actionlib_tutorials.msg.DoTrajFeedback()
        self._result2 = actionlib_tutorials.msg.DoTrajResult()
        self._action_name2 = 'trajectory_action'
        print (self._action_name2)
        self._as2 = actionlib.SimpleActionServer(self._action_name2, actionlib_tutorials.msg.DoTrajAction, execute_cb=self.traj_callback, auto_start = False)
        self._as2.start()
        print("READY: CSV TRAJECTORY ACTION.")

        #KEEPING THIS CODE UNTIL PREDEFINEDTRAJECTORY IS VERIFIED.
        self._feedbackfig8 = actionlib_tutorials.msg.DoTrajFeedback()
        self._resultfig8 = actionlib_tutorials.msg.DoTrajResult()
        self._action_name_fig8 = 'fig8_'
        print (self._action_name_fig8)
        self._asfig8 = actionlib.SimpleActionServer(self._action_name_fig8, actionlib_tutorials.msg.DoTrajAction, execute_cb=self.drones_fig8_callback, auto_start = False)
        self._asfig8.start()
        print("READY: FIG8.")

#	ACTION: LAND (AND CONFIRM IF ALIVE)
#		MOVE DRONE [goal.id] to Point [goal.point] - query arrival at 200Hz - report on duration [goal.time_elapsed]

#       [name='land_', drone: goal.id, variable:'_cf2', callback:'execute_cb_cf3_go']

        self._feedback_cf3_go = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf3_go = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf3_go = 'land_'
        print (self._action_name_cf3_go)
        self._as_cf3_go = actionlib.SimpleActionServer(self._action_name_cf3_go, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf3_go, auto_start = False)
        self._as_cf3_go.start()
        print("READY: LAND ACTION.")

#	ACTION: FLY TO WAYPOINT  
#		MOVE DRONE [goal.id] to Point [goal.point] - query arrival within loop - report on duration [goal.time_elapsed]

#       [name='detect_perimeter', drone: goal.id, variable:'_to_waypoint', callback:'to_waypoint']

        self._feedback_to_waypoint = actionlib_tutorials.msg.my_newFeedback()
        self._result_to_waypoint = actionlib_tutorials.msg.my_newResult()
        self._action_name_to_waypoint = 'detect_perimeter'
        print (self._action_name_to_waypoint)
        self._as_to_waypoint = actionlib.SimpleActionServer(self._action_name_to_waypoint, actionlib_tutorials.msg.my_newAction, execute_cb=self.to_waypoint, auto_start = False)
        self._as_to_waypoint.start()
        print("READY: FLY TO WAYPOINT ACTION.")

#	ACTION: FOLLOW-ME 
#		MOVE DRONE [goal.id] to Point [goal.point] (currently cf2) - query arrival at 200Hz - report on duration [goal.time_elapsed]
#       [name='cf3_follow_cf2', drone: goal.id, variable:'_cf3_follow_cf2', callback:'execute_cb_cf3_follow_cf2']

        self._feedback_cf3 = actionlib_tutorials.msg.my_newFeedback()
        self._result_cf3 = actionlib_tutorials.msg.my_newResult()
        self._action_name_cf3 = 'cf3_follow_cf2'
        print (self._action_name_cf3)
        self._as_cf3 = actionlib.SimpleActionServer(self._action_name_cf3, actionlib_tutorials.msg.my_newAction, execute_cb=self.execute_cb_cf3_follow_cf2, auto_start = False)
        self._as_cf3.start()
        print("FUNCTION: FOLLOW-ME ACTION.")



#		FUNCTION: RANDOM WALK
#		MOVE DRONE [goal.id] to Point [goal.point] (currently random) - query arrival at 200Hz - report on duration [goal.time_elapsed]
#       [name='random_walk', drone: goal.id, variable:'_rw', callback:'rw_callback']
        self._feedback_rw = actionlib_tutorials.msg.DoTrajFeedback()
        self._result_rw = actionlib_tutorials.msg.DoTrajResult()
        self._action_name_rw = 'random_walk'
        print (self._action_name_rw)
        self.success_rw  = True
        self._as_rw = actionlib.SimpleActionServer(self._action_name_rw, actionlib_tutorials.msg.DoTrajAction, execute_cb=self.rw_callback, auto_start = False)
        self._as_rw.start()
        print("Ready for Random Walk.")

#		"""safe down
#		MOVE DRONE x goal.id
#        cf3_follow_cf2 ACTION SERVER [name='safe_down', drone: goal.id, variable:'_cf3_follow_cf2', callback:'execute_cb_cf3_follow_cf2']"""
        self._feedback_sd = actionlib_tutorials.msg.DoTrajFeedback()
        self._result_sd = actionlib_tutorials.msg.DoTrajResult()
        self._action_name_sd = 'safe_down'
        print (self._action_name_sd)
        self._as_sd = actionlib.SimpleActionServer(self._action_name_sd, actionlib_tutorials.msg.DoTrajAction, execute_cb=self.safe_down, auto_start = False)
        self._as_sd.start()
        print("Ready for Random Walk.")

#		"""my hover
#		MOVE DRONE x goal.id
#        cf3_follow_cf2 ACTION SERVER [name='cf3_follow_cf2', drone: goal.id, variable:'_cf3_follow_cf2', callback:'execute_cb_cf3_follow_cf2']"""
        self._feedback_mh = actionlib_tutorials.msg.DoTrajFeedback()
        self._result_mh = actionlib_tutorials.msg.DoTrajResult()
        self._action_name_mh = 'ready'
        print (self._action_name_mh)
        self._as_mh = actionlib.SimpleActionServer(self._action_name_mh, actionlib_tutorials.msg.DoTrajAction, execute_cb=self.my_hover, auto_start = False)
        self._as_mh.start()
        print("Ready for Random Walk.")

        #SERVICES: Check if Services work on Action Server (ROS2 functionality)
        #self.setupKillService()


    #"""INCOMPLETE CODE: THIS IS AN ATTEMPT TO INTEGRATE THE CS API MOTOR-KILL VIA A ROS SERVICE"""
    def handleKillService(self, req):
        for cf in self.allcfs.crazyflies:
            if cf.id == 2:
                print(cf.id)
                cf.cmdStop()
        return 1.0


    #"""THIS CODE *WAITS FOR* POSE MESSAGES"""
    def PoseListener(self):
		#"""CALLBACK CURRENTLY FOR PYTHON API"""
        self.cfx_callback()



	#"""OLD CODE *SETS UP* ROS POSE MESSAGES. NO LONGER USEFUL USING PYTHON API, BUT MIGHT REVERT."""
    def setupKillService(self):
        s = rospy.Service('kill_service', actionlib_tutorials.srv.killMotors(), self.handleKillService)
        self._as.start()
        print("ready to kill.")



    def cfx_callback(self):
        
        for cf in self.allcfs.crazyflies:
            #"""POSE TAKEN FROM CF API"""
            if cf.id == 1:
                self.cf1_pose.x = cf.position()[0]
                self.cf1_pose.y = cf.position()[1]
                self.cf1_pose.z = cf.position()[2]
            if cf.id == 2:
                self.cf2_pose.x = cf.position()[0]
                self.cf2_pose.y = cf.position()[1]
                self.cf2_pose.z = cf.position()[2]
            if cf.id == 3:
                self.cf3_pose.x = cf.position()[0]
                self.cf3_pose.y = cf.position()[1]
                self.cf3_pose.z = cf.position()[2]
            if cf.id == 4:
                self.cf4_pose.x = cf.position()[0]
                self.cf4_pose.y = cf.position()[1]
                self.cf4_pose.z = cf.position()[2]
            if cf.id == 5:
                self.cf5_pose.x = cf.position()[0]
                self.cf5_pose.y = cf.position()[1]
                self.cf5_pose.z = cf.position()[2]
            if cf.id == 6:
                self.cf6_pose.x = cf.position()[0]
                self.cf6_pose.y = cf.position()[1]
                self.cf6_pose.z = cf.position()[2]


        #"""CONTINUOUS CHECK FOR COLLISIONS BETWEEN CF2 AND CF3"""
        collision_distance = self.euclidean_distance(self.cf1_pose, 2)

        if collision_distance < self.collision_tolerance:
            # Collision detected! 
            collision_publisher.publish("home")
            # Code should execute in state machine - but putting this for safety.
            # for cf in self.allcfs.crazyflies:
            #     cf.land(0.04, 2.5)
            # rospy.sleep(self.sleeptime)


####################################################################################################
#   ACTION EXECUTE_CALLBACKS. 

	#"""FOLLOWING PRE-DESIGNED TRAJECTORIES."""
    def traj_callback(self, goal):
        speak (engine, "c f "+str(goal.id)+" spiral")

		#"""MOVES DRONE self.allcfs.crazyflies[0] TO Spiral Trajectory
        #trajectory_action ACTION SERVER [name='trajectory_action', drone: self.allcfs.crazyflies[0], variable:'_as_cf2', callback:'traj_callback']"""
        self.initial_pose = Point()

       # helper variables
        #r = rospy.Rate(10)
        self.PoseListener()
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                self.initial_pose.x = cf.position()[0]
                self.initial_pose.y = cf.position()[1]
                self.initial_pose.z = cf.position()[2]
                # self.final_pose.x = cf.position()[0] + 0.05
                # self.final_pose.y = cf.position()[1] + -0.29
                # self.final_pose.z = cf.position()[2] + 0.8
        #rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)

        self._feedback2.time_elapsed = Duration(5)
        self.traj_success = False

		#"""SELECT A TRAJECTORY"""
        # traj:.csv // trials: repetitions // timescale: speed // sleeptime: set as priority.
        # better arguments: (goal.id, goal.shape, goal.repeat, goal.speed_coefficient)
        if goal.shape == 5:
            selected_traj = self.heli
            TRIALS = 1
            TIMESCALE = 0.5
            SLEEPTIME = 14*TIMESCALE
            REVERSE = False

        if goal.shape == 8:
            selected_traj = self.fig8
            TRIALS = 1
            TIMESCALE = 1
            SLEEPTIME = 14*TIMESCALE
            REVERSE = False
	
        if goal.shape != 5:
            selected_traj = self.heli
            TRIALS = 1
            TIMESCALE = 1
            SLEEPTIME = 14*TIMESCALE
            REVERSE = False

        #"""CODE TO FOLLOW A TRAJECTORY"""
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                if self.arm == True:
                    cf.uploadTrajectory(0, 0, selected_traj)
                    reverse = False
                    cf.startTrajectory(0, timescale=TIMESCALE, reverse = REVERSE)
                    #rospy.sleep(SLEEPTIME)


        
        #"""PREEMPTION CODE [TO VERIFY]"""
        while self.traj_success == False:
            #self.perimeter_monitor(goal.id, self.initial_pose)

            if self._as2.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name2)
                self._as2.set_preempted()
                break
            rospy.sleep(SLEEPTIME)
            self.traj_success = True


        if self.traj_success == True:
            print("Reached the perimeter!!")
            self.traj_success = False
            self.counter = 0
            self._result2.time_elapsed = Duration(5)
            self._result2.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback2)
            rospy.loginfo('%s: Succeeded' % self._action_name2)
            self._as2.set_succeeded(self._result2)


    #"""MOVES DRONE self.allcfs.crazyflies[0] TO CSV Trajectory
    #trajectory_action ACTION SERVER [name='trajectory_action', drone: self.allcfs.crazyflies[0], variable:'_as_cf2', callback:'traj_callback']"""
    
    def drones_fig8_callback(self, goal):

        speak (engine, "FIGURE OF EIGHT")
        self.initial_pose = Point()
        self.initial_pose2 = Point()

        
       # helper variables
        #r = rospy.Rate(10)
        self.PoseListener()
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                self.initial_pose.x = cf.position()[0]
                self.initial_pose.y = cf.position()[1]
                self.initial_pose.z = cf.position()[2]

        #rospy.wait_for_message('/tf', tf2_msgs.msg.TFMessage, timeout=None)

		#"""HELPER VARIABLES FOR THE ACTION SERVER INSTANCE"""
        # append the seeds for the fibonacci sequence
        self._feedbackfig8.time_elapsed = Duration(5)
        self.success == False

		#"""CODE TO FOLLOW A TRAJECTORY"""
        TRIALS = 1
        TIMESCALE = 1
        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id :
                if self.arm == True:
                    cf.takeoff(targetHeight=0.6, duration=3.0)
                    cf.uploadTrajectory(0, 0, self.fig8)
                    #timeHelper.sleep(2.5)
                    cf.startTrajectory(0, timescale=TIMESCALE)
                    #timeHelper.sleep(1.0)
                    rospy.sleep(10)



		#"""PREEMPTION CODE [TO VERIFY]"""
        while self.success == False:

            print ("Not yet...")
            self.perimeter_monitor(goal.id, self.initial_pose)

            if self._asfig8.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_fig8)
                self._asfig8.set_preempted()
                for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id or cf.id == goal.shape:
                        print("LANDING cf...", goal.id)
                        cf.land(0.04, 2.5)
                break

            #print("press button to continue...")
            #self.swarm.input.waitUntilButtonPressed()


            # try:

            #     ###self._feedback.sequence.append(currentPose)
            #     # publish the feedback
            #     self._as2.publish_feedback(self._feedback2)
            #     #rospy.loginfo('%s: Now with tolerance %i with current pose [%s]' % (self._action_name, goal.order, ','.join(map(str,self._feedback.sequence))))

            # except rospy.ROSInterruptException:
            #     print("except clause opened")
            #     rospy.loginfo('Feedback did not go through.')
            #     pass


        while self.success == True:
            # for cf in self.allcfs.crazyflies:
            #     cf.land(0.04, 2.5)
            print("Reached the perimeter!!")
            self.success = False
            self._resultfig8.time_elapsed = Duration(5)
            self._resultfig8.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedbackfig8)
            rospy.loginfo('%s: Succeeded' % self._action_name_fig8)
            self._asfig8.set_succeeded(self._result2)



		#"""END OF CODE TO VERIFY"""

    # print ("CF2 Action Server callback")
    # """MOVING cf2 TO GOAL goal.point
    # detectperimeter ACTION SERVER [name='detectperimeter', drone: cf2, variable:'_to_waypoint', callback:'to_waypoint']"""

    def to_waypoint(self, goal):


        #speak (engine, "Moving to point.")

        self.PoseListener()

        
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_to_waypoint.position = Point()
        self._feedback_to_waypoint.time_elapsed = Duration(5)

        self.success_to_waypoint = False
        self.enable_cfx = True
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])

        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                print("send COMMANDS to cf...", goal.id)
                #self._feedback_cf2.position.position.x = cf.position()[0]
                #self._feedback_cf2.position.position.y = cf.position()[1]
                #self._feedback_cf2.position.position.z = cf.position()[2]

        #Takeoff activates if takeoff has not occurred.
        for cf in self.allcfs.crazyflies:
            if self.enable_cfx == True:                                   
                if cf.id == goal.id:
                    cf.takeoff(targetHeight=0.6, duration=3.0)
                    cf.goTo(self.waypoint, yaw=0, duration=3.0)
                    #print("drone pose is", self.cf2_pose)
                    #self.enable_cfx == False

        while self.success_to_waypoint == False:
            self.PoseListener()

            print ("point is", goal.point)
            print("id is " + str(goal.id))

            if self._as_to_waypoint.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_to_waypoint)
                self._as_to_waypoint.set_preempted()
                break

            print ("Not yet...")
            #now we test if he has reached the desired point.
            self.perimeter_monitor(goal.id, goal.point)

            try:
                # publish the feedback
                self._as_to_waypoint.publish_feedback(self._feedback_to_waypoint)

            except rospy.ROSInterruptException:
                rospy.loginfo("except clause opened")
                pass


        if self.success_to_waypoint == True:

            print("Reached the perimeter!!")
            self.success_to_waypoint = False
            self._result_to_waypoint.time_elapsed = Duration(5)
            self._result_to_waypoint.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_to_waypoint)
            rospy.loginfo('%s: Succeeded' % self._action_name_to_waypoint)
            self._as_to_waypoint.set_succeeded(self._result_to_waypoint)

    # CF2 Action Server callback
    #"""MOVING cf2 TO GOAL goal.point
    #detectperimeter1 ACTION SERVER [name='detectperimeter1', drone: cf3, variable:'_cf3_go', callback:'execute_cb_cf3_go']"""
		
    def execute_cb_cf3_go(self, goal):

	
        speak (engine, "WAYPOINT.")

        self.PoseListener()
        self.initial_pose = Point()
        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf3_go.position = Pose()
        self._feedback_cf3_go.time_elapsed = Duration(5)

        self.success_cf3 = False
        self.enable_cf3 = True
        self.waypoint = np.array([goal.point.x, goal.point.y, goal.point.z])


        for cf in self.allcfs.crazyflies:
            if cf.id == goal.id:
                # sending POSITION FEEDBACK to cf[goal.id]
                self._feedback_cf3_go.position.position.x = cf.position()[0]
                self._feedback_cf3_go.position.position.y = cf.position()[1]
                self._feedback_cf3_go.position.position.z = cf.position()[2]

        for cf in self.allcfs.crazyflies:
            if self.enable_cf3 == True:                                     #ENABLE
                if cf.id == goal.id:

                    cf.land(0.04, 2.5)                                     #LAND
                    rospy.sleep(2)

                    self.success_cf3 = True
                    self.initial_pose.x = cf.position()[0]
                    self.initial_pose.y = cf.position()[1]
                    self.initial_pose.z = cf.position()[2]
                    #self.enable_cfx == False

        while self.success_cf3 == False:
            print ("Action not succeeded yet.")
            
            #Start callback to check for success
            self.PoseListener()

            print ("point is", goal.point)
            print("id is " + str(goal.id))

            if self._as_cf3_go.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf3_go)
                self._as_cf3_go.set_preempted()
                break

            
            #now we test if he has reached the desired point.
            self.perimeter_monitor(goal.id, self.initial_pose)

            try:
                # publish the feedback
                self._as_cf3_go.publish_feedback(self._feedback_cf3_go)

            except rospy.ROSInterruptException:
                rospy.loginfo("except clause opened")
                pass

        print("Drone has reached the perimeter.")
        self.success_cf3 = False
        self._result_cf3_go.time_elapsed = Duration(5)
        self._result_cf3_go.updates_n = 1
        rospy.loginfo('My feedback: %s' % self._feedback_cf3_go)
        rospy.loginfo('%s: Succeeded' % self._action_name_cf3_go)
        self._as_cf3_go.set_succeeded(self._result_cf3_go)



    def rw_callback_copy(self, goal):
        print(goal.id)
        
        if self.success_rw != False:
            # generate random point
            #ROS_INFO_STREAM(str(target))
            #rosout.log(str(target))
            # start = time.time()
            self.success_rw = False
            rpoint = (np.random.rand(3) * 2) - np.ones(3)
            shape = np.array([0.5,0.5,0.3])
            center = np.array([0,0,0.7])
            self.target = np.multiply(shape, rpoint) + center
            self.PoseListener()
            #dif = np.array([self.cf4_pose.x - self.cf2_pose.x, self.cf4_pose.y - self.cf2_pose.y])
            drone = np.array([self.cf4_pose.x, self.cf4_pose.y,  self.cf4_pose.z])
            me = np.array([self.cf1_pose.x, self.cf1_pose.y, self.cf1_pose.z])
            dif = self.target - drone
            angle = (np.arctan2(dif[1],dif[0]) + (np.pi*2))%np.pi
            rospy.logfatal('NEW: '+ str(self.target))
        else:
             # We have reached the goal.

            self._result_rw.time_elapsed = Duration(5)
            self._result_rw.updates_n = 1
            self._feedback_rw.time_elapsed = Duration(5)
            #rospy.loginfo('My feedback: %s' % self._feedback_rw)
            rospy.loginfo('%s: Succeeded' % self._action_name_rw)
            self._as_rw.set_succeeded(self._result_rw)

                    
        while self.success_rw == False:
            # correct drone
            if self._as_rw.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_rw)
                self._as_rw.set_preempted()
                break
            for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id:
                        cfcoords = cf.position()
                        diff = self.target - cfcoords
                        distance = np.linalg.norm(diff)
                        rospy.logfatal(str(distance)[:4]+' to '+str(self.target)+" id:"+str(self.uniqueid)[8:])
                        if distance < 0.3:
                            rospy.logfatal('__WORKS__')
                            self.success_rw = True
                        else:
                            self.PoseListener()
                            drone = np.array([self.cf4_pose.x, self.cf4_pose.y,  self.cf4_pose.z])
                            #me = np.array([self.cf2_pose.x, self.cf2_pose.y, self.cf2_pose.z])
                            dif = drone - cfcoords
                            angle = (np.arctan2(dif[1],dif[0]) + (np.pi))%(np.pi*2) - np.pi
                            cf.goTo(cfcoords + 0.1*diff/distance, yaw=angle, duration=0.6)
                            time.sleep(0.6)


        #over = time.time()
        if self.success_rw == True:
            ### we have reache the goal
            print("Reached the goal")
            self._result_rw.time_elapsed = Duration(5)
            self._result_rw.updates_n = 1
            self._feedback_rw.time_elapsed = Duration(5)
            #rospy.loginfo('My feedback: %s' % self._feedback_rw)
            rospy.loginfo('%s: Succeeded' % self._action_name_rw)
            self._as_rw.set_succeeded(self._result_rw)  

    def rw_callback (self, goal):
        center = np.array([0, 0, 0.7])
        vel = np.array([0.0,0.0,0.0])
        cf = None
        for drone in self.allcfs.crazyflies:
            if drone.id == goal.id:
                cf = drone

        while  not self._as_rw.is_preempt_requested():
            dif = center - cf.position()
            dist = np.linalg.norm(dif)
            if dist > 0.1:
                acc = 0.0005*dif/(dist*dist)
                rospy.logfatal(str(np.linalg.norm(acc)))   
                vel+=acc
            vnorm = np.linalg.norm(vel)
            if vnorm > 0.3:
                cf.cmdVelocityWorld(0.3*vel/vnorm,yawRate=0)
            else:
                cf.cmdVelocityWorld(vel,yawRate=0)
            
            if vnorm > 0.8:
                vel = 0.8*vel/vnorm

        self._as_rw.set_preempted()

        #over = time.time()
        if self.success_rw == True:
            ### we have reache the goal
            print("Reached the goal")
            self._result_rw.time_elapsed = Duration(5)
            self._result_rw.updates_n = 1
            self._feedback_rw.time_elapsed = Duration(5)
            #rospy.loginfo('My feedback: %s' % self._feedback_rw)
            rospy.loginfo('%s: Succeeded' % self._action_name_rw)
            self._as_rw.set_succeeded(self._result_rw)    

    def safe_down(self, goal):
        vel = np.array([0.0,0.0,-0.2])
        cf = None
        for drone in self.allcfs.crazyflies:
            if drone.id == goal.id:
                cf = drone
        start = time.time()
        while  (time.time() - start) < 5:
            cf.cmdVelocityWorld(vel,yawRate=0)
            vel = vel *0.99999
        
        cf.emergency()

        #over = time.time()
        if self.success_rw == True:
            ### we have reache the goal
            print("Reached the goal")
            self._result_rw.time_elapsed = Duration(5)
            self._result_rw.updates_n = 1
            self._feedback_rw.time_elapsed = Duration(5)
            #rospy.loginfo('My feedback: %s' % self._feedback_rw)
            rospy.loginfo('%s: Succeeded' % self._action_name_rw)
            self._as_rw.set_succeeded(self._result_rw)    

    def my_hover(self, goal):
        vel = np.array([0.0,0.0,0.8])
        cf = None
        for drone in self.allcfs.crazyflies:
            if drone.id == goal.id:
                cf = drone
        start = time.time()
        while  (time.time() - start) < 1.5:
            cf.cmdVelocityWorld(vel,yawRate=0)
        self.success_mh = True
        print("Reached the goal")
        self._result_mh.time_elapsed = Duration(5)
        self._result_mh.updates_n = 1
        self._feedback_mh.time_elapsed = Duration(5)
        #rospy.loginfo('My feedback: %s' % self._feedback_mh)
        rospy.loginfo('%s: Succeeded' % self._action_name_mh)
        self._as_mh.set_succeeded(self._result_mh)   


    def execute_cb_cf3_follow_cf2(self, goal):
		# Called by detect_perimeter1 ACTION SERVER
		#"""MOVES DRONE goal.id TO POSE goal.point"""
        
        speak (engine, "FOLLOW ME")

        self.PoseListener()

        print ("point is", goal.point)
        print("id is " + str(goal.id))

        self._feedback_cf3.position = Pose()
        self._feedback_cf3.time_elapsed = Duration(5)

        self.success_cf3 = False
        y_offset = 0
        x_offset = -0.3

        while self.success_cf3 == False:

            self.PoseListener()
            print("in false loop...")

            if self._as_cf3.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name_cf3)
                self._as_cf3.set_preempted()
                for cf in self.allcfs.crazyflies:
                    if cf.id == goal.id:
                        print("LANDING cf...", goal.id)
                        cf.land(0.04, 2.5)
                break

            # setting cf2 as the waypoint.
            self.waypoint = np.array([self.cf2_pose.x + x_offset, self.cf2_pose.y + y_offset, self.cf2_pose.z])
            print("waypoint is", self.waypoint)

            for cf in self.allcfs.crazyflies:
                if cf.id == goal.id:
                    cf.goTo(self.waypoint, yaw=0, duration=self.justenoughsleeptime)
                    #rospy.sleep(self.justenoughsleeptime)


            self.flag_monitor() #STOP WHEN HAND SIGN APPEARS.



        if self.success_cf3 == True:

            print("Reached the perimeter!!")
            self.success_cf3 = False
            self._result_cf3.time_elapsed = Duration(5)
            self._result_cf3.updates_n = 1
            rospy.loginfo('My feedback: %s' % self._feedback_cf3)
            rospy.loginfo('%s: Succeeded' % self._action_name_cf3)
            self._as_cf3.set_succeeded(self._result_cf3)


    #""" ACTION CONDITIONALS: VALIDATION FUNCTIONS """

    def euclidean_distance(self, goal_pose, id):
        """Euclidean distance between current pose and the goal."""
        if id == 1:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf1_pose.x), 2) + pow((goal_pose.y - self.cf1_pose.y), 2) + pow((goal_pose.z - self.cf1_pose.z), 2))

        elif id == 2:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf2_pose.x), 2) + pow((goal_pose.y - self.cf2_pose.y), 2) + pow((goal_pose.z - self.cf2_pose.z), 2))

        elif id == 3:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf3_pose.x), 2) + pow((goal_pose.y - self.cf3_pose.y), 2) + pow((goal_pose.z - self.cf3_pose.z), 2))

        elif id == 4:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf4_pose.x), 2) + pow((goal_pose.y - self.cf4_pose.y), 2) + pow((goal_pose.z - self.cf4_pose.z), 2))

        elif id == 5:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf5_pose.x), 2) + pow((goal_pose.y - self.cf5_pose.y), 2) + pow((goal_pose.z - self.cf5_pose.z), 2))

        elif id == 6:
            euclidean_distance= sqrt(pow((goal_pose.x - self.cf6_pose.x), 2) + pow((goal_pose.y - self.cf6_pose.y), 2) + pow((goal_pose.z - self.cf6_pose.z), 2))

        else:
            print ("no id detected... aborting?", id)
        return euclidean_distance


    def perimeter_monitor(self, id, goal):
		#"""Check if drone enters a certain tolerance from the goal"""
        distance_tolerance = 0.2

        if self.euclidean_distance(goal, id) < distance_tolerance:
            self.success_to_waypoint = True
            self.success = True


    def flag_monitor(self):
		#"""Trigger function: when specific ROS Message appears, enable trigger variable."""
		# Used with the simulator to transfer events (txa#008)
        try:
            follow_trigger = rospy.wait_for_message("/swarmfollow", String, timeout=0.3)
            if follow_trigger.data == "STOP_FOLLOW_ME":
                return True
            else:
                return False
        except:
            pass



#"""CTRL-C HANDLER"""
signal.signal(signal.SIGINT, signal_handler)


if __name__ == "__main__":
    #rospy.init_node('hi') # CANNOT do this: conflicts with CrazyflieAPI. (txa#007)

    print ("Server name is:", rospy.get_name()) 

    collision_publisher = rospy.Publisher('/collision', String, queue_size=10)
    perimeter_server = PerimeterMonitor(str(rospy.get_name())+str(1))
    #position_subscriber = rospy.Subscriber('/tf', TFMessage, perimeter_server.cfx_callback)

    # Currently used to allow continuous callbacks
    rospy.spin()
