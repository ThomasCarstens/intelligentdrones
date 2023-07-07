#! /usr/bin/env python
from __future__ import print_function
import rospy

import threading
from math import sqrt, pow
import smach_ros
import smach

import actionlib
from actionlib_tutorials.msg import my_newAction, my_newGoal
from geometry_msgs.msg import Point
from smach import StateMachine, Concurrence
from smach_ros import ActionServerWrapper, ServiceState, SimpleActionState, MonitorState, IntrospectionServer
import std_srvs.srv

import sys

def polygonial():
    ids = [2, 1, 4]
    #define the differents points


    home_points = [Point(), Point(), Point()]

    home_points[0].x = 0.0
    home_points[0].y = 0.0
    home_points[0].z = 0.3

    home_points[1].x = 0.0
    home_points[1].y = -0.5
    home_points[1].z = 0.5

    home_points[2].x = 0.5
    home_points[2].y = -0.5
    home_points[2].z = 0.5


    home_points_heli = [Point(), Point(), Point()]

    home_points_heli[0].x = 0.0
    home_points_heli[0].y = 0.0
    home_points_heli[0].z = 0.3

    home_points_heli[1].x = 0.0
    home_points_heli[1].y = 0.9-0.4
    home_points_heli[1].z = 0.9

    home_points_heli[2].x = -0.3
    home_points_heli[2].y = 0.0
    home_points_heli[2].z = 0.3

    my_points = [Point(), Point(), Point(), Point(), Point(), Point(), Point(), Point()]

    my_points[0].x = 0.5
    my_points[0].y = -0.5
    my_points[0].z = 1.0

    my_points[1].x = 0.5
    my_points[1].y = -1.0
    my_points[1].z = 1.0

    my_points[2].x = 0.0
    my_points[2].y = -1.0
    my_points[2].z = 1.0

    my_points[3].x = 0
    my_points[3].y = -0.5
    my_points[3].z = 1.0

    # @ Higher altitude and bigger rectangle (limit cascading air)
    my_points[4].x = 0.5
    my_points[4].y = 0.0
    my_points[4].z = 1.5

    my_points[5].x = 0.5
    my_points[5].y = -1.5
    my_points[5].z = 1.5

    my_points[6].x = 0.0
    my_points[6].y = -1.5
    my_points[6].z = 1.5

    my_points[7].x = 0
    my_points[7].y = 0.0
    my_points[7].z = 1.5



    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm0:

        StateMachine.add('drone2-2',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points[2], id = ids[0] )),
                        transitions={'succeeded' : 'drone3-1', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        StateMachine.add('drone3-1',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points[1], id = ids[1] )),
                        transitions={'succeeded' : 'infinit_loop', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        # Concurrent land - different from individual land below.

        land_sm =  Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('land_all', land_sm)

        with land_sm:
            #  #Land Drone If Aborted
            Concurrence.add('LAND_DRONE1',
                            SimpleActionState('land_drone1',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])  ))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE2',
                            SimpleActionState('land_drone2',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])  ))

            ############################################

        sm1 = Concurrence(['succeeded', 'aborted', 'preempted'],
                'succeeded',
                )

        StateMachine.add('infinit_loop', sm1)

        with sm1:
            drone1 = StateMachine(outcomes=['succeeded','aborted','preempted'])  

            Concurrence.add('DRONE1', drone1)

            with drone1:
                #State order (points defined above)
                order = (5, 6, 7, 0, 1, 2, 3, 4)
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = my_points[order[i]].x
                    point_for_state.y = my_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = my_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = my_points[order[i]].z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE1-' + str(order[i]),
                                     SimpleActionState('drone1detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[0])),
                                       transitions={'succeeded' : 'DRONE1-' + str(order[i+1]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})

                #make it infinit
                smach.StateMachine.add('DRONE1-' + str(order[-1]),
                               SimpleActionState('drone1detect_perimeter',
                                                    my_newAction, goal = my_newGoal(point = my_points[order[-1]], id = ids[0])),
                              transitions={'succeeded' : 'DRONE1-'  + str(order[0]), 'aborted' : 'LAND_DRONE1', 'preempted' : 'LAND_DRONE1'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE1',
                               SimpleActionState('land_drone1',
                                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])),
                              transitions={'succeeded' : 'LAND_DRONE1'})



            drone2 = StateMachine(outcomes=['succeeded','aborted','preempted']) 


            Concurrence.add('DRONE2', drone2)
            # Open the container
            with drone2:
                #add each state
                order = (3, 4, 5, 6, 7, 0, 1, 2 )
                for i in range(7):
                    point_for_state = Point()
                    point_for_state.x = my_points[order[i]].x
                    point_for_state.y = my_points[order[i]].y
                    if i <= 4 :
                        point_for_state.z = my_points[order[i]].z + (i*0.1)
                    else :
                        point_for_state.z = my_points[order[i]].z - (i*0.1) + 0.8 
                    StateMachine.add('DRONE2-' + str(order[i]),
                                     SimpleActionState('drone2detect_perimeter',
                                                          my_newAction, goal = my_newGoal(point = point_for_state, id = ids[1])),
                                       transitions={'succeeded' : 'DRONE2-' + str(order[i+1]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})

                #make it infinit
                smach.StateMachine.add('DRONE2-' + str(order[-1]),
                             SimpleActionState('drone2detect_perimeter',
                                                     my_newAction, goal = my_newGoal(point = my_points[order[-1]], id = ids[1])),
                             transitions={'succeeded' : 'DRONE2-' + str(order[0]), 'aborted' : 'LAND_DRONE2', 'preempted' : 'LAND_DRONE2'})


                # #Land Drone If Aborted
                smach.StateMachine.add('LAND_DRONE2',
                               SimpleActionState('land_drone2',
                                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])),
                              transitions={'succeeded' : 'LAND_DRONE2'})



    # Attach a SMACH introspection server
    sis = IntrospectionServer('Infinite_polygonial', sm0, '/INFINITE_POLY')
    sis.start()

    # Set preempt handler
    smach_ros.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()



if __name__ == '__main__':
    rospy.init_node('INFINITE_POLY')
    t1 = threading.Thread(target=polygonial)
    t1.start()
    rospy.spin()
