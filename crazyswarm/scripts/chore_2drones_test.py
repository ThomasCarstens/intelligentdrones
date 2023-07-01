#! /usr/bin/env python
from __future__ import print_function
import rospy

import threading
from math import sqrt, pow
import smach_ros
import smach

import actionlib
from actionlib_tutorials.msg import my_newAction, my_newGoal, MachineAction, FibonacciAction, FibonacciGoal, DoTrajAction, DoTrajGoal, DoTrajFeedback
from geometry_msgs.msg import Point
from smach import StateMachine, Concurrence
from smach_ros import ActionServerWrapper, ServiceState, SimpleActionState, MonitorState, IntrospectionServer
import std_srvs.srv



import sys

def polygonial():
    ids = [2, 1, 4]
    #define the differents points
    test_point= Point()
    test_point.x = 0
    test_point.y = 0
    test_point.z = 0.5

    home_points_fo8 = [Point(), Point(), Point()]

    home_points_fo8[0].x = 0.0
    home_points_fo8[0].y = 0.0
    home_points_fo8[0].z = 0.3

    home_points_fo8[1].x = 0.3
    home_points_fo8[1].y = 0.0
    home_points_fo8[1].z = 0.3

    home_points_fo8[2].x = -0.3
    home_points_fo8[2].y = 0.0
    home_points_fo8[2].z = 0.3


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
    my_points[0].z = 0.5

    my_points[1].x = 0.7
    my_points[1].y = 0.0
    my_points[1].z = 0.5

    my_points[2].x = 0.5
    my_points[2].y = 0.5
    my_points[2].z = 0.5

    my_points[3].x = 0
    my_points[3].y = 0.7
    my_points[3].z = 0.5

    my_points[4].x = -0.5
    my_points[4].y = 0.5
    my_points[4].z = 0.5

    my_points[5].x =  -0.7
    my_points[5].y = 0
    my_points[5].z = 0.5

    my_points[6].x = -0.5
    my_points[6].y = -0.5
    my_points[6].z = 0.5

    my_points[7].x = 0
    my_points[7].y = -0.7
    my_points[7].z = 0.5



    # Create a SMACH state machine
    sm0 = StateMachine(outcomes=['succeeded','aborted','preempted'])
    #progressively add drones
    with sm0:



        StateMachine.add('drone2-HELI',
                         SimpleActionState('drone2detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[2], id = ids[0] )),
                        transitions={'succeeded' : 'drone3-HELI', 'aborted' : 'land_all', 'preempted' : 'land_all'})
                        
        StateMachine.add('drone3-HELI',
                         SimpleActionState('drone3detect_perimeter',
                                            my_newAction, goal = my_newGoal(point = home_points_fo8[1], id = ids[1] )),
                        transitions={'succeeded' : 'HELI_EXECUTE', 'aborted' : 'land_all', 'preempted' : 'land_all'})




        heli_sm = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('HELI_EXECUTE', heli_sm, transitions={'succeeded' : 'LAND_END', 'aborted' : 'land_all', 'preempted' : 'land_all'})

        with heli_sm:
            Concurrence.add('HELI_EXECUTE_drone2',
            SimpleActionState('heli_drone2',
                            DoTrajAction, goal = DoTrajGoal(shape = 0, id = ids[0])))

            Concurrence.add('HELI_EXECUTE_drone3',
            SimpleActionState('heli_drone3',
                                DoTrajAction, goal = DoTrajGoal(shape = 0, id = ids[1])))





        fig8_end = Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('LAND_END', fig8_end, transitions={'succeeded' : 'land_all', 'aborted' : 'land_all', 'preempted' : 'land_all'})
        ## Bug BBL: DO NOT try to decide on outcomes in a concurrence, do it in the SM.add!!





        with fig8_end:
            Concurrence.add('FIG8_END_drone2',
                SimpleActionState('land_drone2',
                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0]) ))

            Concurrence.add('FIG8_END_drone3',
                SimpleActionState('land_drone3',
                                    my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1]) ))

                                   
        


        land_sm =  Concurrence(['succeeded', 'aborted', 'preempted'], 'succeeded')

        StateMachine.add('land_all', land_sm)

        with land_sm:



            # DUPLICATA FOR HIGH LEVEL
            #  #Land Drone If Aborted
            Concurrence.add('LAND_DRONE1',
                            SimpleActionState('land_drone1',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[0])  ))

            # #Land Drone If Aborted
            Concurrence.add('LAND_DRONE2',
                            SimpleActionState('land_drone2',
                                                my_newAction, goal = my_newGoal(point = my_points[3], id = ids[1])  ))


            ############################################




    # Attach a SMACH introspection server
    sis = IntrospectionServer('chore_2drones_test', sm0, '/chore_2drones_test')
    sis.start()

    # Set preempt handler
    smach_ros.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()



if __name__ == '__main__':
    rospy.init_node('smach_usecase_step_06')
    t1 = threading.Thread(target=polygonial)
    t1.start()
    rospy.spin()
