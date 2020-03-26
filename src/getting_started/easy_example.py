#!/usr/bin/env python

import smach
import smach_ros

import rospy

from time import sleep


class Foo(smach.State):
    def __init__(self, outcomes=['outcome1', 'outcome2']):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("entered state FOO, counter = {}".format(self.counter))        
        sleep(3)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'
        

class Bar(smach.State):
    def __init__(self, outcomes=['outcome2']):
        smach.State.__init__(self, outcomes=['outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo("entered state BAR, counter = {}".format(self.counter))
        sleep(3)
        self.counter += 1
        if userdata:
            return 'outcome2'
        

if __name__ == "__main__":

    rospy.init_node("easy_state_machine")
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])
    with sm:
        smach.StateMachine.add('FOO', Foo(), transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
 
        smach.StateMachine.add('BAR', Bar(), transitions={'outcome2':'FOO'})

    outcome = sm.execute()