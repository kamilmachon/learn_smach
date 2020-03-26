#!/usr/bin/env python

import smach
import smach_ros

import rospy

from time import sleep 


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                outcomes=['outcome1', 'outcome2'],
                input_keys=['foo_counter_in'],
                output_keys=['foo_counter_out'])

    def execute(self, userdata):
        sleep(4)
        rospy.loginfo('executing state FOO')
        if userdata.foo_counter_in < 3:
            userdata.foo_counter_out = userdata.foo_counter_in + 1
            return 'outcome1'
        else:  
            return 'outcome2'

    
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in'])

    def execute(self, userdata):
        sleep(4)
        rospy.loginfo('executing state BAR')
        rospy.loginfo('bar input state = {}'.format(userdata.bar_counter_in))
        return 'outcome1'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter', 
                                          'foo_counter_out':'sm_counter'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter'})



    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')

    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    sis.stop()


if __name__ == '__main__':
    main()
