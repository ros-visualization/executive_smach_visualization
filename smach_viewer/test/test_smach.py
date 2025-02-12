#!/usr/bin/env python3

import rospy
import smach
import smach_ros

class FOO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OUTCOME1','OUTCOME2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'OUTCOME1'
        else:
            return 'OUTCOME2'

class BAR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['OUTCOME2'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'OUTCOME2'

def main():
    rospy.init_node('smach_sample2')

    sm_top = smach.StateMachine(outcomes=['OUTCOME3'])

    with sm_top:
        smach.StateMachine.add('FOO', FOO(), transitions={'OUTCOME1':'BAR', 'OUTCOME2':'OUTCOME3'})
        smach.StateMachine.add('BAR', BAR(), transitions={'OUTCOME2':'FOO'})

    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

