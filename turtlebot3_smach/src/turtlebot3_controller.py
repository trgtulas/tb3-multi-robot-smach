#!/usr/bin/env python3 

import rospy 
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
import time
import smach, smach_ros
from smach import State, StateMachine

status = {'robot1':0, 'robot2':0, 'robot3':0}
station = {'station1': [2,0.5,0], 'station2': [0.5,2,0], 'station3': [0,-2,0]}

def check_robot_status():
    for robot in status:
        if status[robot] == 0:
            status[robot] = 1
            return robot
        rospy.loginfo('There is no available robot at the moment')
        return None
    
def check_station(s):
    if station[s][2] == 0:
        rospy.loginfo('Ready to dock.')
        station[s][2] = 1
        return True

class TakeOrder(State):
    def __init__(self):
        State.__init__(self, outcomes=['Taken', 'Failed'], output_keys=['station_out'])
        self.order = None

    def order_callback(self, msg):

        rospy.loginfo(f"Received order: {msg.data}")
        self.order = msg.data

    def execute(self, user_data):
        self.order = None
        station_sub = rospy.Subscriber('/station_order', String, self.order_callback)
        while self.order == None:
            rospy.sleep(0.1)
        user_data.station_out = self.order
        if check_station(self.order) == True:
            self.order = None
            return 'Taken'
        else:
            rospy.logwarn('TAKEORDER IS FAILED!!')
            return 'Failed'


class NavTo(State):
    def __init__(self):
        State.__init__(self, outcomes=['Arrived', 'Failed'], input_keys=['station_out'])
        self.goal_pub = rospy.Publisher('robot1/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.goal_status_sub = rospy.Subscriber('/robot1/move_base/result', MoveBaseActionResult, self.goal_status_callback)
        self.goal_status = -1
    def goal_status_callback(self, msg):
        self.goal_status = msg.status.status

    def execute(self, user_data):
        goal_station = user_data.station_out
        rospy.logwarn(goal_station)
        goal = MoveBaseActionGoal()
        
        goal.goal.target_pose.pose.position.x = station[goal_station][0]
        rospy.loginfo(station[goal_station][0])

        goal.goal.target_pose.pose.position.y = station[goal_station][1]
        rospy.loginfo(station[goal_station][1])

        goal.goal.target_pose.pose.orientation.w= 1
        goal.goal.target_pose.header.frame_id = 'map'

        self.goal_pub.publish(goal)
        while self.goal_status == -1:
            rospy.sleep(0.1)
        rospy.logwarn(f"while statement: {self.goal_status}")
        if self.goal_status == 3:
            rospy.logwarn(f"if statement: {self.goal_status}")
            return 'Arrived'
        else:
            rospy.logwarn('NAVTO IS FAILED!!')
            rospy.logwarn(f"failed statement: {self.goal_status}")
            return 'Failed'
            
class Report(State):
    def __init__(self):
        State.__init__(self, outcomes=['done'])
    def execute(self, user_data):
        rospy.loginfo('FSM is completed')
        return 'done'
    
def main():
    rospy.init_node('multirobots_smach')
        
    sm = StateMachine(outcomes=['DONE'])

    with sm:
        StateMachine.add('TAKEORDER', TakeOrder(), transitions={'Failed':'FAILED', 'Taken':'NAVTO'})
        StateMachine.add('FAILED', Report(), transitions={'done':'DONE'})
        StateMachine.add('NAVTO', NavTo(), transitions={'Failed':'FAILED', 'Arrived':'REPORT'})
        StateMachine.add('REPORT', Report(), transitions={'done':'DONE'})
    
    sis = smach_ros.IntrospectionServer('sm_cmdvel_view', sm, '/SM')
    sis.start()

    outcome = sm.execute()
    rospy.loginfo("SM outcome: %s", outcome)    

    sis.stop()

if __name__ == '__main__':
    main()