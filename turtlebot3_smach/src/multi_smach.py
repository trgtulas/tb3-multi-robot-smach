#!/usr/bin/env python3 
import threading
import rospy 
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Twist, Pose
from turtlebot3_smach.msg import Order, StationStatus, RobotStatus
import time
import smach, smach_ros
from smach import State, StateMachine
from turtlebot3_smach.srv import StartJob, StartJobRequest, EndJob, EndJobRequest
from sensor_msgs.msg import LaserScan

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['Start', 'Loop'], output_keys=['station', 'start_flag'])
        self.robot_ns = rospy.get_param('~robot_ns')  
        self.start_flag = False
        self.command = None
        self.order_sub = rospy.Subscriber('/order_topic', Order, self.order_cb)
        self.start_job = rospy.ServiceProxy(f'/{self.robot_ns}/start_job', StartJob)
        self.order = Order()

    def order_cb(self, msg):
        self.order = msg

    def execute(self, user_data):
        rospy.sleep(1)
        rospy.loginfo(f"start_flag={self.start_flag}")
        rospy.wait_for_service(f'/{self.robot_ns}/start_job')
        try:
            req = StartJobRequest()
            req.status = 'waiting'
            resp = self.start_job(req)

            user_data.station = resp.station
            if resp.command == 'start':
                self.start_flag = True
                # rospy.logwarn(f"Service response {resp.command}")
            elif resp.command == 'wait':
                rospy.logwarn(f'{self.robot_ns} Waiting for order...')
                return 'Loop'
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        if self.start_flag:
            rospy.loginfo('Smach is Started.')
            user_data.start_flag = self.start_flag
            return 'Start'
        else:
            rospy.loginfo('waiting for new order.')
            rospy.sleep(1)
            return 'Loop'

class NavTo(State):
    def __init__(self):
        State.__init__(self, outcomes=['Sent', 'Failed'], input_keys=['station'])
        self.robot_ns = rospy.get_param('~robot_ns')  # use private param
        self.goal_topic = f'/{self.robot_ns}/move_base_simple/goal'
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        # self.goal_result = rospy.Subscriber(f'/{robot_ns}/move_base_result', MoveBaseActionResult, self.goal_result_cb)
        self.goal_status = None

    def goal_result_cb(self, msg):
        self.goal_status = msg.status.status

    def execute(self, user_data):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.orientation.w = 1.0 
        goal.pose.orientation.z = 0.4
        rospy.logwarn(f'{self.robot_ns} is moving to Station position -> {user_data.station}')
        goal.pose.position.x = user_data.station.x
        goal.pose.position.y = user_data.station.y
        self.goal_pub.publish(goal)
        goal_status = rospy.wait_for_message(f'/{self.robot_ns}/move_base/result', MoveBaseActionResult, timeout=120)
        if goal_status.status.status == 3:
            rospy.logwarn(f'{self.robot_ns} is arrived succesfully')
            return 'Sent'
        else:
            rospy.logwarn('robot couldnt arrived.')
            return 'Failed'
        
class FindBand(State):
    def __init__(self):
        State.__init__(self, outcomes=['Found', 'Failed'])
        self.robot_ns = rospy.get_param('~robot_ns') 
        self.intensities = []
        self.line_intens = 100
        self.stop_intens = 10
        self.controler_pub = rospy.Publisher(f'/{self.robot_ns}/cmd_vel', Twist, queue_size=10)
        self.intensity_sub = rospy.Subscriber(f'/{self.robot_ns}/intensity_scan', LaserScan, self.intensity_cb)
        self.left_most = 0
        self.right_most = 0

    def intensity_cb(self, msg):
        self.intensities = msg.intensities
        for i in self.intensities:
            if rospy.is_shutdown():
                return
            if i == self.line_intens:
                self.left_most = self.intensities.index(i) 
                break

        l = len(self.intensities) - 1
        while l != 0 and not rospy.is_shutdown():
            if self.intensities[l] == self.line_intens:
                self.right_most = l
                break
            l -= 1
        
    def execute(self, user_data):
        cmd = Twist()  
        time = rospy.Time.now()
        while not rospy.is_shutdown():            
            cmd.angular.z = -0.3
            cmd.linear.x = 0
            self.controler_pub.publish(cmd)
            if  self.intensities.count(self.line_intens) > 60:  # line width
                cmd.angular.y = 0
                cmd.angular.z = 0
                cmd.linear.x = 0
                cmd.linear.y = 0
                cmd.linear.z = 0
                self.controler_pub.publish(cmd)
                rospy.sleep(1)
                rospy.logwarn(f'{self.robot_ns} found the line.')
                return 'Found'
            if rospy.Time.now() - time == 6:
                rospy.logwarn(f'{self.robot_ns} couldnt fine the line')
                return 'Failed'
        
        
class Trackline(State):
    def __init__(self):
        State.__init__(self, outcomes=['Succeed', 'Failed'])
        self.robot_ns = rospy.get_param('~robot_ns')  # use private param

        self.intensity_sub = rospy.Subscriber(f'/{self.robot_ns}/intensity_scan', LaserScan, self.intensity_cb)
        self.controler_pub = rospy.Publisher(f'/{self.robot_ns}/cmd_vel', Twist, queue_size=10)
        self.intensities = []
        self.line_intens = 100
        self.stop_intens = 10
        self.kp = 0.05
        self.ki = 0.01
        self.kd = 0.1
        self.integral = 0
        self.last_error = 0
        self.left_most = 0
        self.right_most = 0

    def intensity_cb(self, msg):
        self.intensities = msg.intensities
        for i in self.intensities:
            if rospy.is_shutdown():
                return
            if i == self.line_intens:
                self.left_most = self.intensities.index(i) 
                break

        l = len(self.intensities) -1 
        while l != 0 and not rospy.is_shutdown():
            if self.intensities[l] == self.line_intens:
                self.right_most = l
                break
            l -= 1

    def update_pid(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        output = (self.kp * error +
                   self.ki * self.integral +
                  self.kd * derivative
                  )
        return output  
    
    def execute(self, user_data):
        cmd = Twist()
        while not rospy.is_shutdown():
            # pid updating and docking here \!!!!!/
            # pid updating and docking here  \!!!/
            # pid updating and docking here   \!/
            # pid updating and docking here    ! 
            current_time = rospy.Time.now()
            current_point = (self.left_most + self.right_most) / 2
            set_point = 45
            error = set_point - current_point
            dt = 0.0000001
            correction = self.update_pid(error, dt)
            last_time = rospy.Time.now()
            cmd.angular.z = correction
            cmd.linear.x = 0.1
            self.controler_pub.publish(cmd)
            # pid updating and docking here    ! 
            # pid updating and docking here   \!/
            # pid updating and docking here  \!!!/
            # pid updating and docking here \!!!!!/
            if self.stop_intens in self.intensities:
                cmd = Twist()
                cmd.angular.z = 0
                cmd.linear.x = 0
                self.controler_pub.publish(cmd)
                rospy.logwarn('Band is tracked.')
                return 'Succeed'
        return 'Failed'

class ReverseTrackline(State):
    def __init__(self):
        State.__init__(self, outcomes=['Succeed', 'Failed'], input_keys=['station'])
        self.robot_ns = rospy.get_param('~robot_ns')  
        self.intensity_sub = rospy.Subscriber(f'/{self.robot_ns}/intensity_scan', LaserScan, self.intensity_cb)
        self.controler_pub = rospy.Publisher(f'/{self.robot_ns}/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher(f'/{self.robot_ns}/move_base_simple/goal', PoseStamped, queue_size=10)
        self.intensities = []
        self.line_intens = 100
        self.stop_intens = 10
        self.kp = 0.005
        self.ki = 0.005
        self.kd = 0.1
        self.integral = 0
        self.last_error = 0
        self.left_most = 0
        self.right_most = 0

    def intensity_cb(self, msg):
        self.intensities = msg.intensities
        for i in self.intensities:
            if rospy.is_shutdown():
                return
            if i == self.line_intens:
                self.left_most = self.intensities.index(i) 
                break

        l = len(self.intensities) -1 
        while l != 0 and not rospy.is_shutdown():
            if self.intensities[l] == self.line_intens:
                self.right_most = l
                break
            l -= 1

    def update_pid(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        output = (self.kp * error +
                   self.ki * self.integral  
                #    self.kd * derivative
                  )
        return output  

    def execute(self, user_data):
        rospy.sleep(1)
        cmd = Twist()
        while not rospy.is_shutdown():
            # pid updating and docking here \!!!!!/
            # pid updating and docking here  \!!!/
            # pid updating and docking here   \!/
            # pid updating and docking here    ! 
            current_time = rospy.Time.now()
            current_point = (self.left_most + self.right_most) / 2
            set_point = 45
            error = set_point - current_point
            dt = 0.0001
            correction = self.update_pid(error, dt)
            last_time = rospy.Time.now()
            cmd.angular.z = 0.0
            cmd.linear.x = -0.05
            self.controler_pub.publish(cmd)
            # pid updating and docking here    ! 
            # pid updating and docking here   \!/
            # pid updating and docking here  \!!!/
            # pid updating and docking here \!!!!!/
            resp = None
            if (self.intensities.count(self.line_intens) == 0 and 
                self.intensities.count(self.stop_intens) == 0):
                cmd = Twist()
                cmd.angular.z = 0
                cmd.linear.x = 0
                self.controler_pub.publish(cmd)
                rospy.logwarn('Robot is out.')
                rospy.logwarn(f"Calling service: /{self.robot_ns}/end_job")
                rospy.wait_for_service(f'/{self.robot_ns}/end_job')
                end_job = rospy.ServiceProxy(f'/{self.robot_ns}/end_job', EndJob)
               
                try:
                    req = EndJobRequest()
                    resp = end_job(req)
                    if resp is None:
                        rospy.logwarn(f'resp is returens as None {self.robot_ns}')
                        return 'Failed'
                    if resp.response == 'done':
                        rospy.logwarn(f'{self.robot_ns} -> Job is succeed.')
                        goal = PoseStamped()
                        goal.pose.position.x = resp.waiting_location.x
                        goal.pose.position.y = resp.waiting_location.y
                        goal.pose.orientation.w = 1.0
                        goal.header.stamp = rospy.Time.now()
                        goal.header.frame_id = 'map'
                        time.sleep(0.2)
                        self.goal_pub.publish(goal)
                        return 'Succeed'
                    elif resp.response == 'error':
                        rospy.logwarn('No valid command.')
                        return 'Failed'
                    else:
                        rospy.logwarn('Unknown response for end_job')
                        return 'Failed'
                except rospy.ServiceException as e:
                    rospy.logerr(f'Unknown response {e}')
            


class Failed(State):
    def __init__(self):
        State.__init__(self, outcomes=['Failed'])
    
    def execute(self, user_data):
        return 'Failed'
        
def main():
    rospy.init_node('Smach')

    sm = StateMachine(outcomes=['DONE'])

    sm.userdata.start_flag = False
    with sm:

        StateMachine.add('IDLE', Idle(), transitions={'Start':'NAVTO', 'Loop':'IDLE'})
        StateMachine.add('NAVTO', NavTo(), transitions={'Sent':'FINDBAND', 'Failed':'FAILED'})
        StateMachine.add('FINDBAND', FindBand(), transitions={'Found':'TRACKLINE', 'Failed':'FAILED'})
        StateMachine.add('TRACKLINE', Trackline(), transitions={'Succeed':'REVERSE_TRACKLINE', 'Failed':'FAILED'})
        StateMachine.add('REVERSE_TRACKLINE', ReverseTrackline(), transitions={'Succeed':'IDLE', 'Failed':'FAILED'})
        StateMachine.add('FAILED', Failed(), transitions={'Failed':'DONE'})
    
        
    sis = smach_ros.IntrospectionServer('sm_cmdvel_view', sm, '/SM')
    sis.start()

    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    rospy.spin()
 
    sis.stop()
    smach_thread.join()

if __name__ == '__main__':
    main()
    rospy.spin()
