#!/usr/bin/env python3 
import threading
import rospy 
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from turtlebot3_smach.msg import Order, StationStatus, RobotStatus
import time
import smach, smach_ros
from smach import State, StateMachine
from turtlebot3_smach.srv import StartJob, StartJobResponse, ReserveStation, ReserveStationRequest, ReserveRobot, ReserveRobotRequest, Release, ReleaseRequest

class Idle(State):
    def __init__(self):
        State.__init__(self, outcomes=['Start', 'Loop', 'Stop'], 
                       input_keys=['is_running'])
        
        
        self.start_flag = False
        self.is_running = False
        self.cmd = None
        self.station_order = None
        self.start_service = rospy.Service("start_job", StartJob, self.handle_start)
        self.order_sub = rospy.Subscriber('/order_topic', Order, self.order_cb)
        self.order = Order()
        rospy.loginfo('Waiting for order...')

    def order_cb(self, msg):
        self.order = msg

    def handle_start(self, req):
        self.cmd = (req.command or "").strip().lower()
        if self.cmd == "start":
            if not self.is_running:
                self.start_flag = True        
                feedback = "Started"
                self.is_running = True
            else:
                feedback = "Running"
            return StartJobResponse(response=feedback)
        else:
            return StartJobResponse(response=f"unknown command: {req.command}")

    def execute(self, user_data):
        rospy.loginfo(f"[Idle] is_running={self.is_running}, start_flag={self.start_flag}")
        if user_data.is_running == False:
            self.is_running = False
        if self.start_flag:
            rospy.loginfo('Smach is Started.')
            self.start_flag = False            
            return 'Start'
        else:
            rospy.sleep(1)
            return 'Loop'
   
class CheckStationStatus(State):
    def __init__(self):
        State.__init__(self, outcomes=['Ready', 'Failed'], output_keys=['station_out'])
        self.order_sub = rospy.Subscriber('/order_topic', 
                                                   Order, self.order_cb)
        self.order_station = ''
        self.order = Order()

    def order_cb(self, msg):
        self.order = msg

    def execute(self, user_data):
        order_name = (self.order.job or "").strip()
        rospy.logwarn(order_name)
        if order_name is None:
            rospy.logwarn("CHECK_STATION: self.order.job is empty")
            return 'Failed'
        
        self.order_station = getattr(self.order, order_name, StationStatus())
        station_status = (self.order_station.station_status or "").strip()

        if station_status == 'Idle':
            user_data.station_out = self.order_station

            #Client needed here...
            rospy.wait_for_service('/reserve_station')
            reserve_station = rospy.ServiceProxy('/reserve_station', ReserveStation)
            try:
                req = ReserveStationRequest()
                req.station = self.order_station.station_id
                resp = reserve_station(req)
                rospy.logwarn(f"Service response {resp.response}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
            #Client needed here...

            rospy.loginfo('Ready to dock.')
            return 'Ready'
        elif station_status == 'In_Progress':
            rospy.logwarn('Station is in progress.')
            return 'Failed'
        else:
            rospy.logwarn('Unknown Command')
            return 'Failed'
        
class CheckRobotStatus(State):
    def __init__(self):
        State.__init__(self, outcomes=['Idle', 'Failed'],
                        output_keys=['idle_robot_out'])
        self.order_sub = rospy.Subscriber('/order_topic', Order, self.order_cb)
        self.order = Order()
        self.idle_robot = ''
        self.idle_robots = []
    def order_cb(self, msg):
        self.idle_robots = msg.idle_robots
    def execute(self, user_data):
        if not self.idle_robots:
            rospy.logwarn('CHECK_ROBOT: self.idle_robots is empty')
            return 'Failed'
        elif  self.idle_robots.count == 0:
            rospy.logwarn('CHECK_ROBOT: There is no Idle robot at the moment.')
            return 'Failed'
        else:
            self.idle_robot = self.idle_robots.pop()
            user_data.idle_robot_out = self.idle_robot
            rospy.wait_for_service('/reserve_robot')
            reserve_robot = rospy.ServiceProxy('/reserve_robot', ReserveRobot)

            try:
                req = ReserveRobotRequest()
                req.idle_robot = self.idle_robot
                resp = reserve_robot(req)
                rospy.logwarn(f"Service response {resp.response}")

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
            rospy.logwarn(f"{self.idle_robot} is assigned for the job")
            return 'Idle'

# class NavTo(State):
#     def __init__(self):
#         State.__init__(self, outcomes=['Sent', 'Failed'],
#                         input_keys=['station_out', 'idle_robot_out', 'is_running'],
#                           output_keys=['is_runniing'])
#         self.idle_robot = None
#         self.goal_pub = rospy.Publisher(f"{self.idle_robot}/move_base/goal", 
#                                         MoveBaseActionGoal, queue_size=10)
#         self.order_sub = rospy.Subscriber('/order_topic', Order, self.order_cb)
#         self.order = Order()

#     def order_cb(self, msg):
#         self.order = msg

#     def execute(self, user_data):
#         goal = MoveBaseActionGoal()
#         order_station = user_data.station_out
#         goal.goal.target_pose.pose.position.x = self.order.order_station.position.x
#         goal.goal.target_pose.pose.position.y = self.order.order_station.position.y
#         goal.goal.target_pose.pose.orientation.w = 1
#         goal.goal.target_pose.header.frame_id = 'map'
#         self.goal_pub.publish(goal)
#         if goal is not None:
#             rospy.loginfo('Goal is sent.')
#             user_data.is_running = False
#             return 'Sent'
#         else:
#             rospy.logwarn("Goal couldn't be sent to the order topic!")
#             return 'Failed'

class NavTo(State):
    def __init__(self):
        State.__init__(
            self,
            outcomes=['Sent', 'Failed'],
            input_keys=['station_out', 'idle_robot_out'],
            output_keys=['is_running'],
        )
        self.publishers = {}

    def _get_robot_ns(self, idle_robot_out):
        """
        Accept either a plain namespace string like 'robot1'
        or an object with a .ns/.name field.
        """
        if isinstance(idle_robot_out, str):
            return idle_robot_out.lstrip('/')  
        for attr in ('ns', 'name', 'namespace'):
            if hasattr(idle_robot_out, attr):
                return str(getattr(idle_robot_out, attr)).lstrip('/')
        return None

    def execute(self, user_data):
        robot_ns = self._get_robot_ns(user_data.idle_robot_out)
        target   = user_data.station_out  
        if not robot_ns or target is None:
            rospy.logwarn("NavTo: missing robot_ns or station_out")
            return 'Failed'

        if robot_ns not in self.publishers:
            topic = f"/{robot_ns}/move_base_simple/goal"
            self.publishers[robot_ns] = rospy.Publisher(topic, PoseStamped, queue_size=10)
            rospy.loginfo(f"NavTo: created publisher {topic}")
            start = rospy.Time.now()
            while (self.publishers[robot_ns].get_num_connections() == 0) and not rospy.is_shutdown():
                if (rospy.Time.now() - start).to_sec() > 1.0:
                    break
                rospy.sleep(0.05)

        pub = self.publishers[robot_ns]

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = target.position.x
        goal.pose.position.y = target.position.y
        goal.pose.orientation.w = 1.0  # yaw = 0

        pub.publish(goal)
        rospy.loginfo(f"NavTo: sent goal to /{robot_ns}/move_base_simple/goal "
                      f"({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f})")
        user_data.is_running = False

        # rospy.wait_for_service('release')
        # release = rospy.ServiceProxy('release', Release)

        # try:
        #         req = ReleaseRequest()
        #         req.station = user_data.station_out
        #         req.robot = user_data.idle_robot_out
        #         resp = release(req)
        #         rospy.logwarn(f"Service response {resp.response}")

        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")
        return 'Sent'

class Failed(State):
    def __init__(self):
        State.__init__(self, outcomes=['Try again'])
    
    def execute(self, user_data):
        return 'Try again'
        
def main():
    rospy.init_node('Smach')

    sm = StateMachine(outcomes=['DONE'])
    sm.userdata.is_running = False
    
    with sm:

        StateMachine.add('IDLE', Idle(), transitions={'Start':'CHECK_STATION', 'Loop':'IDLE', 'Stop': 'DONE'})
        StateMachine.add('CHECK_STATION', CheckStationStatus(), transitions={'Ready':'CHECK_ROBOT', 'Failed':'FAILED'})
        StateMachine.add('CHECK_ROBOT', CheckRobotStatus(), transitions={'Idle':'NAVTO', 'Failed':'FAILED'})
        StateMachine.add('NAVTO', NavTo(), transitions={'Sent':'IDLE', 'Failed':'FAILED'})
        StateMachine.add('FAILED', Failed(), transitions={'Try again':'IDLE'})
    
        
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
