#!/usr/bin/env python3 
import rospy
from turtlebot3_smach.msg import Order
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from turtlebot3_smach.srv import StartJob, StartJobRequest, ReserveStation, ReserveStationResponse, ReserveRobot, ReserveRobotResponse, Release, ReleaseResponse

robots = {'robot1':    [   ( -1  , -0.5  )    ,   'Idle'   ],
          'robot2':    [   ( -1.5, -0.5  )    ,   'Idle'   ],           
          'robot3':    [   ( -2.0, -0.5  )    ,   'Idle'   ]
          }
     
stations = {'station1': [  ( 2.0  ,  0.5)   , 'Idle' ],
            'station2': [  ( 0.5  ,  2.0)   , 'Idle' ],
            'station3': [  ( 0.0  , -2.0)   , 'Idle' ]
            }

idle_robots = list(robots.keys())
idle_robots.reverse()

class Core:
    def __init__(self):
        self.reserve_station_service = rospy.Service("reserve_station", ReserveStation, 
                                                     self.handler_reserve_station)
        self.reserve_robot_service = rospy.Service("reserve_robot", ReserveRobot, 
                                                     self.handler_reserve_robot)
        # self.release_service = rospy.Service("release", Release, 
        #                                              self.handler_release)
        self.result_subs = {}
        self.home_goal_pubs = {}
        self.last_reserved_station = None                 
        self.assignment = {}                               
        self.returning = {robot_id: False for robot_id in robots}     


        for robot_id in robots.keys():
            topic = f'/{robot_id}/move_base/result'
            sub = rospy.Subscriber(topic, MoveBaseActionResult, self.mb_result_cb(robot_id))
            self.result_subs[robot_id] = sub
        
        for robot_id in stations.keys():
            topic = f'/{robot_id}/move_base_simple/goal'
            pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
            self.home_goal_pubs[robot_id] = pub

        self.order_topic_pub = rospy.Publisher('/order_topic', Order, queue_size=10)
        self.station_order_sub = rospy.Subscriber('/station_order', String, self.station_order_cb)
        self.waiting_station = {'robot1': (-1, -0.5), 'robot2':(-1.5, -0.5), 'robot3': (-2, -0.5)}
        self.station_order = ''
        self.order = Order()

    def mb_result_cb(self, robot_id):
        def cb(msg):
            status = msg.status.status
        
            if status != 3:
                return  

            if robots.get(robot_id)[1] == 'In_Progress':
                station_id = self.assignment.get(robot_id)
                if station_id and station_id in stations:
                    stations[station_id][1] = 'Idle'  
                    rospy.loginfo(f"{robot_id} reached {station_id} → station set Idle")
                else:
                    rospy.logwarn(f"{robot_id} reached goal but no station assignment found")

                # send robot to its home/waiting position
                home_xy = self.waiting_station.get(robot_id)
                if home_xy:
                    self._publish_home(robot_id, home_xy)
                    self.returning[robot_id] = True
                    robots[robot_id][1] = 'Returning'
                else:
                    rospy.logwarn(f"no home position for {robot_id}; leaving state as In_Progress")

                return

            if robots[robot_id][1] == 'Returning' and self.returning.get(robot_id, False):
                robots[robot_id][1] = 'Idle'
                self.returning[robot_id] = False
                if robot_id not in idle_robots:
                    idle_robots.append(robot_id)
                # clear assignment; job is done
                self.assignment.pop(robot_id, None)
                rospy.loginfo(f"{robot_id} is back home → robot set Idle")
            return cb

    def _publish_home(self, robot_id, home_xy):
        pub = self.homing_pubs[robot_id]
        start = rospy.Time.now()
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > 1.0:
                break
            rospy.sleep(0.05)

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(home_xy[0])
        goal.pose.position.y = float(home_xy[1])
        goal.pose.orientation.w = 1.0
        pub.publish(goal)
        rospy.loginfo(f"homing {robot_id} -> {home_xy}")

    def station_order_cb(self, msg):
        rospy.logwarn(f"Received new station order: {msg.data}")
        self.station_order = msg.data
        if self.station_order != None:
            self.call_start_service()
            # self.run()   
            # self.order_topic_pub.publish(self.order)

    def call_start_service(self):
        rospy.wait_for_service('/start_job')
        start_job = rospy.ServiceProxy('/start_job', StartJob)
        try:
            req = StartJobRequest()
            req.command = "start"
            resp = start_job(req)
            rospy.loginfo(f"Service response {resp.response}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def handler_reserve_station(self, req):
        rospy.logwarn(f"self.station_order -> {self.station_order}")
        if self.station_order != '':
            rospy.logwarn(f"self.station_order -> {self.station_order}")
            rospy.logwarn(f"self.order.job -> {self.order.job}")
            self.station_order = ''
            stations.get(req.station)[1] = 'In_Progress'
            response = 'Updated'
            return ReserveStationResponse(response=response)
        else:
            rospy.logwarn('lack of statemnt!!!!!')

    def handler_reserve_robot(self, req):
        idle_robot = req.idle_robot
        if idle_robots.index(idle_robot) == len(idle_robots) - 1:
            idle_robots.pop()
            robots.get(idle_robot)[1] = 'In_Progress'
            rospy.logwarn(f"{idle_robot} is assigned for the job.")
            response = 'Reserved.'
            return ReserveRobotResponse(response=response)
        else:
            rospy.logwarn('lack of statement!!!!!')

    # def handler_release(self, req):
    #     station = req.station
    #     robot = req.robot

    #     if 
    #         response = 'Released'
    #         return ReleaseResponse(response=response)
    #     else:
    #         rospy.logwarn('lack of statemnt!!!!!')
        
    def run(self):

        self.order.job = self.station_order
        self.order.idle_robots = idle_robots

        self.order.robot1.robot_id = list(robots.keys())[0]
        self.order.robot1.waiting_location.x = robots.get('robot1')[0][0]
        self.order.robot1.waiting_location.y = robots.get('robot1')[0][1]
        self.order.robot1.robot_status = robots.get('robot1')[1]

        self.order.robot2.robot_id = list(robots.keys())[1]
        self.order.robot2.waiting_location.x = robots.get('robot2')[0][0]
        self.order.robot2.waiting_location.y = robots.get('robot2')[0][1]
        self.order.robot2.robot_status = robots.get('robot2')[1]

        self.order.robot3.robot_id = list(robots.keys())[2]
        self.order.robot3.waiting_location.x = robots.get('robot3')[0][0]
        self.order.robot3.waiting_location.y = robots.get('robot3')[0][1]
        self.order.robot3.robot_status = robots.get('robot3')[1]
        
        self.order.station1.station_id = list(stations.keys())[0]
        self.order.station1.position.x = stations.get('station1')[0][0]
        self.order.station1.position.y = stations.get('station1')[0][1]
        self.order.station1.station_status = stations.get('station1')[1]

        self.order.station2.station_id = list(stations.keys())[1]
        self.order.station2.position.x = stations.get('station2')[0][0]
        self.order.station2.position.y = stations.get('station2')[0][1]
        self.order.station2.station_status = stations.get('station2')[1]

        self.order.station3.station_id = list(stations.keys())[2]
        self.order.station3.position.x = stations.get('station3')[0][0]
        self.order.station3.position.y = stations.get('station3')[0][1]
        self.order.station3.station_status = stations.get('station3')[1]

        self.order_topic_pub.publish(self.order)


def main():
    
    rospy.logwarn_once('debug main')
    while not rospy.is_shutdown():
        rospy.loginfo('Data is being sent...')
        core.run()
        rospy.sleep(0.3)
        


if __name__ == '__main__':
    rospy.init_node('Core_node')
    core = Core()

    main()