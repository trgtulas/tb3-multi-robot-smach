#!/usr/bin/env python3 
import rospy
from turtlebot3_smach.msg import Order
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionResult
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from turtlebot3_smach.srv import StartJob, StartJobResponse, EndJob, EndJobResponse

robots = {'robot1':    [   ( -1.0  , -0.5  )    ,   'Idle'   ],
          'robot2':    [   ( -1.5, -0.5  )    ,   'Idle'   ],           
          'robot3':    [   ( -2.0, -0.5  )    ,   'Idle'   ]
          }
     
stations = {'station1': [  ( 1.7  ,  0.45)   , 'Idle' ],
            'station2': [  ( 0.25  ,  2.0)   , 'Idle' ],
            'station3': [  ( -0.3  , -2.0)   , 'Idle' ]
            }

idle_robots = list(robots.keys())

class Core:
    def __init__(self):
        self.result_subs = {}
        self.waiting_station = {'robot1': (-2.0, 0.5), 'robot2':(-2.0, 0.0), 'robot3': (-2.0, -0.5)}
        self.order = Order()
        self.assigned_jobs = {}
        self.active_jobs = {}
        self.job = None
        for robot_id in robots.keys():
            topic = f'/{robot_id}/move_base/result'
            sub = rospy.Subscriber(topic, MoveBaseActionResult, self.mb_result_cb(robot_id))
            self.result_subs[robot_id] = sub
        
        self.get_order_sub = rospy.Subscriber('/station_order', String, self.station_order_cb)
        self.order_topic_pub = rospy.Publisher('/order_topic', Order, queue_size=10)

        for robot_id in robots.keys():
            rospy.Service(f'/{robot_id}/start_job', StartJob, self.start_handler(robot_id))

        for robot_id in robots.keys():
            rospy.Service(f'/{robot_id}/end_job', EndJob, self.end_job_handler(robot_id))

        rospy.loginfo("Core: ready (idle_robots=%s)", idle_robots)

    def station_order_cb(self, msg):
        station_id = (msg.data or '').strip()

        if not idle_robots:
            rospy.logwarn('No idle robot right now')
            return
        
        robot_id = idle_robots.pop(0)
        self.assigned_jobs[robot_id] = station_id  

    def start_handler(self, robot_id):
        def handle_start(req):
            status = req.status
            resp = StartJobResponse()
            station_id = self.assigned_jobs.get(robot_id, None)

            if not station_id:
                rospy.logwarn(f'No order for {robot_id} at the moment')
                resp.command = 'wait'
                return resp
            
            if station_id not in list(stations.keys()):
                rospy.logwarn('Wrong station_id')
                self.assigned_jobs.pop(robot_id, None)
                idle_robots.insert(0, robot_id)
                resp.command = 'error'
                return resp

            if stations.get(station_id)[1] == 'In_Progress':
                rospy.logwarn(f'{station_id} is in progress')
                resp.command = 'wait'
                return resp 
            
            if status == 'waiting' and stations[station_id][1] == 'Idle':
                rospy.logwarn(f'start with {station_id}')
                resp.station.x = stations[station_id][0][0]
                resp.station.y = stations[station_id][0][1]
                stations.get(station_id)[1] = 'In_Progress'
                robots[robot_id][1] = 'In_Progress'
                resp.command = 'start'
                self.assigned_jobs.pop(robot_id, None)
                self.active_jobs[robot_id] = station_id
                return resp
        return handle_start
    
    def end_job_handler(self, robot_id):
        def handle_end(req):
            resp = EndJobResponse()
            if robot_id not in robots:
                rospy.logwarn(f'No valid robot_id -> {robot_id}')
                resp.response = 'error'
                return resp
            
            station_id = self.active_jobs[robot_id]
            if station_id not in stations:
                rospy.logwarn(f'No valid station_id -> {station_id}')
                resp.response = 'error'
                return resp

            if robot_id not in self.active_jobs:
                rospy.logwarn(f'No active job for {robot_id}')
                resp.response = 'error'
                return resp

            if self.active_jobs[robot_id] == station_id:
                stations[station_id][1] = 'Idle'
                robots[robot_id][1] = 'Idle'
                resp.waiting_location.x = self.waiting_station[robot_id][0]
                resp.waiting_location.y = self.waiting_station[robot_id][1]
                idle_robots.append(robot_id)
                self.active_jobs.pop(robot_id, None)
                resp.response = 'done'
                return resp
            return resp
        return handle_end


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


    def run(self):
        self.order.idle_robots = idle_robots
        self.order.assigned_jobs = self.assigned_jobs
        self.order.active_jobs = self.active_jobs
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
    
    while not rospy.is_shutdown():
        # rospy.loginfo('Data is being sent...')
        core.run()
        rospy.sleep(0.3)
        


if __name__ == '__main__':
    rospy.init_node('Master_Node')
    core = Core()
    main()