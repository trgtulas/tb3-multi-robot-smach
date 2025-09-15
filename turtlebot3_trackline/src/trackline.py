#!/usr/bin/env python3 

import rospy 
import time
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionGoal

class Trackline:
    def __init__(self):
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.intensity_sub = rospy.Subscriber('/intensity_scan', LaserScan, self.intensity_cb)
        self.controler_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.intensities = []
        self.line_intens = 100
        self.stop_intens = 10
        self.kp = 0.1
        self.ki = 0.001
        self.kd = 1
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

        l = len(self.intensities)
        l -= 1
        while l != 0 and not rospy.is_shutdown():
            if self.intensities[l] == self.line_intens:
                self.right_most = l
                break
            l -= 1

    def move_robot(self, x = 1.8, y = 0.5):
        rospy.loginfo(f'x -> {x}  y -> {y}')
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.orientation.w = 0.91
        goal.pose.orientation.z = 0.4
        goal.pose.position.y = y
        time.sleep(1)
        self.goal_pub.publish(goal)
        msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=None)
        if msg.status.status == 3:
            return
        else:
            rospy.logwarn('robot didnt reached to goal!!!')

    def find_the_line(self):     
        cmd = Twist()  
        while self.intensities.count(self.line_intens) < 61:               
            cmd.angular.z = -0.2
            cmd.linear.x = 0
            self.controler_pub.publish(cmd)
        if  self.intensities.count(self.line_intens) > 61:
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            cmd.linear.x = 0
            cmd.linear.y = 0
            cmd.linear.z = 0
            self.controler_pub.publish(cmd)
            rospy.logwarn('Line is found.')
            return 
        else:
            rospy.logwarn("Lıne couldn't be found!!")

    def docking(self):
        rospy.logwarn(f'left_most -> {self.left_most}')
        rospy.logwarn(f'right_most -> {self.right_most}')
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
            dt = 0.00001
            correction = self.update_pid(error, dt)
            last_time = rospy.Time.now()
            cmd.angular.z = correction
            cmd.linear.x = 0.05
            self.controler_pub.publish(cmd)
            # pid updating and docking here 
            # pid updating and docking here 
            # pid updating and docking here 
            if self.stop_intens in self.intensities:
                cmd = Twist()
                cmd.angular.z = 0
                cmd.linear.x = 0
                self.controler_pub.publish(cmd)
                return


    def update_pid(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        rospy.logwarn(f'error -> {self.last_error}')
        output = (self.kp * error +
                   self.ki * self.integral +
                  self.kd * derivative
                  )
        return output   
        

def main():
    rospy.init_node('trackline')
    rospy.loginfo('Initializing Trackline...')

    trackline = Trackline()
    rospy.loginfo('Moving to goal...')
    trackline.move_robot()
    rospy.loginfo('Finding the line...')
    trackline.find_the_line()
    rospy.loginfo('Docking...')
    trackline.docking()
    rospy.loginfo('Completed operations.')

if __name__ == '__main__':
    # rospy.logdebug('degub1')
    # rospy.logdebug('degub2')
    main()
    rospy.spin() 