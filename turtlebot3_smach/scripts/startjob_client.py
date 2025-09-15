#!/usr/bin/env python3
import rospy
from turtlebot3_smach.srv import StartJob

def main():
    rospy.init_node("job_start_client")
    rospy.wait_for_service("start_job")
    start_job = rospy.ServiceProxy("start_job", StartJob)
    rospy.logwarn('loooop')
    # send the command
    resp = start_job(command="start")
    rospy.loginfo(f"Server feedback: {resp.response}")

if __name__ == "__main__":
    main()