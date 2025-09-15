#!/usr/bin/env python3
import rospy
from turtlebot3_smach.srv import StartJob, StartJobResponse



def handle_start(req):
    global start_flag, is_running
    cmd = (req.command or "").strip().lower()

    if cmd == "start":
        start_flag = True          # your SMACH loop should consume this
        feedback = "Started"

        return StartJobResponse(response=feedback)

    else:
        return StartJobResponse(response=f"unknown command: {req.command}")

def main():
    rospy.init_node("job_start_server")
    rospy.Service("start_job", StartJob, handle_start)
    rospy.loginfo("Service /start_job (StartJob) ready.")
    rospy.spin()

if __name__ == "__main__":
    main()