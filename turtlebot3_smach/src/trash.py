#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

order = None
station = {
    'station1': [2, 0.5, 0],   # [x, y, status] where status: 0=free, 1=occupied
    'station2': [0.5, 2, 0],
    'station3': [0, -2, 0]
}

def check_station(name: str) -> bool:
    """Return True if station becomes reserved (was free), False otherwise."""
    if name is None:
        return False
    if name not in station:
        rospy.logwarn(f"Unknown station: {name}")
        return False
    # status index = 2
    if station[name][2] == 0:
        station[name][2] = 1
        rospy.loginfo(f"{name} ready to dock (now reserved).")
        return True
    else:
        rospy.loginfo_throttle(5.0, f"{name} is already occupied.")
        return False

def order_callback(msg: String):
    global order
    order = msg.data.strip()
    rospy.loginfo(f"Received order: {order}")

def main():
    rospy.init_node('check_order', anonymous=True)
    rospy.Subscriber('/station_order', String, order_callback)
    global order
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if order:
            if check_station(order):
                # Processed this order; clear it so we don't reserve repeatedly
                rospy.loginfo(f"Order handled: {order}")
                # If you want to trigger other actions, do it here.
                # (e.g., publish a goal, notify another node, etc.)
                pass
            # Clear the order regardless to avoid re-processing the same message
            # Remove this line if you want to keep retrying when occupied.
            order_processed = order
            order = None
            rospy.logdebug(f"Cleared order: {order_processed}")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
