Start simulation first:

```bash
roslaunch turtlebot_smach spawn_robots.launch
roslaunch multi_navigation multi_turtlebot3_navigation.launch
```
Start smach and master module:

```bash
roslaunch turtlebot_smach smach.launch
rosrun turtlebot3_smach master.py
```

Create new job by publish message to `/station_order` topic:
Ex:
```bash
rostopic pub /station_order std_msgs/String "data: 'station1'"
```
