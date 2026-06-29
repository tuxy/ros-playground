# ROS2 warehouse/logistics system

A sample project I made to learn a little more about ROS.

Right now:

```
robot_navigator -> (Pose2D) -> telemetry_monitor
robot_navigator -> (GetBayNumber) -> robot_navigator
```

No movement has been implemented, although I plan to add:
 - `robot_navigator` as an action server (gets target goal, moves to target goal)
 - Using `tf2` for transforms to determine robot location in spawnCallback
 - Implementing lifecycles and status messages for `robot_navigator`

