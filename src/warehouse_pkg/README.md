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

All the extra node types will be made in the same package, although it is better practice to move them to another package.

## Project brief (outline)

 - The fleet_manager service will provide the bay number on calling. Each `robot_navigator` is responsible for calling this service and getting a bay number based on their serial number. 
 - `robot_navigator` should linearly interpolate towards their target at a specific speed, whilst avoiding other robots using transforms. It should also track its own state, and understand when it can receive tasks, and also when it can stop in an emergency.
 - Use a best-case publish to `telemetry_monitor` under namespaces with a custom `RobotStatus` message.
 - An action client (also `fleet_manager`) is the control interface, which can send targets to `robot_navigator` (the server) for specific targets, sends feedback and return a custom result message, all in the `RobotTask` type.
