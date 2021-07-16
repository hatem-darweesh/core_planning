# op_blinker_propagation node

This node subscribes to `op_current behavior` ([autoware_msgs/Waypoint](https://github.com/streetdrone-home/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/Waypoint.msg)) and then publishes to `/lamp_cmd`  ([autoware_msgs/LampCmd](https://github.com/Autoware-AI/messages/blob/master/autoware_msgs/msg/LampCmd.msg))

The `/lamp_cmd` topic is then used by the [`twist_gate`](https://github.com/Autoware-AI/core_planning/tree/master/twist_gate) topic to propagate the lamp commands to the SSC and finally to the PACMOD system.

```
┌────────────────────┐   ┌──────────────────────┐   ┌─────────┐   ┌──────────┐
│/op_current_behavior├──►│op_blinker_propagation├──►│/lamp_cmd├──►│twist_gate│
└────────────────────┘   └──────────────────────┘   └─────────┘   └──────────┘
```