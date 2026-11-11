# Four-module source graph

This diagram describes source connections, not a live ROS capture. The approved
obsolete partial-layout diagrams and their generators have been removed.

```mermaid
flowchart LR
  Operator[Operator or autonomy] --> Supervisor
  Supervisor -->|vehicle/cmd_vel| ROMB[Original ROMB and adapter]
  ROMB -->|kinematics/joint_setpoints| Supervisor
  Supervisor -->|wheel_modules/command| Driver[miControl SDO driver]
  Driver --> FL[front_left: 1 and 2]
  Driver --> FR[front_right: 3 and 4]
  Driver --> RL[rear_left: 7 and 8]
  Driver --> RR[rear_right: 5 and 6]
  Driver -->|joint_states| ROMB
  ROMB -->|vehicle/measured_velocity| Odometry
  Driver -->|wheel_modules/status| Supervisor
```

All four modules and eight joints are required. Existing operator, battery,
scanner and safety inputs retain their current paths. See the
[four-module setup](four_module_setup.md).
