# VT Franka Controller

This package is meant for the Franka-side machine only.

It owns:

- the Polymetis robot and gripper interfaces
- the 300 Hz interpolated Cartesian impedance control loop
- the controller HTTP API used by the workspace machine

It does not include Quest teleop, GelSight, ROS2 publishers, or recording.

