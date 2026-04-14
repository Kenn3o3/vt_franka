# VT Franka

Clean split of the Franka real-world stack used for visuotactile teleoperation and reactive policy deployment.

## Layout

- `shared/`: cross-machine models, math, calibration, timing, and interpolation utilities.
- `robot_controller/`: Polymetis-backed high-frequency controller for the Franka-side machine.
- `robot_workspace/`: Quest teleop, Quest feedback publishing, GelSight and Orbbec handling, recording, and rollout utilities.
- `docs/`: setup, architecture, and migration notes.
- `ops/`: shell helpers for common launch patterns.

The original `factoryil` tree is left untouched and is used only as a reference source.
