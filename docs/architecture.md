# Architecture

## Split

- `robot_controller`
  - Runs on the Franka-side machine.
  - Owns Polymetis, the 300 Hz control loop, and the controller HTTP API.
- `robot_workspace`
  - Runs on the workstation.
  - Owns Quest teleop, Quest feedback publishing, GelSight, recording, and rollout.
- `shared`
  - Owns the cross-machine contracts and motion math.

## Runtime data flow

1. Quest posts controller poses to `robot_workspace` on HTTP `:8082`.
2. Workspace teleop converts Unity poses into the Franka robot frame and sends TCP targets to `robot_controller`.
3. Controller interpolates and updates Polymetis Cartesian impedance targets at 300 Hz.
4. Workspace polls controller state and publishes robot pose and force arrows back to Quest over UDP.
5. GelSight captures locally on the workspace, sends tactile arrows to Quest, and records raw marker streams.
6. Each workspace component records its own raw stream into the active episode directory.
7. Offline postprocessing aligns those streams by timestamp.

## Recording model

The clean repo does not depend on ROS time synchronizers for v1. Instead:

- `quest_messages.jsonl`: raw Quest controller input
- `teleop_commands.jsonl`: translated TCP and gripper commands
- `controller_state.jsonl`: controller state sampled by the state bridge
- `gelsight_markers.jsonl`: normalized tactile marker locations and offsets
- `gelsight_frames/`: optional recorded tactile frames

An episode is “active” when `recording/active_episode.json` exists under the workspace recording root.

