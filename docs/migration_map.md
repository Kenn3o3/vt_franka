# Migration Map

## Old repo to new repo

- `reactive_diffusion_policy/real_world/robot/franka_server.py`
  - now `robot_controller/src/vt_franka_controller/...`
- `reactive_diffusion_policy/teleop.py`
  - split into `robot_workspace/teleop/quest_server.py` and `robot_workspace/publishers/state_bridge.py`
- `reactive_diffusion_policy/real_world/publisher/gelsight_camera_publisher.py`
  - now `robot_workspace/sensors/gelsight/...`
- `reactive_diffusion_policy/record_data.py` and `real_world/teleoperation/data_recorder.py`
  - replaced by the raw per-stream recording model in `robot_workspace/recording/...`
- `reactive_diffusion_policy/env/real_bimanual/real_env.py`
  - replaced by the lighter single-arm `robot_workspace/rollout/real_env.py`

## Intentional v1 changes

- Single-arm only
- Polymetis only
- No Hydra in the real-world stack
- No ROS2 dependency in the critical control/teleop path
- Recording is stream-based rather than synchronized message-filter capture

