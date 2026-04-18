# Data Collection Pipeline 

### 3.1 Controller machine

The controller machine still runs the low-level robot stack as a long-lived service:

- Polymetis robot server
- Polymetis gripper server
- `vt-franka-controller run`

Run the following commands in three terminals:

Terminal C1:

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

Terminal C2:

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

Terminal C3:

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/robot_controller
vt-franka-controller run --config /home/medair/vt_franka/robot_controller/config/controller.yaml
```

### 3.2 Workspace machine

Normal collection should use one command:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace collect \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --run fold_cloth
```

Visualize:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace visualize \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --episode-dir /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/data/runs/rotate_spoon_20260415_174623/episodes/episode_0000
```

Replay:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace rollout-once \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --policy vt_franka_workspace.rollout.replay_policy:build_replay_policy \
  --episode-dir /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/data/runs/fold_cloth_20260415_185010/episodes/episode_0000 \
  --go-ready
```

This enters an operator mode with:

- health checks
- worker startup
- browser operator UI on `http://127.0.0.1:8083/operator`
- hotkeys
- episode lifecycle management
- automatic postprocess and QC

The browser UI is the primary operator surface:

- structured readiness and worker state
- recent logs in a dedicated panel instead of terminal spam
- buttons for reset/start/stop/discard/quit
- a frozen Orbbec pre-episode view when the next episode is allowed to start

Terminal hotkeys still work as a fallback.

Recommended hotkeys:

- `R`: start next episode
- `E`: end and save current episode
- `D`: discard current episode
- `H`: move robot to configured `ready` pose
- `Q`: quit operator mode

Optional:

- `N`: annotate the current episode with a short note
- `P`: pause teleop command forwarding without shutting workers down

Postprocessing semantics for `aligned_episode.npz`:

- `timestamps` are observation times on the aligned grid.
- Proprioception and camera fields are selected causally from the latest sample at or before each aligned timestamp.
- Teleop action fields are selected from the first teleop command strictly after each aligned timestamp.
- Steps without a valid near-future teleop action are dropped from the aligned training episode instead of being paired with stale commands.
- The aligned file also stores per-step source timestamps so the observation/action pairing can be audited directly.
