# Operator Collection Pipeline V2

### 3.1 Controller machine

The controller machine still runs the low-level robot stack as a long-lived service:

- Polymetis robot server
- Polymetis gripper server
- `vt-franka-controller run`

### 3.2 Workspace machine

Normal collection should use one command:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace collect \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --run fold_cloth
```

This enters an operator mode with:

- health checks
- worker startup
- live status HUD
- hotkeys
- episode lifecycle management
- automatic postprocess and QC

Recommended hotkeys:

- `R`: start next episode
- `E`: end and save current episode
- `D`: discard current episode
- `H`: move robot to configured `ready` pose
- `Q`: quit operator mode

Optional:

- `N`: annotate the current episode with a short note
- `P`: pause teleop command forwarding without shutting workers down

## 4. Process Model

V2 should keep two modes:

- Debug mode:
  - existing commands remain unchanged
- Operator mode:
  - new `collect` command supervises all workers in one process

The `collect` supervisor should own these workers:

- Quest teleop HTTP server
- controller state monitor
- Quest feedback publisher
- optional Orbbec recorder
- optional GelSight recorder
- recording coordinator
- operator UI / hotkey loop

This should be implemented in-process, not as another shell wrapper.

That is feasible with the current codebase because:

- [QuestTeleopService](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/teleop/quest_server.py) already has `start()` and `stop()` for its control loop
- [StateBridge](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/publishers/state_bridge.py) already has `start()` and `stop()`
- [OrbbecRgbRecorder](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/sensors/orbbec/recorder.py) already accepts a `stop_event`
- [GelsightPublisher](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/sensors/gelsight/publisher.py) already accepts a `stop_event`

The only part that needs a thin integration wrapper is the FastAPI server startup for teleop.

## 5. Core Architecture Changes

### 5.1 Replace duplicate controller polling with one shared monitor

This is the most important robustness change.

Current behavior:

- teleop pulls controller state over HTTP every loop
- state bridge also pulls controller state over HTTP every loop

V2 should introduce a shared `ControllerStateMonitor` on the workspace side:

- polls `GET /api/v1/state` once at a configured rate, for example `30` to `60 Hz`
- stores the latest state, timestamps, success/failure counters, and gap statistics
- provides thread-safe access to the latest cached state
- feeds:
  - teleop control
  - Quest state/force feedback
  - controller-state recording
  - operator HUD
  - QC summary

Effects:

- removes redundant network traffic
- reduces controller API load
- centralizes timeout handling
- makes degraded-state detection much easier

### 5.2 Replace file-based episode control with an in-memory coordinator

Current recording uses [EpisodeSessionManager](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/recording/session.py) plus `active_episode.json`.

That should remain for debug compatibility, but operator mode should use an in-memory `RecordingCoordinator` as the source of truth.

The coordinator should:

- own current run state
- own current episode state
- create directories and manifests
- expose `is_recording` to workers
- support pre-roll and post-roll buffering
- mirror a small `active_episode.json` file only for crash recovery or external inspection

### 5.3 Introduce ring-buffered recording

Workers should be started once and kept alive, but they should not continuously write to disk.

Each stream should support:

- preview mode:
  - data flows
  - health is visible
  - no disk writes
- buffered mode:
  - keep the last `N` seconds in memory
- recording mode:
  - flush pre-roll buffer into the new episode
  - then append live samples to disk

Recommended defaults:

- controller state pre-roll: `2.0 s`
- Orbbec pre-roll: `1.0 s`
- GelSight pre-roll: `1.0 s`
- teleop command pre-roll: `1.0 s`

This removes the current tradeoff between:

- starting everything early and recording too much junk
- starting everything late and missing the first useful seconds

### 5.4 Make internal dependencies direct, not HTTP

Current GelSight code fetches gripper status by calling the workspace teleop HTTP endpoint.

In operator mode that should become a direct in-process dependency:

- the teleop worker exposes current gripper state through a thread-safe provider
- GelSight reads that provider directly

The HTTP endpoint can remain for compatibility, but it should no longer be the internal data path in `collect` mode.

## 6. Operator State Machine

The workspace supervisor should run the following state machine:

1. `BOOTING`
2. `PREFLIGHT`
3. `READY`
4. `PREROLL`
5. `RECORDING`
6. `STOPPING`
7. `POSTPROCESSING`
8. `DEGRADED`
9. `SHUTDOWN`

Behavior:

- `BOOTING`
  - start workers
- `PREFLIGHT`
  - verify controller health, Quest heartbeat, calibration, optional sensor readiness
- `READY`
  - all required workers healthy
  - operator may press `R` or `H`
- `PREROLL`
  - create episode directory
  - flush ring buffers
  - wait short configurable countdown, for example `2.0 s`
- `RECORDING`
  - all enabled streams append to the current episode
- `STOPPING`
  - stop writes
  - finalize manifests
- `POSTPROCESSING`
  - align the episode
  - compute QC summary
- `DEGRADED`
  - block new episode start until required health recovers

Policy for failures during recording:

- controller state monitor failure is critical
- Quest heartbeat loss is warning-level unless teleop is the active control source
- optional sensor failures mark the episode degraded, but do not necessarily require aborting the episode

Suggested thresholds:

- if controller-state gap exceeds `0.5 s`, mark episode degraded
- if controller-state gap exceeds `2.0 s`, auto-stop the episode and mark it unusable for training

## 7. Run and Episode Structure

V2 should add a run layer above episodes.

Recommended layout:

```text
data/
  runs/
    task_xxx_20260414_173000/
      run_manifest.json
      operator_events.jsonl
      latest_status.json
      episodes/
        episode_0000/
          episode_manifest.json
          qc.json
          aligned_episode_manifest.json
          aligned_episode.npz
          streams/
            controller_state.jsonl
            teleop_commands.jsonl
            quest_messages.jsonl
            orbbec_rgb.jsonl
            gelsight_markers.jsonl
            orbbec_rgb/
            gelsight_frames/
        episode_0001/
          ...
```

`run_manifest.json` should contain:

- run id
- task name
- operator
- workspace hostname
- controller host
- calibration version
- enabled sensors
- configuration snapshot
- start time

`episode_manifest.json` should contain:

- episode index
- episode id
- run id
- operator notes
- start time
- stop time
- outcome
- health summary
- enabled streams

## 8. Ready Pose Design

The ready pose should move into controller config and controller API.

Recommended controller config extension:

```yaml
control:
  named_joint_poses:
    home: [...]
    ready: [...]
  named_pose_durations_sec:
    home: 8.0
    ready: 5.0
```

Recommended controller API extension:

- `POST /api/v1/actions/ready`
- or a generic `POST /api/v1/actions/named-pose` with `{"name": "ready"}`

Why joint-space first:

- it is already compatible with the current controller design
- it is safer than asking operators to edit a Cartesian pose script
- it avoids adding IK or motion-generation complexity to the first V2 pass

If a Cartesian ready pose is still desired later, it should be added as a second feature, not the default v2 path.

## 9. What Is Recorded

### 9.1 Default raw streams

Required:

- `controller_state`
- `teleop_commands`

Optional by config:

- `orbbec_rgb`
- `gelsight_markers`
- `gelsight_frames`
- `quest_messages`

`quest_messages` should be debug-only by default.

The current raw Quest payload log is useful for debugging, but it is usually not necessary for the dataset itself and can grow quickly.

### 9.2 Default aligned training export

V2 should keep TCP-space action as the primary action interface because the current real environment already executes:

- `target_tcp`
- `gripper_closed` or `gripper_width`

See [real_env.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/rollout/real_env.py).

However, the aligned dataset should export both TCP and joint proprioception by default.

Recommended aligned arrays:

- `timestamps`
- `robot_tcp_pose`
- `robot_tcp_velocity`
- `robot_tcp_wrench`
- `robot_joint_positions`
- `robot_joint_velocities`
- `gripper_width`
- `gripper_force`
- `action_target_tcp`
- `action_gripper_closed`
- `action_gripper_width`
- `controller_state_valid`
- `controller_state_age_sec`
- `orbbec_rgb_frame_paths`
- `gelsight_marker_locations`
- `gelsight_marker_offsets`

This is a direct improvement over the current [postprocess.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/recording/postprocess.py), which exports TCP state and gripper state but not joint arrays.

### 9.3 Semantics

Proprioception should be defined as:

- primary:
  - TCP pose
  - TCP velocity
  - TCP wrench
  - gripper width
  - gripper force
- additional:
  - joint positions
  - joint velocities

Action should be defined as:

- primary:
  - commanded TCP target
  - commanded gripper state

Joint action should not be the main action format in V2 because the current teleop and rollout stack do not command joints.

## 10. QC and Health Reporting

Every episode should automatically produce `qc.json`.

At minimum it should report:

- controller-state sample count
- controller-state effective Hz
- maximum controller-state gap
- teleop command effective Hz
- Orbbec effective Hz
- GelSight effective Hz
- stream start and end times
- missing-stream flags
- degraded-health intervals
- pass/fail recommendation for training use

Suggested rules:

- fail if controller state is missing entirely
- fail if maximum controller-state gap exceeds `2.0 s`
- warn if controller-state rate drops below the configured minimum
- warn if enabled sensors never produced samples

This is needed because a raw episode can exist on disk while still being unsuitable for training.

## 11. CLI Surface

Recommended new workspace commands:

- `vt-franka-workspace collect`
  - main operator mode
- `vt-franka-workspace check`
  - one-shot preflight and connectivity check
- `vt-franka-workspace postprocess-run`
  - optional bulk postprocess for all episodes in a run

Recommended controller additions:

- `vt-franka-controller check`
  - local health and bind-address check
- controller API named-pose action

The current commands should remain:

- `teleop`
- `state-bridge`
- `orbbec`
- `gelsight`
- `episode-start`
- `episode-stop`
- `postprocess`

Those remain useful for development and isolated debugging.

## 12. Settings Additions

Recommended additions to [settings.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/settings.py):

- `collect.required_workers`
- `collect.optional_workers`
- `collect.pre_roll_sec`
- `collect.post_roll_sec`
- `collect.start_countdown_sec`
- `collect.auto_postprocess`
- `collect.auto_qc`
- `collect.record_raw_quest_messages`
- `collect.controller_gap_warn_sec`
- `collect.controller_gap_fail_sec`
- `collect.quest_heartbeat_timeout_sec`
- `collect.default_run_root`

Recommended additions to controller settings:

- named poses
- named pose durations
- optional controller-side health thresholds

## 13. Implementation Plan

### Phase 1: control plane

- add `collect` CLI entry
- add run manager
- add operator HUD and hotkeys
- add shared controller-state monitor
- keep existing stream recorders but gate them through a new coordinator

### Phase 2: recording robustness

- add ring buffers
- add pre-roll and post-roll
- add per-episode events log
- add QC summary

### Phase 3: controller ergonomics

- add named ready pose to controller config and API
- add workspace `H` hotkey
- add bulk run postprocess

### Phase 4: dataset cleanup

- export joints in aligned episodes
- make raw Quest logging optional
- add validity masks and latency metrics

## 14. Concrete Code Mapping

Recommended new workspace modules:

- `robot_workspace/src/vt_franka_workspace/collect/supervisor.py`
- `robot_workspace/src/vt_franka_workspace/collect/controller_monitor.py`
- `robot_workspace/src/vt_franka_workspace/collect/operator_ui.py`
- `robot_workspace/src/vt_franka_workspace/collect/run_manager.py`
- `robot_workspace/src/vt_franka_workspace/collect/health.py`
- `robot_workspace/src/vt_franka_workspace/collect/workers.py`

Recommended existing modules to extend:

- [cli.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/cli.py)
- [settings.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/settings.py)
- [session.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/recording/session.py)
- [postprocess.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/recording/postprocess.py)
- [client.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/controller/client.py)
- [app.py](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/src/vt_franka_controller/api/app.py)
- [service.py](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/src/vt_franka_controller/control/service.py)
- [settings.py](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/src/vt_franka_controller/settings.py)

## 15. Recommended Default V2 Behavior

If implemented today, the default operator experience should be:

1. Start controller stack on controller PC.
2. Run `vt-franka-workspace collect --run task_xxx` on workspace PC.
3. Wait until the HUD shows:
   - controller healthy
   - Quest connected
   - required sensors ready
4. Press `H` if you want the robot moved to the configured ready pose.
5. Set up the scene.
6. Press `R`.
7. Wait through a short countdown.
8. Collect the demo.
9. Press `E`.
10. Review the immediate QC result.
11. Repeat with `R` for the next episode.
12. Press `Q` when the run is finished.

This keeps the current architecture recognizable, but changes the operator experience from a debug workflow into a data-collection workflow.
