# Policy Inference Pipeline

This document is the operational runbook for policy rollout with `vt_franka`.

Current scope of `vt_franka` rollout:

- Single-arm Franka only
- Controller-side execution through `vt-franka-controller`
- Workspace-side rollout through `vt-franka-workspace rollout`
- Policy entrypoint is a Python callable specified as `module:function`

Because the new repo does not yet contain the original RDP training or checkpoint loader, inference is driven by a policy wrapper function. A smoke-test policy is included in:

- [policies.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/examples/policies.py)

## 0. Required Setup Documents

This rollout runbook assumes the machines are already installed and configured.

Use these setup documents first:

- [controller_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/controller_setup.md)
- [workspace_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/workspace_setup.md)

This document does not repeat installation commands.

## 1. Which Processes Are Required

### Required for rollout

- Controller machine:
  - Polymetis robot server
  - Polymetis gripper server
  - `vt-franka-controller`
- Workspace machine:
  - `vt-franka-workspace state-bridge`
  - `vt-franka-workspace rollout`

### Recommended during rollout

- `vt-franka-workspace teleop`
  - keeps the `get_current_gripper_state` endpoint alive for GelSight latency matching
  - allows manual takeover if needed
- `vt-franka-workspace gelsight`
  - if you want tactile arrows on Quest or tactile recording during rollout

## 2. Policy Environment

Default path:

- Run `vt-franka-workspace rollout` inside the existing `vt-franka-workspace` conda environment.
- You do not need a second conda environment by default.

If your policy has conflicting dependencies:

- Create a separate inference environment.
- Install `vt-franka-workspace` and your policy package into that environment.
- Run only the `rollout` process in that separate environment.
- Keep `teleop`, `state-bridge`, and optional sensor publishers in the original workspace environment if you want.

## 3. Start the Controller Side

Open three terminals on the controller machine.

Before opening the terminals, run this preflight once:

```bash
ping -c 3 172.16.0.2
uname -a
cat /sys/kernel/realtime
groups | grep realtime
ulimit -r
ulimit -l
```

### Terminal C1: Polymetis robot server

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

What this launches:

- Polymetis Franka robot gRPC server on `127.0.0.1:50051`

### Terminal C2: Polymetis gripper server

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

What this launches:

- Polymetis Franka hand gRPC server on `127.0.0.1:50052`

### Terminal C3: Controller API

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
vt-franka-controller run --config /home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml
```

What this launches:

- FastAPI controller service on `0.0.0.0:8092`
- The 300 Hz controller loop that accepts higher-level workspace commands

Health checks:

```bash
curl http://127.0.0.1:8092/api/v1/health
curl http://127.0.0.1:8092/api/v1/state
```

## 4. Start the Workspace Side

Open up to four terminals on the workspace machine.

### Terminal W1: Optional episode recording

If you want to record the rollout:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace episode-start \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --name rollout_001
```

What this does:

- Creates an active rollout episode directory

### Terminal W2: Robot state bridge

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace state-bridge \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- Controller-state polling and Quest UDP feedback publishing

### Terminal W3: Optional teleop server

Recommended if you want manual takeover or proper GelSight latency matching:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace teleop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- Quest teleop HTTP server on `<WORKSPACE_IP>:8082`
- Manual takeover path if you want to interrupt rollout

### Terminal W4: Optional GelSight publisher

If you want tactile arrows and tactile recording during rollout:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace gelsight \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- GelSight capture and tactile recording or Quest tactile visualization

## 5. Smoke-Test the Rollout Stack

The built-in `hold_current_pose` policy simply keeps the robot at its current TCP pose and current gripper width.

Run this first before connecting your real model wrapper:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace rollout \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --policy vt_franka_workspace.examples.policies:hold_current_pose
```

A second smoke-test policy moves the TCP slightly along `+x`:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace rollout \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --policy vt_franka_workspace.examples.policies:nudge_x
```

## 6. Run Your Own Policy Wrapper

Your policy function must return either:

```python
def my_policy(observation: dict) -> dict:
    return {
        "target_tcp": [x, y, z, qw, qx, qy, qz],
        "gripper_width": 0.05,
    }
```

or:

```python
def my_policy(observation: dict) -> dict:
    return {
        "target_tcp": [x, y, z, qw, qx, qy, qz],
        "gripper_closed": True,
    }
```

The current observation always includes:

- `observation["controller_state"]`
- optionally `observation["tactile"]` if you extend `RealWorldEnv`

### Example: local wrapper module

Create a wrapper file, for example:

```bash
mkdir -p /home/zhenya/kenny/visuotact/vt_franka/local_policies
```

```python
# /home/zhenya/kenny/visuotact/vt_franka/local_policies/my_policy.py
def rollout_policy(observation: dict) -> dict:
    state = observation["controller_state"]
    return {
        "target_tcp": state["tcp_pose"],
        "gripper_width": state["gripper_width"],
    }
```

Run it:

```bash
export PYTHONPATH=/home/zhenya/kenny/visuotact/vt_franka:$PYTHONPATH

conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace rollout \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --policy local_policies.my_policy:rollout_policy
```

## 7. Integrate A Real Checkpoint Loader

If you want to roll out an actual RDP checkpoint, wrap your model-loading code behind a `module:function` entrypoint and make the function return the action dict expected above.

Typical pattern:

1. Load the checkpoint once at module import time or on first call.
2. Convert `observation["controller_state"]` and optional tactile inputs into your model input tensors.
3. Run the forward pass.
4. Convert model output into `target_tcp` and gripper commands.

The new repo does not yet include the original training workspace or checkpoint format loader, so this wrapper is the integration point.

## 8. Stop And Postprocess Recorded Rollouts

If you started an episode:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace episode-stop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

Then align the recorded streams:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace postprocess \
  --episode-dir EPISODE_DIR
```
