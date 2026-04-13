# Policy Inference Pipeline

This document is the step-by-step command runbook for policy rollout with `vt_franka`.

Current scope of `vt_franka` rollout:

- Single-arm Franka only
- Controller-side execution through `vt-franka-controller`
- Workspace-side rollout through `vt-franka-workspace rollout`
- Policy entrypoint is a Python callable specified as `module:function`

Because the new repo does not yet contain the original RDP training/checkpoint loader, inference is driven by a policy wrapper function. A smoke-test policy is included in:

- [policies.py](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/src/vt_franka_workspace/examples/policies.py)

## 0. When to Run Which Processes

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

## 1. One-Time Setup

Use the same base setup as:

- [controller_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/controller_setup.md)
- [workspace_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/workspace_setup.md)

For a fresh controller machine, do not skip the controller-host prerequisites in `controller_setup.md`:

- direct 1 GbE wired link to the robot
- `PREEMPT_RT` kernel booted
- realtime group and `limits.conf` configured
- `LIBFRANKA_VERSION` matched to the robot firmware
- Polymetis built against that matching `libfranka`

## 2. Start the Controller Side

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

### Terminal C2: Polymetis gripper server

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

### Terminal C3: Controller API

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
vt-franka-controller run --config /home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml
```

Health checks:

```bash
curl http://127.0.0.1:8092/api/v1/health
curl http://127.0.0.1:8092/api/v1/state
```

## 3. Start the Workspace Side

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

### Terminal W2: Robot state bridge

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace state-bridge \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

### Terminal W3: Optional teleop server

Recommended if you want manual takeover or proper GelSight latency matching:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace teleop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

### Terminal W4: Optional GelSight publisher

If you want tactile arrows and tactile recording during rollout:

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace gelsight \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

## 4. Smoke-Test the Rollout Stack With a Built-In Policy

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

## 5. Run Your Own Policy Wrapper

Your policy function must satisfy:

```python
def my_policy(observation: dict) -> dict:
    ...
    return {
        "target_tcp": [x, y, z, qw, qx, qy, qz],
        "gripper_width": 0.05,
    }
```

or:

```python
def my_policy(observation: dict) -> dict:
    ...
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

```bash
cat > /home/zhenya/kenny/visuotact/vt_franka/local_policies/my_policy.py <<'PY'
def rollout_policy(observation: dict) -> dict:
    state = observation["controller_state"]
    return {
        "target_tcp": state["tcp_pose"],
        "gripper_width": state["gripper_width"],
    }
PY
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

## 6. Integrate a Real Checkpoint Loader

If you want to roll out an actual RDP checkpoint, wrap your model-loading code behind a `module:function` entrypoint and make the function return the action dict expected above.

Typical pattern:

1. Load checkpoint once at module import time or on first call.
2. Convert `observation["controller_state"]` and your tactile inputs into your model input tensor format.
3. Run the forward pass.
4. Convert model output into:
   - `target_tcp`
   - `gripper_width` or `gripper_closed`

The new repo does not yet include the original training workspace or checkpoint format loader, so this wrapper is the integration point.

## 7. Stop and Postprocess Recorded Rollouts

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

## 8. Quick Failure Checks

### Controller state not coming back

```bash
curl http://<CONTROLLER_IP>:8092/api/v1/state
```

### Workspace cannot reach controller

```bash
curl http://<CONTROLLER_IP>:8092/api/v1/health
```

### Controller machine is not actually in RT mode

```bash
uname -a
cat /sys/kernel/realtime
groups | grep realtime
ulimit -r
ulimit -l
```

### Quest feedback not visible

Make sure `state-bridge` is running and Quest IP is correct in [workspace.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml).

### GelSight arrows not updating during rollout

Start `teleop` as well as `gelsight`, since the GelSight publisher uses:

```bash
curl http://<WORKSPACE_IP>:8082/get_current_gripper_state
```

for latency-matching state.

## 9. Notes

- The rollout runner is intentionally lightweight.
- The command path for execution is:
  - policy callable
  - `vt-franka-workspace rollout`
  - controller HTTP API
  - `vt-franka-controller`
  - Polymetis
  - Franka FCI
- If you need the exact original RDP model/checkpoint integration, that should be added as a separate model-loader module on top of this command path.
