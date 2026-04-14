# Data Collection Pipeline

This document is the operational runbook for collecting single-arm Franka demonstrations with `vt_franka`.

It assumes the following split:

- `robot_controller` runs on the Franka-side Ubuntu computer.
- `robot_workspace` runs on the workstation connected to Quest 3 and optional workspace-side sensors such as Orbbec RGB or GelSight.

The original `factoryil` tree is kept only as a reference. All commands below use `vt_franka`.

## 0. Machine Roles

### Controller Machine

- Direct Ethernet to the Franka controller
- Runs Polymetis + `vt-franka-controller`
- No Quest or camera dependencies required

### Workspace Machine

- Same Wi-Fi network as Quest 3
- Runs `vt-franka-workspace teleop`
- Runs `vt-franka-workspace state-bridge`
- Optionally runs `vt-franka-workspace orbbec`
- Optionally runs `vt-franka-workspace gelsight`
- Starts and stops recording episodes

## 1. Required Setup Documents

This runbook assumes both machines are already installed and configured.

Use these setup documents first:

- Controller machine setup:
  [controller_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/controller_setup.md)
- Workspace machine setup:
  [workspace_setup.md](/home/zhenya/kenny/visuotact/vt_franka/docs/workspace_setup.md)

This runbook does not repeat installation commands.

## 2. Current Network Example

If your current lab network is unchanged, use:

- Workspace PC: `192.168.217.109`
- Controller PC Wi-Fi: `192.168.217.180`
- Quest 3: `192.168.217.221`
- Franka robot controller: `172.16.0.2`

Important:

- TactAR should connect to the workspace PC IP, not the controller PC IP.
- The workspace talks to the controller API over Wi-Fi.
- The controller PC talks to the Franka over its dedicated wired interface.

## 3. Pre-Run Checks

### 3.1 Controller machine

Before launching anything:

- Power on the Franka
- Unlock brakes in Franka Desk
- Enable FCI in Franka Desk

Then verify:

```bash
ping -c 3 172.16.0.2
uname -a
cat /sys/kernel/realtime
groups | grep realtime
ulimit -r
ulimit -l
```

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/robot_controller
vt-franka-controller run --config /home/medair/vt_franka/robot_controller/config/controller.yaml
```

### 3.2 Workspace machine

Verify network reachability:

```bash
curl http://192.168.217.180:8092/api/v1/health
ping -c 3 192.168.217.221
```

If you use Orbbec, verify the SDK and the camera:

```bash
python -c "import pyorbbecsdk; print('pyorbbecsdk ok')"
cd /home/zhenya/kenny/visuotact/factoryil/src/third_party/pyorbbecsdk/examples
python enumerate.py
```

If you use GelSight, verify the V4L2 device:

```bash
v4l2-ctl --list-devices
ffplay /dev/video0
```

Optional Quest browser connectivity test:

```bash
python -m http.server 8000 --bind 0.0.0.0
```

Then open `http://192.168.217.109:8000` in the Quest browser.

## 4. Start the Controller Side

Open three terminals on the controller PC.

### Terminal C1: Polymetis robot server

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

What this launches:

- Polymetis Franka robot gRPC server on `127.0.0.1:50051`
- The low-level real-time robot client that talks to Franka FCI

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
- The 300 Hz bridge from workspace commands to Polymetis Cartesian impedance control

Controller health checks:

```bash
curl http://127.0.0.1:8092/api/v1/health
curl http://127.0.0.1:8092/api/v1/state
```

Optional safe start pose:

```bash
curl -X POST http://127.0.0.1:8092/api/v1/actions/home
```

## 5. Start the Workspace Side

Open four or five terminals on the workspace PC.

### Terminal W1: Start a new episode

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace episode-start \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --name demo_001
```

What this does:

- Creates the episode directory
- Writes `active_episode.json`
- Tells the running workspace services where to write stream files

### Terminal W2: Quest teleop server

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace teleop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- HTTP server on `0.0.0.0:8082`
- Quest message listener at `/unity`
- Teleop loop that converts Quest hand pose into robot TCP targets

### Terminal W3: Robot state bridge

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace state-bridge \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- Controller-state polling loop
- UDP publisher from workspace to Quest for robot pose and force feedback
- `controller_state.jsonl` recording when an episode is active

### Terminal W4: Optional Orbbec RGB recorder

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace orbbec \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- Orbbec RGB capture loop
- `orbbec_rgb.jsonl` metadata recording
- Recorded images under `streams/orbbec_rgb/` when saving is enabled

### Terminal W5: Optional GelSight publisher

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace gelsight \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

What this launches:

- GelSight capture and marker-tracking loop
- UDP tactile-arrow publisher to Quest
- GelSight stream recording

For your current initial test, run `W1`, `W2`, `W3`, and `W4`. Do not run `W5` yet if you are not using GelSight.

## 6. Open TactAR And Teleoperate

Open TactAR only after `W2` and `W3` are already running.

### 6.1 On Quest 3

- Open TactAR
- Enter the workspace PC IP, not the controller PC IP
- For your current setup, enter `192.168.217.109`
- If TactAR asks for a port, use `8082`
- Press `A + X` to calibrate
- Hold the grip button to enable tracking
- Use the trigger to close or open the gripper

What the controls do:

- `A + X`: calibrate the Quest frame to the robot/world frame
- Grip button: deadman switch; hold to stream motion commands, release to stop motion commands
- Trigger: close or open the gripper

How to move the robot:

- Hold the grip button
- Move the tracked controller in the air
- The workspace teleop server converts that hand motion into robot TCP targets
- The controller machine interpolates those targets and executes them at 300 Hz
- Release the grip button to stop sending motion targets

### 6.2 During collection

The active episode directory will receive:

- `streams/quest_messages.jsonl`
- `streams/teleop_commands.jsonl`
- `streams/controller_state.jsonl`
- `streams/orbbec_rgb.jsonl`
- `streams/orbbec_rgb/` if Orbbec frame saving is enabled
- `streams/gelsight_markers.jsonl`
- `streams/gelsight_frames/` if frame saving is enabled

## 7. Stop and Postprocess the Episode

### 7.1 Stop teleoperation and publishers

When you want to end the demo:

- Release the grip button
- Let the robot stop moving
- Open the gripper if desired
- Stop terminals `W2`, `W3`, `W4`, and `W5` with `Ctrl+C` if they are running

### 7.2 Close the active episode

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace episode-stop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

### 7.3 Postprocess into an aligned episode file

Replace `EPISODE_DIR` with the actual path printed by `episode-start`.

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace postprocess \
  --episode-dir EPISODE_DIR
```

This writes:

- `aligned_episode.npz`
- `aligned_episode_manifest.json`

## 8. Quick Failure Checks

### Controller API not reachable

```bash
curl http://<CONTROLLER_IP>:8092/api/v1/health
```

### Quest teleop server not reachable

```bash
curl http://<WORKSPACE_IP>:8082/get_current_gripper_state
```

### TactAR connected to the wrong machine

TactAR must use the workspace PC IP, not the controller PC IP.

### GelSight camera wrong index

```bash
v4l2-ctl --list-devices
ffplay /dev/video0
ffplay /dev/video1
```

### Orbbec SDK not installed

```bash
python -c "import pyorbbecsdk"
```

### Orbbec device not found

```bash
cd /home/zhenya/kenny/visuotact/factoryil/src/third_party/pyorbbecsdk/examples
python enumerate.py
```

### Polymetis robot server not up

```bash
nc -z 127.0.0.1 50051
nc -z 127.0.0.1 50052
```

## 9. Notes

- This v1 stack records raw per-stream data and aligns it offline later.
- The current `vt_franka` Orbbec path is RGB-only. It does not yet migrate the old depth or point-cloud path.
- If GelSight processing harms controller stability, move the GelSight publisher to a separate workspace-class machine and keep the controller machine isolated.
- The controller machine still depends on the Franka RT/libfranka/Polymetis stack. `vt_franka` does not replace that layer.
