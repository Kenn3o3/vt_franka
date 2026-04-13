# Data Collection Pipeline

This document is the step-by-step command runbook for collecting single-arm Franka visuotactile demonstrations with `vt_franka`.

It assumes the following split:

- `robot_controller` runs on the new Franka-side Ubuntu computer.
- `robot_workspace` runs on the workstation connected to Quest 3 and the GelSight camera.

The original `factoryil` tree is kept only as a reference. All commands below use `vt_franka`.

## 0. Machine Roles

### Controller Machine

- Direct Ethernet to the Franka controller
- Runs Polymetis + `vt-franka-controller`
- No Quest or GelSight required

### Workspace Machine

- Same network as Quest 3
- Runs `vt-franka-workspace teleop`
- Runs `vt-franka-workspace state-bridge`
- Runs `vt-franka-workspace gelsight`
- Starts and stops recording episodes

## 1. One-Time Controller Machine Setup

### 1.1 Install the RT-compatible Franka host stack

You need a real-time capable kernel and a `libfranka`/Polymetis stack that matches the robot firmware.

Useful official references:

- Franka FCI overview: <https://support.franka.de/docs/overview.html>
- Franka `libfranka` docs: <https://support.franka.de/docs/libfranka.html>
- Franka FCI docs index: <https://support.franka.de/docs/index.html>

After installing the RT kernel, verify it:

```bash
uname -r
uname -r | grep -E 'rt|realtime'
```

### 1.2 Build Polymetis with Franka support

Replace `LIBFRANKA_VERSION` with the version compatible with your robot firmware.

```bash
cd /home/zhenya/kenny/visuotact
git clone git@github.com:facebookresearch/fairo.git
cd /home/zhenya/kenny/visuotact/fairo/polymetis

conda env create -f ./polymetis/environment.yml
conda activate polymetis-local
pip install -e ./polymetis

export LIBFRANKA_VERSION=<match-your-robot-firmware>
./scripts/build_libfranka.sh "$LIBFRANKA_VERSION"

rm -rf ./polymetis/build
mkdir -p ./polymetis/build
cd ./polymetis/build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_FRANKA=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"

make -j"$(nproc)"
cmake --install . --prefix "$CONDA_PREFIX"
```

### 1.3 Install `vt_franka` controller package

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
conda activate polymetis-local
pip install -e ../shared
pip install -e .
```

### 1.4 Configure the controller

Edit [controller.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml):

- `backend.robot_ip`
- `backend.robot_port`
- `backend.gripper_ip`
- `backend.gripper_port`
- `server.host`
- `server.port`

If Polymetis runs locally on the controller machine, keep:

```yaml
backend:
  robot_ip: 127.0.0.1
  robot_port: 50051
  gripper_ip: 127.0.0.1
  gripper_port: 50052
```

## 2. One-Time Workspace Machine Setup

### 2.1 Install workspace dependencies

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace

conda create -n vt-franka-workspace python=3.10 -y
conda activate vt-franka-workspace

pip install -e ../shared
pip install -e .
```

Install basic system camera tools:

```bash
sudo apt update
sudo apt install -y v4l-utils ffmpeg curl
```

### 2.2 Verify the GelSight camera

```bash
v4l2-ctl --list-devices
ffplay /dev/video0
```

### 2.3 Configure the workspace

Edit [workspace.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml):

- `controller.host`: controller machine IP
- `quest_feedback.quest_ip`: Quest 3 IP
- `gelsight.camera_index`: GelSight V4L2 index
- `recording.root_dir`: where episodes should be stored

## 3. Pre-Run Robot and Network Checks

### 3.1 On the controller machine

Power on the Franka, unlock brakes, and enable FCI in Franka Desk.

Check robot connectivity:

```bash
ping -c 3 172.16.0.2
```

### 3.2 On the workspace machine

Check Quest connectivity:

```bash
ping -c 3 <QUEST_IP>
```

Optional browser reachability test:

```bash
python3 -m http.server 8000 --bind 0.0.0.0
```

Then open `http://<WORKSPACE_IP>:8000` in the Quest browser.

## 4. Start the Controller Pipeline

Open three terminals on the controller machine.

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

### Terminal C3: `vt_franka` controller API

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

## 5. Start the Workspace Data Collection Pipeline

Open four terminals on the workspace machine.

### Terminal W1: Start a new episode

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace episode-start \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml \
  --name demo_001
```

This prints the episode directory and creates the active episode marker.

### Terminal W2: Quest teleop server

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace teleop \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

### Terminal W3: Robot state bridge

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace state-bridge \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

### Terminal W4: GelSight publisher

```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
vt-franka-workspace gelsight \
  --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

## 6. Collect the Demonstration

### 6.1 On Quest 3

- Open TactAR
- Enter the workspace machine IP
- Press `A + X` to calibrate
- Hold the grip button to enable tracking
- Use the trigger to close/open the gripper

### 6.2 During collection

The active episode directory will receive:

- `streams/quest_messages.jsonl`
- `streams/teleop_commands.jsonl`
- `streams/controller_state.jsonl`
- `streams/gelsight_markers.jsonl`
- `streams/gelsight_frames/` if frame saving is enabled

## 7. Stop and Postprocess the Episode

### 7.1 Stop teleoperation and publishers

Stop terminals `W2`, `W3`, and `W4` with `Ctrl+C`.

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

### GelSight camera wrong index

```bash
v4l2-ctl --list-devices
ffplay /dev/video0
ffplay /dev/video1
```

### Polymetis robot server not up

```bash
nc -z 127.0.0.1 50051
nc -z 127.0.0.1 50052
```

## 9. Notes

- This v1 stack records raw per-stream data and aligns it offline later.
- If GelSight processing harms controller stability, move the GelSight publisher to a separate workspace-class machine and keep the controller machine isolated.
- The controller machine still depends on the Franka RT/libfranka/Polymetis stack. `vt_franka` does not replace that layer.

