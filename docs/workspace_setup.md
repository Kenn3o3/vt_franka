# Workspace Setup

`robot_workspace` is the workstation-side package for teleop, camera capture, recording, and rollout.

## Expected machine role

- Ubuntu workstation on the same network as Quest
- Optional GelSight camera attached over USB
- Optional Orbbec RGB camera attached over USB
- Network access to the controller API running on the Franka-side machine

## Environment

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
conda env create -f environment.yml
conda activate vt-franka-workspace
```

If you use ROS2 Humble on this machine, source ROS before starting optional ROS-aware tools.

If the environment already exists and you want to refresh the editable installs:

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_workspace
conda activate vt-franka-workspace
pip install -r requirements.txt
```

If you will use an Orbbec camera on this machine, install the Orbbec Python SDK and udev rules in the same environment:

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/third_party
pip install pyorbbecsdk2-2.0.18-cp310-cp310-linux_x86_64.whl
```

This uses the vendored third-party SDK package only as an installer source. The `vt_franka` runtime still stays in the new repo.

## Config

Edit [workspace.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml):

- `controller.host`, `controller.port`: controller API endpoint
- `teleop.host`, `teleop.port`: Quest teleop HTTP bind address
- `quest_feedback.quest_ip`: Quest IP for UDP feedback
- `gelsight.camera_index`: GelSight V4L2 device index
- `orbbec.enabled`: enable the Orbbec RGB recorder
- `orbbec.serial_number`: optional fixed Orbbec serial number if multiple cameras are connected
- `orbbec.color_width`, `orbbec.color_height`, `orbbec.color_fps`: requested Orbbec color profile
- `recording.root_dir`: episode storage root
- `calibration.calibration_dir`: calibration JSON directory

## Run components

```bash
vt-franka-workspace teleop --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
vt-franka-workspace state-bridge --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
vt-franka-workspace gelsight --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
vt-franka-workspace orbbec --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

## Recording

```bash
vt-franka-workspace episode-start --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml --name peel_trial
vt-franka-workspace episode-stop --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
vt-franka-workspace postprocess --episode-dir /absolute/path/to/episode
```
