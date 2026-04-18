## Demo Setup

This demo keeps the responsibility split clean:

- robot controller PC:
  - runs `launch_robot`
  - runs `launch_gripper`
  - runs `robot_controller/scripts/demo.py`
  - serves live robot state on HTTP for the workspace PC
- workspace PC:
  - runs one long-lived publisher command
  - reads robot state from the controller PC
  - reads GelSight directly from the workspace PC USB device
  - publishes robot EE pose, robot force arrow, and GelSight tactile arrows to Quest

## Prerequisites

- `robot_workspace/config/workspace.yaml` on the workspace PC must point `controller.host` to the robot controller PC IP.
- `robot_workspace/config/workspace.yaml` on the workspace PC must point `quest_feedback.quest_ip` to the Quest headset IP.
- For tactile arrows, GelSight must either be enabled in YAML or you must pass `--with-gelsight` to the demo publisher command.

## Robot Controller PC

From `/home/zhenya/kenny/visuotact/vt_franka`:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vt-franka-controller
```

Start the robot and gripper servers using your normal launch flow:

```bash
launch_robot
launch_gripper
```

Run the demo motion script plus read-only state server:

```bash
python robot_controller/scripts/demo.py
```

The script serves robot state on:

```text
http://0.0.0.0:8092/api/v1/state
```

## Workspace PC

From `/home/zhenya/kenny/visuotact/vt_franka`:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vt-franka-workspace
```

Run the demo publisher:

```bash
vt-franka-workspace demo-publish --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml --with-gelsight
```

This single command does all of the following:

- pulls robot state from the controller PC
- publishes robot EE pose to Quest
- publishes robot force arrow to Quest
- runs GelSight locally on the workspace PC
- publishes GelSight tactile arrows to Quest

If GelSight is already enabled in YAML, you can omit `--with-gelsight`:

```bash
vt-franka-workspace demo-publish --config /home/zhenya/kenny/visuotact/vt_franka/robot_workspace/config/workspace.yaml
```

## Quest

Open the Quest app after both machines are running.

Expected result:

- robot EE anchor follows the real robot through the workspace publisher
- robot force arrow is attached to the EE anchor
- GelSight tactile arrows are attached to the EE anchor and update during the whole demo
