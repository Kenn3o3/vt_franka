# Data Collection Pipeline 

### 3.1 Controller machine

The controller machine still runs the low-level robot stack as a long-lived service:

- Polymetis robot server
- Polymetis gripper server
- `vt-franka-controller run`

Run the following commands in three terminals:

C1:
```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

C2:
```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

C3:
```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
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

