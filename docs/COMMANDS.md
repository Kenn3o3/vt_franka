### Controller PC:

#### Terminal C1: Polymetis robot server

```bash
conda activate polymetis-local
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

#### Terminal C2: Polymetis gripper server

```bash
conda activate polymetis-local
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

#### Terminal C3: Controller API

```bash
conda activate polymetis-local
vt-franka-controller run --config robot_controller/config/controller.yaml
```

### Workspace PC:
```bash
conda activate vt-franka-workspace
cd /home/zhenya/kenny/visuotact/vt_franka
```

用 USB 连接 Meta Quest 到电脑

```bash
adb devices
adb reverse tcp:8082 tcp:8082
adb shell settings put global stay_on_while_plugged_in 3
adb shell am broadcast -a com.oculus.vrpowermanager.prox_close --ei timeout 0
adb shell setprop debug.oculus.guardian_pause 1
```

In the Quest TactAR app, set the workstation IP to:

```text
127.0.0.1
```

用这个command采集数据：
```bash
vt-franka-workspace collect \
  --workspace-config robot_workspace/config/workspace.yaml \
  --task pencil_insertion_demo
```

之后对齐数据用这个command，生成不同模型可用的统一的数据format：

```bash
vt-franka-workspace make-dataset \
  robot_workspace/data/collect/pencil_insertion_demo \
  --name real_pencil_insertion_demo \
  --target-hz 10 \
  --overwrite
```

里面的action label需要是真的commanded action同步得出的而不是从observed next ee得出的。

之后把所有代码相关的文件rsync到remote PC 上面（/mnt/pfs_cuhk/kenny/vt_franka的目录，exclude所有大文件包括dataset）

```bash
rsync -avh --info=progress2 \
  -e "ssh -i ~/.ssh/zlkenny -p 538 -o IdentitiesOnly=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=6" \
  --exclude='.git/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  --exclude='robot_workspace/data' \
  /home/zhenya/kenny/visuotact/vt_franka/ \
  zlkenny@120.48.58.215:/mnt/pfs_cuhk/kenny/vt_franka/
```

之后把统一好的可训练数据e.g., real_pencil_insertion rsync到remote PC上面

```bash
rsync -avh --info=progress2 \
  -e "ssh -i ~/.ssh/zlkenny_yzy673_ed25519 -p 538 -o IdentitiesOnly=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=6" \
  /mnt/kenny_ssd/vt_franka/data/datasets/pencil_insertion_demo/real_pencil_insertion_demo/ \
  zlkenny@120.48.58.215:/mnt/pfs_cuhk/kenny/vt_franka/robot_workspace/data/datasets/pencil_insertion_demo/real_pencil_insertion_demo/
```

之后不同的模型可以在remote直接通过比如以下的命令进行训练：

```bash
tmux new -s pencil_insertion_demo_vista_so3

cd /mnt/pfs_cuhk/kenny/vt_franka
export PYTHONPATH=robot_workspace/src:shared/src:${PYTHONPATH:-}
conda activate isp

CUDA_VISIBLE_DEVICES=7 python -m vt_franka_workspace.cli train-visuotactile \
  --workspace-config robot_workspace/config/workspace.yaml \
  --task-name pencil_insertion_demo \
  --model vista_so3 \
  --dataset-name real_pencil_insertion_demo \
  --dataset-dir robot_workspace/data/datasets/pencil_insertion_demo/real_pencil_insertion_demo \
  --checkpoint-dir robot_workspace/data/checkpoints/pencil_insertion_demo/vista_so3 \
  --device cuda:0 \
  --extra-arg training.checkpoint_every=30
```

这里训练会自动生成模型specific的dataset cache，应该用的action也是commanded action而不是observed next ee。

训练结束后

用比如以下的指令下载需要模型的checkpoint到local pc：

```bash
mkdir -p /mnt/kenny_ssd/vt_franka/data/checkpoints/pencil_insertion_demo/vista_so3/checkpoints

rsync -avh --info=progress2 \
  -e "ssh -i ~/.ssh/zlkenny_yzy673_ed25519 -p 538 -o IdentitiesOnly=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=6" \
  --exclude='backend_dataset/' \
  --exclude='checkpoints/' \
  zlkenny@120.48.58.215:/mnt/pfs_cuhk/kenny/vt_franka/robot_workspace/data/checkpoints/pencil_insertion_demo/vista_so3/ \
  /mnt/kenny_ssd/vt_franka/data/checkpoints/pencil_insertion_demo/vista_so3/

rsync -avh --info=progress2 \
  -e "ssh -i ~/.ssh/zlkenny_yzy673_ed25519 -p 538 -o IdentitiesOnly=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=6" \
  zlkenny@120.48.58.215:/mnt/pfs_cuhk/kenny/vt_franka/robot_workspace/data/checkpoints/pencil_insertion_demo/vista_so3/checkpoints/epoch=209.ckpt \
  zlkenny@120.48.58.215:/mnt/pfs_cuhk/kenny/vt_franka/robot_workspace/data/checkpoints/pencil_insertion_demo/vista_so3/checkpoints/epoch=209.info.json \
  /mnt/kenny_ssd/vt_franka/data/checkpoints/pencil_insertion_demo/vista_so3/checkpoints/
```

然后直接在local跑推理：

```bash
source /home/zhenya/miniforge3/etc/profile.d/conda.sh
conda activate /mnt/kenny_ssd/conda_envs/isp_real
export PYTHONPATH=$PWD/robot_workspace/src:$PWD/shared/src:$PYTHONPATH

python -m vt_franka_workspace.cli run-policy \
  --workspace-config robot_workspace/config/workspace.yaml \
  --task pencil_insertion_demo \
  --inference-config robot_workspace/config/inference/pencil_insertion_visuotactile.yaml \
  --policy-config robot_workspace/config/policies/visuotactile_pencil_insertion_demo_vista_so3_epoch059_ddim16.yaml
```

```bash
python -m vt_franka_workspace.cli run-policy \
  --workspace-config robot_workspace/config/workspace.yaml \
  --task pencil_insertion_demo \
  --inference-config robot_workspace/config/inference/pencil_insertion_visuotactile.yaml \
  --policy-config robot_workspace/config/policies/visuotactile_pencil_insertion_demo_vista_so3_epoch209_ddim16.yaml
```
