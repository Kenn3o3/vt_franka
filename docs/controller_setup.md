# Controller Setup

`robot_controller` is the only package that should be installed on the Franka-side machine.

## Expected machine role

- Ubuntu machine directly connected to the Franka controller over Ethernet
- Polymetis robot server and gripper server available locally
- No Quest, GelSight, or ROS2 dependency required for the controller package itself

## Environment

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
conda env create -f environment.yml
conda activate vt-franka-controller
pip install -r requirements.txt
```

If you already maintain a Polymetis-specific Conda env, install `../shared` and `.` into that env instead.

## Config

Edit [controller.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml):

- `server.host`, `server.port`: controller API bind address
- `backend.robot_ip`, `backend.robot_port`: Polymetis robot endpoint
- `backend.gripper_ip`, `backend.gripper_port`: Polymetis gripper endpoint
- `control.cartesian_stiffness`, `control.cartesian_damping`: impedance settings

## Run

```bash
vt-franka-controller run --config /home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml
```

## Health check

```bash
curl http://<controller-host>:8092/api/v1/health
curl http://<controller-host>:8092/api/v1/state
```

