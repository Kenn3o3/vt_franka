# Controller Setup

`robot_controller` is the only package that should be installed on the Franka-side machine.

## Expected machine role

- Ubuntu machine directly connected to the Franka controller over Ethernet
- Polymetis robot server and gripper server available locally

> Note: No Quest, GelSight, or ROS2 dependency required for the controller package itself

## Machine prerequisites

Before installing `robot_controller`, make sure the new Ubuntu machine is actually ready to host the Franka real-time stack.

### 1. Match `libfranka` to the robot firmware

Your `libfranka` version must match the Franka Robot System Version shown in Franka Desk.

As a rule of thumb:

- robot system `>= 5.9.0` needs `libfranka >= 0.18.0`
- robot system `>= 5.7.2` and `< 5.9.0` usually needs `libfranka >= 0.15.0` and `< 0.18.0`

### 2. Verify the direct Ethernet link to the robot

The controller machine should use a dedicated wired interface to the robot. Do not run the real-time control path over Wi-Fi or VPN.

Check interface state and routing:

```bash
ip -br addr
ip route
sudo ethtool eno2 | egrep "Speed|Duplex|Link detected"
```

Expected:

- `Speed: 1000Mb/s`
- `Duplex: Full`
- `Link detected: yes`

If the robot is on `172.16.0.2/24`, a typical host-side address is `172.16.0.3/24`. Then verify:

```bash
ping -c 5 172.16.0.2
```

### 3. Install and verify the RT kernel and realtime privileges

For the Polymetis-on-Franka + Ubuntu 20.04 setup, we need the `PREEMPT_RT` kernel.

The procedure below adds an RT kernel alongside the current generic kernel. If this machine already boots an RT kernel, skip to step `3.8`.

#### 3.1 Record the current machine state

```bash
lsb_release -a
uname -a
ip -br addr
ip route
dpkg -l | grep -E 'linux-(image|headers)' | grep -E 'generic|rt' || true
```

Output:

```
(base) medair@medair:~/vt_franka$ lsb_release -a
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 20.04.6 LTS
Release:        20.04
Codename:       focal
(base) medair@medair:~/vt_franka$ uname -a
Linux medair 5.15.0-139-generic #149~20.04.1-Ubuntu SMP Wed Apr 16 08:29:56 UTC 2025 x86_64 x86_64 x86_64 GNU/Linux
(base) medair@medair:~/vt_franka$ ip -br addr
lo               UNKNOWN        127.0.0.1/8 ::1/128 
eno2             DOWN           
wlx90de80def2bb  UP             10.13.179.165/16 fe80::3f49:a56:d2e2:f562/64 
tailscale0       UNKNOWN        100.107.196.33/32 fd7a:115c:a1e0::fa39:c422/128 fe80::b319:a213:1846:3afd/64 
(base) medair@medair:~/vt_franka$ ip route
default via 10.13.63.254 dev wlx90de80def2bb proto dhcp metric 600 
10.13.0.0/16 dev wlx90de80def2bb proto kernel scope link src 10.13.179.165 metric 600 
169.254.0.0/16 dev wlx90de80def2bb scope link metric 1000 
(base) medair@medair:~/vt_franka$ dpkg -l | grep -E 'linux-(image|headers)' | grep -E 'generic|rt' || true
ii  linux-headers-5.15.0-139-generic            5.15.0-139.149~20.04.1               amd64        Linux kernel headers for version 5.15.0 on 64 bit x86 SMP
ii  linux-headers-5.9.1-rt20                    5.9.1-rt20-2                         amd64        Linux kernel headers for 5.9.1-rt20 on amd64
ii  linux-headers-generic-hwe-20.04             5.15.0.139.149~20.04.1               amd64        Generic Linux kernel headers
ii  linux-image-5.15.0-139-generic              5.15.0-139.149~20.04.1               amd64        Signed kernel image generic
ii  linux-image-5.9.1-rt20                      5.9.1-rt20-2                         amd64        Linux kernel, version 5.9.1-rt20
ii  linux-image-generic-hwe-20.04               5.15.0.139.149~20.04.1               amd64        Generic Linux kernel image
```

#### 3.2 Check Secure Boot before building the RT kernel

An unsigned manually built kernel may not boot if Secure Boot is enabled.

```bash
sudo apt update
sudo apt install -y mokutil
mokutil --sb-state
```

If Secure Boot is enabled, disable it in the BIOS or UEFI settings before trying to boot the RT kernel.

#### 3.3 Install kernel build dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential bc curl ca-certificates gnupg2 \
  libssl-dev lsb-release libelf-dev bison flex \
  fakeroot dpkg-dev cpio kmod rsync libncurses-dev \
  xz-utils dwarves
```

#### 3.4 Download the known-good Ubuntu 20.04 RT kernel sources

These version numbers match the Polymetis Franka hardware prerequisites.

```bash
mkdir -p ~/rt_kernel_build
cd ~/rt_kernel_build

curl -SLO https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/linux-5.11.tar.xz
curl -SLO https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.11/older/patch-5.11-rt7.patch.xz

xz -dk linux-5.11.tar.xz
xz -dk patch-5.11-rt7.patch.xz
```

#### 3.5 Extract the kernel, apply the RT patch, and seed the config

```bash
cd ~/rt_kernel_build
rm -rf linux-5.11
tar xf linux-5.11.tar
cd linux-5.11

patch -p1 < ../patch-5.11-rt7.patch
cp -v /boot/config-$(uname -r) .config
```

#### 3.6 Configure the kernel for RT

First apply the known required config adjustments:

```bash
cd ~/rt_kernel_build/linux-5.11

scripts/config --set-str SYSTEM_TRUSTED_KEYS ""
scripts/config --set-str MODULE_SIG_KEY "certs/signing_key.pem"
scripts/config --set-str SYSTEM_REVOCATION_KEYS "" || true
scripts/config --disable DEBUG_INFO || true
scripts/config --disable DEBUG_INFO_BTF || true
```

Then run:

```bash
make oldconfig
```

When prompted for the preemption model, choose:

- `Fully Preemptible Kernel (Real-Time)`

For the remaining prompts, keep the defaults by pressing `Enter`.

Verify the key config values:

```bash
grep -E 'CONFIG_(PREEMPT.*RT|SYSTEM_TRUSTED_KEYS|SYSTEM_REVOCATION_KEYS|MODULE_SIG_KEY)' .config
```

You want to see:

- `CONFIG_PREEMPT_RT=y` or the RT preemption option enabled
- `CONFIG_SYSTEM_TRUSTED_KEYS=""`
- `CONFIG_MODULE_SIG_KEY="certs/signing_key.pem"`

Output:

```
(base) medair@medair:~/rt_kernel_build/linux-5.11$ grep -E 'CONFIG_(PREEMPT.*RT|SYSTEM_TRUSTED_KEYS|SYSTEM_REVOCATION_KEYS|MODULE_SIG_KEY)' .config
CONFIG_PREEMPT_RT=y
CONFIG_MODULE_SIG_KEY="certs/signing_key.pem"
CONFIG_SYSTEM_TRUSTED_KEYS=""
```

#### 3.7 Build and install the RT kernel packages

Use `bindeb-pkg` here so the build does not require a git repository.

```bash
cd ~/rt_kernel_build/linux-5.11/tools/objtool

mv elf.h objtool_elf.h

sed -i 's/"elf.h"/"objtool_elf.h"/' \
  objtool.h elf.c special.h warn.h

sed -i 's#../../elf.h#../../objtool_elf.h#' \
  arch/x86/decode.c
```

This avoids the header-name collision and lets `make tools/objtool` proceed on the tested Ubuntu 20.04 host environment. After applying the rename, rerun:

```bash
cd ~/rt_kernel_build/linux-5.11
make -j"$(nproc)" bindeb-pkg
```

After the build finishes:

```bash
cd ~/rt_kernel_build
ls -1 *.deb | grep -E 'linux-(image|headers).*5\.11.*rt7'
sudo dpkg -i linux-headers-5.11.0-rt7_*.deb linux-image-5.11.0-rt7_*.deb
sudo update-grub
```

Reboot:

```bash
sudo reboot
```

If the machine does not automatically boot the RT kernel, open GRUB and select the `5.11.0-rt7` kernel entry manually.

#### 3.8 Verify that the machine is actually running the RT kernel

```bash
uname -a
cat /sys/kernel/realtime
dpkg -l | grep -E 'linux-(image|headers)' | grep -E 'generic|rt7'
```

You want:

- `uname -a` to contain `PREEMPT_RT`
- `/sys/kernel/realtime` to print `1`
- both the RT kernel and the previous generic kernel packages to remain installed

Output:

```
(base) medair@medair:~/vt_franka$ uname -a
Linux medair 5.11.0-rt7 #1 SMP PREEMPT_RT Mon Apr 13 17:21:27 CST 2026 x86_64 x86_64 x86_64 GNU/Linux
(base) medair@medair:~/vt_franka$ cat /sys/kernel/realtime
1
(base) medair@medair:~/vt_franka$ dpkg -l | grep -E 'linux-(image|headers)' | grep -E 'generic|rt7'
ii  linux-headers-5.11.0-rt7                    5.11.0-rt7-1                         amd64        Linux kernel headers for 5.11.0-rt7 on amd64
ii  linux-headers-5.15.0-139-generic            5.15.0-139.149~20.04.1               amd64        Linux kernel headers for version 5.15.0 on 64 bit x86 SMP
ii  linux-headers-generic-hwe-20.04             5.15.0.139.149~20.04.1               amd64        Generic Linux kernel headers
ii  linux-image-5.11.0-rt7                      5.11.0-rt7-1                         amd64        Linux kernel, version 5.11.0-rt7
ii  linux-image-5.15.0-139-generic              5.15.0-139.149~20.04.1               amd64        Signed kernel image generic
ii  linux-image-generic-hwe-20.04               5.15.0.139.149~20.04.1               amd64        Generic Linux kernel image
```

If `/sys/kernel/realtime` is missing, you are still booted into the generic kernel.

#### 3.9 Grant realtime privileges to the user that will launch Polymetis

```bash
sudo addgroup realtime
sudo usermod -a -G realtime "$(whoami)"
```

Use a dedicated limits file instead of editing `/etc/security/limits.conf` directly:

```bash
sudo tee /etc/security/limits.d/franka-realtime.conf >/dev/null <<'EOF'
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF
```

Reboot or fully log out and log back in:

```bash
sudo reboot
```

#### 3.10 Verify realtime privileges after logging back in

```bash
groups | grep realtime
ulimit -r
ulimit -l
```

Expected:

- `groups` contains `realtime`
- `ulimit -r` is `99`
- `ulimit -l` is `102400`

If `ulimit -r` is still `0`, you are likely in a session that was started before the group and limits changes took effect.

### 4. Install Polymetis with Franka support

This new repo expects the controller machine to use Polymetis as the robot-side backend. Build Polymetis with the `LIBFRANKA_VERSION` that matches your robot firmware.

We need `libfranka >= 0.18.0`

```bash
cd ~/vt_franka
git clone https://github.com/facebookresearch/fairo.git
cd ~/vt_franka/fairo/polymetis

conda env create -f ./polymetis/environment.yml
conda activate polymetis-local
pip install -e ./polymetis
```

```bash
LIBFRANKA_SRC=/home/medair/vt_franka/fairo/polymetis/polymetis/src/clients/franka_panda_client/third_party/libfranka
LIBFRANKA_BUILD=/home/medair/vt_franka/fairo/polymetis/libfranka-openrobots-build

git -C "$LIBFRANKA_SRC" checkout 0.18.0
git -C "$LIBFRANKA_SRC" submodule update --init --recursive

rm -rf "$LIBFRANKA_BUILD"
mkdir -p "$LIBFRANKA_BUILD"

env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/noetic/bin \
  LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu \
  bash -lc "
    cd '$LIBFRANKA_BUILD' &&
    cmake '$LIBFRANKA_SRC' \
      -Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio \
      -DCMAKE_PREFIX_PATH=/opt/openrobots \
      -Dfmt_DIR=/usr/lib/x86_64-linux-gnu/cmake/fmt \
      -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF \
      -DBUILD_EXAMPLES=OFF &&
    cmake --build . -j\$(nproc)
  "
```

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis/polymetis

rm -rf build
mkdir build
cd build

cmake .. \
  -DFranka_DIR=/home/medair/vt_franka/fairo/polymetis/libfranka-openrobots-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_FRANKA=ON \
  -DBUILD_TESTS=OFF \
  -DBUILD_DOCS=OFF \
  -DBUILD_ALLEGRO=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"

make -j"$(nproc)"

```

Keep this set when launching Franka-side binaries:

```bash
export LD_LIBRARY_PATH=/home/medair/vt_franka/fairo/polymetis/libfranka-openrobots-build:/opt/openrobots/lib:$LD_LIBRARY_PATH
```

## Environment

Install a few basic host tools first if they are missing:

```bash
sudo apt update
sudo apt install -y ethtool curl
```

Install `robot_controller` into the same Conda environment that already contains your working Polymetis build.

```bash
cd /home/medair/vt_franka/robot_controller
conda activate polymetis-local
pip install -e ../shared
pip install -e .
```

### Optional standalone dev or mock-only env

Use this only if you want a separate environment for non-hardware development.

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
conda env create -f environment.yml
conda activate vt-franka-controller
pip install -r requirements.txt
```

## Config

Edit [controller.yaml](/home/medair/vt_franka/robot_controller/config/controller.yaml):

- `server.host`, `server.port`: controller API bind address
- `backend.robot_ip`, `backend.robot_port`: Polymetis robot endpoint
- `backend.gripper_ip`, `backend.gripper_port`: Polymetis gripper endpoint
- `control.cartesian_stiffness`, `control.cartesian_damping`: impedance settings

## Run

Start the Polymetis servers first:

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

### Minimal Polymetis motion test

For a minimal robot-side test of Polymetis connectivity and motion, you do not need to start `vt-franka-controller run`. The script [polymetis_minimal_motion.py](/home/medair/vt_franka/robot_controller/scripts/polymetis_minimal_motion.py) connects directly to the local Polymetis robot server at `127.0.0.1:50051`, prints the current end-effector pose and joints, moves to the in-script target pose, prints state again, moves to the in-script target joints, then prints final state.

Edit the target commands directly in [polymetis_minimal_motion.py](/home/medair/vt_franka/robot_controller/scripts/polymetis_minimal_motion.py):

- `TARGET_POSITION_M`
- `TARGET_RPY_DEG`
- `TARGET_JOINTS_RAD`
- `POSE_MOVE_TIME_S`
- `JOINT_MOVE_TIME_S`
- `ASK_FOR_CONFIRMATION`

Launch sequence on the robot controller machine:

Terminal 1:

```bash
conda activate polymetis-local
cd /home/medair/vt_franka/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

Terminal 2:

```bash
conda activate polymetis-local
cd /home/medair/vt_franka
python /home/medair/vt_franka/robot_controller/scripts/polymetis_minimal_motion.py
```

Optional connectivity check before running the script:

```bash
conda activate polymetis-local
nc -z 127.0.0.1 50051 && echo OK
```

`launch_gripper.py` is not required for this minimal arm motion test.

Then start `robot_controller`:

```bash
vt-franka-controller run --config /home/medair/vt_franka/robot_controller/config/controller.yaml
```

## Health check

```bash
curl http://<controller-host>:8092/api/v1/health
curl http://<controller-host>:8092/api/v1/state
```

Recommended preflight checks before running the controller:

```bash
ping -c 3 172.16.0.2
uname -a
cat /sys/kernel/realtime
groups | grep realtime
ulimit -r
ulimit -l
```
