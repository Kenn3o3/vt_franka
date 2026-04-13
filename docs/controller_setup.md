# Controller Setup

`robot_controller` is the only package that should be installed on the Franka-side machine.

## Expected machine role

- Ubuntu machine directly connected to the Franka controller over Ethernet
- Polymetis robot server and gripper server available locally
- No Quest, GelSight, or ROS2 dependency required for the controller package itself

## Fresh machine prerequisites

Before installing `robot_controller`, make sure the new Ubuntu machine is actually ready to host the Franka real-time stack.

### 1. Match `libfranka` to the robot firmware

Your `libfranka` version must match the Franka Robot System Version shown in Franka Desk.

As a rule of thumb:

- robot system `>= 5.9.0` needs `libfranka >= 0.18.0`
- robot system `>= 5.7.2` and `< 5.9.0` usually needs `libfranka >= 0.15.0` and `< 0.18.0`

Do not guess this step. Confirm the robot version first, then choose the matching `LIBFRANKA_VERSION` for the Polymetis build.

### 2. Verify the direct Ethernet link to the robot

The controller machine should use a dedicated wired interface to the robot. Do not run the real-time control path over Wi-Fi or VPN.

Check interface state and routing:

```bash
ip -br addr
ip route
sudo ethtool <ROBOT_NIC> | egrep "Speed|Duplex|Link detected"
```

You want to see:

- `Speed: 1000Mb/s`
- `Duplex: Full`
- `Link detected: yes`

If the robot is on `172.16.0.2/24`, a typical host-side address is `172.16.0.3/24`. Then verify:

```bash
ping -c 5 172.16.0.2
```

### 3. Install and verify the RT kernel and realtime privileges

For the Polymetis-on-Franka setup, Ubuntu 20.04 is still acceptable. The missing requirement on a fresh machine is the `PREEMPT_RT` kernel, not a different Ubuntu release.

The procedure below adds an RT kernel alongside the current generic kernel. It should not remove your existing Conda environments, repositories, or normal user-space packages. Keep the generic kernel installed as a fallback boot option.

If this machine already boots an RT kernel, skip to step `3.8`.

#### 3.1 Record the current machine state

```bash
lsb_release -a
uname -a
ip -br addr
ip route
dpkg -l | grep -E 'linux-(image|headers)' | grep -E 'generic|rt' || true
```

Optional but recommended backup of package and environment metadata:

```bash
mkdir -p ~/controller_host_backup_$(date +%F)
cd ~/controller_host_backup_$(date +%F)
conda env list > conda_env_list.txt
apt-mark showmanual > apt_manual.txt
dpkg --get-selections > dpkg_selections.txt
dkms status > dkms_status.txt || true
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

#### 3.7 Build and install the RT kernel packages

Use `bindeb-pkg` here so the build does not require a git repository.

```bash
cd ~/rt_kernel_build/linux-5.11
make -j"$(nproc)" bindeb-pkg
```

This can take a long time.

After the build finishes:

```bash
cd ~/rt_kernel_build
ls -1 *.deb | grep -E 'linux-(image|headers).*5\.11.*rt7'
sudo dpkg -i linux-headers-5.11.0-rt7_*.deb linux-image-5.11.0-rt7_*.deb
sudo update-grub
```

Do not purge the current generic kernel packages. Keep them installed as a fallback.

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

## Environment

Install a few basic host tools first if they are missing:

```bash
sudo apt update
sudo apt install -y ethtool curl
```

### Recommended for real hardware

Install `robot_controller` into the same Conda environment that already contains your working Polymetis build.

```bash
cd /home/zhenya/kenny/visuotact/vt_franka/robot_controller
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

Edit [controller.yaml](/home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml):

- `server.host`, `server.port`: controller API bind address
- `backend.robot_ip`, `backend.robot_port`: Polymetis robot endpoint
- `backend.gripper_ip`, `backend.gripper_port`: Polymetis gripper endpoint
- `control.cartesian_stiffness`, `control.cartesian_damping`: impedance settings

## Run

Start the Polymetis servers first:

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

```bash
conda activate polymetis-local
cd /home/zhenya/kenny/visuotact/fairo/polymetis
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

Then start `robot_controller`:

```bash
vt-franka-controller run --config /home/zhenya/kenny/visuotact/vt_franka/robot_controller/config/controller.yaml
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
