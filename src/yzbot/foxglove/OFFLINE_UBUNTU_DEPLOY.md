# Offline Foxglove on Ubuntu

The competition runtime is self-contained on Ubuntu. Windows, Codex, a cloud
account, and Internet access are not part of the runtime path.

## Target

- Ubuntu 22.04 with a graphical session
- ROS 2 Humble
- Workspace layout: `<workspace>/src/yzbot`
- Firefox or Chromium installed on Ubuntu

## One-time installation

Run this while preparing the Ubuntu machine. Network access may be required
only for package installation.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosbridge-server \
  curl iproute2 tar coreutils x11-utils firefox
```

Build the workspace after copying the complete `yzbot` directory, including
`foxglove/offline_studio_1.87.0_web.tar.gz`:

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Offline launch

No network connection or login is required after installation.

```bash
cd <workspace>/src/yzbot
DISPLAY=:0 python3 -u start_competition.py --world world3 --no-rviz
```

The launcher starts all visualization components locally on Ubuntu:

- offline Foxglove Web UI: `http://127.0.0.1:8088`
- ROSBridge WebSocket: `ws://127.0.0.1:9090`
- `rosapi_node`
- Ubuntu Firefox or Chromium

`127.0.0.1` refers to the Ubuntu machine itself. It does not route through a
Windows host.

## Preflight

Use the same arguments planned for the real run and add `--dry-run`:

```bash
DISPLAY=:0 python3 -u start_competition.py \
  --world world3 --no-rviz --dry-run
```

The preflight fails if the offline archive, ROSBridge, rosapi, browser, map,
points file, or path matrix is missing.

Stop the launcher with `Ctrl+C`. It owns and stops the local HTTP server,
ROSBridge, rosapi, browser, Gazebo, Nav2, MoveIt, perception, and task nodes.
