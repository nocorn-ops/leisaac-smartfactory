#!/bin/bash
# LeIsaac Docker 一键启动脚本

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker
docker run --gpus all -it --rm --network=host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $SCRIPT_DIR:/workspace/leisaac \
  nvcr.io/nvidia/isaac-sim:5.1.0 \
  bash -c "
    cd /workspace/leisaac
    pip install -q -e source/leisaac 2>/dev/null
    pip install -q -e dependencies/IsaacLab/source/isaaclab --no-deps 2>/dev/null
    echo '=== Launching LeIsaac ==='
    python scripts/environments/teleoperation/teleop_se3_agent.py --task LeIsaac-Lift-Cube-SO101-v0 --teleop_device keyboard
  "
