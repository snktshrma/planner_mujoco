# Setup

```bash
export ROBOT_MODEL_PATH=/path/to/franka_emika_panda/scene.xml
colcon build --packages-select robot_control
source install/setup.bash
```

# Usage

## Publisher

```bash
ros2 run robot_control publisher
```

Publishes joint states and camera streams. Press `1-6` to toggle camera streams.

## Demo

```bash
python3 robot_control/demo.py
```

## Keyboard Control

```bash
python3 robot_control/keyboard_control.py
```

Keys: W/S (Z), A/D (Y), Q/E (X), R/F (roll), T/G (pitch), Y/H (yaw), SPACE (reset), ESC (quit)
