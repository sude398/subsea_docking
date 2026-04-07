# Subsea Docking System: Usage Guide for `updated_b.py`

## Overview
`updated_b.py` acts as the primary stabilized downward docking controller for the ArduSub ROV using static camera alignments. It maintains all core vision processing from the initial prototype codebase but introduces professional fallback behaviors, anti-jitter queues, and live system tuning.

## Core Features & Benefits
1. **Signal Filtering (Anti-Jitter)**: 
   - A `MovingAverageFilter` eliminates violent motor thrashing. Underwater optic ripples can cause ArUco tags to mathematically jump several pixels instantly, but the 5-frame moving average smoothly irons this data out before passing it into the PID logic.
2. **"Target Lost" Timeout (Failsafe)**:
   - When descending, light or physical barriers can temporarily obscure a marker. Instead of panicking directly into ascending (`SEARCHING`) mode, the ROV issues a 1.5-second `HOLD POSITION` timeout. If the marker re-appears immediately, the downward trajectory seamlessly resumes.
3. **Safe State Transitions**:
   - Built around the internal `ROVState` Enum, preventing accidental logical overwrites from arbitrary strings. 
4. **Dynamic Parameter Access**:
   - `kp`, `ki`, `kd`, and distance parameters can be tuned dynamically via standard ROS `ros2 param set /downward_docking_node <param> <value>` poolside.

## When to Use This Script
- **Fixed-Camera Missions:** Recommended if your camera mount does **not** have an independent servo motor, or if it is hardware-locked to exactly 90-degrees downward.
- **Turbid/High-Noise Environments**: Provides immense stability over standard logic during heavy ROV shaking due to the signal filtration.
- **Hardware Limitations**: Only uses pure `/dev/ttyACM0` PyMAVLink control for inputs; it does NOT communicate over `mavros` for joystick/camera overrides.

## Usage / Startup
1. Ensure the camera is pointed downward as best as possible.
2. Launch the node:
```bash
ros2 run <your_subsea_package> updated_b.py
```
3. Poolside Tuning: Watch the live `rqt_image_view` output and modify PIDs live if the vehicle's sway is too slow:
```bash
ros2 param set /downward_docking_node kp 0.002
```
