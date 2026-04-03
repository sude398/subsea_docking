# Subsea Docking System: Usage Guide for `updated_b-2.py`

## Overview
`updated_b-2.py` is the ultimate iteration of the downward docking script, designed specifically for ROVs equipped with an independent pan/tilt camera servo mount. It inherits every stability update from `updated_b.py` (Anti-jitter queues, Timeout Failsafes, Dynamic PIDs) but intelligently integrates automatic **Active Gimbal Tracking** to follow the Docking marker via MAVROS RC Overrides.

## Core Features & Benefits
1. **Dynamic Camera Control (MAVROS `OverrideRCIn`)**:
   - The script controls the MAVROS `Channel 9` (`index 8`) directly. Instead of just hovering over a marker, the camera actively tracks it.
2. **Proportional Error Minimization**:
   - Whenever the `err_y` strays from center, the `cam_tilt_pwm` smoothly scales in response (via the `cam_kp` ratio). If the ArUco marker floats towards the top of the monitor, the servo tilts up to keep the docking target centered.
3. **Clipped Safe Angles (Chassis Evasion)**:
   - Configured with `cam_tilt_max` to `1800` rather than `1900` to guarantee the vehicle's skids/chassis frame does not physically obscure the camera feed. 
4. **Automated Search Panning**:
   - If the failsafe timeout breaks and the ROV enters `SEARCHING` mode, the camera gracefully retracts its angle and smoothly pans into its optimal searching vantage point (`cam_tilt_search`: 1750 PWM).

## When to Use This Script
- **Gimbal/Servo Equipped Hardware**: Crucial if you possess an active camera tilt system connected to MAVROS that you wish to utilize during docking.
- **Shallow / Tightly Coupled Operations**: In scenarios where the docking target drifts below or above the camera’s view angle very easily during descents, the dynamic camera tracking guarantees the system will not lose optical lock.
- **Note**: You **must** have MAVROS operational alongside the script (routing `/mavros/rc/override`) for the tracking components to communicate properly.

## Usage / Startup
1. Ensure MAVROS is initiated (`ros2 launch mavros apm.launch` etc).
2. Launch the node:
```bash
ros2 run <your_subsea_package> updated_b-2.py
```
3. **Calibrating Optimal Camera Physics**: If you still see the chassis block the screen when it looks down, adjust the `cam_tilt_max` limit while in the water:
```bash
ros2 param set /downward_docking_and_camera_node cam_tilt_max 1700
```
This restricts the dynamic algorithm from tilting too aggressively.
