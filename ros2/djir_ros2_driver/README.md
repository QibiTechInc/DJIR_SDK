# djir_ros2_driver

ROS 2 (Jazzy) driver for DJI RS 2 / DJI R SDK gimbals over Linux SocketCAN.

## Build

```bash
sudo ip link set can0 up type can bitrate 1000000
ln -s /path/to/DJIR_SDK/ros2/djir_ros2_driver ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select djir_ros2_driver
# If a Python venv without catkin_pkg is on PATH, add:
#   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

The package compiles the SDK sources from this repository (`../../src`).

## Run

```bash
ros2 launch djir_ros2_driver djir_driver.launch.py can_interface:=can0
```

## Interface

All angles are radians, joint order is yaw, roll, pitch.

| Name | Type | Description |
|---|---|---|
| `joint_states` | `sensor_msgs/JointState` (pub) | Joint (motor encoder) angles |
| `~/attitude` | `geometry_msgs/Vector3Stamped` (pub) | World attitude, x=roll y=pitch z=yaw |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` (pub) | `CAN link` and `Gimbal` status |
| `~/joint_command` | `sensor_msgs/JointState` (sub) | Target joint angles |
| `~/velocity_command` | `sensor_msgs/JointState` (sub) | Joint speeds (velocity field) |
| `~/recenter` | `std_srvs/Trigger` | Gimbal Recenter |
| `~/calibrate` | `std_srvs/SetBool` | `true` start / `false` stop auto calibration (gimbal moves) |
| `~/reset` | `std_srvs/Trigger` | Reopen CAN, clear counters and command cache |
| `~/set_absolute_mode` | `std_srvs/SetBool` | `true` absolute (default) / `false` incremental position mode |

### Position commands

Two modes, selected by the `command_mode` parameter (default `absolute`):

- **absolute**: `position` is the target joint angle. Only axes whose value
  changed since the last accepted command are sent; the others are flagged
  "invalid" in the protocol so the gimbal leaves them alone. Missing joints are
  also left alone. If nothing changed, nothing is sent.
- **incremental**: `position` is a relative move. Every message moves the
  gimbal; axes with 0 or missing are left alone.

Switch at runtime:

```bash
ros2 param set /djir_driver command_mode incremental
ros2 service call /djir_driver/set_absolute_mode std_srvs/srv/SetBool "{data: true}"
```

Other notes:

- Speed: put a positive value in `velocity[i]` (rad/s) and the move time is
  computed from the distance. Otherwise `max_velocity` (if > 0) or
  `default_move_time` is used. Move time is 0.1 s .. 25.5 s.
- With empty `name`, `position` is interpreted as `[yaw, roll, pitch]`.

```bash
ros2 topic pub --once /djir_driver/joint_command sensor_msgs/msg/JointState \
  "{name: [gimbal_yaw_joint], position: [0.5], velocity: [0.3]}"
```

### Velocity commands

Publish at >= 5 Hz. The driver re-sends the last command every 100 ms (the
gimbal stops a speed command after 0.5 s) and stops + releases speed control
`velocity_timeout` seconds after the last message.

```bash
ros2 topic pub -r 10 /djir_driver/velocity_command sensor_msgs/msg/JointState \
  "{name: [gimbal_yaw_joint], velocity: [0.2]}"
```

### Diagnostics

- **CAN link**: interface up, bus state (ERROR-ACTIVE/PASSIVE/BUS-OFF), packet
  counters, reply timeouts, CAN ACK errors (gimbal off / wiring).
- **Gimbal**: feedback age, last command result, auto calibration progress,
  firmware version, angle limits and motor stiffness (from parameter push).

Parameters are documented in `config/djir_driver.yaml`.
