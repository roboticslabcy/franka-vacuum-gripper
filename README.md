# franka_vacuum_gripper

ROS2 driver for the Franka cobot pump (vacuum gripper) end-effector, built on top of the libfranka `VacuumGripper` API.

## Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/franka_vacuum_gripper/state` | `franka_vacuum_gripper/msg/VacuumGripperState` | Gripper state at the configured rate |

#### `VacuumGripperState` fields

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `in_control_range` | `bool` | Vacuum is within the setpoint area |
| `part_present` | `bool` | Object detected (vacuum above threshold H2) |
| `part_detached` | `bool` | Object released after a suction cycle |
| `device_status` | `uint8` | `0`=GREEN, `1`=YELLOW, `2`=ORANGE, `3`=RED |
| `actual_power` | `uint16` | Current pump power [%] |
| `vacuum` | `uint16` | Current system vacuum [mbar] |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/franka_vacuum_gripper/vacuum` | `franka_vacuum_gripper/srv/Vacuum` | Activate vacuum (pick) |
| `/franka_vacuum_gripper/drop_off` | `franka_vacuum_gripper/srv/DropOff` | Release object (place) |
| `/franka_vacuum_gripper/stop` | `std_srvs/srv/Trigger` | Stop any running operation |

#### `Vacuum.srv`

```
uint8  vacuum      # Vacuum setpoint [mbar]
int32  timeout_ms  # Timeout [ms]
string profile     # Production setup profile: "P0" | "P1" | "P2" | "P3"
---
bool   success
string message
```

#### `DropOff.srv`

```
int32  timeout_ms  # Timeout [ms]
---
bool   success
string message
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_ip` | `string` | **required** | IP/hostname of the Franka robot |
| `state_publish_rate` | `double` | `30.0` | State publish rate [Hz] |

---

## Building

```bash
cd ~/robotics/workspaces/franka_planning_ws
colcon build --packages-select franka_vacuum_gripper
source install/setup.bash
```

---

## Running

### Launch file (recommended)

```bash
ros2 launch franka_vacuum_gripper vacuum_gripper.launch.py robot_ip:=<ROBOT_IP>
```

Optional argument:

```bash
ros2 launch franka_vacuum_gripper vacuum_gripper.launch.py \
  robot_ip:=192.168.1.1 \
  state_publish_rate:=50.0
```

### Direct node

```bash
ros2 run franka_vacuum_gripper franka_vacuum_gripper_node \
  --ros-args -p robot_ip:=192.168.1.1
```

---

## Usage examples

### Monitor state

```bash
ros2 topic echo /franka_vacuum_gripper/state
```

### Pick an object (activate vacuum)

```bash
ros2 service call /franka_vacuum_gripper/vacuum \
  franka_vacuum_gripper/srv/Vacuum \
  "{vacuum: 60, timeout_ms: 5000, profile: 'P0'}"
```

### Place an object (drop off)

```bash
ros2 service call /franka_vacuum_gripper/drop_off \
  franka_vacuum_gripper/srv/DropOff \
  "{timeout_ms: 3000}"
```

### Emergency stop

```bash
ros2 service call /franka_vacuum_gripper/stop std_srvs/srv/Trigger "{}"
```

---

## Demo script

A complete pick-and-place demo is included:

```bash
ros2 run franka_vacuum_gripper vacuum_gripper_demo.py
```

The demo:
1. Reads and prints the initial gripper state
2. Activates vacuum with a 60 mbar setpoint (5 s timeout)
3. Prints the updated state
4. Drops off the object (3 s timeout)
5. Calls stop to clean up

---

## Production setup profiles

The `profile` parameter maps to the `ProductionSetupProfile` enum in libfranka:

| Value | libfranka enum |
|-------|---------------|
| `"P0"` | `kP0` (default) |
| `"P1"` | `kP1` |
| `"P2"` | `kP2` |
| `"P3"` | `kP3` |

Consult the cobot-pump manual for the difference between profiles.

---

## Architecture

```
franka_vacuum_gripper_node
├── /franka_vacuum_gripper/state  [publisher, VacuumGripperState]
├── /franka_vacuum_gripper/vacuum  [service, Vacuum]
├── /franka_vacuum_gripper/drop_off  [service, DropOff]
└── /franka_vacuum_gripper/stop  [service, Trigger]
```

The node is implemented as a composable component (`rclcpp_components`) so it can be loaded into a component container alongside other Franka nodes.
