# UAV Tag Tracking with ROS 2 and PX4

This ROS 2 package implements a UAV control system that uses AprilTag detections to generate offboard trajectory setpoints for PX4 drones. The node listens to AprilTag pose data, applies TF transforms, and sends real-time position commands to track the tag.

---

## 🚀 Overview

This project bridges AprilTag-based localization with PX4 offboard control using ROS 2. It enables a UAV to track AprilTags in real-time using transform lookups and publishes precise position setpoints.

### ✅ Key Features

* Subscribes to AprilTag detections (`/tag_detections` or `/tf`)
* Listens to PX4 odometry and vehicle control mode
* Broadcasts essential TF frames: `map`, `ned`, `base_link`, `tracking_camera`, `apriltag`
* Computes relative tag position with respect to the UAV base
* Publishes:

  * `OffboardControlMode` messages to `/fmu/in/offboard_control_mode`
  * `TrajectorySetpoint` messages to `/fmu/in/trajectory_setpoint`
* Locks Z-height when offboard is enabled
* Robust transform lookup with fallback handling

---

## 📦 Package Information

| Item                | Details               |
| ------------------- | --------------------- |
| **Package Name**    | `tagtransform`        |
| **Node Executable** | `tagtransform_node`   |
| **Language**        | C++                   |
| **ROS Version**     | ROS 2 (Foxy or later) |
| **License**         | MIT                   |

---

## 📂 File Structure

```
uav-tag-tracking-ros2/
├── src/
│   ├── tracking_subscriber.cpp       # Main implementation
│   └── tracking_subscriber.hpp       # Header definitions
├── package.xml                       # Dependency manifest
├── CMakeLists.txt                    # Build configuration
└── README.md                         # Project documentation
```

---

## 🔧 Installation

1. Clone this repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/SwarajMundruppadyRao/uav-tag-tracking-ros2.git
```

2. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select tagtransform
source install/setup.bash
```

---

## 🚀 Usage

Before running the node, make sure PX4 SITL or hardware is running and the relevant topics are being published.

### Run the Node:

```bash
ros2 run tagtransform tagtransform_node
```

### Ensure the following:

* AprilTag pose is published to `/tag_detections` (or `/tf` for simulation)
* PX4 publishes `/fmu/out/vehicle_odometry` and `/fmu/out/vehicle_control_mode`
* UAV is set to OFFBOARD mode via your ground control station

---

## 📋 Dependencies

The following ROS 2 packages are required (auto-resolved by `package.xml` and `CMakeLists.txt`):

* `rclcpp`
* `geometry_msgs`
* `px4_msgs`
* `tf2_ros`
* `ament_cmake`

Make sure you have PX4 ROS 2 messages (`px4_msgs`) installed and sourced.

---

## 🧐 Internals & Logic

### Transform Frames Used:

* `map → ned → base_link → tracking_camera → apriltag`
* AprilTag's relative position is computed using TF2 lookups
* When OFFBOARD mode is detected, the node latches `base_link.z` as the reference height and publishes trajectory setpoints accordingly

### Offboard Publishing Strategy:

* Publishes `OffboardControlMode` every other cycle
* Publishes `TrajectorySetpoint` after an initial buffer count (default: 30 cycles)

---

## 🧰 Example Output (Logs)

```text
Latched reference Z (NED) = -1.25 m
Publishing Trajectory Translation: x=0.53, y=0.08, z=-1.25
Published TrajectorySetpoint
Transform from base_link to apriltag:
Translation: x=0.53, y=0.08, z=0.00
```

---

## 👤 Author

**Swaraj Mundruppady Rao**
📧 [swarajmr@umd.edu](mailto:swarajmr@umd.edu)

---

## 📜 License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).

---

## 🌐 Repository

GitHub: [https://github.com/SwarajMundruppadyRao/uav-tag-tracking-ros2](https://github.com/SwarajMundruppadyRao/uav-tag-tracking-ros2)
