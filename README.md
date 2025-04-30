# Autotank

Autotank is a C++ project demonstrating the development of an autonomous mobile robot platform. It leverages the **`protocore`** framework for robust task management, state coordination, and message-based communication. The project emphasizes a clear separation between application logic and the underlying hardware or simulation environment (Gazebo), facilitating development and testing.

---

## Features

- **`protocore` Integration**: Core functionalities like task lifecycle management, state synchronization (`StateManager`), and asynchronous message passing (`Broker`, `MessageQueue`) are provided by the external `protocore` framework.
- **Hardware/Simulation Abstraction (HAL)**: A well-defined Hardware Abstraction Layer allows the application to run seamlessly in Gazebo simulation (`USE_SIM=ON`) or on target hardware (`USE_SIM=OFF`) with minimal code changes.
- **Modular CSCs (Computer Software Components)**:
    - **Sensor Tasks**: Dedicated tasks for processing data from various sensors:
        - `IMUTask`: Handles Inertial Measurement Unit data.
        - `OdomTask`: Processes wheel odometry data.
        - `LidarTask`: Manages 2D Lidar scans.
    - **`LocalizationTask`**: Fuses sensor data (IMU, Odometry) and command velocity inputs to estimate the robot's pose (position and orientation) and twist (linear and angular velocity). Publishes `LocalizationEstimateMsg`. (Utilizes `kfplusplus` for its Extended Kalman Filter implementation).
    - **`MotionControlTask`**: Receives localization estimates and target waypoints, calculates safe and appropriate velocity commands (`CmdVelMsg`), and sends them to the differential drive interface.
    - **`NavigationTask`**: Responsible for path planning, obstacle avoidance, and waypoint following based on the localization estimate and map data.
    - **`MappingTask`**: Builds and maintain a representation of the environment, with online mapping with a occupancy grid and Bresenham line algorithm for free spaces
    - **`SafetyMonitorTask`**: checks LiDAR and velocity for iminent collisions and will safety alerts to the `MotionControlTask`
    - **`MapService`**: Provides a global, heap-free mapping and planning utility shared across tasks. It maintains a fixed-size occupancy grid updated via `update_map()` using Bresenham’s line algorithm. Path planning is performed with `plan_path()` (A* search), and `find_frontiers()` identifies candidate exploration targets. Internally, it uses a **sequence lock** to allow one writer (e.g., `MappingTask`) and multiple lock-free readers (e.g., `NavigationTask`) to safely access the map concurrently with real-time guarantees.
- **Message Definitions**: Custom message types (`msg/*.h`) define the data structures for inter-task communication (e.g., `IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`, `CmdVelMsg`, `LocalizationEstimateMsg`).
- **Extensible Design**: Easily add new sensor tasks, control algorithms, or higher-level capabilities like navigation and mapping.

---

## File Structure

```plaintext
src/
├── csc/
│   ├── control/motion_control/          # MotionControlTask, DiffDrive
│   ├── localization/                    # EKF-based state estimation
│   ├── mapping/                         # Lidar-based mapping
│   ├── navigation/                      # Behavior planner + waypoint generator
│   ├── safety_monitor/                  # Emergency stop logic
│   └── services/map_service/            # A*, occupancy grid
│
│   └── sensors/
│       ├── imu/imu_task.cpp             # IMU interface
│       ├── odom/odom_task.cpp           # Odometry interface
│       └── lidar/lidar_task.cpp         # 2D LiDAR interface
│
├── hal/
│   └── gazebo/                          # Simulation drivers (imu, odom, lidar, drive)
└── main.cpp                             # Entry point and task setup
---
```

## Architecture Overview

Autotank utilizes the task-based architecture provided by `protocore`. Each significant function (sensor handling, localization, control) is encapsulated within a `Task`.

- **Communication**: Tasks communicate asynchronously by publishing and subscribing to specific message types via the `protocore` `Broker`. Message queuing and thread management are handled by the `protocore::Task` base class and `MessageQueue`.
- **State Management**: The `protocore::StateManager` orchestrates the lifecycle of all registered tasks (e.g., `INITIALIZING`, `RUNNING`, `STOPPED`), ensuring synchronized state transitions across the system.
- **Data Flow**:
    1. **Sensor Tasks** (`IMUTask`, `OdomTask`, `LidarTask`) retrieve raw sensor data via the HAL (either Gazebo simulation or hardware). Each task processes its input and publishes a standardized message:
    - `IMUDataMsg` from `IMUTask`
    - `OdomDataMsg` from `OdomTask`
    - `LidarDataMsg` from `LidarTask`

    2. **`LocalizationTask`** subscribes to all relevant sensor messages and `CmdVelMsg` to support motion prediction. It runs an Extended Kalman Filter (EKF) using `kfplusplus`, producing a fused estimate of the robot’s state. This estimate is published as a `LocalizationEstimateMsg`.

    3. **`MappingTask`** listens for `LidarDataMsg` and `LocalizationEstimateMsg`. Once a valid pose estimate is available, it **directly calls** the global `MapService` to integrate the LiDAR scan. The `MapService`:
        - Performs ray-tracing from the robot's pose to mark free and occupied cells in a fixed-size occupancy grid.
        - Uses a **sequence lock (seqlock)** to enable safe concurrent access: one writer (`MappingTask`) and multiple lock-free readers (e.g., `NavigationTask`).
        - Operates entirely without heap allocations to meet real-time constraints.

    4. **`NavigationTask`** consumes the localization estimate and interfaces with the `MapService` to:
    - Evaluate current behavior mode (e.g., `GO_HOME`, `EXPLORE`).
    - Plan a path using A* via `MapService::plan_path()`.
    - In `EXPLORE` mode, call `MapService::find_frontiers()` to detect free-unknown boundaries and choose reachable goals.
    - Generate a waypoint path (`MapService::Path`) and publish waypoints as `WaypointMsg`.

    5. **`MotionControlTask`** subscribes to `LocalizationEstimateMsg` and `WaypointMsg`. It calculates a safe velocity command using a simple feedback controller:
    - Rotates in place to align with the target.
    - Drives forward with yaw correction.
    - Publishes the resulting `CmdVelMsg`.
    It also listens for `SafetyAlertMsg` and overrides movement commands with zero velocity if a collision is imminent.

    6. The **`DiffDrive` HAL** (e.g., `gazebo_diff_drive`) receives `CmdVelMsg` and translates the motion command into simulator-specific or hardware-specific actuation.

```
+----------------+        +------------------+       +-------------------+
|  Sensor Tasks  | -----> |  Localization    | ----> |  Motion Control   |
| (IMU, Odom,    |        |  (EKF Fusion)    |       |  (DiffDrive Cmds) |
|  Lidar)        |        +------------------+       +-------------------+
|                |                      |                      ^
|                |                      v                      |
|                |            +------------------+             |
|                +----------> |    Mapping Task  | <--------- -+
|                             | (Occupancy Grid) |             |
|                             +------------------+             |
|                                     ^                        |
|                                     |                        |
|                          +---------------------+             |
|                          |   Navigation Task   | ------------+
|                          |  (Plan + Behavior)  |
|                          +---------------------+
```

```
                      +----------------------------+
                      │        MapService          │
                      │  Global Occupancy Grid     │
                      │  - update_map(scan, pose)  │<──┐
                      │  - plan_path(start, goal)  │   │
                      │  - find_frontiers(pose)    │   │
                      │  - uses sequence lock      │   │
                      +----------------------------+   │
                               ^    ^    ^             │
                               │    │    │             │
       +--------------------+  │    │    │             │
       │   MappingTask      │──┘    │    │    +--------------------+
       │                    │       │    └────>   NavigationTask   │
       │ - Subscribes to    │       │         │                    │
       │   Lidar + Pose     │       └─────────┤ - Calls:           │
       │ - Calls update_map │                 │   plan_path()      │
       +--------------------+                 │   find_frontiers() │
                                              +--------------------+

                     [Seqlock Behavior Summary]
              +----------------------------------------+
              │ Writers (e.g., update_map):            │
              │   seq += 1  (odd)                      │
              │   modify grid                          │
              │   seq += 1  (even)                     │
              │                                        │
              │ Readers (e.g., plan_path):             │
              │   do {                                 │
              │     before = seq.load()                │
              │     ... read grid ...                  │
              │     after = seq.load()                 │
              │   } while (before != after)            │
              +----------------------------------------+
```
---

## Gazebo Simulation Setup

### Prerequisites
Ensure Gazebo **Garden** (which includes `gz-transport14`) is installed. Installation instructions can be found on the [Gazebo website](https://gazebosim.org/docs/garden). You may need to install development libraries:
```bash
# Example for Ubuntu
sudo apt-get update
sudo apt-get install gz-transport14-dev # Or the specific version needed
# May also need gz-msgs9-dev, gz-sim8-dev depending on full setup
```

### Running the Simulation
1.  **Start Gazebo**:
    ```bash
    # Navigate to the directory containing the world file
    cd gazebo/world
    # Launch the simulation world
    gz sim autotank_world.sdf
    ```
2.  **Build and Run Autotank in Simulation Mode**:
    ```bash
    # Navigate to the project root
    cd /path/to/autotank
    # Configure CMake for simulation build
    cmake -B build -S . -DUSE_SIM=ON
    # Build the project
    cmake --build build -j$(nproc)
    # Run the executable
    ./build/autotank
    ```
3.  **Verify Operation**:
    - Check the terminal output from `./autotank` for task state transitions and message logs (e.g., "IMU data received").
    - Use Gazebo Transport command-line tools to inspect topics:
        ```bash
        gz topic -l # List topics
        gz topic -e -t /imu # Echo IMU messages
        gz topic -e -t /odom # Echo Odometry messages
        gz topic -e -t /lidar # Echo Lidar messages
        gz topic -e -t /cmd_vel # Echo velocity commands sent by autotank
        ```

---

## Hardware Mode
For real hardware operation:
1.  **Implement Hardware HAL**: Create C++ implementation files in `src/hal/hardware/` (or similar) for each interface defined in `csc/` (e.g., `hardware_imu.cpp`, `hardware_diff_drive.cpp`). These will interact with your specific hardware drivers/SDKs.
2.  **Build with Hardware Mode Enabled**:
    ```bash
    # Navigate to the project root
    cd /path/to/autotank
    # Configure CMake for hardware build
    cmake -B build -S . -DUSE_SIM=OFF
    # Build the project
    cmake --build build -j$(nproc)
    # Run the executable (likely on the target hardware)
    ./build/autotank
    ```
3.  **Connect Sensors and Actuators**: Ensure hardware is correctly connected and powered.
4.  **Monitor Task Execution**: Observe console output and use appropriate hardware debugging tools.

---

## Build Instructions

### Prerequisites

- **Compiler**: A C++ compiler with C++17 support (e.g., GCC, Clang).
- **CMake**: Version 3.15 or higher.
- **Gazebo Garden & Libraries** (Optional, for simulation mode): `gz-transport14-dev`, `gz-msgs9-dev`, etc.
- **External Dependencies**:
    - **Protocore**: Core tasking/messaging framework. [Protocore on GitHub](https://github.com/A-Hopkins/protocore)
    - **kfplusplus**: Kalman Filter library. [Kfplusplus on GitHub](https://github.com/A-Hopkins/kfplusplus)

### Building the Project

1.  **Clone Repositories**:
    ```bash
    # Clone autotank
    git clone <autotank_repo_url> ~/workspace/autotank
    cd ~/workspace/autotank

    # Clone dependencies (place them where the symlinks expect or update CMakeLists.txt)
    git clone https://github.com/A-Hopkins/protocore.git ~/workspace/protocore
    git clone https://github.com/A-Hopkins/kfplusplus.git ~/workspace/kfplusplus

    # Create external directory and symlinks (if not using submodules/CMake FetchContent)
    mkdir -p external
    ln -sf ~/workspace/protocore external/protocore
    ln -sf ~/workspace/kfplusplus external/kfplusplus
    ```
    *Alternatively, consider using Git submodules or CMake's `FetchContent` to manage dependencies.*

2.  **Configure the Project with CMake**:
    ```bash
    # For Simulation Mode:
    cmake -B build -S . -DUSE_SIM=ON

    # For Hardware Mode:
    cmake -B build -S . -DUSE_SIM=OFF
    ```

3.  **Compile the Project**:
    ```bash
    cmake --build build -j$(nproc)
    ```
    The executable `autotank` will be located in the `build` directory.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---
