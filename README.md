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
    - **`LocalizationTask`**: Fuses sensor data (IMU, Odometry, Lidar) and command velocity inputs to estimate the robot's pose (position and orientation) and twist (linear and angular velocity). Publishes `LocalizationEstimateMsg`. (Utilizes `kfplusplus` for potential filter implementations).
    - **`MotionControlTask`**: Receives localization estimates and target waypoints (future), calculates safe and appropriate velocity commands (`CmdVelMsg`), and sends them to the differential drive interface.
- **Message Definitions**: Custom message types (`msg/*.h`) define the data structures for inter-task communication (e.g., `IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`, `CmdVelMsg`, `LocalizationEstimateMsg`).
- **Extensible Design**: Easily add new sensor tasks, control algorithms, or higher-level capabilities like navigation and mapping.
- **Planned Features**:
    - **`NavigationTask`**: Will be responsible for path planning, obstacle avoidance, and waypoint following based on the localization estimate and map data.
    - **`MappingTask`**: Will build and maintain a representation of the environment, potentially performing the mapping part of SLAM (Simultaneous Localization and Mapping), while localization handles the 'SL' part.

---

## File Structure

- **`src/`**: Source code for the application.
    - **`csc/`**: Computer Software Components (Core application logic).
        - `control/`: Control-related tasks.
            - `motion_control/`: Differential drive motion control (`MotionControlTask`, `DiffDrive` interface).
        - `localization/`: State estimation (`LocalizationTask`).
        - `sensors/`: Sensor processing tasks (`IMUTask`, `OdomTask`, `LidarTask`) and their interfaces (`imu.h`, `odom.h`, `lidar.h`).
        - `navigation/` (Future): Navigation logic.
        - `mapping/` (Future): Mapping logic.
    - **`hal/`**: Hardware Abstraction Layer implementations.
        - `gazebo/`: Implementations for Gazebo simulation (`gazebo_imu.cpp`, `gazebo_odom.cpp`, `gazebo_lidar.cpp`, `gazebo_diff_drive.cpp`, `gazebo_helpers.h`).
        - `hardware/` (Future): Implementations for specific hardware drivers.
    - **`main.cpp`**: Application entry point, task initialization, and `protocore` setup.
- **`include/`**: Header files.
    - **`csc/`**: Headers for Computer Software Components.
    - **`hal/`**: Headers for Hardware Abstraction Layer interfaces (defined within `csc/` subdirs like `sensors/imu/imu.h`).
    - **`msg/`**: Message definitions.
        - `common_types/`: Reusable data structures (e.g., `Header`, `Pose`, `Twist`).
        - `msg_variant_types.h`: Project-specific variant definition overriding `protocore`'s default.
        - Individual message headers (e.g., `imu_msg.h`, `odom_msg.h`).
- **`external/`**: Contains dependencies linked via CMake (e.g., `protocore`, `kfplusplus`). Managed via symlinks or Git submodules.
- **`gazebo/`**: Gazebo simulation files.
    - `world/`: SDF world files (`autotank_world.sdf`).
    - `models/` (Optional): Custom Gazebo models.
- **`CMakeLists.txt`**: Build configuration script.
- **`.vscode/`**: VS Code specific settings (launch, tasks).
- **`README.md`**: This file.
- **`LICENSE`**: Project license.

---

## Architecture Overview

Autotank utilizes the task-based architecture provided by `protocore`. Each significant function (sensor handling, localization, control) is encapsulated within a `Task`.

- **Communication**: Tasks communicate asynchronously by publishing and subscribing to specific message types via the `protocore` `Broker`. Message queuing and thread management are handled by the `protocore::Task` base class and `MessageQueue`.
- **State Management**: The `protocore::StateManager` orchestrates the lifecycle of all registered tasks (e.g., `INITIALIZING`, `RUNNING`, `STOPPED`), ensuring synchronized state transitions across the system.
- **Data Flow**:
    1.  **Sensor Tasks** (`IMUTask`, `OdomTask`, `LidarTask`) interface with the HAL (Gazebo or hardware) to acquire raw sensor data. They process this data and publish it as specific message types (e.g., `IMUDataMsg`, `OdomDataMsg`, `LidarDataMsg`).
    2.  **`LocalizationTask`** subscribes to sensor messages and potentially `CmdVelMsg` (for motion prediction). It fuses this information using an estimation filter (e.g., EKF via `kfplusplus`) to produce a `LocalizationEstimateMsg` containing the robot's estimated pose and twist.
    3.  **(Future) `NavigationTask`** subscribes to `LocalizationEstimateMsg` and map data (from `MappingTask`). It determines the desired path and intermediate waypoints, potentially publishing navigation goals.
    4.  **`MotionControlTask`** subscribes to `LocalizationEstimateMsg` (and navigation goals from `NavigationTask` in the future). It calculates the necessary linear and angular velocities to follow the path or reach the goal, considering safety constraints. It publishes these as a `CmdVelMsg`.
    5.  The **`DiffDrive` HAL implementation** (e.g., `gazebo_diff_drive`) receives the `CmdVelMsg` and translates it into commands for the simulation or hardware actuators.
    6.  **(Future) `MappingTask`** subscribes to sensor data (e.g., `LidarDataMsg`) and `LocalizationEstimateMsg` to build and update the map.

```
+--------------+       +--------------+       +--------------+       +-----------------+       +-------------+
| Sensor Tasks | ----> | Localization | ----> | Navigation   | ----> | Motion Control  | ----> | Diff Drive  |
| (IMU, Odom,  |       | Task         |       | Task (Future)|       | Task            |       | (HAL)       |
| Lidar)       |       +--------------+       +--------------+       +-----------------+       +-------------+
+--------------+              |                      |                        ^
       |                      |                      |                        |
       |                      v                      v                        |
       |                +--------------+       +--------------+               |
       +--------------->| Mapping Task |<----->| Navigation   |---------------+
                        | (Future)     |       | Task (Future)|
                        +--------------+       +--------------+
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
