# Autotank

Autotank is a C++ project used to explore the use of robotics hardware and Gazebo simulation,
with an emphasis on abstracting the application layer from the underlying simulator or hardware.
The framework facilitates task management, state coordination, inter-task communication through a message-passing system, and a dedicated broker for pub/sub and message queuing for IPC.

---

## Features

- **Task Management**:  
  - Implements a base task class (`BaseTask`) that encapsulates threading, message queuing, and state transition functionalities.  
  - Each task can subscribe to and publish messages, enabling decoupled communication among different system components.

- **State Management**:  
  - A centralized `StateManager` coordinates state transitions across tasks.  
  - It ensures that all registered tasks acknowledge state changes before progressing, promoting synchronized operations.

- **Message-Based Communication**:  
  - Uses a custom message system defined in `msg.h` with various message types (e.g., `STATE`, `STATE_ACK`) and priorities.  
  - A `Broker` class manages the publisher-subscriber model, ensuring efficient message routing between tasks.  
  - A `MessageQueue` class handles inter-process communication (IPC), decoupling message handling from task logic.

- **Simulation and Hardware Abstraction**:  
  - The application logic remains independent of the execution environment, allowing seamless switching between Gazebo simulation and real hardware.

- **Modular and Extensible Design**:  
  - Easily extendable to incorporate additional tasks, sensors, and functionalities.  
  - Example usage is provided in `main.cpp` where example tasks are registered and managed by the `StateManager`.

---

## File Structure

- **Core Functionality**  
  - `src/core/base_task.h` & `src/core/base_task.cpp`:  
    Define and implement the `BaseTask` class, providing core mechanisms for task operation, state transitions, and message handling.
  
  - `src/core/state_manager.h` & `src/core/state_manager.cpp`:  
    Implement the `StateManager` which oversees state transitions and manages the lifecycle of registered tasks.
  
  - `src/core/msg.h`:  
    Contains the definition of the `Msg` class used for inter-task communication, including enumerations for message types and priorities.

  - `src/core/broker.h`:
    Implement the `Broker` class, which manages the pub/sub system and routes messages between tasks.

  - `src/core/message_queue.h`:
    Provide the `MessageQueue` class, which handles IPC and decouples message handling from task logic.

- **Hardware Abstraction Layer (HAL)**  
  - `src/hal/imu.h` & `src/hal/gazebo_imu.cpp`:  
    Provide a basic interface for an IMU. The Gazebo simulation implementation leverages Gazebo Transport libraries to simulate sensor data.

  - **Application Entry Point**  
  - `src/main.cpp`:  
    Demonstrates how to create tasks (e.g., `ExampleTask`), register them with the `StateManager`, and perform state transitions.

- **Build Configuration**  
  - `CMakeLists.txt`:  
    Provides the CMake build configuration. It includes options for simulation support (via Gazebo Transport) and defines the necessary source files and compiler settings.

---

## Architecture Overview

Autotank follows a **publisher-subscriber** model for inter-component communication. Components (tasks) publish messages that other subscribed components receive and process accordingly.

### Message Flow
- Tasks can **subscribe** to specific message types.
- The `StateManager` acts as a central orchestrator, ensuring tasks transition properly between states.
- Messages carry state updates and other data, facilitating communication between different system modules.

### Task Lifecycle
Each task:
1. **Subscribes** to relevant message types via the `Broker`.
2. **Processes** incoming messages asynchronously using the `MessageQueue`.
3. **Transitions** between predefined states (e.g., IDLE, RUNNING, STOPPED) based on received messages.
4. **Publishes** acknowledgment messages to confirm state transitions.

---

## Gazebo Simulation Setup

### Prerequisites
Ensure Gazebo and its transport library are installed:
```bash
sudo apt-get install gazebo11 libgz-transport11-dev
```

### Running the Simulation
1. **Start Gazebo**:
   ```bash
   cd gazebo/world
   gz sim autotank_world.sdf
   ```
2. **Run Autotank in Simulation Mode**:
   ```bash
   cmake -DUSE_SIM=ON ..
   make
   ./autotank
   ```
3. **Verify Sensor Data**:
   - The IMU sensor should publish simulated data.
   - Messages should be exchanged between tasks, coordinating state transitions.

---

## Hardware Mode
For real hardware operation:
1. **Build with Hardware Mode Enabled**:
   ```bash
   cmake -DUSE_SIM=OFF ..
   make
   ./autotank
   ```
2. **Connect Sensors and Actuators**:
   - Ensure IMU and other peripherals are properly connected.
   - Modify `hal/` implementations to interface with actual hardware.

3. **Monitor Task Execution**:
   - Observe task transitions via console output.
   - Debug message exchanges using logging.

---

## Build Instructions

### Prerequisites

- **Compiler**: A C++ compiler with C++17 support.
- **CMake**: Version 3.15 or higher.
- **Optional**: Gazebo Transport 14 (for simulation mode).
- **Protocore**: External framework for core work

### Building the Project

1. **Clone the Repository**:  
   Ensure that you have the project source code on your local machine. You will also need to clone the protocore repo and link it with an external folder
   
   ```bash
   # Create external directory
   mkdir -p workspace/autotank/external
   
   # Create symlink to protocore
   ln -sf ~/workspace/protocore workspace/autotank/external/protocore
   ```

2. **Create a Build Directory**:
   ```bash
   mkdir build
   cd build
   ```

3. **Configure the Project with CMake**:  
   ```bash
   cmake ..
   ```

4. **Compile the Project**:
   ```bash
   make
   ```

---

## License

This project is licensed under the MIT License.

---
