# Hexagon Air Bearing Automation System

This project implements a fully automated robotic system for handling air bearings using RoboDK. The robot follows a predefined sequence to pick, flip, load, screw, unload, and reset bearings in a CNC-adjacent setup. The system is built with non-blocking motion logic, state machine control, and logging/resume features.

## Features

- ⚙️ **State Machine Control**  
  Task flow is driven by a robust, enum-based state machine for clarity and maintainability.

- 🚦 **Non-Blocking Robot Commands**  
  All robot movements are executed asynchronously (`blocking=False`) for optimized cycle times.

- 🎯 **Offset-Aware Movement**  
  Joint positions are dynamically offset to account for bearing tray layout and flipping.

- 🔌 **Automated Tool & I/O Control**  
  Digital outputs manage vacuum and blow-off tools based on the robot's current state.

- 🧠 **Checkpoint Recovery**\
  Supports recovery from the last known robot state using a CSV checkpoint file.

- 📊 **Logging System**\
  Logs timestamped robot state transitions and key variables for debugging and performance tracking.

- 🔄 **Scalable and Modular Design**\
  Easily configurable for various tray layouts, part sizes, and task variations.

## Getting Started

### Prerequisites

- [RoboDK](https://robodk.com/)
- Python 3.9+
- RoboDK Python API (typically installed with RoboDK)

### Running the Program

1. Open RoboDK and connect your robot setup.
2. Clone this repo and navigate to its folder:
   ```bash
   git clone https://github.com/your-username/robotic-bearing-loader.git
   cd robotic-bearing-loader
   ```
3. Run the main script:
  ```bash
  python main.py
  ```

## File Structure

- `main.py` – Core script running the state machine and robot tasks.
- `robot_log.csv` – Automatically generated log of state transitions.
- `last_state.csv` – Stores last checkpoint for recovery.

## Safety Notes

Ensure the robot is connected and homed before starting.
Use dry-run testing with RoboDK’s simulator before running on physical hardware.

