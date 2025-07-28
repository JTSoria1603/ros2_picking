

# ros2_picking

Robotic picking system based on ROS 2

## Description
This package contains a simulation of a robotic picking system, composed of several ROS 2 nodes that represent typical sensors and actuators of an industrial cell:

- **scanner_node**: Publishes random barcodes on the `barcode` topic and offers a service to get the last published code.
- **door_handle_node**: Simulates a door sensor and actuator, publishing its state on the `door_closed` topic and offering services to toggle and query the door state.
- **emergency_button_node**: Simulates an emergency button, publishing its state on the `emergency_button` topic and offering services to press, release, and query the button state.
- **stack_light_node**: Simulates a stack light, subscribing to the door and emergency button states, and publishing the stack light state on the `stack_light` topic.

## Repository structure

```
src/
  scanner_node/
  door_handle_node/
  emergency_button_node/
  stack_light_node/
  cell_launch/
```

## Launching the system

To launch all the nodes of the picking cell, use the launch file:

```
ros2 launch cell_launch cell.launch.py
```

This will start all the required nodes and display their output on the screen.


## Available Services

Each node provides the following ROS 2 services:

- **scanner_node**
  - `/get_barcode` (Trigger): Returns the last published barcode.

- **door_handle_node**
  - `/toggle_door` (Trigger): Toggles the door state (open/closed).
  - `/get_door_state` (Trigger): Returns the current door state.

- **emergency_button_node**
  - `/press_emergency_button` (Trigger): Sets the emergency button to PRESSED.
  - `/release_emergency_button` (Trigger): Sets the emergency button to RELEASED.
  - `/get_emergency_button_state` (Trigger): Returns the current emergency button state.

- **stack_light_node**
  - `/get_stack_light_state` (Trigger): Returns the current stack light state.


## Requirements
- ROS 2 (tested on Humble)
- Python 3.8+

## Installation and Replication

Follow these steps to install and replicate the project:

1. **Clone the repository:**
   ```bash
   git clone https://github.com/JTSoria1603/ros2_picking.git
   cd ros2_picking
   ```

2. **Install dependencies:**
   Make sure you have a working ROS 2 (Humble) environment and Python 3.8+.
   If using a ROS 2 workspace, move or link the `ros2_picking` folder into your `src/` directory.

3. **Build the workspace:**
   ```bash
   cd ~/ros2_ws  # or your workspace root
   colcon build
   source install/setup.bash
   ```

4. **Launch the system:**
   ```bash
   ros2 launch cell_launch cell.launch.py
   ```

This will start all nodes and display their output. You can interact with the services using `ros2 service call`.

## Credits
Developed by JTSoria1603.
