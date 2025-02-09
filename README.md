# ROS2 Nav2 Tag Game with Behavior Trees

This project is designed for a student course focusing on Behavior Trees (BT) in ROS2 Navigation2. Students will work in pairs to develop behavior trees that enable robots to play a game of tag in a simulated environment.

## Project Overview

Two TurtleBot3 robots will participate in a game of tag where:
- One robot is designated as "it" (the chaser)
- The other robot must flee from the chaser
- Roles are assigned using an environment variable (ROBOT_ROLE=0 for chaser, ROBOT_ROLE=1 for runner)

Students will learn:
- ROS2 Navigation2 fundamentals
- Behavior Tree concepts and implementation
- Robot motion planning and control
- Multi-robot coordination

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- Nav2 stack
- TurtleBot3 packages

### Installation

#### Option 1: Direct Installation

1. Install ROS2 Humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)

2. Install required packages:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```

3. Clone this repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository-url> tag_bt_nav2
cd ~/ros2_ws
colcon build
```

#### Option 2: Using VSCode DevContainer (Recommended)

This method provides a consistent development environment for all students.

Prerequisites:
1. Install [Docker](https://docs.docker.com/engine/install/ubuntu/)
2. Install [VSCode](https://code.visualstudio.com/)
3. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) in VSCode

Steps:
1. Clone this repository:
```bash
git clone <repository-url> tag_bt_nav2
```

2. Open the project in VSCode:
```bash
code tag_bt_nav2
```

3. When prompted "Reopen in Container", click "Reopen in Container". Alternatively:
   - Press F1 (or Ctrl+Shift+P)
   - Type "Dev Containers: Reopen in Container"
   - Press Enter

4. Wait for the container to build (this may take several minutes the first time)

The container includes:
- ROS2 Humble
- Nav2 stack
- TurtleBot3 packages
- All necessary development tools

## Usage

1. Source your ROS2 workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

2. Launch the simulation with two robots:

For the chaser robot (Terminal 1):
```bash
export ROBOT_ROLE=0
ros2 launch tag_bt_nav2 my_bt.launch.py
```

For the runner robot (Terminal 2):
```bash
export ROBOT_ROLE=1
ros2 launch tag_bt_nav2 my_bt.launch.py
```

## Assignment Tasks

1. Implement a behavior tree for the chaser robot that:
   - Localizes itself in the map
   - Detects the runner robot's position
   - Plans and executes paths to catch the runner

2. Implement a behavior tree for the runner robot that:
   - Localizes itself in the map
   - Detects the chaser's position
   - Plans and executes evasive maneuvers

## Project Structure

- `launch/`: Launch files for the simulation
- `config/`: Configuration files including nav2 parameters
- `behavior_trees/`: BT XML files for both robots
- `worlds/`: Gazebo world files

## Contributing

This is an educational project. Students should:
1. Fork this repository
2. Create a branch for their team
3. Implement their behavior trees
4. Submit a pull request with their solution

## License

This project is licensed under the MIT License - see the LICENSE file for details

