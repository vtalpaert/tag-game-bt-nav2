# ROS2 Nav2 Tag Game with Behavior Trees

This project is designed for a student course focusing on Behavior Trees (BT) in ROS2 Navigation2. Students will work in pairs to develop behavior trees that enable robots to play a game of tag in a simulated environment.

## Project Overview

Two TurtleBot3 robots will participate in a game of tag where:
- Robot1 is designated as "it" (the chaser)
- Robot2 must flee from the chaser
- Each robot uses its own behavior tree configuration file (robot1_bt.xml and robot2_bt.xml)

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

First, fork this repository to your own GitHub account:
1. Visit the repository page
2. Click the "Fork" button in the top-right corner
3. Select your GitHub account as the destination

#### Option 1: Direct Installation

1. Install ROS2 Humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)

2. Install required packages:
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```

3. Clone your forked repository:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/tag_bt_nav2.git
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

2. Start the Gazebo server:
```bash
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so /opt/ros/humble/share/nav2_bringup/worlds/world_only.model
```

3. Launch the simulation with two robots:
```bash
ros2 launch tag_bt_nav2 my_bt.launch.py
```

This will launch both robots with their respective behavior trees:
- Robot1 (chaser) using robot1_bt.xml
- Robot2 (runner) using robot2_bt.xml

You can interact with the robots using RViz to set initial positions and monitor their behavior.

## Assignment Tasks

Students will work in pairs, with each team implementing both a chaser and runner behavior tree:

1. Chaser Behavior Tree (robot1_bt.xml):
   - Localizes itself in the map
   - Detects the runner robot's position
   - Plans and executes paths to catch the runner

2. Runner Behavior Tree (robot2_bt.xml):
   - Localizes itself in the map
   - Detects the chaser's position
   - Plans and executes evasive maneuvers

### Grading Criteria

1. Basic Requirements (Pass/Fail):
   - Each student must have meaningful commits in the git history
   - Both behavior trees must be functional
   - Code must be well-documented and follow ROS2 conventions

2. Bonus Points:
   - Teams can earn extra points through a competition
   - Behavior trees from different teams will be mixed and matched
   - Points awarded based on:
     - Chaser effectiveness when paired with other teams' runners
     - Runner effectiveness when paired with other teams' chasers
     - Innovation in strategy and implementation

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

