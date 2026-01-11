# LLM Task Planner for Robotics in ROS 2 Humble
[![Python](https://img.shields.io/badge/python-3.10-blue)](https://www.python.org/)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green)](https://docs.ros.org/en/humble/)
[![OpenAI](https://img.shields.io/badge/OpenAI-API-red?logo=openai)](https://platform.openai.com/)
[![HuggingFace](https://img.shields.io/badge/HuggingFace-API-orange?logo=huggingface)](https://huggingface.co/)
[![OpenRouter](https://img.shields.io/badge/OpenRouter-API-purple?logo=openai)](https://openrouter.ai/)

---

**A ROS 2 Humble framework integrating Large Language Models (LLMs) with Nav2 for high-level task planning and autonomous robot navigation.**

---

## Table of Contents

1. [Overview](#overview)
2. [Features](#features)
3. [Project Structure](#project-structure)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Contributing](#contributing)

---

## Overview

This project combines the reasoning capabilities of Large Language Models with ROS 2's Nav2 navigation stack. It allows a mobile robot to:

- Plan complex tasks based on high-level natural language instructions
- Navigate autonomously in a simulated environment
- Integrate sensor data and costmaps for safe movement

It is designed to be **modular and simulation-ready**, making it easy to extend for research or development purposes.

---

## Features

- **Task Planning with LLMs**: Generate sequences of robot actions from natural language instructions.
- **Autonomous Navigation**: Leverages ROS 2 Nav2 stack for path planning, local and global costmaps.
- **Simulation Ready**: Compatible with Gazebo or other ROS 2 simulation environments.
- **Modular Architecture**: Easy to extend with new behaviors or navigation algorithms.

---

## Project Structure

```bash
LLM-Task-Planner/
├── src/                      # Source packages
│   ├── llm_nav/              # LLM-based task planning package
│   │   ├── behavior_trees/   # Behavior tree XMLs and configurations
│   │   ├── launch/           # ROS 2 launch files for nodes
│   │   ├── llm_nav/          # Node implementations
│   │   ├── maps/             # Map YAML / PGM files
│   │   ├── params/           # Parameter files
│   │   ├── rviz/             # RViz visualization configs
│   │   ├── resource/         # Other resources
│   │   └── test/             # Unit and integration tests
│   └── my_gazebo_worlds/     # Custom simulation worlds
│       ├── launch/           # Launch files for simulation
│       ├── maps/             # Map YAML / PGM files
│       ├── models/           # Simulation models
│       ├── photos/           # Screenshots and images
│       └── worlds/           # Gazebo world files
├── README.md                 # Project description and instructions
└── requirements.txt          # Python dependencies
```

**Notes:**  

- `src/` contains all ROS 2 packages (`llm_nav`, `my_gazebo_worlds`).  
- `llm_nav/`: main ROS2 package that implements the task planner and integration with Nav2.  
- `my_gazebo_worlds/`: auxiliary ROS2 package that contains simulation environments and assets.   
 

---

## Installation

1. **Install ROS 2 Humble**  
    - Follow the official guide: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

2. **Clone this repository**

```bash
git clone https://github.com/Dacossti/LLM-Task-Planner.git
cd LLM-Task-Planner/llm_ros2_humble
```

3. **Install ROS dependencies**

```bash
rosdep install --from-paths src --ignore-src -r -y
```

4. Install Python dependencies

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```


4. **Build the workspace**

```bash
colcon build
source install/setup.bash
```


## Usage

- Launch Navigation and Gazebo and RViz Simulation:

```bash
ros2 launch llm_nav bringup.launch.py use_nav2:=true instruction:="your_instruction"
```

Instruction example: `Bring me coffee in the dining room`