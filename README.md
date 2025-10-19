# WATonomous ASD Admissions Assignment

## Prerequisite Installation
These steps are to setup the monorepo to work on your own PC. We utilize docker to enable ease of reproducibility and deployability.

> Why docker? It's so that you don't need to download any coding libraries on your bare metal pc, saving headache :3

1. This assignment is supported on Linux Ubuntu >= 22.04, Windows (WSL), and MacOS. This is standard practice that roboticists can't get around. To setup, you can either setup an [Ubuntu Virtual Machine](https://ubuntu.com/tutorials/how-to-run-ubuntu-desktop-on-a-virtual-machine-using-virtualbox#1-overview), setting up [WSL](https://learn.microsoft.com/en-us/windows/wsl/install), or setting up your computer to [dual boot](https://opensource.com/article/18/5/dual-boot-linux). You can find online resources for all three approaches.
2. Once inside Linux, [Download Docker Engine using the `apt` repository](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
3. You're all set! You can begin the assignment by visiting the WATonomous Wiki.

Link to Onboarding Assignment: https://wiki.watonomous.ca/
# Robot Navigation System

A ROS2-based autonomous navigation system for a differential-drive robot, using LiDAR and odometry to navigate from point A to point B while avoiding obstacles.

## 🎥 Demo

Watch the system in action:  
[![Robot Navigation Demo](https://img.youtube.com/vi/fCdanhtq-hcy/0.jpg)](https://www.youtube.com/watch?v=fCdanhtq-hc)

*(Click the image above to watch on YouTube)*

---

## Features

- **LiDAR-based Perception:** Converts laser scans into local occupancy grids (costmaps).  
- **Global Mapping:** Fuses local costmaps with odometry to build a global map of the environment.  
- **Path Planning:** Uses A* algorithm to plan optimal paths from current position to goal.  
- **Path Following:** Implements Pure Pursuit controller for smooth trajectory tracking.  
- **Dockerized:** Fully containerized ROS2 stack for reproducible builds and testing.  
- **Visualization:** Real-time robot behavior visualized using Foxglove Studio.

## Architecture

The system consists of the following main modules:

1. **Costmap Node** – Processes LiDAR data into a local occupancy grid.  
2. **Map Memory Node** – Aggregates local costmaps into a global map using odometry.  
3. **Planner Node** – Computes optimal paths to user-specified goal points using A*.  
4. **Control Node** – Follows the planned path using Pure Pursuit control.

## Getting Started

### Prerequisites

- ROS2 Humble  
- C++17  
- Docker  
- Foxglove Studio (for visualization)

### Build and Run

```bash
# Build the workspace
./watod build

# Run in Docker
docker-compose up
