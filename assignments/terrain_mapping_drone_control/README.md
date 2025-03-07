# Assignment 3: Rocky Times Challenge - Search, Map, & Analyze

This ROS2 package implements an autonomous drone system for geological feature detection, mapping, and analysis using an RGBD camera and PX4 SITL simulation.

## Challenge 
To develop a controller for a PX4-powered drone to efficiently search, map, and analyze cylindrical rock formations in an unknown environment. The drone will identify two rock formations (10m and 7m tall cylinders), estimate their dimensions, and successfully land on top of the taller cylinder.

### Mission Objectives
1. Search and locate all cylindrical rock formations
2. Map and analyze rock dimensions:
   - Estimate height and diameter of each cylinder
   - Determine positions in the world frame
3. Land safely on top of the taller cylinder
4. Complete mission while logging time and energy performance.

---
## Submission

### Literature Review
Best Strategies for Search:
1. Lawnmower (Systematic Sweep)
Description: The drone flies in a series of parallel straight lines, covering the entire area systematically, like mowing a lawn. It typically moves back and forth at a fixed altitude, adjusting the spacing between lines based on the RGBD camera’s field of view (FOV).
Use Case: Ideal for exhaustive coverage of a bounded area when the environment’s size is known.
2. Spiral Search
Description: The drone starts at a central point and follows an outward spiral pattern, gradually increasing the radius. It continues until the entire area is covered or the targets are found.
Use Case: Effective when the starting point is near the likely location of targets or when the search area is circular.
3. Random Search
Description: The drone moves in random directions, adjusting its path based on sensor inputs or until it detects the targets. It relies on probabilistic coverage rather than a predefined pattern.
Use Case: Useful in highly uncertain environments where no prior knowledge exists, but less efficient for systematic coverage.
4. Frontier-Based Exploration
Description: The drone identifies "frontiers" (boundaries between explored and unexplored areas) using its RGBD camera and map data (e.g., from RTAB-Map). It prioritizes moving toward the nearest frontier to maximize new information gain.
Use Case: Optimal for unknown environments where real-time mapping is available, as in this ROS2 setup.
5. Grid-Based Search with Optimization
Description: The area is divided into a grid of cells, and the drone follows an optimized path (e.g., using A* or Traveling Salesman Problem algorithms) to visit each cell. It adjusts based on detected obstacles or targets.
Use Case: Balances coverage and efficiency when computational resources allow path optimization.

---
## Methodology

### 1. Selecting Optimal Search Stratergy:
We will compare outputs from two strategies: Spiral Search and Lawnmower (Bourstrophedon pattern) Search

**Optimal/Safe Choice:** Lawnmower Search

1. Guaranteed Coverage: Ensures detection of both 10m and 7m cylinders by calculating passes based on camera FOV within known bounds.
2. Safety and Precision: Straight-line paths reduce instability, aiding safe and precise landing on the 10m cylinder with PX4 control.
3. Time-Energy Balance: Predictable completion time, with energy optimized by efficient altitude and early stopping after detection.
4. Robustness: Consistently finds rocks across all 15 trials, regardless of position, unlike center-dependent Spiral.
Mapping Support: Structured movement enhances RTAB-Map’s mapping for accurate dimensions and 3D reconstruction.

**Why Other Search Strategies Falls Short?**
-


### 2. Estimating Dimensions of the Cylinder

Using Front-Facing Camera for Dimension Estimation:

1. Field of View (FOV): During the Lawnmower pattern at 10m altitude, the front camera can see the full height of the cylinders (10m and 7m) as the drone approaches them, capturing both the top and base in its FOV from a horizontal perspective.
2. Depth Data: The RGBD camera provides per-pixel depth, allowing us to measure real-world distances directly, not just pixel sizes.
3. Contrast with Downward Camera: At 10m altitude, the downward camera only sees the cylinder tops (and maybe the ArUco marker) when directly above. It can’t measure height until after detection, and even then, it requires hovering adjustments. The front camera handles detection and estimation in one pass.


### 3. Saving Energy (energy-conscious path planning)




### 4. Safe and precise landing on the target cylinder




### 5.










---
## Prerequisites

- ROS2 Humble
- PX4 SITL Simulator
- RTAB-Map ROS2 package
- OpenCV
- Python 3.8+

## Repository Setup

### If you already have a fork of the course repository:

```bash
# Navigate to your local copy of the repository
cd ~/RAS-SES-598-Space-Robotics-and-AI

# Add the original repository as upstream (if not already done)
git remote add upstream https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI.git

# Fetch the latest changes from upstream
git fetch upstream

# Checkout your main branch
git checkout main

# Merge upstream changes
git merge upstream/main

# Push the updates to your fork
git push origin main
```

### If you don't have a fork yet:

1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork:
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

### Create Symlink to ROS2 Workspace

```bash
# Create symlink in your ROS2 workspace
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/terrain_mapping_drone_control .
```

### Copy PX4 Model Files

Before building the package, you need to copy the required PX4 model files:

```bash
# Navigate to the package
cd ~/ros2_ws/src/terrain_mapping_drone_control

# Make the setup script executable
chmod +x scripts/setup_px4_model.sh

# Run the setup script to copy model files
./scripts/setup_px4_model.sh
```

## Building and Running

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py

```
## Extra credit -- 3D reconstruction (50 points)
Use RTAB-Map or a SLAM ecosystem of your choice to map both rocks, and export the world as a mesh file, and upload to your repo. Use git large file system (LFS) if needed. 

## License

This assignment is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). 
For more details: https://creativecommons.org/licenses/by-nc-sa/4.0/
