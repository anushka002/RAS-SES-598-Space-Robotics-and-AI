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

| Trial 1 | Trial 2 | Trial 3 |
|--------|---------|---------|
| ![TRIALGIF2](https://github.com/user-attachments/assets/1c9cb637-9e3a-43b8-bf91-6a311dd4bee0) | ![TRIALGIF3](https://github.com/user-attachments/assets/9e7fe304-f3e8-4785-a6c3-df1c969eea9c) | ![trialgif4](https://github.com/user-attachments/assets/01897582-220c-4057-8f86-02725953c99d) |
---
## Submission

### Literature Review
Comparison of Search Strategies

## 🔍 Comparison of Search Strategies

| Strategy                         | Description                                                                                                                                     | Use Case                                                                                     | This Project |
|----------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------|--------------|
| 1. Lawnmower (Systematic Sweep)  | Drone flies in parallel lines (like mowing a lawn), covering the area uniformly. Line spacing depends on the RGBD camera's field of view (FOV). | Best for exhaustive coverage of a known-size environment.                                    | ✅ Implemented |
| 2. Spiral Search                 | Starts from the center and expands outward in a spiral pattern, increasing radius incrementally.                                                | Ideal when the target is expected near the center or when the area is circular.              | ❌ Not Used   |
| 3. Random Search                 | Drone selects random directions, relying on chance-based coverage and sensor feedback to find targets.                                         | Useful in highly uncertain or unmapped environments, though potentially inefficient.         | ❌ Not Used   |
| 4. Frontier-Based Exploration    | Identifies frontiers (boundaries between explored and unexplored regions) and prioritizes them using RGBD camera and real-time mapping.        | Optimal in unknown environments using SLAM (e.g., RTAB-Map) to maximize exploration utility. | ❌ Not Used   |
| 5. Grid-Based Search with Optimization | Area is divided into discrete grid cells; drone follows an optimized path (e.g., A*, TSP) to maximize efficiency.                        | Best when combining systematic coverage with path planning in semi-known or cluttered areas. | ❌ Not Used   |

📌 This project uses a Lawnmower (Systematic Sweep) strategy to perform area coverage. If an ArUco marker is detected during the sweep, the drone aborts the survey and transitions into a precision approach and landing mode.



---
## Methodology

### 1. Selecting Optimal Search Stratergy:
We will compare outputs from two strategies: Spiral Search and Lawnmower (Bourstrophedon pattern) Search

**Optimal/Safe Choice:** **_Lawnmower Search_**

1. Guaranteed Coverage: Ensures detection of both 10m and 7m cylinders by calculating passes based on camera FOV within known bounds.
2. Safety and Precision: Straight-line paths reduce instability, aiding safe and precise landing on the 10m cylinder with PX4 control.

---

### Safe and precise landing on the target cylinder (My Accoplishment)

### 🧠 Mission Workflow

1. Drone takes off to a predefined altitude (e.g., 12 meters).
2. Executes a boustrophedon coverage of a specified 2D area.
3. During the survey, the downward-facing camera monitors for ArUco markers.
4. Upon detecting a marker:
   - The drone aborts the remainder of the survey.
   - Calculates the marker’s global position using solvePnP and onboard odometry.
   - Gradually approaches the marker location using position setpoints.
   - Hovers and stabilizes over the marker.
   - Initiates a smooth vertical descent and lands directly over the target.

---
### 📦 Project Structure

All logic is encapsulated in a single ROS 2 Python node:

| Component              | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| ArUco Detection         | OpenCV-based detection from `/drone/down_mono` camera stream                |
| Marker Pose Estimation | Uses solvePnP + camera intrinsics + PX4 odometry for global position inference |
| State Machine           | TAKEOFF → SURVEY → GOTO_ARUCO → HOVER → DESCEND → LAND                     |
| PX4 Control             | Publishes to `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command`, etc. |

---

### ✈️ ROS 2 Topics Used

- `/drone/down_mono` — input camera feed (mono8)
- `/drone/down_mono/camera_info` — camera intrinsics for solvePnP
- `/fmu/out/vehicle_odometry` — PX4 odometry data
- `/fmu/in/offboard_control_mode` — offboard position control
- `/fmu/in/trajectory_setpoint` — position setpoints
- `/fmu/in/vehicle_command` — arming, landing, and mode commands

---

## 🔧 Setup Instructions

1. Ensure ROS 2 Humble, PX4 SITL, and MAVROS interface are properly set up.
2. Calibrate the camera and ensure intrinsics are published to `/drone/down_mono/camera_info`.
3. Clone this repository and build:
   ```bash
   colcon build --packages-select terrain_mapping_drone_control
   source install/setup.bash
   ```
4. Run the mission node:
   ```bash
   ros2 run terrain_mapping_drone cylinder_landing.launch.py
   ./QGroundControl.AppImage
   ./aruco_lawnmower_landing.py
   ```

---

### ⚙️ Parameters

- ArUco marker size: 0.8 meters
- Camera assumed to be mounted facing downward
- Takeoff height: 15.0 meters (adjustable)
- Landing height: 0.2 meters (hover-close to ground)
- Survey resolution: 2.0 m steps (modifiable)

---

### 📌 Notes

- ArUco detection uses OpenCV 4.7+ (compatible with cv2.aruco.ArucoDetector).
- Marker detection starts only after successful takeoff.
- Boustrophedon survey is immediately aborted if an ArUco marker is detected.
- The drone hovers for a short period over the marker before initiating descent.

---
### 🤝 RESULTS

During testing in Gazebo simulation, the drone successfully:

- Performed a complete takeoff and transition into a boustrophedon coverage pattern.
- Detected the ArUco marker in real-time using the downward camera stream.
- Computed the marker's approximate global position using solvePnP and PX4 odometry.
- Aborted the remaining survey path upon detection and redirected toward the marker.
- Hovered above the estimated marker location for stabilization.
- Performed a gradual descent and executed a controlled landing sequence.

📌 However, in repeated trials, the drone consistently landed slightly off-center — adjacent to the ArUco marker (e.g. next to the cylinder) instead of directly above it.

This offset is likely due to:

- Frame misalignment or transform errors between camera and body frames.
- Inaccurate offset mapping of solvePnP tvec into world frame.
- Inconsistent detection depth (Z) or noisy odometry during descent.



## 🎯 Experimental Trials Overview

Below are the recorded results from three autonomous drone missions using the ArUco-guided Boustrophedon landing system. Each trial includes full execution of the lawnmower survey, ArUco marker detection, hover stabilization, and landing maneuver.

---

### 🧪 Trial 1

<img src="https://github.com/user-attachments/assets/1c9cb637-9e3a-43b8-bf91-6a311dd4bee0" alt="TRIALGIF2" width="800"/>

✅ Lawnmower Coverage Complete  
✅ ArUco Detected Mid-Survey  
✅ Smooth Hovering Above Marker  
✅ Controlled Descent Executed  
❌ Landed Slightly Off-Center

---

### 🧪 Trial 2

<img src="https://github.com/user-attachments/assets/9e7fe304-f3e8-4785-a6c3-df1c969eea9c" alt="TRIALGIF3" width="800"/>

✅ Lawnmower Coverage Complete  
✅ ArUco Detected Mid-Survey  
✅ Hovering Maintained Over Target  
✅ Descent Triggered and Landed  
❌ Minor Offset in Final Touchdown

---

### 🧪 Trial 3

<img src="https://github.com/user-attachments/assets/01897582-220c-4057-8f86-02725953c99d" alt="TRIALGIF4" width="800"/>

✅ Lawnmower Pattern Followed  
✅ ArUco Detected from Aerial View  
✅ Stabilization Achieved  
✅ Soft Descent Performed  
❌ Touchdown Slightly Beside Marker

---
## ✅ Results Summary

| Trial   | Boustrophedon (Lawnmower) | ArUco Detection | Hover Above Marker | Controlled Descent | Landed Precisely on Marker | Output|
|---------|----------------------------|------------------|---------------------|---------------------|------------------------------|-----|
| Trial 1 | ✅ Completed               | ✅ Detected      | ✅ Stabilized       | ✅ Performed        | ❌ Landed Slightly Off       |![TRIALGIF2](https://github.com/user-attachments/assets/1c9cb637-9e3a-43b8-bf91-6a311dd4bee0) |
| Trial 2 | ✅ Completed               | ✅ Detected      | ✅ Stabilized       | ✅ Performed        | ❌ Landed Slightly Off       |![TRIALGIF3](https://github.com/user-attachments/assets/9e7fe304-f3e8-4785-a6c3-df1c969eea9c) |
| Trial 3 | ✅ Completed               | ✅ Detected      | ✅ Stabilized       | ✅ Performed        | ❌ Landed Slightly Off       |![trialgif4](https://github.com/user-attachments/assets/01897582-220c-4057-8f86-02725953c99d) |

📌 Observation:  
In all trials, the complete mission pipeline functioned as expected — including survey, marker detection, hover stabilization, and descent. However, the drone consistently landed slightly beside the ArUco marker rather than directly on top of it. This minor offset may stem from coordinate frame mismatch, noisy tvec estimation, or accumulated drift.

---
## Setup Guide

**Prerequisites**

- ROS2 Humble
- PX4 SITL Simulator (Tested with PX4-Autopilot main branch 9ac03f03eb)
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

Copy the custom PX4 model files to the PX4-Autopilot folder

```bash
# Navigate to the package
cd ~/ros2_ws/src/terrain_mapping_drone_control

# Make the setup script executable
chmod +x scripts/deploy_px4_model.sh

# Run the setup script to copy model files
./scripts/deploy_px4_model.sh -p /path/to/PX4-Autopilot
```

## Building and Running

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select terrain_mapping_drone_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization with your PX4-Autopilot path
ros2 launch terrain_mapping_drone_control cylinder_landing.launch.py

# OR you can change the default path in the launch file
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
```
## Extra credit -- 3D reconstruction (50 points)
Use RTAB-Map or a SLAM ecosystem of your choice to map both rocks, and export the world as a mesh file, and upload to your repo. Use git large file system (LFS) if needed. 

## License

This assignment is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0). 
For more details: https://creativecommons.org/licenses/by-nc-sa/4.0/
