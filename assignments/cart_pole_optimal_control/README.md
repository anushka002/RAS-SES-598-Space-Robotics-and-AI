# Cart-Pole Optimal Control
This repository This repository contains the code and detailed report for optimizing a Linear Quadratic Regulator (LQR) controller for the cart-pole system under seismic disturbances. The objective is to stabilize the pole while ensuring the cart remains within its physical limits despite external disturbances. This assignment incorporates an earthquake force generator, allowing students to simulate and control systems under seismic conditions, linking to the Virtual Shake Robot covered later in the course. The techniques learned in managing dynamic disturbances and maintaining stability are valuable for optimizing control in space robotics applications, such as Lunar landers and orbital debris removal systems. The goal is to evaluate the performance of the LQR controller and explore improvements using reinforcement learning (extra credit) for enhanced stability and control under external disturbances.

---
## Objectives:

The goal of this project is to analyze and tune the provided LQR controller to ensure stable operation of the cart-pole system under earthquake-induced disturbances. 

### Core Requirements
1. Analyze and tune the provided LQR controller to:
   - Maintain the pendulum in an upright position
   - Keep the cart within its ±2.5m physical limits
   - Achieve stable operation under earthquake disturbances
2. Document your LQR tuning approach:
   - Analysis of the existing Q and R matrices
   - Justification for any tuning changes made
   - Analysis of performance trade-offs
   - Experimental results and observations
3. Analyze system performance:
   - Duration of stable operation
   - Maximum cart displacement
   - Pendulum angle deviation
   - Control effort analysis

### Learning Outcomes
This project provides hands-on experience in optimal control using LQR controller, understanding system behaviour under external disturbances, and practical expereince with ROS2 and Gazebo, essential for real-world robotic applications like Lunar landers and orbital debris removal robots.

Additionally, the project involves:

1. **LQR Tuning & Justification**: Analyzing the given Q and R matrices, justifying modifications, and evaluating performance trade-offs.
2. **Performance Analysis**: Measuring stability duration, cart displacement, pendulum deviation, and control effort.
3. **Experimental Validation**: Running simulations in ROS2 and Gazebo, analyzing results, and documenting findings.

### Extra Credit
Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR
---

## System Description

### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

---
## Implementation

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]
---

## Getting Started

### Prerequisites
- ROS2 Humble or Jazzy
- Gazebo Garden
- Python 3.8+
- Required Python packages: numpy, scipy

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=humble

# Install ROS2 packages
sudo apt update
sudo apt install -y \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-rviz2

# Install Python dependencies
pip3 install numpy scipy control
```

### Repository Setup

#### If you already have a fork of the course repository:
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

#### If you don't have a fork yet:
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
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/cart_pole_optimal_control .
```

### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```
This will start:
- Gazebo simulation (headless mode)
- RViz visualization showing:
  * Cart-pole system
  * Force arrows (control and disturbance forces)
  * TF frames for system state
- LQR controller
- Earthquake force generator
- Force visualizer

### Visualization Features
The RViz view provides a side perspective of the cart-pole system with:

#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)
2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)
Arrow lengths are proportional to force magnitudes.
---

## Literature Review  
Several studies have explored optimal control strategies for the inverted pendulum system, particularly using **Linear Quadratic Regulator (LQR)** and **Reinforcement Learning (RL)** approaches. Prior work has demonstrated that LQR provides a reliable, mathematically optimal method for stabilizing inverted pendulums, while RL offers adaptive learning-based control. This project builds upon these concepts by systematically tuning LQR parameters and experimenting with RL-based control under seismic disturbances.

- The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
-
-

---
## Tuning Parameters

### Tuning Parameters Table

The following table summarizes the tuning parameters used for the LQR controller and their corresponding effects on performance:

| Q Matrix Weights        | R Value | Stability   | Cart Constraint Violation | Control Effort |
|-------------------------|---------|-------------|---------------------------|----------------|
| [1, 1, 10, 10] (Default)| 0.1     | Moderate    | High                      | Moderate       |
| [5, 1, 15, 10]          | 0.1     | Improved    | Moderate                  | High           |
| [5, 2, 20, 15]          | 0.05    | Best        | Low                       | Higher         |
| [10, 2, 30, 20]         | 0.02    | Overaggressive | Low                    | Very High      |
| [1, 1, 10, 10]          | 0.1     | Moderate    | High                      | Moderate       |
| [5, 1, 15, 10]          | 0.1     | Improved    | Moderate                  | High           |
| [5, 2, 20, 15]          | 0.05    | Best        | Low                       | Higher         |
| [10, 2, 30, 20]         | 0.02    | Overaggressive | Low                    | Very High      |



### Comparison and Selection of the Best Optimal Solution
Based on the results from the above table, the following parameter set was found to provide the best performance behaviour by the system:
```
Q = 
R = 
```
This configuration ensures:

- The pendulum remains stable under seismic disturbances.
- The cart stays within the physical boundaries with minimal constraint violations.
- Control effort is optimized for energy efficiency.

## Results and Discussion
After implementing and tuning the LQR controller, the following results were observed:

Stability: The pendulum remained upright under varying seismic disturbances.
Constraint Satisfaction: The cart remained within the designated boundary limits.
Control Effort: The controller minimized energy consumption while maintaining system performance.




## Extra Credits
--The reinforcement learning (DQN) controller, while providing smoother control, had slower response times and required significant training.


## Conclusion
The LQR controller was successfully tuned to achieve robust performance in stabilizing the cart-pole system under seismic disturbances. The optimized controller achieved a good trade-off between system stability, constraint satisfaction, and control efficiency. While reinforcement learning showed potential for smoother control, it was slower and required more training time. Therefore, the LQR controller remains the preferred method for this task.









---
---
## Analysis Requirements

### Performance Metrics
Students should analyze:
1. Stability Metrics:
   - Maximum pole angle deviation
   - RMS cart position error
   - Peak control force used
   - Recovery time after disturbances

2. System Constraints:
   - Cart position limit: ±2.5m
   - Control rate: 50Hz
   - Pole angle stability
   - Control effort efficiency

### Analysis Guidelines
1. Baseline Performance:
   - Document system behavior with default parameters
   - Identify key performance bottlenecks
   - Analyze disturbance effects

2. Parameter Effects:
   - Analyze how Q matrix weights affect different states
   - Study R value's impact on control aggressiveness
   - Document trade-offs between objectives

3. Disturbance Response:
   - Characterize system response to different disturbance frequencies
   - Analyze recovery behavior
   - Study control effort distribution

## Evaluation Criteria
### Core Assignment (100 points)
1. Analysis Quality (40 points)
   - Depth of parameter analysis
   - Quality of performance metrics
   - Understanding of system behavior

2. Performance Results (30 points)
   - Stability under disturbances
   - Constraint satisfaction
   - Control efficiency

3. Documentation (30 points)
   - Clear analysis presentation
   - Quality of data and plots
   - Thoroughness of discussion

### Extra Credit (up to 30 points)
- Reinforcement Learning Implementation (30 points)

## Tips for Success
1. Start with understanding the existing controller behavior
2. Document baseline performance thoroughly
3. Make systematic parameter adjustments
4. Keep detailed records of all tests
5. Focus on understanding trade-offs
6. Use visualizations effectively

## Submission Requirements
1. Technical report including:
   - Analysis of controller behavior
   - Performance data and plots
   - Discussion of findings
2. Video demonstration of system performance
3. Any additional analysis tools or visualizations created

## License
This work is licensed under a [Creative Commons Attribution 4.0 International License](http://creativecommons.org/licenses/by/4.0/).
[![Creative Commons License](https://i.creativecommons.org/l/by/4.0/88x31.png)](http://creativecommons.org/licenses/by/4.0/) 
