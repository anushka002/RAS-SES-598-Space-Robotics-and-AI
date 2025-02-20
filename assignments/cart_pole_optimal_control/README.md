# Cart-Pole Optimal Control
This repository This repository contains the code and detailed report for optimizing a Linear Quadratic Regulator (LQR) controller for the cart-pole system under seismic disturbances. The objective is to stabilize the pole while ensuring the cart remains within its physical limits despite external disturbances. This assignment incorporates an earthquake force generator, allowing students to simulate and control systems under seismic conditions, linking to the Virtual Shake Robot covered later in the course. The techniques learned in managing dynamic disturbances and maintaining stability are valuable for optimizing control in space robotics applications, such as Lunar landers and orbital debris removal systems. The goal is to evaluate the performance of the LQR controller and explore improvements using reinforcement learning (extra credit) for enhanced stability and control under external disturbances.

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

---
# Submission

## Literature Review  
Several studies have explored optimal control strategies for the inverted pendulum system, particularly using **Linear Quadratic Regulator (LQR)** and **Reinforcement Learning (RL)** approaches. Prior work has demonstrated that LQR provides a reliable, mathematically optimal method for stabilizing inverted pendulums, while RL offers adaptive learning-based control. This project builds upon these concepts by systematically tuning LQR parameters and experimenting with RL-based control under seismic disturbances.

- The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
![image](https://github.com/user-attachments/assets/b9fef232-ee9d-44f6-98c2-ecb5007744e8)    ![image](https://github.com/user-attachments/assets/44fd7e5d-2fe3-4f93-894e-dcc876634442)


### 1. Understanding Q and R Matrices in LQR

In **Linear Quadratic Regulator (LQR)**, we define a **cost function**:
```
J = ∫ (x^T Q x + u^T R u) dt
```
where:
- **x** is the state vector (e.g., cart position, cart velocity, pole angle, pole angular velocity).
- **u** is the control input (e.g., force applied to the cart).
- **Q** is a **state-cost matrix**, penalizing deviation from the desired state.
- **R** is a **control-cost matrix**, penalizing excessive control effort.
#### **Objective:**
- **Higher values in Q** → Penalize large deviations in state variables (better tracking).
- **Higher values in R** → Penalize excessive control effort (reduces actuator strain).
- **Balanced tuning ensures stable control while avoiding excessive energy consumption.**
---
### 2. Choosing the Initial Q and R Matrices
#### **State Vector for the Cart-Pole System**
```
x = [ x_cart  v_cart  θ_pole  ω_pole ]^T
```
Each state variable represents:
1. **x_cart** → Position of the cart  
2. **v_cart** → Velocity of the cart  
3. **θ_pole** → Angle of the pole (from upright)  
4. **ω_pole** → Angular velocity of the pole  
A **good initial choice** for the Q matrix can be:
```
Q = [ q1  0   0   0  
      0  q2  0   0  
      0   0  q3  0  
      0   0   0  q4 ]
```
where:
- **q1** → Penalizes cart displacement (higher value to keep the cart near the center).
- **q2** → Penalizes cart velocity (moderate value to prevent excessive motion).
- **q3** → Penalizes pole angle deviation (higher value for fast recovery).
- **q4** → Penalizes pole angular velocity (moderate value for smoother motion).
### **Selecting the R Matrix**
```
R = [ r ]
```
- **A larger R** → Encourages less aggressive control input (reducing motor strain).
- **A smaller R** → Allows more aggressive control actions but may cause oscillations.
- **Typical range**: Start with `r = 1` and adjust based on response.

---
## Tuning Q and R Matrices for LQR in a Cart-Pole System

### Step 1: Start with a Default Setup
```
Q = [ 1  0   0   0  
      0  1   0   0  
      0  0  10   0  
      0  0   0   1  ]

R = [1]
```

### Step 2: Prioritize Pole Stabilization First
- Increase `q3` (pole angle penalty) significantly, e.g., `q3 = 50`.
- Increase `q4` slightly to avoid excessive oscillation.

### Step 3: Ensure Smooth Cart Motion
- Adjust `q1` and `q2` to prevent excessive movement.
- Keep them lower than `q3` (e.g., `q1 = 1, q2 = 1`).

### Step 4: Fine-Tune Control Effort (R Value)
- If the system is oscillating too much → Increase `R`.
- If control is too slow → Decrease `R`.


## Experimental Tuning Approach

### *Step 1: Simulate with Default Q and R*
- Check if the system stabilizes and how much control force is required.

### *Step 2: Increase Q Values for Better Tracking*
- If the pole is too slow to recover → Increase `q3`.
- If the cart moves too much → Increase `q1`.
- If oscillations are too high → Increase `q4`.

### *Step 3: Adjust R to Limit Aggressive Control*
- If the control effort is too high → Increase `R` (e.g., `R = 10`).
- If the response is sluggish → Decrease `R` (e.g., `R = 0.1`).

### Tuning Parameters Table

The following table summarizes the tuning parameters used for the LQR controller and their corresponding effects on performance:

| Q Matrix Weights        | R Value | Max Pole Angle Deviation | RMS Cart Position Error | Peak Control Force (Control Effort) | Recovery Time | Stability       | Overall Performance | Notes |
|-------------------------|---------|--------------------------|-------------------------|-------------------------|---------------|----------------|----------------------|-------|
| [1, 1, 10, 10] (default)| 0.1     | -22.1                    | 2.5                    | 85.21                   | 12            | Below average       | Needs Improvement   |Default parameters, Poor performance |
| [5, 1, 15, 10]         |  0.1     | 45.4                     | 2.48                    | 73.85                   | 12            | Below average       | Needs Improvement   |Poor performance |
| [10, 1, 10, 10]         | 0.1    | 64.44                     | 2.34                     | 75.38                  | 3.08         | Below Average         |Needs Improvement           |Default parameters, baseline performance |
| [50, 2, 50, 10]        | 0.1    | 54.16                 | 0.622                     | 71.72               | Very Short    | Little aggressive | Fair          |Fair performance |
| [30, 5, 30, 10]         | 0.05     | 10.63                 | 0.14                | 75.05                | 0.2        | Great       | Great                  |Great performance |
| [30, 5, 30, 10]         | 0.02    | 23.76                      | 0.354                     | 119.36                    | 0.586         | Well Stable         | Very Good           |Great performance  |
| [30, 5, 30, 10] ✅          | 0.01 ✅     | 6.28                 | 0.073                | 67.09                  | Very Short    | Best ✅            | Excellent Stability           |Best performance, Optimal ✅  |
| [30, 5, 30, 10]         | 0.005    | 24.6                  | 2.35                 | 109.83               | 0.647            | Below Average | Needs Improvement          |Very poor performance |

| [30, 5, 30, 10]         | 0.1    | 24.6                  | 2.35                 | 59.83               | 0.647            | Below Average | Needs Improvement          |Poor performance |


### Comparison and Selection of the Best Optimal Solution
Based on the results from the above table, the following parameter set was found to provide the best performance behaviour by the system:
```
✅ Q = [30, 5, 30, 10]
✅ R = [0.005]
```
### Image Output:
![final_result](https://github.com/user-attachments/assets/8c530e98-391c-49bc-a1a9-67136bceb787)

### Video Output 

![BEST](https://github.com/user-attachments/assets/55e6f3fc-d5f4-43b4-8934-5d6404c4262b)

### Plots



This configuration ensures:
- The pendulum remains stable under seismic disturbances.
- The cart stays within the physical boundaries with minimal constraint violations.
- Control effort is optimized for energy efficiency.

✅ **Fast stabilization** of the pole  
✅ **Minimal cart movement**  
✅ **Moderate control effort** to avoid excessive motor strain  



---
## Results and Discussion

- **Tuning Q and R is an iterative process**: Start with reasonable values and refine them based on simulations.
- **Focus on pole stabilization first** by increasing `q3` and `q4`.
- **Ensure smooth cart motion** by keeping `q1` and `q2` moderate.
- **Adjust `R` to balance aggressive control and actuator effort**.

After implementing and tuning the LQR controller, the following results were observed:

Stability: The pendulum remained upright under varying seismic disturbances.
Constraint Satisfaction: The cart remained within the designated boundary limits.
Control Effort: The controller minimized energy consumption while maintaining system performance.


## Conclusion
The LQR controller was successfully tuned to achieve robust performance in stabilizing the cart-pole system under seismic disturbances. 
By carefully tuning the weight parameters in the cost function, we can achieve a well-balanced control strategy for the cart-pole system. 
Prioritizing pole stabilization ensures the system remains upright, while adjusting cart motion parameters prevents excessive movement. 
Finally, fine-tuning the control effort (`R`) helps strike a balance between responsiveness and stability. Iteratively refining these values based on system behavior will lead to an optimal control policy that maintains stability with minimal energy consumption.
The optimized LQR controller achieved a good trade-off between system stability, constraint satisfaction, and control efficiency.

---

## Help / Resources

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
- ROS2 Humble or Jazzy ✅
- Gazebo Garden ✅
- Python 3.8+ ✅
- Required Python packages: numpy, scipy ✅

#### Installation Commands
```bash
# Set ROS_DISTRO as per your configuration
export ROS_DISTRO=jazzy ✅

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


### Extra Credit
Reinforcement Learning Implementation:
   - Implement a basic DQN (Deep Q-Network) controller
   - Train the agent to stabilize the pendulum
   - Compare performance with the LQR controller
   - Document training process and results
   - Create training progress visualizations
   - Analyze and compare performance with LQR
- Reinforcement Learning Implementation (30 points)

### Extra Credit (up to 30 points)

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
