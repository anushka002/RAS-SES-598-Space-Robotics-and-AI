# First-Order Boustrophedon Navigator

This repository documents the implementation and tuning of a first-order boustrophedon navigator using ROS2 with Turtlesim. The project focuses on achieving a precise lawnmower survey pattern by tuning a PD controller, minimizing cross-track error, and ensuring smooth motion through systematic parameter adjustments.

## Overview
Boustrophedon patterns, inspired by ox-turning plow trajectories, are fundamental in robotic coverage applications like space exploration, Earth observation, and underwater mapping. This assignment leverages ROS2 and Turtlesim to simulate these patterns in a 2D environment, offering hands-on experience in PD control tuning and trajectory optimization.

## Objective
- Implement a precise boustrophedon survey pattern.
- Tune PD controller parameters to minimize cross-track error.
- Analyze trajectory performance metrics.

## Getting Started
### Prerequisites
- **OS**: Ubuntu 22.04
- **ROS2**: Humble

### Repository Setup
1. **Clone Repository**:
   ```bash
   cd ~/
   git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
   ```
2. **Create Symlink**:
   ```bash
   cd ~/ros2_ws/src
   ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
   ```
3. **Build Package**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select first_order_boustrophedon_navigator
   source install/setup.bash
   ```

### Running the Demo
Launch the simulation:
```bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```
Monitor performance:
```bash
ros2 topic echo /cross_track_error
```
Visualize trajectory and performance:
```bash
ros2 run rqt_plot rqt_plot
```
Add topics:
- `/turtle1/pose/x`
- `/turtle1/pose/y`
- `/turtle1/cmd_vel/linear/x`
- `/turtle1/cmd_vel/angular/z`
- `/cross_track_error`

## Literature Review
**1. Boustrophedon Pattern for Lawn Mowing** 

- The boustrophedon pattern ensures systematic and efficient area coverage with minimal overlap, dividing the workspace into cells and using back-and-forth sweeping motions. <br>
- It is widely used in structured environments like lawns and integrates SLAM and adaptive path planning to handle obstacles dynamically.<br>
- References:<br>
[Choset, H., & Pignon, P. (1997). Coverage path planning: The boustrophedon cellular decomposition. Proceedings of the International Conference on Field and Service Robotics](https://link.springer.com/chapter/10.1007/978-1-4471-1273-0_32)<br>

**2. Simulation Using Turtlesim**

- Turtlesim, a ROS-based 2D simulator, helps visualize robotic motion and test algorithms such as coverage path planning in an interactive and simple environment.<br>
- It is ideal for simulating basic tasks like boustrophedon patterns and waypoint navigation before transitioning to more advanced simulators.<br>
- References:<br>
[Quigley, M., Conley, K., Gerkey, B., Faust, J., Foote, T., Leibs, J., ... & Ng, A. Y. (2009). ROS: an open-source Robot Operating System. ICRA Workshop on Open Source Software](https://www.researchgate.net/publication/233881999_ROS_an_open-source_Robot_Operating_System)<br>


## Tuning Methodology
### Implementation Steps
In this assignment, I followed a systematic approach to tune the control parameters for the TurtleBot, ensuring optimal performance in path tracking. The methodology included the following:

1. **Starting with Low Gains**  
   We initialized the proportional (`Kp`) and derivative (`Kd`) gains with small values to prevent oscillations or instability in the robot's movement.
2. **Testing One Parameter at a Time**  
   Parameters were adjusted incrementally, focusing on one variable at a time (e.g., `Kp`, `Kd`, linear velocity, or angular velocity). This allowed us to evaluate the specific impact of each parameter on performance.
3. **Evaluating Both Straight-Line Tracking and Cornering**  
   The robot's behavior was assessed during both straight-line navigation and while taking turns, ensuring balanced tuning for different path scenarios.
4. **Using `rqt_plot` and `rqt_reconfigure` for Real-Time Visualization**  
   We leveraged `rqt_plot` to monitor metrics like cross-track error in real time. This provided immediate feedback on the robot's performance and guided parameter adjustments. We have used `ros2 run rqt_reconfigure rqt_reconfigure` to dynamically change the parameters for tuning.
5. **Balancing Speed and Accuracy**  
   Trade-offs between speed (linear velocity) and accuracy (cross-track error) were considered to strike a balance between efficiency and reliability.


## Parameter Tuning Analysis
### For Spacing = 1.0
| **Tuning Iteration** | **Kp_linear** | **Kd_linear** | **Kp_angular** | **Kd_angular** | **Average Cross-Track Error** | **Smoothness Score**  |
|----------------------|---------------|---------------|----------------|----------------|-------------------------------|-----------------------|
| 1                    | 1.0           | 0.1           | 1.0            | 0.1            | 0.99                          | Poor                  |
| 2                    | 5.0           | 0.1           | 1.0            | 0.1            | 1.05                          | Poor                  |
| 3                    | 5.0           | 0.1           | 5.0            | 0.1            | 0.267                         | Good                  |
| 4                    | 7.0           | 0.1           | 5.0            | 0.1            | 0.28                          | Good                  |
| 5                    | 7.0           | 0.1           | 7.0            | 0.1            | 0.16                          | Excellent             |
| 6                    | 7.0           | 0.3           | 7.0            | 0.3            | 0.197                         | No good curves        |
| 7                    | 7.0           | 0.3           | 7.0            | 0.01           | 0.142                         | Excellent             |
| 8                    | 7.0           | 0.6           | 8.0            | 0.01           | 0.104 *                       | Excellent             |
| 9                    | 9.0           | 0.8           | 8.0            | 0.01           | 0.102                         | Excellent             |
| 10                   | 9.5           | 1.0           | 8.0            | 0.01           | 0.097                         | Good but unstable     |
| 11                   | 9.5           | 0.5           | 8.0            | 0.01           | 0.112                         | Excellent             |
| 12                   | 9.5           | 0.8           | 9.0            | 0.01           | 0.108                         | Excellent             |
| 13                   | 10.0          | 0.8           | 9.0            | 0.01           | 0.062 *                       | Good but unstable     |
| 14                   | 9.5           | 0.8           | 5.0            | 0.01           | 0.224                         | Smooth but high error |
| 15                   | 8.0           | 0.8           | 7.0            | 0.01           | 0.137                         | Excellent             |

## Parameter Values for Spacing = 1.0

    Kp_linear_: 7.0
    Kd_linear (Derivative Gain): 0.6
    Kp_angular: 8.0
    Kd_angular: 0.01

### Trajectory Plots
#### Plots showing:
1. Cross-track error over time.
2. Trajectory plot.
3. Velocity profiles.

#### Spacing = 1.0 <br>
![final_result](https://github.com/user-attachments/assets/760b4bf3-a366-4296-8388-fb7bfdd9992e) <br>

#### Result Plot
![plot_for_spacing_1](https://github.com/user-attachments/assets/04475087-b5fd-46c0-a8cb-27de0cdba330) <br>


## Performance Analysis:

- Cross-Track Error:
        Average: 0.104 rad/s
        Maximum: 0.229
        These values reflect good accuracy, with minimal deviation from the desired path.

- Smoothness:
        Score: 10/10 — The robot follows a smooth trajectory with no sudden changes in direction.

- Cornering Performance:
        Rating: Excellent — The robot navigates turns efficiently with high stability due to well-balanced gains.

- Overall Impression:
        Spacing of 1.0 is ideal for applications requiring faster coverage over large areas, but it may leave small gaps between adjacent paths.

## For Spacing = 0.4
| **Tuning Iteration** | **Kp_linear** | **Kd_linear** | **Kp_angular** | **Kd_angular** | **Average Cross-Track Error** | **Smoothness Score** |
|----------------------|---------------|---------------|----------------|----------------|-------------------------------|-----------------------|
| 1                    | 7.0           | 0.6           | 9.0            | 0.5            | 0.102                         | Poor                  |
| 2                    | 9.5           | 1.0           | 8.0            | 0.01           | 0.097                         | Good but unstable     |
| 3                    | 7.5           | 0.8           | 7.0            | 0.01           | 0.136                         | Poor                  |-
| 4                    | 9.5           | 0.8           | 9.0            | 0.01           | 0.81                          | Poor                  |
| 5                    | 10.0          | 0.8           | 9.0            | 0.01           | 0.069                         | Low error, unstable   |
| 6                    | 9.5           | 0.8           | 5.0            | 0.01           | 0.245                         | Smooth but high error |
| 7                    | 8.0           | 0.8           | 7.0            | 0.01           | 0.210                         | Poor                  |
| 8                    | 8.0           | 0.5           | 8.0            | 0.01           | 0.105 *                       | Excellent pattern     |

## Parameter Values for Spacing = 0.4

    Kp_linear: 8.0
    Kd_linear: 0.5
    Kp_angular: 8.0
    Kd_angular: 0.01

### Trajectory Plots
#### Plots showing:
1. Cross-track error over time.
2. Trajectory plot.
3. Velocity profiles.

#### Spacing = 0.4 <br>
![final_submission](https://github.com/user-attachments/assets/5439f215-298c-44cd-9615-380218a05c2f) <br>

#### Result Plot
![plot_for_spacing_2](https://github.com/user-attachments/assets/b0638cab-57d9-4292-b761-91d39e3e104d) <br>



## Performance Analysis:

- Cross-Track Error:
        Average: 0.105
        Maximum: 0.233
        Similar to spacing 1.0, with only minor deviations. 
        However, the denser path coverage compensates for these small inaccuracies.

- Smoothness:
        Score: 10/10 — Despite reduced linear velocity, the robot maintains a smooth and stable trajectory.

- Cornering Performance:
        Rating: Excellent — The robot handles tighter turns more effectively due to smaller spacing and lower angular velocity, enhancing precision.

- Overall Impression:
        Spacing of 0.4 ensures denser coverage, making it ideal for tasks that require detailed path-following and minimal gaps between paths.



## Results
### Performance Metrics
### Spacing = 1.0
| **Metric**              | **Value** |
|-------------------------|-----------|
| Average Cross-Track Error | 0.104   |
| Maximum Cross-Track Error | 0.229   |
| Smoothness Score         | 10/10     |
| Cornering Performance    | Excellent |

## Reduced Spacing Parameter to 0.4
To optimize the lawnmower-style survey and enhance coverage, we reduced the spacing between each pass to 0.4. This adjustment allows the robot to take more frequent passes, thereby covering a larger area in less time. By decreasing the spacing, the robot can achieve finer granularity in its path planning, leading to improved efficiency in mapping and survey tasks. Additionally, this modification ensures that the robot can navigate narrower spaces with greater precision, reducing gaps in the coverage and enhancing the overall performance of the lawnmower pattern. 

### Performance Metrics
### Spacing = 0.4
| **Metric**              | **Value** |
|-------------------------|-----------|
| Average Cross-Track Error | 0.105   |
| Maximum Cross-Track Error | 0.233   |
| Smoothness Score         | 10/10     |
| Cornering Performance    | Excellent |


# Analysis Summary: Choosing the Best Spacing (1.0 vs. 0.4)

To determine the best spacing between 1.0 and 0.4, we analyzed key performance metrics, including cross-track error (average and maximum), smoothness score, and cornering performance. 
Here is a detailed comparison and analysis for both spacing values:

### Comparison and Final Decision

| Metric                    | Spacing = 1.0   | Spacing = 0.4   |
|---------------------------|-----------------|-----------------|
| **Kp_linear** | 7.0             | 8.0             |
| **Kd_linear**   | 0.6             | 0.5             |
| **Kp_angular**        | 8.0             | 8.0             |
| **Kd_angular**       | 0.01             | 0.01            |
| **Average Cross-Track Error** | 0.104           | 0.105           |
| **Maximum Cross-Track Error** | 0.229           | 0.233           |
| **Smoothness**            | 10/10           | 10/10           |
| **Cornering Performance** | Excellent       | Excellent       |
| **Coverage Area** | Average       | Excellent       |

### Final Result plot
#### TurtleBot pattern <br>
![final_submission](https://github.com/user-attachments/assets/5439f215-298c-44cd-9615-380218a05c2f) <br>

#### Result Plot
![plot_for_spacing_2](https://github.com/user-attachments/assets/b0638cab-57d9-4292-b761-91d39e3e104d) <br>



*(Include additional generated plots in the `results/` directory)*


### Why Choose Spacing = 0.4:

Spacing 0.4 is preferred because it ensures denser coverage of the area, making it suitable for applications that require higher precision, such as detailed surveys, inspections, or mapping. 
Although the cross-track error is slightly higher compared to spacing 1.0, the overall performance remains smooth and stable, with excellent cornering abilities. 
The tighter spacing ensures no significant gaps are left between adjacent paths, which is critical for thorough coverage tasks. 
The chosen parameters effectively balance precision and stability, making spacing 0.4 the optimal choice for scenarios where comprehensive coverage is a priority.

### Challenges and Solutions
- **Challenge 1**: Large oscillations with low proportional gains.
  - **Solution**: Incrementally increased `Kp_linear` and `Kp_angular` values.
- **Challenge 2**: Instability at high speeds.
  - **Solution**: Limited maximum velocity using ROS2 parameter settings.
- **Challenge 3**: Poor cornering at high angular velocities.
  - **Solution**: Adjusted Kd_angular to reduce overshooting.


## Conclusion

### Objective of Parameter Tuning
The goal was to tune the proportional (`kp`) and derivative (`kd`) gains for both linear and angular velocities to optimize the controller's performance, ensuring smooth and accurate trajectory following for the lawnmower task.

---

### Tuning Parameters
#### 1. **Tuning `kp_linear` (Proportional Gain for Linear Velocity)**
- **Impact**:
  - Increasing `kp_linear` improved the speed at which the robot approached the desired trajectory but caused overshooting if set too high.
  - Decreasing it led to slower corrections, causing the robot to lag behind the trajectory.
- **Optimal Value**:
  - A moderate value was selected to balance response speed and minimize overshooting.
---
#### 2. **Tuning `kd_linear` (Derivative Gain for Linear Velocity)**
- **Impact**:
  - Higher `kd_linear` helped dampen oscillations caused by the proportional gain, ensuring a smoother approach to the trajectory.
  - Setting it too high, however, resulted in slower corrections and sluggish behavior.
- **Optimal Value**:
  - The value was adjusted to provide sufficient damping without compromising responsiveness.
---
#### 3. **Tuning `kp_angular` (Proportional Gain for Angular Velocity)**
- **Impact**:
  - Increasing `kp_angular` enhanced the robot’s ability to align quickly with the desired heading, reducing angular errors.
  - Excessive values caused the robot to oscillate around the desired heading.
  - Lower values led to slower corrections, resulting in larger heading deviations.
- **Optimal Value**:
  - A value that provided rapid and stable alignment was chosen after iterative testing.
---
#### 4. **Tuning `kd_angular` (Derivative Gain for Angular Velocity)**
- **Impact**:
  - Higher `kd_angular` reduced angular oscillations caused by `kp_angular` but, if set too high, made the robot’s angular corrections sluggish.
  - Lower values led to underdamped oscillations.
- **Optimal Value**:
  - A moderate value was chosen to complement `kp_angular` for stable angular corrections.
---
### Interdependencies Between Parameters
- Adjustments to one parameter often influenced the tuning of others. For example:
  - Increasing `kp_linear` required re-tuning `kd_linear` to dampen overshooting.
  - High `kp_angular` necessitated a well-tuned `kd_angular` to prevent oscillations.
- Iterative fine-tuning was necessary to achieve a balance between linear and angular responses.
---
### Testing and Validation
- The parameters were tested across various scenarios, including sharp turns, straight paths, and transitions between segments of the lawnmower pattern.
- The final values were validated by evaluating the robot's ability to maintain a smooth trajectory with minimal deviations and oscillations.
---
### Outcome
- The tuned controller achieved a robust performance, accurately following the lawnmower trajectory while minimizing overshooting, oscillations, and lag.
- This demonstrated the importance of carefully balancing proportional and derivative gains for both linear and angular velocity.
---
## Key Insights
- **Proportional gains** control the speed of error correction but can lead to instability if too high.
- **Derivative gains** are crucial for damping oscillations but must be carefully calibrated to avoid sluggish behavior.
- A **systematic, iterative tuning process** is essential to achieve a stable and responsive controller.
---

## Extra Credits:
## Custom ROS2 Message for Performance Metrics

This custom ROS2 message type is designed to publish detailed performance metrics for a robotic system. It includes fields for cross-track error, angular error, velocities, distance to the next waypoint, completion percentage, and other relevant metrics to monitor the robot's performance in real-time. The custom message is implemented in a ROS2 node to publish these metrics, providing valuable insights for path planning, navigation, and performance evaluation. This implementation demonstrates the creation of custom ROS2 interfaces and the efficient publishing of real-time data.

## Message Structure

The custom message `PerformanceMetrics` contains the following fields:

- **cross_track_error**: `Float64` – The deviation of the robot from its path.
- **angular_error**: `Float64` – The difference between the robot's current orientation and its desired orientation.
- **linear_velocity**: `Float64` – The current linear speed of the robot.
- **angular_velocity**: `Float64` – The current angular speed (rotation) of the robot.
- **distance_to_next_waypoint**: `Float64` – The distance to the next target position.
- **completion_percentage**: `Float64` – The percentage of task completion (e.g., how much of the path or task has been completed).
- **average_cross_track_error**: `Float64` – The average cross-track error over the course of the task.
- **max_cross_track_error**: `Float64` – The maximum observed cross-track error during the task.
- **min_cross_track_error**: `Float64` – The minimum observed cross-track error during the task.

## Usage

1. Define the message type in a `.msg` file.
2. Implement a publisher node to broadcast the message containing the performance metrics.
3. Subscribe to the message in other parts of the system to utilize these performance metrics.

## Message published for spacing = 1.0

cross_track_error: -0.002800464630126953 <br>
angular_error: -9.593147856229445e-07 <br>
linear_velocity: 0.18078167408025014 <br>
angular_velocity: -7.761139362750856e-06 <br>
distance_to_next_waypoint: 0.0 <br>
completion_percentage: 100.0 <br>
average_cross_track_error: 0.10406629622779656 <br>
max_cross_track_error: 0.22505712509155273 <br>
min_cross_track_error: 0.0011987686157226562 <br>


## Message published for spacing = 0.4


cross_track_error: -0.0011090755462617174 <br>
angular_error: 1.8753365605209127e-06 <br>
linear_velocity: 0.16777750319735663 <br>
angular_velocity: 1.525506179415553e-05 <br>
distance_to_next_waypoint: 0.0 <br>
completion_percentage: 100.0 <br>
average_cross_track_error: 0.1050251419030254 <br>
max_cross_track_error: 0.2342102050781243 <br>
min_cross_track_error: 0.0007004737854034992 <br>


## Implementation Details

- The custom message type is created using ROS2 interface tools, and is included in the appropriate ROS2 package.
- The publisher node will publish performance metrics at regular intervals or when significant changes in robot state occur.
- Metrics such as cross-track error, velocity, and distance to the next waypoint are updated in real-time based on the robot's sensors and odometry.

This custom message is essential for real-time monitoring, debugging, and performance analysis of robotic navigation tasks.

### References for Custom Message
- ROS2 documentation for controller tuning and visualization tools (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

## Conclusion
This project demonstrates the significance of PD tuning in achieving precise and efficient boustrophedon patterns. By systematically analyzing performance metrics and refining parameters, the implemented navigator achieved minimal cross-track error and smooth motion, showcasing practical applications for robotic coverage tasks.
