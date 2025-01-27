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

## Results
### Performance Metrics
# Spacing = 1.0
| **Metric**              | **Value** |
|-------------------------|-----------|
| Average Cross-Track Error | 0.104   |
| Maximum Cross-Track Error | 0.229   |
| Smoothness Score         | 10/10     |
| Cornering Performance    | Excellent |
# Summary
The average cross-track error of 0.104 indicates good accuracy, meaning the robot stays close to the desired path. The maximum cross-track error of 0.229 shows occasional deviations, though the overall performance remains reliable. The smoothness score of 10/10 confirms the robot's path is smooth with minimal abrupt changes in direction. Cornering performance being excellent reflects the robot’s ability to handle turns efficiently without significant instability or erratic movement. This indicates that the chosen parameters balance responsiveness and stability well for the task at hand.


## Reduced Spacing Parameter to 0.4
To optimize the lawnmower-style survey and enhance coverage, we reduced the spacing between each pass to 0.5. This adjustment allows the robot to take more frequent passes, thereby covering a larger area in less time. By decreasing the spacing, the robot can achieve finer granularity in its path planning, leading to improved efficiency in mapping and survey tasks. Additionally, this modification ensures that the robot can navigate narrower spaces with greater precision, reducing gaps in the coverage and enhancing the overall performance of the lawnmower pattern. 

### Performance Metrics
# Spacing = 0.4
| **Metric**              | **Value** |
|-------------------------|-----------|
| Average Cross-Track Error | 0.105   |
| Maximum Cross-Track Error | 0.233   |
| Smoothness Score         | 10/10     |
| Cornering Performance    | Excellent |

### Trajectory Plots
Plots showing:
1. Cross-track error over time.
2. Trajectory plot.
3. Velocity profiles.

![Final Result](assignments/first_order_boustrophedon_navigator/results/final_result.png)

*(Include additional generated plots in the `results/` directory)*

## Parameter Tuning Analysis
# For Spacing = 1.0
| **Tuning Iteration** | **Kp_linear** | **Kd_linear** | **Kp_angular** | **Kd_angular** | **Average Cross-Track Error** | **Smoothness Score** |
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


# For Spacing = 0.4
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


## Challenges and Solutions
- **Challenge 1**: Large oscillations with low proportional gains.
  - **Solution**: Incrementally increased `Kp_linear` and `Kp_angular` values.
- **Challenge 2**: Over-corrections at corners.
  - **Solution**: Adjusted derivative gains to dampen oscillations.


## Analysis Summary:

The parameter tuning analysis for the boustrophedon pattern demonstrates a clear progression in performance as the tuning parameters evolve. The results suggest that, as the proportional (Kp) and derivative (Kd) values for both linear and angular control increase, the system improves in terms of reducing the average cross-track error and enhancing smoothness. Specifically, iteration 9 achieved the best average cross-track error of 0.102 and a smoothness score of 10/10, though it exhibited some instability towards the end of the tuning process. The optimal configuration found in iteration 7, with Kp_linear = 7.0, Kd_linear = 0.3, Kp_angular = 7.0, and Kd_angular = 0.01, provided the lowest average cross-track error of 0.142 and excellent smoothness. However, higher Kp values, such as those in iterations 13 and 14, led to high error despite smoothness improvements.

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

## Implementation Details

- The custom message type is created using ROS2 interface tools, and is included in the appropriate ROS2 package.
- The publisher node will publish performance metrics at regular intervals or when significant changes in robot state occur.
- Metrics such as cross-track error, velocity, and distance to the next waypoint are updated in real-time based on the robot's sensors and odometry.

This custom message is essential for real-time monitoring, debugging, and performance analysis of robotic navigation tasks.


## Conclusion
This project demonstrates the significance of PD tuning in achieving precise and efficient boustrophedon patterns. By systematically analyzing performance metrics and refining parameters, the implemented navigator achieved minimal cross-track error and smooth motion, showcasing practical applications for robotic coverage tasks.

## References
- ROS2 documentation for controller tuning and visualization tools (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)