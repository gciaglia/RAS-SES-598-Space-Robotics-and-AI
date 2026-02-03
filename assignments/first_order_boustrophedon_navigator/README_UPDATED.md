# SES598 HW1: First-Order Boustrophedon Navigator

In this assignment, the goal is to understand the provided code in ROS2 with Turtlesim, and refactor and/or tune the navigator to implement a precise lawnmower survey (a boustrophedon pattern). The current code will do a lawmower pattern, which is not a uniform lawnmower survey. The goal is to determined the gains for the controller that produces an optimized lawnmower survey with minimal cross track error [1].


## Background
Boustrophedon patterns (from Greek: "ox-turning", like an ox drawing a plow) are fundamental coverage survey trajectories useful in space exploration and Earth observation. These patterns are useful for:

- **Space Exploration**: Rovers could use boustrophedon patterns to systematically survey areas of interest, ensuring complete coverage when searching for geological samples or mapping terrain. However, due to energy constraints, informative paths are usually optimized, and this results in paths that are sparser than complete coverage sampling, and may still produce high-accuracy reconstructions. 
  
- **Earth Observation**: Aerial vehicles employ these patterns for:
  - Agricultural monitoring and precision farming
  - Search and rescue operations
  - Environmental mapping and monitoring
  - Geological or archaeological surveys
  
- **Ocean Exploration**: Autonomous underwater vehicles (AUVs) use boustrophedon patterns to:
  - Map the ocean floor
  - Search for shipwrecks or aircraft debris
  - Monitor marine ecosystems
  
The efficiency and accuracy of these surveys depend heavily on the robot's ability to follow the prescribed path with minimal deviation (cross-track error). This assignment simulates these real-world challenges in a 2D environment using a first-order dynamical system (the turtlesim robot).

## Objective
Tune a PD controller to make a first-order system execute the most precise boustrophedon pattern possible. The goal is to minimize the cross-track error while maintaining smooth motion.

## The Challenge

### 1. Controller Tuning (60 points)
#### 1.1 Original Tuning Paramters

```python
# Controller parameters to tune
self.Kp_linear = 1.0   # Proportional gain for linear velocity
self.Kd_linear = 0.1   # Derivative gain for linear velocity
self.Kp_angular = 1.0  # Proportional gain for angular velocity
self.Kd_angular = 0.1  # Derivative gain for angular velocity
```

### 2. Pattern Parameters (20 points)
Optimize the boustrophedon pattern parameters:
```python
# Pattern parameters to tune
self.spacing = 1.0     # Spacing between lines
```

**Please note** that it was decided in class the week of January 26th that this was a parameter that did not need to be tuned.

### 3. Analysis and Documentation (20 points)

#### 3.1 Tuning Methodology

##### 3.1.1 Final Parameter Values and Justification

#### 3.2 Performance Plots and Metrics

##### 3.2.1 Cross-track error over time

<img width="800" alt="Cross Track Error" src="cross_track_error.png" />

##### 3.2.2 Trajectory plot

<img width="800" alt="Trajectory Plot" src="trajectory.png" />

##### 3.2.3 Velocity profiles

<img width="800" alt="Velocity Profiles" src="velocity_profiles.png" />

##### 3.2.4 Performance metrics and analysis

#### 3.3 Challenges encountered and solutions

#### 3.4 Comparison of different parameter sets



## 4 References: 
Jnaneshwar Das Arizona State University. [Course Repoository Source] (https://github.com/DREAMS-lab/ses598-space-robotics-and-ai-2026/tree/main/assignments/first_order_boustrophedon_navigator)



### 4.1 Setup Instructions

### Repository Setup
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork (outside of ros2_ws):
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

3. Create a symlink to the assignment in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
```

### Building and Running
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
```

2. Launch the demo:
```bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```

3. Monitor performance:
```bash
# View cross-track error as a number
ros2 topic echo /cross_track_error

# Or view detailed statistics in the launch terminal
```

4. Visualize trajectory and performance:
```bash
ros2 run rqt_plot rqt_plot
```
Add these topics:
- /turtle1/pose/x
- /turtle1/pose/y
- /turtle1/cmd_vel/linear/x
- /turtle1/cmd_vel/angular/z
- /cross_track_error

<img width="1385" height="542" alt="image" src="https://github.com/user-attachments/assets/54e15cb9-f60a-48df-b337-2c9c4d54da77" />


## Tips for Success
- Start with low gains and increase gradually
- Test one parameter at a time
- Pay attention to both straight-line tracking and cornering
- Use rqt_plot to visualize performance in real-time
- Consider the trade-off between speed and accuracy

