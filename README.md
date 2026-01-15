# Monte Carlo Localization (ROS2)

This repository implements **Monte Carlo Localization (MCL)** using a particle filter in **ROS2 (Kilted)**.
The system localizes a robot in a known landmark map using noisy odometry and landmark observations
provided by a simulated fake robot.

The fake robot simulation and message interfaces are provided as part of the lecture "Autonomous Mobile Robots" at TU Darmstadt by Prof. Kucner.
The Monte Carlo Localization algorithm itself is fully implemented in `src/mcl.cpp`. The corresponding ROS2 node is in `src/mcl_node.cpp`.

This repository is intended for **educational purposes**.

---

## Algorithm Overview

The Monte Carlo Localization follows the standard particle filter cycle:

1. Initialization: uniform distribution over map bounds

2. Motion update: odometry-based model with Gaussian noise

3. Measurement update: landmark likelihood in robot frame

4. Resampling: low-variance resampling when particle weights degenerate

5. Pose estimation: weighted mean of particle positions and circular mean for orientation

---

## Features

- Particle initialization uniformly over the map
- Odometry-based motion update with Gaussian noise
- Landmark-based measurement update (likelihood model)
- Log-likelihood stabilization using log-sum-exp
- Low-variance resampling
- Pose estimation using weighted mean (circular mean for orientation)
- Particle visualization via `PoseArray`
- Fully integrated ROS2 node architecture

---

## Requirements

- ROS2 **Kilted**
- `colcon`
- ROS2 packages:
  - `rclcpp`
  - `geometry_msgs`
  - `nav_msgs`
  - `sensor_msgs`
  - `tf2`
  - `tf2_ros`
  - `tf2_geometry_msgs`

---

## Run
Build the project with `colcon`:

```bash
colcon build
source install/setup.bash
```

Start the nodes in a cmd: 
```bash
ros2 launch fake_robot fake_robot.launch.py
```

For visualization, start RViz2 in a different cmd:
```bash
rviz2
```

---

## Important Parameters
| Parameter                  | Description                          |
| -------------------------- | ------------------------------------ |
| num_particles              | Number of particles used in MCL      |
| measurement_noise_variance | Landmark measurement noise           |
| robot_noise_variance_x     | Translation noise for noisy odometry |
| robot_noise_variance_theta | Rotation noise for noisy odometry    |
| observation_radius         | Landmark observation range           |

---

## Topics
#### Published by Fake Robot
- /robot_gt – Ground truth odometry
- /robot_noisy – Noisy odometry (used for motion update)
- /landmarks_gt – Ground truth landmarks (map)
- /landmarks_observed – Observed landmarks in robot frame

#### Published by MCL
- /mcl_pose – Estimated robot pose (PoseStamped)
- /particles – Particle set (PoseArray)

---

## Expected Behavior
- Particles initially uniformly distributed

- Particles converge toward the true robot pose when landmarks are observed

- Estimated pose remains closer to ground truth than noisy odometry

- Particle cloud expands when no observations are available and contracts when measurements are received
