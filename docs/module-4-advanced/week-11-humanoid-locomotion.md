---
id: week-11-humanoid-locomotion
title: ' Humanoid Locomotion'
sidebar_label: 'week 11 : Humanoid Locomotion'
---

# Humanoid Locomotion

Humanoid Locomotion is the study and implementation of **walking, running, and balance control** in humanoid robots. It combines **kinematics, dynamics, sensor feedback, and control algorithms** to enable robots to move safely and efficiently in complex environments.

This lesson introduces the **fundamentals, control strategies, and ROS 2 integration** for humanoid locomotion.

---

## Learning Objectives

By the end of this lesson, students will be able to:

- Understand the **mechanics of humanoid locomotion**  
- Describe **bipedal walking patterns and gait cycles**  
- Apply **inverse kinematics and dynamics** for humanoid movement  
- Use sensors for **balance and stability**  
- Implement locomotion control using **ROS 2 Humble**  
- Simulate humanoid walking in **Gazebo, Unity, or Isaac Sim**  
- Analyze trade-offs in humanoid locomotion design  

---

## Prerequisites

- Completed Week 1-8: ROS 2, simulation, and sensor integration
- Completed Week 2: Humanoid Fundamentals (DOF, actuators)
- Understanding of kinematics and dynamics principles
- Familiarity with control theory and feedback systems
- ROS 2 Humble with MoveIt or similar motion planning library
- Access to simulation platform (Gazebo, Unity, or Isaac Sim)
- Python 3.8+ and NumPy for kinematic calculations

---

## 1. Introduction to Humanoid Locomotion

Humanoid locomotion involves:

- **Bipedal walking**: Alternating leg motion for forward movement  
- **Running**: Faster motion with flight phase  
- **Turning & pivoting**: Changing direction smoothly  
- **Balance maintenance**: Preventing falls  

✅ Challenges include **stability, energy efficiency, and terrain adaptation**.

---

## 2. Gait Cycle

A gait cycle is the **sequence of movements during walking**:

1. **Stance phase**: Foot in contact with ground  
2. **Swing phase**: Foot moves forward  
3. **Double support phase**: Both feet in contact  

✅ Understanding gait cycles is essential for **stable humanoid walking**.

---

## 🔹 3. Kinematics and Dynamics

### Forward Kinematics:
- Compute **end-effector (foot) position** from joint angles  

### Inverse Kinematics:
- Compute **joint angles** to achieve desired foot position  

### Dynamics:
- Compute **forces, torques, and motion stability**  
- Consider **gravity, momentum, and ground reaction forces**

✅ Combined kinematics & dynamics are essential for **walking and balance**.

---

## 👣 4. Balance and Stability

### Key Concepts:
- **Center of Mass (CoM)**  
- **Zero Moment Point (ZMP)**: Point where robot does not tip  
- **Foot placement strategies**  

### Sensors Used:
- IMU (Inertial Measurement Unit)  
- Force sensors in feet  
- Joint encoders  

✅ Continuous feedback from sensors allows **real-time correction**.

---

## 5. Humanoid Locomotion Control Strategies

### 1. Open-Loop Control
- Predefined joint trajectories  
- No feedback from sensors  
- Simple but less robust  

### 2. Closed-Loop Control
- Uses sensor feedback (IMU, force sensors)  
- Adjusts motion in real-time  
- More stable and adaptable  

### 3. Model Predictive Control (MPC)
- Predicts future states using **robot model**  
- Optimizes stability and energy  
- Common in advanced humanoid robots  

---

## 6. ROS 2 Integration

Humanoid locomotion can be implemented in ROS 2 using:

- **Joint state publishers** → Control leg joints  
- **IMU and foot sensors** → Feedback for balance  
- **Motion planning nodes** → Generate walking trajectories  
- **Gazebo/Unity/Isaac Sim** → Simulate locomotion  

✅ Enables **testing and tuning locomotion safely in simulation**.

---

## 7. Practical Examples

### Example 1: Forward Walking
- Publish joint angles via ROS 2  
- Use IMU feedback to correct balance  
- Simulate walking in Gazebo  

### Example 2: Turning
- Adjust foot trajectory  
- Maintain CoM stability  
- Execute turn smoothly  

### Example 3: Obstacle Negotiation
- Use vision/LiDAR to detect obstacles  
- Adjust gait dynamically  
- Maintain stability  

---

## 8. Humanoid Locomotion in AI & Robotics

- Integration with **AI planners** for dynamic environments  
- Training **reinforcement learning agents** for walking  
- Combine with **VLA models** for perception-driven navigation  
- Enables **humanoids to perform complex tasks safely**

---

## 🧪 9. Hands-On Exercises

### Exercise 1: Calculate ZMP for Static Balance (Beginner)
**Objective**: Understand Zero Moment Point calculation for a simple standing humanoid

**Steps**:
1. Create a Python script to calculate ZMP position
2. Define robot mass distribution and foot contact points
3. Compute ZMP and verify stability condition

```python
#!/usr/bin/env python3
import numpy as np

class ZMPCalculator:
    def __init__(self):
        self.gravity = 9.81  # m/s^2

    def calculate_zmp(self, masses, positions, accelerations):
        """
        Calculate ZMP position for a humanoid robot
        masses: list of body segment masses (kg)
        positions: list of [x, y, z] positions of each mass (m)
        accelerations: list of [ax, ay, az] accelerations (m/s^2)
        """
        total_force_z = 0
        moment_x = 0
        moment_y = 0

        for m, pos, acc in zip(masses, positions, accelerations):
            # Total vertical force
            fz = m * (self.gravity + acc[2])
            total_force_z += fz

            # Moments around x and y axes
            moment_x += m * (pos[2] * acc[0] + pos[0] * (self.gravity + acc[2]))
            moment_y += m * (pos[2] * acc[1] + pos[1] * (self.gravity + acc[2]))

        # ZMP coordinates
        zmp_x = moment_y / total_force_z
        zmp_y = -moment_x / total_force_z

        return np.array([zmp_x, zmp_y, 0])

    def check_stability(self, zmp, support_polygon):
        """
        Check if ZMP is inside support polygon
        support_polygon: list of [x, y] foot boundary points
        """
        from matplotlib.path import Path
        polygon = Path(support_polygon)
        return polygon.contains_point([zmp[0], zmp[1]])

# Example usage
calc = ZMPCalculator()

# Simplified humanoid with 3 segments
masses = [5.0, 30.0, 5.0]  # left leg, torso, right leg (kg)
positions = [[-0.1, 0, 0.5], [0, 0, 0.9], [0.1, 0, 0.5]]  # x, y, z (m)
accelerations = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]  # static case

zmp = calc.calculate_zmp(masses, positions, accelerations)
print(f"ZMP Position: x={zmp[0]:.3f}m, y={zmp[1]:.3f}m")

# Define foot support polygon (rectangular stance)
support_polygon = [[-0.15, -0.1], [0.15, -0.1], [0.15, 0.1], [-0.15, 0.1]]
stable = calc.check_stability(zmp, support_polygon)
print(f"Robot is {'STABLE' if stable else 'UNSTABLE'}")
```

**Expected Output**:
```
ZMP Position: x=0.000m, y=0.000m
Robot is STABLE
```

**Challenge**: Modify the code to simulate forward leaning (add positive x-acceleration) and observe ZMP shift.

---

### Exercise 2: Implement Basic Gait Pattern Generator (Intermediate)
**Objective**: Generate foot trajectories for bipedal walking using ROS 2

**Tasks**:
1. Create a ROS 2 node that publishes joint trajectories
2. Implement a simple walking gait pattern
3. Visualize the trajectory in RViz

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class GaitPatternGenerator(Node):
    def __init__(self):
        super().__init__('gait_pattern_generator')

        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/humanoid/joint_trajectory',
            10
        )

        # Gait parameters
        self.step_length = 0.2  # meters
        self.step_height = 0.05  # meters
        self.gait_period = 2.0  # seconds per step cycle
        self.current_time = 0.0

        # Timer for trajectory generation
        self.timer = self.create_timer(0.05, self.generate_trajectory)

        self.get_logger().info('Gait Pattern Generator initialized')

    def calculate_swing_foot_trajectory(self, t, phase):
        """
        Calculate foot position during swing phase
        t: normalized time (0 to 1) within swing phase
        phase: 'left' or 'right'
        """
        # Swing trajectory using polynomial
        x = self.step_length * t
        z = self.step_height * np.sin(np.pi * t)
        y = 0.1 if phase == 'left' else -0.1  # foot separation

        return [x, y, z]

    def inverse_kinematics_leg(self, foot_position):
        """
        Simplified 3-DOF leg IK (hip, knee, ankle)
        Returns joint angles in radians
        """
        x, y, z = foot_position

        # Simplified planar IK for demonstration
        l1 = 0.4  # upper leg length
        l2 = 0.4  # lower leg length

        # Distance to target
        r = np.sqrt(x**2 + z**2)

        # Knee angle (law of cosines)
        knee_angle = -np.arccos(
            (l1**2 + l2**2 - r**2) / (2 * l1 * l2)
        )

        # Hip angle
        alpha = np.arctan2(x, z)
        beta = np.arccos((r**2 + l1**2 - l2**2) / (2 * r * l1))
        hip_angle = alpha - beta

        # Ankle angle (keep foot flat)
        ankle_angle = -(hip_angle + knee_angle)

        return [hip_angle, knee_angle, ankle_angle]

    def generate_trajectory(self):
        """Generate and publish walking trajectory"""
        self.current_time += 0.05

        # Normalized gait cycle time (0 to 1)
        t_cycle = (self.current_time % self.gait_period) / self.gait_period

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]

        point = JointTrajectoryPoint()

        # Simple gait: alternate swing phases
        if t_cycle < 0.5:
            # Left leg swing
            left_foot_pos = self.calculate_swing_foot_trajectory(t_cycle * 2, 'left')
            left_angles = self.inverse_kinematics_leg(left_foot_pos)

            # Right leg stance (simplified)
            right_angles = [0.0, 0.0, 0.0]
        else:
            # Right leg swing
            right_foot_pos = self.calculate_swing_foot_trajectory((t_cycle - 0.5) * 2, 'right')
            right_angles = self.inverse_kinematics_leg(right_foot_pos)

            # Left leg stance
            left_angles = [0.0, 0.0, 0.0]

        point.positions = left_angles + right_angles
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms

        msg.points.append(point)
        self.trajectory_pub.publish(msg)

        if int(self.current_time) % 2 == 0 and abs(self.current_time - int(self.current_time)) < 0.05:
            self.get_logger().info(f'Step cycle: {t_cycle:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GaitPatternGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Behavior**:
- Node publishes joint trajectories at 20Hz
- Alternating swing phases for left and right legs
- Smooth transitions between stance and swing

**Testing**:
```bash
# Terminal 1: Run the gait generator
ros2 run my_humanoid_pkg gait_generator

# Terminal 2: Monitor trajectories
ros2 topic echo /humanoid/joint_trajectory

# Terminal 3: Visualize in RViz
ros2 run rviz2 rviz2
```

---

### Exercise 3: IMU-Based Balance Controller (Advanced)
**Objective**: Implement closed-loop balance control using IMU feedback

**Tasks**:
1. Create a ROS 2 node that reads IMU data
2. Implement a PID controller for balance correction
3. Adjust ankle torques to maintain upright posture

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu',
            self.imu_callback,
            10
        )

        # Publishers
        self.ankle_torque_pub = self.create_publisher(
            Float64MultiArray,
            '/humanoid/ankle_torque_cmd',
            10
        )

        # PID gains for balance control
        self.kp_pitch = 50.0  # Proportional gain
        self.ki_pitch = 2.0   # Integral gain
        self.kd_pitch = 10.0  # Derivative gain

        self.kp_roll = 50.0
        self.ki_roll = 2.0
        self.kd_roll = 10.0

        # State variables
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.integral_pitch = 0.0
        self.integral_roll = 0.0
        self.dt = 0.01  # 100Hz control loop

        # Target orientation (upright)
        self.target_pitch = 0.0
        self.target_roll = 0.0

        # Control loop timer
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Balance Controller initialized')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def imu_callback(self, msg):
        """Process IMU orientation data"""
        # Extract orientation from quaternion
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        self.current_pitch = pitch
        self.current_roll = roll

    def control_loop(self):
        """PID control loop for balance maintenance"""
        # Pitch control (sagittal plane - forward/backward balance)
        pitch_error = self.target_pitch - self.current_pitch
        self.integral_pitch += pitch_error * self.dt
        derivative_pitch = (self.current_pitch - self.prev_pitch) / self.dt

        ankle_pitch_torque = (
            self.kp_pitch * pitch_error +
            self.ki_pitch * self.integral_pitch -
            self.kd_pitch * derivative_pitch
        )

        # Roll control (frontal plane - side-to-side balance)
        roll_error = self.target_roll - self.current_roll
        self.integral_roll += roll_error * self.dt
        derivative_roll = (self.current_roll - self.prev_roll) / self.dt

        ankle_roll_torque = (
            self.kp_roll * roll_error +
            self.ki_roll * self.integral_roll -
            self.kd_roll * derivative_roll
        )

        # Anti-windup: limit integral term
        max_integral = 5.0
        self.integral_pitch = np.clip(self.integral_pitch, -max_integral, max_integral)
        self.integral_roll = np.clip(self.integral_roll, -max_integral, max_integral)

        # Torque limits
        max_torque = 100.0  # Nm
        ankle_pitch_torque = np.clip(ankle_pitch_torque, -max_torque, max_torque)
        ankle_roll_torque = np.clip(ankle_roll_torque, -max_torque, max_torque)

        # Publish ankle torque commands
        torque_msg = Float64MultiArray()
        torque_msg.data = [
            ankle_pitch_torque,  # Left ankle pitch
            ankle_roll_torque,   # Left ankle roll
            ankle_pitch_torque,  # Right ankle pitch
            ankle_roll_torque    # Right ankle roll
        ]
        self.ankle_torque_pub.publish(torque_msg)

        # Update previous values
        self.prev_pitch = self.current_pitch
        self.prev_roll = self.current_roll

        # Logging
        if abs(pitch_error) > 0.05 or abs(roll_error) > 0.05:
            self.get_logger().warn(
                f'Balance correction - Pitch: {np.degrees(pitch_error):.2f}°, '
                f'Roll: {np.degrees(roll_error):.2f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Testing in Gazebo**:
1. Launch humanoid simulation with IMU sensor
2. Run balance controller node
3. Apply external disturbances (push robot in Gazebo)
4. Observe corrective ankle torques maintaining balance

**Expected Behavior**:
- Robot maintains upright posture despite disturbances
- Ankle torques adjust in real-time based on IMU feedback
- Smooth recovery to vertical orientation

---

### Exercise 4: Walking Pattern with ZMP Constraint (Advanced)
**Objective**: Generate walking trajectories that maintain ZMP within support polygon

**Tasks**:
1. Implement preview control for ZMP tracking
2. Generate center of mass trajectory
3. Compute foot placement for stable walking

```python
#!/usr/bin/env python3
import numpy as np
from scipy.linalg import solve_discrete_are

class ZMPWalkingController:
    def __init__(self):
        self.dt = 0.01  # 10ms control cycle
        self.com_height = 0.8  # meters
        self.gravity = 9.81

        # Preview control horizon
        self.preview_steps = 100

        # State-space model for inverted pendulum (simplified)
        self.setup_state_space_model()
        self.compute_preview_gains()

    def setup_state_space_model(self):
        """Linear inverted pendulum model"""
        omega = np.sqrt(self.gravity / self.com_height)

        # State: [x, x_dot, x_ddot]^T
        # Control: zmp position
        self.A = np.array([
            [1, self.dt, self.dt**2/2],
            [0, 1, self.dt],
            [0, omega**2, 1]
        ])

        self.B = np.array([[0], [0], [-omega**2]])
        self.C = np.array([[1, 0, -1/(omega**2)]])

    def compute_preview_gains(self):
        """Compute optimal preview control gains"""
        # LQR weights
        Q = np.array([[1.0]])  # ZMP tracking weight
        R = np.array([[1e-6]]) # Control effort weight

        # Solve discrete-time algebraic Riccati equation
        P = solve_discrete_are(self.A, self.B,
                               self.C.T @ Q @ self.C, R)

        # Compute feedback gain
        self.K = np.linalg.inv(R + self.B.T @ P @ self.B) @ self.B.T @ P @ self.A

        self.get_logger().info(f'Preview gains computed: K = {self.K}')

    def generate_zmp_reference(self, num_steps, step_duration):
        """Generate ZMP reference trajectory for walking"""
        zmp_ref = []

        for step in range(num_steps):
            t_step = step * step_duration

            # Alternate foot placement
            if step % 2 == 0:
                zmp_x = step * 0.2  # 20cm steps
                zmp_y = 0.1  # Left foot
            else:
                zmp_x = step * 0.2
                zmp_y = -0.1  # Right foot

            # Hold ZMP at each foot for step duration
            for _ in range(int(step_duration / self.dt)):
                zmp_ref.append([zmp_x, zmp_y])

        return np.array(zmp_ref)

    def compute_com_trajectory(self, zmp_reference):
        """Compute CoM trajectory using preview control"""
        num_samples = len(zmp_reference)
        com_trajectory = np.zeros((num_samples, 3))

        # Initial state: [position, velocity, acceleration]
        state = np.array([[0.0], [0.0], [0.0]])

        for i in range(num_samples):
            # Preview window
            preview_end = min(i + self.preview_steps, num_samples)
            zmp_preview = zmp_reference[i:preview_end, 0]  # x-direction

            # Control input (desired ZMP)
            u = zmp_preview[0] if len(zmp_preview) > 0 else 0.0

            # State update
            state = self.A @ state + self.B * u

            # Store CoM state
            com_trajectory[i] = state.flatten()

        return com_trajectory

# Example usage
controller = ZMPWalkingController()
zmp_ref = controller.generate_zmp_reference(num_steps=10, step_duration=1.0)
com_traj = controller.compute_com_trajectory(zmp_ref)

print(f"Generated {len(com_traj)} CoM trajectory points")
print(f"Final CoM position: {com_traj[-1, 0]:.3f}m")
```

**Challenge**: Integrate this with Exercise 2's gait generator to create a complete walking system with ZMP stability guarantees.

---

### Exercise 5: Simulation in Gazebo (Practical Integration)
**Objective**: Test all components in a complete simulation environment

**Steps**:
1. Launch humanoid robot in Gazebo with physics simulation
2. Run all control nodes (gait generator, balance controller)
3. Monitor ZMP, CoM, and stability metrics
4. Record and analyze walking performance

```bash
# Launch Gazebo simulation
ros2 launch humanoid_simulation humanoid_gazebo.launch.py

# Start gait pattern generator
ros2 run humanoid_control gait_generator

# Start balance controller
ros2 run humanoid_control balance_controller

# Monitor robot state
ros2 topic echo /humanoid/joint_states
ros2 topic echo /humanoid/imu

# Visualize in RViz
ros2 launch humanoid_visualization rviz_display.launch.py
```

**Performance Metrics to Monitor**:
- Step length and frequency
- ZMP trajectory vs. reference
- Body tilt angle (from IMU)
- Energy consumption (joint torques)
- Walking speed and stability

**Expected Outcomes**:
- Stable walking for 10+ steps
- ZMP remains within support polygon
- Body tilt < 5 degrees during walking
- Smooth transitions between steps  

---

## 10. Knowledge Check Quiz

**Question 1**: What is the Zero Moment Point (ZMP) in humanoid locomotion?

- A) The point where the robot's total mass is concentrated
- B) The point on the ground where the sum of all moments equals zero ✓
- C) The center of the robot's foot during stance phase
- D) The highest point of the foot trajectory during swing phase

**Answer**: B. The ZMP is the point on the ground where the net moment from gravity and inertial forces equals zero. For stable walking, the ZMP must remain inside the support polygon (the convex hull of contact points with the ground). This is a fundamental stability criterion for humanoid locomotion.

---

**Question 2**: During a complete gait cycle, which phase involves the foot moving forward through the air?

- A) Stance phase
- B) Double support phase
- C) Swing phase ✓
- D) Flight phase

**Answer**: C. The swing phase is when the foot is not in contact with the ground and moves forward. During stance phase, the foot supports the robot's weight. Double support occurs when both feet are on the ground simultaneously during the transition between steps. Flight phase only occurs during running when both feet are off the ground.

---

**Question 3**: Which control strategy provides the most adaptability for humanoid walking on uneven terrain?

- A) Open-loop control with predefined trajectories
- B) Closed-loop control with sensor feedback ✓
- C) Pure feedforward control
- D) Static balance control only

**Answer**: B. Closed-loop control uses real-time sensor feedback (IMU, force sensors, joint encoders) to continuously adjust the robot's motion in response to disturbances and terrain variations. Open-loop control executes predefined trajectories without feedback, making it less robust to unexpected conditions. Model Predictive Control (MPC) is an advanced form of closed-loop control particularly effective for humanoid locomotion.

---

**Question 4**: What is the primary purpose of inverse kinematics in humanoid locomotion?

- A) Calculate foot position from joint angles
- B) Compute required joint angles to achieve desired foot position ✓
- C) Measure forces at the foot contact points
- D) Determine the robot's center of mass

**Answer**: B. Inverse kinematics (IK) solves for the joint angles needed to place the end-effector (foot) at a desired position and orientation. This is essential for gait planning where we specify foot trajectories, and IK converts them to motor commands. Forward kinematics does the opposite—computing foot position from known joint angles.

---

**Question 5**: Which sensor combination is most critical for real-time balance control in humanoid robots?

- A) Camera and LiDAR
- B) IMU and force sensors ✓
- C) GPS and magnetometer
- D) Ultrasonic sensors and encoders

**Answer**: B. The IMU (Inertial Measurement Unit) provides orientation and angular velocity data crucial for detecting body tilt, while force sensors in the feet measure ground reaction forces and help compute the ZMP. Together, these enable real-time balance corrections through ankle and hip strategies. Joint encoders are also important but secondary to IMU and force sensors for balance control.  

---

## 11. Glossary

- **Gait Cycle:** Sequence of movements during walking  
- **CoM:** Center of Mass  
- **ZMP:** Zero Moment Point  
- **IMU:** Inertial Measurement Unit  
- **MPC:** Model Predictive Control  
- **Joint Encoder:** Measures joint angles  

---

## 12. Further Reading

### Official Documentation and Tutorials
1. **ROS 2 Control Documentation** - [https://control.ros.org/](https://control.ros.org/)
   - Complete guide to robot control interfaces, controllers, and hardware abstraction in ROS 2

2. **MoveIt 2 Motion Planning** - [https://moveit.ros.org/](https://moveit.ros.org/)
   - Motion planning framework compatible with humanoid robots, includes inverse kinematics solvers

3. **Gazebo Humanoid Robot Tutorials** - [https://classic.gazebosim.org/tutorials?tut=drcsim_install](https://classic.gazebosim.org/tutorials?tut=drcsim_install)
   - Simulation setup for humanoid robots including ATLAS, Valkyrie, and custom models

### Foundational Research Papers
4. **Kajita, S., et al. (2003).** "Biped walking pattern generation by using preview control of zero-moment point." *IEEE International Conference on Robotics and Automation (ICRA)*, Vol. 2, pp. 1620-1626. DOI: 10.1109/ROBOT.2003.1241826
   - Seminal paper on ZMP-based walking using model predictive control, foundation for modern humanoid locomotion

5. **Vukobratović, M. & Borovac, B. (2004).** "Zero-moment point—Thirty five years of its life." *International Journal of Humanoid Robotics*, 1(1), pp. 157-173. DOI: 10.1142/S0219843604000083
   - Comprehensive review of ZMP theory and applications in biped locomotion

6. **Pratt, J., et al. (2006).** "Capture point: A step toward humanoid push recovery." *IEEE-RAS International Conference on Humanoid Robots*, pp. 200-207. DOI: 10.1109/ICHR.2006.321385
   - Introduction to capture point concept for balance recovery and push resistance

### Advanced Control Techniques
7. **Wieber, P.-B. (2006).** "Trajectory free linear model predictive control for stable walking in the presence of strong perturbations." *IEEE-RAS International Conference on Humanoid Robots*, pp. 137-142. DOI: 10.1109/ICHR.2006.321375
   - Model predictive control without predefined trajectories for robust walking

8. **Englsberger, J., et al. (2015).** "Three-dimensional bipedal walking control based on divergent component of motion." *IEEE Transactions on Robotics*, 31(2), pp. 355-368. DOI: 10.1109/TRO.2015.2405592
   - Divergent Component of Motion (DCM) control for 3D humanoid walking

### Inverse Kinematics and Dynamics
9. **Siciliano, B., et al. (2009).** "Robotics: Modelling, Planning and Control." *Springer*, ISBN: 978-1-84628-641-4
   - Comprehensive textbook covering kinematics, dynamics, and control (Chapters 3-4 on humanoid systems)

10. **Featherstone, R. (2014).** "Rigid Body Dynamics Algorithms." *Springer*, ISBN: 978-1-4899-7560-7
    - Essential algorithms for computing robot dynamics efficiently, critical for real-time control

### Reinforcement Learning Approaches
11. **Heess, N., et al. (2017).** "Emergence of locomotion behaviours in rich environments." *arXiv preprint arXiv:1707.02286*
    - Deep reinforcement learning for complex locomotion behaviors in simulated humanoids

12. **Peng, X. B., et al. (2018).** "DeepMimic: Example-guided deep reinforcement learning of physics-based character skills." *ACM Transactions on Graphics*, 37(4), Article 143. DOI: 10.1145/3197517.3201311
    - Learning humanoid locomotion from motion capture data using deep RL

### Simulation Platforms
13. **NVIDIA Isaac Sim Documentation** - [https://docs.omniverse.nvidia.com/isaacsim/](https://docs.omniverse.nvidia.com/isaacsim/)
    - High-fidelity physics simulation for humanoid robots with GPU acceleration

14. **PyBullet Humanoid Examples** - [https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_envs](https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_envs)
    - Open-source physics simulation with humanoid walking examples and RL integration

### Video Lectures and Courses
15. **MIT OpenCourseWare: Underactuated Robotics** - [https://underactuated.mit.edu/](https://underactuated.mit.edu/)
    - Course by Russ Tedrake covering dynamics, control, and planning for legged robots (Chapters 4-5 on walking)

16. **Stanford CS223A: Introduction to Robotics** - Available on YouTube
    - Lectures covering kinematics, dynamics, and trajectory planning applicable to humanoid systems  

---

## Lesson Summary

This lesson introduced **humanoid locomotion**, including gait cycles, kinematics & dynamics, balance control, and ROS 2-based locomotion strategies. Students learned how **sensors, control algorithms, and simulations** enable humanoid robots to walk, turn, and maintain stability safely.

---

📌 *This lesson prepares students for advanced humanoid robotics, AI-driven locomotion, and simulation-based testing using ROS 2.*

---

**Version**: ROS 2 Humble  
**License**: CC BY-SA 4.0
