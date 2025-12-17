---
id: week-12-inverse-kinematics
title: ' Inverse Kinematics'
sidebar_label: 'week 12 : Inverse Kinematics'
---


# Inverse Kinematics (IK)

Inverse Kinematics (IK) is a fundamental concept in robotics that allows a robot to **compute the required joint angles** to achieve a desired **end-effector position and orientation**. It is widely used in **humanoid robots, manipulators, and autonomous robotic arms** for precise motion control.

This lesson introduces **IK principles, mathematical formulations, ROS 2 integration, and practical applications**.

---

## Learning Objectives

By the end of this lesson, students will be able to:

- Understand what **Inverse Kinematics** is  
- Differentiate between **Forward Kinematics and Inverse Kinematics**  
- Solve IK for simple manipulators  
- Apply IK in **humanoid robot limbs**  
- Integrate IK with **ROS 2 Humble** for motion planning  
- Use simulation environments like **Gazebo, Unity, or Isaac Sim** to test IK  
- Understand challenges like **singularities and redundancy**  

---

## Prerequisites

- Completed Week 1-8: ROS 2, simulation, and sensor integration
- Completed Week 11: Humanoid Locomotion (understanding kinematics)
- Strong foundation in linear algebra and trigonometry
- Familiarity with coordinate transformations and matrices
- ROS 2 Humble with MoveIt motion planning framework
- Python 3.8+ with NumPy and SciPy for mathematical computations
- Access to simulation platform (Gazebo, Unity, or Isaac Sim)

---

## 1. What is Inverse Kinematics?

Inverse Kinematics is the process of:

- Finding **joint angles (θ1, θ2, …, θn)**  
- That result in a desired **end-effector position and orientation** (x, y, z, roll, pitch, yaw)  

✅ Example:
- Desired position: `(x=0.5, y=0.2, z=0.3)`  
- IK computes: `Joint1=30°, Joint2=45°, Joint3=10°`  
- Robot moves its arm to reach that position  

---

## 🔹 2. Forward Kinematics vs Inverse Kinematics

| Feature                | Forward Kinematics (FK)           | Inverse Kinematics (IK)           |
|------------------------|---------------------------------|----------------------------------|
| Input                  | Joint angles                    | End-effector pose               |
| Output                 | End-effector pose               | Joint angles                     |
| Computation            | Direct, simple                  | Requires solving equations      |
| Use                    | Simulation, animation            | Motion planning, manipulation  |

✅ IK is **more complex but essential for task-oriented motion**.

---

## 3. IK Mathematical Formulation

For a robot arm with `n` joints:

1. **Forward Kinematics:**
\[
T = f(\theta_1, \theta_2, ..., \theta_n)
\]

2. **Inverse Kinematics:**
\[
\theta_1, \theta_2, ..., \theta_n = f^{-1}(x, y, z, \phi, \theta, \psi)
\]

Where:
- \(T\) = end-effector pose  
- \(\theta_i\) = joint angles  
- \(x, y, z\) = position  
- \(\phi, \theta, \psi\) = orientation  

✅ IK often requires **numerical methods** due to non-linear equations.

---

## 👣 4. IK Solution Methods

### 1. Analytical Solutions
- Closed-form equations  
- Accurate and fast  
- Limited to simple manipulators  

### 2. Numerical Solutions
- Iterative methods like **Jacobian Inverse** or **Gradient Descent**  
- Works for complex robots  
- Handles redundancy and constraints  

### 3. Hybrid Approaches
- Combine analytical & numerical  
- Improves speed and reliability  

---

## 5. ROS 2 Integration

IK can be implemented in ROS 2 using:

- **MoveIt 2** → Motion planning and IK  
- **Joint trajectory controllers** → Execute IK solutions  
- **Robot description (URDF/Xacro)** → Defines robot kinematics  
- **Simulation environments** → Test IK in Gazebo / Unity / Isaac Sim  

✅ Enables **task-oriented motion for humanoid robots and manipulators**.

---

## 6. Practical Examples

### Example 1: 2-DOF Arm
- Desired end-effector position: `(x, y)`  
- Use analytical IK formulas to compute joint angles  
- Publish angles to ROS 2 topic to move arm  

### Example 2: 6-DOF Manipulator
- Use MoveIt 2 IK solver  
- Plan trajectory to pick-and-place an object  
- Simulate in Gazebo or Unity  

### Example 3: Humanoid Arm
- Apply IK for reaching tasks  
- Combine with balance control for stable motion  

---

## 7. Tools & Technologies Used

- ROS 2 Humble  
- MoveIt 2  
- Python / C++  
- URDF / Xacro Robot Models  
- Gazebo / Unity / Isaac Sim  
- NumPy / SciPy for numerical IK  
- OpenCV (optional, for vision-guided IK)  

---

## 🧪 8. Hands-On Exercises

### Exercise 1: Analytical IK for 2-DOF Planar Arm (Beginner)
**Objective**: Implement closed-form inverse kinematics solution for a simple 2-link planar robot arm

**System Requirements**:
- Python 3.8+
- NumPy for mathematical calculations
- Matplotlib for visualization

**Background**:
For a 2-DOF planar arm with link lengths `L1` and `L2`, given target position `(x, y)`:

**Implementation**:
```python
import numpy as np
import matplotlib.pyplot as plt

class PlanarArm2DOF:
    def __init__(self, L1=1.0, L2=1.0):
        """Initialize 2-DOF planar arm with link lengths"""
        self.L1 = L1
        self.L2 = L2

    def inverse_kinematics(self, x, y):
        """
        Compute joint angles for target position (x, y)
        Returns: (theta1, theta2) in radians, or None if unreachable
        """
        # Check if target is reachable
        distance = np.sqrt(x**2 + y**2)
        if distance > (self.L1 + self.L2) or distance < abs(self.L1 - self.L2):
            print(f"Target ({x:.2f}, {y:.2f}) is unreachable!")
            return None

        # Compute theta2 using law of cosines
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

        # Elbow-down solution
        theta2 = np.arccos(np.clip(cos_theta2, -1.0, 1.0))

        # Compute theta1
        k1 = self.L1 + self.L2 * np.cos(theta2)
        k2 = self.L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return theta1, theta2

    def forward_kinematics(self, theta1, theta2):
        """Compute end-effector position from joint angles"""
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return x, y

    def visualize(self, theta1, theta2, target_x, target_y):
        """Visualize arm configuration"""
        # Joint positions
        x0, y0 = 0, 0
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        x2, y2 = self.forward_kinematics(theta1, theta2)

        plt.figure(figsize=(8, 8))
        plt.plot([x0, x1, x2], [y0, y1, y2], 'b-o', linewidth=3, markersize=10, label='Arm')
        plt.plot(target_x, target_y, 'r*', markersize=20, label='Target')
        plt.plot(x2, y2, 'go', markersize=12, label='End-Effector')

        # Workspace circle
        circle = plt.Circle((0, 0), self.L1 + self.L2, fill=False, linestyle='--', color='gray')
        plt.gca().add_patch(circle)

        plt.xlim(-2.5, 2.5)
        plt.ylim(-2.5, 2.5)
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.title(f'2-DOF IK: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°')
        plt.show()

# Test the IK solver
arm = PlanarArm2DOF(L1=1.0, L2=1.0)

# Test targets
targets = [(1.5, 0.5), (0.8, 1.2), (1.8, 0.3)]

for target_x, target_y in targets:
    result = arm.inverse_kinematics(target_x, target_y)
    if result:
        theta1, theta2 = result
        print(f"\nTarget: ({target_x}, {target_y})")
        print(f"Solution: θ1 = {np.degrees(theta1):.2f}°, θ2 = {np.degrees(theta2):.2f}°")

        # Verify with forward kinematics
        x_fk, y_fk = arm.forward_kinematics(theta1, theta2)
        error = np.sqrt((target_x - x_fk)**2 + (target_y - y_fk)**2)
        print(f"FK Verification: ({x_fk:.4f}, {y_fk:.4f}), Error: {error:.6f}")

        arm.visualize(theta1, theta2, target_x, target_y)
```

**Tasks**:
1. Run the code and understand the analytical IK equations
2. Test with different link lengths (L1=1.5, L2=0.8)
3. Try unreachable targets and observe error handling
4. Implement "elbow-up" solution as alternative configuration
5. Add animation showing arm movement from current to target pose

---

### Exercise 2: Jacobian-Based IK for 3-DOF Arm (Intermediate)
**Objective**: Implement numerical IK using Jacobian transpose method

**Background**:
The Jacobian matrix `J` relates joint velocities to end-effector velocities:
```
v = J * θ_dot
```

For IK, we iteratively update joint angles to minimize position error.

**Implementation**:
```python
import numpy as np

class Arm3DOF:
    def __init__(self, L1=1.0, L2=0.8, L3=0.6):
        """3-DOF spatial arm"""
        self.L = [L1, L2, L3]

    def forward_kinematics(self, theta):
        """Compute end-effector position for joint angles theta=[θ1, θ2, θ3]"""
        theta = np.array(theta)
        x = (self.L[0] * np.cos(theta[0]) +
             self.L[1] * np.cos(theta[0] + theta[1]) +
             self.L[2] * np.cos(theta[0] + theta[1] + theta[2]))

        y = (self.L[0] * np.sin(theta[0]) +
             self.L[1] * np.sin(theta[0] + theta[1]) +
             self.L[2] * np.sin(theta[0] + theta[1] + theta[2]))

        z = 0.5  # Simplified Z-axis component
        return np.array([x, y, z])

    def compute_jacobian(self, theta):
        """Compute 3x3 Jacobian matrix"""
        theta = np.array(theta)

        # Partial derivatives (simplified for planar + Z component)
        J = np.zeros((3, 3))

        # dX/dθ
        J[0, 0] = -(self.L[0] * np.sin(theta[0]) +
                    self.L[1] * np.sin(theta[0] + theta[1]) +
                    self.L[2] * np.sin(theta[0] + theta[1] + theta[2]))
        J[0, 1] = -(self.L[1] * np.sin(theta[0] + theta[1]) +
                    self.L[2] * np.sin(theta[0] + theta[1] + theta[2]))
        J[0, 2] = -self.L[2] * np.sin(theta[0] + theta[1] + theta[2])

        # dY/dθ
        J[1, 0] = (self.L[0] * np.cos(theta[0]) +
                   self.L[1] * np.cos(theta[0] + theta[1]) +
                   self.L[2] * np.cos(theta[0] + theta[1] + theta[2]))
        J[1, 1] = (self.L[1] * np.cos(theta[0] + theta[1]) +
                   self.L[2] * np.cos(theta[0] + theta[1] + theta[2]))
        J[1, 2] = self.L[2] * np.cos(theta[0] + theta[1] + theta[2])

        # dZ/dθ (simplified)
        J[2, :] = [0.1, 0.05, 0.02]

        return J

    def inverse_kinematics_jacobian(self, target, theta_init=None,
                                    max_iter=1000, tolerance=1e-4, alpha=0.1):
        """
        Jacobian transpose IK solver

        Args:
            target: Target position [x, y, z]
            theta_init: Initial joint angles
            max_iter: Maximum iterations
            tolerance: Convergence threshold
            alpha: Step size
        """
        if theta_init is None:
            theta = np.zeros(3)
        else:
            theta = np.array(theta_init)

        target = np.array(target)

        for iteration in range(max_iter):
            # Compute current position
            current_pos = self.forward_kinematics(theta)

            # Error vector
            error = target - current_pos
            error_magnitude = np.linalg.norm(error)

            if error_magnitude < tolerance:
                print(f"Converged in {iteration} iterations")
                return theta, True

            # Compute Jacobian
            J = self.compute_jacobian(theta)

            # Jacobian transpose method
            delta_theta = alpha * J.T @ error

            # Update joint angles
            theta += delta_theta

            # Joint limits (optional)
            theta = np.clip(theta, -np.pi, np.pi)

            if iteration % 100 == 0:
                print(f"Iter {iteration}: Error = {error_magnitude:.6f}")

        print(f"Did not converge after {max_iter} iterations")
        return theta, False

# Test Jacobian-based IK
arm = Arm3DOF(L1=1.0, L2=0.8, L3=0.6)

# Target position
target = np.array([1.5, 0.8, 0.5])

print(f"Target position: {target}")
print("\nSolving IK using Jacobian Transpose...\n")

theta_solution, converged = arm.inverse_kinematics_jacobian(target, alpha=0.5)

if converged:
    print(f"\nSolution: θ = {np.degrees(theta_solution)} degrees")

    # Verify solution
    final_pos = arm.forward_kinematics(theta_solution)
    error = np.linalg.norm(target - final_pos)
    print(f"Final position: {final_pos}")
    print(f"Position error: {error:.6f}")
```

**Tasks**:
1. Run the Jacobian IK solver and analyze convergence
2. Experiment with different step sizes (alpha values)
3. Compare Jacobian transpose vs. Jacobian pseudo-inverse methods
4. Add joint velocity limits to the solver
5. Implement damped least squares (DLS) for singularity handling

---

### Exercise 3: ROS 2 + MoveIt 2 IK Integration (Intermediate)
**Objective**: Use MoveIt 2 framework for IK solving with ROS 2 Humble

**System Requirements**:
- ROS 2 Humble
- MoveIt 2 installed
- URDF robot model

**Setup**:
```bash
# Install MoveIt 2
sudo apt install ros-humble-moveit

# Create workspace
mkdir -p ~/ros2_ik_ws/src
cd ~/ros2_ik_ws/src

# Clone example robot (UR5 manipulator)
git clone https://github.com/ros-industrial/universal_robot.git -b ros2
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git

cd ~/ros2_ik_ws
colcon build
source install/setup.bash
```

**Python IK Client**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client_node')

        # Create service client for IK
        self.ik_service = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

        self.get_logger().info('IK service connected')

    def compute_ik(self, target_pose):
        """
        Compute IK for target pose

        Args:
            target_pose: geometry_msgs/PoseStamped

        Returns:
            Joint angles if solution found, None otherwise
        """
        # Create IK request
        ik_request = PositionIKRequest()
        ik_request.group_name = 'manipulator'  # MoveIt planning group
        ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped = target_pose
        ik_request.timeout.sec = 5

        # Call IK service
        request = GetPositionIK.Request()
        request.ik_request = ik_request

        future = self.ik_service.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.error_code.val == 1:  # SUCCESS
            self.get_logger().info('IK solution found!')
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position

            for name, pos in zip(joint_names, joint_positions):
                self.get_logger().info(f'{name}: {pos:.4f} rad')

            return joint_positions
        else:
            self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
            return None

def main():
    rclpy.init()
    node = IKClient()

    # Create target pose
    target = PoseStamped()
    target.header.frame_id = 'base_link'
    target.header.stamp = node.get_clock().now().to_msg()

    # Position
    target.pose.position.x = 0.4
    target.pose.position.y = 0.2
    target.pose.position.z = 0.5

    # Orientation (quaternion)
    target.pose.orientation.x = 0.0
    target.pose.orientation.y = 0.707
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.707

    # Compute IK
    joint_solution = node.compute_ik(target)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Tasks**:
1. Launch MoveIt 2 with demo robot: `ros2 launch moveit2_tutorials demo.launch.py`
2. Run the IK client script
3. Modify target positions and observe different solutions
4. Visualize IK solutions in RViz
5. Add collision checking to IK requests

---

### Exercise 4: Denavit-Hartenberg (DH) Parameters for IK (Advanced)
**Objective**: Define robot kinematics using DH convention and solve IK

**Background**:
DH parameters provide a standardized way to describe robot kinematics with 4 parameters per joint:
- `θ` (theta): Joint angle
- `d`: Link offset
- `a`: Link length
- `α` (alpha): Link twist

**Implementation**:
```python
import numpy as np
from scipy.optimize import least_squares

class DHRobot:
    def __init__(self, dh_params):
        """
        Initialize robot with DH parameters

        Args:
            dh_params: List of [theta, d, a, alpha] for each joint
        """
        self.dh_params = np.array(dh_params)
        self.n_joints = len(dh_params)

    def dh_transform(self, theta, d, a, alpha):
        """Compute transformation matrix from DH parameters"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        T = np.array([
            [ct,    -st*ca,  st*sa,   a*ct],
            [st,     ct*ca, -ct*sa,   a*st],
            [0,      sa,     ca,      d   ],
            [0,      0,      0,       1   ]
        ])
        return T

    def forward_kinematics(self, joint_angles):
        """
        Compute end-effector pose from joint angles

        Returns: 4x4 transformation matrix
        """
        T = np.eye(4)

        for i, theta in enumerate(joint_angles):
            # Get DH parameters for joint i
            _, d, a, alpha = self.dh_params[i]

            # Update theta (joint variable)
            T_i = self.dh_transform(theta, d, a, alpha)
            T = T @ T_i

        return T

    def get_position(self, joint_angles):
        """Extract position from transformation matrix"""
        T = self.forward_kinematics(joint_angles)
        return T[:3, 3]

    def inverse_kinematics_numerical(self, target_position, initial_guess=None):
        """
        Numerical IK solver using least squares optimization

        Args:
            target_position: Desired [x, y, z] position
            initial_guess: Initial joint angles
        """
        if initial_guess is None:
            initial_guess = np.zeros(self.n_joints)

        def objective(theta):
            """Objective function: position error"""
            current_pos = self.get_position(theta)
            return target_position - current_pos

        # Solve using least squares
        result = least_squares(
            objective,
            initial_guess,
            bounds=([-np.pi]*self.n_joints, [np.pi]*self.n_joints),
            ftol=1e-6,
            xtol=1e-6
        )

        if result.success:
            return result.x, True
        else:
            return result.x, False

# Example: 3-DOF robot arm (SCARA-like)
dh_params = [
    # [theta, d,   a,   alpha]
    [0,      0.1, 0.4, 0],      # Joint 1
    [0,      0,   0.3, 0],      # Joint 2
    [0,      0,   0.2, 0],      # Joint 3
]

robot = DHRobot(dh_params)

# Test IK
target = np.array([0.6, 0.3, 0.1])
print(f"Target position: {target}")

theta_solution, success = robot.inverse_kinematics_numerical(target)

if success:
    print(f"\nIK Solution: {np.degrees(theta_solution)} degrees")

    # Verify
    achieved_pos = robot.get_position(theta_solution)
    error = np.linalg.norm(target - achieved_pos)
    print(f"Achieved position: {achieved_pos}")
    print(f"Position error: {error:.6f} m")
else:
    print("IK solver failed to converge")
```

**Tasks**:
1. Define DH parameters for a 6-DOF robot (e.g., UR5, ABB IRB120)
2. Implement forward kinematics validation tests
3. Compare numerical IK convergence for different initial guesses
4. Add orientation constraints to IK objective function
5. Visualize workspace reachability using DH model

---

### Exercise 5: Humanoid Arm IK with ROS 2 Control (Advanced)
**Objective**: Implement IK for humanoid robot arm and integrate with ROS 2 controllers

**System Requirements**:
- ROS 2 Humble
- Gazebo Classic or Gazebo Sim
- Humanoid robot URDF (e.g., Poppy, Robonaut, or custom)

**Humanoid Arm IK Node**:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidArmIK(Node):
    def __init__(self):
        super().__init__('humanoid_arm_ik')

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/arm_controller/commands',
            10
        )

        # Subscribers
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/arm/target_pose',
            self.target_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Arm parameters (7-DOF humanoid arm)
        self.link_lengths = [0.3, 0.25, 0.05, 0.25, 0.05, 0.15, 0.08]
        self.current_joints = np.zeros(7)

        self.get_logger().info('Humanoid Arm IK node initialized')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        # Extract arm joints (assuming naming convention)
        arm_indices = [i for i, name in enumerate(msg.name)
                      if 'arm' in name.lower()]
        if len(arm_indices) >= 7:
            self.current_joints = np.array([msg.position[i] for i in arm_indices[:7]])

    def target_callback(self, msg):
        """Compute IK when new target received"""
        target_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        self.get_logger().info(f'New target: {target_pos}')

        # Solve IK
        joint_solution = self.solve_ik_ccd(target_pos, max_iter=100)

        if joint_solution is not None:
            # Publish joint commands
            cmd = Float64MultiArray()
            cmd.data = joint_solution.tolist()
            self.joint_cmd_pub.publish(cmd)

            self.get_logger().info(f'IK solution sent: {np.degrees(joint_solution)[:3]}...')
        else:
            self.get_logger().error('IK failed - target unreachable')

    def forward_kinematics_7dof(self, joints):
        """Compute end-effector position for 7-DOF arm"""
        x = 0.0
        y = 0.0
        z = 0.0

        # Simplified FK for demonstration
        cumulative_angle = 0.0
        for i, (angle, length) in enumerate(zip(joints, self.link_lengths)):
            cumulative_angle += angle
            if i % 2 == 0:
                x += length * np.cos(cumulative_angle)
                z += length * np.sin(cumulative_angle)
            else:
                y += length * np.sin(cumulative_angle)

        return np.array([x, y, z])

    def solve_ik_ccd(self, target, max_iter=100, tolerance=0.01):
        """
        Cyclic Coordinate Descent (CCD) IK solver
        Efficient for redundant manipulators like humanoid arms
        """
        joints = self.current_joints.copy()

        for iteration in range(max_iter):
            # Iterate through joints backwards (from end to base)
            for i in range(len(joints) - 1, -1, -1):
                # Current end-effector position
                current_pos = self.forward_kinematics_7dof(joints)

                # Check convergence
                error = np.linalg.norm(target - current_pos)
                if error < tolerance:
                    self.get_logger().info(f'CCD converged in {iteration} iterations')
                    return joints

                # Compute vectors
                ee_vector = current_pos - self.get_joint_position(joints, i)
                target_vector = target - self.get_joint_position(joints, i)

                # Compute rotation angle
                ee_norm = np.linalg.norm(ee_vector)
                target_norm = np.linalg.norm(target_vector)

                if ee_norm > 1e-6 and target_norm > 1e-6:
                    cos_angle = np.dot(ee_vector, target_vector) / (ee_norm * target_norm)
                    cos_angle = np.clip(cos_angle, -1.0, 1.0)
                    delta_angle = np.arccos(cos_angle)

                    # Determine rotation direction (simplified)
                    cross = np.cross(ee_vector, target_vector)
                    if cross[2] < 0:
                        delta_angle = -delta_angle

                    # Update joint angle with damping
                    joints[i] += 0.3 * delta_angle

                    # Apply joint limits
                    joints[i] = np.clip(joints[i], -np.pi, np.pi)

        self.get_logger().warn('CCD did not converge')
        return None

    def get_joint_position(self, joints, joint_idx):
        """Get position of specific joint"""
        pos = np.zeros(3)
        cumulative_angle = 0.0

        for i in range(joint_idx):
            cumulative_angle += joints[i]
            length = self.link_lengths[i]
            if i % 2 == 0:
                pos[0] += length * np.cos(cumulative_angle)
                pos[2] += length * np.sin(cumulative_angle)
            else:
                pos[1] += length * np.sin(cumulative_angle)

        return pos

def main():
    rclpy.init()
    node = HumanoidArmIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Launch File** (`humanoid_arm_ik.launch.py`):
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='humanoid_arm_ik',
            name='humanoid_arm_ik',
            output='screen'
        )
    ])
```

**Tasks**:
1. Launch Gazebo with humanoid robot model
2. Run the IK node and publish target poses
3. Implement self-collision avoidance in IK solver
4. Add joint velocity smoothing for natural motion
5. Test reaching tasks: picking objects, waving, pointing
6. Compare CCD vs. Jacobian-based methods for 7-DOF arms

---  

---

## 9. Knowledge Check Quiz

**Question 1**: What is the primary difference between forward kinematics (FK) and inverse kinematics (IK)?

- A) FK is faster to compute than IK
- B) FK computes end-effector pose from joint angles, while IK computes joint angles from desired end-effector pose ✓
- C) FK is used for simulation and IK is used for real robots
- D) FK requires numerical methods while IK uses analytical solutions

**Answer**: B. Forward kinematics takes joint angles as input and computes the resulting end-effector position and orientation. Inverse kinematics solves the reverse problem: given a desired end-effector pose, it computes the joint angles needed to achieve that pose. This makes IK essential for task-oriented robot motion.

---

**Question 2**: Which numerical method iteratively updates joint angles using the relationship between joint velocities and end-effector velocities?

- A) Cyclic Coordinate Descent (CCD)
- B) Jacobian-based methods ✓
- C) Genetic algorithms
- D) Analytical closed-form solutions

**Answer**: B. Jacobian-based IK methods use the Jacobian matrix J, which relates joint velocities (θ_dot) to end-effector velocities (v) through v = J * θ_dot. Common approaches include Jacobian transpose, Jacobian pseudo-inverse, and damped least squares (DLS) methods that iteratively minimize position error.

---

**Question 3**: What are Denavit-Hartenberg (DH) parameters used for in robotics?

- A) Controlling robot speed
- B) Standardized representation of robot link transformations ✓
- C) Measuring joint torques
- D) Calibrating sensors

**Answer**: B. DH parameters provide a systematic convention for describing the kinematics of serial manipulators using four parameters per joint: theta (joint angle), d (link offset), a (link length), and alpha (link twist). This standardization simplifies forward and inverse kinematics calculations.

---

**Question 4**: Which IK challenge occurs when a robot manipulator loses one or more degrees of freedom in certain configurations?

- A) Redundancy
- B) Workspace limits
- C) Singularities ✓
- D) Joint limits

**Answer**: C. Singularities are configurations where the Jacobian matrix becomes rank-deficient, causing the robot to lose mobility in certain directions. At singularities, small changes in joint angles produce little or no motion in the end-effector, making IK solutions unstable or impossible. Common examples include fully extended or folded arm configurations.

---

**Question 5**: In ROS 2, which framework provides IK solving capabilities with collision checking and trajectory planning?

- A) ros2_control
- B) Navigation2
- C) MoveIt 2 ✓
- D) tf2

**Answer**: C. MoveIt 2 is the primary motion planning framework for ROS 2, providing IK solvers (KDL, TRAC-IK, etc.), collision checking, trajectory planning, and manipulation capabilities. It integrates with robot descriptions (URDF) and provides service interfaces like `/compute_ik` for inverse kinematics queries.

---  

---

## 10. Glossary

- **IK (Inverse Kinematics):** Compute joint angles for desired end-effector pose  
- **FK (Forward Kinematics):** Compute end-effector pose from joint angles  
- **URDF/Xacro:** Robot description files  
- **MoveIt 2:** ROS 2 motion planning framework  
- **Jacobian:** Matrix relating joint velocities to end-effector velocities  
- **Redundancy:** Extra DOFs allowing multiple solutions  

---

## 11. Further Reading

### Official Documentation
1. **MoveIt 2 Documentation** - [https://moveit.ros.org/](https://moveit.ros.org/)
   - Comprehensive guides for motion planning, IK solvers, and ROS 2 integration

2. **KDL (Kinematics and Dynamics Library)** - [https://www.orocos.org/kdl.html](https://www.orocos.org/kdl.html)
   - C++ library for kinematic chains, used by MoveIt for IK solving

3. **ROS 2 Control Documentation** - [https://control.ros.org/](https://control.ros.org/)
   - Hardware interfaces and controllers for executing IK solutions

### Research Papers
4. **Buss, S. R. (2004).** "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods." *IEEE Journal of Robotics and Automation*
   - Comprehensive comparison of numerical IK techniques

5. **Beeson, P., & Ames, B. (2015).** "TRAC-IK: An Open-Source Library for Improved Solving of Generic Inverse Kinematics." *IEEE-RAS International Conference on Humanoid Robots*
   - Advanced IK solver combining analytical and numerical methods

6. **Aristidou, A., & Lasenby, J. (2011).** "FABRIK: A Fast, Iterative Solver for the Inverse Kinematics Problem." *Graphical Models*
   - Forward And Backward Reaching Inverse Kinematics algorithm

### Tutorials and Courses
7. **Modern Robotics: Mechanics, Planning, and Control** - Kevin Lynch & Frank Park
   - Chapter 6 covers comprehensive IK theory with DH parameters and screw theory

8. **MoveIt 2 Tutorials** - [https://moveit.picknik.ai/main/index.html](https://moveit.picknik.ai/main/index.html)
   - Step-by-step guides for setting up IK with ROS 2 robots

9. **Introduction to Robotics: Mechanics and Control** - John J. Craig
   - Classic textbook covering analytical IK solutions for common manipulator types

### Humanoid Robotics Applications
10. **Siciliano, B., et al. (2009).** "Robotics: Modelling, Planning and Control"
    - Advanced topics including redundancy resolution and optimization-based IK

11. **NASA Robonaut IK Papers** - Technical reports on humanoid arm control
    - Real-world applications of IK in space robotics

### Software Libraries and Tools
12. **PyBullet Inverse Kinematics** - [https://pybullet.org/](https://pybullet.org/)
    - Physics simulation with built-in IK solvers for testing

13. **RoboDK** - [https://robodk.com/](https://robodk.com/)
    - Industrial robot programming with visual IK debugging

14. **DART (Dynamic Animation and Robotics Toolkit)** - [https://dartsim.github.io/](https://dartsim.github.io/)
    - Physics engine with advanced IK capabilities

---  

---

## Lesson Summary

This lesson introduced **Inverse Kinematics**, covering its definition, mathematical formulation, solution methods, ROS 2 integration, and applications in **humanoid robots and manipulators**. Students learned how **IK allows robots to reach desired positions accurately** and how it is essential for **task-oriented robotic motion**.

---

📌 *This lesson prepares students for advanced motion planning, humanoid manipulation, and AI-driven robotics using ROS 2.*

---

**Version**: ROS 2 Humble  
**License**: CC BY-SA 4.0
