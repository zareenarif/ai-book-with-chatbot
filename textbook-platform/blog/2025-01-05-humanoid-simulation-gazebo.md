---
slug: humanoid-robot-simulation-gazebo
title: Simulating Humanoid Robots with Gazebo and ROS 2
authors: [physical-ai-team]
tags: [gazebo, simulation, humanoid, ros2, tutorial]
---

Before deploying code to expensive physical robots, it's crucial to test in simulation. This tutorial shows you how to set up a humanoid robot simulation using Gazebo and ROS 2.

<!-- truncate -->

## Why Simulate Humanoid Robots?

Simulating humanoid robots offers several advantages:

- **Safety**: Test dangerous scenarios without risk to hardware or people
- **Cost**: No need for expensive physical robots during development
- **Speed**: Iterate quickly without hardware setup time
- **Reproducibility**: Exact conditions can be replicated
- **Scale**: Test with multiple robots simultaneously

### Challenges of Humanoid Simulation

Humanoid robots are complex to simulate due to:
- **High degrees of freedom** (20-30+ joints)
- **Complex dynamics** (balance, contact forces)
- **Computational demands** (physics simulation)
- **Sim-to-real gap** (transferring learned behaviors)

## Setting Up Gazebo with ROS 2

### Prerequisites

```bash
# Install Gazebo Fortress (recommended for ROS 2 Humble)
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control

# Verify installation
gz sim --version
```

### Install Example Humanoid Models

```bash
# Install example robot descriptions
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro

# Clone humanoid description package (example)
cd ~/ros2_ws/src
git clone https://github.com/robotis-git/DARwIn-OP.git
```

## Creating a Simple Humanoid URDF

Let's create a simplified biped robot for learning:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Left leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_hip" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 0.1 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>

  <!-- Add right leg, arms, head similarly... -->
</robot>
```

## Launch Configuration

Create `humanoid_gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get URDF file
    urdf_file = PathJoinSubstitution([
        FindPackageShare('my_humanoid_description'),
        'urdf',
        'simple_humanoid.urdf'
    ])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_humanoid',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Controlling the Humanoid

### Joint Position Control

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers for each joint group
        self.left_leg_pub = self.create_publisher(
            Float64MultiArray,
            '/left_leg_controller/commands',
            10
        )
        self.right_leg_pub = self.create_publisher(
            Float64MultiArray,
            '/right_leg_controller/commands',
            10
        )

        # Timer for walking motion
        self.timer = self.create_timer(0.01, self.control_loop)
        self.time = 0.0

        self.get_logger().info('Humanoid controller started')

    def control_loop(self):
        """Simple sinusoidal walking motion"""
        import math

        # Left leg joints (hip, knee, ankle)
        left_leg_cmd = Float64MultiArray()
        left_leg_cmd.data = [
            0.2 * math.sin(2 * math.pi * 0.5 * self.time),  # Hip
            0.3 * math.sin(2 * math.pi * 0.5 * self.time + math.pi/2),  # Knee
            0.1 * math.sin(2 * math.pi * 0.5 * self.time)   # Ankle
        ]

        # Right leg (opposite phase)
        right_leg_cmd = Float64MultiArray()
        right_leg_cmd.data = [
            0.2 * math.sin(2 * math.pi * 0.5 * self.time + math.pi),
            0.3 * math.sin(2 * math.pi * 0.5 * self.time + 3*math.pi/2),
            0.1 * math.sin(2 * math.pi * 0.5 * self.time + math.pi)
        ]

        self.left_leg_pub.publish(left_leg_cmd)
        self.right_leg_pub.publish(right_leg_cmd)

        self.time += 0.01


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
```

## Advanced Simulation Features

### 1. Sensor Integration

Add sensors to your URDF:

```xml
<!-- IMU Sensor -->
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>100.0</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <remapping>~/out:=data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>

<!-- Camera -->
<gazebo reference="head_link">
  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### 2. Contact Sensors (Foot Pressure)

```xml
<gazebo reference="left_foot">
  <sensor name="left_foot_contact" type="contact">
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <plugin name="left_foot_plugin" filename="libgazebo_ros_bumper.so">
      <ros>
        <namespace>/contacts</namespace>
        <remapping>bumper_states:=left_foot</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

### 3. Physics Parameters

Tune physics for realistic humanoid behavior:

```xml
<gazebo>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <gravity>0 0 -9.81</gravity>
  </physics>
</gazebo>
```

## Balance Control Example

Simple PD controller for maintaining balance:

```python
class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher for ankle torques
        self.torque_pub = self.create_publisher(
            Float64MultiArray,
            '/ankle_torque_controller/commands',
            10
        )

        # PD gains
        self.kp = 50.0
        self.kd = 10.0
        self.target_angle = 0.0

    def imu_callback(self, msg):
        # Extract pitch from IMU orientation
        from transforms3d.euler import quat2euler
        roll, pitch, yaw = quat2euler([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        # Compute error and derivative
        error = self.target_angle - pitch
        angular_velocity = msg.angular_velocity.y

        # PD control
        torque = self.kp * error - self.kd * angular_velocity

        # Publish ankle torques
        cmd = Float64MultiArray()
        cmd.data = [torque, torque]  # Both ankles
        self.torque_pub.publish(cmd)
```

## Tips for Successful Humanoid Simulation

1. **Start Simple**: Begin with a 2D biped before full 3D humanoid
2. **Validate Physics**: Check joint limits, masses, and inertias
3. **Tune Step Size**: Smaller time steps = more accurate but slower
4. **Monitor Performance**: Use `gz stats` to check real-time factor
5. **Log Data**: Record simulation data for analysis
6. **Gradual Complexity**: Add features incrementally (joints → sensors → control)

## Common Issues and Solutions

### Robot Falls Through Ground
- Check collision geometries
- Verify contact parameters
- Ensure proper surface friction

### Unstable Simulation
- Reduce physics time step
- Increase solver iterations
- Check joint limits and damping

### Slow Performance
- Simplify collision meshes
- Use lower sensor update rates
- Reduce physics accuracy if acceptable

## Next Steps

- Explore **Isaac Sim** for GPU-accelerated simulation
- Learn **MuJoCo** for efficient contact dynamics
- Study **whole-body controllers** for complex motions
- Implement **reinforcement learning** for locomotion

Check out our lessons on:
- [Gazebo Simulation Basics](/docs/book/chapters/module-2-robotics-control/week-05-gazebo-simulation)
- [NVIDIA Isaac Sim](/docs/book/chapters/module-3-simulation/week-07-isaac-sim)
- [Humanoid Locomotion](/docs/book/chapters/module-4-advanced/week-11-locomotion)

## Resources

- [Gazebo Documentation](https://gazebosim.org/docs)
- [ROS 2 Control](https://control.ros.org/master/index.html)
- [Humanoid Robotics Handbook](https://link.springer.com/book/10.1007/978-94-007-6046-2)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)

Happy simulating!
