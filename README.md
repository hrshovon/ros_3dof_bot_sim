# Assignment 
The repository contains all the necessary documents and codes to execute and operate the assignment project. 

Prerequisites:
1. Ubuntu 20.04 and newer.
2. ROS humble. It may also be compatible with other versions
3. ROS moveit and rviz

# Setup instructions
I did not use any exclusive libraries. Just standard ***ros noetic*** libraries and PCL and Boost.
Assuming that ros noetic is already installed. 
One thing that might require installation is ***catkin_tools***. Simply run 
```
sudo apt-get install python3-catkin-tools
```
Steps to get things up and running:
1. create workspace anywhere. For example, at home folder, run ```mkdir -p ~/ros_ws/src```
2. ```cd ros_ws/src```
3. ```git clone ```
4. ```cd .../```
5. ```catkin build```
6. ```source devel/setup.bash```
After that, everything should be good to go.

# Creating a xacro file to describe the robot
Here is also the xacro file for optimization

<?xml version="1.0"?>
<!--
This is a very simple 3 DOF robot arm.
Three  movements are:
1. revolve around z axis (-90 to +90 degrees)
2. Upper arm and lower arm joints(0 to 90 degrees)
Each link has been colored differently to understand its movement.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="first_arm">
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>
  <material name="yellow">
    <color rgba="0 1 1 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 0.5"/>
  </material>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>

  <xacro:property name="radius_base" value="0.3" />
  <xacro:property name="height_base" value="0.2" />
  
  <xacro:property name="radius_yaw" value="0.1" />
  <xacro:property name="height_yaw" value="0.2" />

  <xacro:property name="radius_arm" value="0.1" />
  <xacro:property name="height_arm" value="0.6" />
  
  <xacro:property name="radius_wrist" value="0.05" />
  <xacro:property name="height_wrist" value="0.2" />
  
  <xacro:property name="radius_finger_fixed" value="0.02" />
  <xacro:property name="height_finger_fixed" value="0.1" />


  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="3.06" />
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>
  </link>


  <link name="base_bot">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="3.06" />
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>
    <visual>
      <geometry>
        <cylinder length="${height_base}" radius="${radius_base}" />
      </geometry>
      <origin xyz="0 0 0.1"/>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${height_base}" radius="${radius_base}" />
      </geometry>
      <origin xyz="0 0 0.1"/>
    </collision>
  </link>
  
  <joint name="base_to_base_bot" type="fixed">
    <parent link="base_link"/>
    <child link="base_bot"/>
    <origin xyz="0 0 0"/>
  </joint>
  
  <link name="base_yaw">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="3.06" />
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>
    <visual>
      <geometry>
        <cylinder length="${height_yaw}" radius="${radius_yaw}" />
      </geometry>
      <origin xyz="0 0 0.1"/>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${height_yaw}" radius="${radius_yaw}" />
      </geometry>
      <origin xyz="0 0 0.1"/>
    </collision>
  </link>
  
  <joint name="joint_0" type="revolute">
    <dynamics damping="1.0"/>
    <parent link="base_bot"/>
    <child link="base_yaw"/>
    <origin xyz="0 0 0.2"/>
    <limit effort="0.0" lower="-1.57" upper="1.57" velocity="1.0"/>
    <axis xyz="0 0 1"/>
  </joint>
  
  <link name="upper_arm">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="3.06" />
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>
    <visual>
      <geometry>
        <cylinder length="${height_arm}" radius="${radius_arm}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0.1 0 0.3"/>
      <material name="yellow"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${height_arm}" radius="${radius_arm}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0.1 0 0.3"/>
    </collision>
  </link>
  
  <joint name="joint_1" type="revolute">
    <dynamics damping="1.0"/>
    <parent link="base_yaw"/>
    <child link="upper_arm"/>
    <origin xyz="0.1 0 0.2"/>
    <limit effort="0.0" lower="0" upper="1.57" velocity="2.0"/>
    <axis xyz="1 0 0"/>
  </joint>

  <link name="arm">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="3.06" />
      <inertia ixx="0.3" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.3" />
    </inertial>
    <visual>
      <geometry>
        <cylinder length="${height_arm}" radius="${radius_arm}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0 0 0.3"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${height_arm}" radius="${radius_arm}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0 0 0.3"/>
    </collision>
  </link>
  
  <joint name="joint_2" type="revolute">
    <dynamics damping="1.0"/>
    <parent link="upper_arm"/>
    <child link="arm"/>
    <origin xyz="-0.1 0 0.6"/>
    <limit effort="0.0" lower="0" upper="1.57" velocity="2.0"/>
    <axis xyz="1 0 0"/>
  </joint>
  
  <link name="finger_low">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.01"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <geometry>
        <cylinder length="${height_finger_fixed}" radius="${radius_finger_fixed}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0 0 0.02"/>
      <material name="yellow"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="${height_finger_fixed}" radius="${radius_finger_fixed}" />
      </geometry>
      <origin rpy = "0 0 0" xyz="0 0 0.6"/>
    </collision>
  </link>
  
  <joint name="joint_fixed_finger" type="fixed">
    <parent link="arm"/>
    <child link="finger_low"/>
    <origin xyz="0 0 0.6"/>
  </joint>
  
</robot>
# Testing
This following command was used to create the cisualization
ros2 launch urdf_tutorial display.launch.py model:=src/bot_resources/urdf/bot_arm.xacro
The resulting ARM is this:


```python
%%HTML
<iframe width="560" height="315" src="https://www.youtube.com/embed/eFMjEBVJh6E"></iframe>
```


<iframe width="560" height="315" src="https://www.youtube.com/embed/eFMjEBVJh6E"></iframe>



# Checking urdf limits
Out set limits
1. Joint0 = -90 to +90 or -1.57 radians to 1.57 radians.
2. Joint1 = Joint2 = 0 to +90 or 0 to 1.57 radians.
The video above demonstrates that

# Create ROS workspace
Initially ROS2 humble was chosen as environment. But after facing several issues, ROS1 notetic was finally selected as work environment

# Integration with moveit
Now to integrate with moveit, we need to generate a moveit configuration. For that, moveit setup assistant was used to generate the config files.

# Custom ikfast kinematics solver
For kinematics solver, An IKFast kinematics solver was created. Since our robot is 3 dof, Direction3D was selected as solver after some experimentation. But for some strange issues, it does not work with pose and only works with Joint States. May be because there is not enough degrees of freedom or some end effector setting issues

# Language of choice
Given the number of available tutorials and api support, CPP was chosen as language.

# Part 1: Checking basic movement, self collision, ik and fk

There is one single file for that. This file does the following:
1. Prints various information regarding the robot movegroup.
2. Sets joint goals and moves the robot.
3. Calculates ik and fk for random movement.
4. Adds object to the scene

To run this part, at first, load the demo environment

```
roslaunch bot_config demo.launch
```
From a second terminal, run
```
rosrun basic_motion basic_motion_obstacle_ikfk
```
This should cover part 1.

***Note***: Self collision check worked but collision check with other bodies did not. But when collision objects are added, the system does take those objects into consideration when planning path

# Part 2 and 3: Adding camera and getting point cloud

For this task, there are several options. But for simplicity, I simply followed ***moveit_tutotrials*** method of using a bag file to get a simulated pointcloud. Some parameters were adjusted to align the pointcloud with our custom robot. 
The resulting work generates an octomap and that automatically allows for obstacle avoidance.

To run it, at first load the appropriate environment:
```
roslaunch bagpub bagpub_simple.launch
```
This will load up the environment with octomap point cloud.

![image.png](attachment:5273b8a2-54b2-4c44-92d2-e5b3fb250ec5.png)

Now we can open a second terminal and load our simple motion planner which will try to move the robot arm over an obstacle. There are two planned motions. First one just puts the arm in a convenient place. Second one is where path planning happens.
command to run it
```
rosrun simple_movement_test simple_motion
```

# Part 4 Simplify point cloud to mesh
Here, I was honestly a bit unsure on what to do. Becasue of encountering various errors and issues in the beginning, I could not start this part in time. So for mesh, I tried something similar. Approximating an obstacle as a cylinder using SACSsegmentation. 
I probably would have tried voxblox or PCL based implementation for this. Currently I do not have any experience regarding mesh topics in ROS.

To run the approximation. Run
```
roslaunch bagpub bagpub_mesh.launch
```
That will approximate a cylinder from point cloud data. 
After that, please run
```
roslaunch bagpub bagpub_simple.launch
```
