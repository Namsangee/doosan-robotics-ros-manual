.. _p3020_detailed_guide:

P3020 MoveIt 2 & ros2_control Integration Guide
===============================================

This guide explains how to integrate the **P3020** (5-DoF) with **ROS 2 Jazzy + MoveIt 2 + ros2_control**.
It also applies to other Doosan models with different DoF—adjust joint lists and limits as needed.

.. important::

   The P3020 includes a **dummy joint** (``joint_4``) that is **fixed** and **excluded** from control and planning.
   Make sure ``joint_4`` is omitted wherever joint lists are declared (controllers, MoveIt, initial positions, limits).

Quick Checklist
---------------

- [ ] In URDF/Xacro, set ``joint_4`` to ``type="fixed"`` (keep inertials on all links).
- [ ] Exclude ``joint_4`` from **command/state interfaces** (ros2_control Xacros).
- [ ] Exclude ``joint_4`` from **MoveIt** joint lists (controllers, initial positions, limits).
- [ ] Exclude ``joint_4`` from **runtime controller** joint lists (e.g., ``dsr_controller2.yaml``).

1. URDF Configuration
---------------------

**File**: ``dsr_description2/urdf/p3020.urdf``

- Use ``type="fixed"`` for passive/dummy joints.
- Provide **inertial parameters** for **all** links (including those connected to fixed joints).
- Active joints (``joint_1``, ``joint_2``, ``joint_3``, ``joint_5``, ``joint_6``) must be ``revolute`` or ``continuous``.

**Example**:

.. code-block:: xml

   <joint name="joint_4" type="fixed">
     <origin xyz="0 -0.89 -0.1731" rpy="1.5708 0 0"/>
     <parent link="link_3"/>
     <child link="link_4"/>
   </joint>

.. tip::

   Keep ``joint_4`` in the kinematic chain (as fixed) so link indexing and visuals remain consistent.

2. Xacro Modifications
----------------------

2.1 **ros2_control.xacro**

**File**: ``dsr_description2/config/ros2_control.xacro``

- Declare **command** and **state** interfaces **only** for controllable joints.
- Do **not** declare interfaces for fixed joints (e.g., **exclude** ``joint_4``).

.. code-block:: xml

   <!-- ... surrounding <ros2_control> omitted for brevity ... -->
   <joint name="joint_3">
     <command_interface name="position">
       <param name="min">${neg_pi}</param>
       <param name="max">${pi}</param>
     </command_interface>
     <command_interface name="velocity">
       <param name="min">-3.15</param>
       <param name="max">3.15</param>
     </command_interface>
     <state_interface name="position">
       <param name="initial_value">0.0</param>
     </state_interface>
     <state_interface name="velocity"/>
   </joint>

   <!-- Fixed joint_4 intentionally excluded from interfaces
   <joint name="joint_4">
     <command_interface name="position">
       <param name="min">${neg_double_pi}</param>
       <param name="max">${double_pi}</param>
     </command_interface>
     <command_interface name="velocity">
       <param name="min">-3.2</param>
       <param name="max">3.2</param>
     </command_interface>
     <state_interface name="position">
       <param name="initial_value">0.0</param>
     </state_interface>
     <state_interface name="velocity"/>
   </joint>
   -->
   <!-- ... -->

2.2 **urdf.xacro**

**File**: ``dsr_description2/urdf/p3020.urdf.xacro``

- Import any model macros (e.g., ``macro.p3020.white.xacro``).
- Ensure fixed joints (e.g., ``joint_4``) are defined with ``type="fixed"`` inside those macros.

.. code-block:: xml

   <!-- link_4 (Dummy Link for joint_4) -->
   <link name="link_4">
     <inertial>
       <mass value="6.057"/>
       <origin xyz="-2.2e-05 -0.12656 -0.30746"/>
       <inertia ixx="0.56389" ixy="5.1281e-06" ixz="-0.00030672"
                iyy="0.56341" iyz="3.2061e-05" izz="0.023517"/>
     </inertial>
   </link>

   <!-- Dummy joint_4 -->
   <joint name="joint_4" type="fixed">
     <parent link="link_3"/>
     <child link="link_4"/>
     <origin rpy="1.5708 0 0" xyz="0 -0.89 -0.1731"/>
     <axis xyz="0 0 1"/>
     <!-- <limit effort="163" lower="0" upper="0" velocity="0"/> -->
   </joint>

2.3 **dsr.ros2_control.xacro** (MoveIt mock control)

**File**: ``dsr_moveit_config_p3020/config/dsr.ros2_control.xacro``

- Uses ``mock_components/GenericSystem``.
- **Exclude** fixed joints from the joint list.
- Reads default positions from ``initial_positions.yaml``.

.. code-block:: xml

   <!-- ... within <ros2_control> -->
   <joint name="joint_3">
     <command_interface name="position"/>
     <state_interface name="position">
       <param name="initial_value">${initial_positions['joint_3']}</param>
     </state_interface>
     <state_interface name="velocity">
       <param name="initial_value">0.0</param>
     </state_interface>
   </joint>

   <!-- Fixed joint_4 intentionally excluded
   <joint name="joint_4">
     <command_interface name="position"/>
     <state_interface name="position">
       <param name="initial_value">${initial_positions['joint_4']}</param>
     </state_interface>
     <state_interface name="velocity">
       <param name="initial_value">0.0</param>
     </state_interface>
   </joint>
   -->
   <!-- ... -->

2.4 **MoveIt URDF Xacro**

**File**: ``dsr_moveit_config_p3020/config/p3020.urdf.xacro``

- Load macros from ``dsr_description2``.
- Include ``dsr.ros2_control.xacro`` to provide a simulated control interface.

3. YAML Configuration
---------------------

3.1 **initial_positions.yaml**

**File**: ``dsr_moveit_config_p3020/config/initial_positions.yaml``

- Remove or comment out fixed joints (``joint_4``):

.. code-block:: yaml

   initial_positions:
     joint_1: 0.0
     joint_2: 0.0
     joint_3: 0.0
     # joint_4: 0.0
     joint_5: 0.0
     joint_6: 0.0

3.2 **joint_limits.yaml**

**File**: ``dsr_moveit_config_p3020/config/joint_limits.yaml``

- Do **not** define limits for fixed joints (remove ``joint_4``). Example snippet:

.. code-block:: yaml

   joint_3:
     has_velocity_limits: true
     max_velocity: 3.67
     has_acceleration_limits: true
     max_acceleration: 0.734
     min_position: -2.0
     max_position: 2.0
   # joint_4:  # Fixed — do not include limits
   #   has_velocity_limits: true
   #   max_velocity: 6.98
   #   has_acceleration_limits: true
   #   max_acceleration: 1.396
   #   min_position: -3.14
   #   max_position: 3.14
   joint_5:
     has_velocity_limits: true
     max_velocity: 6.98
     has_acceleration_limits: true
     max_acceleration: 1.396
     min_position: -2.0
     max_position: 2.0

3.3 **moveit_controllers.yaml**

**File**: ``dsr_moveit_config_p3020/config/moveit_controllers.yaml``

- Define the controller with **movable joints only**:

.. code-block:: yaml

   trajectory_execution:
     allowed_execution_duration_scaling: 1.2
     allowed_goal_duration_margin: 0.5
     allowed_start_tolerance: 0.01
     trajectory_duration_monitoring: true

   moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

   moveit_simple_controller_manager:
     controller_names:
       - dsr_moveit_controller

     dsr_moveit_controller:
       action_ns: follow_joint_trajectory
       type: FollowJointTrajectory
       default: true
       joints:
         - joint_1
         - joint_2
         - joint_3
         # - joint_4   # fixed — exclude
         - joint_5
         - joint_6

3.4 **ros2_controllers.yaml**

**File**: ``dsr_moveit_config_p3020/config/ros2_controllers.yaml``

- Declare a ``JointTrajectoryController`` and ``joint_state_broadcaster``.
- The ``joints`` list must **exclude** fixed joints.

.. code-block:: yaml

   controller_manager:
     ros__parameters:
       update_rate: 100  # Hz

       dsr_moveit_controller:
         type: joint_trajectory_controller/JointTrajectoryController

       joint_state_broadcaster:
         type: joint_state_broadcaster/JointStateBroadcaster

   dsr_moveit_controller:
     ros__parameters:
       command_interfaces:
         - position
       state_interfaces:
         - position
         - velocity
       joints:
         - joint_1
         - joint_2
         - joint_3
         # - joint_4   # fixed — exclude
         - joint_5
         - joint_6

3.5 **dsr_controller2.yaml**

**File**: ``dsr_controller2/config/dsr_controller2.yaml``

- Often generated dynamically by launch.
- If editing manually, **exclude** fixed joints from the controller’s ``joints:`` list.

.. code-block:: yaml

   joints:
     - joint_1
     - joint_2
     - joint_3
     # - joint_4   # fixed — exclude
     - joint_5
     - joint_6
