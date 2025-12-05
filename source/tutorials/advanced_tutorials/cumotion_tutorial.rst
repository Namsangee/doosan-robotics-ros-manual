.. _cumotion_tutorial:

cuMotion Integration(Jetson thor)
==================================

Overview
--------

**cuMotion** provides GPU-accelerated motion planning for Doosan robots by integrating
NVIDIA Isaac ROS with MoveIt 2 and the ROS 2 control stack.

This document describes the complete procedure to configure and build an integrated development environment combining NVIDIA Isaac ROS (Release 4.0), 
**Jetson AGX Thor**, and Doosan ROS 2 (Jazzy) for GPU-accelerated motion planning using cuMotion and MoveIt 2.

.. note::

   This tutorial is based on the following validated environment:

   - Isaac ROS: **Release 4.0**
   - Jetpack: **7.0**
   - NVIDIA Driver: **570**
   - ROS 2: **Jazzy**
   - HardwareL **Jetson AGX Thor**

.. raw:: html

   <br>
   <br>

System Prerequisites
-------------------------------------------------

Before configuring cuMotion, you **must complete the official Isaac ROS base setup**
provided by **NVIDIA**. This ensures:

The following steps must be completed in order before proceeding.

Jetson Thor Flashing and JetPack Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Follow the official NVIDIA documentation to flash Jetson Thor and install JetPack 7.0:

https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/index.html

Isaac ROS Setup for Jetson Thor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Complete the Isaac ROS environment setup for Jetson Thor using the following documentation:

https://nvidia-isaac-ros.github.io/getting_started/index.html


.. raw:: html

   <br>
   <br>


Workspace Setup
----------------

This section describes the process of creating separate and independent workspaces for the **NVIDIA Isaac ROS cuMotion** stack 
and the Doosan ROS 2 robot control stack. By isolating these two environments, 
the system maintains a clean separation between GPU-accelerated core motion planning (Isaac ROS + cuMotion) and vendor-specific robot hardware control (Doosan ROS 2).

Isaac ROS Workspace
~~~~~~~~~~~~~~~~~~~~
All repositories must be checked out to the release-4.0 branch to ensure compatibility.


.. code-block:: bash

   mkdir -p ~/workspaces/isaac_ros-dev/src
   echo 'export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"' >> ~/.bashrc
   source ~/.bashrc
   cd ~/workspaces/isaac_ros-dev/src

   git clone --recurse-submodules -b release-4.0 \
   https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common

   git clone --recurse-submodules -b release-4.0 \
   https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git isaac_ros_cumotion

.. raw:: html

   <br>
   <br>

Doosan ROS 2 + cuMotion Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   git clone -b jazzy https://github.com/DoosanRobotics/doosan-robot2.git
   git clone -b jetson_thor https://github.com/DoosanRobotics/doosanrobotics_cumotion_driver

.. raw:: html

   <br>
   <br>

Container Execution
-------------------------------------------------

This section integrates the Doosan cuMotion Docker environment with the
Isaac ROS workspace by copying the required Docker build and runtime scripts.

Command
~~~~~~~~
 Copying the ``docker/`` and ``scripts/`` Directories

.. code-block:: bash

   cp -r ~/ros2_ws/src/cumotion/docker   ~/workspaces/
   cp -r ~/ros2_ws/src/cumotion/scripts ~/workspaces/

.. code-block:: bash

   cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts
   ./run_dev.sh

After copying, the workspace directory structure should be organized as follows:

.. code-block:: text

   workspaces/
   ├── isaac_ros-dev/
   ├── docker/
   └── scripts/


Isaac ROS CLI Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section configures the **Isaac ROS Command Line Interface (CLI)** to enable
Docker-based multi-layer image composition with the Doosan integration layer.
This configuration allows Isaac ROS to recognize the custom **Doosan Docker image layer**
during the build process.

.. code-block:: bash

   mkdir -p ~/workspaces/isaac_ros-dev/.isaac-ros-cli


Create the following configuration file:

``config.yaml``

.. code-block:: yaml

   environment:
     mode: docker

   docker:
     image:
       additional_image_keys:
         - doosan

Docker Image Layer Composition
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The integrated Docker image is built using a **multi-layer image composition strategy**.
Each layer provides a dedicated software role:

- ``noble``  
  Base Ubuntu operating system with NVIDIA CUDA runtime

- ``ros2_jazzy``  
  Isaac ROS Jazzy middleware and core robotics stack

- ``doosan``  
  Doosan ROS 2 hardware interface and cuMotion integration layer

This layered design ensures a clean separation between:
- Operating system and CUDA runtime
- Isaac ROS middleware
- Doosan robot-specific control software


Building the Integrated Docker Image
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This step builds the **fully integrated Docker image** that includes:

- Isaac ROS
- cuMotion
- Doosan ROS 2 hardware interface
- All dependent dependencies

.. code-block:: bash

   isaac-ros activate --build-local


During this process, the following actions are performed automatically:

- Multi-layer Docker image build
- Isaac ROS development container initialization
- Execution of Doosan-specific build hooks
- Full ``colcon build`` of the integrated workspace


ROS Environment Setup
-------------------------------------------------

Command
~~~~~~~~

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   cd /workspaces/isaac_ros-dev
   source install/setup.bash
   cd /ros2_ws
   source install/setup.bash

.. raw:: html

   <br>
   <br>

cuMotion Launch
-------------------------------------------------

This section launches the full cuMotion-based motion planning and execution pipeline for the Doosan robot in either real hardware mode or virtual simulation mode.
In ````real`` mode, the system establishes a direct network connection to the physical robot controller using the specified IP address 
and executes trajectories on the real hardware through the ROS 2 control interface.
In ``virtual mode``, the same motion planning pipeline is executed against a
simulated robot instance, allowing safe algorithm testing, parameter tuning, and integration validation without physical hardware.

Command
~~~~~~~~

- **Real Robot Mode**

  .. code-block:: bash

      ros2 launch dsr_cumotion start_cumotion.launch.py \
      mode:=real host:=192.168.137.100 gripper:=true

- **Virtual Robot Mode**

  .. code-block:: bash

      ros2 launch dsr_cumotion start_cumotion.launch.py \
      mode:=virtual host:=127.0.0.1 gripper:=true

**cuMotion Launch Arguments**

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Argument
     - Default
     - Description
   * - ``mode``
     - ``real``
     - Selects the execution environment. ``real`` connects to the physical Doosan robot, while ``virtual`` runs the Doosan Emulator or simulation environment.
   * - ``host``
     - ``127.0.0.1``
     - Target controller address. Use the real robot controller IP in ``real`` mode, or the emulator host IP (local emulator uses ``127.0.0.1``) in ``virtual`` mode.
   * - ``model``
     - ``m1013``
     - Robot model selection. Currently, **only the M1013 model is supported**.
   * - ``gripper``
     - ``true``
     - End-effector configuration. ``false`` loads the robot only, ``true`` loads the OnRobot VGC10 model.
   * - ``enable_nvblox``
     - ``false``
     - Enables real-time 3D environment reconstruction using NVBlox. This option requires high GPU memory and should be disabled on laptops or low-memory GPUs to allow **cuMotion-only operation**.
   * - ``enable_cumotion``
     - ``true``
     - Enables the cuMotion GPU-accelerated motion planning node.
   * - ``enable_attach``
     - ``true``
     - Enables the object attach/detach interface for pick-and-place operations.
   * - ``obstacle``
     - ``true``
     - Enables static obstacle generation through the MoveIt PlanningScene.
   * - ``use_sim_time``
     - ``false``
     - Selects the time source. ``false`` uses system wall time (must be used in ``real`` mode), and ``true`` enables simulated time for virtual environments.

.. raw:: html

   <br>
   <br>

Motion Command Topics
----------------------

Pose Command
~~~~~~~~~~~~~
This command moves the end-effector to an **absolute target pose in the robot base frame**.  
It is used when both the target **position and orientation** need to be explicitly specified.

.. code-block:: bash
   # euler
   ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'pose',
      x: 0.0, y: 0.0, z: 0.0, 
      rx: 0.0, ry: 0.0, rz: 0.0, 
      max_vel_scale: 0.5, max_acc_scale: 0.4}" --once

   # Quaternion
   ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'pose',
      x: 0.0, y: 0.0, z: 0.0, 
      qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0, 
      max_vel_scale: 0.8, max_acc_scale: 0.6}" --once


Joint Command
~~~~~~~~~~~~~~
This command moves the robot in **joint space by directly specifying each joint angle**.  
It is used when only the **target joint configuration** is required, without defining a Cartesian path.

.. code-block:: bash

   ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'joint', 
      joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
      max_vel_scale: 0.6, max_acc_scale: 0.4}" --once


Named Command
~~~~~~~~~~~~~~
This command moves the robot to a **predefined named pose (e.g., home, ready)** registered in the `NamedExecutor`.  
It is well-suited for **repetitive motions and initial pose setup**.

.. code-block:: bash

   ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'named', 
      name: 'HOME', 
      max_vel_scale: 0.6, max_acc_scale: 0.5}" --once


Relative Command (TCP)
~~~~~~~~~~~~~~~~~~~~~~~
This command performs an **incremental (relative) motion** based on the current robot state.  
It is mainly used for **fine adjustments (micro adjustments)** in either the TCP frame or the base frame.

.. code-block:: bash

   ros2 topic pub /target_pose dsr_cumotion_msgs/TargetPose "{move_type: 'relative', 
      dx: 0.0, dy: 0.0, dz: 0.0, 
      drx: 0.0, dry: 0.0, drz: 0.0, 
      max_vel_scale: 0.5, max_acc_scale: 0.5}" --once

.. raw:: html

   <br>
   <br>


Object Attach / Detach
-----------------------
This node serves as an automatic Pick & Place sequence execution server for the Doosan robot.
It performs motion execution using the MoveIt 2 + cuMotion motion planning pipeline, 
and controls object grasping and releasing in simulation through the Isaac ROS AttachObject action,
enabling both attach (grasp) and detach (release) operations.

Attach
~~~~~~~

.. code-block:: bash

   ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 0}"

Detach
~~~~~~

.. code-block:: bash

   ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 1}"


Obstacle Manager
-----------------

The ``obstacle_manager`` node is a **dynamic collision object manager** used by both **MoveIt 2 and the cuMotion motion planning pipeline**.
This node loads predefined static and mesh-based obstacles from a YAML configuration file at startup and publishes them to the
MoveIt **Planning Scene**.

At initialization, the node reads the specified YAML file and loads the following obstacle types:

- ``BOX``
- ``SPHERE``
- ``CYLINDER``
- ``MESH``

All configured obstacles are published **once** to the ``/planning_scene`` topic after a **2-second delay**.

Each collision object contains the following information:
- Reference coordinate frame (``frame_id``)
- 3D position
- Optional orientation
- Geometric dimensions or mesh scale

For ``MESH`` objects, the node uses the **trimesh** library to load a 3D mesh file (e.g., STL) and converts it into
a ROS-compatible collision object.

In addition, the node subscribes to the ``/collision_remove`` topic, allowing:
- **Selective removal** of a single collision object by ID
- **Complete removal** of all collision objects by publishing an empty string

This enables dynamic environment updates during runtime while maintaining a consistent planning scene
for cuMotion and MoveIt 2.

Usage
~~~~~~

The default obstacle configuration file is:

::

   dsr_cumotion/config/obstacle.yaml

If ``frame_id`` is not explicitly specified, it is automatically set to:

::

   base_link

Example YAML Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: yaml

   objects:
     - id: {name}
       type: cylinder
       position: [0.0, 0,0, 0.0]
       dimensions: [0.0, 0.0]   # radius, height

     - id: {name}
       type: cylinder
       position: [0.0, 0.0, 0.0]
       dimensions: [0.0, 0.0]   # radius, height

     - id: {name}
       type: cylinder
       position: [0.0, 0.0, 0.0]
       dimensions: [0.0, 0.0]   # radius, height

     - id: {name}
       type: mesh
       mesh_path: "path"         # file path
       position: [0.0, 0.0, 0.0]
       orientation: [0.0, 0.0, 0.0, 1.0]
       scale: [1.0, 1.0, 1.0]

YAML Parameter Specification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 15 15 50

   * - Key
     - Type
     - Required
     - Description
   * - ``id``
     - ``string``
     - Yes
     - Unique name of the collision object (MoveIt scene object ID). This ID is also used for object removal.
   * - ``type``
     - ``string``
     - Yes
     - Shape type of the obstacle. One of ``BOX``, ``SPHERE``, ``CYLINDER``, or ``MESH`` (case-insensitive).
   * - ``position``
     - ``[x, y, z]``
     - Yes
     - Center position of the object in meters, defined relative to ``frame_id``.
   * - ``orientation``
     - ``[qx, qy, qz, qw]``
     - No
     - Object orientation in quaternion format. Default is ``[0, 0, 0, 1]``.
   * - ``frame_id``
     - ``string``
     - No
     - Reference coordinate frame of the object. Defaults to ``base_link`` if not specified.
   * - ``dimensions``
     - ``list``
     - Shape-dependent
     - Shape dimensions (e.g., BOX: ``[x, y, z]``, CYLINDER: ``[radius, height]``).
   * - ``mesh_path`` / ``mesh_resource``
     - ``string``
     - MESH only
     - Path to the mesh file. Absolute or relative path is supported.
   * - ``scale``
     - ``[sx, sy, sz]``
     - No (MESH only)
     - Mesh scaling factor. Default is ``[1.0, 1.0, 1.0]``.

Collision Object Removal
~~~~~~~~~~~~~~~~~~~~~~~~~

Remove a specific object by ID:

.. code-block:: bash

   ros2 topic pub /collision_remove std_msgs/msg/String "{data: 'cyl1'}" --once

Remove all collision objects:

.. code-block:: bash

   ros2 topic pub /collision_remove std_msgs/msg/String "{data: ''}" --once

Launch Integration
~~~~~~~~~~~~~~~~~~~

The obstacle manager is automatically enabled when launching cuMotion with:

::

   obstacle:=true or false

Example:

.. code-block:: bash

   ros2 launch dsr_cumotion start_cumotion.launch.py \
     mode:=virtual \
     host:=127.0.0.1 \
     obstacle:=true

Major Package Overview
----------------------

This section describes the **core packages** that form the
**Doosan + Isaac ROS + cuMotion integrated system**, including their
roles and key responsibilities within the overall architecture.


``dsr_cumotion``
~~~~~~~~~~~~~~~~

**Role: Core Integration Package for Doosan, MoveIt 2, and cuMotion**

The ``dsr_cumotion`` package is the **central integration layer** of the system.
It connects the following components into a single execution pipeline:

- Doosan ROS 2 hardware interface
- MoveIt 2 motion planning framework
- NVIDIA cuMotion planner execution
- Pick-and-place task server
- Planning scene and static obstacle management


Key Responsibilities
^^^^^^^^^^^^^^^^^^^^

- Provides the **main system launch entry point** (``start_cumotion.launch.py``)
- Configures the **cuMotion and MoveIt 2 planning pipelines**
- Manages **robot model integration**:

  - URDF
  - SRDF
  - XRDF

- Provides a **Pick-and-Place task execution server**
- Manages **static obstacles using the Planning Scene**
- Manages **workspace boundaries (workbound)**

This package directly controls the **core motion planning and execution behavior of the robot**.


``dsr_cumotion_goal_interface``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Role: Motion Command Dispatch Interface**

The ``dsr_cumotion_goal_interface`` package receives **high-level user commands**
and acts as the **command gateway** that forwards them to the
**MoveIt 2 + cuMotion execution pipeline**.


Key Responsibilities
^^^^^^^^^^^^^^^^^^^^

- Subscribes to the ``/target_pose`` topic
- Selects the appropriate execution strategy based on the command type:

  - Absolute pose motion
  - Joint-space motion
  - Named pose motion
  - Relative TCP motion

- Sends motion goals to the **MoveIt 2 Action Server**
- Monitors execution status and feedback
- Executes commands sequentially using a **multi-command queue**

This package serves as the **intermediate control layer between user commands and physical robot execution**.


``dsr_cumotion_msgs``
~~~~~~~~~~~~~~~~~~~~~

**Role: System-Wide Message and Service Interface Definition Package**

The ``dsr_cumotion_msgs`` package defines all **custom ROS 2 messages and service types**
used throughout the system for **motion-level and task-level control**.


Key Responsibilities
^^^^^^^^^^^^^^^^^^^^

- Defines the **unified motion command message** that supports:

  - Absolute pose commands
  - Joint commands
  - Named target commands
  - Relative TCP commands

- Defines the **Pick-and-Place task control service interface**, including:

  - Approach → attach → retreat sequence
  - Approach → detach → retreat sequence

- Provides the **standard API contract** between:

  - User applications
  - Command interface nodes
  - Planning and execution subsystems


References
----------

- `cuMotion tutorial <https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_cumotion/index.html>`_
- `cuMotion github <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/tree/release-3.2>`_
