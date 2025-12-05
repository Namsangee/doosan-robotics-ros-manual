.. _cumotion_tutorial:

cuMotion Integration
====================

Overview
--------

**cuMotion** provides GPU-accelerated motion planning for Doosan robots by integrating
NVIDIA Isaac ROS with MoveIt 2 and the ROS 2 control stack.

This tutorial describes how to set up the full software environment and execute
the cuMotion-based motion planning pipeline.

.. note::

   This tutorial is based on the following validated environment:

   - Isaac ROS: **Release 3.2**
   - Isaac Sim: **4.2.0**
   - NVIDIA Driver: **570**
   - CUDA: **12.8** (Docker base: 12.6)
   - ROS 2: **Humble**

.. raw:: html

   <br>
   <br>

System Prerequisites
-------------------------------------------------

Before configuring cuMotion, you **must complete the official Isaac ROS base setup**
provided by **NVIDIA**. This ensures:

- Proper GPU access inside Docker
- CUDA and driver compatibility
- Isaac ROS development container initialization

`Official Isaac ros setup guide <https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html>`_ 

Only after completing the guide above, proceed with the Doosan + cuMotion integration
steps described below.


.. raw:: html

   <br>
   <br>

NVIDIA Driver Setup
-------------------------------------------------
This step installs and verifies the NVIDIA GPU driver required for all CUDA-based computation and GPU acceleration. 
The NVIDIA driver enables the operating system to communicate directly with the GPU hardware and is a mandatory prerequisite for running Isaac ROS, cuMotion, and all GPU-accelerated Docker containers.

Command
~~~~~~~~

.. code-block:: bash

   cat /proc/driver/nvidia/version
   sudo ubuntu-drivers list
   sudo ubuntu-drivers install nvidia:570

.. raw:: html

   <br>
   <br>

NVIDIA Container Toolkit
-------------------------------------------------

The **NVIDIA Container Toolkit** enables Docker containers to access the host system’s GPU resources directly. 
It provides the necessary runtime libraries and container hooks that allow CUDA, cuMotion, 
and Isaac ROS to execute GPU workloads inside containers without performance degradation.


Command
~~~~~~~

.. code-block:: bash

   sudo apt-get update && sudo apt-get install -y --no-install-recommends curl gnupg2

   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
   sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

   curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
   sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
   sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

   sudo apt-get update

   export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.0-1
   sudo apt-get install -y \
     nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
     nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
     libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
     libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}

   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl daemon-reload && sudo systemctl restart docker

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

.. code-block:: bash

   mkdir -p ~/workspaces/isaac_ros-dev/src
   echo 'export ISAAC_ROS_WS="${HOME}/workspaces/isaac_ros-dev/"' >> ~/.bashrc
   source ~/.bashrc
   cd ~/workspaces/isaac_ros-dev/src

   git clone --recursive -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
   git clone --recursive -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion.git

.. raw:: html

   <br>
   <br>

Doosan ROS 2 + cuMotion Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src

   git clone -b humble https://github.com/DoosanRobotics/doosan-robot2.git
   git clone -b humble https://github.com/DoosanRobotics/doosanrobotics_cumotion_driver

.. raw:: html

   <br>
   <br>

Container Execution
-------------------------------------------------

This step initializes and launches the Docker-based runtime environments required for the full cuMotion and Doosan ROS 2 integration. 
The startup-doosan.sh script prepares the Doosan cuMotion Docker environment by configuring the build context, applying required settings, 
and starting the Doosan-specific container for robot control and execution.

Next, the ``run_dev.sh`` script launches the NVIDIA Isaac ROS development container, which provides the GPU-accelerated runtime environment 
for cuMotion and motion planning.

Command
~~~~~~~~
.. code-block:: bash

   cd ~/ros2_ws/src/cumotion/dsr_cumotion/docker
   chmod +x startup-doosan.sh
   ./startup-doosan.sh

.. code-block:: bash

   cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common/scripts
   ./run_dev.sh

GPU Verification
~~~~~~~~~~~~~~~~~

.. code-block:: bash

   nvidia-smi

.. raw:: html

   <br>
   <br>

ROS Environment Setup
-------------------------------------------------

Command
~~~~~~~~

.. code-block:: bash

   source /opt/ros/humble/setup.bash
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
       mode:=real host:=192.168.137.100 gripper:=none

- **Virtual Robot Mode**

  .. code-block:: bash

     ros2 launch dsr_cumotion start_cumotion.launch.py \
       mode:=virtual host:=127.0.0.1 gripper:=none

Gripper Configuration
~~~~~~~~~~~~~~~~~~~~~~

**Arguments**

- ``gripper:=none``   – Robot only  
- ``gripper:=vgc10``  – OnRobot VGC10  
- ``gripper:=2f85``   – Robotiq 2F-85  

.. warning::

   - Robotiq 2F-85 is supported **only in virtual mode**
   - VGC10 gripper controller is not launched automatically and must be started separately


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
     - ``none``
     - End-effector configuration. ``none`` loads the robot only, ``vgc10`` loads the OnRobot VGC10 model (controller must be started separately), and ``2f85`` loads the Robotiq 2F-85 model (**virtual mode only**).
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

   ros2 topic pub /target_pose dsr_cumotion_msgs/msg/TargetPose "{
     x: 0.35, y: 0.20, z: 0.40,
     rx: 90.0, ry: 0.0, rz: 180.0,
     max_vel_scale: 0.5, max_acc_scale: 0.4
   }" --once

**TargetPose.msg**

This message defines an **absolute target TCP pose**.  
The orientation can be represented using **either Euler angles or a quaternion**.

.. code-block:: bash

   # TargetPose.msg
   # Absolute Cartesian pose command (Euler angles)
   # Orientation (either quaternion or euler, one of them can be 0)

   float64 x                  # Target position (m)
   float64 y
   float64 z
   float64 rx                 # Target orientation (deg) - Roll
   float64 ry                 # Pitch
   float64 rz                 # Yaw

   # Optional parameters
   float64 qx
   float64 qy
   float64 qz
   float64 qw

   float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
   float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)

Joint Command
~~~~~~~~~~~~~~
This command moves the robot in **joint space by directly specifying each joint angle**.  
It is used when only the **target joint configuration** is required, without defining a Cartesian path.

.. code-block:: bash

   ros2 topic pub /target_joint dsr_cumotion_msgs/msg/TargetJoint "{
     joints: [0.0, -90.0, 90.0, 0.0, 90.0, 0.0],
     max_vel_scale: 0.6,
     max_acc_scale: 0.4
   }" --once

**TargetJoint.msg**

This message represents a **joint-space motion command**, where each joint angle is provided as an array.

.. code-block:: bash

   # TargetJoint.msg
   # Joint-space motion command (in radians)

   float64[] joints  # Target joint angles (rad)

   # Optional parameters
   float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
   float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)

Named Command
~~~~~~~~~~~~~~
This command moves the robot to a **predefined named pose (e.g., home, ready)** registered in the `NamedExecutor`.  
It is well-suited for **repetitive motions and initial pose setup**.

.. code-block:: bash

   ros2 topic pub /target_named dsr_cumotion_msgs/msg/TargetNamed "{
     target_name: 'home',
     max_vel_scale: 0.8,
     max_acc_scale: 0.6
   }" --once

**TargetNamed.msg**

This message sends the **name of a predefined target pose** as a string.

.. code-block:: bash

   # TargetNamed.msg
   # Named target command (predefined pose name)

   string target_name          # Example: "home", "ready"

   # Optional parameters
   float64 max_vel_scale       # Velocity scaling (0.0 ~ 1.0, 0 means default)
   float64 max_acc_scale       # Acceleration scaling (0.0 ~ 1.0, 0 means default)

Relative Command (TCP)
~~~~~~~~~~~~~~~~~~~~~~~
This command performs an **incremental (relative) motion** based on the current robot state.  
It is mainly used for **fine adjustments (micro adjustments)** in either the TCP frame or the base frame.

.. code-block:: bash

   ros2 topic pub /target_relative dsr_cumotion_msgs/msg/TargetRelative "{
     reference_frame: 'tcp',
     dx: 0.0, dy: 0.00, dz: 0.20,
     drx: 0.0, dry: 0.0, drz: 0.0,
     max_vel_scale: 0.5,
     max_acc_scale: 0.5
   }" --once

**TargetRelative.msg**

This message specifies **relative translational (dx, dy, dz) and rotational (drx, dry, drz) increments**
with respect to the current TCP or base frame.

.. code-block:: bash

   # TargetRelative.msg
   # Relative motion command with respect to the current pose (Euler angles)

   string reference_frame     # "base" or "tcp" (default: "tcp")

   float64 dx                 # Relative translation (m)
   float64 dy
   float64 dz
   float64 drx                # Relative rotation (deg)
   float64 dry
   float64 drz

   # Optional parameters
   float64 max_vel_scale      # Velocity scaling (0.0 ~ 1.0, 0 means default)
   float64 max_acc_scale      # Acceleration scaling (0.0 ~ 1.0, 0 means default)

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


References
----------

- `cuMotion tutorial <https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_cumotion/index.html>`_
- `cuMotion github <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/tree/release-3.2>`_
