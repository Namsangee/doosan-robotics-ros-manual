.. _cumotion_tutorial:

cuMotion Integration
====================

Overview
--------

cuMotion provides GPU-accelerated motion planning for Doosan robots by integrating
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
provided by NVIDIA. This ensures:

- Proper GPU access inside Docker
- CUDA and driver compatibility
- Isaac ROS development container initialization

`Official guide: <https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/index.html>`_

.. raw:: html

   <br>
   <br>

Only after completing the guide above, proceed with the Doosan + cuMotion integration
steps described below.


.. raw:: html

   <br>
   <br>

NVIDIA Driver Setup
-------------------------------------------------

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

Attach
~~~~~~~

.. code-block:: bash

   ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 0}"

Detach
~~~~~~

.. code-block:: bash

   ros2 service call /attach_detach_command dsr_cumotion_msgs/srv/PickPlace "{motion_type: 1}"

References
----------

- `cuMotion tutorial <https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_cumotion/index.html>`_
- `cuMotion github <https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/tree/release-3.2>`_