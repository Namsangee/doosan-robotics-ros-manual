.. _package:

Packages
--------

This section provides an overview of the main ROS 2 packages included in the Doosan Robotics integration stack.  
Each package serves a distinct role within the architecture, from hardware control to motion planning and simulation.

dsr_bringup2
~~~~~~~~~~~~
Launch management package for starting real or virtual robots.  
Includes integrated launch files for RViz2, MoveIt 2, and Gazebo.  
Supports configurable launch arguments (e.g., `mode`, `model`, `host`, `port`) for flexible deployment.

dsr_common2
~~~~~~~~~~~
Provides shared utilities and communication layers across all Doosan packages.  
Integrates the Doosan Robot Framework Library (DRFL) for command-level control, I/O access, and state monitoring.  
Also includes service callbacks, namespace utilities, and logging support.

dsr_controller2
~~~~~~~~~~~~~~~
Defines custom ROS 2 controllers (`dsr_controller2`, `dsr_joint_trajectory`, etc.) using `ControllerInterface`.  
Acts as a bridge between ROS 2 control logic and Doosan’s native DRFL command interface.

dsr_description2
~~~~~~~~~~~~~~~~
Contains robot model definitions (URDF/Xacro), 3D meshes, and SRDF files.  
Required for RViz2 visualization, MoveIt 2 planning, and simulation environments.

dsr_example2
~~~~~~~~~~~~
Includes Python and C++ sample scripts demonstrating motion commands, tool usage, and error handling.  
Useful for learning DRFL workflows and API usage.

dsr_hardware2
~~~~~~~~~~~~~
Implements a custom `ros2_control` hardware interface as a `SystemInterface` plugin.  
Handles `read()` and `write()` cycles to synchronize joint states and commands with the robot or emulator.

dsr_msgs2
~~~~~~~~~
Defines Doosan-specific ROS 2 message and service types.  
Enables fine-grained robot and tool control (e.g., `GetToolForce`, `SetDigitalOutput`).

dsr_mujoco
~~~~~~~~~~
Enables MuJoCo simulation feature.  
Includes MuJoCo launch and config files.

dsr_tests
~~~~~~~~~
Provides test nodes, launch tests, and regression checks.  
Supports continuous integration and system validation with MoveIt 2 or Gazebo.

dsr_moveit_config_{model}
~~~~~~~~~~~~~~~~~~~~~~~~~~
Auto-generated MoveIt 2 configuration packages for each robot model.  
Includes planning pipeline setup, controller configurations, kinematics plugins, OMPL settings, and RViz2 presets.

dsr_gazebo2
~~~~~~~~~~~
Provides Gazebo simulation support with Harmonic plugin integration.  
Includes robot spawners, world files, and dynamic simulation components for testing without hardware.

.. note::

   Package names ending in `2` indicate compatibility with **ROS 2 Humble** and later.  
   These are not compatible with ROS 1 or earlier ROS 2 distributions.
