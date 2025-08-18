.. _installation:

Installation
============



Prerequisites
-------------

This package is developed for **ROS 2 Jazzy**.  
Please ensure you have a working ROS 2 Jazzy installation by following the `official installation guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

To use the **emulator in virtual mode**, **Docker** is required.  
Install Docker by following the `Docker official installation guide for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_.

.. note::

   **Environment Specifications** |br|
   - OS: Ubuntu 24.04 LTS |br|
   - ROS 2: Jazzy Jalisco |br|
   - Language: Python ≥ 3.10, C++17 |br|


Required Dependencies
---------------------

Install the necessary system and ROS 2 dependencies:

.. code-block:: bash

   sudo apt update
   sudo apt install -y libpoco-dev libyaml-cpp-dev wget \
     ros-jazzy-control-msgs ros-jazzy-realtime-tools ros-jazzy-xacro \
     ros-jazzy-joint-state-publisher-gui ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers ros-jazzy-gazebo-msgs ros-jazzy-moveit-msgs \
     dbus-x11 ros-jazzy-moveit-configs-utils ros-jazzy-moveit-ros-move-group

Install Gazebo Simulator support:

.. code-block:: bash

   sudo apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control


Workspace & Package Setup
--------------------------

Create your workspace and clone the repository:

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone -b jazzy https://github.com/doosan-robotics/doosan-robot2.git

Install package dependencies using rosdep:

.. code-block:: bash

   cd ~/ros2_ws
   rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

Emulator Setup (Optional)
--------------------------

If you plan to use **emulator mode**, install the virtual DRCF emulator using:

.. code-block:: bash

   cd ~/ros2_ws/src/doosan-robot2
   chmod +x ./install_emulator.sh
   sudo ./install_emulator.sh

.. note::
   This will install Docker and virtual controller components needed for simulation mode.

Build the Package
-----------------

Before building, clean up previous artifacts (recommended):

.. code-block:: bash

   cd ~/ros2_ws
   rm -rf build/ install/ log/

Then build the workspace:

.. code-block:: bash

   colcon build
   source install/setup.bash

.. note::
   To use ROS2 with Version 3.x Controller, specify the build option:


   .. code-block:: bash

      colcon build --cmake-args -DDRCF_VER=3



Test the Installation
---------------------

To verify your setup, launch the RViz2 demo:

.. code-block:: bash

   ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py


Devcontainer installation(Optional)
-----------------------------------

:ref:`devcontainer installation <devcontainer>`
