.. _installation:

Installation
============



Prerequisites
-------------

This package is developed for **ROS 2 Humble**.  
Please ensure you have a working ROS 2 Humble installation by following the `official installation guide <https://docs.ros.org/en/humble/Installation.html>`_.

To use the **emulator in virtual mode**, **Docker** is required.  
Install Docker by following the `Docker official installation guide for Ubuntu <https://docs.docker.com/engine/install/ubuntu/>`_.

.. note::

   **Environment Specifications** |br|
   - OS: Ubuntu 22.04 LTS |br|
   - ROS 2: Humble Hawksbill |br|
   - Language: Python ≥ 3.10, C++17 |br|


Required Dependencies
---------------------

Install the necessary system and ROS 2 dependencies:

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget \
                           ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
                           ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
                           ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
                           dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
                           ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control



Install Gazebo Simulator support:

.. code-block:: bash

   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt-get update
   sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ros-gz


Workspace & Package Setup
--------------------------

Create your workspace and clone the repository:

.. code-block:: bash

   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

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
