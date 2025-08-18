.. _launch_parameters:

Launch Parameters
=================
This section describes configurable launch parameters used to start the robot system with different modes and options.  
These parameters can be passed via the command line using `ros2 launch` or modified within launch files.

**Example command:**

.. code-block:: bash

     ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual \
       host:=127.0.0.1 port:=12345 model:=m1013 gui:=true \
       name:=dsr01 color:=white


**mode**
--------

Defines whether the robot runs in physical or virtual mode.

- ``mode:=real``  
  Runs a physical robot.  
  (Default IP: ``192.168.127.100``, Port: ``12345``)

- ``mode:=virtual``  
  Runs the robot in virtual (simulated) mode.  
  (Default IP: ``127.0.0.1``, Port: ``12345``)

The emulator will start and stop automatically during the launch lifecycle.


For more information about the ``mode`` argument, see :ref:`operation_modes`.


**name**
--------

Sets the ROS namespace used for the robot.

- Default: ``dsr01``


**host**
--------
Sets the IP address of the **Doosan Robotics Controller**.  

- Default: ``192.168.137.100``  
- For virtual mode: ``127.0.0.1``


**port**
--------
Specifies the TCP port used to connect to the robot controller.  

- Default: ``12345``


**model**
---------
Defines the robot model name.  

- Example: ``m1013``, ``a0509``

.. note::
   Ensure it matches your actual robot hardware or simulation target.


**color**
---------
Sets the color of the robot arm (affects visualization).  

- Options: ``white`` or ``blue``

.. note::
   Only ``white`` is supported for model ``E0609``.


**gui**
-------
Enable or disable the GUI (e.g., RViz2) during launch.  

- Options: ``true`` / ``false``


**gz**
------
Enable or disable the Gazebo Simulation environment.  

- Options: ``true`` / ``false``


**rt_host**
-----------
Specifies the IP address used for the real-time robot controller connection.  

- Default: ``192.168.137.50``  
- Used primarily for motion stream interfaces or internal sync mechanisms.
