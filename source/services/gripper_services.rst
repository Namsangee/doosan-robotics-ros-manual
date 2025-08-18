.. _gripper_services:

gripper Services
================

.. contents::
   :depth: 1
   :local:
   
.. _Robotiq2FClose:


Robotiq2FMove
-------------

Open or close the virtual Robotiq gripper by input value (width).

**Request:**

.. code-block::

   float64 width   # 0.0 (open) ~ 0.8 (close)

**Response:**

.. code-block::

   bool success

.. _Robotiq2FOpen:


Robotiq2FOpen
-------------

Open the virtual Robotiq gripper.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool success

.. _SerialSendData:

Robotiq2FClose
--------------

Close the virtual Robotiq gripper.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   bool success

.. _Robotiq2FMove:



SerialSendData
--------------

Send byte data to another device.  
You can operate the real Robotiq gripper via Modbus RTU.

**Request:**

.. code-block::

   string data

**Response:**

.. code-block::

   bool success

.. raw:: html

   <br><br>
