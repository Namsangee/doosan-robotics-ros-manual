.. _io_services:

io Services
===========

.. contents::
   :depth: 1
   :local:

.. _SetCtrlBoxDigitalOutput:

SetCtrlBoxDigitalOutput
-------------------------
This is a service to check the current signal status of the digital contact mounted on the end of the robot from the robot controller.

.. code-block::

   int8       index    # ctrlbox digital output port(1 ~ 16)
   int8       value    0 = ON, 1 = OFF !!! 매뉴얼과 반대

**Response:**

.. code-block::

   bool        success


.. _GetCtrlBoxDigitalOutput:

GetCtrlBoxDigitalOutput
-----------------------
This is a service to check the current output status of the digital contact mounted on the control box in the robot controller.

**Request:**

.. code-block::

   int8       index    # Control box digital output port (1 ~ 16)

**Response:**

.. code-block::

   int8       value    # Current output status (0 = ON, 1 = OFF) !!!! manual은 반대로
   bool       success


.. _GetCtrlBoxDigitalInput:

GetCtrlBoxDigitalInput
----------------------

This service reads the I/O signals from digital contact points of the controller and reads the digital input contact value.

**Request:**

.. code-block::

   int8        index    # Digital Input in Control Box (1 ~ 16)

**Response:**

.. code-block::

   int8        value    # 0 = OFF, 1 = ON
   bool        success

.. _SetToolDigitalOutput:

SetToolDigitalOutput
---------------------
This service sends the signal of the robot tool from the digital contact point.

**Request:**

.. code-block::

   int8       index   # flange digital output port(1 ~ 6)
   int8       value   # 0 : ON, 1 : OFF

**Response:**

.. code-block::

   bool       success


.. _GetToolDigitalOutput:

GetToolDigitalOutput
--------------------

This service gets the current tool IO output status.

**Request:**

.. code-block::

   int8       index    # Flange digital output port (1 ~ 6)

**Response:**

.. code-block::

   int8       value    # Current output status (0 = ON, 1 = OFF) !!! manual과 반대
   bool       success

.. _GetToolDigitalInput:

GetToolDigitalInput
-------------------

This service reads the signals from digital contact points of the controller.

**Request:**

.. code-block::

   int8        index    # Digital Input in Flange (1 ~ 6)

**Response:**

.. code-block::

   int8        value    # 0 = OFF, 1 = ON
   bool        success

.. _SetCtrlBoxAnalogOutputType:

SetCtrlBoxAnalogOutput
----------------------
This service outputs the channel value corresponding to the controller analog output.


**Request:**

.. code-block::

   int8        channel  # 1 = ch1, 2 = ch2 
   float64     value

**Response:**

.. code-block::

   bool        success


.. _SetCtrlBoxAnalogInputType:

SetCtrlBoxAnalogInputType
-------------------------
This service sets the channel mode of the controller analog input.


**Request:**

.. code-block::

   int8        channel  # 1 = ch1, 2 = ch2 
   int8        mode     # 0 = current, 1 = voltage

**Response:**

.. code-block::

   bool        success

.. _SetCtrlBoxAnalogOutput:


SetCtrlBoxAnalogOutputType
--------------------------
This service sets the channel mode of the controller analog output.


**Request:**

.. code-block::

   int8        channel  # 1 = ch1, 2 = ch2 
   int8        mode     # 0 = current, 1 = voltage

**Response:**

.. code-block::

   bool        success

.. _GetCtrlBoxAnalogInput:

GetCtrlBoxAnalogInput
---------------------
This service reads the channel value corresponding to the controller analog input.

**Request:**

.. code-block::

   int8        channel    # 1 = ch1, 2= ch2

**Response:**

.. code-block::

   float64     value
   bool        success

