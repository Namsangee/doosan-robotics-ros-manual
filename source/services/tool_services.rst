.. _tool_services:

tool Services
=============

.. contents::
   :depth: 1
   :local:

.. _ConfigCreateTool:

ConfigCreateTool
----------------

This is a service to register and use the tool information to be installed at the end of the robot in advance for safety. 
Tool information registered using this service is stored in the memory, so it must be set again after rebooting, 
but if registered in the T/P application, the initialization process It can be reused as it is added from.

**Request:**

.. code-block::

   string          name        # tool name 
   float64         weight      # tool weight 
   float64[3]      cog         # Center of gravity
   float64[6]      inertia     # tool inertia 

**Response:**

.. code-block::

   bool success

.. _ConfigDeleteTool:

ConfigDeleteTool
----------------

It is a service to delete tool information registered in advance in the robot controller.

**Request:**

.. code-block::

   string          name        # tool name 

**Response:**

.. code-block::

   bool success

.. _GetCurrentTool:

GetCurrentTool
--------------

It is a service that fetches the currently set TOOL information from the robot controller. 
If there is no tool information set, an empty string is returned.

**Request:**

.. code-block::

   (None)

**Response:**

.. code-block::

   string         info # tool name
   bool        success

.. _SetCurrentTool:

SetCurrentTool
--------------

This is a service to set the information about the currently installed tool among the tool information registered in advance in the robot controller. 
If there is no currently installed tool, passing an empty string will initialize the currently set information.

**Request:**

.. code-block::

   string          name        # tool name

**Response:**

.. code-block::

   bool            success

.. _SetToolShape:

SetToolShape
------------

This service activates the tool shape information of the entered name among the tool shape information registered in the Teach Pendant.
If there is no tool currently installed, passing an empty string will initialize the currently set information.

This service is only available in M2.4 version or higher.

**Request:**

.. code-block::

   string          name        # Tool name registered in the Teach Pendant

**Response:**

.. code-block::

   bool            success