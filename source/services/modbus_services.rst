.. _modbus_services:

modbus Services
===============

.. contents::
   :depth: 1
   :local:

.. _ConfigCreateModbus:

ConfigCreateModbus
------------------

This service registers the Modbus signal. The Modbus I/O must be set in the Teach Pendant I/O set-up menu. Use this command only for testing if it is difficult to use the Teach Pendant. 
The Modbus menu is disabled in the Teach Pendant if it is set using this command.

**Request:**

.. code-block::

   string      name       # Modbus signal symbol 
   string      ip         # External device IP
   int32       port       # External device port     
   int8        reg_type   # <MODBUS_REGISTER_TYPE> (0: discrete input, 1: coil, 2: input register, 3: holding register)
   int8        index      # Modbus signal index (0 ~ 9999)
   int8        value      # Modbus signal value (unsigned value; 0 ~ 65535)
   int32       slave_id   # Slave ID of the ModbusTCP (0: Broadcast, 1~247 or 255: Default for ModbusTCP)
                          # <slave_id is only available in M2.40 or later versions>  

**Response:**

.. code-block::

   bool success

.. note::
   
   The slaveid argument is only available for versions M2.40 and higher.

.. _ConfigDeleteModbus:

ConfigDeleteModbus
------------------

This service deletes the registered Modbus signal. The Modbus I/O must be set in the Teach Pendant I/O set-up menu. Use this command only for testing if it is difficult to use the Teach Pendant. 
The Modbus menu is disabled in the Teach Pendant if it is set using this command.

**Request:**

.. code-block::

   string      name       # Modbus signal symbol 

**Response:**

.. code-block::

   bool success

.. _GetModbusInput:

GetModbusInput
--------------

This service reads the signal from the Modbus system.

**Request:**

.. code-block::

   string      name       # Modbus signal symbol

**Response:**

.. code-block::

   int32       value      # Modbus signal value
   bool        success

.. _SetModbusOutput:

SetModbusOutput
---------------

This service sends the signal to an external Modbus system.

**Request:**

.. code-block::

   string      name       # Modbus signal symbol
   int32       value      # Modbus register value

**Response:**

.. code-block::

   bool        success

.. raw:: html

   <br><br>
