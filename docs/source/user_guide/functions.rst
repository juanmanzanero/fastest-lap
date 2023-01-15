Functions
=========

This the reference page for all the functions included in Fastest-lap.


Configuration
-----------------------

``set_print_level``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Adjusts the level of detail of the screen output. (Default: 2)
    
    .. _f_set_print_level:
      
    +------------------------------------------------------------------------+
    | **C API**                                                              |
    +------------------------------------------------------------------------+
    | .. code-block:: c                                                      |
    |                                                                        |
    |   void set_print_level(int print_level);                               |
    +------------------------------------------------------------------------+
    | **Python API**                                                         |
    +------------------------------------------------------------------------+
    | .. code-block:: Python                                                 |
    |                                                                        |
    |   def set_print_level(print_level)                                     |
    +------------------------------------------------------------------------+

    ``print_level`` ranges from 0 to 2: ``print_level=0`` supresses all outputs, and ``print_level=2`` is the fully detailed output.


Factories
-----------------

``create_scalar``
^^^^^^^^^^^^^^^^^

    Creates a scalar variable in the internal memory and assigns it an initial value

    .. _f_create_scalar:

    +------------------------------------------------------------------------+
    | **C API**                                                              |
    +------------------------------------------------------------------------+
    | .. code-block:: c                                                      |
    |                                                                        |
    |   void create_scalar(const char* variable_name, double variable_name); |
    +------------------------------------------------------------------------+
    | **Python API**                                                         |
    +------------------------------------------------------------------------+
    | .. code-block:: Python                                                 |
    |                                                                        |
    |   def create_scalar(variable_name, variable_value)                     |
    +------------------------------------------------------------------------+
                                        
    
``create_vector``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Creates a vector in the internal memory and assigns it an initial value.
    In C you need to specify how many elements it should contain, whereas in python it is automatically inferred.

    .. _f_create_vector:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void create_vector(const char* variable_name, const int vector_size, double* values); |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def create_vector(variable_name, variable_values)                                     |
    +-----------------------------------------------------------------------------------------+


``create_vehicle_from_xml``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Creates a vehicle from an XML database file.

	.. _f_create_vehicle_from_xml:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void create_vehicle_from_xml(const char* vehicle_name, const char* database_xml_file);|
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def create_vehicle_from_xml(vehicle_name, database_xml_file)                          |
    +-----------------------------------------------------------------------------------------+

    This database file is passed as the argument ``database_xml_file``. Its path can be absolute or relative but if you use relative paths remember to make sure the program is run from the proper folder!
    
    After the creation, the vehicle is stored in the internal memory with the name provided in ``vehicle_name``.
    
    
``create_vehicle_empty``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Creates an **empty** vehicle of a given type.

    .. _f_create_vehicle_empty:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void create_vehicle_empty(const char* vehicle_name, const char* vehicle_type);        |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def create_vehicle_empty(vehicle_name, vehicle_type)                                  |
    +-----------------------------------------------------------------------------------------+

    All the car parameters (for example, the vehicle mass) will be defaulted to `0.0` and they must be later set using :ref:`vehicle_set_parameter() <f_vehicle_set_parameter>`.
    
    The type of the car model is specified through ``vehicle_type``.
    
    Two types are currently supported: ``"f1-3dof"`` and ``kart-6dof``.

``create_track(track_name, track_file, options)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   
	creates a circuit from an XML file

	.. _f_create_track:

	.. code-block:: C

    		void create_track(const char* name, const char* track_file, const char* options);


	- The track is stored in the internal memory with name ``track_name``.
	- ``track_file`` is the path to the XML file that defines the circuit geometry. These files can be created with the circuit preprocessor. For example, `this <https://github.com/juanmanzanero/fastest-lap/blob/main/database/tracks/catalunya_2022/catalunya_2022_adapted.xml>`_ is the file that defines Circuit de Catalunya. Its path can be absolute or relative but if you use relative paths remember to make sure the program is run from the proper folder! 
	- ``options`` is a string that can be used to specify extra options in XML format. The root node must be ``<options/>``. Available options are:

	.. list-table::
   		:widths: 25 25 25 25 
   		:header-rows: 1

		* - option
		  - sub-option 
		  - sub-sub-option
		  - description
		* - output_variables
		  - 
		  - 
		  - to specify data from the circuit that you want to save in the internal memory
		* -                 
		  - prefix
		  - 
		  - string to prepend to the variables that will be saved (e.g. ``"track_data/"``)
		* -                 
		  - variables
		  - 
		  - list of the selected variables that will be saved 
		* -                 
		  -          
		  - s
		  - arclength (distance traveled along the circuit) of the control points

	For example, to save the arclength control points of the circuit as ``"track_data/s"`` in the internal memory (as vector) you can use: 

	.. code-block:: python

		options  = "<options>"
		options += "    <output_variables>"
		options += "        <prefix>track_data/</prefix>"        
		options += "        <variables>"
		options += "            <s/>"   
		options += "        </variables>"
		options += "    </output_variables>"
		options += "</options>"


	this can be later downloaded by calling ``download_vector("track_data/s")``



Destructors
-----------

``delete_vehicle(vehicle_name)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	
	deletes the vehicle with name ``vehicle_name`` from the internal memory

	.. _f_delete_vehicle:

	.. code-block:: C

		void delete_vehicle(const char* vehicle_name);

``delete_track(track_name)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	
	deletes the track with name ``track_name`` from the internal memory

	.. _f_delete_track:

	.. code-block:: C

		void delete_track(const char* track_name);


``clear_tables()``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	
	erases all vector and scalar variables from the internal memory (vehicles and tracks are kept)

	.. _f_clear_tables:

	.. code-block:: C

		void clear_tables();



``clear_tables_by_prefix(prefix)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	erases all vector and scalar variables whose name starts with ``prefix``
	
	.. _f_clear_tables_by_prefix:

	.. code-block:: C

		void clear_tables_by_prefix(const char* prefix);


Modifiers 
-------------

``vehicle_set_parameter``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    .. _f_vehicle_set_parameter:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void create_vehicle_empty(const char* vehicle_name, const char* vehicle_type);        |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def create_vehicle_empty(vehicle_name, vehicle_type)                                  |
    +-----------------------------------------------------------------------------------------+



Getters 
------------
