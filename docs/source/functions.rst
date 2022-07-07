Functions
=========

This the reference page for all the functions included in Fastest-lap.


Configuration
-----------------------

``set_print_level(print_level)`` 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
	adjusts the level of detail of the screen output. 

	.. code-block:: C

    		void set_print_level(int print_level);    // C
	
	.. code-block:: python

    		def set_print_level(print_level)    # Python



	``print_level`` ranges from 0 to 2: ``print_level=0`` supresses all outputs, and ``print_level=2`` is the fully detailed output.


Factories
-----------------

``create_vehicle(vehicle_name, vehicle_type, database_file)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	creates a vehicle from an XML database file.

	.. _f_create_vehicle:

	.. code-block:: C

   		void create_vehicle(const char* vehicle_name, const char* vehicle_type, const char* database_file);    // C


	- The vehicle will be stored in the internal memory with name ``vehicle_name``.
	- ``vehicle_type`` is the type of the vehicle. Two types are currently supported: ``"f1-3dof"`` and ``"kart-6dof"``
	- ``database_file`` is the path to the XML database file that defines the car parameters. Its path can be absolute or relative but if you use relative paths remember to make sure the program is run from the proper folder!

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

``create_vector(data, n, name)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	creates a vector in the internal memory with name ``name``, size ``n``, and the values given by ``data``

	.. _f_create_vector:

	.. code-block:: C

		void create_vector(double* data, const int n, const char* name);


``create_scalar(data, name)``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	creates a scalar in the internal memory with name ``name`` and value ``value``

	.. _f_create_scalar: 

	.. code-block:: C

		void create_scalar(double value, const char* name);


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


Getters 
------------
