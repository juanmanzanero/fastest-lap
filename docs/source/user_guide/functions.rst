Functions
=========

This the reference page for all the functions included in Fastest-lap API. Both the C and Python API versions are provided.


Configuration
-------------

``set_print_level``
^^^^^^^^^^^^^^^^^^^

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
---------

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
^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^^^^^^^^^

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

``create_track_from_xml``
^^^^^^^^^^^^^^^^^^^^^^^^^
   
    Creates a circuit from an XML file.
    This XML file contains the geometrical description of the track: the centerline, heading angle, curvature, and track limits.
    
	.. _f_create_track_from_xml:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void create_track_from_xml(const char* track_name, const char* track_xml_file);       |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def create_track_from_xml(track_name, track_xml_file);                                |
    +-----------------------------------------------------------------------------------------+

    Examples of track XML files can be found in the `database <https://github.com/juanmanzanero/fastest-lap/tree/main/database/tracks>`_ folder
    
``copy_variable``
^^^^^^^^^^^^^^^^^
   
    Creates a new instance of a given existing variable under a new name
    
    .. _f_copy_variable:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void copy_variable(const char* source_name, const char* destination_name);            |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def copy_variable(source_name, destination_name)                                      |
    +-----------------------------------------------------------------------------------------+
    
``move_variable``
^^^^^^^^^^^^^^^^^
   
    Renames an existing to a new name
    
    .. _f_move_variable:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void move_variable(const char* old_name, const char* new_name);                       |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def move_variable(old_name, new_name)                                                 |
    +-----------------------------------------------------------------------------------------+

Destructors
-----------

``delete_variable``
^^^^^^^^^^^^^^^^^^^
	
    Deletes a variable with name ``variable_name`` from the internal memory.

	.. _f_delete_vehicle:

    +-----------------------------------------------------------------------------------------+
    | **C API**                                                                               |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                       |
    |                                                                                         |
    |   void delete_variable(const char* variable_name);                                      |
    +-----------------------------------------------------------------------------------------+
    | **Python API**                                                                          |
    +-----------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                  |
    |                                                                                         |
    |   def delete_variable(variable_name)                                                    |
    +-----------------------------------------------------------------------------------------+
    
    ``delete_variable`` accepts regular expressions.
    For example one can delete all the variables under the prefix ``run/`` by using
    ``delete_variable("run/*")``.


Modifiers 
---------

``vehicle_set_parameter``
^^^^^^^^^^^^^^^^^^^^^^^^^

    Sets a parameter from the physical model of an existing vehicle.
    
    .. _f_vehicle_set_parameter:

    +------------------------------------------------------------------------------------------------------------------+
    | **C API**                                                                                                        |
    +------------------------------------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                                                |
    |                                                                                                                  |
    |   void vehicle_set_parameter(const char* vehicle_name, const char* parameter_name, const double parameter_value);|
    +------------------------------------------------------------------------------------------------------------------+
    | **Python API**                                                                                                   |
    +------------------------------------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                                           |
    |                                                                                                                  |
    |   def vehicle_set_parameter(vehicle_name, parameter_name, parameter_value)                                       |
    +------------------------------------------------------------------------------------------------------------------+

    ``vehicle_name`` is the name of the vehicle to be modified, ``parameter_name`` is the path to the selected parameter, and ``parameter_value`` its new given value.
    
    For example, to set the mass of a vehicle to 795.0, one can use ``vehicle_set_parameter(vehicle_name, "vehicle/chassis/mass", 795.0)``.
    
    The full list of model parameters can be found :ref:`here <models>`.
    
``vehicle_declare_new_constant_parameter``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Among all the physical parameters of a model, selects a parameter to perform its sensitivity analysis after the computation of an optimal laptime.
    
    +------------------------------------------------------------------------------------------------------------------+
    | **C API**                                                                                                        |
    +------------------------------------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                                                |
    |                                                                                                                  |
    |   void vehicle_declare_new_constant_parameter(const char* vehicle_name, const char* parameter_name,              |
    |                                               const char* parameter_alias, const double parameter_value);        |
    +------------------------------------------------------------------------------------------------------------------+
    | **Python API**                                                                                                   |
    +------------------------------------------------------------------------------------------------------------------+
    | .. code-block:: Python                                                                                           |
    |                                                                                                                  |
    |   def vehicle_declare_new_constant_parameter(vehicle_name, parameter_name, parameter_alias, parameter_value)     |
    +------------------------------------------------------------------------------------------------------------------+
    
    ``parameter_name`` is the physical parameter that will be studied (e.g. ``vehicle/chassis/mass``).
    Parameter alias is the name by which the sensitivity analysis will be found in the internal memory (e.g. one can simply call it ``mass``), and ``parameter_value`` is a new value for the parameter.
    
``vehicle_declare_new_variable_parameter``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Among all the physical parameters of a model, selects a parameter to perform its sensitivity analysis after the computation of an optimal laptime.
    
    As opposed to constant parameters, variable parameters are allowed to vary along the circuit.
    
    +------------------------------------------------------------------------------------------------------------------+
    | **C API**                                                                                                        |
    +------------------------------------------------------------------------------------------------------------------+
    | .. code-block:: c                                                                                                |
    |                                                                                                                  |
    |   void vehicle_declare_new_variable_parameter(const char* vehicle_name, const char* parameter_name,              |
    |                                               const char* parameter_aliases, const int number_of_values,         |
    |                                               const double* parameter_values, const int number_of_mesh_points,   |
    |                                               const int* mesh_parameter_indexes, const double* mesh_points);     |
    +------------------------------------------------------------------------------------------------------------------+
    
    ``parameter_name`` is the physical parameter that will be studied (e.g. ``vehicle/chassis/aerodynamics/cd``).
    Parameter aliases are the names by which the sensitivity analysis will be found in the internal memory, separated by a semicolon (e.g. ``"cd1;cd2"``).
    ``number_of_values`` is the number of different values that the parameter can take, and ``parameter_values`` the values.
    
    ``number_of_mesh_points`` is the number of spatial points in which value breakpoints are specified, ``mesh_parameter_indexes`` selects which value will be used for each breakpoint,
    and ``mesh_points`` is the mesh arclength breakpoints.
    
    The value of the parameter in an arbitrary position is computed using a linear interpolation using these breakpoints.
    
    For example, this can be used to define a DRS. We can define two values of the drag coefficient ``cd_drs_on`` and ``cd_drs_off``. If we have a DRS zone from ``s=100`` to ``s=700``, then the arguments are
    
    - ``parameter_name = "vehicle/chassis/aerodynamics/cd"``
    - ``parameter_aliases = "cd_drs_on;cd_drs_off"``
    - ``number_of_values = 2``
    - ``parameter_values = {cd_drs_on, cd_drs_off}``
    - ``number_of_mesh_points = 6``
    - ``mesh_parameter_indexes = {1, 1, 0, 0, 1, 1}``
    - ``mesh_points = {0.0, 100.0, 101.0, 700.0, 701.0, track_length}``


Getters 
-------
