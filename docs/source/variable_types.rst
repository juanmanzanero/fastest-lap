Variable types
==============

This page provides a reference on the types of variables that are currently supported in Fastest-lap

.. list-table::
   :widths: 25 25 25 25 25
   :header-rows: 1

   * - Variable type
     - Description 
     - Create
     - Modify
     - Get     
   * - Scalar
     - Store one real number
     - :ref:`create_scalar() <f_create_scalar>`
     -       
     - download_scalar()
   * - Vector
     - Store multiple real numbers
     - :ref:`create_vector() <f_create_vector>`
     -
     - download_vector()
   * - Vehicle
     - Store one vehicle
     - :ref:`create_vehicle_from_xml() <f_create_vehicle_from_xml>`
     - vehicle_set_parameter()
     - vehicle_save_as_xml()
   * - 
     - 
     - create_vehicle_empty()
     - vehicle_declare_new_constant_parameter()
     - 
   * -
     -
     -
     - vehicle_change_track()
     -
   * - Track
     - Store one circuit
     - :ref:`create_track_from_xml() <f_create_track_from_xml>`
     -
     - track_get_data()
