Quickstart
==========

.. admonition:: Tutorial prerequisites

 * A suitable python shell (e.g. Anaconda)
 * Knowledge of XML (this `video <https://www.youtube.com/watch?v=1JblVElt5K0>`_ can serve as a nice introduction)

Fastest-lap can be very easily invoked from scripting languages such as Python and MATLAB. Let's do a super fast and simple system test: let's compute a lap around Circuit de Catalunya using Python.

This example is based on the python notebook `1-simple-lap <https://github.com/juanmanzanero/fastest-lap/tree/main/examples/python/f1/optimal-laptime/1-simple-lap>`_ that can be found in the repository.

The entry point to fastest-lap, is the file ``fastest_lap.py``. It is located under ``examples/python`` for Mac and Linux, and under ``include`` for Windows. This file already knows how to find the C++ dynamic library, so you do not need to worry about it.

We start by including the ``fastest_lap`` module. In this tutorial, every string called as ``"/path/to/whatever"`` is a placeholder for you to introduce the real path in your system to the indicated file.

.. code-block:: python

    import sys,os,inspect
    sys.path.append("/path/to/folder/where/fastest_lap.py/is/found/")
    import fastest_lap

This command imports the Fastest-lap python API functions, and also loads the C++ library. The C++ library is a collection of functions responsible of the computations, plus its internal memory where cars, circuits, and results are stored. From this point, Fastest-lap is ready.

We can create a car model by calling ``load_vehicle()``

.. code-block:: python

    vehicle_name = "car"
    fastest_lap.load_vehicle(vehicle_name,"limebeer-2014-f1","/path/to/database/vehicles/f1/mercedes-2020-catalunya.xml");

This creates a car of type `limebeer-2014-f1 <https://web.archive.org/web/20200320055720id_/https://ora.ox.ac.uk/objects/uuid:ce1a7106-0a2c-41af-8449-41541220809f/download_file?safe_filename=Perantoni%2Band%2BLimebeer%252C%2BOptimal%2Bcontrol%2Bfor%2Ba%2BFormula%2BOne%2Bcar%2Bwith%2Bvariable%2Bparameters.pdf&file_format=application%2Fpdf&type_of_work=Journal+article>`_ in the Fastest-lap C++ internal memory by the name of ``"car"``. If you try to create another car with the same name, the application will throw an error.

Next, we can load a circuit from an XML file by calling ``load_track()``


.. code-block:: python

    track_name = "catalunya"
    s = fastest_lap.load_track("/path/to/database/tracks/catalunya/catalunya_adapted.xml",track_name);

This loads the circuit into the Fastest-lap C++ internal memory, by the name of ``"catalunya"``, but also returns the circuit arclength points used for the optimal laptime calculation that will follow.


Car and circuit are ready. Let's compute the optimal laptime. We start by setting the options configure the computation. Options are written in XML format.


.. code-block:: python

    options  = "<options>"
    options += "    <save_variables>"
    options += "        <prefix>run/</prefix>"
    options += "        <variables>"
    options += "            <x/>"
    options += "            <y/>"
    options += "            <delta/>"
    options += "            <throttle/>"
    options += "            <u/>"
    options += "            <s/>"
    options += "            <time/>"
    options += "            <psi/>"
    options += "            <omega/>"
    options += "            <v/>"
    options += "        </variables>"
    options += "    </save_variables>"
    options += "    <print_level> 5 </print_level>"
    options += "</options>"


These options simply specify a group of variables (``x``, ``y``, ``delta``, ``throttle``, ...) that will be saved after the simulation for later postprocessing and visualization, and also sets the Ipopt internal print level to 5, which gives enough representative data of how the solution is converging. The ``<prefix/>`` node specifies a folder in the internal memory where the variables will be stored (for example, the ``x`` coordinate of the car will be stored in ``"run/x"``). Check in this `link <about:blank>`_ the full list of options.

To compute the laptime, we call ``optimal_laptime()``

.. code-block:: python

    fastest_lap.optimal_laptime(vehicle_name, track_name, s, options);

After approximately 30 seconds, the results should be ready. The time histories are stored in the C++ internal memory. To download them, you can call ``download_vector()``

.. code-block:: python

    x        = fastest_lap.download_vector("run/x");
    y        = fastest_lap.download_vector("run/y");
    delta    = fastest_lap.download_vector("run/delta");
    throttle = fastest_lap.download_vector("run/throttle");
    u        = fastest_lap.download_vector("run/u");
    s        = fastest_lap.download_vector("run/s");
    time     = fastest_lap.download_vector("run/time");
    psi      = fastest_lap.download_vector("run/psi");
    omega    = fastest_lap.download_vector("run/omega");
    v        = fastest_lap.download_vector("run/v");

Now the variables are ready in the python workspace. You can plot, analyze, and visualize them. I hope you enjoy it!
