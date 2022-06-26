# Fastest-lap 🏁🏎

Fastest-lap is a vehicle dynamics simulator. It can be used to understand vehicle dynamics, to learn about driving techniques, to design car prototypes, or just for fun!

[![MacOS](https://github.com/juanmanzanero/fastest-lap/actions/workflows/macos.yml/badge.svg)](https://github.com/juanmanzanero/fastest-lap/actions/workflows/macos.yml)
[![Linux](https://github.com/juanmanzanero/fastest-lap/actions/workflows/linux.yml/badge.svg)](https://github.com/juanmanzanero/fastest-lap/actions/workflows/linux.yml)
[![Windows](https://github.com/juanmanzanero/fastest-lap/actions/workflows/windows.yml/badge.svg)](https://github.com/juanmanzanero/fastest-lap/actions/workflows/windows.yml)
[![Documentation Status](https://readthedocs.org/projects/fastest-lap/badge/?version=latest)](https://fastest-lap.readthedocs.io/en/latest/?badge=latest)
[![codecov](https://codecov.io/gh/juanmanzanero/fastest-lap/branch/main/graph/badge.svg?token=YOS7XJ8ZGP)](https://codecov.io/gh/juanmanzanero/fastest-lap)

![test](https://user-images.githubusercontent.com/26557659/173203219-077be886-7c84-49a8-a4c7-762c9f6933f7.png)

<p align="center">
  <img src="https://pbs.twimg.com/media/FLbX1kTWQAArl-O?format=jpg&name=large" />
</p>


### What can be done

- [Numerical G-G diagram][gg]: given a vehicle, and a speed, to compute its ax-ay diagram. The G-G diagram is a useful technique in vehicle design and parameters exploration.

  This is solved as an optimization problem: for a given lateral acceleration, find the minimum/maximum feasible longitudinal acceleration.

- [Optimal laptime simulation][optimal-laptime]: given a vehicle, and a circuit, to compute the optimal controls that minimize the laptime.

  This problem is solved using a first order collocation method, the trapezoidal rule, with higher-order methods planned to be implemented soon. The NLP is solved using [Ipopt][ipopt], and [CppAD][cppad] to enhance its performance (a lap-time around Circuit de Catalunya can be obtained with 500 points in approximately 1 minute).

  This is not a quasi-steady-state simulation. The model solves the fully transient states as in the dynamic equations without steady-state assumptions.

https://user-images.githubusercontent.com/26557659/163474269-5c195f4b-2109-419d-af49-7b7fa86a603d.mp4



[gg]: https://github.com/juanmanzanero/fastest-lap/tree/main/examples/python/kart/gg-diagram
[optimal-laptime]: https://github.com/juanmanzanero/fastest-lap/tree/main/examples/python/f1/optimal-laptime

### The approach

The core of the software is a C++ library, that can be used through a Python API. Full documentation is not yet available but some examples can be found in [examples/python][examples-python]. Fastest-lap is very efficient, being able to compute a full optimal lap in less than 1 minute.

[examples-python]: https://github.com/juanmanzanero/fastest-lap/tree/main/examples/python

### Dynamic models

The code implements two car models:

- A [3DOF car model][link8] (longitudinal, lateral, and yaw), currently used for F1 simulations. The default parameters used can be found in [./database/limebeer-2014-f1.xml][database-f1]
- A [6DOF car model][link2] (longitudinal, lateral, vertial, yaw, pitch and roll), currently used for Go-kart simulations. The default parameters used can be found in [./database/roberto-lot-2016-kart.xml][database]

[database]: https://github.com/juanmanzanero/fastest-lap/blob/main/database/roberto-lot-kart-2016.xml
[database-f1]: https://github.com/juanmanzanero/fastest-lap/blob/main/database/limebeer-2014-f1.xml

### Circuits

Circuits are modeled from paths created from google earth, for example, the right track limit of Catalunya is included in this repository ([database/google_earth/Catalunya_right.kml][catalunya_right.kml]). Circuits are then preprocessed with a tool included herein to extract a reference line, its curvature, and the distance to the left/right track limits ([database/catalunya_discrete.xml][catalunya_discrete]).

[catalunya_right.kml]: https://github.com/juanmanzanero/fastest-lap/blob/main/database/google_earth/Catalunya_right.kml
[catalunya_discrete]: https://github.com/juanmanzanero/fastest-lap/blob/main/database/catalunya_discrete.xml

### Dependencies

Fastest-lap uses several open-source libraries:

- [Ipopt][ipopt]: Interior Point OPTimizer, is an open source software package for large-scale nonlinear optimization. Used within this project to obtain the solution to optimal laptime problems written as NLP (Non-linear programming problem).
- [CppAD][cppad]: C++ Algorithmic Differentiation. Distributed alongside Ipopt, it is used to compute analytical derivatives.
- [Tinyxml2][tinyxml2]: TinyXML-2 is a simple, small, efficient, C++ XML parser, used to read XML files (e.g. model parameters, tracks,...)
- [logger-cpp][loggercpp]: a simple logger in C++, to handle print levels, and other interesting add-ons
- [lion-cpp][lioncpp]: lightweigh interfaces for optimization and numerics, a C++ package manager for all the libraries mentioned above, plus other numerical methods such as mechanical frames, vector algebra, and Runge--Kutta schemes

[ipopt]: https://github.com/coin-or/Ipopt
[cppad]: https://github.com/coin-or/CppAD
[tinyxml2]: https://github.com/leethomason/tinyxml2
[loggercpp]: https://github.com/juanmanzanero/logger-cpp
[lioncpp]: https://github.com/juanmanzanero/lion-cpp

### Installation

#### Windows 10

Precompiled binaries are available to download for every release.
  - [v0.1](https://github.com/juanmanzanero/fastest-lap/releases/tag/v0.1)

Download and unzip. The contents of the zip folder are:
  - bin: the dynamic libraries. Fastest-lap C++ core is there. If fastest-lap is used from MATLAB, point `loadlibrary()` to this directory.
  - include: fastestlapc.h and fastest_lap.py. To use python scripts, make sure this folder is on the `PYTHONPATH`
  - examples: python notebook examples.
  - database: car and track data

#### Mac and Linux

This project uses CMake to build the source code and produce the binaries.

The canonical steps to compile a CMake project are: (assume `$FASTESTLAP` is the source code top level.)

1.  Create a build folder.

```
mkdir ${FASTESTLAP}/build
```

2. From the build folder, run cmake

```
cd ${FASTESTLAP}/build && cmake ..
```

The options available for cmake are:

```
-DCMAKE_BUILD_TYPE=Debug/Release
-DCMAKE_INSTALL_PREFIX=/path/to/install/dir
-DCODE_COVERAGE=Yes/No: enables code coverage (if so, use with -DCMAKE_BUILD_TYPE=Debug)
-DBUILD_DOC=Yes/No: builds doxygen documentation
```

At this stage, CMake will download and install all the thirdparty dependencies.

3. Compile

```
make
```

4. Test (optional but recommended)

```
ctest --verbose
```

5. Install (optional)

```
make install
```

#### Linux

A Docker build environment is provided and can be used to compile the shared library and generate the Python bindings.

```shell
sh ./src/scripts/linux/docker_compile.sh
```


### Documentation

Read the latest fastest-lap [online documentation](http://fastest-lap.readthedocs.io/)

### References

[1] [Tremlett, A. J., and D. J. N. Limebeer. "Optimal tyre usage for a formula one car." Vehicle System Dynamics 54.10 (2016): 1448-1473.][link1]<br/>
[2] [Lot, Roberto, and Nicola Dal Bianco. "Lap time optimisation of a racing go-kart." Vehicle System Dynamics 54.2 (2016): 210-230.][link2]<br/>
[3] [Dal Bianco, Nicola, Roberto Lot, and Marco Gadola. "Minimum time optimal control simulation of a GP2 race car." Proceedings of the Institution of Mechanical Engineers, Part D: Journal of Automobile Engineering 232.9 (2018): 1180-1195.][link3]<br/>
[4] [Lot, Roberto, and Matteo Massaro. "A symbolic approach to the multibody modeling of road vehicles." International Journal of Applied Mechanics 9.05 (2017): 1750068.][link4]<br/>
[5] [Kelly, Daniel P., and Robin S. Sharp. "Time-optimal control of the race car: a numerical method to emulate the ideal driver." Vehicle System Dynamics 48.12 (2010): 1461-1474.][link5]<br/>
[6] [Piccinini, Mattia. "Path planning and control of self-driving vehicles at the limits of handling"][link6]<br/>
[7] [Casanova, D. "On minimum time vehicle manoeuvring: the theoretical optimal lap"][link7]<br/>
[8] [Perantoni, G. et al. "Optimal Control for a Formula One Car with Variable Parameters"][link8]<br/>

[link1]: https://www.tandfonline.com/doi/abs/10.1080/00423114.2016.1213861
[link2]: https://www.tandfonline.com/doi/abs/10.1080/00423114.2015.1125514
[link3]: https://journals.sagepub.com/doi/pdf/10.1177/0954407017728158?casa_token=KJUTgUXmw7UAAAAA:rpL6chgRsgy6e8KagZ50jVeLOmITur5phRQYuh_PIY-WW7mMbEHSp-VCWvz3-wZ2FxkeeyhJR_t2
[link4]: https://www.worldscientific.com/doi/abs/10.1142/S1758825117500685
[link5]: https://www.tandfonline.com/doi/abs/10.1080/00423110903514236
[link6]: https://www.researchgate.net/publication/336880897_Path_Planning_and_Control_of_Self-Driving_Vehicles_at_the_Limits_of_Handling
[link7]: https://dspace.lib.cranfield.ac.uk/handle/1826/1091
[link8]: https://web.archive.org/web/20200320055720id_/https://ora.ox.ac.uk/objects/uuid:ce1a7106-0a2c-41af-8449-41541220809f/download_file?safe_filename=Perantoni%2Band%2BLimebeer%252C%2BOptimal%2Bcontrol%2Bfor%2Ba%2BFormula%2BOne%2Bcar%2Bwith%2Bvariable%2Bparameters.pdf&file_format=application%2Fpdf&type_of_work=Journal+article
