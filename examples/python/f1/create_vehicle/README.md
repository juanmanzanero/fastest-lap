# Create vehicle

This notebook shows two methods to load a vehicle ready to be used for simulations


```python
# Put parent folder in the pythonpath
import sys,os,inspect

import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))))
import fastest_lap
from fastest_lap import KMH
```

## 1 Via XML file

One can use an XML file containing the whole set of parameters required to build the model. In this case, the file limebeer-2014-f1.xml.

The call to load_vehicle takes three arguments: the car given name, the model type (limebeer-2014-f1 in this case, is the 3DOF model), and the path to the XML database file


```python
# Load vehicle
vehicle_xml=fastest_lap.load_vehicle("car-via-xml","limebeer-2014-f1","../../../../database/limebeer-2014-f1.xml");
```

## 2 Via the set_parameter functions

The second method consists in creating a default car, and then supply the parameters one by one via the set_parameter function


```python
# Load empty vehicle
vehicle_manual=fastest_lap.load_vehicle("car-manual","limebeer-2014-f1","");
```


```python
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-axle/track", 1.46);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-axle/inertia", 1.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-axle/smooth_throttle_coeff", 1.0e-5);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-axle/brakes/max_torque", 5000.0);

fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/track", 1.46);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/inertia", 1.55);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/smooth_throttle_coeff", 1.0e-5);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/differential_stiffness", 10.47);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/brakes/max_torque", 5000.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-axle/engine/maximum-power", 735.499);

fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/mass", 660.0);
fastest_lap.set_matrix_parameter(vehicle_manual,"vehicle/chassis/inertia", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 450.0]);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/aerodynamics/rho", 1.2);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/aerodynamics/area", 1.5);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/aerodynamics/cd", 0.9);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/aerodynamics/cl", 3.0);
fastest_lap.set_vector_parameter(vehicle_manual,"vehicle/chassis/com", [0.0, 0.0, -0.3]);
fastest_lap.set_vector_parameter(vehicle_manual,"vehicle/chassis/front_axle", [1.8, 0.0, -0.33]);
fastest_lap.set_vector_parameter(vehicle_manual,"vehicle/chassis/rear_axle", [-1.6, 0.0, -0.33]);
fastest_lap.set_vector_parameter(vehicle_manual,"vehicle/chassis/pressure_center", [-0.1, 0.0, -0.3]);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/brake_bias", 0.6);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/roll_balance_coefficient", 0.5);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/chassis/Fz_max_ref2", 1.0);

fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/radius",0.330); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/radial-stiffness",0.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/radial-damping",0.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/Fz-max-ref2", 1.0 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/reference-load-1", 2000.0 ); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/reference-load-2", 6000.0 ); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/mu-x-max-1", 1.75 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/mu-x-max-2", 1.40 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/kappa-max-1", 0.11 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/kappa-max-2", 0.10 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/mu-y-max-1", 1.80 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/mu-y-max-2", 1.45 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/lambda-max-1", 9.0 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/lambda-max-2", 8.0 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/Qx", 1.9 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/front-tire/Qy", 1.9 );

fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/radius",0.330); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/radial-stiffness",0.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/radial-damping",0.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/Fz-max-ref2", 1.0 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/reference-load-1", 2000.0 ); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/reference-load-2", 6000.0 ); 
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/mu-x-max-1", 1.75 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/mu-x-max-2", 1.40 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/kappa-max-1", 0.11 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/kappa-max-2", 0.10 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/mu-y-max-1", 1.80 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/mu-y-max-2", 1.45 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/lambda-max-1", 9.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/lambda-max-2", 8.0);
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/Qx", 1.9 );
fastest_lap.set_scalar_parameter(vehicle_manual,"vehicle/rear-tire/Qy", 1.9 );

```
