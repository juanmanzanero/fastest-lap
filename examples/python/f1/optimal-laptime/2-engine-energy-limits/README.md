# F1 optimal Lap Time


```python
# Put parent folder in the pythonpath
import sys,os,inspect
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))))
import fastest_lap
from fastest_lap import KMH
```


```python
# Load vehicle
vehicle = "car"
fastest_lap.load_vehicle(vehicle,"limebeer-2014-f1","../../../../../database/vehicles/f1/mercedes-2020-catalunya.xml");
fastest_lap.set_scalar_parameter(vehicle,"vehicle/rear-axle/smooth_throttle_coeff", 1.0e-2);
```


```python
# Load track
track="catalunya"
s=fastest_lap.load_track("../../../../../database/tracks/catalunya/catalunya_discrete.xml",track);
```

# Lap without limits


```python
# Compute optimal laptime
options  = "<options>"
options += "    <save_variables>"
options += "        <prefix>run_without_limits/</prefix>"
options += "        <variables>"
options += "            <throttle/>"
options += "            <u/>"
options += "            <s/>"
options += "            <integral_quantities.engine-energy/>"
options += "        </variables>"
options += "    </save_variables>"
options += "    <print_level> 5 </print_level>"
options += "</options>"

fastest_lap.optimal_laptime(vehicle,track,s,options);

u_wo             = fastest_lap.download_vector("run_without_limits/u");
throttle_wo      = fastest_lap.download_vector("run_without_limits/throttle");
s_wo             = fastest_lap.download_vector("run_without_limits/s");
engine_energy    = fastest_lap.download_scalar("run_without_limits/integral_quantities.engine-energy");
print(f'Engine energy used: {engine_energy:.3f}.')
```

    Engine energy used: 32.977.


# Lap with limits


```python
# Compute optimal laptime
options  = "<options>"
options += "    <save_variables>"
options += "        <prefix>run/</prefix>"
options += "        <variables>"
options += "            <throttle/>"
options += "            <u/>"
options += "            <s/>"
options += "            <integral_quantities.engine-energy/>"
options += "        </variables>"
options += "    </save_variables>"
options += "    <integral_constraints>"
options += "        <engine-energy>"
options += "            <lower_bound> 0.0 </lower_bound>"
options += "            <upper_bound> 25.0 </upper_bound>"
options += "        </engine-energy>"
options += "    </integral_constraints> "
options += "    <print_level> 5 </print_level>"
options += "</options>"

fastest_lap.optimal_laptime(vehicle,track,s,options);

throttle = fastest_lap.download_vector("run/throttle");
u        = fastest_lap.download_vector("run/u");
s        = fastest_lap.download_vector("run/s");
engine_energy = fastest_lap.download_scalar("run/integral_quantities.engine-energy");
```

## Throttle


```python
import numpy as np
import matplotlib.pyplot as plt
import mplcyberpunk

plt.style.use("cyberpunk")

plt.figure(figsize=(20,3))
plt.plot(s,np.maximum(0,throttle_wo));
plt.plot(s,np.maximum(0,throttle));
mplcyberpunk.add_glow_effects()

```


    
![png](output_9_0.png)
    

