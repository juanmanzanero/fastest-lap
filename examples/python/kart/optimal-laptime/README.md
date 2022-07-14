# Optimal Lap Time


```python
# Put parent folder in the pythonpath
import sys,os,inspect

import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))))
import fastest_lap
from fastest_lap import KMH
```


```python
# Load vehicle
vehicle = "car"
fastest_lap.create_vehicle_from_xml(vehicle,"../../../../database/vehicles/kart/roberto-lot-kart-2016.xml");
```


```python
# Load track
track="vendrell"
fastest_lap.create_track_from_xml(track,"../../../../database/tracks/vendrell/vendrell.xml");
s = fastest_lap.track_download_data(track, "arclength");
```


```python
# Compute optimal laptime
options  = "<options>"
options += "    <output_variables>"
options += "        <prefix>run/</prefix>"
options += "        <variables>"
options += "            <chassis.position.x/>"
options += "            <chassis.position.y/>"
options += "            <front-axle.steering-angle/>"
options += "            <rear-axle.throttle/>"
options += "            <chassis.velocity.x/>"
options += "            <road.arclength/>"
options += "            <time/>"
options += "            <chassis.attitude.yaw/>"
options += "            <chassis.omega.z/>"
options += "            <chassis.velocity.y/>"
options += "        </variables>"
options += "    </output_variables>"
options += "    <print_level> 5 </print_level>"
options += "</options>"

fastest_lap.optimal_laptime(vehicle,track,s,options);

x        = fastest_lap.download_vector("run/chassis.position.x");
y        = fastest_lap.download_vector("run/chassis.position.y");
delta    = fastest_lap.download_vector("run/front-axle.steering-angle");
throttle = fastest_lap.download_vector("run/rear-axle.throttle");
u        = fastest_lap.download_vector("run/chassis.velocity.x");
s        = fastest_lap.download_vector("run/road.arclength");
time     = fastest_lap.download_vector("run/time");
psi      = fastest_lap.download_vector("run/chassis.attitude.yaw");
omega    = fastest_lap.download_vector("run/chassis.omega.z");
v        = fastest_lap.download_vector("run/chassis.velocity.y");
```

    Calculation finished


## GPS


```python
fastest_lap.plot_optimal_laptime(s,x,y,track);
plt.gca().invert_yaxis();
```


    
![png](Optimal_laptime_files/Optimal_laptime_6_0.png)
    


## Steering


```python
plt.figure(figsize=(20,3))
plt.plot(delta,color="orange");
```


    
![png](Optimal_laptime_files/Optimal_laptime_8_0.png)
    


## Torque


```python
plt.figure(figsize=(20,3))
plt.plot(throttle,color="orange");
```


    
![png](Optimal_laptime_files/Optimal_laptime_10_0.png)
    

