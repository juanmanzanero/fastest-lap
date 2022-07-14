# F1 optimal Lap Time


```python
# Put parent folder in the pythonpath
import sys,os,inspect

import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))))
import fastest_lap
from fastest_lap import KMH
```


```python
# Load vehicle
vehicle = "car"
fastest_lap.create_vehicle_from_xml(vehicle,"../../../../../database/vehicles/f1/mercedes-2020-catalunya.xml");
```


```python
# Load track
track="catalunya"
fastest_lap.create_track_from_xml(track,"../../../../../database/tracks/catalunya/catalunya_adapted.xml");
s=fastest_lap.track_download_data(track,"arclength");
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
options += "            <chassis.throttle/>"
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
throttle = fastest_lap.download_vector("run/chassis.throttle");
u        = fastest_lap.download_vector("run/chassis.velocity.x");
s        = fastest_lap.download_vector("run/road.arclength");
time     = fastest_lap.download_vector("run/time");
psi      = fastest_lap.download_vector("run/chassis.attitude.yaw");
omega    = fastest_lap.download_vector("run/chassis.omega.z");
v        = fastest_lap.download_vector("run/chassis.velocity.y");
```

## GPS


```python
import numpy as np
fastest_lap.plot_optimal_laptime(s,x,y,track);
plt.gca().invert_xaxis()

```


    
![png](Optimal_laptime_files/Optimal_laptime_6_0.png)
    


## Steering


```python
plt.figure(figsize=(20,3))
plt.plot(np.array(delta)*180.0/3.14,color="orange");
```


    
![png](Optimal_laptime_files/Optimal_laptime_8_0.png)
    


## Torque


```python
plt.figure(figsize=(20,3))
plt.plot(throttle,color="orange");
```


    
![png](Optimal_laptime_files/Optimal_laptime_10_0.png)
    



```python
import numpy as np
plt.figure(figsize=(20,3))
plt.plot(np.array(u)*3.6,color="orange");
```


    
![png](Optimal_laptime_files/Optimal_laptime_11_0.png)
    



```python

"""Overlaying speed traces of two laps
======================================
Compare two fastest laps by overlaying their speed traces.
"""


import matplotlib.pyplot as plt
import fastf1.plotting


fastf1.Cache.enable_cache('.')  # replace with your cache directory

# enable some matplotlib patches for plotting timedelta values and load
# FastF1's default color scheme
fastf1.plotting.setup_mpl()

# load a session and its telemetry data
quali = fastf1.get_session(2020, 'Spanish Grand Prix', 'Q')
laps = quali.load_laps(with_telemetry=True)


##############################################################################
# First, we select the two laps that we want to compare

ver_lap = laps.pick_driver('VER').pick_fastest()
ham_lap = laps.pick_driver('HAM').pick_fastest()

##############################################################################
# Next we get the telemetry data for each lap. We also add a 'Distance' column
# to the telemetry dataframe as this makes it easier to compare the laps.

ver_tel = ver_lap.get_car_data().add_distance()
ham_tel = ham_lap.get_car_data().add_distance()

##############################################################################
# Finally, we create a plot and plot both speed traces.
# We color the individual lines with the driver's team colors.

rbr_color = fastf1.plotting.team_color('RBR')
mer_color = fastf1.plotting.team_color('MER')

```

    utils          INFO 	NumExpr defaulting to 4 threads.
    /Users/juanmanzanero/opt/anaconda3/lib/python3.8/site-packages/fastf1/core.py:1095: FutureWarning: `Session.load_laps` is deprecated and will beremoved in a future version.
    Use `Session.load` instead.
      warnings.warn("`Session.load_laps` is deprecated and will be"
    core           INFO 	Loading data for Spanish Grand Prix - Qualifying [v2.2.3]
    api            INFO 	Using cached data for driver_info
    api            INFO 	Using cached data for timing_data
    api            INFO 	Using cached data for timing_app_data
    core           INFO 	Processing timing data...
    api            INFO 	Using cached data for session_status_data
    api            INFO 	Using cached data for track_status_data
    api            INFO 	Using cached data for car_data
    api            INFO 	Using cached data for position_data
    api            INFO 	Using cached data for weather_data
    core           INFO 	Finished loading data for 20 drivers: ['44', '77', '33', '11', '18', '23', '55', '4', '16', '10', '5', '26', '3', '7', '31', '20', '8', '63', '6', '99']



```python

fig, ax = plt.subplots(figsize=(20,5))
ax.plot(ver_tel['Distance'], ver_tel['Speed'], color=rbr_color, label='VER')
ax.plot(ham_tel['Distance'], ham_tel['Speed'], color=mer_color, label='HAM')
ax.plot(np.array(s)-168.0,np.array(u)*3.6, label="Fastest-lap")
ax.set_xlabel('Distance in m')
ax.set_ylabel('Speed in km/h')

ax.legend()
plt.suptitle(f"Fastest Lap Comparison \n "
             f"{quali.weekend.name} {quali.weekend.year} Qualifying")

plt.show()
```

    /Users/juanmanzanero/opt/anaconda3/lib/python3.8/site-packages/fastf1/core.py:919: FutureWarning: The property `Session.weekend` has been renamed to `Session.event`.
     The old property will be removed ina future version.
      warnings.warn("The property `Session.weekend` has been renamed to "
    /Users/juanmanzanero/opt/anaconda3/lib/python3.8/site-packages/fastf1/events.py:634: FutureWarning: The `Weekend.name` property is deprecated and will beremoved in a future version.
    Use `Event['EventName']` or `Event.EventName` instead.
      warnings.warn(



    
![png](Optimal_laptime_files/Optimal_laptime_13_1.png)
    



```python

fig, ax = plt.subplots(figsize=(20,4))
ax.plot(ver_tel['Distance'], ver_tel['Throttle'], color=rbr_color, label='VER')
ax.plot(ham_tel['Distance'], ham_tel['Throttle'], color=mer_color, label='HAM')
throttle_pos = [None]*len(throttle);
for i in range(len(throttle)):
    throttle_pos[i] = max(throttle[i],0.0);
ax.plot(np.array(s)-168,np.array(throttle_pos)*100, label="Fastest-lap")
ax.set_xlabel('Distance in m')
ax.set_ylabel('Throttle')
plt.ylim((-10,110))

ax.legend()
plt.suptitle(f"Fastest Lap Comparison \n "
             f"{quali.weekend.name} {quali.weekend.year} Qualifying")

plt.show()
```

    /Users/juanmanzanero/opt/anaconda3/lib/python3.8/site-packages/fastf1/core.py:919: FutureWarning: The property `Session.weekend` has been renamed to `Session.event`.
     The old property will be removed ina future version.
      warnings.warn("The property `Session.weekend` has been renamed to "
    /Users/juanmanzanero/opt/anaconda3/lib/python3.8/site-packages/fastf1/events.py:634: FutureWarning: The `Weekend.name` property is deprecated and will beremoved in a future version.
    Use `Event['EventName']` or `Event.EventName` instead.
      warnings.warn(



    
![png](Optimal_laptime_files/Optimal_laptime_14_1.png)
    


# Study of the numerical errors (nothing to see here)


```python
import numpy as np
from math import fabs, pi, sin, cos


psi_computed = np.zeros(len(psi))
psi_computed[0] = psi[0]
x_computed = np.zeros(len(x))
x_computed[0] = x[0]
y_computed = np.zeros(len(x))
y_computed[0] = y[0]
for i in range(1,len(time)):
    psi_computed[i] = psi_computed[i-1] + 0.5*(omega[i]+omega[i-1])*(time[i]-time[i-1])
    x_computed[i] = x_computed[i-1] + 0.5*(u[i]*cos(psi_computed[i])+u[i-1]*cos(psi_computed[i-1])-v[i]*sin(psi_computed[i])-v[i-1]*sin(psi_computed[i-1]))*(time[i]-time[i-1])
    y_computed[i] = y_computed[i-1] + 0.5*(u[i]*sin(psi_computed[i])+u[i-1]*sin(psi_computed[i-1])+v[i]*cos(psi_computed[i])+v[i-1]*cos(psi_computed[i-1]))*(time[i]-time[i-1])
    
for i in range(len(psi)):
    if ( psi_computed[i] > psi[i] + 1.8*np.pi):
        psi_computed[i] = psi_computed[i] - 2.0*np.pi;

plt.figure(figsize=(30,3));
plt.plot(time,psi);
plt.plot(time,psi_computed);
plt.plot(time,2.0*np.array(omega));
plt.show()
plt.figure(figsize=(30,3));
plt.plot(time,(np.array(psi_computed)-np.array(psi))*180.0/3.14);
plt.ylim(-10,10);
plt.figure(figsize=(30,3));
plt.plot(time,x);
plt.plot(time,x_computed)
plt.plot(time,y);
plt.plot(time,y_computed)
plt.figure(figsize=(30,3));
plt.plot(time,np.array(x_computed)-np.array(x))
plt.plot(time,np.array(y_computed)-np.array(y))
plt.figure(figsize=(30,30))
plt.plot(x,y)
plt.plot(x_computed,y_computed)
plt.gca().set_aspect('equal')
```


    
![png](Optimal_laptime_files/Optimal_laptime_16_0.png)
    



    
![png](Optimal_laptime_files/Optimal_laptime_16_1.png)
    



    
![png](Optimal_laptime_files/Optimal_laptime_16_2.png)
    



    
![png](Optimal_laptime_files/Optimal_laptime_16_3.png)
    



    
![png](Optimal_laptime_files/Optimal_laptime_16_4.png)
    



```python

```
