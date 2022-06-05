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
fastest_lap.load_vehicle(vehicle,"roberto-lot-kart-2016","../../../../database/vehicles/kart/roberto-lot-kart-2016.xml");
```


```python
# Load track
track="vendrell"
s=fastest_lap.load_track("../../../../database/tracks/vendrell/vendrell.xml",track);
```


```python
# Compute optimal laptime
data = fastest_lap.optimal_laptime(vehicle,track,s,["x","y","delta","throttle"]);
x = data["x"];
y = data["y"];
delta = data["delta"];
throttle = data["throttle"];
```

## GPS


```python
fastest_lap.plot_optimal_laptime(s,x,y,track);
plt.gca().invert_yaxis();
```


    
![png](output_6_0.png)
    


## Steering


```python
plt.figure(figsize=(20,3))
plt.plot(delta,color="orange");
```


    
![png](output_8_0.png)
    


## Torque


```python
plt.figure(figsize=(20,3))
plt.plot(throttle,color="orange");
```


    
![png](output_10_0.png)
    



```python

```
