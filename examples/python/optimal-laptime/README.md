# Optimal Lap Time


```python
# Put parent folder in the pythonpath
import sys,os,inspect

import matplotlib.pyplot as plt
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))
import fastest_lap
from fastest_lap import KMH
```


```python
# Load vehicle
vehicle=fastest_lap.load_vehicle("../../../database/roberto-lot-kart-2016.xml","car");
```


```python
# Load track
track=fastest_lap.load_track("../../../database/catalunya.xml","catalunya",0.2);
```


```python
# Compute optimal laptime
x,y,delta,T = fastest_lap.optimal_laptime(vehicle,track,2,500);
```

## GPS


```python
fastest_lap.plot_optimal_laptime(x,y,track,2.0);
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
plt.plot(T,color="orange");
```


    
![png](output_10_0.png)
    

