# G-G diagram


```python
# Put parent folder in the pythonpath
import sys,os,inspect
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))))
import fastest_lap
from fastest_lap import KMH
```


```python
# Load vehicle
vehicle=fastest_lap.load_vehicle("../../../database/roberto-lot-kart-2016.xml","car");
```


```python
# Compute and plot gg_diagram
fastest_lap.plot_gg(*fastest_lap.gg_diagram(vehicle,50.0*KMH,100));
```


    
![png](output_3_0.png)
    



```python

```
