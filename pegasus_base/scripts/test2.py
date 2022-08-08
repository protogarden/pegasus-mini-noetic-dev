import numpy as np ##install sudo apt-get install python-numpy python-scipy python-matplotlib, pip3 install pyyaml
from scipy.interpolate import interp1d
import pylab
import yaml
 

with open("../config/battery_config.yaml") as f:
    pubsec = yaml.safe_load(f)

y =  pubsec[0]['Battery_Voltage']
b_percent = pubsec[1]['Battery Percentage']
x = []

for i in range(len(y)):

    z = 1 - b_percent[i]
    x.append(z)
print(x)



#f_nearest = interp1d(x, y, kind='nearest')
#f_linear  = interp1d(x, y)
f_cubic   = interp1d(y, x, kind='cubic')
print(f_cubic(11))
