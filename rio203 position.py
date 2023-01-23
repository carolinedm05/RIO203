# -*- coding: utf-8 -*-
"""
Created on Fri Jan 20 09:45:36 2023

@author: carol
"""

import serial, string

from sklearn.decomposition import PCA
from sklearn.neighbors import KNeighborsClassifier

output = " "
ser = serial.Serial('/dev/ttyUSB0', 4800,8,'N',1,timeout=1)
if(ser.readline() = "start"):
    while output != "stop":
        output = ser.readline()
        print output

    
    
output_split = output.plit()    
res = [eval(i) for i in output_split]

    
    
fingerprints = [
    [-10, -20, -30], #valeur de RSSI de la base de données
    [-50, -60, -70],
    [-60,-40,-1],
    ]
    label = [
        1,
        2,
        3,
    ]
    



if __name__ == '__main__':
    _fingerprint_example()
    _room_example()

