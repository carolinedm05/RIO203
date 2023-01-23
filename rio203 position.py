# -*- coding: utf-8 -*-
"""
Created on Fri Jan 20 09:45:36 2023

@author: carol
"""

import serial, string
import numpy as np

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
    
def closest_RSSI(vec,vec_list):
    min_distance = float('inf')
    closest_vec = None
    for v in vec_list:
        distance = np.linalg.norm(vec - v)
        if distance < min_distance:
            min_distance = distance
            closest_vec = v
    return closest_vec



