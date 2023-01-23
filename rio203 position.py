# -*- coding: utf-8 -*-
"""
Created on Fri Jan 20 09:45:36 2023

@author: carol
"""

import serial, string
import numpy as np

# Lecture des valeurs de RSSI lu par le wismote connecté au raspberry

"""
output = " "
ser = serial.Serial('/dev/ttyUSB0', 4800,8,'N',1,timeout=1)
if(ser.readline() == "start"):
    while output != "stop":
        output = ser.readline()
        print (output)

# Séparer la sortie string en 3 RSSI string puis le convertir en un entier
    
output_split = output.plit()    
res = [eval(i) for i in output_split]
"""

#valeur de RSSI de la base de données en dictionnaire pour associé des RSSI à un mettre carré de la salle 
    
fingerprints_dic = {}
fingerprints_dic[1] = [-10, -20, -30]
fingerprints_dic[2] = [-60,-40,-1]
fingerprints_dic[3] = [-30, -40, 20]
fingerprints_dic[4] = [-60,60,30]
fingerprints_dic[5] = [40, 30, 5]
fingerprints_dic[6] = [6,4,-1]
fingerprints_dic[7] = [-10, -50, -3]
fingerprints_dic[8] = [14,-4,-1]

#print(fingerprints_dic)
    
def closest_RSSI(vec,vec_dict):
    min_distance = float('inf')
    closest_id = None
    vec = np.array(vec)
    for id, v in vec_dict.items():
        v = np.array(v)
        distance = np.linalg.norm(vec - v)
        if distance < min_distance:
            min_distance = distance
            closest_id = id
    return closest_id

#test 
print (closest_RSSI([0,0,0], fingerprints_dic))



